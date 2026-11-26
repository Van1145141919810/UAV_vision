# -*- coding: utf-8 -*-
"""bench_plans.py —— 决定"送多宽的图" / "要不要两级" / "要不要连问三次"

三批实验，全部用那段真实立面录像（run_20261109_164902.mp4），镜像补边造真值分散。

  W  送图宽度 640 / 1280 / 1920
     · 同一块 1920×1080 原生窗口（红点原生尺寸）→ 再缩到 1280 / 640
     · 直接回答："网页改成 1920、VLM 就吃这张图" 会不会比现在（640）更好

  L2 两级串联
     · 一级：1920 窗口切 3 条 → 选一条
     · 二级：把**它选中的那条**裁出来放大到 1280 宽，再切 3 条 → 选一条
     · 合并精度 = 最终子格命中；错的分两种：一级就错（后面全废）/ 一级对二级错

  M3 连问三次取中值
     · 同一帧问 3 次：单次准确率 vs 中值后准确率

用法
    python eval\\bench_plans.py --exp W,L2,M3
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "eval"))
sys.path.insert(0, str(Path(r"D:\Desktop\无人系统\_ref\s14")))

from bench_lat_crop import (ask, cell_pos, crop_shift, draw_cols, dump,  # noqa: E402
                            first_digit, imwrite_u, imwrite_u as _imw, load_frames,
                            padded, pick_by_d, red_diam, Q_COL, FX, DOT_D, CW, CH)

OUT_JSON = ROOT / "eval" / "plans.json"
OUT_DIR = ROOT / "eval" / "plans_figs"
W1920 = (1920, 1080)
PAD_Y = 360
TOP_CROP = 80       # 录屏顶部状态文字条的高度


def padded2(img):
    """切掉顶部 80 行状态文字条；左右镜像补边、上下复制补边。
    · 左右用 REFLECT：立面是重复结构，镜像后看着仍然是立面（横向要自由移动）
    · 上下用 REPLICATE：REFLECT 会把门/玻璃翻过来糊在顶上，模型会去认那个假东西"""
    tmp = cv2.copyMakeBorder(img[TOP_CROP:], 0, 0, 640, 640, cv2.BORDER_REFLECT_101)
    return cv2.copyMakeBorder(tmp, PAD_Y, PAD_Y, 0, 0, cv2.BORDER_REPLICATE)


def window1920(f, tx, ty):
    """从补边图取 1920×1080 原生窗口，红点落在 (tx,ty)。"""
    pim = padded2(f["clean"])
    W, H = W1920
    gx, gy = f["gx"], f["gy"] - TOP_CROP
    x0 = int(round(gx + 640 - tx))
    y0 = int(round(gy + PAD_Y - ty))
    x0 = max(0, min(pim.shape[1] - W, x0))
    y0 = max(0, min(pim.shape[0] - H, y0))
    return pim[y0:y0 + H, x0:x0 + W].copy(), (gx + 640 - x0, gy + PAD_Y - y0)


Q_GLASS4 = ("这张图被一条竖线分成左右两半（左半是 1 号，右半是 2 号，编号画在顶部）。"
            "画面里有一块【明显的玻璃门 / 玻璃窗】。它在 1 号还是 2 号？"
            "只回答一个数字；如果你确实看不清它在哪一半，就回答 0。")

Q_HALF_V4 = ("这张图被一条横线分成上下两半（上半是 1 号，下半是 2 号，编号画在左侧）。"
             "图中有一个【贴在白色纸上的红色圆形标记】。它在 1 号还是 2 号？"
             "只回答一个数字；如果你确实看不清它在哪一半，就回答 0。")


def draw_half_v(img):
    """上下二分：一条横线，没有"中间"可躲。"""
    out = img.copy()
    H, W = out.shape[:2]
    ym = H // 2
    cv2.line(out, (0, ym), (W - 1, ym), (0, 255, 255), 2)
    for t, y in (("1", 120), ("2", ym + 120)):
        for c, th in (((0, 0, 0), 8), ((0, 255, 255), 4)):
            cv2.putText(out, t, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def resize_w(img, w):
    if img.shape[1] == w:
        return img
    h = int(round(img.shape[0] * w / float(img.shape[1])))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


Q_COL4 = ("这张图被竖向切成 3 条编号的竖条（编号画在每条竖条上端）。"
          "图中有一个【贴在白色纸上的红色圆形标记】。它在几号竖条？"
          "只回答一个数字；如果你确实看不清它落在哪一条，就回答 0。")

Q_HALF4 = ("这张图被一条竖线分成左右两半（左半是 1 号，右半是 2 号，编号画在顶部）。"
           "图中有一个【贴在白色纸上的红色圆形标记】。它在 1 号还是 2 号？"
           "只回答一个数字；如果你确实看不清它在哪一半，就回答 0。")

Q_HALF = ("这张图被一条竖线分成左右两半（左半是 1 号，右半是 2 号，编号画在顶部）。"
          "图中有一个【贴在白色纸上的红色圆形标记】。它在 1 号还是 2 号？只回答一个数字。")


def draw_half(img):
    """二分：只有一条竖线，没有"中间"这个选项可躲。"""
    out = img.copy()
    H, W = out.shape[:2]
    xm = W // 2
    cv2.line(out, (xm, 0), (xm, H - 1), (0, 255, 255), 2)
    for t, x in (("1", 14), ("2", xm + 14)):
        for c, th in (((0, 0, 0), 8), ((0, 255, 255), 4)):
            cv2.putText(out, t, (x, 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="W,L2,M3")
    ap.add_argument("--frames", default="9,15,52", help="按红点直径挑帧（越大越近）")
    ap.add_argument("--reps", type=int, default=1)
    a = ap.parse_args()
    exps = set(a.exp.replace("E", "").replace("e", "").split(","))
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    frames = load_frames()
    picks = pick_by_d(frames, [float(x) for x in a.frames.split(",")], tol=0.22)
    print("可用帧 %d；选中 %s" % (len(frames), [(k, round(FX * DOT_D / v["d"], 1)) for k, v in picks]),
          flush=True)
    rng = random.Random(20261123)
    rows = []

    # ── W：送图宽度 ────────────────────────────────────────────────
    if "W" in exps:
        print("\n=== W 送图宽度（同一块 1920 窗口，再缩）===", flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                for rep in range(a.reps):
                    tx = cell_pos(cell, W1920[0], rng)
                    ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                    win, (lx, ly) = window1920(f, tx, ty)
                    truth = int(lx / (W1920[0] / 3.0)) + 1
                    deff = red_diam(win, int(lx), int(ly))
                    line = {}
                    for w in (1920, 1280, 640):
                        img = draw_cols(resize_w(win, w))
                        got = first_digit(ask(img, Q_COL))
                        line[w] = got
                        rows.append(dict(exp="W", frame=k, width=w, cell=cell, rep=rep,
                                         truth=truth, d_eff=round(deff, 1), got=got,
                                         ok=got == truth))
                        dump(rows)
                    print("[W ] 帧%-4d 真%d 点%5.1fpx | 1920→%-4s %s | 1280→%-4s %s | 640→%-4s %s"
                          % (k, truth, deff,
                             line[1920], "对" if line[1920] == truth else "错",
                             line[1280], "对" if line[1280] == truth else "错",
                             line[640], "对" if line[640] == truth else "错"), flush=True)
                    if rep == 0:
                        imwrite_u(OUT_DIR / ("W_f%d_c%d_1920.jpg" % (k, cell)), draw_cols(win))
        for w in (1920, 1280, 640):
            sub = [r for r in rows if r["exp"] == "W" and r["width"] == w]
            hit = sum(1 for r in sub if r["ok"])
            print("  W %-4d 宽: %d/%d = %.1f%%" % (w, hit, len(sub), 100.0 * hit / max(1, len(sub))))

    # ── L2：两级串联 ──────────────────────────────────────────────
    if "L2" in exps:
        print("\n=== L2 两级串联（一级 1920 选条 → 二级裁条放大到 1280 再选）===", flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                tx = cell_pos(cell, W1920[0], rng)
                ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                win, (lx, ly) = window1920(f, tx, ty)
                truth1 = int(lx / (W1920[0] / 3.0)) + 1
                # 一级
                c1 = first_digit(ask(draw_cols(win), Q_COL))
                rec = dict(exp="L2", frame=k, cell=cell, truth1=truth1, c1=c1,
                           ok1=(c1 == truth1))
                # 二级：裁"它选中的那条"
                if c1 is None:
                    rec.update(state="l1_fail", ok=False)
                else:
                    x0 = int((c1 - 1) * W1920[0] / 3.0)
                    sub = win[:, x0:x0 + W1920[0] // 3]
                    truth2 = int((lx - x0) / (W1920[0] / 9.0)) + 1
                    # 红点不在它选的那条里 → 二级物理上找不到
                    if not (1 <= truth2 <= 3):
                        rec.update(state="wrong_col", ok=False, c2=None)
                    else:
                        big = resize_w(sub, 1280)
                        c2 = first_digit(ask(draw_cols(big), Q_COL))
                        rec.update(state="ok" if (truth1 == c1 and c2 == truth2) else "l2_miss",
                                   c2=c2, truth2=truth2,
                                   ok=(truth1 == c1 and c2 == truth2))
                        if cell == 1 and k == picks[0][0]:
                            imwrite_u(OUT_DIR / ("L2_f%d_c%d_big.jpg" % (k, cell)),
                                      draw_cols(big))
                rows.append(rec)
                dump(rows)
                print("[L2] 帧%-4d 真%d | 一级答%-4s %s | 二级 %s  %s"
                      % (k, truth1, c1, "对" if rec["ok1"] else "错",
                         rec.get("state"), "命中" if rec["ok"] else "未命中"), flush=True)
        sub = [r for r in rows if r["exp"] == "L2"]
        hit = sum(1 for r in sub if r["ok"])
        st = Counter(r["state"] for r in sub)
        print("  L2 合并 %d/%d = %.1f%%   （一级对 %d / 一级就错 %d / 一级对二级错 %d）"
              % (hit, len(sub), 100.0 * hit / max(1, len(sub)),
                 sum(1 for r in sub if r["ok1"]), st.get("wrong_col", 0) + st.get("l1_fail", 0),
                 st.get("l2_miss", 0)))

    # ── M3：连问三次取中值 ────────────────────────────────────────
    if "M3" in exps:
        print("\n=== M3 连问三次取中值（1920 窗口，左右）===", flush=True)
        n1 = n3 = tot = 0
        for (k, f) in picks:
            for cell in (1, 2, 3):
                tx = cell_pos(cell, W1920[0], rng)
                ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                win, (lx, ly) = window1920(f, tx, ty)
                truth = int(lx / (W1920[0] / 3.0)) + 1
                got = [first_digit(ask(draw_cols(win), Q_COL)) for _ in range(3)]
                vals = [g for g in got if g is not None]
                med = int(np.median(vals)) if vals else None
                tot += 1
                n1 += 1 if got[0] == truth else 0
                n3 += 1 if med == truth else 0
                rows.append(dict(exp="M3", frame=k, cell=cell, truth=truth, got=got,
                                 med=med, ok1=got[0] == truth, ok3=med == truth))
                dump(rows)
                print("[M3] 帧%-4d 真%d | 三次 %s → 中值 %s  %s"
                      % (k, truth, got, med, "对" if med == truth else "错"), flush=True)
        print("  M3 单次第 1 问 %d/%d = %.1f%%   三次中值 %d/%d = %.1f%%"
              % (n1, tot, 100.0 * n1 / max(1, tot), n3, tot, 100.0 * n3 / max(1, tot)))

    # ── B2：二分（左半 / 右半），没有"中间"可躲 ────────────────────
    if "B2" in exps:
        print("\n=== B2 二分左/右（1920 与 1280）===", flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                for rep in range(max(1, a.reps)):
                    tx = cell_pos(cell, W1920[0], rng)
                    ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                    win, (lx, ly) = window1920(f, tx, ty)
                    truth = 1 if lx < W1920[0] / 2.0 else 2
                    deff = red_diam(win, int(lx), int(ly))
                    line = {}
                    for w in (1920, 1280):
                        got = first_digit(ask(draw_half(resize_w(win, w)), Q_HALF), 1, 2)
                        line[w] = got
                        rows.append(dict(exp="B2", frame=k, width=w, cell=cell, rep=rep,
                                         truth=truth, d_eff=round(deff, 1), got=got,
                                         ok=got == truth))
                        dump(rows)
                    print("[B2] 帧%-4d 真%d 点%5.1fpx | 1920→%-4s %s | 1280→%-4s %s"
                          % (k, truth, deff,
                             line[1920], "对" if line[1920] == truth else "错",
                             line[1280], "对" if line[1280] == truth else "错"), flush=True)
        for w in (1920, 1280):
            sub = [r for r in rows if r["exp"] == "B2" and r["width"] == w]
            hit = sum(1 for r in sub if r["ok"])
            print("  B2 %-4d 宽: %d/%d = %.1f%%" % (w, hit, len(sub), 100.0 * hit / max(1, len(sub))))

    # ── C4：三分 + "看不清就答 0" ──────────────────────────────────
    if "C4" in exps:
        print("\n=== C4 三分 + 看不清答 0（1920）===", flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                tx = cell_pos(cell, W1920[0], rng)
                ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                win, (lx, ly) = window1920(f, tx, ty)
                truth = int(lx / (W1920[0] / 3.0)) + 1
                deff = red_diam(win, int(lx), int(ly))
                txt = ask(draw_cols(win), Q_COL4)
                got = first_digit(txt, 0, 3)
                rows.append(dict(exp="C4", frame=k, cell=cell, truth=truth,
                                 d_eff=round(deff, 1), got=got, ok=got == truth, raw=txt[:20]))
                dump(rows)
                print("[C4] 帧%-4d 真%d 点%5.1fpx | 答%-4s %s  raw=%r"
                      % (k, truth, deff, got, "对" if got == truth else "错", txt[:16]), flush=True)
        sub = [r for r in rows if r["exp"] == "C4"]
        hit = sum(1 for r in sub if r["ok"])
        zero = sum(1 for r in sub if r["got"] == 0)
        wrong_mid = sum(1 for r in sub if r["got"] == 2 and r["truth"] != 2)
        print("  C4 命中 %d/%d = %.1f%%   答0(看不清) %d   答'中'但错 %d"
              % (hit, len(sub), 100.0 * hit / max(1, len(sub)), zero, wrong_mid))

    # ── B2Z：二分 + 看不清答 0 ────────────────────────────────────
    if "B2Z" in exps:
        print("\n=== B2Z 二分 + 看不清答 0（1920）===", flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                tx = cell_pos(cell, W1920[0], rng)
                ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                win, (lx, ly) = window1920(f, tx, ty)
                truth = 1 if lx < W1920[0] / 2.0 else 2
                deff = red_diam(win, int(lx), int(ly))
                got = first_digit(ask(draw_half(win), Q_HALF4), 0, 2)
                rows.append(dict(exp="B2Z", frame=k, cell=cell, truth=truth,
                                 d_eff=round(deff, 1), got=got, ok=got == truth))
                dump(rows)
                print("[B2Z] 帧%-4d 真%d 点%5.1fpx | 答%-4s %s"
                      % (k, truth, deff, got, "对" if got == truth else "错"), flush=True)
        sub = [r for r in rows if r["exp"] == "B2Z"]
        hit = sum(1 for r in sub if r["ok"])
        zero = sum(1 for r in sub if r["got"] == 0)
        bad = sum(1 for r in sub if r["got"] not in (0, r["truth"]))
        print("  B2Z 命中 %d/%d = %.1f%%   答0(看不清) %d   自信答错 %d"
              % (hit, len(sub), 100.0 * hit / max(1, len(sub)), zero, bad))

    # ── B2Z / B2ZV 扩样：二分 + 看不清答 0，左右 / 上下 ─────────────
    for tag, axis, q, drawer in (("B2Z", "col", Q_HALF4, draw_half),
                                 ("B2ZV", "row", Q_HALF_V4, draw_half_v)):
        if tag not in exps:
            continue
        print("\n=== %s %s 二分 + 看不清答 0（1920，扩样）==="
              % (tag, "左右" if axis == "col" else "上下"), flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                for rep in range(max(1, a.reps)):
                    if axis == "col":
                        tx = cell_pos(cell, W1920[0], rng)
                        ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                    else:
                        tx = W1920[0] / 2.0 + rng.uniform(-120, 120)
                        ty = cell_pos(cell, W1920[1], rng)
                    win, (lx, ly) = window1920(f, tx, ty)
                    if axis == "col":
                        truth = 1 if lx < W1920[0] / 2.0 else 2
                    else:
                        truth = 1 if ly < W1920[1] / 2.0 else 2
                    deff = red_diam(win, int(lx), int(ly))
                    got = first_digit(ask(drawer(win), q), 0, 2)
                    rows.append(dict(exp=tag, frame=k, cell=cell, rep=rep, truth=truth,
                                     d_eff=round(deff, 1), got=got, ok=got == truth))
                    dump(rows)
                    print("[%s] 帧%-4d 放%d→真%d 点%5.1fpx | 答%-4s %s"
                          % (tag, k, cell, truth, deff, got,
                             "对" if got == truth else ("看不清" if got == 0 else "错")),
                          flush=True)
        sub = [r for r in rows if r["exp"] == tag]
        hit = sum(1 for r in sub if r["ok"])
        zero = sum(1 for r in sub if r["got"] == 0)
        bad = sum(1 for r in sub if r["got"] not in (0, r["truth"]))
        print("  %s 命中 %d/%d = %.1f%%   看不清 %d   自信答错 %d"
              % (tag, hit, len(sub), 100.0 * hit / max(1, len(sub)), zero, bad))
        for lo, hi, lab in ((0, 12, "点 <12px (≥27m)"), (12, 22, "点 12~22px (15~27m)"),
                            (22, 40, "点 22~40px (8~15m)"), (40, 999, "点 >40px (<8m)")):
            s3 = [r for r in sub if lo <= r["d_eff"] < hi]
            if s3:
                h3 = sum(1 for r in s3 if r["ok"])
                z3 = sum(1 for r in s3 if r["got"] == 0)
                b3 = sum(1 for r in s3 if r["got"] not in (0, r["truth"]))
                print("       %-18s 命中 %2d/%2d  看不清 %d  自信错 %d"
                      % (lab, h3, len(s3), z3, b3))

    # ── G：同一张图，问「玻璃」vs 问「红点」 ──────────────────────
    if "G" in exps:
        print("\n=== G 同一张图：问『玻璃门/窗在哪半』 vs 问『红点在哪半』 ===", flush=True)
        for (k, f) in picks:
            for cell in (1, 2, 3):
                for rep in range(max(1, a.reps)):
                    tx = cell_pos(cell, W1920[0], rng)
                    ty = W1920[1] / 2.0 + rng.uniform(-80, 80)
                    win, (lx, ly) = window1920(f, tx, ty)
                    truth = 1 if lx < W1920[0] / 2.0 else 2
                    deff = red_diam(win, int(lx), int(ly))
                    g_dot = first_digit(ask(draw_half(win), Q_HALF4), 0, 2)
                    g_gls = first_digit(ask(draw_half(win), Q_GLASS4), 0, 2)
                    rows.append(dict(exp="G", frame=k, cell=cell, rep=rep, truth=truth,
                                     d_eff=round(deff, 1), dot=g_dot, glass=g_gls,
                                     ok_dot=g_dot == truth, ok_glass=g_gls == truth))
                    dump(rows)
                    print("[G ] 帧%-4d 真%d 点%5.1fpx | 问红点→%-4s %s | 问玻璃→%-4s %s"
                          % (k, truth, deff,
                             g_dot, "对" if g_dot == truth else ("看不清" if g_dot == 0 else "错"),
                             g_gls, "对" if g_gls == truth else ("看不清" if g_gls == 0 else "错")),
                          flush=True)
        sub = [r for r in rows if r["exp"] == "G"]
        for key, lab in (("dot", "问红点(7~12px)"), ("glass", "问玻璃门/窗")):
            hit = sum(1 for r in sub if r["ok_" + key])
            zero = sum(1 for r in sub if r[key] == 0)
            bad = sum(1 for r in sub if r[key] not in (0, r["truth"]))
            print("  G %-16s 命中 %d/%d = %.1f%%   看不清 %d   自信错 %d"
                  % (lab, hit, len(sub), 100.0 * hit / max(1, len(sub)), zero, bad))

    # ── CL：闭环收敛仿真（问窗户 → 转到它说的那一半中心 → 再问）──────
    if "CL" in exps:
        print("\n=== CL 闭环收敛（问玻璃门/窗，按答案转 → 重拍 → 再问，最多 4 步）===", flush=True)
        HALF = W1920[0] / 2.0
        ok_cnt = 0
        trials = 0
        for (k, f) in picks:
            for start in (300, 1500, 1750):       # 目标一开始偏左 / 偏右 / 很偏右
                trials += 1
                lx_target = start
                hist = []
                converged = False
                for step in range(4):
                    # 把红点摆到 lx_target 处
                    win, (lx, ly) = window1920(f, lx_target, W1920[1] / 2.0)
                    err0 = abs(lx - HALF) / HALF
                    if err0 <= 0.25:
                        converged = True
                        break
                    got = first_digit(ask(draw_half(win), Q_GLASS4), 0, 2)
                    hist.append((round(lx), got))
                    if got == 0:
                        break                        # 看不清 → 停（本方案里交回人）
                    aim = HALF / 2.0 if got == 1 else HALF * 1.5   # 那一半的中心
                    lx_target = lx - (aim - HALF)                  # 转 aim→中心 后红点的新位置
                    lx_target = max(60, min(W1920[0] - 60, lx_target))
                ok_cnt += 1 if converged else 0
                rows.append(dict(exp="CL", frame=k, start=start, hist=hist,
                                 converged=converged))
                dump(rows)
                print("[CL] 帧%-4d 起点x=%4d → %s  收敛=%s"
                      % (k, start, hist, "✅" if converged else "❌"), flush=True)
        print("  CL 收敛 %d/%d = %.1f%%（判据：红点进画面中间 1/2 宽，最多 4 步）"
              % (ok_cnt, trials, 100.0 * ok_cnt / max(1, trials)))

    print("\n结果 -> %s" % OUT_JSON)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
