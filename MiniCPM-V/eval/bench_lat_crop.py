# -*- coding: utf-8 -*-
"""bench_lat_crop.py —— 方向能力的两项真测（去黄框对照 + 左右真值分散）

背景（为什么必须重做）
──────────────────────────────────────────────────────────────
1. 之前 40 帧的 3 带测试，真值 **全部落在第 2 带 / 第 2 条**：飞机正对红点直飞，
   红点 x 只在 785~1220 之间（1920 帧里恒在中三分之一）。
   所以「列 80%」= 「永远答 2」，零信息量。左右一次没测过。
2. 那段录像 `run_20261109_164902.mp4` **烧进了 CV 的黄框 + 黄字「1: 8px 37.1m」**，
   正好画在红点上。模型很可能在看黄框。77.5% 有多少是这个的功劳，此前没人知道。

三个实验
──────────────────────────────────────────────────────────────
  E1  去黄框对照：全帧 1920→1280（和原 bench 同口径），问纵向 3 带
      条件 = 原帧(有黄框) / inpaint 后(干净)  → 差值 = 黄框贡献
  E2  左右真值分散：把 1280×720 窗口摆到让红点落在 1/2/3 条竖条上
      用**镜像补边**（BORDER_REFLECT）实现位移，红点像素尺寸**不变**
  E3  上下真值分散：同 E2，但竖直方向摆位（红点天然只落在 2/3 带）

用法
    python eval\\bench_lat_crop.py --exp 1        # 去黄框对照
    python eval\\bench_lat_crop.py --exp 2,3      # 左右 / 上下
    python eval\\bench_lat_crop.py --exp 9        # 只导出对照图
"""
from __future__ import annotations

import argparse
import base64
import json
import pickle
import random
import sys
import urllib.request
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
S14 = Path(r"D:\Desktop\无人系统\_ref\s14")
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(S14))

from vb09.perception.stereo_rig import StereoRig            # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
GT = ROOT / "eval" / "gt_cache.pkl"
OUT_JSON = ROOT / "eval" / "lat_crop.json"
OUT_DIR = ROOT / "eval" / "lat_crop_figs"

FX = 1626.6183
DOT_D = 0.20
FW, FH = 1920, 1080
CW, CH = 1280, 720

Q_ROW = ("这张图被横向切成 3 条带编号的横带（编号画在每条带左上角）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号横带？只回答一个数字。")
Q_COL = ("这张图被竖向切成 3 条编号的竖条（编号画在每条竖条上端）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号竖条？只回答一个数字。")


# ---------------------------------------------------------------- 基础
def ask(img, prompt, npd=8):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        return ""
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=180) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def dump(rows):
    """每跑完一条就落盘 —— 中途 Ctrl+C / 被杀也留得住已出的数。"""
    json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


def first_digit(txt, lo=1, hi=3):
    for ch in txt or "":
        if ch.isdigit() and lo <= int(ch) <= hi:
            return int(ch)
    return None


def imwrite_u(path, img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if ok:
        Path(path).write_bytes(buf.tobytes())


def deoverlay(bgr):
    """把烧进画面的黄框/黄字 inpaint 掉（红点是红的，不误伤）。"""
    b, g, r = (bgr[:, :, 0].astype(np.int16), bgr[:, :, 1].astype(np.int16),
               bgr[:, :, 2].astype(np.int16))
    m = ((r > 140) & (g > 140) & (b < 130) & (abs(r - g) < 60)).astype(np.uint8) * 255
    if int((m > 0).sum()) < 40:
        return bgr.copy()
    m = cv2.dilate(m, np.ones((3, 3), np.uint8), iterations=2)
    return cv2.inpaint(bgr, m, 3, cv2.INPAINT_TELEA)


def draw_cols(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        x0, x1 = int(i * W / n), int((i + 1) * W / n)
        cv2.rectangle(out, (x0, 0), (x1 - 1, H - 1), (0, 255, 255), 2)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (x0 + 14, 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def draw_rows(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        y0, y1 = int(i * H / n), int((i + 1) * H / n)
        cv2.rectangle(out, (0, y0), (W - 1, y1), (0, 255, 255), 2)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (14, y0 + 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def red_diam(bgr, cx, cy, win=30):
    x0, y0 = int(max(0, cx - win)), int(max(0, cy - win))
    sub = bgr[y0:y0 + 2 * win, x0:x0 + 2 * win]
    if sub.size == 0:
        return 0.0
    b, g, r = (sub[:, :, 0].astype(np.int16), sub[:, :, 1].astype(np.int16),
               sub[:, :, 2].astype(np.int16))
    m = ((r - np.maximum(g, b)) > 45).astype(np.uint8)
    n, _, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return 0.0
    return float(2.0 * np.sqrt(max(stats[1:, 4]) / np.pi))


# ---------------------------------------------------------------- 取帧
def load_frames():
    with open(GT, "rb") as f:
        d = pickle.load(f)
    gt = {}
    gt.update(d["FAR"])
    gt.update(d["NEAR"])
    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
    cap = cv2.VideoCapture(str(VID))
    want, out, i = set(gt.keys()), {}, 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if i in want:
            L, _ = rig.rectify(fr, fr)
            gx, gy, dd = gt[i]
            if 0 <= gx < FW and 0 <= gy < FH and dd > 0:
                out[i] = dict(raw=L, clean=deoverlay(L), gx=float(gx), gy=float(gy), d=float(dd))
        i += 1
    cap.release()
    return out


def pick_by_d(frames, targets, tol=0.18):
    """每个目标直径挑一帧（相对误差 < tol，取最接近的）。"""
    picks = []
    for t in targets:
        best, bd = None, 1e9
        for k, v in frames.items():
            e = abs(v["d"] - t) / t
            if e < tol and e < bd:
                bd, best = e, k
        if best is not None and best not in [p[0] for p in picks]:
            picks.append((best, frames[best]))
    return picks


# ---------------------------------------------------------------- 镜像补边裁切
PAD_X, PAD_Y = 640, 360


def padded(img):
    return cv2.copyMakeBorder(img, PAD_Y, PAD_Y, PAD_X, PAD_X, cv2.BORDER_REFLECT_101)


def crop_shift(pimg, gx, gy, tx, ty):
    """从补边图上取 CW×CH 窗口，使红点落在窗口内 (tx, ty)。红点像素尺寸不变。"""
    x0 = int(round(gx + PAD_X - tx))
    y0 = int(round(gy + PAD_Y - ty))
    x0 = max(0, min(pimg.shape[1] - CW, x0))
    y0 = max(0, min(pimg.shape[0] - CH, y0))
    return pimg[y0:y0 + CH, x0:x0 + CW].copy(), (gx + PAD_X - x0, gy + PAD_Y - y0)


def cell_pos(cell, span, rng, jitter=0.26):
    f = 0.5 + rng.uniform(-jitter, jitter)
    return (cell - 1 + f) * span / 3.0


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="1")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--frames", type=int, default=40, help="E1 抽多少帧")
    ap.add_argument("--d", default="9,15,30,52")
    a = ap.parse_args()
    exps = set(a.exp.replace("E", "").replace("e", "").split(","))
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    frames = load_frames()
    print("可用帧 %d（已生成 inpaint 干净版）" % len(frames), flush=True)
    rng = random.Random(20261123)
    rows = []

    # ── E1 去黄框对照（全帧 1920→1280，与旧 bench 同口径）────────────────
    if "1" in exps:
        ks = sorted(frames)
        idx = np.linspace(0, len(ks) - 1, a.frames).astype(int)
        picks = [ks[i] for i in idx]
        print("\n=== E1 全帧 3 带（有黄框 vs 干净）%d 帧 ===" % len(picks), flush=True)
        for n, k in enumerate(picks):
            f = frames[k]
            trow = int(f["gy"] / (FH / 3.0)) + 1
            res = {}
            for cond in ("raw", "clean"):
                img = cv2.resize(f[cond], (CW, CH), interpolation=cv2.INTER_AREA)
                got = first_digit(ask(draw_rows(img), Q_ROW))
                res[cond] = got
            rows.append(dict(exp="E1", frame=k, d_px=round(f["d"], 1),
                             dist_m=round(FX * DOT_D / f["d"], 1), truth=trow,
                             got_raw=res["raw"], got_clean=res["clean"],
                             ok_raw=res["raw"] == trow, ok_clean=res["clean"] == trow))
            r = rows[-1]
            print("[%2d/%2d] 帧%-4d %5.1fm 真%d | 有框→%-4s %s | 干净→%-4s %s"
                  % (n + 1, len(picks), k, r["dist_m"], trow, res["raw"],
                     "对" if r["ok_raw"] else "错", res["clean"],
                     "对" if r["ok_clean"] else "错"), flush=True)
            dump(rows)
        for cond in ("raw", "clean"):
            sub = [r for r in rows if r["exp"] == "E1"]
            hit = sum(1 for r in sub if r["ok_" + cond])
            print("  E1 %-6s %d/%d = %.1f%%"
                  % ("有黄框" if cond == "raw" else "干净", hit, len(sub), 100.0 * hit / len(sub)))
        maj = Counter(r["truth"] for r in rows if r["exp"] == "E1").most_common(1)[0]
        print("  E1 多数类基线（永远答 %d）%d/%d" % (maj[0], maj[1], len([r for r in rows if r["exp"] == "E1"])))

    # ── E2/E3 真值分散（镜像补边，红点尺寸不变）────────────────────────
    for exp, axis, q in (("E2", "col", Q_COL), ("E3", "row", Q_ROW)):
        if exp[1:] not in exps:
            continue
        picks = pick_by_d(frames, [float(x) for x in a.d.split(",")])
        print("\n=== %s %s 真值分散（镜像补边）帧：%s ==="
              % (exp, "左右" if axis == "col" else "上下", [p[0] for p in picks]), flush=True)
        for (k, f) in picks:
            pim = padded(f["clean"])
            for cell in (1, 2, 3):
                for rep in range(a.reps):
                    if axis == "col":
                        tx, ty = cell_pos(cell, CW, rng), CH / 2.0 + rng.uniform(-60, 60)
                    else:
                        tx, ty = CW / 2.0 + rng.uniform(-60, 60), cell_pos(cell, CH, rng)
                    crop, (lx, ly) = crop_shift(pim, f["gx"], f["gy"], tx, ty)
                    truth = (int(lx / (CW / 3.0)) + 1) if axis == "col" else (int(ly / (CH / 3.0)) + 1)
                    deff = red_diam(crop, int(lx), int(ly))
                    got = first_digit(ask(draw_cols(crop) if axis == "col" else draw_rows(crop), q))
                    rows.append(dict(exp=exp, frame=k, d_px=round(f["d"], 1),
                                     dist_m=round(FX * DOT_D / f["d"], 1), d_eff=round(deff, 1),
                                     cell=cell, rep=rep, truth=truth, got=got, ok=got == truth))
                    print("[%s] 帧%-4d %5.1fm 放%d号格→真%d 点%4.1fpx | 答%-4s %s"
                          % (exp, k, FX * DOT_D / f["d"], cell, truth, deff, got,
                             "对" if got == truth else "错"), flush=True)
                    if rep == 0:
                        imwrite_u(OUT_DIR / ("%s_%s_f%d_c%d.jpg" % (exp, axis, k, cell)),
                                  draw_cols(crop) if axis == "col" else draw_rows(crop))
                    dump(rows)
        sub = [r for r in rows if r["exp"] == exp]
        hit = sum(1 for r in sub if r["ok"])
        print("  %s 合计 %d/%d = %.1f%%" % (exp, hit, len(sub), 100.0 * hit / len(sub)))
        for cell in (1, 2, 3):
            s2 = [r for r in sub if r["cell"] == cell]
            h2 = sum(1 for r in s2 if r["ok"])
            print("     真值 %d 号: %d/%d" % (cell, h2, len(s2)))

    json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % OUT_JSON)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
