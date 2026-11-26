# -*- coding: utf-8 -*-
"""bench_merge_opt.py —— 回答两个问题：
   ① 两次询问到底是不是"排队"？能不能并行？
   ② 能不能**只问一次**就同时拿到行和列？（省一半时间和算力）

① 并发测试：两个不同 nonce 的请求同时发，看 wall time 对比串行
② 精度测试（16 帧真录像，真值来自链接轨迹）：
     V0 基线：横 3 带、竖 3 条，各问一次（已知 78%/80%）
     V1 一次问 9 格（3×3 编号画进图里，答 1~9）
     V2 一次问两个数（同一张 3×3 图，答"先横带后竖条"）
   若 V1/V2 能到 70%+，以后就只问一次。
"""
from __future__ import annotations

import base64
import json
import pickle
import sys
import threading
import time
import urllib.request
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

Q_ROW = ("这张图被横向切成 3 条带编号的横带（编号画在每条带左上角）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号横带？只回答一个数字。")
Q_COL = ("这张图被竖向切成 3 条编号的竖条（编号画在每条竖条上端）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号竖条？只回答一个数字。")
Q9 = ("这张图被切成 9 个带编号的格子（3 行 3 列，编号 1~9 画在每格左上角）。"
      "图中有一个【贴在白色纸上的红色圆形标记】。它在几号格？只回答一个数字。")
Q2 = ("这张图被切成 3 行 3 列。图中有一个【贴在白色纸上的红色圆形标记】。"
      "请回答两个数字：先答它在第几行（1~3，从上往下数），再答第几列（1~3，从左往右数）。"
      "格式示例：2 1。只回答这两个数字。")


def _post(img, prompt, npd=12):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body,
                                headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=300) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def draw_rows(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        y0 = int(i * H / n)
        cv2.rectangle(out, (0, y0), (W - 1, int((i + 1) * H / n)), (0, 255, 255), 2)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (14, y0 + 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def draw_cols(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        x0 = int(i * W / n)
        cv2.rectangle(out, (x0, 0), (int((i + 1) * W / n) - 1, H - 1), (0, 255, 255), 2)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (x0 + 14, 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def draw_9(img):
    out = img.copy()
    H, W = out.shape[:2]
    k = 1
    for r in range(3):
        for c in range(3):
            y0, y1 = int(r * H / 3), int((r + 1) * H / 3)
            x0, x1 = int(c * W / 3), int((c + 1) * W / 3)
            cv2.rectangle(out, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 2)
            for t, cc, th in ((str(k), (0, 0, 0), 8), (str(k), (0, 255, 255), 4)):
                cv2.putText(out, t, (x0 + 12, y0 + 52), cv2.FONT_HERSHEY_SIMPLEX, 1.5, cc, th)
            k += 1
    return out


def digits(txt, lo=1, hi=3):
    return [int(c) for c in (txt or "") if c.isdigit() and lo <= int(c) <= 9]


def main():
    fr_img = None
    with open(GT, "rb") as f:
        d = pickle.load(f)
    gt = {}
    gt.update(d["FAR"])
    gt.update(d["NEAR"])

    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
    cap = cv2.VideoCapture(str(VID))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()

    # ---------- ① 并发 vs 串行 ----------
    print("=" * 80)
    print("① 两次询问是不是排队（同一个模型实例）")
    print("=" * 80)
    i0 = sorted(gt)[len(gt) // 2]
    L, _ = rig.rectify(frames[i0], frames[i0])
    img = cv2.resize(L, (640, 360), interpolation=cv2.INTER_AREA)
    labels = [draw_rows(img), draw_cols(img)]

    t0 = time.time()
    _post(labels[0], Q_ROW + "（测序 A）")
    t_a = time.time() - t0
    t0 = time.time()
    _post(labels[1], Q_COL + "（测序 B）")
    t_b = time.time() - t0
    seq = t_a + t_b
    print("串行：%.2f s + %.2f s = %.2f s" % (t_a, t_b, seq))

    res = {}

    def worker(k):
        t = time.time()
        _post(labels[k], (Q_ROW if k == 0 else Q_COL) + "（并发 C%d）" % int(time.time() * 1000))
        res[k] = time.time() - t

    t0 = time.time()
    th = [threading.Thread(target=worker, args=(k,)) for k in (0, 1)]
    for t in th:
        t.start()
    for t in th:
        t.join()
    wall = time.time() - t0
    print("并发：两个请求各自 %.2f / %.2f s，wall = %.2f s" % (res.get(0, -1), res.get(1, -1), wall))
    print("→ %s" % ("并行生效（wall ≈ 单次）" if wall < seq * 0.75
                    else "确实串行排队（wall ≈ 两次之和）"))

    # ---------- ② 能否只问一次 ----------
    print()
    print("=" * 80)
    print("② 只问一次的两种问法 vs 基线（16 帧）")
    print("=" * 80)
    ks = sorted(gt)
    idx = np.linspace(0, len(ks) - 1, 16).astype(int)
    picks = [ks[i] for i in idx]

    print("%-6s %-7s %-6s %-8s %-8s %-8s %-8s"
          % ("帧", "点径", "真(行,列)", "V1九格", "V2两数", "V0基线行", "耗时V1/V2"))
    print("-" * 80)
    v1 = v2 = v0 = 0
    n = 0
    t_v1 = t_v2 = 0.0
    for i in picks:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        gx, gy, gd = gt[i]
        tr, tc = int(gy / (H / 3)) + 1, int(gx / (W / 3)) + 1
        im = cv2.resize(L, (640, 360), interpolation=cv2.INTER_AREA)

        a1 = _post(draw_9(im), Q9)
        g1 = digits(a1)
        c1 = (g1[0] if g1 else None)

        t0 = time.time()
        a2 = _post(draw_9(im), Q2)
        t_v2 += time.time() - t0
        g2 = digits(a2)
        pair = (g2[0], g2[1]) if len(g2) >= 2 else (None, None)

        t0 = time.time()
        ar = _post(draw_rows(im), Q_ROW)
        t_v1 += time.time() - t0
        gr = digits(ar)
        gr = gr[0] if gr else None

        ok1 = (c1 == (tr - 1) * 3 + tc)
        ok2 = (pair == (tr, tc))
        ok0 = (gr == tr)
        v1 += ok1
        v2 += ok2
        v0 += ok0
        n += 1
        print("%-6d %-7.1f %-6s %-8s %-8s %-8s %.1f"
              % (i, gd, "%d,%d" % (tr, tc),
                 "%s%s" % (c1, "对" if ok1 else "错"),
                 "%s%s" % (str(pair), "对" if ok2 else "错"),
                 "%s%s" % (gr, "对" if ok0 else "错"),
                 t_v1 + t_v2))

    print()
    print("V1 九格一次问   %2d/%d = %.0f%%" % (v1, n, 100.0 * v1 / n))
    print("V2 两个数一次问 %2d/%d = %.0f%%" % (v2, n, 100.0 * v2 / n))
    print("V0 基线(只问行) %2d/%d = %.0f%%  → 行与列各问一次才是 78%%/80%%" % (v0, n, 100.0 * v0 / n))
    print("V2 平均一次 %.2f s；V1 平均一次 %.2f s" % (t_v2 / n, t_v1 / n))

    out = ROOT / "eval" / "merge_opt.json"
    json.dump({"serial_s": seq, "parallel_wall_s": wall,
               "V1_9grid": [v1, n], "V2_two_numbers": [v2, n], "V0_row_only": [v0, n]},
              open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("结果 -> %s" % out)


if __name__ == "__main__":
    main()
