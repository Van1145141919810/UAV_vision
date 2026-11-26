# -*- coding: utf-8 -*-
"""bench_vlm_strategy.py —— 把 VLM 做成"方向顾问"的可行性实验

测三件事（全部只当建议，不进飞控）：

  A. 方向格：横 3 带（上下）+ 竖 3 条（左右）各问一次 → 3×3 共 9 个方向格
     指标：行命中 / 列命中 / 格命中 / 相邻帧抖动 / 中值滤波能否压实 / 单次耗时
  B. 它能不能自己判远近：同一帧贴不同大小的红点，问"很远/中等/很近"
  C. 由此得出"远给方向、近交 CV"的切换判据

为什么是 3 选 1 而不是 4 或 9：
    真实帧实测（band_real.json）：3 带 3/3、4 带 2/3、6 带 1/3。
    小模型的可靠档位就在 3 选 1，所以用"两次 3 选 1"拼出 9 个方向格，
    而不是直接问"9 格里是哪一格"。
"""
from __future__ import annotations

import base64
import json
import pickle
import sys
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
from mcpv.cv import imread_unicode                        # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
GT = ROOT / "eval" / "gt_cache.pkl"
NREC = 40                       # 抽多少帧
TILE_W = 1280                   # 送模型前统一缩到 1280 宽

Q_ROW = ("这张图被横向切成 3 条带编号的横带（编号画在每条带左上角）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号横带？只回答一个数字。")
Q_COL = ("这张图被竖向切成 3 条编号的竖条（编号画在每条竖条上端）。"
         "图中有一个【贴在白色纸上的红色圆形标记】。它在几号竖条？只回答一个数字。")
Q_RANGE = ("图中有一个红色圆点（贴在白色纸上）。它在画面里看起来有多大？"
           "只回答一个字母：A = 很小（一眨眼就错过）；B = 中等；C = 很大（占了不小一块）。")


def ask(img, prompt, npd=12):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body,
                                headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(rq, timeout=300) as r:
        txt = (json.loads(r.read()).get("response") or "").strip()
    return txt, time.time() - t0


def draw_rows(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        y0, y1 = int(i * H / n), int((i + 1) * H / n)
        cv2.rectangle(out, (0, y0), (W - 1, y1), (0, 255, 255), 3)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (14, y0 + 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def draw_cols(img, n=3):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        x0, x1 = int(i * W / n), int((i + 1) * W / n)
        cv2.rectangle(out, (x0, 0), (x1 - 1, H - 1), (0, 255, 255), 3)
        for t, c, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, t, (x0 + 14, 62), cv2.FONT_HERSHEY_SIMPLEX, 2.0, c, th)
    return out


def first_digit(txt, lo, hi):
    for ch in txt:
        if ch.isdigit() and lo <= int(ch) <= hi:
            return int(ch)
    return None


def median_smooth(vals, k=5):
    out = []
    for i in range(len(vals)):
        w = [v for v in vals[max(0, i - k // 2):i + k // 2 + 1] if v is not None]
        out.append(int(np.median(w)) if w else None)
    return out


def main():
    if not GT.exists():
        print("缺真值 gt_cache.pkl，先跑 bench_band_real.py"); return
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

    ks = sorted(k for k in gt if k < len(frames))
    idx = np.linspace(0, len(ks) - 1, NREC).astype(int)
    picks = [ks[i] for i in idx]

    print("=" * 92)
    print("A. 3×3 方向格（横 3 带 + 竖 3 条，各一次 3 选 1）  %d 帧" % len(picks))
    print("=" * 92)
    print("%-6s %-7s %-6s %-5s %-6s %-5s %-6s %-6s %s"
          % ("帧", "点径px", "真行", "答行", "真列", "答列", "行对", "列对", "耗时s"))
    print("-" * 92)
    rows = []
    for i in picks:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        gx, gy, gd = gt[i]
        trow = int(gy / (H / 3.0)) + 1
        tcol = int(gx / (W / 3.0)) + 1
        img = cv2.resize(L, (TILE_W, int(TILE_W * H / W)), interpolation=cv2.INTER_AREA)
        a_row, dt1 = ask(draw_rows(img, 3), Q_ROW)
        a_col, dt2 = ask(draw_cols(img, 3), Q_COL)
        g_row = first_digit(a_row, 1, 3)
        g_col = first_digit(a_col, 1, 3)
        rows.append(dict(frame=int(i), d_px=float(gd), trow=trow, tcol=tcol,
                         grow=g_row, gcol=g_col,
                         ok_row=g_row == trow, ok_col=g_col == tcol,
                         dt=round(dt1 + dt2, 2), raw_row=a_row[:12], raw_col=a_col[:12]))
        print("%-6d %-7.1f %-6d %-5s %-6d %-5s %-6s %-6s %.2f"
              % (i, gd, trow, g_row, tcol, g_col,
                 "对" if g_row == trow else "错", "对" if g_col == tcol else "错", dt1 + dt2))

    n = len(rows)
    hr = sum(1 for r in rows if r["ok_row"])
    hc = sum(1 for r in rows if r["ok_col"])
    hb = sum(1 for r in rows if r["ok_row"] and r["ok_col"])
    jr = sum(1 for a, b in zip(rows, rows[1:]) if a["grow"] != b["grow"])
    jc = sum(1 for a, b in zip(rows, rows[1:]) if a["gcol"] != b["gcol"])

    mr = median_smooth([r["grow"] for r in rows])
    mc = median_smooth([r["gcol"] for r in rows])
    hrm = sum(1 for r, v in zip(rows, mr) if v == r["trow"])
    hcm = sum(1 for r, v in zip(rows, mc) if v == r["tcol"])
    hbm = sum(1 for r, v, w in zip(rows, mr, mc) if v == r["trow"] and w == r["tcol"])

    print()
    print("行命中 %d/%d = %.0f%%   列命中 %d/%d = %.0f%%   格命中 %d/%d = %.0f%%"
          % (hr, n, 100.0 * hr / n, hc, n, 100.0 * hc / n, hb, n, 100.0 * hb / n))
    print("相邻帧答案跳变：行 %d 次 / 列 %d 次（共 %d 个相邻对）" % (jr, jc, n - 1))
    print("中值滤波(窗5)后：行 %d/%d，列 %d/%d，格 %d/%d"
          % (hrm, n, hcm, n, hbm, n))
    print("单帧两次调用平均耗时 %.2f s（含两问）" % (sum(r["dt"] for r in rows) / n))

    print()
    print("=" * 92)
    print("B. 它能不能自己判远近（同一帧贴不同直径的红点，问很小/中等/很大）")
    print("=" * 92)
    bg0 = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    br = []
    print("%-8s %-8s %-8s %s" % ("点径px", "物理距离", "期望档", "回答"))
    for d_px, expect, dist in ((8, "A", "约 40 m"), (22, "B", "约 15 m"), (65, "C", "约 5 m")):
        bg = cv2.resize(bg0, (TILE_W, int(TILE_W * bg0.shape[0] / bg0.shape[1])),
                        interpolation=cv2.INTER_AREA)
        img = bg.copy()
        h, w = img.shape[:2]
        cv2.circle(img, (int(w * 0.55), int(h * 0.55)), max(1, d_px // 2),
                   (28, 28, 208), -1, lineType=cv2.LINE_AA)
        a, _ = ask(img, Q_RANGE, 8)
        br.append(dict(d_px=d_px, expect=expect, ans=a[:1]))
        print("%-8d %-8s %-8s %s" % (d_px, dist, expect, a[:24]))

    out = ROOT / "eval" / "vlm_strategy.json"
    json.dump({"dir_grid": rows, "range": br}, open(out, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % out)


if __name__ == "__main__":
    main()
