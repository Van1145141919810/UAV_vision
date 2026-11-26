# -*- coding: utf-8 -*-
"""bench_marker_a4.py —— 20 cm（A4 可印）方案在 50 m 的真实可行性

═══════════════════════════════════════════════════════════════════
背景约束
───────────────────────────────────────────────────────────────────
A4 = 210 x 297 mm
  · 26 cm 圆（260 mm）> 210 mm 宽 → **A4 印不下**
  · 20 cm 圆（200 mm）= 210 mm 宽 - 两边各 5 mm → **刚好能印**
  · 26 cm 需要 A3（297 x 420 mm）

═══════════════════════════════════════════════════════════════════
20 cm 在各距离的像素直径
───────────────────────────────────────────────────────────────────
  d = 1566 x 0.20 / D
  30 m → 10.4 px
  40 m →  7.8 px
  50 m →  6.3 px   ← 起飞距离，关键档
  60 m →  5.2 px

上一轮实测（精确绘制）显示：项目检测器在 ~6 px 附近**因背景而异**，
所以 50 m 这一档必须两套背景都测，并给出"放宽门槛"的兜底。

用法：python eval/bench_marker_a4.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, find_candidates      # noqa: E402
from mcpv import config as C                            # noqa: E402

F = float(C._K1[0])
W, H, TX, TY = 1920, 1080, 1210, 430
SIZES = [("A4 20cm", 0.20), ("A3 26cm", 0.26), ("老 15cm", 0.15)]
DISTS = [30, 40, 50, 60]


def draw_exact(bgr, cx, cy, d_px, color=(30, 30, 205)):
    r = d_px / 2.0
    y0, y1 = int(cy - r) - 2, int(cy + r) + 3
    x0, x1 = int(cx - r) - 2, int(cx + r) + 3
    yy, xx = np.mgrid[y0:y1, x0:x1]
    mask = ((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r
    patch = bgr[y0:y1, x0:x1]
    patch[mask] = color
    bgr[y0:y1, x0:x1] = patch
    return bgr


def low_thr_hit(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    for i in range(1, n):
        a = st[i][4]
        if a < 4:
            continue
        cx = st[i][0] + st[i][2] / 2
        cy = st[i][1] + st[i][3] / 2
        if abs(cx - TX) < 60 and abs(cy - TY) < 60:
            return True, int(a)
    return False, 0


def main():
    bgs = []
    for n in ("win05.jpg", "win03.jpg"):
        b = imread_unicode(ROOT / "data" / "samples" / n)
        b = cv2.resize(b, (W, H), interpolation=cv2.INTER_AREA)
        b = (b.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)
        bgs.append((n, b))

    print("每格：项目检测器 / 低阈值 各自在【两套背景】上的命中情况")
    print("     ✓✓ = 都命中    ✓✗ = 只有一套    ✗✗ = 都不行\n")

    for label, real in SIZES:
        print(f"=== {label} ===")
        print("  距离  直径px   项目检测器      低阈值")
        for D in DISTS:
            d = F * real / D
            res_proj, res_low = [], []
            for _, bg in bgs:
                fr = draw_exact(bg.copy(), TX, TY, d)
                ok, buf = cv2.imencode(".jpg", fr, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
                fr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                try:
                    cands = find_candidates(fr)
                    hit = any(abs(c.cx - TX) < 60 and abs(c.cy - TY) < 60
                              for c in cands)
                except Exception:
                    hit = False
                res_proj.append("✓" if hit else "✗")
                ok2, area = low_thr_hit(fr)
                res_low.append("✓" if ok2 else "✗")
            print(f"  {D:<6}{d:<9.1f}{''.join(res_proj):<16}{''.join(res_low)}")
        print()


if __name__ == "__main__":
    main()
