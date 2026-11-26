# -*- coding: utf-8 -*-
"""bench_marker_distance.py —— 50 米外窗户上的红点，到底能不能识别？

═══════════════════════════════════════════════════════════════════
先算账（这一步就能定性）
───────────────────────────────────────────────────────────────────
红圆实际直径 15 cm（config.MARKER_REAL_M，用户 2026-11-04 尺量）
焦距 f = 1566 px（对应 1920 宽的单目）

    像素直径 d = f × 0.15 / D

    D=10 m → 23.5 px
    D=20 m → 11.7 px
    D=30 m →  7.8 px
    D=50 m →  4.7 px    ← 师兄问的距离
    D=100 m → 2.35 px

对普通 CV（HSV 阈值 + 连通域）来说，4.7 px 直径 ≈ 17 个像素，
在 JPEG 压缩、抖动、光照变化下非常脆弱。

但算账不等于结论——所以下面用实验验证：
把带红点的原图按比例缩小，模拟各个距离下的成像，看检测器在哪一档开始失败。

═══════════════════════════════════════════════════════════════════
用法：python eval/bench_marker_distance.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, find_candidates      # noqa: E402
from mcpv import config as C                            # noqa: E402

F = C._K1[0]                     # 1566.15
REAL = C.MARKER_REAL_M           # 0.15 m
DISTANCES = [5, 10, 20, 30, 50, 80, 100]


def red_diameter(bgr):
    """用最宽松的红色阈值测出红圆直径（像素）。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 90, 60), (12, 255, 255))
    m2 = cv2.inRange(hsv, (168, 90, 60), (180, 255, 255))
    m = cv2.bitwise_or(m1, m2)
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return 0.0, 0
    i = 1 + int(np.argmax(st[1:, 4]))
    x, y, w, h, a = st[i]
    return float(max(w, h)), int(a)


def main():
    src = ROOT / "data" / "samples" / "test3.jpg"
    img = imread_unicode(src)
    if img is None:
        raise SystemExit(f"读不到 {src}")
    H, W = img.shape[:2]

    d0, a0 = red_diameter(img)
    print(f"原图 {W}x{H}，红圆直径约 {d0:.0f} px（面积 {a0} px）")
    if d0 <= 0:
        raise SystemExit("原图里没检测到红色区域")
    # 原图对应多远？  d = f*REAL/D  →  D = f*REAL/d
    D0 = F * REAL / d0
    print(f"→ 原图相当于在 {D0:.2f} m 处拍的\n")

    print(f"{'距离':<8}{'理论直径':<10}{'缩放后图宽':<12}{'红圆实测px':<12}"
          f"{'CV检测':<8}{'candidate数'}")
    print("-" * 64)
    for D in DISTANCES:
        d_theory = F * REAL / D
        k = d_theory / d0                    # 相对原图的缩放比
        w2, h2 = max(8, int(W * k)), max(8, int(H * k))
        small = cv2.resize(img, (w2, h2), interpolation=cv2.INTER_AREA)
        # 用 JPEG 压缩一下，更接近真实链路
        ok, buf = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        small = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        d_meas, a_meas = red_diameter(small)
        try:
            cands = find_candidates(small)
            ncand = len(cands)
            hit = any(abs(c.w - d_meas) < max(3, d_meas * 0.6) for c in cands) \
                if d_meas > 0 else False
        except Exception as e:
            ncand = -1
            hit = False
        print(f"{D:<8}{d_theory:<10.1f}{w2}x{h2:<8}{d_meas:<12.1f}"
              f"{('✓' if hit else '✗'):<8}{ncand}")

    print("\n说明：")
    print("  · CV 检测 = find_candidates()（项目现用的红色候选检测）")
    print("  · 缩放后图宽很小，说明红点在画面里只占几个像素")


if __name__ == "__main__":
    main()
