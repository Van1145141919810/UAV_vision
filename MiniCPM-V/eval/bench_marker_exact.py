# -*- coding: utf-8 -*-
"""bench_marker_exact.py —— 用"精确像素面积"重画红点，修正合成偏差

═══════════════════════════════════════════════════════════════════
为什么要重做
───────────────────────────────────────────────────────────────────
之前用 cv2.circle(radius=r, lineType=LINE_AA) 画红点，实测发现
**画出来的直径比理论值大 2~3 像素**：

    理论 8.1 px → 实测连通域等效直径 10.9 px（+35%）
    理论 5.1 px → 实测 8.8 px（+73%）

原因是抗锯齿边缘 + 半径取整。真实相机里的红圆不会有这个放大，
所以此前的"50 m 8 px 能检出"是**偏乐观**的。

本脚本改用精确绘制：按面积算像素，不加抗锯齿，再跑同一套矩阵。
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


def draw_exact(bgr, cx, cy, d_px, color=(30, 30, 205)):
    """按直径精确画实心圆（无抗锯齿）：半径内像素全填。"""
    r = d_px / 2.0
    y0, y1 = int(cy - r) - 2, int(cy + r) + 3
    x0, x1 = int(cx - r) - 2, int(cx + r) + 3
    yy, xx = np.mgrid[y0:y1, x0:x1]
    mask = ((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r
    patch = bgr[y0:y1, x0:x1]
    patch[mask] = color
    bgr[y0:y1, x0:x1] = patch
    return bgr


def measure(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i][4])
        if a < 2:
            continue
        if best is None or a > best[0]:
            best = (a, st[i])
    return best


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)

    print("① 绘制精度校验（应接近理论值）")
    print(f"{'距离':<7}{'理论px':<10}{'实际画出的等效直径':<20}{'偏差'}")
    print("-" * 56)
    frames = {}
    for D in (30, 50, 80):
        d = F * 0.26 / D
        fr = draw_exact(bg.copy(), TX, TY, d)
        ok, buf = cv2.imencode(".jpg", fr, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        fr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        b = measure(fr)
        dm = 2 * (b[0] / np.pi) ** 0.5 if b else 0
        frames[D] = fr
        print(f"{D:<7}{d:<10.1f}{dm:<20.1f}{(dm - d):+.1f} px")

    print("\n② 26 cm 红圆：项目检测器 / 低阈值 在修正后的图上还灵不灵")
    print(f"{'距离':<7}{'直径px':<9}{'项目检测器':<14}{'低阈值'}")
    print("-" * 56)
    for D in (30, 50, 80):
        fr = frames[D]
        d = F * 0.26 / D
        try:
            cands = find_candidates(fr)
            hit = any(abs(c.cx - TX) < 60 and abs(c.cy - TY) < 60 for c in cands)
            s1 = f"{len(cands)} 候选 {'✓' if hit else '✗'}"
        except Exception as e:
            s1 = f"异常{type(e).__name__}"
        b = measure(fr)
        s2 = f"找到 {b[0]}px ✓" if b else "未找到 ✗"
        print(f"{D:<7}{d:<9.1f}{s1:<14}{s2}")

    # 15 cm 对照
    print("\n③ 15 cm 红圆（当前方案）对照")
    print(f"{'距离':<7}{'直径px':<9}{'项目检测器':<14}{'低阈值'}")
    print("-" * 56)
    for D in (30, 50, 80):
        d = F * 0.15 / D
        fr = draw_exact(bg.copy(), TX, TY, d)
        ok, buf = cv2.imencode(".jpg", fr, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        fr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        try:
            cands = find_candidates(fr)
            hit = any(abs(c.cx - TX) < 60 and abs(c.cy - TY) < 60 for c in cands)
            s1 = f"{len(cands)} 候选 {'✓' if hit else '✗'}"
        except Exception as e:
            s1 = f"异常{type(e).__name__}"
        b = measure(fr)
        s2 = f"找到 {b[0]}px ✓" if b else "未找到 ✗"
        print(f"{D:<7}{d:<9.1f}{s1:<14}{s2}")


if __name__ == "__main__":
    main()
