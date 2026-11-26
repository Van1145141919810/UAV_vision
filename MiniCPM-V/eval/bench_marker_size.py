# -*- coding: utf-8 -*-
"""bench_marker_size.py —— 加大红点能不能让 50 m 出框？（尺寸 × 距离 矩阵）

═══════════════════════════════════════════════════════════════════
要回答的问题
───────────────────────────────────────────────────────────────────
把红圆从 15 cm 加大到 26 cm（或更大），能在 50 m 内稳定出框吗？

像素直径  d = f × D_real / D       f = 1566 px

  15 cm @ 50 m = 4.7 px
  20 cm @ 50 m = 6.3 px
  26 cm @ 50 m = 8.1 px      ← 师兄问的
  30 cm @ 50 m = 9.4 px

之前的实测结论：8 px 左右是"项目检测器"能稳定出框的下限。
所以 26 cm 理论上够——但要用**项目现有检测器**验证，不能只看低阈值。

═══════════════════════════════════════════════════════════════════
每个组合测什么
───────────────────────────────────────────────────────────────────
  A 项目检测器 find_candidates：有没有给出红色候选、中心偏多少
  B 低阈值连通域：有没有找到
  C 对照：同背景**不放红点**时，检测器会不会误报（面积门槛随尺寸变化）

用法：python eval/bench_marker_size.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, find_candidates      # noqa: E402
from mcpv import config as C                            # noqa: E402

F = float(C._K1[0])
W, H = 1920, 1080
TX, TY = 1180, 420
SIZES_CM = [15, 20, 26, 30, 40]
DISTS = [30, 40, 50, 60, 80]


def red_blob(bgr, min_area=4):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    for i in range(1, n):
        if st[i][4] >= min_area:
            return True
    return False


def frame_with(bg, d_px, draw=True):
    f = bg.copy()
    if draw:
        cv2.circle(f, (TX, TY), max(1, int(round(d_px / 2))),
                   (30, 30, 205), -1, lineType=cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", f, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def main():
    bgs = []
    for n in ("win05.jpg", "win03.jpg"):
        b = imread_unicode(ROOT / "data" / "samples" / n)
        b = cv2.resize(b, (W, H), interpolation=cv2.INTER_AREA)
        b = (b.astype(np.float32) * 0.72).clip(0, 255).astype(np.uint8)
        bgs.append((n, b))

    # 先看对照组：不放红点时检测器有没有误报
    print("【对照组：不放红点】")
    for n, b in bgs:
        try:
            cands = find_candidates(b)
            print(f"  {n:<10} 项目检测器 {len(cands)} 候选"
                  + ("  ← 误报！" if cands else "  ✓ 干净")
                  + f"  低阈值 {'有红块' if red_blob(b) else '无 ✓'}")
        except Exception as e:
            print(f"  {n:<10} 异常 {type(e).__name__}")

    print("\n【红点检测矩阵】  A=项目检测器出候选  B=低阈值找到  误差=中心偏差px")
    for size_cm in SIZES_CM:
        real = size_cm / 100.0
        print(f"\n=== 红点直径 {size_cm} cm ===")
        hdr = "  距离  " + "".join(f"{d} m".rjust(14) for d in DISTS)
        print(hdr)
        row_a, row_b, row_e = "  A   ", "  B   ", "  误差"

        for D in DISTS:
            d_px = F * real / D
            hits_a, hits_b, errs = 0, 0, []
            for _, b in bgs:
                f = frame_with(b, d_px, True)
                try:
                    cands = find_candidates(f)
                    ok = any(abs(c.cx - TX) < 60 and abs(c.cy - TY) < 60
                             for c in cands)
                    if ok:
                        hits_a += 1
                        c0 = min(cands, key=lambda c: (c.cx - TX) ** 2 + (c.cy - TY) ** 2)
                        errs.append((((c0.cx - TX) ** 2 + (c0.cy - TY) ** 2) ** 0.5))
                except Exception:
                    pass
                if red_blob(f):
                    hits_b += 1

            mark_a = "✓" if hits_a == len(bgs) else ("~" if hits_a else "✗")
            mark_b = "✓" if hits_b == len(bgs) else ("~" if hits_b else "✗")
            row_a += f"{mark_a}({d_px:.0f}px)".rjust(14)
            row_b += mark_b.rjust(14)
            row_e += (f"{np.mean(errs):.0f}" if errs else "-").rjust(14)

        print(row_a)
        print(row_b)
        print(row_e)

    print("\n判读：✓=两张背景都成功  ~=只成功一张  ✗=都没成功")
    print("      (xxpx) 是该距离下的理论像素直径")


if __name__ == "__main__":
    main()
