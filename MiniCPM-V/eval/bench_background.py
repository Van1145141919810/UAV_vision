# -*- coding: utf-8 -*-
"""bench_background.py —— 红点贴在"窗户中央 / 墙 / 窗框 / 砖墙"上，检测有区别吗？

═══════════════════════════════════════════════════════════════════
要澄清的误解
───────────────────────────────────────────────────────────────────
检测器**只认红色，不认窗户**。所以理论上贴哪儿都能检出。

但实际存在三个**与背景有关**的影响：
  1. 背景颜色（红色砖墙/红色招牌会混）
  2. 背景亮度（对比度）
  3. 反光（贴玻璃上可能被镜面反光冲淡）

本脚本在同一张立面图上选 5 个不同性质的背景位置，
用同样的红点尺寸画上去，比较"能不能检出 + 红色像素数 + 局部对比度"。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, find_candidates      # noqa: E402

W, H = 1920, 1080
F = 1566.1
REAL = 0.26
DIST = 50.0


def draw_sub(frame, cx, cy, d, color=(25, 25, 200), S=4):
    r = max(0.6, d / 2)
    half = int(r) + 3
    size = 2 * half + 1
    x0, y0 = int(cx) - half, int(cy) - half
    big = np.zeros((size * S, size * S, 3), np.uint8)
    cv2.circle(big, (int(round((cx - x0) * S)), int(round((cy - y0) * S))),
               int(round(r * S)), color, -1, lineType=cv2.LINE_AA)
    small = cv2.resize(big, (size, size), interpolation=cv2.INTER_AREA)
    m = small.any(axis=2)
    reg = frame[y0:y0 + size, x0:x0 + size]
    reg[m] = small[m]
    frame[y0:y0 + size, x0:x0 + size] = reg


def red_stats(frame, cx, cy, half=12):
    """中心区的红色像素数与局部对比度（红-绿）。"""
    p = frame[max(0, cy - half):cy + half, max(0, cx - half):cx + half]
    hsv = cv2.cvtColor(p, cv2.COLOR_BGR2HSV)
    red = cv2.bitwise_or(cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
                         cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    b, g, r = (p[:, :, i].astype(float) for i in range(3))
    contrast = float((r - g).max())
    return int(red.sum() / 255), contrast


def main():
    img = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    d = F * REAL / DIST
    print(f"红点 {REAL*100:.0f} cm @ {DIST:.0f} m → {d:.1f} px\n")

    # 在图上挑几个不同性质的背景点（人工看 win05 选的）
    spots = [
        ("玻璃中央（深色）", 300, 260),
        ("混凝土墙面（浅灰）", 620, 620),
        ("窗框金属条（亮）", 210, 300),
        ("玻璃+窗帘（中灰）", 420, 470),
        ("下方瓷砖/抹灰", 700, 860),
    ]

    # 追加两种"红色背景"干扰场景
    def add_brick(f, cx, cy, w=26, h=12):
        cv2.rectangle(f, (cx - w, cy - h), (cx + w, cy + h), (40, 45, 150), -1)

    spots_extra = [("★ 红砖旁边（砖更大）", 760, 700, "brick")]
    print(f"{'背景':<22}{'局部红色px':<12}{'最大红-绿差':<12}"
          f"{'检测器':<10}{'红色像素@点内'}")
    print("-" * 74)
    for name, cx, cy in spots:
        f0 = img.copy()
        ok, buf = cv2.imencode(".jpg", f0, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        f0 = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        r0, c0 = red_stats(f0, cx, cy)

        f1 = f0.copy()
        draw_sub(f1, cx, cy, d)
        ok, buf = cv2.imencode(".jpg", f1, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        f1 = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        r1, c1 = red_stats(f1, cx, cy)

        try:
            cands = find_candidates(f1)
            hit = any(abs(c.cx - cx) < 25 and abs(c.cy - cy) < 25 for c in cands)
            det = f"{len(cands)}候选 {'✓' if hit else '✗'}"
        except Exception as e:
            det = f"异常{type(e).__name__}"

        print(f"{name:<22}{r0:<12}{c0:<12}{det:<10}{r1}")

    # 红砖场景：砖块 52x24 px，明显大于 8 px 的红点
    cx, cy = 760, 700
    f1 = img.copy()
    ok, buf = cv2.imencode(".jpg", f1, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    f1 = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    add_brick(f1, cx + 40, cy)
    draw_sub(f1, cx, cy, d)
    ok, buf = cv2.imencode(".jpg", f1, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    f1 = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    try:
        cands = find_candidates(f1)
        near = [c for c in cands if abs(c.cx - cx) < 25 and abs(c.cy - cy) < 25]
        det = f"{len(cands)}候选 命中{len(near)}"
    except Exception as e:
        det = f"异常{type(e).__name__}"
    print(f"{'红砖(52x24px)在旁边':<22}{'-':<12}{'-':<12}{det:<10}-")


if __name__ == "__main__":
    main()
