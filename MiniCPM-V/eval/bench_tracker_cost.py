# -*- coding: utf-8 -*-
"""bench_tracker_cost.py —— 跟踪/预测方案的实测耗时（决定选哪个）

═══════════════════════════════════════════════════════════════════
为什么先测
───────────────────────────────────────────────────────────────────
用户要求"一定要轻量、延迟低"。所以不猜，直接在同一帧上量：

  · OpenCV 自带跟踪器：MOSSE / KCF / CSRT（5.x 里部分可能已移除）
  · 模板匹配（小窗口 NCC）
  · 稀疏光流（LK，用于背景运动预测）
  · 我们现在的 HSV 检测（基线）

判定标准：10.4 fps → 每帧预算 96 ms。
═══════════════════════════════════════════════════════════════════
"""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402

W, H = 960, 540
N = 40


def bench(fn, n=N, warm=3):
    for _ in range(warm):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n * 1000.0


def main():
    img = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    img = (img.astype(np.float32) * 0.8).clip(0, 255).astype(np.uint8)

    # 造一个 60x60 的"目标"区域
    box = (400, 200, 60, 60)
    x, y, w, h = box
    cv2.circle(img, (x + 30, y + 30), 14, (30, 30, 205), -1, lineType=cv2.LINE_AA)
    tmpl = img[y:y + h, x:x + w].copy()

    print(f"帧 {W}x{H}   单帧预算（10.4 fps）= 96 ms   每项跑 {N} 次取均值")
    print("-" * 66)

    rows = []

    # 1) HSV 检测（基线）
    def hsv_detect():
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
            cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
        return n
    rows.append(("HSV 检测（现在用的）", bench(hsv_detect)))

    # 2) 模板匹配（小窗口）
    pad = 90
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(W, x + w + pad), min(H, y + h + pad)
    search = img[y0:y1, x0:x1]

    def tmpl_match():
        r = cv2.matchTemplate(search, tmpl, cv2.TM_CCOEFF_NORMED)
        return cv2.minMaxLoc(r)
    rows.append((f"模板匹配 {w}x{h}@{search.shape[1]}x{search.shape[0]}",
                 bench(tmpl_match)))

    # 3) 稀疏光流（背景 30 个点）
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.full_like(gray, 255)
    mask[y - 20:y + h + 20, x - 20:x + w + 20] = 0      # 挖掉目标区
    pts = cv2.goodFeaturesToTrack(gray, 30, 0.01, 10, mask=mask)
    gray2 = np.roll(gray, 2, axis=1)                    # 造一点平移当"下一帧"

    def lk_flow():
        if pts is None:
            return None
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            gray, gray2, pts, None,
            winSize=(15, 15), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        return p1
    rows.append((f"稀疏光流 {0 if pts is None else len(pts)} 点", bench(lk_flow)))

    # 4) OpenCV 跟踪器（5.x 里可能在 legacy）
    for name, ctor in (("MOSSE", "TrackerMOSSE_create"),
                       ("KCF", "TrackerKCF_create"),
                       ("CSRT", "TrackerCSRT_create")):
        fn = getattr(cv2, ctor, None)
        if fn is None and hasattr(cv2, "legacy"):
            fn = getattr(cv2.legacy, ctor, None)
        if fn is None:
            rows.append((f"{name} 跟踪器", None))
            continue
        try:
            tr = fn()
            tr.init(img, box)
            t0 = time.perf_counter()
            for _ in range(10):
                tr.update(img)
            ms = (time.perf_counter() - t0) / 10 * 1000
            rows.append((f"{name} 跟踪器（update）", ms))
        except Exception as e:
            rows.append((f"{name} 跟踪器", f"不可用({type(e).__name__})"))

    for name, ms in rows:
        if ms is None:
            print(f"  {name:<34} 不存在")
        elif isinstance(ms, str):
            print(f"  {name:<34} {ms}")
        else:
            pct = ms / 96.0 * 100
            flag = "✓" if pct < 20 else ("~" if pct < 50 else "✗ 太重")
            print(f"  {name:<34} {ms:7.2f} ms   占预算 {pct:5.1f}%  {flag}")

    print("\n注：跟踪器只测了 update；init 只在锁定/重捕获时各一次。")


if __name__ == "__main__":
    main()
