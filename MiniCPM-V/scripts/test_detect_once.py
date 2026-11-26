# -*- coding: utf-8 -*-
"""test_detect_once.py —— 抓一帧、跑红点检测、出标注图（静态验证）

用法：python scripts/test_detect_once.py [--marker-cm 20]
"""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode         # noqa: E402

DRONE = "visbot@192.168.2.20"
SSH_KEY = str(Path.home() / ".ssh" / "dsh_drone")
LOCAL = Path.home() / "AppData/Local/Temp" / "live_frame.jpg"
F_PX_1920 = 1566.1
OUT = ROOT / "eval" / "demo_out"


def grab():
    r = subprocess.run(["scp", "-i", SSH_KEY,
                        "-o", "StrictHostKeyChecking=no",
                        f"{DRONE}:/tmp/cpmv_frame.jpg", str(LOCAL)],
                       capture_output=True, timeout=40)
    if r.returncode != 0:
        print("scp 失败:", r.stderr.decode()[:120])
        return None
    return imread_unicode(LOCAL)


def detect(bgr, min_area=4):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
                       cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        a = int(st[i][4])
        if a < min_area:
            continue
        x, y, w, h = st[i][0], st[i][1], st[i][2], st[i][3]
        out.append({"cx": float(cen[i][0]), "cy": float(cen[i][1]),
                    "area": a, "d": 2 * (a / np.pi) ** 0.5,
                    "box": (int(x), int(y), int(w), int(h))})
    out.sort(key=lambda c: -c["d"])
    return out, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker-cm", type=float, default=20.0)
    ap.add_argument("--top", type=int, default=8)
    a = ap.parse_args()

    img = grab()
    if img is None:
        return 1
    H, W = img.shape[:2]
    f = F_PX_1920 * W / 1920.0
    cands, mask = detect(img)

    print(f"画面 {W}x{H}   焦距 {f:.0f} px   红圆 {a.marker_cm:.0f} cm")
    print(f"检出候选 {len(cands)} 个（按像素直径降序）")
    print(f"{'#':<4}{'d_px':<8}{'面积':<8}{'中心':<16}"
          f"{'若红圆是它 → 距离':<18}")
    for i, c in enumerate(cands[:a.top], 1):
        dist = f * (a.marker_cm / 100.0) / c["d"] if c["d"] > 0 else 0
        print(f"{i:<4}{c['d']:<8.1f}{c['area']:<8}"
              f"({c['cx']:.0f},{c['cy']:.0f})".ljust(16)
              + f"{dist:>8.1f} m")

    vis = img.copy()
    for i, c in enumerate(cands[:a.top], 1):
        x, y, w, h = c["box"]
        col = (80, 240, 90) if i == 1 else (60, 220, 255)
        pad = max(6, int(c["d"]))
        cv2.rectangle(vis, (x - pad, y - pad), (x + w + pad, y + h + pad), col, 3)
        cv2.putText(vis, f"{i}:{c['d']:.1f}px", (x - pad, y - pad - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    OUT.mkdir(parents=True, exist_ok=True)
    imwrite_unicode(OUT / "live_detect.jpg", vis, quality=92)
    imwrite_unicode(OUT / "live_mask.png", mask)
    print(f"\n标注图 {OUT / 'live_detect.jpg'}")
    print(f"红色掩膜 {OUT / 'live_mask.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
