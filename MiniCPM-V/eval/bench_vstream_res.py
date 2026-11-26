# -*- coding: utf-8 -*-
"""bench_vstream_res.py —— TCP 视频流分辨率下降后，VLM 还能出框吗？

═══════════════════════════════════════════════════════════════════
为什么这件事决定方案成败
───────────────────────────────────────────────────────────────────
红点的像素直径 = f × 真实直径 ÷ 距离，而 **f 随图像宽度成比例缩小**：

    1920 宽 → f = 1566 px
    1280 宽 → f = 1044 px
     960 宽 → f =  783 px

所以同一颗 20 cm 红圆在 50 m 处：

    1920 宽 → 6.3 px
    1280 宽 → 4.2 px
     960 宽 → 3.1 px   ← 这个尺寸大概率检测不到

═══════════════════════════════════════════════════════════════════
本脚本测什么
───────────────────────────────────────────────────────────────────
对 960 / 1280 / 1920 三种宽度，各画一颗**尺寸正确**的 20 cm@50m 红点
（不是同一颗点缩放，而是按该分辨率的焦距重新算），然后：

  A. VLM grounding 能不能给出合理框（误差 < 200 px 算命中）
  B. CV 低阈值能不能找到
  C. 理论像素直径

用法：python eval/bench_vstream_res.py
"""
import base64
import json
import re
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402
from mcpv import config as C                             # noqa: E402

F1920 = float(C._K1[0])          # 1566.1
REAL = 0.20
DIST = 50.0
MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
P = ("Please provide the bounding box coordinate of the region "
     "this sentence describes: <ref>红色圆点</ref>")


def ask(img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    body = json.dumps({"model": MODEL, "prompt": P,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": 64}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read()).get("response", "").strip(), time.time() - t0


def parse_box(txt):
    m = re.search(r"<box>\s*([\d\s,]+?)\s*</box>", txt)
    if m:
        n = [int(x) for x in re.findall(r"\d+", m.group(1))]
        if len(n) >= 4:
            return n[:4]
    runs = [int(x) for x in re.findall(r"\d+", txt)]
    if len(runs) >= 4:
        c = runs[-4:]
        if all(0 <= v <= 1000 for v in c):
            return c
    return None


def draw_exact(bgr, cx, cy, d_px, color=(30, 30, 205)):
    r = max(0.7, d_px / 2.0)
    y0, y1 = int(cy - r) - 2, int(cy + r) + 3
    x0, x1 = int(cx - r) - 2, int(cx + r) + 3
    yy, xx = np.mgrid[y0:y1, x0:x1]
    mask = ((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r
    patch = bgr[y0:y1, x0:x1]
    patch[mask] = color
    bgr[y0:y1, x0:x1] = patch
    return bgr


def red_area(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    best = 0
    for i in range(1, n):
        best = max(best, int(st[i][4]))
    return best


def main():
    src = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    print(f"20 cm 红圆 @ {DIST:.0f} m —— 各分辨率下的表现")
    print(f"{'宽度':<7}{'f(缩放后)':<11}{'理论px':<9}{'VLM出框':<10}"
          f"{'框误差px':<10}{'CV红像素':<10}{'耗时'}")
    print("-" * 72)

    for width in (1920, 1280, 960):
        f = F1920 * width / 1920.0
        d = f * REAL / DIST
        height = int(width * 9 / 16)
        frame = cv2.resize(src, (width, height), interpolation=cv2.INTER_AREA)
        frame = (frame.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)
        tx, ty = int(width * 0.63), int(height * 0.40)
        frame = draw_exact(frame, tx, ty, d)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        ans, dt = ask(frame)
        box = parse_box(ans)
        if box:
            cx = (box[0] + box[2]) / 2 / 1000 * width
            cy = (box[1] + box[3]) / 2 / 1000 * height
            err = ((cx - tx) ** 2 + (cy - ty) ** 2) ** 0.5
            s_box = "✓" if err < width * 0.10 else "✗"
            s_err = f"{err:.0f}"
        else:
            s_box, s_err = "✗", "-"
        area = red_area(frame)
        print(f"{width:<7}{f:<11.0f}{d:<9.2f}{s_box:<10}{s_err:<10}"
              f"{area:<10}{dt:.1f}s")

    print("\n判读：VLM出框 ✓ = 框中心误差 < 画面宽度的 10%")
    print("      CV红像素 = 低阈值能检出的红色像素数（<5 就基本没戏）")


if __name__ == "__main__":
    main()
