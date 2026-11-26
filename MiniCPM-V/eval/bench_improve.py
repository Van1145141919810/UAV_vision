# -*- coding: utf-8 -*-
"""bench_improve.py —— 方案还能怎么优化？两项实测

═══════════════════════════════════════════════════════════════════
优化点一：红点像素直径 → 单目测距（免费的距离信息）
───────────────────────────────────────────────────────────────────
红圆实际直径已知（26 cm），焦距已知（1566 px），那么找到红点后：

    距离 D = f × 真实直径 / 像素直径

也就是说 **CV 一旦框出红点，距离就白送了**，不需要双目/激光。
但精度受像素量化限制：8 px 的红点差 1 px = 12.5% 测距误差。

实测：各距离下由检出的像素直径反推距离，看误差多大。

═══════════════════════════════════════════════════════════════════
优化点二：送模型的图更大，粗框会不会更准？
───────────────────────────────────────────────────────────────────
Ollama 会按自己的规则缩放输入。我们试 1280 / 1600 / 1920 三档，
看 VLM 粗框误差是否随分辨率改善（若改善明显，值得换成更大输入）。

用法：python eval/bench_improve.py
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

F = float(C._K1[0])
REAL = 0.26
W, H = 1920, 1080
TX, TY = 1210, 430
MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
P = ("Please provide the bounding box coordinate of the region "
     "this sentence describes: <ref>红色圆点</ref>")


def ask(img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
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
        return n[:4] if len(n) >= 4 else None
    runs = [int(x) for x in re.findall(r"\d+", txt)]
    if len(runs) >= 4:
        c = runs[-4:]
        if all(0 <= v <= 1000 for v in c):
            return c
    return None


def red_blob(bgr, min_area=3):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i][4])
        if a < min_area:
            continue
        if best is None or a > best[0]:
            best = (a, float(cen[i][0]), float(cen[i][1]))
    return best


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)

    print("=" * 74)
    print("优化点一：由红点像素直径反推距离（单目测距）")
    print("=" * 74)
    print(f"{'真实距离':<10}{'理论直径':<10}{'CV测得直径':<12}"
          f"{'反推距离':<12}{'误差':<10}{'误差%'}")
    print("-" * 74)
    for D in (30, 50, 80):
        d = F * REAL / D
        frame = bg.copy()
        cv2.circle(frame, (TX, TY), max(1, int(round(d / 2))), (30, 30, 205),
                   -1, lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        b = red_blob(frame, 3)
        if not b:
            print(f"{D:<10}{d:<10.1f}{'未检出':<12}")
            continue
        area, cx, cy = b
        d_meas = 2 * (area / np.pi) ** 0.5
        D_est = F * REAL / d_meas
        print(f"{D:<10}{d:<10.1f}{d_meas:<12.1f}{D_est:<12.1f}"
              f"{abs(D_est - D):<10.1f}{abs(D_est - D) / D * 100:.0f}%")

    print("\n" + "=" * 74)
    print("优化点二：送模型的分辨率对粗框精度的影响")
    print("=" * 74)
    d = F * REAL / 50
    frame = bg.copy()
    cv2.circle(frame, (TX, TY), max(1, int(round(d / 2))), (30, 30, 205), -1,
               lineType=cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

    print(f"{'送模型宽度':<12}{'模型框中心':<18}{'误差px':<10}{'耗时'}")
    print("-" * 74)
    for width in (1280, 1600, 1920):
        k = width / W
        scaled = cv2.resize(frame, (width, int(H * k)), interpolation=cv2.INTER_AREA)
        ans, dt = ask(scaled)
        box = parse_box(ans)
        if not box:
            print(f"{width:<12}{'无输出':<18}")
            continue
        cx = (box[0] + box[2]) / 2 / 1000 * W
        cy = (box[1] + box[3]) / 2 / 1000 * H
        e = ((cx - TX) ** 2 + (cy - TY) ** 2) ** 0.5
        print(f"{width:<12}({cx:.0f},{cy:.0f}){'':<6}{e:<10.0f}{dt:.1f}s")

    print("\n注：像素直径由连通域面积换算 d = 2*sqrt(A/pi)，")
    print("    小目标时量化误差会放大成距离误差。")


if __name__ == "__main__":
    main()
