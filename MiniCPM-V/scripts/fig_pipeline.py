# -*- coding: utf-8 -*-
"""fig_pipeline.py —— 把"VLM 粗定位 → CV 精定位"的成果画成证据图

═══════════════════════════════════════════════════════════════════
这张图要一眼看懂三件事
───────────────────────────────────────────────────────────────────
  1. 大模型给的框**很粗**（误差约 100 px），但足够用
  2. 裁剪必须外扩 150 px（否则真目标落在框外——实测栽过一次）
  3. CV 在框内精定位到**0 误差**

每行一张图，三列：
  左  全画面（1920x1080 缩略）+ VLM 绿框 + 真实位置白十字
  中  VLM 框外扩后的裁剪区（CV 的工作区）
  右  同一区域，CV 找到的精确中心（红框）
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
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv import config as C                              # noqa: E402

F = float(C._K1[0])
W, H = 1920, 1080
TX, TY = 1210, 430
MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
P = ("Please provide the bounding box coordinate of the region "
     "this sentence describes: <ref>红色圆点</ref>")
FONT = r"C:\Windows\Fonts\msyh.ttc"


def ask(img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": P,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": 64}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read()).get("response", "").strip()


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


def red_center(bgr, min_area=3):
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


def to_pil(bgr):
    return Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)

    COL1, COL2, ROWH = 533, 300, 300
    f_h = ImageFont.truetype(FONT, 26)
    f_t = ImageFont.truetype(FONT, 18)
    f_s = ImageFont.truetype(FONT, 16)

    rows = []
    for D in (30, 50, 80):
        d = F * 0.26 / D
        frame = bg.copy()
        cv2.circle(frame, (TX, TY), max(1, int(round(d / 2))), (30, 30, 205),
                   -1, lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        ans = ask(frame)
        box = parse_box(ans)
        x1 = box[0] / 1000 * W; y1 = box[1] / 1000 * H
        x2 = box[2] / 1000 * W; y2 = box[3] / 1000 * H
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        e1 = ((cx - TX) ** 2 + (cy - TY) ** 2) ** 0.5

        pad = max(150, int(max(x2 - x1, y2 - y1)))
        ax1, ay1 = int(max(0, x1 - pad)), int(max(0, y1 - pad))
        ax2, ay2 = int(min(W, x2 + pad)), int(min(H, y2 + pad))
        crop = frame[ay1:ay2, ax1:ax2].copy()

        b = red_center(crop, min_area=3)
        gx, gy = (ax1 + b[1], ay1 + b[2]) if b else (None, None)
        e2 = ((gx - TX) ** 2 + (gy - TY) ** 2) ** 0.5 if b else None

        # 左：全图缩略
        full = frame.copy()
        cv2.rectangle(full, (int(x1), int(y1)), (int(x2), int(y2)), (80, 230, 80), 6)
        cv2.drawMarker(full, (TX, TY), (255, 255, 255), cv2.MARKER_CROSS, 40, 3)
        left = to_pil(cv2.resize(full, (COL1, ROWH)))

        # 中：裁剪区（放大到 COL2）
        mid_img = crop.copy()
        cv2.rectangle(mid_img, (int(x1) - ax1, int(y1) - ay1),
                      (int(x2) - ax1, int(y2) - ay1), (80, 230, 80), 2)
        mid = to_pil(cv2.resize(mid_img, (COL2, COL2), interpolation=cv2.INTER_NEAREST))

        # 右：CV 结果
        fin = crop.copy()
        if b:
            cv2.drawMarker(fin, (int(gx - ax1), int(gy - ay1)), (60, 60, 245),
                           cv2.MARKER_CROSS, 30, 2)
        right = to_pil(cv2.resize(fin, (COL2, COL2), interpolation=cv2.INTER_NEAREST))

        rows.append((D, d, e1, e2, left, mid, right))
        print(f"{D}m 直径{d:.1f} VLM误差{e1:.0f} 最终误差{e2 if e2 is None else round(e2)}")

    rowH = ROWH + 56
    sheet = Image.new("RGB", (COL1 + COL2 * 2 + 60, 74 + rowH * len(rows) + 30),
                      (16, 16, 16))
    dd = ImageDraw.Draw(sheet)
    dd.text((14, 12), "成果：VLM 粗定位 → CV 精定位（26 cm 红圆，1920×1080）",
            font=f_h, fill=(0, 220, 255))
    dd.text((14, 42),
            "左＝全画面与模型给的绿框；中＝外扩后的搜索区；右＝CV 找到的精确中心（红十字）",
            font=f_s, fill=(170, 200, 220))

    y = 74
    for D, d, e1, e2, left, mid, right in rows:
        sheet.paste(left, (14, y))
        sheet.paste(mid, (14 + COL1 + 14, y))
        sheet.paste(right, (14 + COL1 + COL2 + 28, y))
        fin = ("最终误差 0 px" if e2 is not None and e2 < 1
               else (f"最终误差 {e2:.0f} px" if e2 is not None else "CV 未找到"))
        dd.text((14, y + ROWH + 6),
                f"{D} m   红点 {d:.1f} px   模型框误差 {e1:.0f} px   {fin}",
                font=f_t, fill=(255, 220, 90))
        y += rowH

    out = ROOT / "eval" / "figures" / "pipeline_demo.jpg"
    out.parent.mkdir(exist_ok=True)
    sheet.save(out, quality=93)
    print(f"\n出图 {out}  ({sheet.width}x{sheet.height})")


if __name__ == "__main__":
    main()
