# -*- coding: utf-8 -*-
"""fig_grounding.py —— 把"纯大模型出框"的结果画出来给人看

═══════════════════════════════════════════════════════════════════
这张图要回答
───────────────────────────────────────────────────────────────────
"不用 CV，光靠大模型，它框出来的窗户/红点长什么样、准不准？"

分两部分：
  A 红点定位（5 个尺寸）：绿框=模型给的框，白十字=真实位置，标误差
  B 窗户定位：在真实立面图上让它定位"窗户"，看它框在哪；
              再拿一张没有窗户的图（室内桌面）问同样的问题，
              看它会不会硬编一个框出来。

用法：python scripts/fig_grounding.py [--open]
"""
import argparse
import base64
import json
import re
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
FONT = r"C:\Windows\Fonts\msyh.ttc"
S = ROOT / "data" / "samples"


def ask(img, prompt):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": 96}}).encode()
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
        cand = runs[-4:]
        if all(0 <= v <= 1000 for v in cand):
            return cand
    return None


def prompt_for(name):
    return ("Please provide the bounding box coordinate of the region "
            f"this sentence describes: <ref>{name}</ref>")


def draw(bgr, box, color, thick=3):
    h, w = bgr.shape[:2]
    x1, y1 = int(box[0] / 1000 * w), int(box[1] / 1000 * h)
    x2, y2 = int(box[2] / 1000 * w), int(box[3] / 1000 * h)
    cv2.rectangle(bgr, (x1, y1), (x2, y2), color, thick)
    return bgr, ((x1 + x2) // 2, (y1 + y2) // 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", action="store_true")
    a = ap.parse_args()

    f_tag = ImageFont.truetype(FONT, 17)
    f_hdr = ImageFont.truetype(FONT, 22)
    CELL = 300

    # ── A 红点定位 5 档 ────────────────────────────────
    bg = imread_unicode(S / "win05.jpg")
    bg = cv2.resize(bg, (512, 512), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)
    TX, TY = 330, 190

    tiles = []
    for d in (60, 30, 20, 8, 5):
        canvas = bg.copy()
        cv2.circle(canvas, (TX, TY), max(1, d // 2), (30, 30, 205), -1,
                   lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        canvas = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        ans = ask(canvas, prompt_for("红色圆点"))
        box = parse_box(ans)
        vis = canvas.copy()
        cv2.drawMarker(vis, (TX, TY), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)
        if box:
            vis, c = draw(vis, box, (80, 230, 80), 2)
            err = ((c[0] - TX) ** 2 + (c[1] - TY) ** 2) ** 0.5
            label = f"{d}px 误差 {err:.0f}px"
        else:
            label = f"{d}px 无输出"
        print(f"  红点 {d}px -> {ans[:36]}")
        tiles.append((vis, label))

    cells = []
    for vis, label in tiles:
        im = cv2.cvtColor(cv2.resize(vis, (CELL, CELL)), cv2.COLOR_BGR2RGB)
        c = Image.new("RGB", (CELL, CELL + 30), (26, 26, 26))
        c.paste(Image.fromarray(im), (0, 0))
        ImageDraw.Draw(c).text((8, CELL + 5), label, font=f_tag,
                               fill=(255, 220, 90))
        cells.append(c)
    rowA = Image.new("RGB", (CELL * len(cells), CELL + 30), (26, 26, 26))
    for i, c in enumerate(cells):
        rowA.paste(c, (i * CELL, 0))

    # ── B 窗户定位 ─────────────────────────────────────
    rowB_tiles = []
    for name, tag in ((("win04.jpg"), "立面 · 问<ref>窗户</ref>"),
                      (("win08.jpg"), "近距 · 问<ref>窗户</ref>"),
                      (("test2.jpg"), "室内 · 问<ref>窗户</ref>（应无框）")):
        img = imread_unicode(S / name)
        if img is None:
            continue
        img = cv2.resize(img, (512, 512), interpolation=cv2.INTER_AREA)
        ans = ask(img, prompt_for("窗户"))
        box = parse_box(ans)
        vis = img.copy()
        label = f"{name}  "
        if box:
            vis, _ = draw(vis, box, (80, 230, 80), 3)
            label += "模型给出 1 个框"
        else:
            label += "无框"
        print(f"  窗 {name} -> {ans[:40]}")
        im = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        c = Image.new("RGB", (CELL * 5 // 3, CELL + 30), (26, 26, 26))
        c = Image.new("RGB", (400, 400 + 30), (26, 26, 26))
        c.paste(Image.fromarray(cv2.resize(im, (400, 400))), (0, 0))
        ImageDraw.Draw(c).text((8, 405), label[:26], font=f_tag,
                               fill=(255, 220, 90))
        rowB_tiles.append(c)

    rowB = Image.new("RGB", (400 * len(rowB_tiles), 430), (26, 26, 26))
    for i, c in enumerate(rowB_tiles):
        rowB.paste(c, (i * 400, 0))

    W = max(rowA.width, rowB.width)
    H = 40 + rowA.height + 40 + rowB.height + 20
    sheet = Image.new("RGB", (W, H), (16, 16, 16))
    d = ImageDraw.Draw(sheet)
    d.text((10, 8), "纯大模型出框（不接 CV）：grounding 结果", font=f_hdr,
           fill=(0, 220, 255))
    sheet.paste(rowA, (0, 40))
    d.text((10, 40 + rowA.height + 8),
           "立面 / 近距 各给一个框；室内图问『窗户』——看它会不会硬编",
           font=f_hdr, fill=(0, 220, 255))
    sheet.paste(rowB, (0, 40 + rowA.height + 40))

    out = ROOT / "eval" / "figures" / "grounding_demo.jpg"
    out.parent.mkdir(exist_ok=True)
    sheet.save(out, quality=92)
    print(f"\n出图 {out}  ({sheet.width}x{sheet.height})")
    if a.open:
        try:
            import os
            os.startfile(str(out))
        except Exception:
            pass


if __name__ == "__main__":
    main()
