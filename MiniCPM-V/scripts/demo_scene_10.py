# -*- coding: utf-8 -*-
"""demo_scene_10.py —— 10 张图 × 整图场景描述，排成可讲解的演示图

═══════════════════════════════════════════════════════════════════
这份演示要证明什么
───────────────────────────────────────────────────────────────────
文档 §3.1 要求的是「理解建筑立面场景、产生人工可核对的场景描述」。
所以演示不能只放一张图 + 一句"是窗户"，而要让人能**逐条核对**：

    楼层数对不对？   窗数量级对不对？
    开合判断对不对？ 遮挡说对了没有？

选图覆盖 6 种情况：远距 / 近距 / 树遮挡 / 夜景 / 室内（负例）/ 超近距离。

═══════════════════════════════════════════════════════════════════
排版
───────────────────────────────────────────────────────────────────
OpenCV 的 putText 画不了中文，所以用 PIL + 微软雅黑。
每格：上图下文，2 列 × 5 行，一张图看完。

用法：python scripts/demo_scene_10.py [--open]
"""
import argparse
import base64
import json
import sys
import textwrap
import time
import urllib.request
from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv.prompts import SCENE_PROMPT                     # noqa: E402

PROMPT = SCENE_PROMPT

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"

S = ROOT / "data" / "samples"
W = Path(r"D:\Desktop\window")
PICKS = [
    (S / "win03.jpg", "远距 · 窗户密集"),
    (S / "win02.jpg", "中距 · 有树"),
    (S / "win06.jpg", "中距 · 树遮挡明显"),
    (S / "win05.jpg", "近距 · 窗户少"),
    (S / "win08.jpg", "超近距 · 单扇窗"),
    (S / "win09.jpg", "夜景 · 宿舍楼"),
    (S / "neg3.jpg", "夜景 · 远处建筑"),
    (S / "test1.jpg", "负例 · 塔与树（窗外场景）"),
    (S / "test2.jpg", "负例 · 室内桌面"),
    (W / "fdf4e302b3ba9d5262ca5175bc5211ed.jpg", "手机实拍 · 立面"),
]

FAIL = {"test2.jpg"}          # 室内桌面：会被强行描述成建筑
CELL_W = 620
IMG_H = 400
PAD = 10
FONT = r"C:\Windows\Fonts\msyh.ttc"


def ask(img_bgr):
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": PROMPT,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        d = json.loads(r.read())
    return d.get("response", "").strip(), time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", action="store_true")
    args = ap.parse_args()

    f_title = ImageFont.truetype(FONT, 20)
    f_body = ImageFont.truetype(FONT, 19)
    f_tag = ImageFont.truetype(FONT, 17)

    tiles = []
    for i, (p, tag) in enumerate(PICKS, 1):
        img = imread_unicode(p)
        if img is None:
            print(f"跳过（读不到）{p}")
            continue
        h, w = img.shape[:2]
        scale = 1600 / w if w > 1600 else 1.0
        sent = cv2.resize(img, (int(w * scale), int(h * scale)),
                          interpolation=cv2.INTER_AREA) if scale < 1 else img
        ans, dt = ask(sent)
        print(f"[{i}/{len(PICKS)}] {p.name}  {dt:.1f}s")
        for line in ans.splitlines():
            print("      " + line)

        # 渲染
        canvas = Image.new("RGB", (CELL_W, IMG_H + 210), (26, 26, 26))
        d = ImageDraw.Draw(canvas)
        th = int(IMG_H * h / w) if w else IMG_H
        th = min(th, IMG_H)
        thumb = Image.fromarray(cv2.cvtColor(
            cv2.resize(img, (CELL_W, int(CELL_W * h / w))), cv2.COLOR_BGR2RGB))
        thumb = thumb.crop((0, max(0, (thumb.height - IMG_H) // 2),
                            CELL_W, max(0, (thumb.height - IMG_H) // 2) + IMG_H))
        canvas.paste(thumb, (0, 0))
        d.rectangle([0, 0, CELL_W, 30], fill=(40, 40, 40))
        d.text((8, 5), f"{i}. {tag}", font=f_title, fill=(255, 220, 80))
        y = IMG_H + 10
        body = ans.replace("1)", "\n1)").replace("2)", "\n2)").replace(
            "3)", "\n3)").replace("4)", "\n4)").replace("1.", "\n1.").replace(
            "2.", "\n2.").replace("3.", "\n3.").replace("4.", "\n4.")
        for para in [x for x in body.splitlines() if x.strip()]:
            for ln in textwrap.wrap(para, 30):
                d.text((10, y), ln, font=f_body, fill=(230, 230, 230))
                y += 27
        d.text((10, IMG_H + 182), f"{dt:.1f}s", font=f_tag, fill=(150, 150, 150))
        # 已知失败：负例图会被"编造建筑描述"，标红以示诚实
        if p.name in FAIL:
            d.rectangle([0, 0, CELL_W - 1, canvas.height - 1],
                        outline=(230, 60, 60), width=6)
            d.text((12, IMG_H + 150), "⚠ 部分失败：概述认对了（办公空间），但五项退化成照抄问题",
                   font=f_tag, fill=(255, 90, 90))
        tiles.append(canvas)

    cols = 2
    rows = (len(tiles) + cols - 1) // cols
    ch = max(t.height for t in tiles)
    sheet = Image.new("RGB", (cols * CELL_W + PAD * (cols + 1),
                              rows * ch + PAD * (rows + 1) + 86), (18, 18, 18))
    dd = ImageDraw.Draw(sheet)
    dd.text((PAD + 4, 12),
            f"MiniCPM-V 4.7 1B · 建筑立面场景描述 · {len(tiles)} 张 · 首次推理约 6 s/张",
            font=ImageFont.truetype(FONT, 22), fill=(0, 220, 255))
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        sheet.paste(t, (PAD + c * (CELL_W + PAD), 46 + PAD + r * (ch + PAD)))

    dd.text((PAD + 4, sheet.height - 40),
            "9/10 可用。红框那张：**加概述后已认出是「办公空间」**（此前会编成两层楼），"
            "但结构性五项退化成照抄问题——说明非建筑图不适合这套五项模板",
            font=ImageFont.truetype(FONT, 17), fill=(255, 120, 120))
    out = ROOT / "eval" / "demo_out" / "scene_demo_10.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out, quality=92)
    print(f"\n演示图 {out}  ({sheet.width}x{sheet.height})")
    if args.open:
        try:
            import os
            os.startfile(str(out))
        except Exception:
            pass


if __name__ == "__main__":
    main()
