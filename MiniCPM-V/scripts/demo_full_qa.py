# -*- coding: utf-8 -*-
"""demo_full_qa.py —— 全量提问演示：一张图，把该问的都问出来

═══════════════════════════════════════════════════════════════════
为什么分两次问，而不是一次问 11 项
───────────────────────────────────────────────────────────────────
实测：一次问 11 项，模型在**第 3 项就开始照抄问题原文**
（"3) 大概几层楼；4) 大概多少扇窗户；"…），完全没法用。
降到 5~6 项/次就正常。所以拆成两段：

  第一段 场景结构（6 项）
      0) 概述  1) 楼层  2) 窗数  3) 开合  4) 遮挡  5) 清晰度
  第二段 建筑属性（5 项）
      1) 建筑类型  2) 外墙材质  3) 排列是否整齐  4) 附着物  5) 照度

═══════════════════════════════════════════════════════════════════
一个实测缺陷的处理
───────────────────────────────────────────────────────────────────
"清晰度"这一项偶发照抄（回答就是"画面清晰度。"）。
自动检测：若回答与问题文字高度重合 → 判为未作答，单条重问一次。
重问后仍失败的，标成"（未作答）"，不留空、不假装。

═══════════════════════════════════════════════════════════════════
用法：python scripts/demo_full_qa.py [--open]
"""
import argparse
import base64
import json
import re
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
from mcpv.prompts import (SCENE_STRUCT_PROMPT as P1,        # noqa: E402
                           SCENE_ATTR_PROMPT as P2)

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"
FONT = r"C:\Windows\Fonts\msyh.ttc"
S = ROOT / "data" / "samples"
W = Path(r"D:\Desktop\window")


LABEL1 = ["概述", "楼层", "窗数", "开合", "遮挡", "清晰度"]
LABEL2 = ["类型", "外墙", "排列", "附着物", "照度"]
QKEY1 = ["这是什么建筑", "大概几层楼", "多少扇窗户", "开着还是关着", "挡住视线", "画面清晰度"]
QKEY2 = ["建筑类型", "外墙材质", "排列是否整齐", "附着物", "照度"]

PICKS = [
    (S / "win01.jpg", "远距 · 多栋建筑"),
    (S / "win03.jpg", "远距 · 窗户密集"),
    (S / "win02.jpg", "中距 · 有树"),
    (S / "win06.jpg", "中距 · 树遮挡"),
    (S / "win04.jpg", "立面 · 规整"),
    (S / "win08.jpg", "近距 · 单扇窗"),
    (S / "neg3.jpg", "夜景 · 城市背景"),
    (W / "fdf4e302b3ba9d5262ca5175bc5211ed.jpg", "手机实拍"),
]

CELL_W, IMG_H = 640, 390


def ask(img_bgr, prompt):
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        d = json.loads(r.read())
    return d.get("response", "").strip(), time.time() - t0


def parse(ans, n, qkey):
    """把回答拆成 n 条；检测'照抄问题'的情况。"""
    lines = [x.strip() for x in ans.splitlines() if x.strip()]
    out = []
    for i in range(n):
        v = ""
        for ln in lines:
            m = re.match(r"^[0-9０-９]+[)）.、]\s*(.*)$", ln)
            body = m.group(1).strip() if m else ""
            if body and (not m or ln.startswith(str(i))):
                pass
        # 直接按序取
        body_lines = []
        for ln in lines:
            m = re.match(r"^[0-9]+[)）.、]\s*(.*)$", ln)
            body_lines.append(m.group(1).strip() if m else ln)
        v = body_lines[i] if i < len(body_lines) else ""
        v = v.rstrip(";；")
        # 照抄检测：回答里含问题关键词且很短
        if qkey[i] in v and len(v) <= len(qkey[i]) + 8:
            v = ""
        out.append(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", action="store_true")
    a = ap.parse_args()

    f_h = ImageFont.truetype(FONT, 21)
    f_b = ImageFont.truetype(FONT, 17)
    f_t = ImageFont.truetype(FONT, 15)

    tiles = []
    for p, tag in PICKS:
        img = imread_unicode(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        sent = cv2.resize(img, (1600, int(h * 1600 / w)),
                          interpolation=cv2.INTER_AREA) if w > 1600 else img
        a1, d1 = ask(sent, P1)
        a2, d2 = ask(sent, P2)
        r1 = parse(a1, 6, QKEY1)
        r2 = parse(a2, 5, QKEY2)

        # 照抄的项单条重问一次
        retry = 0
        for i, v in enumerate(r1):
            if v == "":
                q = (f"只看这张图回答问题，只回答内容本身，不要复述问题：{QKEY1[i]}？")
                one, _ = ask(sent, q)
                retry += 1
                r1[i] = (one.splitlines()[0] if one else "").strip()
        for i, v in enumerate(r2):
            if v == "":
                q = (f"只看这张图回答问题，只回答内容本身，不要复述问题：{QKEY2[i]}？")
                one, _ = ask(sent, q)
                retry += 1
                r2[i] = (one.splitlines()[0] if one else "").strip()

        print(f"[{tag}] {p.name}  {d1+d2:.1f}s  重问 {retry} 次")
        for k, v in zip(LABEL1, r1):
            print(f"    {k}: {v}")
        for k, v in zip(LABEL2, r2):
            print(f"    {k}: {v}")
        if retry:
            for i, v in enumerate(r1):
                if not v:
                    r1[i] = "（未作答）"
            for i, v in enumerate(r2):
                if not v:
                    r2[i] = "（未作答）"

        # 渲染
        c = Image.new("RGB", (CELL_W, IMG_H + 350), (26, 26, 26))
        d = ImageDraw.Draw(c)
        th = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        th = th.resize((CELL_W, int(CELL_W * h / w)))
        top = max(0, (th.height - IMG_H) // 2)
        c.paste(th.crop((0, top, CELL_W, top + IMG_H)), (0, 0))
        d.rectangle([0, 0, CELL_W, 28], fill=(40, 40, 40))
        d.text((8, 5), tag, font=f_t, fill=(255, 220, 90))

        y = IMG_H + 8
        d.text((8, y), "场景", font=f_h, fill=(0, 200, 255))
        y += 26
        for k, v in zip(LABEL1, r1):
            txt = f"{k}：{v}"
            for ln in textwrap.wrap(txt, 34)[:2]:
                d.text((10, y), ln, font=f_b, fill=(228, 228, 228))
                y += 22
        y += 4
        d.text((8, y), "建筑", font=f_h, fill=(0, 200, 255))
        y += 26
        for k, v in zip(LABEL2, r2):
            txt = f"{k}：{v}"
            for ln in textwrap.wrap(txt, 34)[:2]:
                d.text((10, y), ln, font=f_b, fill=(228, 228, 228))
                y += 22
        d.text((CELL_W - 70, IMG_H + 10), f"{d1+d2:.0f}s", font=f_t,
               fill=(150, 150, 150))
        tiles.append(c)

    cols = 2
    rows = (len(tiles) + cols - 1) // cols
    ch = max(t.height for t in tiles)
    PAD = 12
    sheet = Image.new("RGB", (cols * CELL_W + PAD * (cols + 1),
                              rows * ch + PAD * (rows + 1) + 92), (16, 16, 16))
    dd = ImageDraw.Draw(sheet)
    dd.text((PAD, 12), "MiniCPM-V 4.7 1B · 一张图问全 11 项（分两段问）",
            font=ImageFont.truetype(FONT, 24), fill=(0, 220, 255))
    dd.text((PAD, 42), "场景 6 项 + 建筑属性 5 项 · 每项都能拿图逐条核对",
            font=ImageFont.truetype(FONT, 17), fill=(150, 200, 220))
    dd.text((PAD, sheet.height - 26),
            "★ 楼层/开合/遮挡/清晰度/类型/外墙/排列/附着物/照度 稳定；"
            "**窗数不稳定，只能当量级参考**（win03 标注 109，模型答「三百多」）",
            font=ImageFont.truetype(FONT, 16), fill=(255, 170, 90))
    for i, t in enumerate(tiles):
        r, c_ = divmod(i, cols)
        sheet.paste(t, (PAD + c_ * (CELL_W + PAD), 58 + PAD + r * (ch + PAD)))

    out = ROOT / "eval" / "demo_out" / "full_qa_demo.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out, quality=92)
    print(f"\n演示图 {out}  ({sheet.width}x{sheet.height})")
    if a.open:
        try:
            import os
            os.startfile(str(out))
        except Exception:
            pass


if __name__ == "__main__":
    main()
