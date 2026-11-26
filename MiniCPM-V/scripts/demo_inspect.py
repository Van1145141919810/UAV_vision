# -*- coding: utf-8 -*-
"""demo_inspect.py —— 单图/多图"当场问模型"，把答案烧进图里。

═══════════════════════════════════════════════════════════════════
为什么要加"整体描述"
───────────────────────────────────────────────────────────────────
只问"是不是窗户 / 开还是关"，领导看到的是三个字母，感受不到"模型看懂了"。
加一句**自由描述**在最上面，屏幕上就先出现"窗台上有盆栽""红色圆形图案"
这类人话，直观证明它真的看见了东西。

═══════════════════════════════════════════════════════════════════
四个问题（按顺序）
───────────────────────────────────────────────────────────────────
  1. 整体描述   —— 用不超过十个字描述画面正中心的主要物体
  2. 是不是窗户 —— 闭集 A/B
  3. 开还是关   —— 闭集 A/B/C
  4. 有没有反光 —— 闭集 A/B

用法：
    python scripts/demo_inspect.py 图片1 [图片2 图片3 ...] [--open]
多张图会横向拼成一张对比图（最多 3 张）。
"""
import argparse
import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode      # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"

# 提示词统一从 src/mcpv/prompts.py 取
from mcpv.prompts import QUESTIONS, MEANING, PROMPT_VERSION   # noqa: E402

# 演示顺序：先描述，再三个判定
QS = [(QUESTIONS[k][0], QUESTIONS[k][1], QUESTIONS[k][2])
      for k in ("describe", "is_window", "window_state", "glare")]

KIND = {"AB": {"A": "是", "B": "否"}, }
def ask(p: Path, prompt: str):
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(p.read_bytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=180) as r:
        d = json.loads(r.read())
    return d.get("response", "").strip(), time.time() - t0


def square(img, size):
    h, w = img.shape[:2]
    side = min(h, w)
    y0, x0 = (h - side) // 2, (w - side) // 2
    return cv2.resize(img[y0:y0 + side, x0:x0 + side], (size, size),
                      interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--out", default=str(ROOT / "eval" / "demo_out"))
    ap.add_argument("--size", type=int, default=0,
                    help="0=按张数自动（1-2张560，3张520，4-5张430）")
    ap.add_argument("--open", action="store_true")
    args = ap.parse_args()

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    paths = [Path(p).resolve() for p in args.images][:5]
    if args.size:
        size = args.size
    else:
        size = {1: 560, 2: 560, 3: 520}.get(len(paths), 430)
    n = len(paths)

    LINE = 34
    panel = np.full((size + LINE * (len(QS) + 2), size, 3), 22, np.uint8)
    panels = []
    totals = []

    for idx, src in enumerate(paths):
        img = square(imread_unicode(src), size)
        cv2.putText(img, src.name[:34], (6, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        canvas = np.full((size + LINE * (len(QS) + 2), size, 3), 22, np.uint8)
        canvas[0:size] = img

        tmp = outdir / "_tmp.jpg"
        imwrite_unicode(tmp, img, quality=95)
        total = 0.0
        for i, (key, (label, prompt, allowed)) in enumerate(
                zip(("describe", "is_window", "window_state", "glare"), QS)):
            ans, dt = ask(tmp, prompt)
            total += dt
            y = size + LINE * (i + 1)
            if allowed is None:
                cv2.putText(canvas, f"描述: {ans[:26]}", (8, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 235, 255), 2)
            else:
                L = next((c for c in ans.upper() if c in allowed), "?")
                word = MEANING.get(key, {}).get(L, L)
                cv2.putText(canvas, f"{label}: {word}", (8, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                            (80, 230, 80) if L != "?" else (80, 80, 240), 2)
        cv2.putText(canvas, f"{total:.2f}s", (8, canvas.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        panels.append(canvas)
        totals.append(total)
        tmp.unlink(missing_ok=True)

    sheet = np.hstack(panels)
    bar = np.full((30, sheet.shape[1], 3), 40, np.uint8)
    cv2.putText(bar, f"model {MODEL}   {n} 张   "
                     f"总耗时 {sum(totals):.1f}s", (8, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    sheet = np.vstack([bar, sheet])

    name = paths[0].stem if n == 1 else "compare3"
    out = outdir / f"{name}_inspect.jpg"
    imwrite_unicode(out, sheet, quality=94)

    print(f"每张耗时: {', '.join(f'{t:.2f}s' for t in totals)}")
    print(f"说明图 {out}")
    if args.open:
        try:
            import os
            os.startfile(str(out))
        except Exception as e:
            print(f"（自动打开失败：{e}）")


if __name__ == "__main__":
    main()
