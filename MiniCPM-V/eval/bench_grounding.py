# -*- coding: utf-8 -*-
"""bench_grounding.py —— 用官方 grounding 格式测 MiniCPM-V 4.7 1B 能不能出框

═══════════════════════════════════════════════════════════════════
为什么重测（重要）
───────────────────────────────────────────────────────────────────
官方 Cookbook 写明：MiniCPM-V 4.5/4.6/4.7 支持**视觉定位（grounding）**，
但必须用它的专用格式：

    Please provide the bounding box coordinate of the region
    this sentence describes: <ref>目标名称</ref>

输出形如：<box>x1 y1 x2 y2</box>
坐标是 **0~1000 归一化**（官方示例里除以 1000 再乘宽高换算成像素）。

此前我们用大白话"请给出像素坐标 (x, y)"去问，等于没走 grounding 通道，
所以得到的 389~540 px 误差**不能代表它的定位能力**——这次重测。

═══════════════════════════════════════════════════════════════════
测什么
───────────────────────────────────────────────────────────────────
  1. 大目标（60 px 红圆）—— 先验证格式能不能通
  2. 中目标（20 px）
  3. 小目标（8 px，即 26 cm @ 50 m）
  4. 极小（4.7 px，15 cm @ 50 m）

用法：python eval/bench_grounding.py
"""
import base64
import json
import re
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
SIZE = 512                       # 送模型的画布边长
TX, TY = 330, 190                # 红点放在非中心位置，便于检验是否真定位


def ask(img, prompt):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": 128}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read()).get("response", "").strip()


def parse_box(txt):
    """兼容两种输出：标准 <box>..</box>，以及 Ollama 丢标签后的裸数字。"""
    m = re.search(r"<box>\s*([\d\s,]+?)\s*</box>", txt)
    if m:
        nums = [int(x) for x in re.findall(r"\d+", m.group(1))]
        return nums[:4] if len(nums) >= 4 else None
    # 裸数字：取最后一段连续的 4 个整数
    runs = re.findall(r"\d+", txt)
    if len(runs) >= 4:
        nums = [int(x) for x in runs[-4:]]
        if all(0 <= v <= 1000 for v in nums):
            return nums
    return None


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (SIZE, SIZE), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)

    P = ("Please provide the bounding box coordinate of the region "
         "this sentence describes: <ref>红色圆点</ref>")

    print(f"画布 {SIZE}x{SIZE}   红点真实位置 ({TX},{TY})")
    print(f"{'直径px':<9}{'模型输出':<42}{'换算坐标':<16}{'误差px'}")
    print("-" * 82)

    for d in (60, 30, 20, 8, 5):
        canvas = bg.copy()
        cv2.circle(canvas, (TX, TY), max(1, d // 2), (30, 30, 205), -1,
                   lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        canvas = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        ans = ask(canvas, P)
        box = parse_box(ans)
        if box:
            # 0~1000 归一化 → 像素
            x1 = box[0] / 1000 * SIZE
            y1 = box[1] / 1000 * SIZE
            x2 = box[2] / 1000 * SIZE
            y2 = box[3] / 1000 * SIZE
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            err = ((cx - TX) ** 2 + (cy - TY) ** 2) ** 0.5
            print(f"{d:<9}{ans[:40]:<42}"
                  f"({cx:.0f},{cy:.0f})      {err:.0f}")
        else:
            print(f"{d:<9}{ans[:40]:<42}{'无 <box>':<16}-")

    print("\n说明：坐标按官方口径 0~1000 归一化换算成像素")
    print("      若某档没有 <box> 输出，说明模型认为找不到该目标")


if __name__ == "__main__":
    main()
