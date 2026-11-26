# -*- coding: utf-8 -*-
"""bench_vlm_locator.py —— VLM 到底能不能给坐标？

═══════════════════════════════════════════════════════════════════
为什么"能看见"不等于"能定位"
───────────────────────────────────────────────────────────────────
VLM 的输出是**文字 token**，不是坐标回归。它没有专门的位置输出头。
你问它坐标，它只能"生成"一串像坐标的数字——那是从语言分布里编出来的，
不是从图像里量出来的。

另一个原因：图像被切成视觉 token 后，每个 token 对应一大块区域
（1600x1200 → 737 个 token），所以它的空间分辨率天然是"块级"。

═══════════════════════════════════════════════════════════════════
本脚本测三种问法
───────────────────────────────────────────────────────────────────
  A. 直接要像素坐标    "红点中心坐标是多少？"
  B. 要百分比          "红点在画面的百分之几位置？"
  C. 九宫格            "红点在九宫格的哪一格？"

结论预期（待验证）：
  A 会给出数字，但误差极大；B 稍好；C 可能可用 ——
  那么正确的工程做法就是：**VLM 给粗位置（哪一格），CV 在那一格里精确找**。

用法：python eval/bench_vlm_locator.py
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
SIZE = 384
POSITIONS = [("正中心", 192, 192), ("右上", 300, 90), ("左下", 90, 300)]
GRID = [["左上", "上", "右上"], ["左", "中", "右"], ["左下", "下", "右下"]]


def ask(crop, prompt):
    ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request("http://127.0.0.1:11434/api/generate",
                                 data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read()).get("response", "").strip()


def grid_cell(x, y):
    ci = 0 if x < SIZE / 3 else (1 if x < 2 * SIZE / 3 else 2)
    ri = 0 if y < SIZE / 3 else (1 if y < 2 * SIZE / 3 else 2)
    return GRID[ri][ci]


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (1920, 1080), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.75).clip(0, 255).astype(np.uint8)

    print(f"画布 {SIZE}x{SIZE}（384 见方，红点直径约 20 px）")
    print(f"{'位置':<8}{'真实坐标':<14}{'A 像素坐标':<20}{'误差px':<9}"
          f"{'C 九宫格':<12}{'对?'}")
    print("-" * 78)

    for name, tx, ty in POSITIONS:
        crop = cv2.resize(bg, (SIZE, SIZE), interpolation=cv2.INTER_AREA)
        cv2.circle(crop, (tx, ty), 10, (30, 30, 205), -1, lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        crop = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        a = ask(crop, "画面里有一个红色圆点。请给出它中心的像素坐标，"
                      "格式：(x, y)。只输出坐标，不要解释。")
        m = re.findall(r"(\d+)\s*[,，]\s*(\d+)", a)
        if m:
            px, py = int(m[0][0]), int(m[0][1])
            err = ((px - tx) ** 2 + (py - ty) ** 2) ** 0.5
            coord_s = f"({px},{py})"
            err_s = f"{err:.0f}"
        else:
            coord_s, err_s = repr(a[:16]), "?"

        c = ask(crop, "画面被分成九宫格（上/中/下 × 左/中/右）。"
                      "红色圆点在哪个格子里？只回答格子名称，例如：左上。")
        cell_true = grid_cell(tx, ty)
        hit = "✓" if cell_true in c else "✗"
        print(f"{name:<8}({tx},{ty})       {coord_s:<20}{err_s:<9}"
              f"{c[:6]:<10}(真:{cell_true}){hit}")
        print(f"         └ 模型原话：{a[:46]}")

    print("\n说明：")
    print("  · 真实坐标是画的时候就知道的，模型的数字纯粹是它自己'说'出来的")
    print("  · 九宫格每格 128x128 px，比'要精确坐标'宽松得多")


if __name__ == "__main__":
    main()
