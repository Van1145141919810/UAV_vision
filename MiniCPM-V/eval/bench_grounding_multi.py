# -*- coding: utf-8 -*-
"""bench_grounding_multi.py —— VLM 能不能一次给出"所有窗户"的框？

═══════════════════════════════════════════════════════════════════
为什么测这个
───────────────────────────────────────────────────────────────────
单目标 grounding 已经验证可用（红点 8 px 误差 13 px）。
但真实任务需要的是"一栋楼所有窗在哪"。

关键分歧点：
  · 如果 VLM 能一次返回多个框 → 也许真的可以不依赖分割
  · 如果它只能给一个/给整幅   → 那"普查"必须靠 CV/分割

═══════════════════════════════════════════════════════════════════
三种问法
───────────────────────────────────────────────────────────────────
  A 单目标 grounding（已知只能给一个）
  B 多目标 JSON（官方 Cookbook 里给人物用的那种格式）
  C 直接问"有几扇窗，每扇的框"

用法：python eval/bench_grounding_multi.py
"""
import base64
import json
import re
import sys
import time
import urllib.request
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"

P_SINGLE = ("Please provide the bounding box coordinate of the region "
            "this sentence describes: <ref>窗户</ref>")

P_MULTI_JSON = ('Identify all windows in the image and return their bounding '
                'boxes in JSON format, a list of '
                '{"name_zh": "窗户", "box": "x1 y1 x2 y2"}. '
                'Coordinates are normalized to 0-1000.')

P_COUNT = ("这张图里有几扇窗户？请数一下，然后每一扇给出它的框，"
           "格式：窗户N: x1 y1 x2 y2（坐标归一化到 0-1000）。")


def ask(img, prompt, npred=512):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npred}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=600) as r:
        d = json.loads(r.read())
    return d.get("response", "").strip(), time.time() - t0


def boxes_in(txt):
    """尽力抽出所有 4 元组。"""
    out = []
    for m in re.finditer(r"<box>\s*([\d\s,]+?)\s*</box>", txt):
        n = [int(x) for x in re.findall(r"\d+", m.group(1))]
        if len(n) >= 4:
            out.append(n[:4])
    # "x1 y1 x2 y2" 形式
    for m in re.finditer(r"(\d{1,4})\s+(\d{1,4})\s+(\d{1,4})\s+(\d{1,4})", txt):
        v = [int(g) for g in m.groups()]
        if all(0 <= x <= 1000 for x in v):
            out.append(v)
    # 去重
    seen, uniq = set(), []
    for b in out:
        k = tuple(b)
        if k not in seen:
            seen.add(k)
            uniq.append(b)
    return uniq


def main():
    for name in ("win04.jpg", "win08.jpg"):
        img = imread_unicode(ROOT / "data" / "samples" / name)
        if img is None:
            continue
        img = cv2.resize(img, (768, 768), interpolation=cv2.INTER_AREA)
        print("=" * 72)
        print(f"【{name}】  送模型 {img.shape[1]}x{img.shape[0]}")
        print("=" * 72)

        for tag, prompt in (("A 单目标", P_SINGLE),
                            ("B 多目标JSON", P_MULTI_JSON),
                            ("C 数数+逐个", P_COUNT)):
            ans, dt = ask(img, prompt)
            bs = boxes_in(ans)
            print(f"\n--- {tag}  ({dt:.1f}s)  抽到 {len(bs)} 个框 ---")
            for line in ans.splitlines()[:8]:
                if line.strip():
                    print("   " + line[:100])
            if len(bs) > 2:
                print(f"   …共 {len(bs)} 个框")


if __name__ == "__main__":
    main()
