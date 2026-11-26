# -*- coding: utf-8 -*-
"""bench_scene_prompt.py —— 整图描述的三种问法对比

═══════════════════════════════════════════════════════════════════
要回答的两个问题（用户提的）
───────────────────────────────────────────────────────────────────
1. 要不要**先问整体描述**（"这张图大致有什么"），再问细节？
2. 现在这四个问题（楼层/窗数/开合/遮挡）是**最好的**吗？

═══════════════════════════════════════════════════════════════════
三种问法
───────────────────────────────────────────────────────────────────
  A 现状   一句话里问四件事（楼层/窗数/开合/遮挡）
  B 先概述 + 再四问    两次调用：第一次自由描述，第二次问结构化四项
  C 一次问全    一次调用：先一句话概述整体，再逐条答四项
                （省一次调用，且模型能看到上下文）

  额外试 D：在 C 基础上加第五项「画面是否清晰可辨」
        ——因为"反光/模糊"直接影响可用性，而我们此前那个 glare 字段实测恒答"有"。

═══════════════════════════════════════════════════════════════════
用法：python eval/bench_scene_prompt.py [--n 5]
"""
import argparse
import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"

P_A = ("这是一张建筑外立面的照片。请用两三句话回答四件事："
       "1) 大概有几层楼；2) 大概有多少扇窗户；"
       "3) 窗户是开着还是关着；4) 有没有被树或其它东西遮挡。"
       "不要解释，直接描述你看到的。")

P_B1 = "这张图片里整体都有什么？用两三句话描述你看到的场景。"
P_B2 = P_A

P_C = ("这是一张建筑外立面的照片。先**用一句话**概述画面整体有什么，"
       "然后逐条回答四项：1) 大概几层楼；2) 大概多少扇窗户；"
       "3) 窗户开着还是关着；4) 有没有遮挡。"
       "不要解释，直接说。")

P_D = ("这是一张建筑外立面的照片。先**用一句话**概述画面整体有什么，"
       "然后逐条回答五项：1) 大概几层楼；2) 大概多少扇窗户；"
       "3) 窗户开着还是关着；4) 有没有遮挡；"
       "5) 画面清晰度（清晰 / 略模糊 / 严重模糊）。"
       "不要解释，直接说。")


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--w", type=int, default=1600)
    a = ap.parse_args()

    files = sorted((ROOT / "data" / "samples").glob("win0*.jpg"))[:a.n]

    for p in files:
        img = imread_unicode(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        if w > a.w:
            img = cv2.resize(img, (a.w, int(h * a.w / w)),
                             interpolation=cv2.INTER_AREA)
        print("\n" + "=" * 74)
        print(f"【{p.name}】")
        print("=" * 74)

        outs = {}
        ans, dt = ask(img, P_A)
        outs["A 一句话四问"] = (ans, dt)
        ans1, d1 = ask(img, P_B1)
        ans2, d2 = ask(img, P_B2)
        outs["B 先概述+再四问"] = (f"[概述] {ans1}\n[四项] {ans2}", d1 + d2)
        ans, dt = ask(img, P_C)
        outs["C 一次问全"] = (ans, dt)
        ans, dt = ask(img, P_D)
        outs["D 一次问全+清晰度"] = (ans, dt)

        for k, (t, dt) in outs.items():
            print(f"\n--- {k}  ({dt:.1f}s) ---")
            for line in t.splitlines():
                if line.strip():
                    print("   " + line)


if __name__ == "__main__":
    main()
