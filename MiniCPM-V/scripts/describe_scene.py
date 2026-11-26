# -*- coding: utf-8 -*-
"""describe_scene.py —— 整图场景描述（文档 §3.1 要求的粒度）

═══════════════════════════════════════════════════════════════════
和 demo_inspect.py 的区别
───────────────────────────────────────────────────────────────────
  demo_inspect.py   把图切成 384×384 的小块，问"这一块是不是窗户"
                    → 粒度小，给机器（Jev）用
  本脚本            直接看**整张图**，问"这栋楼什么样"
                    → 粒度和文档一致，给人看

═══════════════════════════════════════════════════════════════════
输出四项（都可人工核对）
───────────────────────────────────────────────────────────────────
  1) 大概几层楼
  2) 大概多少扇窗
  3) 开着还是关着
  4) 有没有遮挡

用法：
  python scripts/describe_scene.py 图片.jpg [--w 1600]
  python scripts/describe_scene.py 图片.jpg --open     # 顺带打开图
"""
import argparse
import base64
import json
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv.prompts import SCENE_PROMPT, SCENE_PROMPT_VERSION  # noqa: E402

PROMPT = SCENE_PROMPT

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"
LOG = ROOT / "eval" / "results" / "scene_log.jsonl"



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--w", type=int, default=1600)
    ap.add_argument("--open", action="store_true")
    a = ap.parse_args()

    src = Path(a.image)
    img = imread_unicode(src)
    if img is None:
        raise SystemExit(f"读不到图片：{src}")
    h, w = img.shape[:2]
    sent = img
    if w > a.w:
        sent = cv2.resize(img, (a.w, int(h * a.w / w)), interpolation=cv2.INTER_AREA)

    ok, buf = cv2.imencode(".jpg", sent, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": PROMPT,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    print(f"图片 {src.name}  原图 {w}x{h}  送模型 {sent.shape[1]}x{sent.shape[0]}")
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        d = json.loads(r.read())
    dt = time.time() - t0
    ans = d.get("response", "").strip()

    print("=" * 62)
    print("  场景描述")
    print("=" * 62)
    for line in ans.splitlines():
        print("  " + line)
    print("-" * 62)
    print(f"  耗时 {dt:.1f} s   视觉token {d.get('prompt_eval_count', 0)}   "
          f"生成 {d.get('eval_count', 0)} tok")

    out = ROOT / "eval" / "demo_out" / f"{src.stem}_scene.txt"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(f"图片: {src}\n原图: {w}x{h}\n耗时: {dt:.1f}s\n\n{ans}\n",
                   encoding="utf-8")
    print(f"  已存 {out}")

    LOG.parent.mkdir(parents=True, exist_ok=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(json.dumps({"ts": datetime.now().astimezone().isoformat(timespec="seconds"),
                            "image": src.name, "orig": [w, h],
                            "sent": [sent.shape[1], sent.shape[0]],
                            "latency_s": round(dt, 1),
                            "prompt_version": SCENE_PROMPT_VERSION,
                            "answer": ans}, ensure_ascii=False) + "\n")

    if a.open:
        try:
            import os
            os.startfile(str(src))
        except Exception:
            pass


if __name__ == "__main__":
    main()
