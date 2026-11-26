# -*- coding: utf-8 -*-
"""bench_scene_desc.py —— 整图级"场景描述"能力测试（文档原文要的粒度）

═══════════════════════════════════════════════════════════════════
为什么单独做这一项
───────────────────────────────────────────────────────────────────
文档 §3.1 写的是「理解建筑立面**场景**、产生人工可核对的**场景描述**」。
"场景"指的是**整张图**，不是我们一直在测的"这一小块是不是窗户"。

我们此前只在测时延时跑过整图（20 秒/张），**从没评过它的描述质量**。
所以这一项是空白，必须补。

═══════════════════════════════════════════════════════════════════
问什么
───────────────────────────────────────────────────────────────────
一句话里塞四个具体问题，逼它给出可核对的信息：
    有几层 / 大约多少扇窗 / 开着还是关着 / 有没有遮挡或反光

人工核对时按这四项逐条打分（对/错/部分对），比"感觉描述得不错"有用。

用法：python eval/bench_scene_desc.py [--n 6] [--w 1600]
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

MODEL = "openbmb/minicpm-v4.7:1b"
OLLAMA = "http://127.0.0.1:11434/api/generate"

PROMPT = ("这是一张建筑外立面的照片。请用两三句话回答四件事："
          "1) 大概有几层楼；2) 大概有多少扇窗户；"
          "3) 窗户是开着还是关着；4) 有没有被树或其它东西遮挡。"
          "不要解释，直接描述你看到的。")


def ask(img_bgr):
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    b64 = base64.b64encode(buf.tobytes()).decode()
    body = json.dumps({"model": MODEL, "prompt": PROMPT, "images": [b64],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        d = json.loads(r.read())
    return (d.get("response", "").strip(), time.time() - t0,
            d.get("prompt_eval_count", 0), d.get("eval_count", 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--w", type=int, default=1600, help="送进模型前缩到的宽度")
    ap.add_argument("--dir", default=str(ROOT / "data" / "samples"))
    ap.add_argument("--pattern", default="*.jpg")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()

    files = sorted(Path(a.dir).glob(a.pattern))[:a.n]
    if not files:
        raise SystemExit("没找到立面图")

    print(f"整图场景描述测试：{len(files)} 张，送模型前缩到 {a.w} 宽")
    print("=" * 74)
    out = []
    for p in files:
        img = imread_unicode(p)
        if img is None:
            continue
        h, w = img.shape[:2]
        if w > a.w:
            img = cv2.resize(img, (a.w, int(h * a.w / w)),
                             interpolation=cv2.INTER_AREA)
        ans, dt, pt, gt = ask(img)
        print(f"\n【{p.name}】 原图 {w}x{h}  送模型 {img.shape[1]}x{img.shape[0]}")
        print(f"  耗时 {dt:.1f}s   视觉token {pt}   生成 {gt} tok")
        print(f"  描述：{ans}")
        out.append({"image": p.name, "orig": [w, h],
                    "sent": [img.shape[1], img.shape[0]],
                    "latency_s": round(dt, 1), "prompt_tokens": pt,
                    "gen_tokens": gt, "answer": ans})

    tag = ("_" + a.tag) if a.tag else ""
    f = ROOT / "eval" / "results" / f"scene_desc_{datetime.now():%Y%m%d_%H%M%S}{tag}.json"
    f.write_text(json.dumps({"model": MODEL, "prompt": PROMPT, "records": out},
                            ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n明细 {f}")


if __name__ == "__main__":
    main()
