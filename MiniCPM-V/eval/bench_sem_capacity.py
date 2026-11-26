# -*- coding: utf-8 -*-
"""bench_sem_capacity.py —— 测"VLM 在多大输入下、多大的点还能可靠找到"

为什么必须测这个
──────────────────────────────────────────────────────────────
前面两轮暴露的是同一件事：**整帧 1920 喂进去，模型实际只拿到 ~531 个图像 token**
（实测：512→137、768→134、1024→134、1920→531），相当于整幅被压成粗网格。
一个 7 px 的红点在那种网格里不到一个格子的 1%，必然"看不见"。

所以问题不是"模型行不行"，而是"**要让它在第几层看、看多大的窗口**"。
本实验给出一条可直接写进设计的曲线：

    输入边长 S，点直径 d → 2×2 象限选中率

真值由我自己贴点决定（不依赖 CV，无争议）。
"""
from __future__ import annotations

import base64
import json
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

Q_QUAD = ("这张图被分成 4 个象限：左上=1，右上=2，左下=3，右下=4。"
          "图中有一个红色圆点，它在哪个象限？只回答一个数字（1~4）。看不清回答 0。")


def ask(img, prompt, num_predict=8):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": num_predict}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def quad_of(x, y, S):
    return (0 if y < S / 2 else 2) + (0 if x < S / 2 else 1) + 1


def main():
    bg0 = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    if bg0 is None:
        print("底图读不到"); return
    # 点放在右下象限里（避开中心，也能检验是否真看位置）
    FX, FY = 0.68, 0.66

    print("输入边长 / 点直径 / 点占宽比 / 真值象限 / 回答 / 对错")
    print("-" * 78)
    rows = []
    for S in (512, 768, 960, 1920):
        bg = cv2.resize(bg0, (S, S), interpolation=cv2.INTER_AREA)
        bg = (bg.astype(np.float32) * 0.80).clip(0, 255).astype(np.uint8)
        for d in (6, 10, 16, 24, 40):
            img = bg.copy()
            cx, cy = int(S * FX), int(S * FY)
            cv2.circle(img, (cx, cy), max(1, d // 2), (28, 28, 208), -1,
                       lineType=cv2.LINE_AA)
            truth = quad_of(cx, cy, S)
            txt = ask(img, Q_QUAD)
            got = None
            for ch in txt:
                if ch.isdigit():
                    got = int(ch)
                    break
            ok = (got == truth)
            rows.append(dict(S=S, d=d, ratio=round(d / S, 4), truth=truth,
                             got=got, ok=ok, raw=txt[:20]))
            print("%-9d %-8d %-10.4f %-10d %-6s %s"
                  % (S, d, d / S, truth, got, "对" if ok else "错"))

    print()
    print("按「点占输入宽度的比例」汇总（找可靠下限）")
    print("-" * 78)
    for S in (512, 768, 960, 1920):
        sub = [r for r in rows if r["S"] == S]
        good = [r for r in sub if r["ok"]]
        th = min([r["d"] for r in good], default=None)
        thr = min([r["ratio"] for r in good], default=None)
        print("  S=%-5d 命中 %d/%d   最小可命中直径 %s px（占宽 %s）"
              % (S, sum(1 for r in sub if r["ok"]), len(sub), th,
                 ("%.4f" % thr) if thr else "-"))

    out = ROOT / "eval" / "sem_capacity.json"
    json.dump(rows, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % out)


if __name__ == "__main__":
    main()
