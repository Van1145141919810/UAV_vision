# -*- coding: utf-8 -*-
"""bench_sem_capacity2.py —— 把编号"画进图里"再问，测真正的可用分辨率下限

上一轮的教训
──────────────────────────────────────────────────────────────
第一版我把象限约定写在**提示词里**（"左上=1，右上=2…"），结果 20 次全答 3 —— 
但二分测试证明图它是看得见的（200~960 都能答"有红点"）。
所以失败原因是**让它把方位映射成编号**，不是它看不见。

本项目已经踩过两次同样的坑：
  · scene.py：选项写在提示词里 → 模型把选项原样抄回来；改成单字母才稳
  · pick_core：编号直接**画在每一格上** → 立刻能用
结论：**编号必须画在图上**，让它做「哪一格」的选择。

本实验用画上去的编号测：
    输入边长 S × 点直径 d → 4 选 1 选中率
再换算成工程规则：在 1920 的帧上，红点 7 px 时，最多能喂多大的窗口。
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
Q = ("这张图被分成 {n} 个带编号的格子（编号画在每格里）。"
     "只有一格里有一个红色圆点。它在几号格？只回答一个数字。")


def ask(img, prompt, npd=12):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body,
                                headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=300) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def draw_grid(img, rows=2, cols=2):
    """把编号画进每个格子（左上角），返回图和每格中心。"""
    out = img.copy()
    H, W = out.shape[:2]
    cells = []
    k = 1
    for r in range(rows):
        for c in range(cols):
            y0, y1 = int(r * H / rows), int((r + 1) * H / rows)
            x0, x1 = int(c * W / cols), int((c + 1) * W / cols)
            cv2.rectangle(out, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 255), 2)
            cv2.putText(out, str(k), (x0 + 12, y0 + 44), cv2.FONT_HERSHEY_SIMPLEX,
                        1.5, (0, 0, 0), 7)
            cv2.putText(out, str(k), (x0 + 12, y0 + 44), cv2.FONT_HERSHEY_SIMPLEX,
                        1.5, (0, 255, 255), 3)
            cells.append((k, x0, y0, x1, y1))
            k += 1
    return out, cells


def main():
    bg0 = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    # 点放在每格中心偏右一点，避开左上角的编号
    FRAC = {1: (0.60, 0.60), 2: (0.85, 0.60), 3: (0.60, 0.90), 4: (0.85, 0.90)}

    print("%-7s %-7s %-9s %-7s %-7s %-6s %s"
          % ("边长S", "点径d", "d/S", "真值格", "回答", "对错", "点像素(2048基准)"))
    print("-" * 84)
    rows = []
    qi = 1
    for S in (512, 960, 1920):
        for d in (8, 16, 32, 70):
            bg = cv2.resize(bg0, (S, S), interpolation=cv2.INTER_AREA)
            bg = (bg.astype(np.float32) * 0.80).clip(0, 255).astype(np.uint8)
            img, cells = draw_grid(bg, 2, 2)
            k, x0, y0, x1, y1 = cells[qi - 1]
            fx, fy = FRAC[qi]
            cx = int(x0 + (x1 - x0) * fx)
            cy = int(y0 + (y1 - y0) * fy)
            cv2.circle(img, (cx, cy), max(1, d // 2), (28, 28, 208), -1,
                       lineType=cv2.LINE_AA)
            txt = ask(img, Q.format(n=4))
            got = None
            for ch in txt:
                if ch.isdigit():
                    got = int(ch)
                    break
            ok = (got == qi)
            rows.append(dict(S=S, d=d, truth=qi, got=got, ok=ok, raw=txt[:20]))
            print("%-7d %-7d %-9.4f %-7d %-7s %-6s %.0f"
                  % (S, d, d / S, qi, got, "对" if ok else "错", d * 2048.0 / S))
            qi = qi % 4 + 1

    print()
    print("按 S 汇总（用「编号画进图里」的问法）")
    print("-" * 84)
    for S in (512, 960, 1920):
        sub = [r for r in rows if r["S"] == S]
        hit = [r for r in sub if r["ok"]]
        small = min([r["d"] for r in hit], default=None)
        print("  S=%-5d 命中 %d/%d   最小可命中 d = %s px（占宽 %s）"
              % (S, len(hit), len(sub), small,
                 ("%.4f" % (small / S)) if small else "-"))

    out = ROOT / "eval" / "sem_capacity2.json"
    json.dump(rows, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % out)


if __name__ == "__main__":
    main()
