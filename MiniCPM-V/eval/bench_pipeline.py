# -*- coding: utf-8 -*-
"""bench_pipeline.py —— 端到端验证"VLM 粗定位 → CV 精定位"（真实 50 m 场景）

═══════════════════════════════════════════════════════════════════
用户的判断（很关键）
───────────────────────────────────────────────────────────────────
"实际情况只有一个窗户贴大红点。那还需要分割吗？
 分割没找到有红点那个窗户怎么办？"

对。只有一个目标时：
  · 不需要分割去"普查所有窗"
  · 分割反而**引入新失败模式**（它覆盖率 71%，正好漏掉那一扇就完了）

所以正确的链路应该是两步，跳过分割：

    VLM grounding("<ref>红色圆点</ref>") → 粗框
              ↓
    CV 在粗框内低阈值找红点 → 精确中心

═══════════════════════════════════════════════════════════════════
本脚本测什么
───────────────────────────────────────────────────────────────────
真实尺度：1920x1080 立面图，红点 26 cm @ 距离 D（D=30/50/80 m）
  · 第一步 VLM 能不能给出大致正确的区域（误差容忍 = 200 px 内算命中）
  · 第二步 CV 在该区域内精定位（最终误差多少像素）
  · 做对照：关掉 VLM，直接全图 CV 会怎样

用法：python eval/bench_pipeline.py
"""
import base64
import json
import re
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402
from mcpv import config as C                             # noqa: E402

F = float(C._K1[0])
W, H = 1920, 1080
MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
P = ("Please provide the bounding box coordinate of the region "
     "this sentence describes: <ref>红色圆点</ref>")


def ask(img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": P,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": 64}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=300) as r:
        d = json.loads(r.read())
    return d.get("response", "").strip(), time.time() - t0


def parse_box(txt):
    m = re.search(r"<box>\s*([\d\s,]+?)\s*</box>", txt)
    if m:
        n = [int(x) for x in re.findall(r"\d+", m.group(1))]
        if len(n) >= 4:
            return n[:4]
    runs = [int(x) for x in re.findall(r"\d+", txt)]
    if len(runs) >= 4:
        c = runs[-4:]
        if all(0 <= v <= 1000 for v in c):
            return c
    return None


def red_center(bgr, min_area=3):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i][4])
        if a < min_area:
            continue
        if best is None or a > best[0]:
            best = (a, float(cen[i][0]), float(cen[i][1]))
    return best


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.78).clip(0, 255).astype(np.uint8)

    TX, TY = 1210, 430          # 放在一扇窗上（非中心）
    print(f"画布 {W}x{H}   红点真实位置 ({TX},{TY})   红点尺寸 26 cm")
    print(f"{'距离':<7}{'直径':<8}{'VLM粗框中心':<18}{'粗定位误差':<12}"
          f"{'CV精定位':<18}{'最终误差':<10}{'耗时'}")
    print("-" * 92)

    for D in (30, 50, 80):
        d = F * 0.26 / D
        frame = bg.copy()
        cv2.circle(frame, (TX, TY), max(1, int(round(d / 2))), (30, 30, 205),
                   -1, lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        ans, t1 = ask(frame)
        box = parse_box(ans)
        if not box:
            print(f"{D:<7}{d:<8.1f}{'VLM 无输出':<18}{'-':<12}{'-':<18}{'-':<10}{t1:.1f}s")
            continue
        x1 = box[0] / 1000 * W
        y1 = box[1] / 1000 * H
        x2 = box[2] / 1000 * W
        y2 = box[3] / 1000 * H
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        e1 = ((cx - TX) ** 2 + (cy - TY) ** 2) ** 0.5

        # CV 在粗框内精定位（框外扩 60px，防止框太紧切掉目标）
        # ★ 关键：VLM 粗框误差实测 86~119 px，外扩必须足够大，
        #   否则真目标会落在裁剪区域之外，整条链路失败（实测 50m 就栽在这）。
        pad = max(150, int(max(x2 - x1, y2 - y1)))
        ax1, ay1 = int(max(0, x1 - pad)), int(max(0, y1 - pad))
        ax2, ay2 = int(min(W, x2 + pad)), int(min(H, y2 + pad))
        if ax2 - ax1 < 8 or ay2 - ay1 < 8:
            ax1, ay1, ax2, ay2 = max(0, int(cx) - 120), max(0, int(cy) - 120), \
                min(W, int(cx) + 120), min(H, int(cy) + 120)
        crop = frame[ay1:ay2, ax1:ax2]
        t0 = time.time()
        b = red_center(crop, min_area=3)
        t2 = time.time() - t0
        if b:
            gx, gy = ax1 + b[1], ay1 + b[2]
            e2 = ((gx - TX) ** 2 + (gy - TY) ** 2) ** 0.5
            print(f"{D:<7}{d:<8.1f}({cx:.0f},{cy:.0f})"
                  f"{'':<8}{e1:<12.0f}({gx:.0f},{gy:.0f})"
                  f"{'':<6}{e2:<10.0f}{t1+t2:.1f}s")
        else:
            print(f"{D:<7}{d:<8.1f}({cx:.0f},{cy:.0f})"
                  f"{'':<8}{e1:<12.0f}{'框内未找到':<18}{'-':<10}{t1+t2:.1f}s")

    print("\n判读：粗定位误差在 100px 内、最终误差在 5px 内 = 这条链路成立")


if __name__ == "__main__":
    main()
