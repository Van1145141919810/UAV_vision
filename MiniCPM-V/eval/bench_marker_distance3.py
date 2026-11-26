# -*- coding: utf-8 -*-
"""bench_marker_distance3.py —— 红点距离测试（含对照实验，这是关键）

═══════════════════════════════════════════════════════════════════
为什么必须加对照
───────────────────────────────────────────────────────────────────
第一版结果：VLM 在 5 m~80 m **全部**答"有红点"，连 2.9 像素那档也说有。
但我们已经证明：**闭集问法下模型严重偏向答"是"**
（200 条客观真值里它说了 187 次"是窗户"）。
所以"它说 A"不等于"它看见了"。

必须做的对照：**同一张裁图，不放红点，再问一次**。
    有红点答 A + 无红点答 A  → 这是偏向，不是检测
    有红点答 A + 无红点答 B  → 这才叫真的看见了

═══════════════════════════════════════════════════════════════════
用法：python eval/bench_marker_distance3.py
"""
import base64
import json
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, find_candidates      # noqa: E402
from mcpv import config as C                            # noqa: E402

F = float(C._K1[0])
REAL = float(C.MARKER_REAL_M)
DIST = [5, 10, 20, 30, 50, 80]
W, H, CX, CY = 1920, 1080, 960, 540
MODEL = "openbmb/minicpm-v4.7:1b"
P = ("只看画面正中心。那里有没有一个红色圆点？"
     "只回答一个字母：A=有，B=没有。")


def ask_vlm(crop):
    ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": P,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request("http://127.0.0.1:11434/api/generate",
                                 data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        ans = json.loads(r.read()).get("response", "").strip()
    return next((c for c in ans.upper() if c in "AB"), "?"), ans


def make_frame(bg, d_px, draw=True):
    canvas = bg.copy()
    if draw:
        cv2.circle(canvas, (CX, CY), max(1, int(round(d_px / 2))),
                   (30, 30, 205), -1, lineType=cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def crop_center(frame, half=192):
    c = frame[max(0, CY - half):CY + half, max(0, CX - half):CX + half]
    return cv2.resize(c, (384, 384), interpolation=cv2.INTER_AREA)


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.75).clip(0, 255).astype(np.uint8)

    print(f"红圆直径 {REAL*100:.0f} cm，焦距 {F:.0f} px，画布 {W}x{H}")
    print(f"{'距离':<6}{'直径px':<9}{'CV找到':<8}{'有红点':<8}{'无红点':<8}{'判定'}")
    print("-" * 66)

    rows = []
    for D in DIST:
        d = F * REAL / D
        with_dot = make_frame(bg, d, True)
        no_dot = make_frame(bg, d, False)

        # CV：find_candidates 返回 Candidate(cx, cy, diameter_px, ...)
        try:
            cands = find_candidates(with_dot)
            hit = any(abs(c.cx - CX) < 40 and abs(c.cy - CY) < 40 for c in cands)
            ncv = f"{len(cands)}{'✓' if hit else '✗'}"
        except Exception as e:
            ncv = f"错误:{type(e).__name__}"

        a1, raw1 = ask_vlm(crop_center(with_dot))
        a2, raw2 = ask_vlm(crop_center(no_dot))

        if a1 == "A" and a2 == "B":
            verdict = "★ 真看见"
        elif a1 == "A" and a2 == "A":
            verdict = "✗ 只是偏向（对照也说有）"
        elif a1 == "B":
            verdict = "✗ 没看见"
        else:
            verdict = "?"

        print(f"{D:<6}{d:<9.1f}{ncv:<8}{a1:<8}{a2:<8}{verdict}")
        rows.append((D, d, a1, a2))

        out = ROOT / "eval" / "demo_out"
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / f"marker_{D}m_with.jpg"), crop_center(with_dot))
        cv2.imwrite(str(out / f"marker_{D}m_without.jpg"), crop_center(no_dot))

    print("\n判读：")
    print("  '有红点 A / 无红点 B' = 模型确实分辨出来了")
    print("  '两档都说 A'          = 只是闭集偏向，不能算识别成功")


if __name__ == "__main__":
    main()
