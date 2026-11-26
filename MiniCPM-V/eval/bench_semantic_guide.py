# -*- coding: utf-8 -*-
"""bench_semantic_guide.py —— 测"用 VLM 做语义引导"到底行不行

为什么写这个
──────────────────────────────────────────────────────────────
现在的线上用法是：CV 出了 ≥2 个候选 → 把候选拼成编号网格 → 让 VLM 选一个编号。
这个用法有两个致命问题：
  1. 只在 CV 已经有答案时才问它（CV 只给 1 个候选时根本不问）；
  2. 输出只有 4 bit（一个编号），没有新增任何信息。
拿这种用法和 CV 比准确率，等于让博士去做选择题再结论"他不如计算器"。

本脚本测的是另一种用法：**让 VLM 输出可用于引导的连续量**。
  A. grounding：官方 <ref>/<box> 格式，直接出框 → 误差多少 px / 多少米
  B. 带选：把画面横切成 4 条编号带，问"红圆在哪条带" → 直接映射成高度
  C. 降采样后重测 A/B：模拟"远到 CV 看不见"，看语义是否还在
  对照：同一帧上"现在的用法"会不会被触发（cands<2 → 静默）

用法：python eval/bench_semantic_guide.py
"""
from __future__ import annotations

import base64
import json
import os
import re
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
S14 = Path(r"D:\Desktop\无人系统\_ref\s14")
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(S14))

from vb09.perception.stereo_rig import StereoRig            # noqa: E402
from vb09.perception.reddot.config import DotConfig          # noqa: E402
from vb09.perception.reddot.detect import DotDetectorCore    # noqa: E402
from mcpv.detect_live import detect_reds                     # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"

P_GROUND = ("Please provide the bounding box coordinate of the region "
            "this sentence describes: <ref>贴在白色纸上的红色圆形标记</ref>")

P_BAND = ("这张图被横向切成 {n} 条编号带（1 在最上，{n} 在最下，数字标在每条带左侧）。"
          "只有一条带里有【贴在白色纸上的红色圆形标记】。"
          "只回答一个数字（1~{n}）。看不清就回答 0。")


def ask(img, prompt, num_predict=128):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": num_predict}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read()).get("response", "").strip()


def parse_box(txt):
    m = re.search(r"<box>\s*([\d\s,]+?)\s*</box>", txt)
    if m:
        n = [int(x) for x in re.findall(r"\d+", m.group(1))]
        if len(n) >= 4:
            return n[:4]
    runs = re.findall(r"\d+", txt)
    if len(runs) >= 4:
        n = [int(x) for x in runs[-4:]]
        if all(0 <= v <= 1000 for v in n):
            return n
    return None


def parse_one(txt, n):
    for ch in (txt or ""):
        if ch.isdigit():
            k = int(ch)
            if 1 <= k <= n:
                return k
            if k == 0:
                return None
    return None


def draw_bands(img, n=4):
    """在图上画 n 条横带 + 左侧大字编号（视觉脚手架）。"""
    out = img.copy()
    H, W = out.shape[:2]
    h = H / float(n)
    for i in range(n):
        y0, y1 = int(i * h), int((i + 1) * h)
        cv2.rectangle(out, (0, y0), (W - 1, y1), (0, 255, 255), 2)
        cv2.putText(out, str(i + 1), (10, y0 + 46), cv2.FONT_HERSHEY_SIMPLEX,
                    1.6, (0, 0, 0), 7)
        cv2.putText(out, str(i + 1), (10, y0 + 46), cv2.FONT_HERSHEY_SIMPLEX,
                    1.6, (0, 255, 255), 3)
    return out


def main():
    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
    cfg = DotConfig()
    cfg.min_d_px, cfg.dot_d, cfg.d_max, cfg.open_k = 7.0, 0.20, 0.28, 1
    cfg0 = DotConfig()          # 他的原版参数，用来看"CV 原版能不能检出"
    rig_lo = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=0.5)
    det0 = DotDetectorCore(rig_lo, cfg0)

    cap = cv2.VideoCapture(str(VID))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()

    # 抽稀扫描，找远/中/近三档（按 dmaj 排序）
    cand = []
    for i in range(0, len(frames), 4):
        L, _ = rig.rectify(frames[i], frames[i])
        ds = detect_reds(L)
        for c in ds:
            cx, cy, d = c["cx"], c["cy"], c["d"]
            if 60 < cx < 1860 and 60 < cy < 1020:
                cand.append((d, i, cx, cy))
    cand.sort()
    if not cand:
        print("没抽到候选"); return
    picks = []
    for frac in (0.02, 0.45, 0.92):
        d, i, cx, cy = cand[int(frac * (len(cand) - 1))]
        if i not in [p[1] for p in picks]:
            picks.append((d, i, cx, cy))
    print("三档帧（按检测直径）：")
    for d, i, cx, cy in picks:
        print("  帧%-4d 直径 %5.1f px  中心 (%d,%d)" % (i, d, cx, cy))

    print()
    print("=" * 96)
    print("%-6s %-9s %-24s %-16s %-9s %-8s" %
          ("帧", "直径px", "grounding 输出", "换算中心", "误差px", "等效米"))
    print("=" * 96)

    rows = []
    for d, i, cx, cy in picks:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        # CV 侧同时报：原版参数能检出几个候选（看"现在的用法"会不会被触发）
        small = cv2.resize(frames[i], (960, 540), interpolation=cv2.INTER_AREA)
        Ls, Rs = rig_lo.rectify(small, small)
        n_orig = len(det0.process(Ls, Rs, i / 10.0).cands)

        ans = ask(L, P_GROUND)
        box = parse_box(ans)
        if box:
            bx = (box[0] + box[2]) / 2.0 / 1000.0 * W
            by = (box[1] + box[3]) / 2.0 / 1000.0 * H
            err = float(np.hypot(bx - cx, by - cy))
        else:
            bx = by = err = float("nan")
        # 等效米：用针孔 d_m = D·f/d_px，误差米 ≈ err·Z/f；Z 由 dot_d/直径反推
        fx = 1626.6183
        Z = fx * 0.20 / max(d, 1e-6)
        err_m = err * Z / fx if not np.isnan(err) else float("nan")
        rows.append(dict(frame=i, d=d, gt=(cx, cy), box=box, est=(bx, by),
                         err_px=err, err_m=err_m, n_orig=n_orig, ans=ans))
        print("%-6d %-9.1f %-24s (%4.0f,%4.0f)   %-9.0f %-8.2f"
              % (i, d, (ans or "")[:22].replace("\n", " "), bx, by, err, err_m))
        print("       └ 原版参数下 CV 候选数 = %d%s"
              % (n_orig, "   ← 现在的用法不会触发（cands<2）" if n_orig < 2 else ""))

    print()
    print("=" * 96)
    print("B. 带选（4 条横带）：直接给出「在哪一层高度」—— CV 不可能回答这个")
    print("=" * 96)
    print("%-6s %-8s %-8s %-10s %-38s %s" % ("帧", "真值带", "答案带", "对错", "原始输出", "缩放"))
    print("-" * 96)
    band_rows = []
    for d, i, cx, cy in picks:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        truth_band = int(cy / (H / 4.0)) + 1
        for tag, img in (("1920", L), ("480", cv2.resize(L, (480, 270), interpolation=cv2.INTER_AREA))):
            im = draw_bands(img, 4)
            ans = ask(im, P_BAND.format(n=4), num_predict=16)
            got = parse_one(ans, 4)
            ok = (got == truth_band)
            band_rows.append(dict(frame=i, scale=tag, truth=truth_band, got=got, ok=ok))
            print("%-6d %-8d %-8s %-10s %-38s %s"
                  % (i, truth_band, got, "对" if ok else "错",
                     (ans or "")[:34].replace("\n", " "), tag))

    okp = sum(1 for r in band_rows if r["ok"])
    print()
    print("带选命中 %d/%d；其中 1920 输入 %d/%d，480（模拟远距）输入 %d/%d"
          % (okp, len(band_rows),
             sum(1 for r in band_rows if r["ok"] and r["scale"] == "1920"),
             sum(1 for r in band_rows if r["scale"] == "1920"),
             sum(1 for r in band_rows if r["ok"] and r["scale"] == "480"),
             sum(1 for r in band_rows if r["scale"] == "480")))

    out = ROOT / "eval" / ("semantic_guide_%s.json"
                           % __import__("time").strftime("%Y%m%d_%H%M%S"))
    json.dump({"grounding": rows, "bands": band_rows},
              open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % out)


if __name__ == "__main__":
    main()
