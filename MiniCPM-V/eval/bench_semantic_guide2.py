# -*- coding: utf-8 -*-
"""bench_semantic_guide2.py —— 第一轮失败后的归因实验

第一轮结果（2026-11-18）：
    整帧 1920 送进去：grounding 误差 420~1116 px；4 条带选恒答"1"（2/6 蒙对）。
第一轮的结论**不能是"模型不行"** —— 必须先分清是「模型能力」还是「喂法」。
本实验逐条排除：

  A 图像到底进模型多少 token？  不同尺寸送进去看 prompt_eval_count
       → 若 1920 与 512 的 token 数差不多，说明被缩了，6.5 px 的红点直接消失
  B 给到"分辨率够"的窗口，它能定位吗？  以真值为中心裁 512
       → 若误差回到十几 px，说明能力在，问题在"怎么把有用区域喂进去"
  C 2×2 切片扫描：每片 960×540，问"这格有没有那个标记"
       → 这是可以工程化的方案：切片保分辨率，用"有/没有"分类代替定位
  D 半幅（960×540）上做带选：带失败是分辨率问题还是位置偏置问题

用法：python eval/bench_semantic_guide2.py
"""
from __future__ import annotations

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
Q_GROUND = ("Please provide the bounding box coordinate of the region "
            "this sentence describes: <ref>贴在白色纸上的红色圆形标记</ref>")
Q_HAS = ("这张图里有没有【贴在白色纸上的红色圆形标记】？"
         "只回答一个数字：1 = 有，0 = 没有。")
Q_BAND = ("这张图被横向切成 {n} 条编号带（1 在最上，{n} 在最下，数字标在每条带左侧）。"
          "只有一条带里有【贴在白色纸上的红色圆形标记】。"
          "只回答一个数字（1~{n}）。看不清就回答 0。")


def ask(img, prompt, num_predict=96):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": num_predict}}).encode()
    req = urllib.request.Request(URL, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read())


def parse_box(txt):
    m = re.search(r"<box>\s*([\d\s,]+?)\s*</box>", txt)
    if m:
        n = [int(x) for x in re.findall(r"\d+", m.group(1))]
        if len(n) >= 4:
            return n[:4]
    return None


def draws_bands(img, n):
    out = img.copy()
    H, W = out.shape[:2]
    h = H / float(n)
    for i in range(n):
        y0, y1 = int(i * h), int((i + 1) * h)
        cv2.rectangle(out, (0, y0), (W - 1, y1), (0, 255, 255), 2)
        for txt, col, th in ((str(i + 1), (0, 0, 0), 7), (str(i + 1), (0, 255, 255), 3)):
            cv2.putText(out, txt, (10, y0 + 46), cv2.FONT_HERSHEY_SIMPLEX, 1.6, col, th)
    return out


def main():
    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
    cap = cv2.VideoCapture(str(VID))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()

    # 用第一轮那三帧
    PICKS = [16, 96, 512]

    print("=" * 92)
    print("A. 图像进模型多少 token（同一张图，不同送法）")
    print("=" * 92)
    L0, _ = rig.rectify(frames[96], frames[96])
    print("%-14s %-10s %-10s %s" % ("送图尺寸", "prompt_eval", "image token", "说明"))
    for name, img in (("512x512", cv2.resize(L0, (512, 512))),
                      ("768x432", cv2.resize(L0, (768, 432))),
                      ("1024x576", cv2.resize(L0, (1024, 576))),
                      ("1920x1080", L0)):
        r = ask(img, "这张图里有什么？一句话。", 24)
        pe = r.get("prompt_eval_count", -1)
        # 纯文字的 prompt token 拿个基线
        base = ask(np.zeros((64, 64, 3), np.uint8), "这张图里有什么？一句话。", 8)
        b = base.get("prompt_eval_count", 0)
        print("%-14s %-10s %-10s %s" % (name, pe, pe - b,
                                        "被缩了（token 不随尺寸增长）" if pe - b < 1200 else "基本按原尺寸"))

    print()
    print("=" * 92)
    print("B. 以真值为中心裁 512，问官方 grounding（分辨率够不够的问题）")
    print("=" * 92)
    print("%-6s %-9s %-30s %-16s %-8s %-8s" % ("帧", "直径px", "输出", "换算中心", "误差px", "等效米"))
    rows_b = []
    for i in PICKS:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        ds = detect_reds(L)
        if not ds:
            continue
        c = min(ds, key=lambda q: q["d"])
        cx, cy, d = c["cx"], c["cy"], c["d"]
        half = 256
        x0, y0 = int(max(0, min(W - 512, cx - half))), int(max(0, min(H - 512, cy - half)))
        crop = L[y0:y0 + 512, x0:x0 + 512]
        r = ask(crop, Q_GROUND, 64)
        ans = (r.get("response") or "").strip()
        box = parse_box(ans)
        if box:
            bx = (box[0] + box[2]) / 2 / 1000 * 512 + x0
            by = (box[1] + box[3]) / 2 / 1000 * 512 + y0
            err = float(np.hypot(bx - cx, by - cy))
        else:
            bx = by = err = float("nan")
        fx = 1626.6183
        Z = fx * 0.20 / max(d, 1e-6)
        em = err * Z / fx if not np.isnan(err) else float("nan")
        rows_b.append(dict(frame=i, d=d, ans=ans, box=box, err_px=err, err_m=em))
        print("%-6d %-9.1f %-30s (%4.0f,%4.0f)   %-8.0f %-8.2f"
              % (i, d, ans[:28].replace("\n", " "), bx, by, err, em))

    print()
    print("=" * 92)
    print("C. 2x2 切片：每片 960x540，问「这格有没有那个标记」")
    print("=" * 92)
    print("%-6s %-10s %-8s %s" % ("帧", "真值片号", "答'有'的片", "各片回答"))
    rows_c = []
    for i in PICKS:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        ds = detect_reds(L)
        if not ds:
            continue
        c = min(ds, key=lambda q: q["d"])
        cx, cy = c["cx"], c["cy"]
        truth = (0 if cy < H / 2 else 2) + (0 if cx < W / 2 else 1) + 1
        ans_list = []
        for k in range(4):
            r0, c0 = k // 2, k % 2
            tile = L[r0 * H // 2:(r0 + 1) * H // 2, c0 * W // 2:(c0 + 1) * W // 2]
            r = ask(tile, Q_HAS, 8)
            txt = (r.get("response") or "").strip()
            ans_list.append(txt[:1])
        said = [k + 1 for k, t in enumerate(ans_list) if t == "1"]
        rows_c.append(dict(frame=i, truth=truth, said=said, ans=ans_list))
        print("%-6d %-10d %-8s %s" % (i, truth, said, ans_list))

    print()
    print("=" * 92)
    print("D. 半幅（960x540）上做 4 带选：看带失败是不是分辨率导致的")
    print("=" * 92)
    print("%-6s %-8s %-8s %-8s %s" % ("帧", "真值带", "答带", "对错", "输出"))
    okd = tot = 0
    for i in PICKS:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        ds = detect_reds(L)
        if not ds:
            continue
        c = min(ds, key=lambda q: q["d"])
        half = cv2.resize(L, (960, 540), interpolation=cv2.INTER_AREA)
        truth = int(c["cy"] / (H / 4.0)) + 1
        r = ask(draws_bands(half, 4), Q_BAND.format(n=4), 16)
        txt = (r.get("response") or "").strip()
        got = None
        for ch in txt:
            if ch.isdigit() and 1 <= int(ch) <= 4:
                got = int(ch)
                break
        okd += (got == truth)
        tot += 1
        print("%-6d %-8d %-8s %-8s %s" % (i, truth, got, "对" if got == truth else "错", txt[:40]))

    print()
    print("D 命中 %d/%d" % (okd, tot))
    out = ROOT / "eval" / ("semantic_guide2_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    json.dump({"crop_grounding": rows_b, "slice_2x2": rows_c},
              open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("结果 -> %s" % out)


if __name__ == "__main__":
    main()
