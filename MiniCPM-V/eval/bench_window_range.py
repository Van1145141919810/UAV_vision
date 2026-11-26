# -*- coding: utf-8 -*-
"""bench_window_range.py —— 用真实立面照回答两件事

  T4 尺度曲线：同一张立面照，把「窗宽」缩到 300/150/72/36/18 px，问"能看见窗户吗"
     → 找它的**窗户识别下限**。45 m 时窗宽是 72 px，所以这条曲线直接回答
       "45 m 在不在 VLM 能力范围内"，不依赖任何方向真值。

  T5 方向是不是良定义：12 张立面照统一缩到「窗宽 72 px」（= 45 m 预算），
     问"窗户主要在左半 / 右半 / 两边都有"（3 选 1，**第 3 项是正经答案**）
     → 多层立面上"哪边"可能根本没有答案；如果它多数答"两边都有"，
       说明**远段给方向这个概念对整片立面不成立**，那比给个假方向好。

真值怎么定（客观，不用人工标）
  · 窗宽：用项目自己的 windows.find_windows 检测，取中位数
  · 左右：所有检出的窗框，若横跨 min_x < 0.25W 且 max_x > 0.75W → 真值=3（两边）
          否则看中位数落在哪半

用法
    python eval\\bench_window_range.py --exp T4,T5
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mcpv.cv import imread_unicode                       # noqa: E402
from mcpv.windows import find_windows                   # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
SAMPLES = ROOT / "data" / "samples"
OUT_JSON = ROOT / "eval" / "window_range.json"
OUT_DIR = ROOT / "eval" / "window_range_figs"

Q_HAS = ("这张图里能看到建筑的窗户（玻璃窗）吗？只回答一个字母：A = 能，B = 不能。")
Q_SIDE = ("这张图是一张建筑外立面。画面里那些【玻璃窗】主要分布在画面的哪一边？"
          "只回答一个数字：1 = 主要在左半，2 = 主要在右半，3 = 左右两边都有、分不出来。")


def ask(img, prompt, npd=8):
    import base64
    import urllib.request
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        return ""
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=180) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def letter(txt, allowed="AB"):
    for ch in (txt or "").upper()[:8]:
        if ch in allowed:
            return ch
    return "?"


def digit(txt, lo=1, hi=3):
    for ch in (txt or "")[:8]:
        if ch.isdigit() and lo <= int(ch) <= hi:
            return int(ch)
    return None


def imwrite_u(path, img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if ok:
        Path(path).write_bytes(buf.tobytes())


def window_w(im):
    """估计这张立面照里「一扇窗」的像素宽度（中位数）。"""
    h, w = im.shape[:2]
    k = 2048.0 / max(w, h)
    small = cv2.resize(im, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA) if k < 1 else im
    bx = find_windows(small)
    if not bx:
        return None, []
    ws = sorted(b[2] for b in bx)
    med = ws[len(ws) // 2]
    return med / (small.shape[1] / float(w)), bx


def scale_to_window(im, med_w, target_px, max_out=1920):
    """缩放，使窗宽 ≈ target_px；宽度超过 max_out 就中心裁到 1920×1080。"""
    s = float(target_px) / max(med_w, 1.0)
    out = cv2.resize(im, (max(1, int(round(im.shape[1] * s))), max(1, int(round(im.shape[0] * s)))),
                     interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    if out.shape[1] > max_out:
        H, W = 1080, max_out
        if out.shape[0] < H:
            out = cv2.resize(out, (int(out.shape[1] * H / out.shape[0]), H))
        x0 = (out.shape[1] - W) // 2
        y0 = (out.shape[0] - H) // 2
        out = out[max(0, y0):max(0, y0) + H, max(0, x0):max(0, x0) + W]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="T4,T5")
    ap.add_argument("--scales", default="300,150,72,36,18")
    ap.add_argument("--files", default="win01,win02,win03,win05,win06,win09,win10,win12")
    ap.add_argument("--t4files", default="win02,win03,win05,win06,win09,win10")
    a = ap.parse_args()
    exps = set(a.exp.replace("T", "").split(","))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []

    files = [SAMPLES / (n.strip() + ".jpg") for n in a.files.split(",")]
    files = [f for f in files if f.exists()]
    t4 = [SAMPLES / (n.strip() + ".jpg") for n in a.t4files.split(",")]
    t4 = [f for f in t4 if f.exists()]

    print("立面照 %d 张；T4 用 %d 张" % (len(files), len(t4)), flush=True)

    if "4" in exps:
        print("\n=== T4 尺度曲线：窗宽缩到 300/150/72/36/18 px，问「能看见窗户吗」===", flush=True)
        for f in t4:
            im = imread_unicode(f)
            med, bx = window_w(im)
            if med is None:
                print("  %s 检测不到窗框，跳过" % f.name, flush=True)
                continue
            print("  %s 原生 %dx%d，窗宽≈%.0f px，检出 %d 个框"
                  % (f.name, im.shape[1], im.shape[0], med, len(bx)), flush=True)
            for s in [int(x) for x in a.scales.split(",")]:
                img = scale_to_window(im, med, s)
                ans = letter(ask(img, Q_HAS), "AB")
                rows.append(dict(exp="T4", file=f.name, target_px=s,
                                 sent_w=int(img.shape[1]), sent_h=int(img.shape[0]),
                                 ans=ans, ok=ans == "A"))
                imwrite_u(OUT_DIR / ("T4_%s_%03dpx.jpg" % (f.stem, s)), img)
                print("     窗%3dpx → 送 %dx%d  答 %s  %s"
                      % (s, img.shape[1], img.shape[0], ans, "对" if ans == "A" else "错"), flush=True)
                json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        for s in [int(x) for x in a.scales.split(",")]:
            sub = [r for r in rows if r["exp"] == "T4" and r["target_px"] == s]
            hit = sum(1 for r in sub if r["ok"])
            print("  T4 窗宽 %3d px：认出窗户 %d/%d = %.0f%%"
                  % (s, hit, len(sub), 100.0 * hit / max(1, len(sub))))

    if "5" in exps:
        print("\n=== T5 方向是否良定义：窗宽统一 72 px（=45 m 预算），问「主要在左/右/两边」===",
              flush=True)
        for f in files:
            im = imread_unicode(f)
            med, bx = window_w(im)
            if med is None:
                continue
            img = scale_to_window(im, med, 72)
            # 真值：用缩放后的坐标重新检测一次
            bx2 = find_windows(img)
            H, W = img.shape[:2]
            if not bx2:
                truth = None
            else:
                xs0 = min(b[0] for b in bx2)
                xs1 = max(b[0] + b[2] for b in bx2)
                if xs0 < 0.25 * W and xs1 > 0.75 * W:
                    truth = 3
                else:
                    med_x = float(np.median([b[0] + b[2] / 2.0 for b in bx2]))
                    truth = 1 if med_x < W / 2.0 else 2
            got = digit(ask(img, Q_SIDE), 1, 3)
            rows.append(dict(exp="T5", file=f.name, truth=truth, got=got,
                             ok=(got == truth), n_box=len(bx2),
                             sent_w=int(W), sent_h=int(H)))
            imwrite_u(OUT_DIR / ("T5_%s_72px.jpg" % f.stem), img)
            print("  %-10s 送 %dx%d 框%2d 个 | 真值 %s | 答 %s  %s"
                  % (f.name, W, H, len(bx2), truth, got,
                     "对" if got == truth else "错"), flush=True)
            json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        sub = [r for r in rows if r["exp"] == "T5"]
        hit = sum(1 for r in sub if r["ok"])
        ans3 = sum(1 for r in sub if r["got"] == 3)
        tru3 = sum(1 for r in sub if r["truth"] == 3)
        print("  T5 命中 %d/%d；它答「两边都有」%d 次，真值「两边都有」%d 次"
              % (hit, len(sub), ans3, tru3))

    json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % OUT_JSON)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
