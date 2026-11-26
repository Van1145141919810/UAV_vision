# -*- coding: utf-8 -*-
"""bench_center_confirm.py —— 测「人把飞机对准一处，VLM 确认中间是不是窗户」

这才是真实的用法（2026-11-23 用户澄清）：
  远距不要求 VLM 说“往哪边飞”，只要求它回答「画面正中心这块是不是窗户」；
  是 → 人按 Enter 粗动一下；不是 → 别动，人自己再对一次。

实验设计（有正例也有负例，避免上次“一律答能”的坑）
  · 立面照缩放到「窗宽 = 72 px」= 45 m 像素预算
  · 正例：把某个检出窗框的中心摆到画面正中
  · 负例：把“离所有窗框都足够远”的点（墙/柱/天空/绿化）摆到画面正中
  · 问：src/mcpv/prompts.py 里的 P_IS_WINDOW（已版本化、已在用）

真值客观：窗框来自项目自己的 windows.find_windows；负例点是算出来的（离框 > 1.2 倍窗宽）。

用法
    python eval\\bench_center_confirm.py --n 6
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
sys.path.insert(0, str(ROOT / "eval"))

from mcpv.cv import imread_unicode                       # noqa: E402
from mcpv.prompts import P_IS_WINDOW                     # noqa: E402
from mcpv.windows import find_windows                    # noqa: E402
from bench_window_range import ask, imwrite_u, scale_to_window, window_w  # noqa: E402

SAMPLES = ROOT / "data" / "samples"
OUT_JSON = ROOT / "eval" / "center_confirm.json"
OUT_DIR = ROOT / "eval" / "center_figs"
CANVAS = (1280, 720)


def pan_center(img, cx, cy):
    """取 CANVAS，使原图上的 (cx,cy) 落在画面正中（左右 reflect、上下 replicate）。"""
    W, H = CANVAS
    px, py = W // 2, H // 2
    t = cv2.copyMakeBorder(img, py, py, px, px, cv2.BORDER_REFLECT_101)
    x0 = max(0, min(t.shape[1] - W, int(round(cx))))
    y0 = max(0, min(t.shape[0] - H, int(round(cy))))
    return t[y0:y0 + H, x0:x0 + W].copy()


def pick_points(bx, W, H, n_pos=1, n_neg=1, rng=None):
    """正例 = 窗框中心；负例 = 离所有框都远的点。"""
    rng = rng or np.random.RandomState(0)
    boxes = [b for b in bx if b[2] > 8 and b[3] > 8]
    if not boxes:
        return [], []
    med = float(np.median([b[2] for b in boxes]))
    ctr = np.array([W / 2.0, H / 2.0])
    boxes.sort(key=lambda b: np.hypot(b[0] + b[2] / 2 - ctr[0], b[1] + b[3] / 2 - ctr[1]))
    pos = [(b[0] + b[2] / 2.0, b[1] + b[3] / 2.0) for b in boxes[:n_pos]]
    neg = []
    tries = 0
    while len(neg) < n_neg and tries < 4000:
        tries += 1
        x = rng.uniform(0.05, 0.95) * W
        y = rng.uniform(0.05, 0.95) * H
        d = min(max(0.0, max(b[0] - x, x - (b[0] + b[2]), b[1] - y, y - (b[1] + b[3])))
                for b in boxes)
        if d > 1.2 * med:
            neg.append((x, y))
    return pos, neg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--files", default="win02,win03,win05,win06,win09,win10")
    ap.add_argument("--pos", type=int, default=1)
    ap.add_argument("--neg", type=int, default=2)
    a = ap.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    files = [SAMPLES / (n.strip() + ".jpg") for n in a.files.split(",") if (SAMPLES / (n.strip() + ".jpg")).exists()]
    rng = np.random.RandomState(20261123)
    print("问法（来自 prompts.py，线上同一句）:\n  %s\n" % P_IS_WINDOW, flush=True)

    for f in files:
        im = imread_unicode(f)
        med, _ = window_w(im)
        if med is None:
            print("  %s 检不到窗框，跳过" % f.name, flush=True)
            continue
        img = scale_to_window(im, med, 72)
        H, W = img.shape[:2]
        bx = find_windows(img)
        pos, neg = pick_points(bx, W, H, a.pos, a.neg, rng)
        print("%-10s 送前 %dx%d  框 %d 个 → 正例 %d / 负例 %d"
              % (f.name, W, H, len(bx), len(pos), len(neg)), flush=True)
        for kind, pts, want in (("pos", pos, "A"), ("neg", neg, "B")):
            for i, (cx, cy) in enumerate(pts):
                crop = pan_center(img, cx, cy)
                ans = ask(crop, P_IS_WINDOW, npd=4)
                got = "A" if "A" in ans.upper()[:6] else ("B" if "B" in ans.upper()[:6] else "?")
                rows.append(dict(file=f.name, kind=kind, cx=round(float(cx), 1),
                                 cy=round(float(cy), 1), want=want, got=got,
                                 ok=got == want, raw=ans[:16]))
                imwrite_u(OUT_DIR / ("%s_%s%d.jpg" % (f.stem, kind, i)), crop)
                print("    %-3s%-2d 中心(%4d,%4d) 期望%s → 答 %s  %s   raw=%r"
                      % (kind, i, cx, cy, want, got, "对" if got == want else "错", ans[:12]),
                      flush=True)
                json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    print()
    for kind, lab in (("pos", "正例（中心是窗户）"), ("neg", "负例（中心不是窗户）")):
        sub = [r for r in rows if r["kind"] == kind]
        hit = sum(1 for r in sub if r["ok"])
        print("  %-18s %d/%d = %.0f%%" % (lab, hit, len(sub), 100.0 * hit / max(1, len(sub))))
    print("\n结果 -> %s" % OUT_JSON)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
