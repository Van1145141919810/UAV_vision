# -*- coding: utf-8 -*-
"""bench_seg_headroom.py —— 现有 CV 窗框在 CMP 标注上的覆盖率（分割模型的"可追赶空间"）

═══════════════════════════════════════════════════════════════════
它回答什么
───────────────────────────────────────────────────────────────────
"如果把前端换成训练过的语义分割，能拿回多少？"

用 CMP 的逐像素窗标注当真值，测现有颜色+几何检测器：
  · 窗像素被框覆盖的比例（覆盖率 / recall）
  · 框内有多少是窗像素（精度 / precision）
分割模型理论上能接近 1.0，所以这组数字就是**可追赶的空间**。

不训练模型，只做基线测量（训练 SegFormer 需要 torch+transformers，另立项）。

用法：python eval/bench_seg_headroom.py --n 40
"""
import argparse
import random
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402
from mcpv.windows import find_windows                     # noqa: E402

CMP = ROOT / "data" / "external" / "cmp" / "base"
WIN_CLS = 3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    random.seed(a.seed)

    imgs = sorted(CMP.glob("*.jpg"))
    random.shuffle(imgs)
    imgs = imgs[:a.n]

    cov_sum, prec_sum, n = 0.0, 0.0, 0
    for p in imgs:
        mp = p.with_suffix(".png")
        if not mp.exists():
            continue
        img = imread_unicode(p)
        mk = np.array(Image.open(str(mp)))
        if img is None or mk is None:
            continue
        H, W = mk.shape[:2]
        gt = (mk == WIN_CLS)
        if gt.sum() < 500:
            continue                      # 这张图几乎没窗，跳过

        # 现有检测器给的框（用缩略图加速）
        scale = 1.0
        small = img
        if max(H, W) > 1200:
            scale = 1200.0 / max(H, W)
            small = cv2.resize(img, None, fx=scale, fy=scale)
        boxes = find_windows(small)

        cover = np.zeros((H, W), bool)
        for (x, y, w, h) in boxes:
            x0, y0 = int(x / scale), int(y / scale)
            x1, y1 = int((x + w) / scale), int((y + h) / scale)
            cover[max(0, y0):min(H, y1), max(0, x0):min(W, x1)] = True

        inter = (cover & gt).sum()
        cov = inter / gt.sum()                     # 窗像素被覆盖的比例
        prec = inter / max(1, cover.sum())         # 框内是窗像素的比例
        cov_sum += cov
        prec_sum += prec
        n += 1
        print(f"  {p.stem}  覆盖 {cov*100:5.1f}%   框内纯度 {prec*100:5.1f}%")

    if not n:
        raise SystemExit("没有可用样本")
    print(f"\n共 {n} 张 CMP 图")
    print(f"  平均窗覆盖率（recall） : {100*cov_sum/n:.1f}%")
    print(f"  平均框内纯度（precision）: {100*prec_sum/n:.1f}%")
    print("\n  → 训练过的语义分割理论上可接近 100%/95%，")
    print("     这中间的差额就是换前端的收益空间。")


if __name__ == "__main__":
    import cv2
    main()
