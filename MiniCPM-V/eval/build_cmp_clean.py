# -*- coding: utf-8 -*-
"""build_cmp_clean.py —— 用 CMP 分割标注构造"真值无歧义"的大样本

═══════════════════════════════════════════════════════════════════
要解决什么
───────────────────────────────────────────────────────────────────
人工标注的 38 条里，很多裁剪"墙和窗都有"，真值只能靠人主观定
（我们的失败样例里就出现"混凝土墙面与窗"这种——模型没错，是定义模糊）。

CMP 每张图有**逐像素的窗户标注**，于是可以计算：
    中心区域（画幅中央 50%）里，窗户像素占多少 = win_frac

按 win_frac 严格分三档：
    win_frac >= 0.60  → 正例（中心以窗为主）     标签 window
    win_frac <= 0.05  → 负例（中心基本没有窗）   标签 none
    0.05 ~ 0.60       → 歧义，**不参与准确率**，只统计模型会不会标"说不清"

这样每条样本的真值都是**算出来的、可复现的**，没有人工主观成分。

用法：
    python eval/build_cmp_clean.py --n 200
"""
import argparse
import csv
import random
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode      # noqa: E402

CMP = ROOT / "data" / "external" / "cmp" / "base"
CROPS = ROOT / "eval" / "crops_clean"
CSV = ROOT / "eval" / "labels_clean.csv"
SIDE = 384
WIN_CLS = 3          # CMP 调色板：3 = window
POS_MIN, NEG_MAX = 0.60, 0.05


def pairs():
    out = []
    for img in sorted(CMP.glob("*.jpg")):
        m = img.with_suffix(".png")
        if m.exists():
            out.append((img, m))
    return out


def center_frac(mask, x0, y0, size, cw, ch):
    """裁剪区域内、中心 50% 子区域的窗户像素占比。"""
    sub = mask[y0:y0 + ch, x0:x0 + cw]
    if sub.size == 0:
        return None
    h, w = sub.shape
    cy0, cy1 = int(h * 0.25), int(h * 0.75)
    cx0, cx1 = int(w * 0.25), int(w * 0.75)
    c = sub[cy0:cy1, cx0:cx1]
    if c.size == 0:
        return None
    return float((c == WIN_CLS).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200, help="目标样本数（正负各半）")
    ap.add_argument("--seed", type=int, default=20261105)
    args = ap.parse_args()
    random.seed(args.seed)

    ps = pairs()
    if not ps:
        raise SystemExit(f"没找到 CMP 图，检查 {CMP}")
    random.shuffle(ps)
    CROPS.mkdir(parents=True, exist_ok=True)

    rows, got = [], {"window": 0, "none": 0, "ambiguous": 0}
    half = args.n // 2

    for img_p, mask_p in ps:
        if got["window"] >= half and got["none"] >= half:
            break
        img = imread_unicode(img_p)
        mk = np.array(Image.open(str(mask_p)))
        if img is None or mk is None:
            continue
        if mk.shape[:2] != img.shape[:2]:
            mk = cv2.resize(mk, (img.shape[1], img.shape[0]),
                            interpolation=cv2.INTER_NEAREST)
        H, W = img.shape[:2]
        # 候选框边长：图片短边的 22%
        side = max(96, int(min(H, W) * 0.22))
        tried = 0
        while tried < 12:
            tried += 1
            x0 = random.randint(0, max(0, W - side))
            y0 = random.randint(0, max(0, H - side))
            f = center_frac(mk, x0, y0, side, side, side)
            if f is None:
                continue
            if POS_MIN <= f and got["window"] < half:
                cls = "window"
            elif f <= NEG_MAX and got["none"] < half:
                cls = "none"
            else:
                got["ambiguous"] += 1
                continue
            patch = img[y0:y0 + side, x0:x0 + side]
            if patch.size == 0:
                continue
            patch = cv2.resize(patch, (SIDE, SIDE), interpolation=cv2.INTER_AREA)
            name = f"clean_{img_p.stem}_x{x0}_y{y0}.jpg"
            imwrite_unicode(CROPS / name, patch, quality=95)
            rows.append(dict(
                image_id=img_p.stem,
                crop=f"crops_clean/{name}",
                source=img_p.name,
                kind="clean_window" if cls == "window" else "clean_none",
                box=f"{x0},{y0},{side},{side}",
                scenario="cmp_clean",
                gt_is_window="1" if cls == "window" else "0",
                gt_window_state="closed" if cls == "window" else "n/a",
                gt_glare="unknown",
                win_frac=round(f, 3),
                note=f"中心窗占比 {f:.2f}",
            ))
            got[cls] += 1
        if len(rows) >= args.n:
            break

    if not rows:
        raise SystemExit("没生成样本")
    with open(CSV, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"生成 {len(rows)} 条 → {CSV}")
    print(f"  正例(中心窗≥{POS_MIN}) {got['window']}")
    print(f"  负例(中心窗≤{NEG_MAX}) {got['none']}")
    print(f"  被判为歧义而丢弃 {got['ambiguous']} 次")
    print(f"裁剪图 → {CROPS}")


if __name__ == "__main__":
    main()
