# -*- coding: utf-8 -*-
"""make_labels.py —— 自动切出评测样本并生成标注模板

═══════════════════════════════════════════════════════════════════
为什么要在"裁剪"上评测，而不是整张图
───────────────────────────────────────────────────────────────────
部署时模型吃的是**框出来的那一小块**（你双击选中的区域），不是整张 4096×3072。
在整图上问"正中是不是窗户"，正中心那些像素根本没有意义。

所以这里按部署形态切样本：
  · 每张原图切 2 类裁剪
      w1,w2  = 窗户检测框最大的两个（正例候选）
      bg1    = 远离所有框的背景块（负例候选，防止模型"见啥都说是窗户"）
  · 另加一组**整图**样本（g1），对应文档 §1.3 说的"真实建筑图像输入"，
    用来单独统计"整图级别的能力"，和裁剪级分开报，不混为一谈。

人工只需要填 gt_* 四列（其余不要动）。

用法：
    python eval/make_labels.py            # 生成 eval/labels.csv（已存在则加 .new）
    python eval/make_labels.py --force    # 覆盖
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mcpv.cv import imread_unicode, imwrite_unicode      # noqa: E402
from mcpv.windows import find_windows                    # noqa: E402

SAMPLES = ROOT / "data" / "samples"
CROPS = ROOT / "eval" / "crops"
LABELS = ROOT / "eval" / "labels.csv"

# 允许从别处导入（如手机拍的图）
import os
if os.environ.get("MCPV_SRC"):
    SAMPLES = Path(os.environ["MCPV_SRC"])
if os.environ.get("MCPV_CROPS"):
    CROPS = Path(os.environ["MCPV_CROPS"])
if os.environ.get("MCPV_LABELS"):
    LABELS = Path(os.environ["MCPV_LABELS"])

SIDE = 384          # 和部署时送给模型的尺寸一致


def _square(img, cx, cy, half):
    h, w = img.shape[:2]
    x0 = max(0, min(w - 2 * half, int(cx - half)))
    y0 = max(0, min(h - 2 * half, int(cy - half)))
    x1, y1 = min(w, x0 + 2 * half), min(h, y0 + 2 * half)
    p = img[y0:y1, x0:x1]
    return p


def _crop_path(f: Path) -> str:
    """整图样本的路径：在项目内用相对路径，外部用绝对路径。"""
    try:
        rel = f.resolve().relative_to(ROOT)
        return "../" + rel.as_posix()
    except ValueError:
        return str(f.resolve())


def build():
    CROPS.mkdir(parents=True, exist_ok=True)
    rows = []
    files = sorted(f for f in SAMPLES.iterdir() if f.suffix.lower() in (".jpg", ".jpeg", ".png"))

    for f in files:
        img = imread_unicode(f)
        h, w = img.shape[:2]
        boxes = sorted(find_windows(img), key=lambda b: -b[2] * b[3])

        made = 0
        # --- 窗户候选（最多两个）---
        for i, (bx, by, bw, bh) in enumerate(boxes[:2]):
            half = max(bw, bh)            # 框外留一圈上下文，和人眼看的方式一致
            patch = _square(img, bx + bw / 2, by + bh / 2, half)
            if patch.size == 0:
                continue
            patch = cv2.resize(patch, (SIDE, SIDE), interpolation=cv2.INTER_AREA)
            out = CROPS / f"{f.stem}_w{i+1}.jpg"
            imwrite_unicode(out, patch, quality=95)
            rows.append(dict(image_id=f.stem, crop=str(out.relative_to(ROOT / "eval")) if str(out).startswith(str(ROOT / "eval")) else f"crops/{out.name}",
                             source=f.name, kind="window_candidate",
                             box=f"{bx},{by},{bw},{bh}",
                             scenario="", gt_is_window="", gt_window_state="",
                             gt_glare="", note=""))
            made += 1

        # --- 背景负例（离所有框都远的一块）---
        cx, cy = w // 2, int(h * 0.85)
        far_ok = all(abs(cx - (b[0] + b[2] / 2)) > w * 0.35 or
                     abs(cy - (b[1] + b[3] / 2)) > h * 0.35 for b in boxes) if boxes else True
        if far_ok:
            patch = _square(img, cx, cy, max(96, int(min(w, h) * 0.12)))
            if patch.size:
                patch = cv2.resize(patch, (SIDE, SIDE), interpolation=cv2.INTER_AREA)
                out = CROPS / f"{f.stem}_bg1.jpg"
                imwrite_unicode(out, patch, quality=95)
                rows.append(dict(image_id=f.stem, crop=str(out.relative_to(ROOT / "eval")) if str(out).startswith(str(ROOT / "eval")) else f"crops/{out.name}",
                                 source=f.name, kind="background",
                                 box="", scenario="", gt_is_window="",
                                 gt_window_state="", gt_glare="", note=""))
                made += 1

        # --- 整图样本（单独一类，用于"整图级能力"统计）---
        rows.append(dict(image_id=f.stem, crop=_crop_path(f),
                         source=f.name, kind="full_image",
                         box="", scenario="", gt_is_window="",
                         gt_window_state="", gt_glare="", note=""))

    with open(LABELS, "w", newline="", encoding="utf-8-sig") as fp:
        wtr = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print(f"生成 {len(rows)} 条样本 → {LABELS}")
    print(f"裁剪图 {len(list(CROPS.glob('*.jpg')))} 张 → {CROPS}")
    print("\n需要人工填的 4 列：")
    print("  scenario          outdoor/indoor/night/far/near/occluded 等")
    print("  gt_is_window      1=是窗户 0=不是（背景/墙面/天空都填 0）")
    print("  gt_window_state   open/closed/n/a")
    print("  gt_glare          yes/no/unknown/n/a")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    if LABELS.exists() and not a.force:
        print(f"{LABELS} 已存在，加 --force 覆盖（会丢掉你填的标注！）")
        sys.exit(1)
    build()
