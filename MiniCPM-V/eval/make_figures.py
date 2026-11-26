# -*- coding: utf-8 -*-
"""make_figures.py —— 生成汇报用的插图（一次性跑完）

产出（都放 eval/figures/）：
  fig_resolution.jpg   同一扇窗缩到不同像素 → 模型"看到的"差别（对应 R01）
  fig_success_fail.jpg 判对 / 判错的样本对照
  fig_cmp_clean.jpg    客观真值样本（中心窗占比 0.7 / 0.0 两档）
  fig_segformer.jpg    SegFormer 预测 vs 人工标注（验证"换前端"）
  fig_phone.jpg        手机实拍样本拼版
"""
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode      # noqa: E402

EVAL = ROOT / "eval"
FIG = EVAL / "figures"
FIG.mkdir(exist_ok=True)
CELL = 210


def tile(files, n, cols=6, cap=None, color=(0, 255, 255)):
    files = files[:n]
    rows = (len(files) + cols - 1) // cols
    sheet = np.full((rows * (CELL + 22), cols * CELL, 3), 28, np.uint8)
    for i, f in enumerate(files):
        r, c = divmod(i, cols)
        im = imread_unicode(f) if isinstance(f, Path) else f
        if im is None:
            continue
        im = cv2.resize(im, (CELL - 4, CELL - 4), interpolation=cv2.INTER_AREA)
        y, x = r * (CELL + 22), c * CELL
        sheet[y + 2:y + 2 + im.shape[0], x + 2:x + 2 + im.shape[1]] = im
        if cap:
            cv2.putText(sheet, cap[i][:26], (x + 4, y + CELL + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return sheet


# ── 1. 分辨率阶梯 ────────────────────────────────────────────
def fig_resolution():
    src = EVAL / "crops" / "win08_w1.jpg"
    img = imread_unicode(src)
    if img is None:
        return
    sizes = [384, 96, 48, 24, 12]
    dist = [8, 33, 65, 131, 261]
    tiles = []
    for s, d in zip(sizes, dist):
        small = cv2.resize(img, (s, s), interpolation=cv2.INTER_AREA)
        up = cv2.resize(small, (CELL, CELL), interpolation=cv2.INTER_NEAREST)
        cv2.putText(up, f"{s}px  ~{d}m", (6, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        tiles.append(up)
    imwrite_unicode(FIG / "fig_resolution.jpg", np.hstack(tiles), quality=93)
    print("fig_resolution.jpg")


# ── 2. 判对 / 判错对照 ───────────────────────────────────────
def fig_success_fail():
    csvs = sorted((EVAL / "results").glob("*.csv"),
                  key=lambda p: p.stat().st_mtime, reverse=True)
    csvs = [c for c in csvs if "open" in c.name or True]
    if not csvs:
        return
    rows = list(csv.DictReader(open(csvs[0], encoding="utf-8-sig")))
    ok, bad = [], []
    for r in rows:
        if r["gt_is_window"] not in ("0", "1"):
            continue
        p = (EVAL / r["crop"]).resolve()
        if not p.exists():
            continue
        hit = (r["pred_is_window"] == r["gt_is_window"])
        g = "窗" if r["gt_is_window"] == "1" else "非窗"
        pr = {"1": "窗", "0": "非窗"}.get(r["pred_is_window"], "?")
        (ok if hit else bad).append((p, f"{'✓' if hit else '✗'} 真:{g} 判:{pr}"))
    a = tile([x[0] for x in ok], 6, cap=[x[1] for x in ok], color=(80, 220, 80))
    b = tile([x[0] for x in bad], 6, cap=[x[1] for x in bad], color=(60, 60, 240))
    hdr1 = np.full((30, a.shape[1], 3), 40, np.uint8)
    cv2.putText(hdr1, "判断正确", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 220, 80), 2)
    hdr2 = np.full((30, b.shape[1], 3), 40, np.uint8)
    cv2.putText(hdr2, "判断错误", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 240), 2)
    imwrite_unicode(FIG / "fig_success_fail.jpg",
                    np.vstack([hdr1, a, hdr2, b]), quality=92)
    print("fig_success_fail.jpg")


# ── 3. 客观真值样本 ─────────────────────────────────────────
def fig_cmp_clean():
    rows = list(csv.DictReader(open(EVAL / "labels_clean.csv", encoding="utf-8-sig")))
    pos = [r for r in rows if r["gt_is_window"] == "1"][:6]
    neg = [r for r in rows if r["gt_is_window"] == "0"][:6]
    a = tile([(EVAL / r["crop"]).resolve() for r in pos], 6,
             cap=[f"窗占比 {r['win_frac']}" for r in pos], color=(80, 220, 80))
    b = tile([(EVAL / r["crop"]).resolve() for r in neg], 6,
             cap=[f"窗占比 {r['win_frac']}" for r in neg], color=(60, 60, 240))
    hdr1 = np.full((30, a.shape[1], 3), 40, np.uint8)
    cv2.putText(hdr1, "客观真值：是窗户（中心窗占比 >= 0.60）", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 80), 2)
    hdr2 = np.full((30, b.shape[1], 3), 40, np.uint8)
    cv2.putText(hdr2, "客观真值：不是窗户（<= 0.05）", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 240), 2)
    imwrite_unicode(FIG / "fig_cmp_clean.jpg",
                    np.vstack([hdr1, a, hdr2, b]), quality=92)
    print("fig_cmp_clean.jpg")


# ── 4. SegFormer 预测 vs 标注 ───────────────────────────────
def fig_segformer():
    try:
        import torch
        from PIL import Image
        from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation
    except Exception as e:
        print("跳过 segformer 图:", e)
        return
    mid = "Xpitfire/segformer-finetuned-segments-cmp-facade"
    proc = AutoImageProcessor.from_pretrained(mid)
    model = AutoModelForSemanticSegmentation.from_pretrained(mid).eval()
    wid = [i for i, n in model.config.id2label.items() if "window" in str(n).lower()][0]

    base = ROOT / "data" / "external" / "cmp" / "base"
    picks = [p for p in sorted(base.glob("*.jpg"))[:6]]
    tiles = []
    for p in picks:
        mp = p.with_suffix(".png")
        if not mp.exists():
            continue
        img = Image.open(p).convert("RGB")
        gt = np.array(Image.open(mp))
        with torch.no_grad():
            logits = model(**proc(images=img, return_tensors="pt")).logits
        logits = torch.nn.functional.interpolate(
            logits, size=gt.shape, mode="bilinear", align_corners=False)
        pred = logits.argmax(1)[0].numpy()

        rgb = np.array(img)[:, :, ::-1].copy()
        overlay_gt = rgb.copy()
        overlay_gt[gt == 3] = (0, 220, 0)
        overlay_pr = rgb.copy()
        overlay_pr[pred == wid] = (0, 120, 255)
        row = np.hstack([cv2.resize(x, (200, 130)) for x in (rgb, overlay_gt, overlay_pr)])
        tiles.append(row)
    if tiles:
        sheet = np.vstack(tiles)
        cv2.putText(sheet, "原图 | 人工标注(绿) | SegFormer预测(橙)", (6, sheet.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        imwrite_unicode(FIG / "fig_segformer.jpg", sheet, quality=92)
        print("fig_segformer.jpg")


# ── 5. 手机实拍拼版 ─────────────────────────────────────────
def fig_phone():
    d = EVAL / "crops_shot"
    fs = sorted(d.glob("*.jpg"))[:12]
    if fs:
        sheet = tile(fs, 12, cols=6)
        imwrite_unicode(FIG / "fig_phone.jpg", sheet, quality=92)
        print("fig_phone.jpg")


if __name__ == "__main__":
    fig_resolution()
    fig_success_fail()
    fig_cmp_clean()
    fig_phone()
    fig_segformer()
