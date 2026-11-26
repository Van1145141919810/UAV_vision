# -*- coding: utf-8 -*-
"""bench_grid.py —— 几何网格校正：用"窗户在立面上成规则网格"这条先验修正分割结果

═══════════════════════════════════════════════════════════════════
为什么值得做（不训练、不花钱）
───────────────────────────────────────────────────────────────────
我们实测分割在宿舍楼立面上一张图出 50~59 个框，比真实窗户多，
且会漏掉一部分。但**窗户有很强的几何先验**：

    · 同一层的窗，高度几乎相同，y 坐标对齐
    · 同一列/同一行的窗，水平间距有规律

于是可以做三件事：
    1. **去误报**：不在任何一行上的孤立框 → 丢
    2. **同行一致性**：高度离本行中位数太远的框 → 丢
    3. **补漏检**：一行里间距明显偏大的缺口 → 插一个候选

═══════════════════════════════════════════════════════════════════
评价方式
───────────────────────────────────────────────────────────────────
自有照片没有像素级真值，所以用两个可客观计算的指标：
  · 框数变化（越多≠越好，我们要的是收敛到合理数量）
  · **行内高度离散度**（同层窗高度应接近；离散度下降说明过滤有效）

另外出拼版图给人看：红=被丢掉的框，绿=保留，蓝=补出来的。

用法：python eval/bench_grid.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode        # noqa: E402

SEG_ID = "Xpitfire/segformer-finetuned-segments-cmp-facade"
FILES = sorted((ROOT / "data" / "samples").glob("win0*.jpg")) + \
        sorted((ROOT / "data" / "samples").glob("win1*.jpg"))
MIN_AREA = 1200
CELL = 380


# ── 网格校正核心 ────────────────────────────────────────────
def cluster_rows(boxes, tol_ratio=0.55):
    """按 y 中心把框聚成"行"。"""
    if not boxes:
        return []
    hs = [b[3] for b in boxes]
    med_h = float(np.median(hs))
    boxes = sorted(boxes, key=lambda b: b[1] + b[3] / 2)
    rows, cur = [], [boxes[0]]
    for b in boxes[1:]:
        cy = b[1] + b[3] / 2
        cy_prev = cur[-1][1] + cur[-1][3] / 2
        if abs(cy - cy_prev) <= tol_ratio * med_h:
            cur.append(b)
        else:
            rows.append(cur)
            cur = [b]
    rows.append(cur)
    return rows


def grid_filter(boxes, h_tol=0.40, min_row=2):
    """去误报：孤立行、行内高度离群、中心线离群。"""
    rows = cluster_rows(boxes)
    keep, drop = [], []
    for r in rows:
        if len(r) < min_row:
            drop += r                     # 单框成不了一行 → 孤立，丢
            continue
        med_h = float(np.median([b[3] for b in r]))
        med_cy = float(np.median([b[1] + b[3] / 2 for b in r]))
        for b in r:
            h, cy = b[3], b[1] + b[3] / 2
            if abs(h - med_h) / med_h > h_tol or abs(cy - med_cy) > 0.45 * med_h:
                drop.append(b)
            else:
                keep.append(b)
    return keep, drop, rows


def grid_complete(keep, max_gap_ratio=1.9):
    """补漏检：同一行里间距明显偏大处插一个候选框。"""
    rows = cluster_rows(keep)
    added = []
    for r in rows:
        if len(r) < 3:
            continue
        r = sorted(r, key=lambda b: b[0])
        gaps = [r[i + 1][0] - (r[i][0] + r[i][2]) for i in range(len(r) - 1)]
        gaps = [g for g in gaps if g > 0]
        if not gaps:
            continue
        med_gap = float(np.median(gaps))
        med_w = int(np.median([b[2] for b in r]))
        med_h = int(np.median([b[3] for b in r]))
        for i in range(len(r) - 1):
            g = r[i + 1][0] - (r[i][0] + r[i][2])
            if med_gap > 0 and g > max_gap_ratio * med_gap:
                n_slot = int(round(g / (med_w + med_gap))) - 1
                for k in range(max(0, min(n_slot, 2))):
                    x = r[i][0] + r[i][2] + int(med_gap) + k * (med_w + int(med_gap))
                    added.append((x, r[i][1], med_w, med_h))
    return added


def height_dispersion(boxes):
    """行内高度离散度（越小越整齐）。"""
    rows = [r for r in cluster_rows(boxes) if len(r) >= 2]
    vals = []
    for r in rows:
        hs = np.array([b[3] for b in r], float)
        if hs.mean() > 0:
            vals.append(hs.std() / hs.mean())
    return float(np.mean(vals)) if vals else None


def main():
    import torch
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForSemanticSegmentation

    print("加载 SegFormer …")
    proc = AutoImageProcessor.from_pretrained(SEG_ID)
    seg = AutoModelForSemanticSegmentation.from_pretrained(SEG_ID).eval()
    wid = [i for i, n in seg.config.id2label.items() if "window" in str(n).lower()][0]

    tiles = []
    print(f"{'图':<12}{'原框':<6}{'保留':<6}{'丢弃':<6}{'补':<4}{'行数':<6}{'离散度(前→后)'}")
    print("-" * 68)
    for p in FILES[:8]:
        img = imread_unicode(p)
        if img is None:
            continue
        pil = Image.fromarray(img[:, :, ::-1])
        with torch.no_grad():
            logits = seg(**proc(images=pil, return_tensors="pt")).logits
        logits = torch.nn.functional.interpolate(
            logits, size=(img.shape[0], img.shape[1]), mode="bilinear",
            align_corners=False)
        mask = (logits.argmax(1)[0].numpy() == wid).astype(np.uint8)

        n, lab, st, cen = cv2.connectedComponentsWithStats(mask, 8)
        boxes = [(int(x), int(y), int(w), int(h))
                 for i in range(1, n)
                 for x, y, w, h, a in [st[i]] if a >= MIN_AREA]

        keep, drop, rows = grid_filter(boxes)
        added = grid_complete(keep)
        d0 = height_dispersion(boxes)
        d1 = height_dispersion(keep)

        vis = img.copy()
        for (x, y, w, h) in drop:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (60, 60, 235), 3)
        for (x, y, w, h) in keep:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (80, 220, 80), 3)
        for (x, y, w, h) in added:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (240, 180, 60), 3)

        print(f"{p.stem:<12}{len(boxes):<6}{len(keep):<6}{len(drop):<6}"
              f"{len(added):<4}{len(rows):<6}"
              f"{('%.2f' % d0) if d0 else '—'} → {('%.2f' % d1) if d1 else '—'}")

        t = cv2.resize(vis, (CELL, int(CELL * img.shape[0] / img.shape[1])))
        cv2.putText(t, f"{p.stem}  {len(boxes)}→{len(keep)}+{len(added)}",
                    (6, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        tiles.append(t)

    if not tiles:
        raise SystemExit("没有可用图")
    H = CELL
    tiles = [cv2.resize(t, (int(t.shape[1] * H / t.shape[0]), H)) for t in tiles]
    rows = []
    per = 4
    for i in range(0, len(tiles), per):
        row = tiles[i:i + per]
        while len(row) < per:
            row.append(np.zeros_like(row[0]))
        rows.append(np.hstack(row))
    sheet = np.vstack(rows)
    out = ROOT / "eval" / "figures" / "grid_correct.jpg"
    out.parent.mkdir(exist_ok=True)
    imwrite_unicode(out, sheet, quality=90)
    print(f"\n绿=保留  红=丢弃  蓝=补出\n拼版 {out}")


if __name__ == "__main__":
    main()
