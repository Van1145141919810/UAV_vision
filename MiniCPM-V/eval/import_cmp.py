# -*- coding: utf-8 -*-
"""import_cmp.py —— 把 CMP 立面库转成评测样本（自带标注，不用人工标）

═══════════════════════════════════════════════════════════════════
为什么用它
───────────────────────────────────────────────────────────────────
CMP Facade DB 每张图都配一张**语义分割掩膜**，窗户是哪一类是标好的。
于是可以自动生成：
    正例 = 以窗户区域为中心裁 384×384
    负例 = 以立面/墙区域为中心裁 384×384
**真值免费、可复现、没有人工标注偏差。**

═══════════════════════════════════════════════════════════════════
局限（必须在报告里写清）
───────────────────────────────────────────────────────────────────
这是**欧洲街景立面**，正对拍摄、光照均匀、无遮挡、无无人机视角。
它与我们"中国宿舍楼 + 远距 + 树挡 + 逆光"差距很大。
所以只用来回答一个问题：
    **模型会不会换一种建筑风格就不认窗了？**（跨域泛化）
不能代表本项目场景的实测能力。

用法：
    python eval/import_cmp.py --per-class 25
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode, imwrite_unicode      # noqa: E402

CMP = ROOT / "data" / "external" / "cmp"
OUT_CROPS = ROOT / "eval" / "crops_cmp"
OUT_CSV = ROOT / "eval" / "labels_cmp.csv"
SIDE = 384

# CMP 掩膜的类别编号（见 CMP_facade_DB_2013.pdf）
CLS = {1: "facade", 2: "molding", 3: "cornice", 4: "pillar", 5: "window",
       6: "door", 7: "sills", 8: "blind", 9: "balcony", 10: "shop",
       11: "deco", 12: "stair", 13: "wall", 14: "car?"}
WINDOW_IDS = {3}          # 见 label_names.txt：3 = window
NONWIN_IDS = {1}          # 1=background（天空/街道）——立面类窗太密，做负例会混进窗


def find_pairs():
    """返回 [(图片路径, 掩膜路径)]。掩膜一般是 *_mask.png。"""
    pairs = []
    for img in CMP.rglob("*.jpg"):
        # CMP base: 掩膜是同名的 .png（如 cmp_b0001.jpg ↔ cmp_b0001.png）
        m = img.with_suffix(".png")
        if not m.exists():
            cand = list(img.parent.glob(img.stem + "*.png"))
            m = cand[0] if cand else None
        if m and m.exists():
            pairs.append((img, m))
    return sorted(pairs)


def largest_blob(mask_bool):
    """找最大连通域，返回 (cx, cy, 面积占比)。"""
    m = (mask_bool.astype(np.uint8)) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    area = st[i, cv2.CC_STAT_AREA]
    return int(cen[i][0]), int(cen[i][1]), area / m.size


def center_ratio(mk, cx, cy, half, ids):
    """以 (cx,cy) 为中心、边长 2*half 的方块里，属于 ids 类别的像素占比。"""
    h, w = mk.shape[:2]
    x0 = max(0, min(w - 2 * half, cx - half))
    y0 = max(0, min(h - 2 * half, cy - half))
    sub = mk[y0:y0 + 2 * half, x0:x0 + 2 * half]
    if sub.size == 0:
        return 0.0
    return float(np.isin(sub, list(ids)).mean())


def crop_at(img, cx, cy, half):
    h, w = img.shape[:2]
    x0 = max(0, min(w - 2 * half, cx - half))
    y0 = max(0, min(h - 2 * half, cy - half))
    p = img[y0:y0 + 2 * half, x0:x0 + 2 * half]
    if p.size == 0:
        return None
    return cv2.resize(p, (SIDE, SIDE), interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-class", type=int, default=25, help="每类最多取几张")
    args = ap.parse_args()

    pairs = find_pairs()
    if not pairs:
        raise SystemExit(f"没找到图片/掩膜配对，检查 {CMP}")
    print(f"找到 {len(pairs)} 组图+掩膜")

    # 先看一张掩膜的类别取值，确认假设
    _, m0 = pairs[0]
    from PIL import Image
    mk = np.array(Image.open(str(m0)))      # mode P → 直接拿到类别索引
    vals = sorted(set(np.unique(mk).tolist()))
    print(f"掩膜里出现的类别值：{vals[:20]}{' …' if len(vals) > 20 else ''}")
    print("  对照：" + ", ".join(f"{v}={CLS.get(v, '?')}" for v in vals[:14]))

    OUT_CROPS.mkdir(parents=True, exist_ok=True)
    rows, got = [], {1: 0, 0: 0}

    for img_p, mask_p in pairs:
        img = imread_unicode(img_p)
        from PIL import Image
        mk = np.array(Image.open(str(mask_p)))
        if img is None or mk is None:
            continue
        if mk.shape[:2] != img.shape[:2]:
            mk = cv2.resize(mk, (img.shape[1], img.shape[0]),
                            interpolation=cv2.INTER_NEAREST)
        h, w = img.shape[:2]
        base_half = max(48, int(min(h, w) * 0.16))

        # 正例：最大窗户连通域
        if got[1] < args.per_class:
            b = largest_blob(np.isin(mk, list(WINDOW_IDS)))
            if b and b[2] > 0.004 and \
               center_ratio(mk, b[0], b[1], base_half, WINDOW_IDS) >= 0.35:
                p = crop_at(img, b[0], b[1], base_half)
                if p is not None:
                    name = f"cmp_{img_p.stem}_win.jpg"
                    imwrite_unicode(OUT_CROPS / name, p, quality=95)
                    rows.append(dict(image_id=img_p.stem,
                                     crop=f"crops_cmp/{name}", source=str(img_p.name),
                                     kind="cmp_window", box=f"{b[0]},{b[1]}",
                                     scenario="cmp_european_street",
                                     gt_is_window="1", gt_window_state="closed",
                                     gt_glare="unknown",
                                     note="CMP自带分割标注：窗户"))
                    got[1] += 1

        # 负例：立面/墙
        if got[0] < args.per_class:
            b = largest_blob(np.isin(mk, list(NONWIN_IDS)))
            if b and b[2] > 0.10 and \
               center_ratio(mk, b[0], b[1], base_half, WINDOW_IDS) < 0.05:
                p = crop_at(img, b[0], b[1], base_half)
                if p is not None:
                    name = f"cmp_{img_p.stem}_wall.jpg"
                    imwrite_unicode(OUT_CROPS / name, p, quality=95)
                    rows.append(dict(image_id=img_p.stem,
                                     crop=f"crops_cmp/{name}", source=str(img_p.name),
                                     kind="cmp_wall", box=f"{b[0]},{b[1]}",
                                     scenario="cmp_european_street",
                                     gt_is_window="0", gt_window_state="n/a",
                                     gt_glare="unknown",
                                     note="CMP自带分割标注：立面/墙"))
                    got[0] += 1

        if got[1] >= args.per_class and got[0] >= args.per_class:
            break

    if not rows:
        raise SystemExit("没生成任何样本（类别编号可能不同，看上面打印的掩膜取值）")

    with open(OUT_CSV, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\n生成 正例 {got[1]} / 负例 {got[0]} → {OUT_CSV}")
    print(f"裁剪图在 {OUT_CROPS}")


if __name__ == "__main__":
    main()
