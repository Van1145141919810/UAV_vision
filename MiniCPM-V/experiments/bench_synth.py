# -*- coding: utf-8 -*-
"""合成受控测试集 + 参数扫描。

为什么要合成：
    网上没有"远景窗户上红圆"的公开标注数据。而真实拍摄没法控制
    "红点占几个像素"，也就没法回答"多大才看得见"。
    合成可以精确控制像素数、模糊、压缩，真值 100% 可靠。

生成规则（尽量贴近真实成像）：
    背景（真实照片）→ 缩到 1920×1080 → 贴一个红色圆 → 高斯模糊（模拟离焦）
    → JPEG Q55（模拟图传压缩）

用法：
    python experiments/bench_synth.py            # 生成 + 扫描
    python experiments/bench_synth.py --keep     # 保留图片（默认也保留）
"""

import sys
import csv
import itertools
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mcpv.cv import imread_unicode, imwrite_unicode   # noqa: E402

SAMPLES = ROOT / "data" / "samples"
OUT = ROOT / "data" / "synthetic"
OUT.mkdir(parents=True, exist_ok=True)

BACKGROUNDS = ["neg1.jpg", "neg2.jpg", "neg3.jpg", "neg4.jpg", "neg5.jpg", "test1.jpg"]
DIAMETERS = [40, 28, 20, 14, 10, 7]          # 红点像素直径
CTRL = 0                                      # 0 = 不放红点（负样本对照）

# 候选落点（避开明显已经是红色的位置）
SPOTS = [(0.62, 0.42), (0.38, 0.55), (0.72, 0.30), (0.45, 0.65), (0.55, 0.25)]


def redness(img, cx, cy, r=60):
    b, g, rr = cv2.split(img.astype(np.int16))
    x0, y0 = max(0, cx - r), max(0, cy - r)
    patch = (rr - np.maximum(g, b))[y0:y0 + 2 * r, x0:x0 + 2 * r]
    return float(patch.mean())


def pick_spot(img):
    h, w = img.shape[:2]
    for fx, fy in SPOTS:
        cx, cy = int(w * fx), int(h * fy)
        if redness(img, cx, cy, 50) < 15:      # 这地方本来不红
            return cx, cy
    return int(w * 0.62), int(h * 0.42)


def make(background: str, d: int, rng_seed: int = 0):
    img = imread_unicode(SAMPLES / background)
    img = cv2.resize(img, (1920, 1080), interpolation=cv2.INTER_AREA)
    cx, cy = pick_spot(img)
    if d > 0:
        cv2.circle(img, (cx, cy), max(1, d // 2), (28, 28, 208), -1, lineType=cv2.LINE_AA)
        sigma = max(0.5, d / 8.0)              # 越小越糊（离焦 + 空气）
        img = cv2.GaussianBlur(img, (0, 0), sigma)
    name = f"{Path(background).stem}_d{d:02d}.jpg"
    path = OUT / name
    imwrite_unicode(path, img, quality=55)
    return name, path, (cx, cy), d


def loose_candidates(bgr):
    """松阈值：把"可能是红点"的全捞出来，带特征，供离线扫阈值用。"""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    b, g, r = cv2.split(bgr.astype(np.int16))
    mask = ((r - np.maximum(g, b)) > 20).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(mask, 8)
    yy, xx = np.ogrid[:h, :w]
    out = []
    for i in range(1, n):
        x, y, ww, hh, area = st[i]
        if area < 20 or area > 0.02 * w * h:
            continue
        sub = (lab[y:y + hh, x:x + ww] == i).astype(np.uint8)
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cnts:
            continue
        cnt = max(cnts, key=cv2.contourArea)
        per = cv2.arcLength(cnt, True)
        circ = 4 * np.pi * cv2.contourArea(cnt) / (per * per) if per > 0 else 0
        cx, cy = int(cen[i][0]), int(cen[i][1])
        de = float(np.sqrt(4 * area / np.pi))
        rad = max(3.0, de / 2)
        dist2 = (xx - cx) ** 2 + (yy - cy) ** 2
        ring = (dist2 >= (rad * 1.5) ** 2) & (dist2 <= (rad * 2.6) ** 2)
        outer = (dist2 >= (rad * 3) ** 2) & (dist2 <= (rad * 6) ** 2)
        rs = float(sat[ring].mean()) if ring.any() else 255
        rv = float(val[ring].mean()) if ring.any() else 0
        ov = float(val[outer].mean()) if outer.any() else 0
        out.append(dict(cx=cx, cy=cy, area=area, de=de, circ=circ,
                        ring_s=rs, ring_v=rv, outer_v=ov,
                        score=rv - 1.5 * rs, dscore=rv - ov))
    return out


def main():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:
            pass
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    rows = []
    print("生成合成集 ...")
    for bg in BACKGROUNDS:
        for d in [CTRL] + DIAMETERS:
            name, path, (tx, ty), dd = make(bg, d)
            bgr = imread_unicode(path)
            cands = loose_candidates(bgr)
            # 有没有一个候选落在真值附近（容差 = max(12, 2d)）
            tol = max(12, 2 * dd) if dd else 10 ** 9
            hit = any(abs(c["cx"] - tx) <= tol and abs(c["cy"] - ty) <= tol for c in cands)
            # 记录"离真值最近的那个候选"的特征（用于扫阈值）
            near = None
            if cands:
                near = min(cands, key=lambda c: (c["cx"] - tx) ** 2 + (c["cy"] - ty) ** 2)
                nd = ((near["cx"] - tx) ** 2 + (near["cy"] - ty) ** 2) ** 0.5
                if nd > tol:
                    near = None
            for c in cands:
                on = (abs(c["cx"] - tx) <= tol and abs(c["cy"] - ty) <= tol) if dd else False
                rows.append([bg, dd, name, c["cx"], c["cy"], round(c["de"], 1),
                             round(c["circ"], 3), round(c["ring_s"], 1),
                             round(c["ring_v"], 1), round(c["outer_v"], 1),
                             round(c["score"], 1), round(c["dscore"], 1),
                             int(on)])
        print(f"  {bg} 完成")

    csv_path = ROOT / "data" / "results" / "合成集候选.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["背景", "真值直径", "文件", "cx", "cy", "等效直径",
                    "圆度", "环带S", "环带V", "外圈V", "score", "dscore", "是否真点"])
        w.writerows(rows)
    print(f"\n候选特征已存: {csv_path}  (共 {len(rows)} 行)")

    # ---------- 离线扫阈值 ----------
    rules = {
        "R0 现行 score>=100 & V>=150":       lambda c: c[10] >= 100 and c[8] >= 150,
        "R1 现行 + 圆度>=0.6":                lambda c: c[10] >= 100 and c[8] >= 150 and c[6] >= 0.6,
        "R2 松 score>=60 & V>=110":          lambda c: c[10] >= 60 and c[8] >= 110,
        "R3 相对 dscore>=40 & S<=60":         lambda c: c[11] >= 40 and c[7] <= 60,
        "R4 相对 dscore>=25 & S<=80":         lambda c: c[11] >= 25 and c[7] <= 80,
        "R5 只要圆度>=0.6（不筛颜色底）":       lambda c: c[6] >= 0.6,
        "R6 全收（CV 上限）":                  lambda c: True,
    }

    print("\n" + "=" * 88)
    heads = ["规则"] + [f"{d}px" for d in DIAMETERS] + ["误报"]
    print(f"{heads[0]:<32}" + "".join(f"{h:>7}" for h in heads[1:]))
    print("-" * 88)
    best = None
    for rname, fn in rules.items():
        line = f"{rname:<32}"
        tot = 0
        for d in DIAMETERS:
            n = 0
            hit = 0
            for r in rows:
                if r[1] != d:
                    continue
                n += 1
                if fn(r) and r[12]:
                    hit += 1
            tot += hit
            line += f"{hit:>7}"
        # 误报：负样本对照图里被规则判成"有"的
        fp = 0
        for r in rows:
            if r[1] == 0 and fn(r):
                fp += 1
        line += f"{fp:>7}"
        print(line)
        if best is None or tot > best[0]:
            best = (tot, rname)
    print("-" * 88)
    print(f"各规则命中合计（不含误报）：最优 = {best[1]}")
    print("注：每格 = 该直径下 6 个背景里有几个找到了真点（满分 6）；"
          "误报列 = 6 张无点对照里命中规则的候选数")


if __name__ == "__main__":
    main()
