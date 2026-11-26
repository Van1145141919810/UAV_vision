# -*- coding: utf-8 -*-
"""特征评估：对比"绝对红度" vs "归一化色度 + 局部色度对比"。

为什么要加这两个特征：
    1) 绝对红度 R-max(G,B) 依赖曝光。反光/过曝时红点会变粉白，直接掉阈值。
       归一化色度 (R-max(G,B))/(R+G+B+1) 对整体明暗不敏感。
    2) 远处/杂乱背景下，"和周围不一样"比"它有多红"更可靠。
       局部色度对比 cdiff = 点内平均色度 - 环带平均色度。

评估集：
    · data/samples     8 张真实图（2 正 6 负）
    · 合成集            6 背景 × 6 直径 + 6 对照（由 bench_synth 现场重生成，位置已知）

用法：
    python experiments/bench_features.py
"""

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "experiments"))

from mcpv.cv import imread_unicode                      # noqa: E402
from bench_synth import make, BACKGROUNDS, DIAMETERS    # noqa: E402

SAMPLES = ROOT / "data" / "samples"

REAL = {
    "test1.jpg": (False, None),
    "test2.jpg": (True, (879, 770)),
    "test3.jpg": (True, (882, 692)),
    "neg1.jpg": (False, None), "neg2.jpg": (False, None),
    "neg3.jpg": (False, None), "neg4.jpg": (False, None),
    "neg5.jpg": (False, None),
}


def features(bgr):
    h, w = bgr.shape[:2]
    b, g, r = cv2.split(bgr.astype(np.float32))
    s = r + g + b + 1.0
    chroma = (r - np.maximum(g, b)) / s
    mask = (chroma > 0.18).astype(np.uint8) * 255
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
        if per <= 0:
            continue
        circ = 4 * np.pi * cv2.contourArea(cnt) / (per * per)
        if circ < 0.55:
            continue
        cx, cy = int(cen[i][0]), int(cen[i][1])
        de = float(np.sqrt(4 * area / np.pi))
        rad = max(3.0, de / 2)
        d2 = (xx - cx) ** 2 + (yy - cy) ** 2
        inside = d2 <= (rad * 0.8) ** 2
        ring = (d2 >= (rad * 1.6) ** 2) & (d2 <= (rad * 2.8) ** 2)
        c_in = float(chroma[inside].mean()) if inside.any() else 0.0
        c_rg = float(chroma[ring].mean()) if ring.any() else 0.0
        out.append(dict(cx=cx, cy=cy, de=de, circ=circ, area=area,
                        cin=round(c_in, 3), crg=round(c_rg, 3),
                        cdiff=round(c_in - c_rg, 3)))
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

    # ---------- 真实 8 张 ----------
    print("=== 真实 8 张：按 cdiff 排序的前三 ===")
    print(f"{'图':<11}{'真值':<5}{'候选':<5}{'前三 (circ / cdiff / 坐标)'}")
    print("-" * 78)
    hit = tot = 0
    for name, (has, dot) in REAL.items():
        img = imread_unicode(SAMPLES / name)
        cs = sorted(features(img), key=lambda c: -c["cdiff"])[:3]
        desc = " | ".join(f"{c['circ']:.2f}/{c['cdiff']:+.2f}@({c['cx']},{c['cy']})" for c in cs)
        print(f"{name:<11}{'有' if has else '无':<5}{len(cs):<5}{desc[:62]}")
        if dot:
            tot += 1
            if any(abs(c["cx"] - dot[0]) < 70 and abs(c["cy"] - dot[1]) < 70 for c in cs):
                hit += 1
    print(f"\n真实正样本：真点进前三 {hit}/{tot}")

    # ---------- 合成集 ----------
    print("\n=== 合成集：cdiff 门限扫描 ===")
    print("（每格 = 6 个背景里几个找到真点；误报 = 6 张对照图里过门限的候选数）")
    data = []
    for bg in BACKGROUNDS:
        for d in [0] + DIAMETERS:
            _, path, (tx, ty), dd = make(bg, d)
            data.append((bg, dd, imread_unicode(path), tx, ty))

    print(f"\n{'cdiff':<8}" + "".join(f"{d:>7}px" for d in DIAMETERS) + f"{'误报':>8}")
    print("-" * 62)
    for thr in [0.00, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35]:
        line = f"{thr:<8.2f}"
        for d in DIAMETERS:
            n = 0
            for bg, dd, img, tx, ty in data:
                if dd != d:
                    continue
                ok = [c for c in features(img) if c["cdiff"] >= thr]
                tol = max(12, 2 * d)
                if any(abs(c["cx"] - tx) <= tol and abs(c["cy"] - ty) <= tol for c in ok):
                    n += 1
            line += f"{n:>7}"
        fp = 0
        for bg, dd, img, tx, ty in data:
            if dd == 0:
                fp += sum(1 for c in features(img) if c["cdiff"] >= thr)
        line += f"{fp:>8}"
        print(line)


if __name__ == "__main__":
    main()
