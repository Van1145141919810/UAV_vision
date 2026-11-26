# -*- coding: utf-8 -*-
"""量"黄框 vs 绿十字"偏差 —— 用精确颜色 + 形状约束，并用检测到的红点做参照

叠加颜色是纯色绘制，核心像素精确等于：
    绿十字 / 选中框 (80, 240, 90)
    候选框        (60, 220, 255)
所以用 tol=12 精确匹配，再按形状区分：
    十字：bbox 15~40 px、面积 40~400、填充率低
    框  ：bbox 更大、空心
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
OUT = ROOT / "eval" / "figures"
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imwrite_unicode                       # noqa: E402
from mcpv.detect_live import detect_reds                  # noqa: E402

GREEN = (80, 240, 90)
YELLOW = (60, 220, 255)
TOP = 52


def mask_exact(bgr, color, tol=12):
    d = np.linalg.norm(bgr.astype(np.int16) - np.array(color, np.int16), axis=2)
    m = (d <= tol).astype(np.uint8)
    m[:TOP, :] = 0
    return m


def comps(m, min_area=25):
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, a = (int(st[i][0]), int(st[i][1]), int(st[i][2]),
                         int(st[i][3]), int(st[i][4]))
        if a < min_area:
            continue
        out.append({"c": (float(cen[i][0]), float(cen[i][1])), "box": (x, y, w, h),
                    "area": a, "fill": a / float(w * h)})
    return out


def pick_cross(cs):
    """十字：bbox 15~40、填充率 < 0.6（两条细线交叉）。"""
    best = None
    for b in cs:
        x, y, w, h = b["box"]
        if not (14 <= w <= 42 and 14 <= h <= 42):
            continue
        if b["fill"] > 0.65:
            continue
        if best is None or b["area"] > best["area"]:
            best = b
    return best


def main():
    cap = cv2.VideoCapture(str(VID))
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    rows, worst = [], []
    for i in range(total):
        ok, fr = cap.read()
        if not ok:
            break
        green = comps(mask_exact(fr, GREEN))
        yellow = comps(mask_exact(fr, YELLOW), min_area=60)
        cross = pick_cross(green)
        dot = None
        cands = detect_reds(fr)
        if cands:
            dot = (cands[0]["cx"], cands[0]["cy"])
        rec = {"frame": i, "t": round(i / fps, 2),
               "cross": cross["c"] if cross else None,
               "dot": dot, "n_yel": len(yellow),
               "d_cross_dot": None, "d_cross_yel": None}
        if cross and dot:
            rec["d_cross_dot"] = round(float(np.hypot(
                cross["c"][0] - dot[0], cross["c"][1] - dot[1])), 1)
        if cross and yellow:
            ds = [float(np.hypot(cross["c"][0] - b["c"][0],
                                 cross["c"][1] - b["c"][1])) for b in yellow]
            rec["d_cross_yel"] = round(min(ds), 1)
            worst.append((min(ds), i, cross["c"], dot))
        rows.append(rec)
    cap.release()

    cr = [r for r in rows if r["cross"]]
    cd = [r for r in cr if r["d_cross_dot"] is not None]
    cy = [r for r in cr if r["d_cross_yel"] is not None]
    print(f"总帧 {total}")
    print(f"  识别到绿十字: {len(cr)} 帧")
    print(f"  其中同时有红点: {len(cd)} 帧   同时有黄框: {len(cy)} 帧")
    if cd:
        d = np.array([r["d_cross_dot"] for r in cd])
        print(f"\n【绿十字 与 检出的红点中心】偏差：")
        print(f"  中位 {np.median(d):5.1f}  90分位 {np.percentile(d,90):5.1f}  "
              f"最大 {d.max():5.1f} px")
        for th in (3, 5, 10, 20, 40):
            n = int((d > th).sum())
            print(f"    > {th:3d} px : {n:4d} 帧 ({n/len(cd)*100:5.1f}%)")
    if cy:
        d2 = np.array([r["d_cross_yel"] for r in cy])
        print(f"\n【绿十字 与 最近黄框中心】偏差：")
        print(f"  中位 {np.median(d2):5.1f}  90分位 {np.percentile(d2,90):5.1f}  "
              f"最大 {d2.max():5.1f} px")
        for th in (3, 5, 10, 20, 40):
            n = int((d2 > th).sum())
            print(f"    > {th:3d} px : {n:4d} 帧 ({n/len(cy)*100:5.1f}%)")
    worst.sort(reverse=True)
    print("\n黄框偏差最大 5 帧：")
    for dd, i, gc, dt in worst[:5]:
        print(f"  帧{i:4d} 偏差{dd:7.1f} 绿({gc[0]:.0f},{gc[1]:.0f}) "
              f"红点({dt[0]:.0f},{dt[1]:.0f})" if dt else "")
    (OUT / "offset_stats.json").write_text(
        json.dumps(rows, ensure_ascii=False), encoding="utf-8")
    print(f"\n逐帧明细: {OUT / 'offset_stats.json'}")


if __name__ == "__main__":
    main()
