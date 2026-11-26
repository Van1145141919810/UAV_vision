# -*- coding: utf-8 -*-
"""grid_win.py —— 路 B：LSD 直线 + 网格，找「玻璃幕墙 / 规整窗」的框

为什么不用 Canny 轮廓
──────────────────────────────────────────────────────────────
玻璃幕墙是**连续一大片浅色玻璃 + 细框线**，没有"一扇窗"的闭合轮廓，
Canny + 轮廓在 40.8 m 上实测 0/4（34 组参数都不行）。
幕墙的几何本质是**两组互相垂直的直线构成网格** → 用直线检测更对口。

做法
  1. LSD 出所有线段（亚像素级）
  2. 按角度分横/竖两组（±25° 内算准）
  3. 把竖线的 x、横线的 y 投影成一维直方图 → 峰 = 网格线
  4. 相邻网格线围出的格子 = 一个窗格
  5. 网格的外接矩形 = 幕墙/窗的大框
  6. 按尺寸/密度过滤

对外两个入口
  cells(img)  -> [ (x,y,w,h), ... ]  窗格框（从小到大）
  region(img) -> (x,y,w,h) 或 None   整片网格的外接框（给"大概位置"用）
"""
from __future__ import annotations

import cv2
import numpy as np


def _lines(img, min_len=18, max_gap=3):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.createCLAHE(3.0, (8, 8)).apply(g)
    g = cv2.GaussianBlur(g, (3, 3), 0)
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    out = lsd.detect(g)[0]
    if out is None:
        return [], []
    H, W = g.shape[:2]
    hor, ver = [], []
    diag = float(np.hypot(H, W))
    for l in out.reshape(-1, 4):
        x1, y1, x2, y2 = map(float, l)
        L = float(np.hypot(x2 - x1, y2 - y1))
        if L < min_len or L > 0.98 * diag:
            continue
        ang = float(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
        if ang < 25 or ang > 155:                 # 近水平
            hor.append((min(x1, x2), max(x1, x2), (y1 + y2) / 2.0, L))
        elif 65 < ang < 115:                      # 近竖直
            ver.append((min(y1, y2), max(y1, y2), (x1 + x2) / 2.0, L))
    return hor, ver


def _peaks(vals, weights, W, min_gap, min_w):
    """一维加权直方图找峰。返回 [(位置, 权重), ...]。"""
    if not vals:
        return []
    hist = np.zeros(W, dtype=np.float64)
    for v, w in zip(vals, weights):
        i = int(round(v))
        if 0 <= i < W:
            hist[i] += w
    hist = cv2.GaussianBlur(hist.reshape(1, -1), (1, 9), 0).ravel()
    out = []
    thr = max(min_w, float(hist.max()) * 0.12)
    i = 0
    while i < W:
        if hist[i] >= thr:
            j = i
            while j + 1 < W and hist[j + 1] >= thr:
                j += 1
            k = int(i + np.argmax(hist[i:j + 1]))
            out.append((k, float(hist[k])))
            i = j + 1 + min_gap
        else:
            i += 1
    return out


def _group(ps, gap):
    """把相邻峰连成组（中间没有明显断档的算同一片网格）。"""
    groups, cur = [], []
    for p in ps:
        if cur and p[0] - cur[-1][0] > gap:
            groups.append(cur)
            cur = []
        cur.append(p)
    if cur:
        groups.append(cur)
    return groups


def grid(img, min_len=18, x_gap=160, y_gap=120, min_cell=12, max_cells=120,
         roi_top=0.58, min_vert=3, pitch_cv=0.40, min_vlen=40):
    """roi_top: 只搜画面上部（地平线以上；地面铺装线会把网格检测淹掉）
    min_vert: 一片网格至少要有几条竖线
    pitch_cv: 竖线间距的离散度上限（真幕墙是等间距的，地砖不是）
    min_vlen: 竖线最短长度（幕墙竖框是贯通的，地面的短边不算）"""
    H, W = img.shape[:2]
    hor, ver = _lines(img, min_len=min_len)
    y_lim = H * roi_top
    hor = [h for h in hor if h[2] < y_lim]
    ver = [v for v in ver if v[0] < y_lim and v[1] < y_lim and (v[1] - v[0]) >= min_vlen]
    px = _peaks([v[2] for v in ver], [v[3] for v in ver], W, 4, 24)
    py = _peaks([h[2] for h in hor], [h[3] for h in hor], H, 4, 24)
    cells, regions = [], []
    for gx in _group(px, x_gap):
        if len(gx) < min_vert:
            continue
        xs = sorted(p[0] for p in gx)
        dx = np.diff(xs)
        if len(dx) == 0:
            continue
        cv_ = float(np.std(dx) / max(np.mean(dx), 1e-6))
        if cv_ > pitch_cv:
            continue
        for gy in _group(py, y_gap):
            if len(gy) < 2:
                continue
            ys = sorted(p[0] for p in gy)
            if min(ys) > y_lim:            # 这片横线全在地平线以下 → 地砖
                continue
            regions.append((min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)))
            for i in range(len(xs) - 1):
                for j in range(len(ys) - 1):
                    w = xs[i + 1] - xs[i]
                    h = ys[j + 1] - ys[j]
                    if w >= min_cell and h >= min_cell and w >= 8 and h >= 8:
                        cells.append((xs[i], ys[j], w, h))
    cells.sort(key=lambda b: -b[2] * b[3])
    cells = cells[:max_cells]
    regions.sort(key=lambda b: -b[2] * b[3])
    return cells, regions


def cells_of(img, **kw):
    return grid(img, **kw)[0]


def region_of(img, **kw):
    r = grid(img, **kw)[1]
    return r[0] if r else None


if __name__ == "__main__":
    import sys
    sys.path.insert(0, r"D:\Desktop\无人系统\MiniCPM-V\eval")
    import bench_lat_crop as L

    fr = L.load_frames()
    picks = L.pick_by_d(fr, [8, 15, 30, 55], tol=0.2)
    print("=== 路 B: LSD + 网格 ===")
    for k, f in picks:
        img = f["clean"]
        cs, rs = grid(img)
        gx, gy = f["gx"], f["gy"]
        hit = [b for b in cs if b[0] - 6 <= gx <= b[0] + b[2] + 6 and b[1] - 6 <= gy <= b[1] + b[3] + 6]
        reg_hit = [b for b in rs if b[0] <= gx <= b[0] + b[2] and b[1] <= gy <= b[1] + b[3]]
        dist = min([np.hypot(b[0] + b[2] / 2 - gx, b[1] + b[3] / 2 - gy) for b in cs], default=9e9)
        print("%-5d %5.1fm 窗格%3d 片 | 含目标 %d | 幕墙大框 %2d 片 含目标 %d | 最近格中心 %.0f px"
              % (k, L.FX * L.DOT_D / f["d"], len(cs), len(hit), len(rs), len(reg_hit), dist))
        vis = img.copy()
        for (x, y, w, h) in cs:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 1)
        for (x, y, w, h) in rs:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 128, 0), 3)
        cv2.circle(vis, (int(gx), int(gy)), 16, (0, 255, 0), 2)
        x0 = int(max(0, min(vis.shape[1] - 900, gx - 450)))
        y0 = int(max(0, min(vis.shape[0] - 620, gy - 340)))
        cv2.imwrite(r"C:\Users\Van Dark Simple\AppData\Local\Temp\grid_%d.png" % k,
                    vis[y0:y0 + 620, x0:x0 + 900])
