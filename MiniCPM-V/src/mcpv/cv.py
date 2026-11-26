# -*- coding: utf-8 -*-
"""传统 CV：归一化色度分割 → 连通域 → 圆度筛选 → 局部色度对比排序。

为什么用"归一化色度"而不是"绝对红度"：
    绝对红度 R-max(G,B) 依赖曝光。玻璃反光/过曝时红点会变成粉白，
    数值直接掉到阈值以下（实测：高光压住一半面积后，红点面积少一半）。

    归一化色度 (R-max(G,B))/(R+G+B+ε) 只看"红占多少比例"，
    对整体明暗不敏感 —— 暗处、亮处、反光处都能用。

为什么用"局部色度对比"排序：
    远处/杂乱背景下，"这个点和周围不一样"比"它有多红"更可靠。
    cdiff = 点内平均色度 − 环带平均色度
    实测（合成集 6 背景 × 6 直径）：按 cdiff 排，7 px 也能 6/6 找到；
    按白底打分排，只有 2/6。
"""

from pathlib import Path
from typing import List

import cv2
import numpy as np

from . import config as C
from .schema import Candidate


def imread_unicode(path: str | Path) -> np.ndarray:
    """读图（支持中文路径；cv2.imread 在 Windows 上会失败）。"""
    buf = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"读不了这张图: {path}")
    return img


def imwrite_unicode(path: str | Path, img: np.ndarray, quality: int = 92) -> None:
    """写图（支持中文路径）。"""
    ok, buf = cv2.imencode(Path(path).suffix or ".jpg", img,
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise ValueError(f"写不了这张图: {path}")
    buf.tofile(str(path))


def find_candidates(bgr: np.ndarray) -> List[Candidate]:
    """找出所有"圆形的红色块"，按局部色度对比从高到低排序。"""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)

    b, g, r = cv2.split(bgr.astype(np.float32))
    total = r + g + b + 1.0
    chroma = (r - np.maximum(g, b)) / total        # 0~1 的归一化红度

    mask = (chroma > C.CHROMA_MIN).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    n, lab, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

    # ---- 第一遍：只算便宜的几何（面积/圆度），收集候选 ----
    # 为什么分两遍：环形带要算"这块周围一圈"的均值，如果每个候选都在全图上
    # 算距离网格，372 个候选 × 1200 万像素 = 28 秒（实测过）。
    # 先按圆度粗排，只对前 MAX_STATS 个算环形带，成本就封顶了。
    raw = []
    for i in range(1, n):
        x, y, ww, hh, area = stats[i]
        if area < C.AREA_MIN or area > C.AREA_MAX_RATIO * w * h:
            continue
        sub = (lab[y:y + hh, x:x + ww] == i).astype(np.uint8)
        contours, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            continue
        cnt = max(contours, key=cv2.contourArea)
        perim = cv2.arcLength(cnt, True)
        if perim < 1:
            continue
        circ = 4 * np.pi * cv2.contourArea(cnt) / (perim * perim)
        if circ < C.CIRC_MIN:
            continue
        raw.append((circ, i, x, y, ww, hh, area,
                    int(centroids[i][0]), int(centroids[i][1])))

    raw.sort(key=lambda t: -t[0])
    raw = raw[:C.MAX_STATS]

    # ---- 第二遍：只在前 MAX_STATS 个候选周围的小窗口里算环形带 ----
    out: List[Candidate] = []
    for circ, i, x, y, ww, hh, area, cx, cy in raw:
        d_eq = float(np.sqrt(4 * area / np.pi))
        rad = max(3.0, d_eq / 2)
        ro = int(rad * C.RING_OUTER) + 2
        x0, x1 = max(0, cx - ro), min(w, cx + ro + 1)
        y0, y1 = max(0, cy - ro), min(h, cy + ro + 1)

        # 局部坐标网格（只覆盖候选周围，几十×几十像素）
        sy, sx = np.ogrid[y0 - cy:y1 - cy, x0 - cx:x1 - cx]
        d2 = sx.astype(np.int32) ** 2 + sy.astype(np.int32) ** 2
        ch = chroma[y0:y1, x0:x1]
        sa = sat[y0:y1, x0:x1]
        va = val[y0:y1, x0:x1]

        inside = d2 <= (rad * 0.8) ** 2
        ring = (d2 >= (rad * C.RING_INNER) ** 2) & (d2 <= (rad * C.RING_OUTER) ** 2)
        c_in = float(ch[inside].mean()) if inside.any() else 0.0
        c_rg = float(ch[ring].mean()) if ring.any() else 0.0
        ring_s = float(sa[ring].mean()) if ring.any() else 255.0
        ring_v = float(va[ring].mean()) if ring.any() else 0.0
        score = ring_v - 1.5 * ring_s

        half = int(max(C.CROP_MIN_HALF, min(C.CROP_MAX_HALF, d_eq * C.CROP_SCALE)))
        out.append(Candidate(
            cx=cx, cy=cy, diameter_px=round(d_eq, 1),
            bbox_px=[cx - half, cy - half, cx + half, cy + half],
            circularity=round(float(circ), 3),
            ring_s=round(ring_s, 1), ring_v=round(ring_v, 1),
            score=round(score, 1),
            cdiff=round(c_in - c_rg, 3),
            passed_cv=True,          # 能走到这里 = 已过圆度/尺寸门槛
            white_ok=bool(score >= C.RING_SCORE_MIN and ring_v >= C.RING_V_MIN),
        ))

    if C.RANK_BY == "cdiff":
        out.sort(key=lambda c: (-c.cdiff, -c.circularity))
    elif C.RANK_BY == "circularity":
        out.sort(key=lambda c: (-c.circularity, -c.diameter_px))
    else:
        out.sort(key=lambda c: -c.score)
    return out


def crop_for_vlm(bgr: np.ndarray, cand: Candidate) -> np.ndarray:
    """按候选的 bbox 裁一块，并统一放大到 CROP_OUT_PX。"""
    h, w = bgr.shape[:2]
    x0, y0, x1, y1 = cand.bbox_px
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    patch = bgr[y0:y1, x0:x1]
    if patch.size == 0:
        patch = bgr
    return cv2.resize(patch, (C.CROP_OUT_PX, C.CROP_OUT_PX),
                      interpolation=cv2.INTER_LANCZOS4)
