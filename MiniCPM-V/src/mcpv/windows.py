# -*- coding: utf-8 -*-
"""窗户候选框（只用于界面提示，不参与任何决策）。

为什么不做"精确检测所有窗户"：
    实测（win06 一层楼）：朴素矩形检测能框到大部分窗，但会拆成窗格碎片、
    还会把树叶/栏杆/空调外机框进来，150 个框人根本点不准。

所以这里的定位是**提示**：
    · 宁多勿漏（用户要的是"别漏"，多余的框不点就行）
    · 严格一点的尺寸/长宽比 + 去嵌套，把碎片压掉
    · 界面上画淡色细框，**不影响"点任意位置"**
"""

from typing import List, Tuple

import cv2
import numpy as np

Box = Tuple[int, int, int, int]      # x, y, w, h


def find_windows(bgr: np.ndarray,
                 min_w: int = 48, min_h: int = 30,
                 aspect=(0.45, 3.2), extent_min=0.45,
                 merge_w: int = 21, max_boxes: int = 60) -> List[Box]:
    """返回窗户候选框（按面积从大到小）。"""
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(2.0, (8, 8)).apply(gray)
    edges = cv2.Canny(gray, 60, 160)
    # 横向闭运算把同一扇窗的窗格连成一整块（merge_w 越大，合并越狠）
    closed = cv2.morphologyEx(
        edges, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (merge_w, 3)), iterations=2)

    cnts, _ = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    boxes: List[Box] = []
    for c in cnts:
        x, y, ww, hh = cv2.boundingRect(c)
        if ww < min_w or hh < min_h:
            continue
        ar = ww / float(hh)
        if ar < aspect[0] or ar > aspect[1]:
            continue
        area = cv2.contourArea(c)
        if area / float(ww * hh) < extent_min:
            continue
        if ww * hh > 0.12 * w * h:
            continue
        boxes.append((x, y, ww, hh))

    # 去嵌套/去重：面积大的优先，被已接受框覆盖 >70% 的丢掉
    boxes.sort(key=lambda b: -(b[2] * b[3]))
    keep: List[Box] = []
    for b in boxes:
        x, y, ww, hh = b
        covered = False
        for k in keep:
            x2, y2, w2, h2 = k
            ix = max(0, min(x + ww, x2 + w2) - max(x, x2))
            iy = max(0, min(y + hh, y2 + h2) - max(y, y2))
            if ix * iy > 0.75 * (ww * hh):
                covered = True
                break
        if not covered:
            keep.append(b)
        if len(keep) >= max_boxes:
            break
    return keep


def estimate_range(boxes, f_px, img_w, real_w=2.0,
                   aspect=(0.35, 2.2), w_frac=(0.02, 0.35)):
    """从一帧里的多个窗框估计"这面墙大概多远"。

    为什么不拿单个框算：
        一个框可能把窗+墙连成一块（实测 win01 出现 645x202、长宽比 3.2 的框，
        按 2 m 反算得 10 m，而实际至少 60 m）。单个框一旦错，距离就错。

    做法：
        1. 先筛掉"不像一扇窗"的框（长宽比、占图比例）
        2. 每个合格框算一个 Z = f * 实物宽 / 框宽
        3. 取中位数（抗一两个坏框）
        4. 用 MAD/中位数 衡量离散度，>0.35 判定为"不可信"

    返回 (Z 或 None, 用到的框数, 离散度)
    """
    import statistics
    cands = []
    for (x, y, bw, bh) in boxes:
        if bh <= 0:
            continue
        ar = bw / float(bh)
        if ar < aspect[0] or ar > aspect[1]:
            continue
        if not (w_frac[0] * img_w <= bw <= w_frac[1] * img_w):
            continue
        cands.append(f_px * real_w / bw)

    if len(cands) < 3:
        return None, len(cands), None

    med = statistics.median(cands)
    mad = statistics.median([abs(c - med) for c in cands])
    spread = mad / med if med > 0 else 1.0
    if spread > 0.35:
        return None, len(cands), spread
    return med, len(cands), spread
