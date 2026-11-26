# -*- coding: utf-8 -*-
"""detect_live.py —— 按真实画面调好的红点检测（替代原来过松的阈值）

═══════════════════════════════════════════════════════════════════
为什么重写
───────────────────────────────────────────────────────────────────
原检测用 HSV S>70 / V>40，在真实室内画面里出 **626 个候选**
（窗帘、木箱、器材全中）。实测各类背景的 S 中位数：
    窗帘(左) 47    窗帘(右) 59    木箱 20    白纸 5
而红圆本身 S 可到 150~250。

所以把阈值提到 S>140 / V>80，候选从 626 降到 22；
再加**圆度 + 长宽比 + 面积范围**三个形状约束，压到个位数。

═══════════════════════════════════════════════════════════════════
形状判据（红圆在画面上就是圆的）
───────────────────────────────────────────────────────────────────
    圆度 fill  = 面积 / (bbox宽 × bbox高)      圆 ≈ 0.785，要求 > 0.55
    长宽比 ar  = w / h                          要求 0.65 ~ 1.55
    面积       = 15 px 起（更小的多半是噪点）
"""
import math

import cv2
import numpy as np

# ── 可调参数 ────────────────────────────────────────────────
S_MIN = 115          # 饱和度下限。实测：窗帘 47~59、粉色墙 73、
                     # 玻璃门上被强光洗过的红点 S 只有 135
                     # —— 原来设 140 正好把它卡掉（现场漏检的真因）。
V_MIN = 60           # 亮度下限（强光下红点 V 很高，但背光时也不该太严）
H_LO, H_HI = 12, 166  # 红色在 HSV 里是 [0,12] ∪ [166,180]
AREA_MIN = 15        # 最小连通域面积（px）
FILL_MIN = 0.55      # 圆度下限
AR_MIN, AR_MAX = 0.65, 1.55   # 长宽比范围
MERGE_GAP = 3        # 形态学闭运算核（把反光打断的圆缝合）

# ── 第二路：红度检测（对又小又淡的红点更稳）──────────────────
RD_MIN = 50          # R - max(G,B) 下限。
                     # 实测：地面 13、树干 36、粉色墙 59~70、真实红点 88。
                     # 单靠它分不开墙和红点（59 vs 88 有重叠），
                     # 真正起作用的是下面的饱和度门。
RD_S_MIN = 90        # ★ 真正的判别门。阈值扫描（同一帧逐像素）：
                     #     R-G>=45,S>=70 → 红点 106 px / 墙面 2872 px（全错）
                     #     R-G>=50,S>=90 → 红点  71 px / 墙面    0 px ✓
                     #   墙面 S 只有 73，红点 S 135。
RD_AREA_MIN = 6      # 比 HSV 路更小也收（远距离红点只有几十像素）
RD_FILL_MIN = 0.42   # 小目标被 JPEG 糊成不规则块，圆度放宽
AREA_MAX = 20000     # 单个目标面积上限（≈直径 160 px）。
                     # 硬底线 3 m 时红点才 104 px（面积 8500），留足余量；
                     # 超过这个面积的多半是墙面/招牌（现场 295 px 那块就是墙）
RD_DEDUP_PX = 12.0   # 与 HSV 路候选中心距离小于此视为同一个


def detect_reds(bgr, max_cand=8):
    """返回按"越像红圆越靠前"排序的候选列表。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, S_MIN, V_MIN), (H_LO, 255, 255)),
        cv2.inRange(hsv, (H_HI, S_MIN, V_MIN), (180, 255, 255)))
    # 闭运算：把高光打断的圆补回来（红纸常有反光缺口）
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         np.ones((MERGE_GAP, MERGE_GAP), np.uint8))

    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    H_img, W_img = bgr.shape[:2]
    hsv_f = hsv.astype(np.float32)
    out = []
    for i in range(1, n):
        a = int(st[i][4])
        if a < AREA_MIN or a > AREA_MAX:
            continue
        x, y, w, h = (int(st[i][0]), int(st[i][1]),
                      int(st[i][2]), int(st[i][3]))
        if w <= 0 or h <= 0:
            continue
        fill = a / float(w * h)
        ar = w / float(h)
        if fill < FILL_MIN or not (AR_MIN <= ar <= AR_MAX):
            continue
        d = 2.0 * math.sqrt(a / math.pi)

        # ★ 环带特征：红圆外面那圈是什么？
        #   我们的标记是"红圆印在白纸上"→ 外圈应当**亮且不饱和**。
        #   地面垫、红积木、红绿灯的外圈都不是白纸，靠这个区分。
        ring_v, ring_s = _ring_stats(hsv_f, float(cen[i][0]),
                                     float(cen[i][1]), d, H_img, W_img)
        s_paper = (min(1.0, max(0.0, (ring_v - 110.0) / 90.0)) * 0.6
                   + min(1.0, max(0.0, (90.0 - ring_s) / 70.0)) * 0.4)

        # 权重调整：白纸分只是"场景加分"，圆度和尺寸才是红圆本身的性质。
        # 实测粉色墙的白纸分 0.72 比玻璃上真红点的 0.35 还高，
        # 所以不能让它主导排序。
        score = (0.40 * s_paper
                 + 0.38 * min(1.0, (fill - FILL_MIN) / (0.785 - FILL_MIN))
                 + 0.22 * min(1.0, d / 40.0))
        out.append({"cx": float(cen[i][0]), "cy": float(cen[i][1]),
                    "d": d, "area": a, "fill": fill, "ar": ar,
                    "ring_v": ring_v, "ring_s": ring_s, "paper": s_paper,
                    "box": (x, y, w, h), "score": score})
    # ── 第二路：红度检测，补上"又小又淡"的红点 ──────────────
    try:
        b, g, r = (bgr[:, :, i].astype(np.int16) for i in range(3))
        rd = r - np.maximum(g, b)
        m2 = (rd >= RD_MIN) & (r >= 60) & (hsv[:, :, 1] >= RD_S_MIN)
        m2 = (m2.astype(np.uint8)) * 255
        m2 = cv2.morphologyEx(m2, cv2.MORPH_CLOSE,
                              np.ones((MERGE_GAP, MERGE_GAP), np.uint8))
        n2, _, st2, cen2 = cv2.connectedComponentsWithStats(m2, 8)
    except Exception:
        n2 = 0

    for i in range(1, n2):
        a = int(st2[i][4])
        if a < RD_AREA_MIN or a > AREA_MAX:
            continue
        x, y, w, h = (int(st2[i][0]), int(st2[i][1]),
                      int(st2[i][2]), int(st2[i][3]))
        if w <= 0 or h <= 0:
            continue
        fill = a / float(w * h)
        ar = w / float(h)
        if fill < RD_FILL_MIN or not (0.45 <= ar <= 2.2):
            continue
        cx, cy = float(cen2[i][0]), float(cen2[i][1])
        # 与已有候选去重
        if any(math.hypot(cx - c["cx"], cy - c["cy"]) < RD_DEDUP_PX
               for c in out):
            continue
        d = 2.0 * math.sqrt(a / math.pi)
        rv, rs = _ring_stats(hsv_f, cx, cy, d, H_img, W_img)
        s_paper = (min(1.0, max(0.0, (rv - 110.0) / 90.0)) * 0.6
                   + min(1.0, max(0.0, (90.0 - rs) / 70.0)) * 0.4)
        out.append({"cx": cx, "cy": cy, "d": d, "area": a, "fill": fill,
                    "ar": ar, "ring_v": rv, "ring_s": rs, "paper": s_paper,
                    "box": (x, y, w, h),
                    "score": 0.5 * s_paper + 0.2 * min(1.0, d / 40.0),
                    "lowconf": True})

    out.sort(key=lambda c: -c["score"])
    return out[:max_cand]


def _ring_stats(hsv_f, cx, cy, d, H, W):
    """取红圆外一圈（1.15~2.0 倍半径）的平均 V 与 S。"""
    r_out = max(4.0, d * 1.0)
    r_in = max(2.0, d * 0.58)
    x0 = int(max(0, cx - r_out))
    x1 = int(min(W, cx + r_out + 1))
    y0 = int(max(0, cy - r_out))
    y1 = int(min(H, cy + r_out + 1))
    if x1 - x0 < 3 or y1 - y0 < 3:
        return 0.0, 255.0
    sub = hsv_f[y0:y1, x0:x1]
    yy, xx = np.mgrid[y0:y1, x0:x1]
    rr = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    m = (rr >= r_in) & (rr <= r_out)
    if m.sum() < 8:
        return 0.0, 255.0
    return float(sub[:, :, 2][m].mean()), float(sub[:, :, 1][m].mean())
