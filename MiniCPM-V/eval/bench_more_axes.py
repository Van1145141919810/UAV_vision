# -*- coding: utf-8 -*-
"""补测：还有哪些优化空间？+ 那 16% 近处漏检到底是什么原因

一、没扫过的关键轴（都用 min_d_px=7 做底）
    red_thr   50→40→35      分割门限（最底层，影响所有斑块）
    open_k    3→1           形态学开运算（3 会把 7px 的点削掉一点）
    shape_px  20→12         小于这个直径用"松形状门限"
    松形状     fill_lo_s/circ_min_s 各降 0.05

二、诊断近处 16% 漏检（264 帧里 41 帧没齐）
    对每个漏检帧，找"离真值最近的候选"，记录距离和尺寸比：
      · 距离 ≤25 px 但尺寸比出界 → 检到了，是尺寸判据的问题
      · 距离 >50 px            → 是真没检到
"""
import sys
from pathlib import Path

import cv2
import numpy as np

S14 = r"D:\Desktop\无人系统\_ref\s14"
MY = r"D:\Desktop\无人系统\MiniCPM-V"
sys.path.insert(0, S14)
sys.path.insert(0, MY + r"\src")

from vb09.perception.stereo_rig import StereoRig            # noqa: E402
from vb09.perception.reddot.config import DotConfig          # noqa: E402
from vb09.perception.reddot.detect import DotDetectorCore    # noqa: E402
from mcpv.detect_live import detect_reds                     # noqa: E402

VID = MY + r"\eval\demo_out\run_20261109_164902.mp4"
rig = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=1.0)

cap = cv2.VideoCapture(VID)
frames = []
while True:
    ok, fr = cap.read()
    if not ok:
        break
    frames.append(fr)
cap.release()
N = len(frames)

# 预先把校正后的左右目算好，避免重复
LEFT, RIGHT = [], []
for f in frames:
    L, R = rig.rectify(f, f)
    LEFT.append(L)
    RIGHT.append(R)
print(f"{N} 帧校正完成")

MINE = [[(c["cx"], c["cy"], c["d"]) for c in detect_reds(L)] for L in LEFT]


def link(seed, start, step):
    tr, cur, i = {start: seed}, seed, start + step
    while 0 <= i < N:
        best, bd = None, 1e9
        for (cx, cy, d) in MINE[i]:
            dd = np.hypot(cx - cur[0], cy - cur[1])
            if dd <= 110 and 0.45 <= d / max(cur[2], 1e-6) <= 2.4 and dd < bd:
                bd, best = dd, (cx, cy, d)
        if best is None:
            i += step
            if abs(i - start) > 25:
                break
            continue
        tr[i], cur, i = best, best, i + step
    return tr


far = min(MINE[60], key=lambda q: np.hypot(q[0] - 1029, q[1] - 508))
near = min(MINE[430], key=lambda q: np.hypot(q[0] - 993, q[1] - 722))
GT = link(far, 60, +1)
GT.update(link(far, 60, -1))
NEAR = link(near, 430, +1)
NEAR.update(link(near, 430, -1))
FAR = {k: v for k, v in GT.items() if k not in NEAR}
print(f"远处段 {len(FAR)} 帧  近处段 {len(NEAR)} 帧")


def evaluate(cfg, tag):
    det = DotDetectorCore(rig, cfg)
    res = {}
    for name, seg in (("远", FAR), ("近", NEAR)):
        hit = 0
        for i in sorted(seg):
            out = det.process(LEFT[i], RIGHT[i], i / 10.0)
            ref = seg[i]
            if any(np.hypot(c.u - ref[0], c.v - ref[1]) <= 12 and
                   0.70 <= c.dmaj / max(ref[2], 1e-6) <= 1.50 for c in out.cands):
                hit += 1
        res[name] = hit
    print(f"{tag:<28}远处 {res['远']:3d}/{len(FAR)}  "
          f"近处 {res['近']:3d}/{len(NEAR)}")
    return res


print("\n=== 一、没扫过的轴 ===")
base = DotConfig()
base.min_d_px, base.dot_d, base.d_max = 7.0, 0.20, 0.28
evaluate(base, "底：min_d_px=7")

for thr in (40, 35):
    c = DotConfig()
    c.min_d_px, c.dot_d, c.d_max = 7.0, 0.20, 0.28
    c.red_thr = thr
    evaluate(c, f"+ red_thr={thr}")

for k in (1,):
    c = DotConfig()
    c.min_d_px, c.dot_d, c.d_max = 7.0, 0.20, 0.28
    c.open_k = k
    evaluate(c, f"+ open_k={k}")

for sp in (12.0,):
    c = DotConfig()
    c.min_d_px, c.dot_d, c.d_max = 7.0, 0.20, 0.28
    c.shape_px = sp
    evaluate(c, f"+ shape_px={sp:.0f}")

c = DotConfig()
c.min_d_px, c.dot_d, c.d_max = 7.0, 0.20, 0.28
c.fill_lo_s, c.circ_min_s = 0.50, 0.55
evaluate(c, "+ 松形状 0.50/0.55")

# 组合
c = DotConfig()
c.min_d_px, c.dot_d, c.d_max = 7.0, 0.20, 0.28
c.red_thr, c.open_k = 40, 1
evaluate(c, "+ red_thr=40 + open_k=1")


# ── 三、open_k 的误检代价：候选/帧 ───────────────────
print("\n=== 三、open_k 对候选数量的影响（误检代价）===")


def cand_stat(cfg, tag):
    det = DotDetectorCore(rig, cfg)
    tot = 0
    nf = 0
    for i in range(0, N, 4):          # 每 4 帧取一帧，够统计
        out = det.process(LEFT[i], RIGHT[i], i / 10.0)
        tot += len(out.cands)
        nf += 1
    print(f"{tag:<28}候选/帧 = {tot/nf:.2f}   （{nf} 帧采样）")
    return tot / nf


b3 = DotConfig(); b3.min_d_px, b3.dot_d, b3.d_max = 7.0, 0.20, 0.28
cand_stat(b3, "open_k=3（他的默认）")
b1 = DotConfig(); b1.min_d_px, b1.dot_d, b1.d_max = 7.0, 0.20, 0.28; b1.open_k = 1
cand_stat(b1, "open_k=1")

# open_k=1 下的召回复核
print("\n=== 四、open_k=1 的最终数字 ===")
evaluate(b1, "open_k=1")
