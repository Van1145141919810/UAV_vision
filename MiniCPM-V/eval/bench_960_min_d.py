# -*- coding: utf-8 -*-
"""在 960×540（现场真实工作分辨率）下重新标定 min_d_px

背景：我之前的 552 帧基准是在 1920×1080 校正图上做的，结论"min_d_px=7 → 远处 99%"。
但现场两处都跑 960×540：
  · 机上检测 det_scale=0.5
  · 地面检测 gcs_gate --scale 0.5，且上行只发 1920×540 并排（每目 960×540）
实测帧55 在 960 下他一个候选都没有，真点 4.4 px 被门限挡掉。

所以这里在 960 下扫 min_d_px ∈ {7,5,4,3,2}，量远处/近处召回。
min_d_px 单位是 "ref px"，检测器内部按 f/ref_f 缩放 → 960 下有效门限 = min_d_px×0.5。
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
cap = cv2.VideoCapture(VID)
frames = []
while True:
    ok, fr = cap.read()
    if not ok:
        break
    frames.append(fr)
cap.release()
N = len(frames)
print(f"{N} 帧")

rig_hi = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=1.0)
rig_lo = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=0.5)

# 真值：1920 校正图 + 我的检测器 + 接力（沿用之前验证过的种子）
print("建真值…")
HI = []
for f in frames:
    L, _ = rig_hi.rectify(f, f)
    HI.append([(c["cx"], c["cy"], c["d"]) for c in detect_reds(L)])


def link(seed, start, step):
    tr, cur, i = {start: seed}, seed, start + step
    while 0 <= i < N:
        best, bd = None, 1e9
        for (cx, cy, d) in HI[i]:
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


far = min(HI[60], key=lambda q: np.hypot(q[0] - 1029, q[1] - 508))
near = min(HI[430], key=lambda q: np.hypot(q[0] - 993, q[1] - 722))
GT = link(far, 60, +1)
GT.update(link(far, 60, -1))
NEAR = link(near, 430, +1)
NEAR.update(link(near, 430, -1))
FAR = {k: v for k, v in GT.items() if k not in NEAR}
print(f"  远处 {len(FAR)} 帧  近处 {len(NEAR)} 帧")

# 预先把 960 校正图算好
# ★ 必须先缩到 960x540 再 rectify：mapL/mapR 是按 960 建的，
#   直接喂 1920 原图只会采到左上角（踩过）
LO = []
for f in frames:
    small = cv2.resize(f, (960, 540), interpolation=cv2.INTER_AREA)
    L, R = rig_lo.rectify(small, small)
    LO.append((L, R))
print("  960 校正图就绪")


def evaluate(min_d_px, open_k=1):
    cfg = DotConfig()
    cfg.min_d_px, cfg.dot_d, cfg.d_max, cfg.open_k = min_d_px, 0.20, 0.28, open_k
    cfg.max_cands = 20
    det = DotDetectorCore(rig_lo, cfg)
    hit_f = tot_f = hit_n = tot_n = 0
    ncand = 0
    for i, (L, R) in enumerate(LO):
        out = det.process(L, R, i / 10.0)
        ncand += len(out.cands)
        for seg, is_far in ((FAR, True), (NEAR, False)):
            ref = seg.get(i)
            if ref is None:
                continue
            gx, gy, gd = ref[0] / 2.0, ref[1] / 2.0, ref[2] / 2.0
            ok = any(np.hypot(c.u - gx, c.v - gy) <= 6 and
                     0.70 <= c.dmaj / max(gd, 1e-9) <= 1.50 for c in out.cands)
            if is_far:
                tot_f += 1; hit_f += ok
            else:
                tot_n += 1; hit_n += ok
    return hit_f, tot_f, hit_n, tot_n, ncand / N


print(f"\n{'min_d_px':<10}{'有效门限(960)':<14}{'远处召回':<14}{'近处召回':<14}{'候选/帧'}")
print("-" * 70)
for md in (7.0, 5.0, 4.0, 3.0, 2.0):
    hf, tf, hn, tn, nc = evaluate(md)
    print(f"{md:<10.1f}{md*0.5:<14.1f}{f'{hf}/{tf} ({hf/tf*100:.0f}%)':<14}"
          f"{f'{hn}/{tn} ({hn/tn*100:.0f}%)':<14}{nc:6.1f}")
