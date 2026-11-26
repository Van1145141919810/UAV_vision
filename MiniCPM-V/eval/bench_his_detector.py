# -*- coding: utf-8 -*-
"""融合实测：用【他的检测器】跑【你的飞行录像】，扫参数找最优

评价标准（按你的优先级）：
  ① 召回：远处小目标能不能检出（最高优先）
  ② 候选数量：每帧多少个候选（太多人选不过来、VLM 也容易错）
  ③ 耗时

真值：用我的全分辨率检测器建一条参考轨迹（帧 35~84 远处段 + 238~502 近处段），
      他的候选只要落在 12 px 内、尺寸比 0.7~1.5 就算命中。

注意：只扫**检测器**参数（detect.py 里用的）。
      min_score / mono_score_min / d_min / d_max 是 Locator（发布层）用的，
      不影响"候选列表里有没有这个点"，所以不在这一轮扫。
"""
import sys
import time
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

# ── 读帧 ──────────────────────────────────────────────
cap = cv2.VideoCapture(VID)
frames = []
while True:
    ok, fr = cap.read()
    if not ok:
        break
    frames.append(fr)
cap.release()
N = len(frames)
print(f"载入 {N} 帧")

# ── 真值：我的检测器 + 接力 ────────────────────────────
print("建参考轨迹（我的检测器，在【校正后左目】上）…")
t0 = time.time()
MYC = []
for f in frames:
    L, _ = rig.rectify(f, f)
    MYC.append([(c["cx"], c["cy"], c["d"]) for c in detect_reds(L)])
my = MYC
print(f"  {N} 帧 {time.time()-t0:.1f}s")


def link(seed, start, step):
    tr = {start: seed}
    cur = seed
    i = start + step
    while 0 <= i < N:
        best, bd = None, 1e9
        for (cx, cy, d) in my[i]:
            dd = np.hypot(cx - cur[0], cy - cur[1])
            if dd > 110:
                continue
            if not (0.45 <= d / max(cur[2], 1e-6) <= 2.4):
                continue
            if dd < bd:
                bd, best = dd, (cx, cy, d)
        if best is None:
            i += step
            if abs(i - start) > 25:
                break
            continue
        tr[i] = best
        cur = best
        i += step
    return tr


far_seed = min(my[60], key=lambda q: np.hypot(q[0] - 1029, q[1] - 508))
near_seed = min(my[430], key=lambda q: np.hypot(q[0] - 993, q[1] - 722))
print(f"  种子 帧60: {tuple(round(x,1) for x in far_seed)}  "
      f"帧430: {tuple(round(x,1) for x in near_seed)}")
GT = {}
GT.update(link(far_seed, 60, +1))
GT.update(link(far_seed, 60, -1))
NEAR = {}
NEAR.update(link(near_seed, 430, +1))
NEAR.update(link(near_seed, 430, -1))
FAR = {k: v for k, v in GT.items() if k not in NEAR}
print(f"  远处段 {len(FAR)} 帧（{min(FAR)}~{max(FAR)}）  近处段 {len(NEAR)} 帧"

      f"（{min(NEAR)}~{max(NEAR)}）")

# ── 扫参数 ────────────────────────────────────────────
BASE = dict(min_d_px=10.0, white_min=6, red_min=60.0, max_d_px=450.0)
VARIANTS = [
    ("① 他的默认", dict()),
    ("② min_d_px=7", dict(min_d_px=7.0)),
    ("③ min_d_px=5", dict(min_d_px=5.0)),
    ("④ 7 + white_min=4", dict(min_d_px=7.0, white_min=4)),
    ("⑤ 7 + white_min=0", dict(min_d_px=7.0, white_min=0)),
    ("⑥ 7 + red_min=55", dict(min_d_px=7.0, red_min=55.0)),
    ("⑦ 7 + white4 + red55", dict(min_d_px=7.0, white_min=4, red_min=55.0)),
    ("⑧ 5 + white4 + red55", dict(min_d_px=5.0, white_min=4, red_min=55.0)),
]

print(f"\n{'方案':<24}{'远处召回':<14}{'近处召回':<14}{'候选/帧':<10}{'ms/帧'}")
print("-" * 78)
results = []
for name, kw in VARIANTS:
    cfg = DotConfig()
    for k, v in {**BASE, **kw}.items():
        setattr(cfg, k, v)
    det = DotDetectorCore(rig, cfg)
    hit_f = tot_f = hit_n = tot_n = 0
    ncand = 0
    tsum = 0.0
    for i, fr in enumerate(frames):
        L, R = rig.rectify(fr, fr)
        t0 = time.time()
        out = det.process(L, R, i / 10.0)
        tsum += (time.time() - t0) * 1000
        ncand += len(out.cands)
        for bag, is_far in ((FAR, True), (NEAR, False)):
            ref = bag.get(i)
            if ref is None:
                continue
            ok = False
            for c in out.cands:
                if np.hypot(c.u - ref[0], c.v - ref[1]) <= 12 and \
                        0.70 <= c.dmaj / max(ref[2], 1e-6) <= 1.50:
                    ok = True
                    break
            if is_far:
                tot_f += 1
                hit_f += ok
            else:
                tot_n += 1
                hit_n += ok
    ms = tsum / N
    results.append((name, hit_f, tot_f, hit_n, tot_n, ncand / N, ms))
    print(f"{name:<24}{f'{hit_f}/{tot_f} ({hit_f/tot_f*100:.0f}%)':<14}"
          f"{f'{hit_n}/{tot_n} ({hit_n/tot_n*100:.0f}%)':<14}"
          f"{ncand/N:6.1f}    {ms:5.1f}")

print("\n判读：远处召回优先；同样召回下选『候选/帧』更少的（人选得快、VLM 也不容易错）")
