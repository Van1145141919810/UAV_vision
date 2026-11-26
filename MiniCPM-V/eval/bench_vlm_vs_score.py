# -*- coding: utf-8 -*-
"""录像对比实验：VLM 选点 vs 按分数 vs 按白环 —— 哪个最准

条件尽量贴近现场：
  · 他的 DotDetectorCore + 定稿参数（min_d_px=7, open_k=1, dot_d=0.20, d_max=0.28）
  · 工作分辨率 960x540/目（和他机上/地面检测一致）
  · 真值 = 我的全分辨率轨迹（建在 1920 校正图上，坐标除以 2 换算到 960）

对比四种"选哪个"的策略：
  A 按 ps 分数最高的
  B 按 nw（白环 8 扇区里白的个数）最高的
  C 先 nw 高、再 ps 高（组合）
  D VLM 并排网格选
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
from mcpv.pick_core import suggest                           # noqa: E402

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

rig_hi = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=1.0)
rig_lo = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=0.5)

cfg = DotConfig()
cfg.min_d_px, cfg.dot_d, cfg.d_max, cfg.open_k = 7.0, 0.20, 0.28, 1
det = DotDetectorCore(rig_lo, cfg)

print("建真值（1920 校正图，我的检测器）…")
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
print(f"  远处段 {len(FAR)} 帧   近处段 {len(NEAR)} 帧")

# 采样：远段 12 帧 + 近段 12 帧，均匀分散
def sample(seg, k):
    ks = sorted(seg)
    if len(ks) <= k:
        return ks
    idx = np.linspace(0, len(ks) - 1, k).astype(int)
    return [ks[i] for i in idx]


PICK = sample(FAR, 12) + sample(NEAR, 12)
print(f"  采样 {len(PICK)} 帧做对比\n")

rows = []
for i in PICK:
    small = cv2.resize(frames[i], (960, 540), interpolation=cv2.INTER_AREA)
    L, R = rig_lo.rectify(small, small)
    out = det.process(L, R, i / 10.0)
    cands = sorted(out.cands, key=lambda c: -c.ps)[:6]  # 只给前6，贴近真人看列表
    if not cands:
        continue
    ref = GT.get(i) or NEAR.get(i)
    gx, gy, gd = ref[0] / 2.0, ref[1] / 2.0, ref[2] / 2.0   # 1920 → 960
    truth = None
    for k, c in enumerate(cands, 1):
        if np.hypot(c.u - gx, c.v - gy) <= 6 and 0.70 <= c.dmaj / max(gd, 1e-9) <= 1.50:
            truth = k
            break
    if truth is None:
        continue      # 这一帧他根本没检出真点，不参与"选得对不对"的对比
    a = 1                                              # 按 ps：cands 已按 ps 排
    b = max(range(1, len(cands) + 1), key=lambda k: cands[k - 1].nw)
    c_ = max(range(1, len(cands) + 1),
             key=lambda k: (cands[k - 1].nw, cands[k - 1].ps))
    res = suggest(L, cands, timeout=40)
    rows.append({"frame": i, "n": len(cands), "truth": truth,
                 "A_ps": a, "B_nw": b, "C_nw_ps": c_,
                 "D_vlm": res["pick"], "ans": res["answer"],
                 "s": res["elapsed"],
                 "far": i in FAR})

print(f"{'帧':<6}{'候选':<6}{'真值':<6}{'按分':<6}{'按白环':<8}{'组合':<6}"
      f"{'VLM':<6}{'秒':<7}VLM回答")
print("-" * 84)
for r in rows:
    print(f"{r['frame']:<6}{r['n']:<6}{r['truth']:<6}{r['A_ps']:<6}"
          f"{r['B_nw']:<8}{r['C_nw_ps']:<6}{str(r['D_vlm']):<6}{r['s']:<7.1f}"
          f"{str(r['ans'])[:26]}")

n = len(rows)
print("\n=== 命中率（真值 = 该候选就是白纸红点）===")
for key, name in (("A_ps", "A 按分数"), ("B_nw", "B 按白环"),
                  ("C_nw_ps", "C 组合"), ("D_vlm", "D VLM")):
    hit = sum(1 for r in rows if r[key] == r["truth"])
    print(f"  {name:<10} {hit:2d}/{n}  ({hit/n*100:.0f}%)")
avg = np.mean([r["s"] for r in rows]) if rows else 0
print(f"\n  VLM 平均耗时 {avg:.1f} 秒/次")
