# -*- coding: utf-8 -*-
"""诚实版效果图 + 排名统计

回答两个问题：
  ① 红点在不在候选里？（召回）
  ② 它在候选里排第几？（排序 —— 决定人好不好选、VLM 会不会错）

做法：
  · 真值 = 之前建的参考轨迹（远处段 0~97、近处段 239~502，肉眼确认过种子）
  · 逐帧跑他的检测器（定稿参数 min_d_px=7），按 ps 降序排名
  · 统计真值命中所处的名次分布
  · 出图：画出**全部**候选（橙=假、绿=真），数字标名次
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

S14 = r"D:\Desktop\无人系统\_ref\s14"
MY = r"D:\Desktop\无人系统\MiniCPM-V"
sys.path.insert(0, S14)
sys.path.insert(0, MY + r"\src")

from vb09.perception.stereo_rig import StereoRig            # noqa: E402
from vb09.perception.reddot.config import DotConfig          # noqa: E402
from vb09.perception.reddot.detect import DotDetectorCore    # noqa: E402
from mcpv.detect_live import detect_reds                     # noqa: E402

VID = MY + r"\eval\demo_out\run_20261109_164902.mp4"
OUT = Path(MY) / "eval" / "figures" / "效果图_候选与排名.jpg"
FONT = r"C:\Windows\Fonts\msyh.ttc"
FX, MARKER = 1566.149, 0.20

cap = cv2.VideoCapture(VID)
frames = []
while True:
    ok, fr = cap.read()
    if not ok:
        break
    frames.append(fr)
cap.release()
N = len(frames)

rig = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=1.0)
cfg = DotConfig()
cfg.min_d_px, cfg.dot_d, cfg.d_max = 7.0, 0.20, 0.28
det = DotDetectorCore(rig, cfg)

# ── 逐帧：rectify 一次，既跑我的（建真值）也跑他的 ──────────
print("逐帧检测…")
MINE, HIS, LEFTS = [], [], []
for i, fr in enumerate(frames):
    L, R = rig.rectify(fr, fr)
    LEFTS.append(L)
    MINE.append([(c["cx"], c["cy"], c["d"]) for c in detect_reds(L)])
    out = det.process(L, R, i / 10.0)
    HIS.append(sorted(out.cands, key=lambda c: -c.ps))
print("  完成")


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

# ── 排名统计 ────────────────────────────────────────
def rank_of(i):
    ref = GT.get(i) or NEAR.get(i)
    if ref is None:
        return None
    for r, c in enumerate(HIS[i], 1):
        if np.hypot(c.u - ref[0], c.v - ref[1]) <= 12 and \
                0.70 <= c.dmaj / max(ref[2], 1e-6) <= 1.50:
            return r
    return 0


for name, seg in (("远处段", FAR), ("近处段", NEAR)):
    rs = [rank_of(i) for i in seg]
    rs = [r for r in rs if r is not None]
    n = len(rs)
    r1 = sum(1 for r in rs if r == 1)
    r3 = sum(1 for r in rs if 1 <= r <= 3)
    r5 = sum(1 for r in rs if 1 <= r <= 5)
    miss = sum(1 for r in rs if r == 0)
    print(f"\n{name}（{n} 帧）：检出 {n-miss}/{n} ({(n-miss)/n*100:.0f}%)")
    print(f"  排第 1  : {r1:3d}/{n} ({r1/n*100:4.0f}%)")
    print(f"  前 3 名 : {r3:3d}/{n} ({r3/n*100:4.0f}%)")
    print(f"  前 5 名 : {r5:3d}/{n} ({r5/n*100:4.0f}%)")

# ── 出图：6 帧，画全部候选 ───────────────────────────
PICK = [55, 90, 280, 340, 420, 500]
f_t = ImageFont.truetype(FONT, 20)
f_s = ImageFont.truetype(FONT, 15)
TILE = 340
W = len(PICK) * (TILE + 8) + 8
H = 40 + TILE + 44
sheet = Image.new("RGB", (W, H), (18, 18, 20))
d = ImageDraw.Draw(sheet)
d.text((10, 8), "定稿参数下的全部候选：绿=真红点(第n名)  橙=假候选  —— 数字是排名",
       font=f_t, fill=(0, 220, 255))

for col, idx in enumerate(PICK):
    x0 = 8 + col * (TILE + 8)
    L = LEFTS[idx]
    r = rank_of(idx)
    cands = HIS[idx]
    # 以真值（或第一名）为中心裁剪
    ref = GT.get(idx) or NEAR.get(idx)
    cx, cy = (int(ref[0]), int(ref[1])) if ref else (
        int(cands[0].u), int(cands[0].v)) if cands else (L.shape[1] // 2, L.shape[0] // 2)
    half = 190
    xa, ya = max(0, cx - half), max(0, cy - half)
    crop = L[ya:ya + 2 * half, xa:xa + 2 * half]
    if crop.size == 0:
        continue
    crop = cv2.resize(crop, (TILE, TILE), interpolation=cv2.INTER_LINEAR)
    k = TILE / (2 * half)
    for rank, c in enumerate(cands[:6], 1):
        px, py = int((c.u - xa) * k), int((c.v - ya) * k)
        is_true = (r == rank)
        col_rgb = (0, 255, 0) if is_true else (0, 165, 255)
        cv2.circle(crop, (px, py), max(7, int(c.dmaj * k / 2)), col_rgb, 2)
        cv2.putText(crop, str(rank), (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col_rgb, 2)
    sheet.paste(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)), (x0, 40))
    dp = cands[0].dmaj if cands else 0
    lab = f"帧{idx}  候选{len(cands)}  真点第{r}名" if r else f"帧{idx}  候选{len(cands)}  未检出"
    d.text((x0 + 4, 40 + TILE + 6), lab, font=f_s,
           fill=(150, 255, 150) if r and r <= 3 else (255, 200, 120))

OUT.parent.mkdir(parents=True, exist_ok=True)
sheet.save(OUT, quality=92)
print(f"\n出图: {OUT}")
