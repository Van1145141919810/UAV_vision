# -*- coding: utf-8 -*-
"""效果图：定稿参数下，他的检测器在这段录像上的实际表现

选 7 个时间点（从远到近），每个：
  · 上：整帧（缩到 1/3），画出检出框
  · 下：该处 3 倍放大，标 d_px / 距离 / 候选数
参数 = 定稿档（min_d_px=7, dot_d=0.20, d_max=0.28）
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
from mcpv.cv import imwrite_unicode                          # noqa: E402

VID = MY + r"\eval\demo_out\run_20261109_164902.mp4"
OUT = Path(MY) / "eval" / "figures" / "效果图_定稿参数.jpg"
FONT = r"C:\Windows\Fonts\msyh.ttc"
FX = 1566.149
MARKER = 0.20

rig = StereoRig.from_file(S14 + r"\stereo_calib.yaml", scale=1.0)
cfg = DotConfig()
cfg.min_d_px = 7.0
cfg.dot_d = 0.20
cfg.d_max = 0.28
det = DotDetectorCore(rig, cfg)

# 真值轨迹（用于挑"哪一个才是红点"）——沿用之前确认的位置
TRUTH = {40: None, 120: None, 200: None, 280: None, 360: None, 440: None, 520: None}

cap = cv2.VideoCapture(VID)
frames = {}
for idx in TRUTH:
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, fr = cap.read()
    if ok:
        frames[idx] = fr
cap.release()

rows = []
for idx, fr in frames.items():
    L, R = rig.rectify(fr, fr)
    out = det.process(L, R, idx / 10.0)
    cands = out.cands
    if not cands:
        rows.append((idx, None, None, 0))
        continue
    # 选分数最高（他实际会选的）
    best = max(cands, key=lambda c: c.ps)
    rows.append((idx, best, L, len(cands)))

# ── 拼图 ─────────────────────────────────────────────
f_t = ImageFont.truetype(FONT, 20)
f_s = ImageFont.truetype(FONT, 15)
TILE_W, TILE_H = 300, 190        # 整帧缩略
CROP = 150                        # 放大裁剪半边长
N = len(rows)
W = N * (TILE_W + 8) + 8
H = 40 + TILE_H + 8 + TILE_W + 34
sheet = Image.new("RGB", (W, H), (18, 18, 20))
d = ImageDraw.Draw(sheet)
d.text((10, 8), "定稿参数（min_d_px=7）下他的检测器在这段录像上的表现 —— 从远到近",
       font=f_t, fill=(0, 220, 255))

for col, (idx, best, L, ncand) in enumerate(rows):
    x0 = 8 + col * (TILE_W + 8)
    if best is None:
        d.text((x0 + 6, 60), f"帧{idx}: 无候选", font=f_s, fill=(255, 120, 120))
        continue
    u, v, dp = best.u, best.v, best.dmaj
    dist = FX * MARKER / max(dp, 1e-6)
    # 整帧
    thumb = cv2.resize(L, (TILE_W, TILE_H))
    sx, sy = TILE_W / L.shape[1], TILE_H / L.shape[0]
    cv2.circle(thumb, (int(u * sx), int(v * sy)), 9, (0, 255, 0), 2)
    cv2.circle(thumb, (int(u * sx), int(v * sy)), 2, (0, 255, 0), -1)
    sheet.paste(Image.fromarray(cv2.cvtColor(thumb, cv2.COLOR_BGR2RGB)),
                (x0, 40))
    d.text((x0 + 4, 40 + TILE_H + 4),
           f"帧{idx}  {dp:.1f}px  ≈{dist:.0f}m  候选{ncand}", font=f_s,
           fill=(200, 230, 200))
    # 放大裁剪
    cx, cy = int(u), int(v)
    a = max(CROP, int(dp * 4))
    xa, ya = max(0, cx - a), max(0, cy - a)
    crop = L[ya:ya + 2 * a, xa:xa + 2 * a]
    if crop.size:
        crop = cv2.resize(crop, (TILE_W, TILE_W), interpolation=cv2.INTER_NEAREST)
        k = TILE_W / (2 * a)
        cv2.circle(crop, (int((cx - xa) * k), int((cy - ya) * k)),
                   max(4, int(dp * k / 2)), (0, 255, 0), 2)
        sheet.paste(Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)),
                    (x0, 40 + TILE_H + 24))

OUT.parent.mkdir(parents=True, exist_ok=True)
sheet.save(OUT, quality=92)
print(f"出图: {OUT}  ({sheet.width}x{sheet.height})")
print("\n各帧读数：")
for idx, best, L, ncand in rows:
    if best:
        dist = FX * MARKER / max(best.dmaj, 1e-6)
        print(f"  帧{idx:3d}: d={best.dmaj:5.1f}px  ≈{dist:5.1f}m  "
              f"候选{ncand:2d}  位置({best.u:.0f},{best.v:.0f})  分{best.ps:.2f}")
    else:
        print(f"  帧{idx:3d}: 无候选")
