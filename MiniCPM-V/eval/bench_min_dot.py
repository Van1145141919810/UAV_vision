# -*- coding: utf-8 -*-
"""更严谨的极小目标测试：保留真实背景，只把红点重画成 3~7 px

避免上一个测试"整幅图缩小"导致的额外退化。
在 41 m 那帧的真实白纸区域上，按 100 m 的像素尺寸重画红点。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
import mcpv.detect_live as D                                # noqa: E402

VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
STRICT = dict(S_MIN=115, V_MIN=60, AREA_MIN=15, FILL_MIN=0.55,
              RD_MIN=50, RD_S_MIN=90, RD_AREA_MIN=6, RD_FILL_MIN=0.42)
LOOSE = dict(S_MIN=80, V_MIN=40, AREA_MIN=4, FILL_MIN=0.30,
             RD_MIN=26, RD_S_MIN=50, RD_AREA_MIN=2, RD_FILL_MIN=0.22)
FX = 1566.149


def draw_dot(img, cx, cy, d, S=8):
    """超采样画直径 d 的圆（避免整数半径量化）。"""
    r = max(0.4, d / 2.0)
    size = int(np.ceil(r)) * 2 + 5
    big = np.zeros((size * S, size * S, 3), np.uint8)
    cv2.circle(big, (size * S // 2, size * S // 2), int(round(r * S)),
               (40, 40, 205), -1, lineType=cv2.LINE_AA)
    small = cv2.resize(big, (size, size), interpolation=cv2.INTER_AREA)
    m = small.any(axis=2)
    x0, y0 = int(cx) - size // 2, int(cy) - size // 2
    reg = img[y0:y0 + size, x0:x0 + size]
    reg[m] = small[m]


cap = cv2.VideoCapture(str(VID))
cap.set(cv2.CAP_PROP_POS_FRAMES, 60)
ok, base = cap.read()
cap.release()
for k, v in STRICT.items():
    setattr(D, k, v)
c = D.detect_reds(base)
dot = min(c, key=lambda q: abs(q["d"] - 7.6))
PX, PY = int(dot["cx"]), int(dot["cy"])
print(f"41 m 帧：红点 {dot['d']:.1f}px @ ({PX},{PY})，用真实背景重画")

print(f"\n   {'直径':<6}{'距离(20cm)':<13}{'距离(26cm)':<13}"
      f"{'严格':<8}{'宽松':<8}{'严检直径':<10}{'宽松直径'}")
print("   " + "-" * 68)
# 先抹掉原来的 7.6px 红点：用周围纸面的中位颜色填充
ERASE = 10
paper = base[PY - 22:PY + 22, PX - 22:PX + 22].reshape(-1, 3)
bright = paper[paper.sum(axis=1) > np.percentile(paper.sum(axis=1), 70)]
FILL = np.median(bright, axis=0).astype(np.uint8)
print(f"   抹点用的纸面颜色 BGR={FILL.tolist()}")

for d in (2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0):
    f = base.copy()
    f[PY - ERASE:PY + ERASE, PX - ERASE:PX + ERASE] = FILL
    draw_dot(f, PX, PY, d)
    ok2, buf = cv2.imencode(".jpg", f, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    f = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    res = {}
    for name, thr in (("严格", STRICT), ("宽松", LOOSE)):
        for k, v in thr.items():
            setattr(D, k, v)
        cs = D.detect_reds(f)
        if cs:
            near = min(cs, key=lambda q: np.hypot(q["cx"] - PX, q["cy"] - PY))
            dn = np.hypot(near["cx"] - PX, near["cy"] - PY)
        else:
            near, dn = None, 999
        res[name] = (dn <= 8, near["d"] if near is not None and dn <= 8 else 0.0)
    print(f"   {d:<6.1f}{FX*0.20/d:<13.0f}{FX*0.26/d:<13.0f}"
          f"{('✓' if res['严格'][0] else '✗'):<8}"
          f"{('✓' if res['宽松'][0] else '✗'):<8}"
          f"{res['严格'][1]:<10.1f}{res['宽松'][1]:.1f}")
