# -*- coding: utf-8 -*-
"""从这次演示录像里抽关键帧，做一张给上级看的拼版

时间点（按日志）：
  12.7s 人工确认 d=8.6px（38 m）
  18.0s 跟踪中 38.5 m，重捕获 score=0.97
  45.0s 重新选中 d=31.7px
  46.5s 接近中 8.4 m，速度开始下降
  48.5s 7.3 m，速度 1.75
  49.5s HANDOFF d=50.1px / 6.26 m
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imwrite_unicode                       # noqa: E402

VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
OUT = ROOT / "eval" / "figures" / "demo_keyframes.jpg"
FONT = r"C:\Windows\Fonts\msyh.ttc"

SHOTS = [
    (12.9, "① 人工确认目标  d≈8.6 px  ≈38 m"),
    (18.2, "② 远距跟踪中  38.5 m  重捕获 score 0.97"),
    (45.2, "③ 丢失后重新锁定  d≈31.7 px"),
    (46.6, "④ 接近中  8.4 m  速度开始下降"),
    (48.6, "⑤ 制动段  7.3 m  速度 1.75 m/s"),
    (49.6, "⑥ HANDOFF  d=50.1 px ≈6.26 m"),
]


def main():
    # ★ 录像写入帧率(≈11.5) 与播放帧率(10) 不一致，按日志秒数取帧会错位。
    #   改用"HANDOFF DONE"橙色大字定位交接帧：颜色 (0,160,255) BGR，
    #   画在 (12, H-170)，scale 0.8。
    cap = cv2.VideoCapture(str(VID))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    hgt = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    band0, band1 = max(0, hgt - 190), max(0, hgt - 150)
    h_first = None
    for i in range(0, total, 2):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, fr = cap.read()
        if not ok:
            continue
        roi = fr[band0:band1, 0:700]
        b, g, r = (roi[:, :, k].astype(np.int16) for k in range(3))
        n_orange = int(((r > 180) & (g > 110) & (g < 210) & (b < 100)).sum())
        if n_orange > 80:
            h_first = i
            break
    cap.release()
    if h_first is None:
        h_first = total - 5
    print(f"  总帧 {total}  交接帧 {h_first}")

    picks = [max(0, h_first - 420), max(0, h_first - 300),
             max(0, h_first - 160), max(0, h_first - 60),
             max(0, h_first - 6), min(total - 1, h_first + 2)]
    LAB = ["① 确认目标后进入接近",
           "② 接近中（距离在缩小）",
           "③ 接近后段（速度已下降）",
           "④ 制动段",
           "⑤ 触发交接前一帧",
           "⑥ HANDOFF 交接瞬间"]
    cap = cv2.VideoCapture(str(VID))
    frames = []
    for i, lab in zip(picks, LAB):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, fr = cap.read()
        frames.append((fr if ok else None, f"{lab}  帧{i}"))
    cap.release()
    print(f"  取帧 {picks}")

    f_t = ImageFont.truetype(FONT, 26)
    f_s = ImageFont.truetype(FONT, 17)
    cw, ch = 640, 360
    cols, rows = 2, 3
    W = cols * cw + 30
    H = 60 + rows * (ch + 30) + 16
    sheet = Image.new("RGB", (W, H), (16, 16, 16))
    d = ImageDraw.Draw(sheet)
    d.text((12, 8), "红点跟踪 → 接近 → 交接   全程录像关键帧",
           font=f_t, fill=(0, 220, 255))
    d.text((12, 38), "1920×1080  10 fps  20 cm 红圆  fx=1566.1  "
                     "交接判据 d_px ≥ 48.2（6.5 m）", font=f_s,
           fill=(170, 200, 225))
    for i, (fr, lab) in enumerate(frames):
        r, c = divmod(i, cols)
        x = 12 + c * (cw + 6)
        y = 64 + r * (ch + 30)
        if fr is None:
            continue
        im = cv2.resize(fr, (cw, ch))
        sheet.paste(Image.fromarray(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)), (x, y))
        d.text((x, y + ch + 4), lab, font=f_s, fill=(120, 230, 120))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(OUT, quality=93)
    print(f"关键帧拼版: {OUT}  ({sheet.width}x{sheet.height})")

    # 顺便从录像补出"交接瞬间"快照（原脚本快照因中文路径静默失败）
    cap = cv2.VideoCapture(str(VID))
    cap.set(cv2.CAP_PROP_POS_FRAMES, min(total - 1, h_first + 1))
    ok, fr = cap.read()
    cap.release()
    if ok:
        p = ROOT / "eval" / "demo_out" / "handoff_164951.jpg"
        imwrite_unicode(p, fr, quality=95)
        print(f"交接快照（从录像补）: {p}")


if __name__ == "__main__":
    main()
