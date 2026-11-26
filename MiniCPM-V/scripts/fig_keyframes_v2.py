# -*- coding: utf-8 -*-
"""从"回放新录像"里抽关键帧，重点展示新的 PRED（预测）区分

新版画面上的颜色：
    STATE: TRACK/APPROACH 绿(80,240,90)   RECAPTURE (60,200,255)
    实测十字 绿(80,240,90)                预测菱形 橙(0,165,255)
    HANDOFF 横幅 橙(0,160,255)  在画面下方 y≈H-170
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
VID = ROOT / "eval" / "demo_out" / "replay_new.mp4"
OUT = ROOT / "eval" / "figures" / "demo_keyframes_v2.jpg"
FONT = r"C:\Windows\Fonts\msyh.ttc"

GREEN = (80, 240, 90)
RECAP = (60, 200, 255)
PRED = (0, 165, 255)
BANNER = (0, 160, 255)


def cnt(img, color, tol=40):
    d = np.linalg.norm(img.astype(np.int16) - np.array(color, np.int16), axis=2)
    return int((d <= tol).sum())


def main():
    cap = cv2.VideoCapture(str(VID))
    fps = cap.get(cv2.CAP_PROP_FPS) or 10
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    st = {"acquire": [], "track": [], "recap": [], "pred": [], "handoff": None}
    for i in range(total):
        ok, fr = cap.read()
        if not ok:
            break
        top = fr[10:46, 0:250]
        band = fr[max(0, H - 190):max(0, H - 150), 0:700]
        if cnt(band, BANNER, 30) > 80:
            if st["handoff"] is None:
                st["handoff"] = i
            continue
        g = cnt(top, GREEN, 35)
        r = cnt(top, RECAP, 35)
        if r > g:
            st["recap"].append(i)
            if cnt(fr, PRED, 30) > 60:
                st["pred"].append(i)
        elif g > 300:
            st["track"].append(i)
        else:
            st["acquire"].append(i)
    cap.release()
    print(f"总帧 {total}  TRACK {len(st['track'])}  RECAPTURE {len(st['recap'])} "
          f"(含PRED {len(st['pred'])})  交接帧 {st['handoff']}")

    def pick(lst, idx=0.5):
        return lst[int(len(lst) * idx)] if lst else None

    picks = []
    picks.append((pick(st["acquire"], 0.6), "候选检出（黄框）"))
    picks.append((pick(st["track"], 0.35), "正常跟踪：绿十字落在红点上"))
    picks.append((pick(st["pred"], 0.5), "丢失预测：橙色菱形 PRED（非实测）"))
    picks.append((pick(st["track"], 0.85), "检测兜底重锁后回到接近"))
    picks.append((max(0, (st["handoff"] or total) - 12), "制动段"))
    picks.append((st["handoff"], "HANDOFF 交接"))
    picks = [p for p in picks if p[0] is not None]

    f_t = ImageFont.truetype(FONT, 24)
    f_s = ImageFont.truetype(FONT, 16)
    cw, ch = 640, 360
    cols = 2
    rows = (len(picks) + cols - 1) // cols
    W = cols * cw + 30
    Ht = 62 + rows * (ch + 32) + 14
    sheet = Image.new("RGB", (W, Ht), (16, 16, 16))
    d = ImageDraw.Draw(sheet)
    d.text((12, 8), "红点跟踪（新版显示）—— 实测与预测已区分", font=f_t,
           fill=(0, 220, 255))
    d.text((12, 36), "绿十字 = 实测；橙色菱形 PRED = 光流外推的预测位置；"
                     "黄框 = 每帧检测候选", font=f_s, fill=(170, 200, 225))
    cap = cv2.VideoCapture(str(VID))
    for k, (idx, lab) in enumerate(picks):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, fr = cap.read()
        r, c = divmod(k, cols)
        x = 12 + c * (cw + 6)
        y = 62 + r * (ch + 32)
        if ok:
            sheet.paste(Image.fromarray(cv2.cvtColor(
                cv2.resize(fr, (cw, ch)), cv2.COLOR_BGR2RGB)), (x, y))
        d.text((x, y + ch + 5), f"{k+1}. {lab}   帧{idx}", font=f_s,
               fill=(120, 230, 120))
    cap.release()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(OUT, quality=93)
    print(f"出图: {OUT}  ({sheet.width}x{sheet.height})")


if __name__ == "__main__":
    main()
