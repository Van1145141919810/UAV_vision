# -*- coding: utf-8 -*-
"""fig_vlm_value.py —— 做"有 VLM vs 无 VLM"的对比图（证明 VLM 的价值）

从两段视频里各抽 3 帧：ACQUIRE（候选初现）、确认后、接近中。
左列无 VLM（锁到红砖），右列有 VLM（锁到红点）。
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imwrite_unicode                        # noqa: E402

FONT = r"C:\Windows\Fonts\msyh.ttc"
OUT = ROOT / "eval" / "figures" / "vlm_vs_novlm.jpg"
DO = ROOT / "eval" / "demo_out"


def frames(path, times):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or 25
    out = []
    for t in times:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
        ok, fr = cap.read()
        out.append(fr if ok else None)
    cap.release()
    return out


def main():
    a = DO / "track_sim_brick_novlm.mp4"
    b = DO / "track_sim_brick.mp4"
    if not a.exists() or not b.exists():
        raise SystemExit("缺视频，先录 brick 两版")

    ts = [2.2, 3.2, 12.0]
    fa, fb = frames(a, ts), frames(b, ts)
    labels = ["① 候选初现", "② 确认后锁定的目标", "③ 接近中"]

    f_t = ImageFont.truetype(FONT, 24)
    f_s = ImageFont.truetype(FONT, 18)
    cells = []
    for i in range(3):
        row = []
        for fr in (fa[i], fb[i]):
            if fr is None:
                row.append(None)
                continue
            im = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
            im = cv2.resize(im, (520, 293))
            row.append(Image.fromarray(im))
        cells.append(row)

    W = 520 * 2 + 40
    H = 60 + (293 + 34) * 3 + 30
    sheet = Image.new("RGB", (W, H), (16, 16, 16))
    d = ImageDraw.Draw(sheet)
    d.text((14, 10), "VLM 语义指认的价值：墙上有一块红砖时", font=f_t,
           fill=(0, 220, 255))
    d.text((14 + 520 + 20, 40), "有 VLM 指认", font=f_s, fill=(120, 230, 120))
    d.text((14, 40), "无 VLM（按尺寸选最大）", font=f_s, fill=(240, 130, 130))

    y = 66
    for i, row in enumerate(cells):
        d.text((14, y), labels[i], font=f_s, fill=(220, 220, 220))
        y += 24
        for j, im in enumerate(row):
            if im is None:
                continue
            sheet.paste(im, (14 + j * (520 + 20), y))
        y += 293 + 10

    d.text((14, H - 26),
           "无 VLM：锁到 38 px 的红砖 → 3.3 s 就误判“交接完成”。"
           "有 VLM：锁到 4.7 px 的红点（偏差 1 px）→ 16.2 s 正常交接。",
           font=f_s, fill=(255, 210, 120))

    OUT.parent.mkdir(exist_ok=True)
    sheet.save(OUT, quality=92)
    print(f"出图 {OUT}  ({sheet.width}x{sheet.height})")


if __name__ == "__main__":
    main()
