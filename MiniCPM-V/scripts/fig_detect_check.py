# -*- coding: utf-8 -*-
"""出一张"检测效果核对图"：全画面 + 目标放大 + 候选表（统一用 MJPG hd 帧）"""
import socket
import struct
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imwrite_unicode                          # noqa: E402
from mcpv.detect_live import detect_reds                     # noqa: E402

FONT = r"C:\Windows\Fonts\msyh.ttc"
OUT = ROOT / "eval" / "demo_out"
DRONE_IP, PORT = "192.168.2.20", 8890


def recv_n(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise ConnectionError
        b += c
    return b


def grab():
    s = socket.create_connection((DRONE_IP, PORT), timeout=8)
    fr, ln = None, 0
    for _ in range(8):
        ln = struct.unpack(">I", recv_n(s, 4))[0]
        data = recv_n(s, ln)
        fr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    s.close()
    return fr, ln


def main():
    frame, jlen = grab()
    H, W = frame.shape[:2]
    f_px = 1566.1 * W / 1920.0
    cands = detect_reds(frame)
    imwrite_unicode(OUT / "live_hd_raw.jpg", frame, quality=95)

    f_t = ImageFont.truetype(FONT, 26)
    f_s = ImageFont.truetype(FONT, 18)
    f_c = ImageFont.truetype(FONT, 17)

    # 左：全画面 + 框
    vis = frame.copy()
    for i, q in enumerate(cands, 1):
        x, y, w, h = q["box"]
        col = (80, 240, 90) if i == 1 else (60, 220, 255)
        pad = max(8, int(q["d"] * 0.8))
        cv2.rectangle(vis, (x - pad, y - pad), (x + w + pad, y + h + pad), col, 4)
        cv2.putText(vis, f"{i}:{q['d']:.0f}px", (x - pad, y - pad - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 3)
    lw = 980
    left = Image.fromarray(cv2.cvtColor(
        cv2.resize(vis, (lw, int(H * lw / W))), cv2.COLOR_BGR2RGB))

    # 右：目标放大 4 倍
    ZOOM, ZS = 150, 4
    c0 = cands[0]
    cx, cy = int(c0["cx"]), int(c0["cy"])
    hh = ZOOM // 2
    x0, y0 = max(0, cx - hh), max(0, cy - hh)
    crop = frame[y0:y0 + ZOOM, x0:x0 + ZOOM]
    big = cv2.resize(crop, (ZOOM * ZS, ZOOM * ZS),
                     interpolation=cv2.INTER_NEAREST)
    cxs, cys = (cx - x0) * ZS, (cy - y0) * ZS
    cv2.drawMarker(big, (cxs, cys), (80, 240, 90), cv2.MARKER_CROSS, 36, 2)
    cv2.circle(big, (cxs, cys), int(c0["d"] * ZS / 2), (60, 220, 255), 2)
    right = Image.fromarray(cv2.cvtColor(big, cv2.COLOR_BGR2RGB))

    # 右下：候选表
    nrows = min(len(cands), 5)
    tbl = Image.new("RGB", (right.width, 32 * (nrows + 2) + 8), (22, 22, 22))
    d = ImageDraw.Draw(tbl)
    d.text((8, 2), "检测候选（按白纸分排序）", font=f_s, fill=(0, 220, 255))
    heads = ["#", "d_px", "圆度", "外圈V/S", "白纸分"]
    xs = [8, 38, 96, 152, 244]
    for i, h in enumerate(heads):
        d.text((xs[i], 28), h, font=f_c, fill=(200, 220, 240))
    for r in range(nrows):
        q = cands[r]
        y = 50 + r * 32
        col = (120, 230, 120) if r == 0 else (215, 215, 215)
        vals = [f"{r+1}", f"{q['d']:.1f}", f"{q['fill']:.2f}",
                f"{q['ring_v']:.0f}/{q['ring_s']:.0f}", f"{q['paper']:.2f}"]
        for i, v in enumerate(vals):
            d.text((xs[i], y), v, font=f_c, fill=col)

    Wtot = lw + max(right.width, tbl.width) + 42
    Htot = 78 + max(left.height, right.height + tbl.height + 14) + 14
    sheet = Image.new("RGB", (Wtot, Htot), (16, 16, 16))
    dd = ImageDraw.Draw(sheet)
    dd.text((14, 8), "真机检测效果核对（1920x1080 MJPG，10.4 fps，单帧 260 KB）",
            font=f_t, fill=(0, 220, 255))
    dd.text((14, 42),
            f"检出 {len(cands)} 个候选；绿框=排第一（白纸红圆 d={c0['d']:.1f}px）"
            f"   若红圆 20 cm → 约 {f_px*0.20/c0['d']:.1f} m；"
            f"26 cm → 约 {f_px*0.26/c0['d']:.1f} m",
            font=f_s, fill=(180, 210, 230))
    sheet.paste(left, (14, 74))
    sheet.paste(right, (lw + 28, 74))
    sheet.paste(tbl, (lw + 28, 74 + right.height + 12))

    out = OUT / "detect_check.jpg"
    sheet.save(out, quality=93)
    print(f"出图 {out}  ({sheet.width}x{sheet.height})")
    print(f"红点 d={c0['d']:.1f}px 中心({cx},{cy})")


if __name__ == "__main__":
    main()
