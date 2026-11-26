# -*- coding: utf-8 -*-
"""静态检测：抓一帧 → 双路检测 → 标注图 + 候选表

输出：
  eval/demo_out/static_check.jpg    核对图（全画面 + 放大 + 表格）
  eval/demo_out/static_raw.jpg      原始帧
"""
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imwrite_unicode                        # noqa: E402
from mcpv.detect_live import detect_reds                   # noqa: E402

DRONE = "visbot@192.168.2.20"
IP, PORT = "192.168.2.20", 8890
KEY = str(Path.home() / ".ssh" / "dsh_drone")
FONT = r"C:\Windows\Fonts\msyh.ttc"
OUT = ROOT / "eval" / "demo_out"

FX, FY, CX, CY = 1566.149, 1573.382, 932.277, 517.261
MARKER_M = 0.20


def recv_n(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise ConnectionError
        b += c
    return b


def grab(n=8):
    s = socket.create_connection((IP, PORT), timeout=8)
    fr = None
    for _ in range(n):
        ln = struct.unpack(">I", recv_n(s, 4))[0]
        data = recv_n(s, ln)
        fr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    s.close()
    return fr


def main():
    # 确保推流在跑
    try:
        try:
            fr = grab(2)
        except Exception:
            print("推流没起，自动启动…")
            subprocess.run(["ssh", "-i", KEY, "-o", "StrictHostKeyChecking=no",
                            DRONE, "bash ~/cpmvbyhuang/start_grabd.sh hd"],
                           capture_output=True, timeout=40)
            time.sleep(8)
            fr = grab(6)
    except Exception as e:
        print("抓帧失败:", e)
        return 1

    H, W = fr.shape[:2]
    k = W / 1920.0
    fx = FX * k
    cands = detect_reds(fr)
    imwrite_unicode(OUT / "static_raw.jpg", fr, quality=95)

    print(f"画面 {W}x{H}   焦距 fx={fx:.1f}   红圆 {MARKER_M*100:.0f} cm")
    print(f"检出候选 {len(cands)} 个")
    print(f"{'#':<3}{'d_px':<8}{'距离m':<9}{'圆度':<7}{'外圈V/S':<11}"
          f"{'白纸分':<8}{'来源':<8}{'中心'}")
    for i, q in enumerate(cands[:8], 1):
        d = q["d"]
        dist = fx * MARKER_M / d if d > 0 else 0
        src = "红度" if q.get("lowconf") else "HSV"
        print(f"{i:<3}{d:<8.1f}{dist:<9.1f}{q['fill']:<7.2f}"
              f"{q['ring_v']:.0f}/{q['ring_s']:.0f}".ljust(11)
              + f"{q['paper']:<8.2f}{src:<8}({q['cx']:.0f},{q['cy']:.0f})")

    # ── 出图 ─────────────────────────────────────────────
    vis = fr.copy()
    for i, q in enumerate(cands[:8], 1):
        x, y, w, h = q["box"]
        col = (80, 240, 90) if i == 1 else (
            (60, 200, 255) if q.get("lowconf") else (150, 170, 190))
        pad = max(6, int(q["d"] * 0.7))
        cv2.rectangle(vis, (x - pad, y - pad), (x + w + pad, y + h + pad), col, 2)
        dist = fx * MARKER_M / q["d"] if q["d"] > 0 else 0
        cv2.putText(vis, f"{i}:{q['d']:.0f}px {dist:.0f}m",
                    (x - pad, y - pad - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    col, 1)

    f_t = ImageFont.truetype(FONT, 22)
    f_s = ImageFont.truetype(FONT, 16)
    lw = 980
    left = Image.fromarray(cv2.cvtColor(
        cv2.resize(vis, (lw, int(H * lw / W))), cv2.COLOR_BGR2RGB))

    if cands:
        c0 = cands[0]
        cx, cy = int(c0["cx"]), int(c0["cy"])
        Z, ZS = 130, 4
        hh = Z // 2
        x0, y0 = max(0, cx - hh), max(0, cy - hh)
        crop = fr[y0:y0 + Z, x0:x0 + Z]
        big = cv2.resize(crop, (Z * ZS, Z * ZS), interpolation=cv2.INTER_NEAREST)
        cv2.drawMarker(big, ((cx - x0) * ZS, (cy - y0) * ZS), (80, 240, 90),
                       cv2.MARKER_CROSS, 30, 1)
        cv2.circle(big, ((cx - x0) * ZS, (cy - y0) * ZS),
                   int(c0["d"] * ZS / 2), (60, 220, 255), 1)
        right = Image.fromarray(cv2.cvtColor(big, cv2.COLOR_BGR2RGB))
    else:
        right = Image.new("RGB", (520, 520), (30, 30, 30))
        ImageDraw.Draw(right).text((10, 10), "未检出任何候选", font=f_s,
                                   fill=(240, 120, 120))

    n = min(len(cands), 6)
    tbl = Image.new("RGB", (right.width, 30 * (n + 2) + 8), (22, 22, 22))
    d = ImageDraw.Draw(tbl)
    d.text((6, 2), "候选（按白纸分排序）", font=f_s, fill=(0, 220, 255))
    for i, h in enumerate(["#", "直径", "距离", "圆度", "来源"]):
        d.text((6 + [0, 26, 96, 176, 250][i], 26), h, font=f_s,
               fill=(200, 220, 240))
    for r in range(n):
        q = cands[r]
        y = 48 + r * 30
        col = (120, 230, 120) if r == 0 else (215, 215, 215)
        dist = fx * MARKER_M / q["d"] if q["d"] > 0 else 0
        vals = [str(r + 1), f"{q['d']:.1f}px", f"{dist:.1f}m",
                f"{q['fill']:.2f}", "红度" if q.get("lowconf") else "HSV"]
        for i, v in enumerate(vals):
            d.text((6 + [0, 26, 96, 176, 250][i], y), v, font=f_s, fill=col)

    Wt = lw + max(right.width, tbl.width) + 40
    Ht = 74 + max(left.height, right.height + tbl.height + 12) + 12
    sheet = Image.new("RGB", (Wt, Ht), (16, 16, 16))
    dd = ImageDraw.Draw(sheet)
    dd.text((12, 8), f"静态检测  {datetime.now():%Y-%m-%d %H:%M:%S}   "
                     f"{W}x{H}   红圆{MARKER_M*100:.0f}cm", font=f_t,
            fill=(0, 220, 255))
    if cands:
        c0 = cands[0]
        dd.text((12, 40),
                f"共 {len(cands)} 个候选；绿框=排第一  d={c0['d']:.1f}px  "
                f"≈{fx*MARKER_M/c0['d']:.1f} m  "
                f"({'红度路' if c0.get('lowconf') else 'HSV路'})",
                font=f_s, fill=(180, 210, 230))
    else:
        dd.text((12, 40), "未检出任何候选", font=f_s, fill=(240, 120, 120))
    sheet.paste(left, (12, 70))
    sheet.paste(right, (lw + 26, 70))
    sheet.paste(tbl, (lw + 26, 70 + right.height + 10))
    p = OUT / "static_check.jpg"
    sheet.save(p, quality=93)
    print(f"\n核对图: {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
