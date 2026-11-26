# -*- coding: utf-8 -*-
"""bench_vlm_cv_gate.py —— 实测"VLM 定格子 + CV 精确找"能不能出框

═══════════════════════════════════════════════════════════════════
用户提出的方案（这条才是对的路）
───────────────────────────────────────────────────────────────────
  VLM 先说出红点在九宫格的哪一格 → 把那一格裁下来 → CV 在里面找红点

但有一个容易被忽略的点：
  **裁剪只缩小搜索范围，不改变红点的像素尺寸。**
  50 m 处红点仍是 4.7 px，裁下来还是 4.7 px。

真正的收益来自两处：
  1. 搜索面积从 1920×1080 降到约 640×360（**误报少一多半**）
  2. 裁下来后可以**放大 3 倍**再跑 CV（4.7 px → 14 px），
     面积阈值和连通域才有意义

═══════════════════════════════════════════════════════════════════
三种做法对比（同一张 50 m 合成图）
───────────────────────────────────────────────────────────────────
  A 全图直接 CV（现有做法）
  B VLM 定格子 → 裁原图区域 → CV
  C VLM 定格子 → 裁原图区域 → 放大 3 倍 → CV      ← 预期最好

评价标准：能不能找到 + 中心误差多少像素
═══════════════════════════════════════════════════════════════════
"""
import base64
import json
import subprocess
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                        # noqa: E402
from mcpv import config as C                             # noqa: E402

F = float(C._K1[0])
REAL = float(C.MARKER_REAL_M)
W, H = 1920, 1080
MODEL = "openbmb/minicpm-v4.7:1b"
GRID = [["左上", "上", "右上"], ["左", "中", "右"], ["左下", "下", "右下"]]


def red_blob(bgr, min_area=3):
    """低阈值找红色连通域，返回最大块的中心与面积。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 40), (14, 255, 255)),
        cv2.inRange(hsv, (166, 70, 40), (180, 255, 255)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i][4])
        if a < min_area:
            continue
        if best is None or a > best[0]:
            best = (a, float(cen[i][0]), float(cen[i][1]))
    return best


def ask_vlm(img_bgr, prompt):
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0}}).encode()
    req = urllib.request.Request("http://127.0.0.1:11434/api/generate",
                                 data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read()).get("response", "").strip()


def cell_of(x, y):
    return GRID[min(2, int(y / (H / 3)))][min(2, int(x / (W / 3)))]


def cell_bbox(name):
    r = next(i for i in range(3) if name in GRID[i])
    c = GRID[r].index(name)
    return (int(c * W / 3), int(r * H / 3), int(W / 3), int(H / 3))


def main():
    bg = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_AREA)
    bg = (bg.astype(np.float32) * 0.72).clip(0, 255).astype(np.uint8)

    P = ("画面被分成九宫格（上/中/下 × 左/中/右）。"
         "图里有一个很小的红色圆点。它在哪个格子里？"
         "只回答格子名称，例如：左上。")

    for D in (50, 80):
        d = F * REAL / D
        tx, ty = 1180, 420          # 故意不放在画面中心
        frame = bg.copy()
        cv2.circle(frame, (tx, ty), max(1, int(round(d / 2))),
                   (30, 30, 205), -1, lineType=cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        print("=" * 70)
        print(f"距离 {D} m   红点直径 {d:.1f} px   真实位置 ({tx},{ty})"
              f"   真实格子 {cell_of(tx, ty)}")
        print("=" * 70)

        # A 全图直接 CV
        b = red_blob(frame, min_area=5)
        if b:
            a_, cx, cy = b
            print(f"  A 全图CV      : 找到 面积{a_}px 中心({cx:.0f},{cy:.0f}) "
                  f"误差 {((cx-tx)**2+(cy-ty)**2)**0.5:.0f} px")
        else:
            print("  A 全图CV      : 未找到")

        # VLM 定格子
        small = cv2.resize(frame, (1600, 900), interpolation=cv2.INTER_AREA)
        ans = ask_vlm(small, P)
        # ★ 解析修正：必须先找三字格，否则"右上"里的"上"会被先匹配到
        three = ["左上", "右上", "左下", "右下"]
        cell = next((g for g in three if g in ans), None)
        if cell is None:
            one = ["上", "中", "下", "左", "右"]
            cell = next((g for g in one if g in ans), "?")
        print(f"  VLM 说        : {cell}   （原话 {ans[:20]!r}）")

        if cell == "?":
            continue
        x0, y0, cw, ch = cell_bbox(cell)
        crop = frame[y0:y0 + ch, x0:x0 + cw]

        # B 裁下来直接 CV
        b = red_blob(crop, min_area=3)
        if b:
            a_, cx, cy = b
            gx, gy = x0 + cx, y0 + cy
            print(f"  B 裁+CV       : 找到 面积{a_}px 中心({gx:.0f},{gy:.0f}) "
                  f"误差 {((gx-tx)**2+(gy-ty)**2)**0.5:.0f} px")
        else:
            print("  B 裁+CV       : 未找到")

        # C 裁 + 放大 3 倍 + CV
        big = cv2.resize(crop, (cw * 3, ch * 3), interpolation=cv2.INTER_CUBIC)
        b = red_blob(big, min_area=10)
        if b:
            a_, cx, cy = b
            gx, gy = x0 + cx / 3, y0 + cy / 3
            print(f"  C 裁+放大3x+CV: 找到 面积{a_}px 中心({gx:.0f},{gy:.0f}) "
                  f"误差 {((gx-tx)**2+(gy-ty)**2)**0.5:.0f} px")
            out = ROOT / "eval" / "demo_out" / f"gate_{D}m.jpg"
            out.parent.mkdir(parents=True, exist_ok=True)
            vis = frame.copy()
            cv2.rectangle(vis, (int(gx - d), int(gy - d)),
                          (int(gx + d), int(gy + d)), (0, 255, 0), 2)
            cv2.rectangle(vis, (x0, y0), (x0 + cw, y0 + ch), (255, 200, 0), 2)
            cv2.imwrite(str(out), vis)
        else:
            print("  C 裁+放大3x+CV: 未找到")
        print()


if __name__ == "__main__":
    main()
