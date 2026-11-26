# -*- coding: utf-8 -*-
"""
pipeline_v2.py —— "CV 找候选 + VLM 判局部" 的完整链路测试。

为什么要裁剪（已被实验证明）：
    整图提问时，模型答的是"画面里有没有红色物体"，
    把目标涂掉它照样说"有"（会被红管道、贴纸、彩旗带偏）。

链路：
    CV：红度分割 → 连通域 → 圆度筛选 → 取前 K 个候选
    VLM：每个候选裁一块（放大）→ 闭集问 "A 标记 / B 其他红色物体"
    决策：任一个判 A ⇒ 有目标

用法：
    python pipeline_v2.py 图片目录
"""

import sys
import os
import csv
import json
import time
import base64
import tempfile
import urllib.request

import cv2
import numpy as np

HOST = "http://127.0.0.1:11434"
MODEL = "openbmb/minicpm-v4.7:1b"

# 真值：文件 -> (有没有, 红点中心坐标 or None)
TRUTH = {
    "test1.jpg": ("无", None),
    "test2.jpg": ("有", (879, 770)),
    "test3.jpg": ("有", (882, 692)),
    "neg1.jpg": ("无", None),
    "neg2.jpg": ("无", None),
    "neg3.jpg": ("无", None),
    "neg4.jpg": ("无", None),
    "neg5.jpg": ("无", None),
}

QC = ("这是从无人机拍摄的画面里裁出来的局部。请只判断一件事："
      "画面正中心的那个红色物体，属于哪一种？\n"
      "A：贴在白纸或玻璃上的圆形标记\n"
      "B：其他红色物体（管道、砖、招牌、叶子、贴纸、灯）\n"
      "只回答 A 或 B。")

TOP_K = 3
EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def fix_console():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:
            pass
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


def imread_u(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def find_candidates(bgr, topk=TOP_K):
    """返回 [(cx, cy, half, area, circ), ...]，按"越像圆点越靠前"排"""
    h, w = bgr.shape[:2]
    b, g, r = cv2.split(bgr.astype(np.int16))
    mask = ((r - np.maximum(g, b)) > 40).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(mask, 8)

    cand = []
    for i in range(1, n):
        x, y, ww, hh, area = st[i]
        if area < 80:
            continue
        if area > 0.03 * w * h:          # 太大，不是"点"
            continue
        sub = (lab[y:y + hh, x:x + ww] == i).astype(np.uint8)
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        per = cv2.arcLength(c, True)
        if per < 1:
            continue
        circ = 4 * np.pi * cv2.contourArea(c) / (per * per)   # 1.0 = 正圆
        if circ < 0.55:                   # 长条（管道/砖缝/叶子）直接排除
            continue
        d_eq = np.sqrt(4 * area / np.pi)
        cx, cy = int(cen[i][0]), int(cen[i][1])
        half = int(max(48, min(240, d_eq * 2.2)))
        cand.append((cx, cy, half, area, circ, d_eq))

    # 打分：圆 + 不太小
    cand.sort(key=lambda t: -(t[4] * min(1.0, t[5] / 12.0)))
    return cand[:topk]


def crop_for(bgr, cx, cy, half, out_px=384):
    h, w = bgr.shape[:2]
    x0, y0 = max(0, cx - half), max(0, cy - half)
    x1, y1 = min(w, cx + half), min(h, cy + half)
    c = bgr[y0:y1, x0:x1]
    return cv2.resize(c, (out_px, out_px), interpolation=cv2.INTER_LANCZOS4)


def ask(crop_bgr, prompt=QC, timeout=600):
    ok, buf = cv2.imencode(".jpg", crop_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    payload = {"model": MODEL, "prompt": prompt, "images": [b64], "stream": False,
               "options": {"temperature": 0}, "keep_alive": "30m", "think": False}
    req = urllib.request.Request(HOST + "/api/generate",
                                 data=json.dumps(payload).encode(),
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    r = json.loads(urllib.request.urlopen(req, timeout=timeout).read())
    return (r.get("response") or "").strip(), time.time() - t0


def main():
    fix_console()
    folder = sys.argv[1] if len(sys.argv) > 1 else r"D:\Desktop\无人系统\MiniCPM-V\test"
    files = [f for f in sorted(os.listdir(folder))
             if f.lower().endswith(EXTS) and f in TRUTH]

    rows = []
    hit = 0
    for f in files:
        p = os.path.join(folder, f)
        bgr = imread_u(p)
        truth, dot = TRUTH[f]
        cands = find_candidates(bgr)
        print(f"\n{f}  真值={truth}  候选 {len(cands)} 个")
        pred = "无"
        detail = []
        for k, (cx, cy, half, area, circ, d_eq) in enumerate(cands):
            crop = crop_for(bgr, cx, cy, half)
            ans, dt = ask(crop)
            v = "A" if ans.strip()[:1].upper() == "A" else ("B" if ans.strip()[:1].upper() == "B" else "?")
            # 这个候选是不是真的红点？
            is_true = ""
            if dot:
                is_true = "★真点" if abs(cx - dot[0]) < 60 and abs(cy - dot[1]) < 60 else ""
            print(f"   #{k+1} ({cx},{cy}) 直径{d_eq:4.0f}px 圆度{circ:.2f} -> {v} [{dt:.1f}s] {is_true}")
            detail.append(f"{v}@{cx},{cy}")
            if v == "A":
                pred = "有"
        ok = (pred == truth)
        hit += ok
        print(f"   预测={pred}  {'✓' if ok else '✗'}")
        rows.append([f, truth, pred, "✓" if ok else "✗", len(cands), ";".join(detail)])

    print("\n" + "=" * 66)
    print(f"链路准确率：{hit}/{len(files)} = {hit/len(files)*100:.0f}%")
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "数据", "裁剪链路_20261102.csv")
    with open(out, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["图片", "真值", "预测", "对错", "候选数", "逐候选判定"])
        w.writerows(rows)
    print("已写：", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
