# -*- coding: utf-8 -*-
"""gdino_window.py —— 路 C：用 Grounding DINO（零样本，文字提示）找窗户 / 玻璃幕墙

模型 IDEA-Research/grounding-dino-tiny（~172M），不经训练，只给文字提示词：
    "window. glass window. curtain wall. glass facade."
就能出框。这是"理解窗户这个概念"最直接的做法。

跑三处
  · 43.0 m 真实帧（帧 246）—— 幕墙玻璃门场景
  · 40.8 m 真实帧（帧 254）
  · 立面照 win06 缩到窗宽 72 px（= 45 m 预算）—— 规整墙窗场景

输出
  eval/gdino.json + eval/figures/gdino_*.jpg
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "eval"))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, r"D:\Desktop\无人系统\_ref\s14")

from PIL import Image                                        # noqa: E402
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor  # noqa: E402

MODEL_ID = "IDEA-Research/grounding-dino-tiny"
PROMPT = "window. glass window. curtain wall. glass door. facade."
OUT_JSON = ROOT / "eval" / "gdino.json"
OUT_DIR = ROOT / "eval" / "figures"


def imwrite_u(path, img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if ok:
        Path(path).write_bytes(buf.tobytes())
    return ok


def detect(model, proc, bgr, box_thr=0.20, text_thr=0.20):
    h, w = bgr.shape[:2]
    img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    inputs = proc(images=img, text=PROMPT, return_tensors="pt")
    t0 = time.time()
    import torch
    with torch.no_grad():
        out = model(**inputs)
    dt = (time.time() - t0) * 1000
    try:
        res = proc.post_process_grounded_object_detection(
            out, inputs["input_ids"], threshold=box_thr, text_threshold=text_thr,
            target_sizes=[(h, w)])
    except TypeError:
        res = proc.post_process_grounded_object_detection(
            out, inputs["input_ids"], box_threshold=box_thr, text_threshold=text_thr,
            target_sizes=[(h, w)])
    r = res[0] if res else {}
    boxes = r.get("boxes")
    scores = r.get("scores")
    labels = r.get("labels") or r.get("text_labels") or []
    out_list = []
    if boxes is not None:
        for i in range(len(boxes)):
            b = [float(v) for v in boxes[i].tolist()]
            out_list.append(dict(box=[round(v, 1) for v in b],
                                 score=round(float(scores[i]), 3),
                                 label=str(labels[i]) if i < len(labels) else "?"))
    out_list.sort(key=lambda d: -d["score"])
    return out_list, dt


def draw(bgr, dets, title):
    vis = bgr.copy()
    h, w = vis.shape[:2]
    for d in dets[:14]:
        x1, y1, x2, y2 = [int(v) for v in d["box"]]
        col = (0, 255, 0) if d["score"] >= 0.30 else (0, 200, 255)
        cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
        t = "%s %.2f" % (d["label"][:14], d["score"])
        cv2.putText(vis, t, (x1 + 3, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    cv2.rectangle(vis, (0, 0), (w, 40), (0, 0, 0), -1)
    cv2.putText(vis, title, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    return vis


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print("加载 %s …" % MODEL_ID, flush=True)
    t0 = time.time()
    proc = AutoProcessor.from_pretrained(MODEL_ID)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID)
    model.eval()
    print("  加载完成 %.0f s" % (time.time() - t0), flush=True)

    import bench_lat_crop as L
    from bench_window_range import scale_to_window, window_w
    from mcpv.cv import imread_unicode

    rows = []
    jobs = []
    fr = L.load_frames()
    for k, lab in ((246, "43.0 m 玻璃门"), (254, "40.8 m 玻璃门")):
        if k in fr:
            jobs.append(("frame%d_%s" % (k, lab.replace(" ", "")), fr[k]["clean"],
                         fr[k]["gx"], fr[k]["gy"]))
    wf = ROOT / "data" / "samples" / "win06.jpg"
    if wf.exists():
        im = imread_unicode(wf)
        med, _ = window_w(im)
        sc = scale_to_window(im, med, 72)
        jobs.append(("win06_72px", sc, None, None))

    for name, img, gx, gy in jobs:
        dets, dt = detect(model, proc, img)
        h, w = img.shape[:2]
        hit = None
        if gx is not None:
            hit = any(d["box"][0] - 8 <= gx <= d["box"][2] + 8 and
                      d["box"][1] - 8 <= gy <= d["box"][3] + 8 for d in dets)
        big = [d for d in dets if d["score"] >= 0.30]
        print("\n[%s] %dx%d  %.0f ms  总框 %d  高置信(≥0.30) %d  含目标: %s"
              % (name, w, h, dt, len(dets), len(big), hit), flush=True)
        for d in dets[:6]:
            print("     %-14s %.3f  box=%s" % (d["label"], d["score"], d["box"]), flush=True)
        m = max(1, w // 1280)
        small = img[::m, ::m]
        dd = [dict(d, box=[v / m for v in d["box"]]) for d in dets]
        imwrite_u(OUT_DIR / ("gdino_%s.jpg" % name),
                  draw(small, dd, "Grounding DINO  %s  %d boxes  %.0fms" % (name, len(dets), dt)))
        rows.append(dict(name=name, w=w, h=h, ms=round(dt, 1), n=len(dets),
                         n_big=len(big), hit=hit, dets=dets[:20]))

    json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\n结果 -> %s" % OUT_JSON, flush=True)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")
    main()
