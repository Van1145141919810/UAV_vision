# -*- coding: utf-8 -*-
"""exp_window_finder.py —— 三路融合定位器的效果图 + 定量实验

产出
  · eval/figures/wf_panel_<case>.jpg   每个用例一张 2×2 面板
        左上 红点检测（红圈）     右上 SegFormer 窗户掩膜（绿）
        左下 Grounding DINO 框（彩）  右下 融合结果（绿框 + 一行字）
  · eval/figures/wf_all.jpg            所有用例纵向拼成一张总图
  · eval/wf_exp.json                   逐用例明细
  · 屏幕表格：出框率 / 框含红点率 / 中心误差 / 距离误差 / 耗时分解

用例
  帧 246 (43.0 m)、254 (40.8 m)、303 (21.7 m) —— 真实录像（玻璃门场景）
  win06、win09（缩到窗宽 72 px = 45 m 预算）—— 白天规整居民楼

用法
    python eval\\exp_window_finder.py
"""
from __future__ import annotations

import base64
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "eval"))

from mcpv.window_finder import WindowFinder, draw_result      # noqa: E402
import bench_lat_crop as L                                    # noqa: E402
from bench_window_range import scale_to_window, window_w      # noqa: E402
from mcpv.cv import imread_unicode                            # noqa: E402

FIG = ROOT / "eval" / "figures"
OUT_JSON = ROOT / "eval" / "wf_exp.json"
PANEL_W = 960          # 每格宽


def imwrite_u(path, img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if ok:
        Path(path).write_bytes(buf.tobytes())
    return ok


def tile(img, w=PANEL_W):
    h = int(round(img.shape[0] * w / float(img.shape[1])))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


def label(img, txt, col=(0, 255, 255)):
    cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(img, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
    return img


def panel(bgr, r, gx, gy):
    h, w = bgr.shape[:2]
    # ① 红点
    p1 = bgr.copy()
    if r.get("dot"):
        cv2.circle(p1, (int(r["dot"]["u"]), int(r["dot"]["v"])),
                   max(12, int(r["dot"]["d_px"] * 1.2)), (0, 0, 255), 3)
    if gx is not None:
        cv2.drawMarker(p1, (int(gx), int(gy)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
    label(p1, "1) red-dot detect  n=%s  %s ms" % (
        (r.get("dot") or {}).get("n", 0), (r.get("ms") or {}).get("dot", "-")))
    # ② SegFormer 掩膜
    p2 = bgr.copy()
    m = None
    if r.get("seg_mask_png"):
        buf = np.frombuffer(base64.b64decode(r["seg_mask_png"]), np.uint8)
        m = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if m is not None and m.shape[:2] == (h, w):
            p2[m > 127] = (p2[m > 127] * 0.35 + np.array([0, 255, 0]) * 0.65).astype(np.uint8)
    label(p2, "2) SegFormer window mask  blobs=%d  %s ms" % (
        len(r.get("seg_blobs") or []), (r.get("ms") or {}).get("seg", "-")))
    # ③ Grounding DINO
    p3 = bgr.copy()
    for i, b in enumerate((r.get("gd_boxes") or [])[:8]):
        x1, y1, x2, y2, sc, lab = b[:6]
        col = (0, 255, 0) if sc >= 0.30 else (0, 200, 255)
        cv2.rectangle(p3, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
        cv2.putText(p3, "%s %.2f" % (str(lab)[:16], sc), (int(x1) + 3, max(16, int(y1) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    if gx is not None:
        cv2.drawMarker(p3, (int(gx), int(gy)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
    label(p3, "3) Grounding DINO (zero-shot)  boxes=%d  %s ms" % (
        len(r.get("gd_boxes") or []), (r.get("ms") or {}).get("gdino", "-")))
    # ④ 融合
    p4 = draw_result(bgr, r, show_note=True)
    if gx is not None:
        cv2.drawMarker(p4, (int(gx), int(gy)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
    label(p4, "4) FUSED  src=%s  dist=%s m  %s ms" % (
        r.get("source"), r.get("dist_m"), (r.get("ms") or {}).get("total", "-")))
    top = np.hstack([tile(p1), tile(p2)])
    bot = np.hstack([tile(p3), tile(p4)])
    return np.vstack([top, bot])


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    print("启动三路融合定位器（第一次要加载两个模型，约 40 s）…", flush=True)
    t0 = time.time()
    wf = WindowFinder()
    ready = wf.wait_ready(300)
    print("预热 %s，用时 %.1f s" % ("完成" if ready else "失败", time.time() - t0), flush=True)

    # 组用例
    cases = []
    fr = L.load_frames()
    for k, lab in ((246, "43.0m"), (254, "40.8m"), (303, "21.7m")):
        if k in fr:
            cases.append((("frame_%s" % lab), fr[k]["clean"], fr[k]["gx"], fr[k]["gy"],
                          L.FX * L.DOT_D / fr[k]["d"]))
    for n in ("win06", "win09"):
        f = ROOT / "data" / "samples" / (n + ".jpg")
        if f.exists():
            im = imread_unicode(f)
            med, _ = window_w(im)
            img = scale_to_window(im, med, 72)
            cases.append(("photo_%s_72px" % n, img, None, None, None))

    rows, panels = [], []
    for name, img, gx, gy, truth_z in cases:
        t1 = time.time()
        r = wf.find(img, want_mask=True)
        wall = (time.time() - t1) * 1000
        row = dict(case=name, source=r["source"], ok=r["ok"], box=r["box"],
                   center=r["center"], dist_m=r["dist_m"], truth_z=truth_z,
                   height_m=r["height_m"], floor=r["floor"], text=r["text"],
                   ms=r.get("ms"), wall_ms=round(wall, 1),
                   n_dot=(r.get("dot") or {}).get("n", 0),
                   n_seg=len(r.get("seg_blobs") or []),
                   n_gd=len(r.get("gd_boxes") or []))
        if gx is not None and r.get("box"):
            x1, y1, x2, y2 = r["box"]
            row["box_has_dot"] = bool(x1 - 10 <= gx <= x2 + 10 and y1 - 10 <= gy <= y2 + 10)
            row["center_err_px"] = round(float(np.hypot(r["center"][0] - gx, r["center"][1] - gy)), 1)
        if truth_z and r.get("dist_m"):
            row["dist_err_pct"] = round(100.0 * (r["dist_m"] - truth_z) / truth_z, 1)
        rows.append(row)
        p = panel(img, r, gx, gy)
        imwrite_u(FIG / ("wf_panel_%s.jpg" % name), p)
        panels.append((name, p))
        print("[%-16s] src=%-11s dist=%-6s (真值 %-6s) 高度=%-6s %-6s  %d ms"
              % (name, r["source"], r["dist_m"], truth_z, r["height_m"], r["floor"],
                 row["wall_ms"]), flush=True)
        print("     %s" % r["text"], flush=True)

    # 总图
    cols = [tile(p, 1400) for _, p in panels]
    if cols:
        H = sum(c.shape[0] for c in cols) + 8 * len(cols)
        canvas = np.zeros((H, 1400, 3), np.uint8)
        y = 0
        for c in cols:
            canvas[y:y + c.shape[0]] = c
            y += c.shape[0] + 8
        imwrite_u(FIG / "wf_all.jpg", canvas)
        print("\n总图 -> %s" % (FIG / "wf_all.jpg"), flush=True)

    json.dump(rows, open(OUT_JSON, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    # 汇总
    print("\n" + "=" * 78)
    print("%-16s %-12s %-8s %-10s %-12s %-10s" %
          ("用例", "来源", "出框", "框含红点", "中心误差px", "距离误差%"))
    for r in rows:
        print("%-16s %-12s %-8s %-10s %-12s %-10s" %
              (r["case"], r["source"], "是" if r["ok"] else "否",
               r.get("box_has_dot", "-"), r.get("center_err_px", "-"),
               r.get("dist_err_pct", "-")))
    ms = [r["ms"] for r in rows if r.get("ms")]
    if ms:
        for k in ("dot", "seg", "gdino", "total"):
            v = [m.get(k) for m in ms if m.get(k) is not None]
            if v:
                print("平均耗时 %-6s %6.0f ms" % (k, sum(v) / len(v)))
    print("\n明细 -> %s" % OUT_JSON)
    wf.close()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
