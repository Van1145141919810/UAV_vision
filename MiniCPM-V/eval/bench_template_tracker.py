# -*- coding: utf-8 -*-
"""验证 tracker_core：模板跟踪 + 光流预测，能否在"双点靠很近"时保持身份

场景构造与 demo_track_sim v2 一致：
  背景 = 真实立面图；目标 = 20 cm 红圆（按物理公式变大）
  可选：旁边 N 像素再放一个**同尺寸**红点（外观完全一样）
  第 8 秒遮挡 1.5 秒

对比两种跟踪方式：
  A 检测+关联（v2 的做法）
  B 模板 NCC + 光流预测（新）

判定：遮挡恢复后，跟踪框是否仍在**原来那个点**上（误差 <30 px 算对）。
"""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv.tracker_core import TemplateTracker              # noqa: E402

W, H = 960, 540
F1920 = 1566.1
REAL = 0.20
D_START, D_END, APPROACH_S = 50.0, 10.0, 13.0
FPS = 10.4


def draw_dot(frame, cx, cy, d, color=(30, 30, 205), S=4):
    r = max(0.6, d / 2)
    half = int(r) + 3
    size = 2 * half + 1
    x0, y0 = int(cx) - half, int(cy) - half
    if x0 < 0 or y0 < 0 or x0 + size > frame.shape[1] or y0 + size > frame.shape[0]:
        return
    big = np.zeros((size * S, size * S, 3), np.uint8)
    cv2.circle(big, (int(r * S), int(r * S)), int(round(r * S)), color, -1,
               lineType=cv2.LINE_AA)
    small = cv2.resize(big, (size, size), interpolation=cv2.INTER_AREA)
    m = small.any(axis=2)
    reg = frame[y0:y0 + size, x0:x0 + size]
    reg[m] = small[m]


def run(close_px, occlude_at=(8.0, 9.5), use_template=True, debug=False,
        cam_vx=0.0):
    base = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    base = cv2.resize(base, (W, H), interpolation=cv2.INTER_AREA)
    base = (base.astype(np.float32) * 0.80).clip(0, 255).astype(np.uint8)
    f = F1920 * W / 1920.0

    tr = TemplateTracker() if use_template else None
    locked = False
    errs = []
    t_cost = []
    dt = 1.0 / FPS
    t = 0.0
    tx0 = ty0 = None
    max_err = 0.0

    while t < 16.0:
        prog = min(1.0, max(0.0, (t - 2.0) / APPROACH_S))
        dist = D_START - (D_START - D_END) * prog
        d_px = f * REAL / dist
        # 相机运动：整帧平移（目标在画面里反向移动）
        shift = int(round(cam_vx * t))
        M = np.float32([[1, 0, shift], [0, 1, 0]])
        frame = cv2.warpAffine(base, M, (W, H), borderMode=cv2.BORDER_REPLICATE)
        tx = int(W * 0.62) - shift
        ty = int(H * 0.40)
        occ = occlude_at[0] <= t < occlude_at[1]
        if not occ:
            draw_dot(frame, tx, ty, d_px)
            if close_px > 0:
                draw_dot(frame, tx + close_px, ty, d_px)
        else:
            cv2.rectangle(frame, (tx - 20, ty - 20), (tx + 20, ty + 20),
                          (58, 62, 58), -1)
            if close_px > 0:
                draw_dot(frame, tx + close_px, ty, d_px)   # 另一个点仍可见

        t0 = time.perf_counter()
        if use_template:
            if not locked and not occ and t > 2.5:
                tr.init(frame, (tx, ty), half_size=max(8, int(d_px)))
                locked = True
            if locked:
                if occ:
                    est = tr.predict(frame, dt)
                    score = 0.0
                else:
                    ok, est, score = tr.update(frame, dt)
                    if not ok:
                        est = tr.predict(frame, dt)
                err = math.hypot(est[0] - tx, est[1] - ty)
                if t > occlude_at[1] + 1.0:      # 只看恢复后 1 秒
                    errs.append(err)
                max_err = max(max_err, err)
                if debug and abs(round(t, 1) - round(t, 0)) < dt / 2:
                    print(f"    t={t:5.1f} occ={int(occ)} est=({est[0]:.0f},"
                          f"{est[1]:.0f}) true=({tx},{ty}) err={err:5.1f} "
                          f"score={score:.2f}")
        t_cost.append((time.perf_counter() - t0) * 1000)
        t += dt

    if not errs:
        return None
    # 只看遮挡结束 1 秒后的误差（恢复情况）
    return {"close": close_px, "max_err": max_err,
            "mean_ms": float(np.mean(t_cost))}


import math  # noqa: E402

if __name__ == "__main__":
    print("模板 NCC + 光流预测：双点靠很近时能不能保持身份")
    print(f"{'双点间距':<10}{'最大误差px':<12}{'平均耗时ms':<12}{'判定'}")
    print("-" * 52)
    cases = [(0, 0.0), (25, 0.0), (0, 1.0), (0, 2.0), (0, 6.0), (25, 2.0)]
    for close, vx in cases:
        r = run(close, use_template=True, cam_vx=vx)
        if not r:
            print(f"{close:<10}无数据")
            continue
        ok = "OK" if r["max_err"] < 30 else "FAIL"
        print(f"close={close:<4} vx={vx:<5}{r['max_err']:<12.0f}"
              f"{r['mean_ms']:<12.1f}{ok}")
