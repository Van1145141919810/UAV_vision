# -*- coding: utf-8 -*-
"""bench_single_target.py —— 单目标：正常流程 + 各类异常的系统测试

═══════════════════════════════════════════════════════════════════
测什么（只测单目标）
───────────────────────────────────────────────────────────────────
  A 正常流程                     50 m → 10 m，无任何干扰
  B 短时遮挡   0.3 / 0.5 / 1.0 / 2.0 / 3.0 / 5.0 秒
  C 部分遮挡   只露出半个圆
  D 光照变暗   目标区域整体压暗 50%
  E 强反光     目标旁边出现一块高亮
  F 画面抖动   高频小位移（模拟风扰）
  G 相机平移   2 px/s（对应丢失后减速的工况）
  H 目标出画   移出边缘 1 秒再回来
  I 尺寸跳变   接近速度突然加倍

═══════════════════════════════════════════════════════════════════
评价
───────────────────────────────────────────────────────────────────
  · 是否走到 HANDOFF（能完成）
  · 恢复后 1 秒内的最大定位误差（px）
  · 是否跟丢后跟到别处（误差 > 60 px 判"跟错"）
  · 跟踪器耗时
═══════════════════════════════════════════════════════════════════
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from mcpv.cv import imread_unicode                         # noqa: E402
from mcpv.tracker_core import TemplateTracker              # noqa: E402

W, H = 960, 540
F1920, REAL = 1566.1, 0.20
D0, D1, T_APP = 50.0, 10.0, 13.0
FPS = 10.4
HANDOFF_1920 = 30.0
NOISE_SIGMA = 3.0      # 传感器噪声标准差（0=关闭）


def draw_dot(frame, cx, cy, d, color=(30, 30, 205), S=4, frac=1.0):
    """frac<1 时只画一部分（模拟部分遮挡）。"""
    r = max(0.6, d / 2)
    half = int(r) + 3
    size = 2 * half + 1
    x0, y0 = int(cx) - half, int(cy) - half
    if x0 < 0 or y0 < 0 or x0 + size > frame.shape[1] or y0 + size > frame.shape[0]:
        return
    big = np.zeros((size * S, size * S, 3), np.uint8)
    cc = int(round((cx - x0) * S))   # 圆心在 patch 内的真实位置
    cd = int(round((cy - y0) * S))
    cv2.circle(big, (cc, cd), int(round(r * S)), color, -1, lineType=cv2.LINE_AA)
    if frac < 1.0:
        cut = int(big.shape[1] * (1.0 - frac))
        big[:, :cut] = 0
    small = cv2.resize(big, (size, size), interpolation=cv2.INTER_AREA)
    m = small.any(axis=2)
    reg = frame[y0:y0 + size, x0:x0 + size]
    reg[m] = small[m]


def run(name, occl=(0.0, 0.0), partial=False, darken=False, glare=False,
        jitter=0.0, camvx=0.0, out_of_frame=0.0, speedup_at=-1.0, verbose=False):
    base = imread_unicode(ROOT / "data" / "samples" / "win05.jpg")
    base = cv2.resize(base, (W, H), interpolation=cv2.INTER_AREA)
    base = (base.astype(np.float32) * 0.80).clip(0, 255).astype(np.uint8)
    f = F1920 * W / 1920.0
    D_HANDOFF = HANDOFF_1920 * W / 1920.0

    trk = TemplateTracker()
    locked = False
    errs = []
    cost = []
    reached = False
    worst = 0.0
    dt = 1.0 / FPS
    t = 0.0

    while t < 20.0:
        prog = min(1.0, max(0.0, (t - 2.0) / T_APP))
        if speedup_at > 0 and t > speedup_at:
            prog = min(1.0, prog + (t - speedup_at) * 0.10)
        d_px = f * REAL / (D0 - (D0 - D1) * prog)

        shift = int(round(camvx * t))
        jx = int(round(jitter * np.sin(t * 37.0)))
        jy = int(round(jitter * np.cos(t * 29.0)))
        M = np.float32([[1, 0, shift + jx], [0, 1, jy]])
        frame = cv2.warpAffine(base, M, (W, H), borderMode=cv2.BORDER_REPLICATE)

        tx = int(W * 0.62) - shift - jx
        ty = int(H * 0.40) - jy

        occ = occl[0] <= t < occl[1]
        oof = out_of_frame > 0 and 8.0 <= t < 8.0 + out_of_frame
        if oof:
            tx = -50                      # 移出画面
        if not occ and not oof:
            draw_dot(frame, tx, ty, d_px, frac=0.5 if partial else 1.0)
        else:
            if not oof:
                cv2.rectangle(frame, (tx - 20, ty - 20), (tx + 20, ty + 20),
                              (58, 62, 58), -1)
        if darken:
            x0, y0 = max(0, tx - 30), max(0, ty - 30)
            p = frame[y0:y0 + 60, x0:x0 + 60]
            if p.size:
                frame[y0:y0 + 60, x0:x0 + 60] = (p * 0.5).astype(np.uint8)
        if glare:
            cv2.circle(frame, (tx + 26, ty - 10), 10, (245, 245, 250), -1)

        # ★ 真实链路退化：vstream 是 JPEG 流，传感器还有噪声
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if NOISE_SIGMA > 0:
            n = np.random.normal(0, NOISE_SIGMA, frame.shape).astype(np.int16)
            frame = np.clip(frame.astype(np.int16) + n, 0, 255).astype(np.uint8)

        t0 = cv2.getTickCount()
        if not locked and not occ and not oof and t > 2.5 and tx > 0:
            trk.init(frame, (tx, ty), half_size=max(8, int(d_px * 1.1)), d_px=d_px)
            locked = True
        if locked:
            if occ or oof:
                trk.predict(frame, dt)
            else:
                ok, cen, sc = trk.update(frame, dt, d_est=d_px)
                if not ok:
                    trk.predict(frame, dt)
            if trk.centre and tx > 0 and t > max(occl[1], 8.0 + out_of_frame) + 0.5:
                e = float(np.hypot(trk.centre[0] - tx, trk.centre[1] - ty))
                errs.append(e)
                worst = max(worst, e)
            if d_px >= D_HANDOFF:
                reached = True
        cost.append((cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000)
        if verbose and abs(t - round(t)) < dt / 2:
            print(f"    t={t:4.1f} d={d_px:5.1f} err={errs[-1] if errs else 0:6.1f}")
        t += dt

    mean_err = float(np.mean(errs)) if errs else -1
    ok = reached and worst < 60
    verdict = ("OK" if ok else
               ("跟错" if worst >= 60 else "未完成"))
    return {"name": name, "reached": reached, "worst": worst,
            "mean": mean_err, "ms": float(np.mean(cost)), "verdict": verdict}


def main():
    cases = [
        ("A 正常流程", dict()),
        ("B 遮挡 0.3s", dict(occl=(8.0, 8.3))),
        ("B 遮挡 0.5s", dict(occl=(8.0, 8.5))),
        ("B 遮挡 1.0s", dict(occl=(8.0, 9.0))),
        ("B 遮挡 2.0s", dict(occl=(8.0, 10.0))),
        ("B 遮挡 3.0s", dict(occl=(8.0, 11.0))),
        ("B 遮挡 5.0s", dict(occl=(8.0, 13.0))),
        ("C 部分遮挡", dict(partial=True)),
        ("D 光照变暗", dict(darken=True)),
        ("E 强反光", dict(glare=True)),
        ("F 画面抖动 2px", dict(jitter=2.0)),
        ("G 相机平移 2px/s", dict(camvx=2.0)),
        ("H 目标出画 1s", dict(out_of_frame=1.0)),
        ("I 尺寸跳变", dict(speedup_at=8.0)),
    ]
    print(f"{'场景':<20}{'到达交接':<10}{'最大误差':<10}{'平均误差':<10}"
          f"{'耗时ms':<9}{'判定'}")
    print("-" * 74)
    rows = []
    for n, kw in cases:
        r = run(n, **kw)
        rows.append(r)
        print(f"{r['name']:<20}{('是' if r['reached'] else '否'):<10}"
              f"{r['worst']:<10.0f}{r['mean']:<10.1f}{r['ms']:<9.1f}{r['verdict']}")
    okn = sum(1 for r in rows if r["verdict"] == "OK")
    print("-" * 74)
    print(f"通过 {okn}/{len(rows)}")


if __name__ == "__main__":
    main()
