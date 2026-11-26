# -*- coding: utf-8 -*-
"""检测方案大比拼：以"召回优先"为评价标准

═══════════════════════════════════════════════════════════════════
先建"参考轨迹"
───────────────────────────────────────────────────────────────────
拿全分辨率 + 当前阈值跑完 552 帧，从一帧"确认是红点"的种子出发，
向前后按"位置连续 + 尺寸连续"逐帧接力，得到一条参考轨迹。
参考轨迹存在的帧 = 评估集；变体只要在 25 px 内、尺寸比 0.5~2.0
检出一个候选，就算召回成功。

═══════════════════════════════════════════════════════════════════
对比的变体
───────────────────────────────────────────────────────────────────
A 全分辨率（现状基线）
B 半分辨率
C 两阶段：半分辨率严格粗扫 → ROI 全分辨率精检
D 两阶段：半分辨率**宽松**粗扫 → ROI 全分辨率精检   ← 为召回做的
E 两阶段：1/4 分辨率宽松粗扫 → ROI 全分辨率精检
F 全分辨率 + 宽松阈值
"""
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"D:\Desktop\无人系统\MiniCPM-V")
sys.path.insert(0, str(ROOT / "src"))
import mcpv.detect_live as D                                # noqa: E402

VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"

STRICT = dict(S_MIN=115, V_MIN=60, AREA_MIN=15, FILL_MIN=0.55,
              RD_MIN=50, RD_S_MIN=90, RD_AREA_MIN=6, RD_FILL_MIN=0.42)
LOOSE = dict(S_MIN=90, V_MIN=45, AREA_MIN=8, FILL_MIN=0.40,
             RD_MIN=32, RD_S_MIN=62, RD_AREA_MIN=4, RD_FILL_MIN=0.30)


def set_thr(d):
    for k, v in d.items():
        setattr(D, k, v)


def detect_at(fr, k, thr):
    """在 k 倍分辨率上检测，坐标还原；返回 [(cx,cy,d)]。"""
    set_thr(thr)
    if k < 1.0:
        small = cv2.resize(fr, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        cs = D.detect_reds(small)
        return [(c["cx"] / k, c["cy"] / k, c["d"] / k) for c in cs]
    cs = D.detect_reds(fr)
    return [(c["cx"], c["cy"], c["d"]) for c in cs]


def refine_rois(fr, seeds, thr, half=220):
    """对每个 seed 在原分辨率上裁 ROI 精检，返回全部命中。"""
    set_thr(thr)
    out = []
    for (sx, sy, sd) in seeds:
        h = max(half, int(sd * 4))
        x0, y0 = max(0, int(sx) - h), max(0, int(sy) - h)
        x1 = min(fr.shape[1], int(sx) + h)
        y1 = min(fr.shape[0], int(sy) + h)
        roi = fr[y0:y1, x0:x1]
        if roi.size == 0:
            continue
        for c in D.detect_reds(roi):
            out.append((c["cx"] + x0, c["cy"] + y0, c["d"]))
    return out


def main():
    cap = cv2.VideoCapture(str(VID))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()
    print(f"载入 {len(frames)} 帧")

    # ── 1. 参考轨迹（全分辨率严格）──────────────────────────
    set_thr(STRICT)
    ref_cands = []
    t0 = time.time()
    for fr in frames:
        ref_cands.append([(c["cx"], c["cy"], c["d"]) for c in D.detect_reds(fr)])
    print(f"参考检测（全分辨率）{ (time.time()-t0)*1000/len(frames):.1f} ms/帧")

    # ★ 两个种子：帧60（远处 8px）与帧430（近处 33px）
    def pick_seed(idx, prefer_d):
        best, bs = 1e9, None
        for (cx, cy, d) in ref_cands[idx]:
            if abs(d - prefer_d) < best:
                best, bs = abs(d - prefer_d), (cx, cy, d)
        return bs

    seeds = {"远处段(帧60起)": (60, pick_seed(60, 8.0)),
             "近处段(帧430起)": (430, pick_seed(430, 33.0))}
    for k, (i, sd) in seeds.items():
        print(f"  种子 {k}: 帧{i} ({sd[0]:.0f},{sd[1]:.0f}) d={sd[2]:.1f}")

    def link(start_i, step):
        track = {start_i: seed}
        cur = seed
        i = start_i + step
        while 0 <= i < len(frames):
            cand = None
            bd = 1e9
            for (cx, cy, d) in ref_cands[i]:
                dd = np.hypot(cx - cur[0], cy - cur[1])
                if dd > 110:
                    continue
                if not (0.45 <= d / max(cur[2], 1e-6) <= 2.4):
                    continue
                if dd < bd:
                    bd, cand = dd, (cx, cy, d)
            if cand is None:
                i += step
                if abs(i - start_i) > 25:      # 连续断 25 帧才放弃
                    break
                continue
            track[i] = cand
            cur = cand
            i += step
        return track

    tracks = {}
    for name, (si, sd) in seeds.items():
        globals()["seed"] = sd
        t = link(si, +1)
        t.update(link(si, -1))
        tracks[name] = t
        print(f"  {name}: 覆盖 {len(t)} 帧 "
              f"（帧 {min(t)}~{max(t)}）")

    def recall(cands_by_frame, ref_track, tol_px=12.0, lo=0.70, hi=1.50):
        """严格召回：命中必须 12px 内且尺寸比 0.7~1.5。"""
        hit = tot = 0
        for i, (rx, ry, rd) in ref_track.items():
            tot += 1
            for (cx, cy, d) in cands_by_frame.get(i, []):
                if np.hypot(cx - rx, cy - ry) <= tol_px and \
                        lo <= d / max(rd, 1e-6) <= hi:
                    hit += 1
                    break
        return hit, tot

    def fp_per_frame(cands_by_frame, ref_track):
        """误检数：离参考目标 >25px 的候选，按帧平均。"""
        nums = []
        for i, lst in cands_by_frame.items():
            ref = ref_track.get(i)
            n = 0
            for (cx, cy, d) in lst:
                if ref is None or np.hypot(cx - ref[0], cy - ref[1]) > 25:
                    n += 1
            nums.append(n)
        return float(np.mean(nums)) if nums else 0.0

    # ── 2. 各变体 ────────────────────────────────────────
    variants = {}

    def run_single(name, k, thr):
        out, t = {}, 0.0
        for i, fr in enumerate(frames):
            t0 = time.time()
            out[i] = detect_at(fr, k, thr)
            t += time.time() - t0
        variants[name] = (out, t / len(frames) * 1000)

    def run_two(name, k, thr_coarse, use_loose_coarse=True):
        out, t = {}, 0.0
        for i, fr in enumerate(frames):
            t0 = time.time()
            seeds = detect_at(fr, k, thr_coarse)
            if seeds:
                hits = refine_rois(fr, seeds, STRICT)
                out[i] = hits
            else:
                out[i] = []
            t += time.time() - t0
        variants[name] = (out, t / len(frames) * 1000)

    print("\n跑变体…")
    run_single("A 全分辨率(严格)", 1.0, STRICT)
    run_single("B 半分辨率(严格)", 0.5, STRICT)
    run_two("C 两阶段 1/2严格", 0.5, STRICT)
    run_two("D 两阶段 1/2宽松", 0.5, LOOSE)
    run_two("E 两阶段 1/4宽松", 0.25, LOOSE)
    run_single("F 全分辨率(宽松)", 1.0, LOOSE)

    print(f"\n{'方案':<20}{'远处召回(严)':<16}{'近处召回(严)':<16}"
          f"{'ms/帧':<8}{'误检/帧'}")
    print("-" * 76)
    for name, (out, ms) in variants.items():
        hf, tf = recall(out, tracks["远处段(帧60起)"])
        hn, tn = recall(out, tracks["近处段(帧430起)"])
        fp = fp_per_frame(out, tracks["近处段(帧430起)"])
        print(f"{name:<20}{f'{hf}/{tf} ({hf/tf*100:.0f}%)':<16}"
              f"{f'{hn}/{tn} ({hn/tn*100:.0f}%)':<16}{ms:6.1f}   {fp:6.1f}")


if __name__ == "__main__":
    main()
