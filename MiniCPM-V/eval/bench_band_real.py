# -*- coding: utf-8 -*-
"""bench_band_real.py —— 真实帧验证「楼层带选」这一步（编号画进图里）

前两轮真实帧测试作废，原因是我取真值用的 `min(cands, key=d)` 会挑到假候选，
真值本身是错的。这一轮改用之前建好的**链接轨迹真值**（gt.pkl，逐帧跟踪、
跨 ±25 帧连续性约束），真值可信。

被测的就是设想里 L1 那一步：
    整帧 + 画好 N 条横带编号 → 问「红圆在第几号带」
若这一步在真实帧上成立，"语义给出高度"就有着落。
"""
from __future__ import annotations

import base64
import json
import pickle
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
S14 = Path(r"D:\Desktop\无人系统\_ref\s14")
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(S14))

from vb09.perception.stereo_rig import StereoRig            # noqa: E402

MODEL = "openbmb/minicpm-v4.7:1b"
URL = "http://127.0.0.1:11434/api/generate"
VID = ROOT / "eval" / "demo_out" / "run_20261109_164902.mp4"
GT_FILES = [ROOT / "eval" / "gt_cache.pkl",
            Path(r"C:\Users\Van Dark Simple\AppData\Local\Temp\ollama_verify\gt.pkl")]

Q = ("这张图被横向切成 {n} 条带编号的横带（编号画在每条带的左上角，{n} 在最下）。"
     "只有一条带里有【贴在白色纸上的红色圆形标记】。"
     "它在几号带？只回答一个数字。")


def ask(img, prompt, npd=12):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    body = json.dumps({"model": MODEL, "prompt": prompt,
                       "images": [base64.b64encode(buf.tobytes()).decode()],
                       "stream": False, "think": False, "keep_alive": "30m",
                       "options": {"temperature": 0, "num_predict": npd}}).encode()
    rq = urllib.request.Request(URL, data=body,
                                headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=300) as r:
        return (json.loads(r.read()).get("response") or "").strip()


def draw_bands(img, n):
    out = img.copy()
    H, W = out.shape[:2]
    for i in range(n):
        y0, y1 = int(i * H / n), int((i + 1) * H / n)
        cv2.rectangle(out, (0, y0), (W - 1, y1), (0, 255, 255), 3)
        for txt, col, th in ((str(i + 1), (0, 0, 0), 8), (str(i + 1), (0, 255, 255), 4)):
            cv2.putText(out, txt, (14, y0 + 60), cv2.FONT_HERSHEY_SIMPLEX, 2.0, col, th)
    return out


def build_gt(rig, frames, cache):
    """从两个目视确认的种子出发做前后链接，得到逐帧真值（与 build_cases.py 同法）。

    为什么用链接而不用"取最小的候选"：后者会挑到红墙假阳性，
    前两轮真实帧实验的结论就是这么被毁掉的。
    """
    from vb09.perception.reddot.config import DotConfig
    from vb09.perception.reddot.detect import DotDetectorCore
    from mcpv.detect_live import detect_reds

    rig_lo = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=0.5)
    cfg = DotConfig()
    cfg.min_d_px, cfg.dot_d, cfg.d_max, cfg.open_k = 7.0, 0.20, 0.28, 1
    det = DotDetectorCore(rig_lo, cfg)

    N = len(frames)
    HI = []
    print("建真值：逐帧检测 %d 帧…" % N, flush=True)
    for f in frames:
        L, _ = rig.rectify(f, f)
        HI.append([(c["cx"], c["cy"], c["d"]) for c in detect_reds(L)])

    def link(seed, start, step):
        tr, cur, i = {start: seed}, seed, start + step
        while 0 <= i < N:
            best, bd = None, 1e9
            for (cx, cy, d) in HI[i]:
                dd = float(np.hypot(cx - cur[0], cy - cur[1]))
                if dd <= 110 and 0.45 <= d / max(cur[2], 1e-6) <= 2.4 and dd < bd:
                    bd, best = dd, (cx, cy, d)
            if best is None:
                i += step
                if abs(i - start) > 25:
                    break
                continue
            tr[i], cur, i = best, best, i + step
        return tr

    far = min(HI[60], key=lambda q: float(np.hypot(q[0] - 1029, q[1] - 508)))
    near = min(HI[430], key=lambda q: float(np.hypot(q[0] - 993, q[1] - 722)))
    A = link(far, 60, +1)
    A.update(link(far, 60, -1))
    B = link(near, 430, +1)
    B.update(link(near, 430, -1))
    d = dict(A)
    d.update(B)
    with open(cache, "wb") as f:
        pickle.dump({"FAR": A, "NEAR": B}, f)
    print("  真值 %d 帧（远段 %d / 近段 %d）→ %s" % (len(d), len(A), len(B), cache), flush=True)
    return d


def main():
    gt = None
    for p in GT_FILES:
        if p.exists():
            with open(p, "rb") as f:
                d = pickle.load(f)
            gt = {}
            gt.update(d["FAR"])
            gt.update(d["NEAR"])
            print("载入真值 %s：%d 帧" % (p, len(gt)))
            break

    rig = StereoRig.from_file(str(S14 / "stereo_calib.yaml"), scale=1.0)
    cap = cv2.VideoCapture(str(VID))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()

    if gt is None:
        gt = build_gt(rig, frames, GT_FILES[0])
    if not gt:
        print("真值建不出来，退出"); return

    # 挑三帧：红点 y 分别落在画面不同高度，且离带边界远（避免"边界效应"争议）
    picks = []
    for target_y in (0.18, 0.52, 0.84):
        best, bd = None, 1e9
        for i, (x, y, d) in gt.items():
            if i >= len(frames):
                continue
            if 100 < x < 1700 and 40 < y < 1000:
                dd = abs(y / 1080.0 - target_y)
                if dd < bd:
                    bd, best = dd, i
        if best is not None:
            picks.append(best)
    picks = sorted(set(picks))
    print("选用帧：%s" % picks)

    print()
    print("%-8s %-6s %-8s %-8s %-8s %-8s %-6s %s"
          % ("帧", "带数n", "真值y", "真值带", "回答", "对错", "距边界", "输出"))
    print("-" * 88)
    rows = []
    for i in picks:
        L, _ = rig.rectify(frames[i], frames[i])
        H, W = L.shape[:2]
        gx, gy, gd = gt[i]
        for n in (3, 4, 6):
            band = int(gy / (H / float(n))) + 1
            frac_in = (gy - (band - 1) * H / n) / (H / n)
            edge = min(frac_in, 1 - frac_in)          # 0.5 = 正中间，0 = 压线
            img = draw_bands(cv2.resize(L, (1280, 720), interpolation=cv2.INTER_AREA), n)
            txt = ask(img, Q.format(n=n))
            got = None
            for ch in txt:
                if ch.isdigit():
                    got = int(ch)
                    break
            ok = (got == band)
            rows.append(dict(frame=int(i), n=n, gy=float(gy), band=band,
                             got=got, ok=ok, edge=round(edge, 2), raw=txt[:16],
                             d_px=float(gd)))
            print("%-8d %-6d %-8.0f %-8d %-8s %-8s %-6.2f %s"
                  % (i, n, gy, band, got, "对" if ok else "错", edge, txt[:24]))

    hit = sum(1 for r in rows if r["ok"])
    print()
    print("带选命中 %d/%d" % (hit, len(rows)))
    for n in (3, 4, 6):
        sub = [r for r in rows if r["n"] == n]
        print("  n=%-2d %d/%d" % (n, sum(1 for r in sub if r["ok"]), len(sub)))

    out = ROOT / "eval" / "band_real.json"
    json.dump(rows, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("结果 -> %s" % out)


if __name__ == "__main__":
    main()
