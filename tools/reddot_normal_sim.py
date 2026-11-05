# -*- coding: utf-8 -*-
"""
tools/reddot_normal_sim.py —— 红点法向 (多视角消二义性) 的合成回归: 已知真值, 不用相机 / cv2
================================================================================================
2026-11-05。把一个已知位置、已知法向的圆盘按针孔投影成 Cand (圆心、长短轴、短轴方向、视差 —— 和
detect.half_max_moments 量的是同一组量), 加噪声, 按飞行轨迹喂 DotLocator, 检查:
  · 只悬停 (单视角): **不许**发 n (两个解分不开; 以前会以 n_sd≈2° 发一个可能错 70–80° 的解)
  · 停靠后上下扫描 ±dz: 发 n, 且离真值 < 8°
  · 正对 / 太远 (视角跨度不够): 可以不发 n, 但**发了就必须对**
  · 整个过程只有一条轨迹, 悬停期间 stage 不是 lost

    python3 tools/reddot_normal_sim.py            # 全部场景, 有 FAIL 退出码 1
    python3 tools/reddot_normal_sim.py -v         # 打印每个场景的过程
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from vb09.perception.reddot.config import DotConfig       # noqa: E402
from vb09.perception.reddot.frame import Cand, DotFrame   # noqa: E402
from vb09.perception.reddot.locate import DotLocator      # noqa: E402

F = 1626.6            # 全分辨率校正焦距 (11-04 标定)
B = 0.0602
CX, CY = 968.4, 514.7
DOT_D = 0.15
DOT = np.array([0.0, 3.0, 1.0])          # 红点 (world)


def look_at(p, q):
    """相机在 p 看向 q → T_WC (OpenCV 相机系: x 右 y 下 z 前)。"""
    z = (q - p) / np.linalg.norm(q - p)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], axis=1)
    T[:3, 3] = p
    return T


def normal_from(az_deg, el_deg):
    """法向: 从 −y (朝起飞点) 转 az (绕 z) / el (抬头)。"""
    a, e = math.radians(az_deg), math.radians(el_deg)
    return np.array([math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)])


_DISK = None


def _disk_pts(n_pts=4000):
    global _DISK
    if _DISK is None:
        rng = np.random.default_rng(0)
        r = np.sqrt(rng.random(n_pts)) * DOT_D / 2
        t = rng.random(n_pts) * 2 * math.pi
        _DISK = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
    return _DISK


def make_cand(T_WC, n, rng, px_sd=0.3, disp_sd=0.5, disp_bias=-1.0):
    """圆盘 → Cand。椭圆参数用投影后均匀圆盘的矩 (= detect 的 4σ 长短轴), 再加噪声。"""
    u1 = np.cross(n, [0.0, 0.0, 1.0])
    if np.linalg.norm(u1) < 1e-6:
        u1 = np.array([1.0, 0.0, 0.0])
    u1 /= np.linalg.norm(u1)
    u2 = np.cross(n, u1)
    d = _disk_pts()
    Pw = DOT + d[:, :1] * u1 + d[:, 1:] * u2
    R, t = T_WC[:3, :3], T_WC[:3, 3]
    Pc = (Pw - t) @ R                            # world → cam
    uv = F * Pc[:, :2] / Pc[:, 2:3] + [CX, CY]
    mu = uv.mean(axis=0)
    ev, evec = np.linalg.eigh(np.cov(uv.T))
    dmaj, dmin = 4 * math.sqrt(ev[1]), 4 * math.sqrt(ev[0])
    m = evec[:, 0]
    pc = (DOT - t) @ R
    Z = pc[2]
    u, v = mu + rng.normal(0, px_sd, 2)
    dmaj += rng.normal(0, px_sd)
    dmin = min(dmaj, dmin + rng.normal(0, px_sd))
    ang = math.atan2(m[1], m[0]) + rng.normal(0, math.radians(1.0))
    disp = F * B / Z + disp_bias + rng.normal(0, disp_sd)
    return Cand(u=u, v=v, xn=(u - CX) / F, yn=(v - CY) / F, fb=F * B, dmaj=dmaj, dmin=dmin,
                mx=math.cos(ang), my=math.sin(ang), area=math.pi * dmaj * dmin / 4, fill=0.78, circ=0.90,
                axr=dmin / dmaj, red=120.0, nw=8, nn=8, disp=disp, pk=0.99, uniq=0.7, dy=0.2, ps=0.9)


def run(name, n_true, standoff, path, seed=0, ext_err_deg=0.3, verbose=False):
    """path: [(dx, dz, 秒数)] 相对停靠点 (停靠点 = 红点沿 −y 方向 standoff 米, 同高) 的分段直线, 30 Hz。
    ext_err_deg: 外参误差 (绕相机 x 轴的固定俯仰偏差)。→ (结果 dict, 过程行)"""
    rng = np.random.default_rng(seed)
    cfg = DotConfig()
    loc = DotLocator(cfg, F, np.eye(3))
    S = DOT + np.array([0.0, -standoff, 0.0])
    a = math.radians(ext_err_deg)
    R_err = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    t, hz = 0.0, 30.0
    p = np.array([0.0, 0.0, 0.0])
    out = {"n_first_t": None, "n_err_max": 0.0, "n_wrong": 0, "max_tracks": 0, "lost": 0, "frames": 0,
           "n_last": None, "why_last": ""}
    lines = []
    for dx, dz, dur in path:
        q = np.array([dx, 0.0, dz])
        steps = max(1, int(dur * hz))
        for k in range(steps):
            pk_ = p + (q - p) * (k + 1) / steps
            cam = S + pk_ + rng.normal(0, 0.01, 3)           # 悬停抖动 1 cm
            T_true = look_at(cam, DOT + rng.normal(0, 0.02, 3))
            c = make_cand(T_true, n_true, rng)
            T_est = T_true.copy()
            T_est[:3, :3] = T_true[:3, :3] @ R_err            # 估计的位姿带外参误差
            fr = DotFrame(t=t, cands=[c])
            loc.update(fr, T_est, t)
            ex = loc.extra(fr, T_est)
            out["frames"] += 1
            out["max_tracks"] = max(out["max_tracks"], len(loc.tracks))
            if fr.stage in ("lost", "search") and out["frames"] > 5:
                out["lost"] += 1
            if "n" in ex:
                e = math.degrees(math.acos(min(1.0, abs(float(np.dot(ex["n"], n_true))))))
                out["n_err_max"] = max(out["n_err_max"], e)
                out["n_wrong"] += e > 15.0
                out["n_overconf"] = out.get("n_overconf", 0) + (e > 3.0 * ex["n_sd_deg"] + 2.0)
                out["n_sd_last"] = ex["n_sd_deg"]
                if out["n_first_t"] is None:
                    out["n_first_t"] = t
                out["n_last"] = e
            out["why_last"] = ex.get("n_why", "")
            if verbose and int(t * hz) % 15 == 0:
                lines.append("  t=%5.2f z=%+.2f stage=%-5s trk=%d n=%s %s" % (
                    t, pk_[2], fr.stage, len(loc.tracks),
                    ("%.1f°off sd%.1f span%.1f" % (out["n_last"], ex["n_sd_deg"], ex["n_span_deg"])) if "n" in ex
                    else "-", ex.get("n_why", "")))
            t += 1.0 / hz
        p = q
    return out, lines


HOVER = [(0.0, 0.0, 6.0)]
VSCAN = [(0.0, 0.0, 2.0), (0.0, -0.3, 2.0), (0.0, +0.3, 4.0), (0.0, 0.0, 2.0), (0.0, 0.0, 2.0)]


def scenarios():
    # (名字, 法向 az, el, 停靠距离, 路径, 期望: "none" 不许发 / "must" 必须发且对 / "maybe" 发了就要对)
    S = []
    for az in (25, 40, 55):
        S.append(("悬停 侧视 %d°" % az, az, 5, 3.0, HOVER, "none"))
        S.append(("上下扫描 侧视 %d°" % az, az, 5, 3.0, VSCAN, "must"))
        S.append(("上下扫描 侧视 −%d°" % az, -az, 5, 3.0, VSCAN, "must"))
    for el in (25, 40):
        S.append(("悬停 俯仰 %d°" % el, 0, el, 3.0, HOVER, "none"))
        S.append(("上下扫描 俯仰 %d°" % el, 0, el, 3.0, VSCAN, "must"))
    S.append(("上下扫描 斜 30°/20°", 30, 20, 3.0, VSCAN, "must"))
    for az in (0, 5, 10):
        S.append(("上下扫描 正对 %d°" % az, az, 3, 3.0, VSCAN, "maybe"))
    S.append(("上下扫描 侧视 35° 5 m", 35, 5, 5.0, VSCAN, "maybe"))
    S.append(("上下扫描 侧视 35° 2.5 m", 35, 5, 2.5, VSCAN, "must"))
    return S


def main():
    verbose = "-v" in sys.argv
    n_fail = 0
    for seed in (0, 1, 2):
        for name, az, el, d, path, exp in scenarios():
            n_true = normal_from(az, el)
            r, lines = run(name, n_true, d, path, seed=seed, verbose=verbose)
            bad = []
            if r["n_wrong"]:
                bad.append("发了错的 n (%d 帧, 最大偏 %.0f°)" % (r["n_wrong"], r["n_err_max"]))
            if exp == "none" and r["n_first_t"] is not None:
                bad.append("单视角不该发 n")
            if exp == "must" and r["n_last"] is None:
                bad.append("扫描后没发 n (%s)" % r["why_last"])
            if exp == "must" and r["n_last"] is not None and r["n_last"] > 8.0:
                bad.append("n 偏 %.1f° > 8°" % r["n_last"])
            if r.get("n_overconf"):
                bad.append("n 的不确定度报小了 (%d 帧偏差 > 3·n_sd + 2°)" % r["n_overconf"])
            if r["max_tracks"] > 1:
                bad.append("轨迹被拆成 %d 条" % r["max_tracks"])
            if r["lost"]:
                bad.append("看得到却 lost %d 帧" % r["lost"])
            n_fail += bool(bad)
            print("%-4s seed%d %-24s n: %s  %s" % (
                "FAIL" if bad else "ok", seed, name,
                ("首次 t=%.1fs 最终偏 %.1f° 报 σ %.1f°" % (r["n_first_t"], r["n_last"], r["n_sd_last"])) if r["n_first_t"] is not None
                else "不发 (%s)" % r["why_last"], "; ".join(bad)))
            for ln in lines:
                print(ln)
    print("=" * 60)
    print("ALL PASS" if not n_fail else "%d FAIL" % n_fail)
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
