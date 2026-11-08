# -*- coding: utf-8 -*-
"""
tools/reddot_track_sim.py —— 红点轨迹 / 分桶后验 / 立体比例自标定 / 选点 的合成回归 (numpy only, 不要 cv2)

    python3 tools/reddot_track_sim.py [--seed N] [-v]

2026-11-08 (PLAN_dot_v2)。合成一次"远处发现 → 抵近 → 停靠 → 侧移"的观测序列, 误差模型按 reddot_exp 实测:
  * 立体比例 k = 1.03 (立体距离 = 1.03 × 真距离)
  * 视差噪声 (半分辨率): 每帧 0.2 px + 每个视点 (0.5 s) 相关的 0.3 px (≈ 全分辨率 0.4 + 0.6, reddot_exp 跨视点实测 0.5–0.8)
  * 位姿 / 时间戳: 视点相关的角度误差 0.35°/轴 (每 1 s 换一个值)
  * 干扰: 0.6 m 外另一个红点, 白边更好 (像素分更高)
场景:
  approach   不选点, 只有一个红点: 抵近后 k 估到 max(1.5%, 2.5σ), 停靠点 (3.5 m) 横向 < 3 cm / 深度 ≤ 2σ 且 ≤ 10 cm,
             全程各轴误差 ≤ 3σ 的比例 ≥ 95% (协方差诚实), 结束时报的 σ 不比实际误差小一半以上
  select_ray 有干扰 (分数更高), 10 m 处按目标的像素选点 → 全程发布的是目标 (不是分数高的干扰)
  far_weak   目标远处没有立体 / 没有白边 (单帧会被 mono_weak 拒) → 按像素选点后照样能跟上
  reset      选点后估计器重建 (inherit) → 选点按位置重新挂上, 不跳到干扰
  scale_prior 第一条轨迹标好的比例 (σ < 2%) 给第二条轨迹当先验, 第二个红点的深度误差 ≤ 2.5σ
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from vb09.perception.reddot.config import DotConfig        # noqa: E402
from vb09.perception.reddot.frame import Cand, DotFrame    # noqa: E402
from vb09.perception.reddot.locate import DotLocator       # noqa: E402

F = 813.3                # 半分辨率 (地面默认 --scale 0.5)
B = 0.0602
CX, CY = 480.0, 270.0
R_BC = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # 机体 x 前 → 相机 z
K_ST = 1.03


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Rsmall(rx, ry, rz):
    """小角度旋转 (rad)。"""
    def R(axis, a):
        c, s = math.cos(a), math.sin(a)
        if axis == 0:
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        if axis == 1:
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R(0, rx) @ R(1, ry) @ R(2, rz)


class World(object):
    def __init__(self, rng, dots, k=K_ST):
        self.rng, self.dots, self.k = rng, dots, k
        self.view_key = None
        self.ang = np.zeros(3)
        self.dview = {}

    def cam_pose(self, p, yaw):
        T = np.eye(4)
        T[:3, :3] = Rz(yaw) @ R_BC
        T[:3, 3] = p
        return T

    def frame(self, t, p, yaw, stereo=True, white=True):
        """真位姿 (p, yaw) 下的观测; 返回 (DotFrame, 估计器用的位姿 = 带视点相关角度误差)。"""
        vk = int(t // 1.0)
        if vk != self.view_key:
            self.view_key = vk
            self.ang = self.rng.normal(0, math.radians(0.35), 3)
        T_true = self.cam_pose(p, yaw)
        T_est = T_true.copy()
        T_est[:3, :3] = T_true[:3, :3] @ Rsmall(*self.ang)
        cands = []
        for di, (P, D, wbonus) in enumerate(self.dots):
            pc = T_true[:3, :3].T @ (P - p)
            Z = pc[2]
            if Z < 0.5:
                continue
            u, v = CX + F * pc[0] / Z, CY + F * pc[1] / Z
            if not (5 < u < 2 * CX - 5 and 5 < v < 2 * CY - 5):
                continue
            dmaj = F * D / Z * 0.985 * (1 + self.rng.normal(0, 0.01))
            if dmaj < 6:
                continue
            key = (di, int(t // 0.5))
            if key not in self.dview:
                self.dview[key] = self.rng.normal(0, 0.3)
            disp = F * B / (self.k * Z) + self.dview[key] + self.rng.normal(0, 0.2)
            c = Cand(u=u + self.rng.normal(0, 0.2), v=v + self.rng.normal(0, 0.2), fb=F * B, dmaj=dmaj, dmin=dmaj * 0.97,
                     mx=0.0, my=1.0, fill=0.78, circ=0.9, axr=0.97, red=110,
                     nw=(8 if wbonus else 6) if white else 0,
                     disp=disp if stereo else float("nan"), pk=0.97 if stereo else 0.0, uniq=0.3, ps=0.8)
            c.xn, c.yn = (c.u - CX) / F, (c.v - CY) / F
            cands.append(c)
        return DotFrame(t=t, cands=cands), T_est


def path_approach(t, p0, p1, v=0.6, t_hold=0.0):
    """p0 → p1 匀速 v, 到了停 (t_hold 后侧移 ±0.3 m 一圈)。"""
    d = p1 - p0
    L = float(np.linalg.norm(d))
    T = L / v
    if t <= T:
        return p0 + d * (t / T)
    tt = t - T - t_hold
    if tt <= 0:
        return p1.copy()
    side = np.array([-d[1], d[0], 0.0]) / max(math.hypot(d[0], d[1]), 1e-9)
    ph = tt / 2.0                                    # 每段 1 s 走 0.3 m
    off = 0.3 * (min(ph, 1.0) - 2.0 * max(0.0, min(ph - 1.0, 1.0)) + max(0.0, min(ph - 2.0, 1.0)))
    return p1 + side * off


def yaw_to(p, P):
    return math.atan2(P[1] - p[1], P[0] - p[0])


def check(name, ok, msg):
    print("%-4s %-12s %s" % ("ok" if ok else "FAIL", name, msg))
    return ok


def sc_approach(seed, verbose):
    rng = np.random.default_rng(seed)
    P = np.array([10.0, 0.0, 1.2])
    W = World(rng, [(P, 0.15, True)])
    cfg = DotConfig()
    loc = DotLocator(cfg, F, np.eye(3))
    p0, p1 = np.array([0.0, 0.3, 1.0]), np.array([6.5, 0.0, 1.2])
    n_in = n_all = 0
    last = None
    for i in range(int((11.0 + 2.0 + 6.0) / 0.1)):
        t = i * 0.1
        p = path_approach(t, p0, p1, t_hold=2.0)
        fr, T = W.frame(t, p, yaw_to(p, P))
        loc.update(fr, T, t)
        if not fr.target.valid:
            continue
        e = fr.target.pos_w - P
        sd = np.sqrt(np.diag(fr.target.cov_w))
        n_all += 3
        n_in += int(np.sum(np.abs(e) <= 3 * sd))
        last = (e, sd, loc.tracks[0].m, loc.tracks[0].sm, fr.target.dist)
        if verbose and i % 10 == 0:
            print("   t=%5.1f dist %5.2f err %s sd %s k %.3f±%.3f" % (t, fr.target.dist, np.round(e * 100, 1),
                                                                    np.round(sd * 100, 1), 1 / last[2], last[3]))
    e, sd, m, sm, dist = last
    ok = True
    k_hat = 1.0 / m
    ok &= check("approach", abs(k_hat - K_ST) < max(0.015, 2.5 * sm), "立体比例 k 估 %.3f (真 %.2f) ±%.3f" % (k_hat, K_ST, sm))
    lat = math.hypot(e[1], e[2])
    ok &= check("approach", lat < 0.03 and abs(e[0]) < min(0.10, max(0.05, 2.0 * sd[0])),
                "停靠后误差 横向 %.1f cm / 深度 %.1f cm (距离 %.1f m, 深度报 σ %.1f cm; 线 = 2σ 且 ≤ 10 cm)"
                % (lat * 100, e[0] * 100, dist, sd[0] * 100))
    frac = n_in / max(n_all, 1)
    ok &= check("approach", frac >= 0.95, "各轴误差 ≤ 3σ 的比例 %.1f%% (协方差诚实)" % (frac * 100))
    ok &= check("approach", np.all(sd * 2 >= np.abs(e) - 0.005),
                "结束时报 σ %s cm vs 误差 %s cm" % (np.round(sd * 100, 1), np.round(e * 100, 1)))
    return ok


def _ray_cmd(T, P, seq):
    """目标在这一帧 (位姿 T, 带误差的估计位姿也行) 里的像素 → 选点指令。"""
    pc = T[:3, :3].T @ (P - T[:3, 3])
    return {"seq": seq, "mode": "ray", "xn": pc[0] / pc[2], "yn": pc[1] / pc[2]}


def sc_select(seed, verbose, far_weak=False, reset_at=None):
    rng = np.random.default_rng(seed + 100)
    P = np.array([10.0, 0.0, 1.2])
    Q = P + np.array([0.0, 0.6, 0.0])                # 干扰: 0.6 m 外, 白边更好
    W = World(rng, [(P, 0.15, False), (Q, 0.15, True)])
    cfg = DotConfig()
    loc = DotLocator(cfg, F, np.eye(3))
    p0, p1 = np.array([0.0, 0.0, 1.0]), np.array([6.5, 0.0, 1.2])
    n_pub = n_wrong = n_lost = 0
    selected = False
    hist = []
    for i in range(int(13.0 / 0.1)):
        t = i * 0.1
        p = path_approach(t, p0, p1)
        dist = float(np.linalg.norm(P - p))
        weak = far_weak and dist > 7.0
        fr, T = W.frame(t, p, yaw_to(p, (P + Q) / 2), stereo=not weak, white=not weak)
        if not selected and t >= 0.5:
            print_ = loc.select(_ray_cmd(T, P, 1), T)
            selected = True
            if verbose:
                print("   选点:", print_)
        if reset_at is not None and abs(t - reset_at) < 1e-6:
            old, loc = loc, DotLocator(cfg, F, np.eye(3))
            loc.inherit(old)
        loc.update(fr, T, t)
        ex = loc.extra(fr, T)
        if not selected:
            continue
        hist.append(ex["sel"]["state"])
        if fr.target.valid and fr.stage not in ("lost", "search"):
            n_pub += 1
            if np.linalg.norm(fr.target.pos_w - P) > np.linalg.norm(fr.target.pos_w - Q):
                n_wrong += 1
        else:
            n_lost += 1
        if verbose and i % 10 == 0:
            print("   t=%4.1f dist %5.2f sel %s pub %s" % (t, dist, ex["sel"], fr.target.valid))
    name = "far_weak" if far_weak else ("reset" if reset_at is not None else "select_ray")
    ok = check(name, n_wrong == 0 and n_pub >= 0.8 * len(hist),
               "选点后发布 %d / %d 帧, 发布成干扰的 %d 帧, 没发布 %d 帧 (状态 %s)"
               % (n_pub, len(hist), n_wrong, n_lost, sorted(set(hist))))
    return ok


def sc_scale_prior(seed, verbose):
    rng = np.random.default_rng(seed + 200)
    P = np.array([10.0, 0.0, 1.2])
    W = World(rng, [(P, 0.15, True)])
    cfg = DotConfig()
    loc = DotLocator(cfg, F, np.eye(3))
    p0, p1 = np.array([0.0, 0.0, 1.0]), np.array([6.5, 0.0, 1.2])
    for i in range(int(12.0 / 0.1)):
        t = i * 0.1
        p = path_approach(t, p0, p1)
        fr, T = W.frame(t, p, yaw_to(p, P))
        loc.update(fr, T, t)
    m1, sm1 = loc.m_glob, loc.sm_glob
    # 第二个红点, 只悬停看 (没有距离变化 → 自己标不了比例), 4 m 处
    P2 = np.array([4.0, 3.0, 1.0])
    W2 = World(rng, [(P2, 0.10, True)])
    t0 = 20.0
    for i in range(30):
        t = t0 + i * 0.1
        p = np.array([0.0, 3.0, 1.0])
        fr, T = W2.frame(t, p, yaw_to(p, P2))
        loc.update(fr, T, t)
    tr2 = max(loc.tracks, key=lambda x: x.t_upd)
    e = fr.target.pos_w - P2
    sd = np.sqrt(np.diag(fr.target.cov_w))
    ok = check("scale_prior", sm1 < 0.02 and abs(tr2.m0 - m1) < 1e-9 and abs(e[0]) <= 2.5 * sd[0],
               "第一条轨迹标的 k %.3f±%.3f → 第二条轨迹先验 k %.3f; 第二个红点 (只悬停 4 m) 深度误差 %.1f cm, 报 σ %.1f cm"
               % (1 / m1, sm1, 1 / tr2.m0, e[0] * 100, sd[0] * 100))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args()
    ok = True
    for s in (a.seed, a.seed + 1, a.seed + 2):
        print("--- seed %d" % s)
        ok &= sc_approach(s, a.v)
        ok &= sc_select(s, a.v)
        ok &= sc_select(s, a.v, far_weak=True)
        ok &= sc_select(s, a.v, reset_at=6.0)
        ok &= sc_scale_prior(s, a.v)
    print("=" * 60)
    print("ALL PASS" if ok else "FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
