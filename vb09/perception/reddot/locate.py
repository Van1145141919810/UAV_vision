# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/locate.py —— 机上: 候选像素量 → 门限 / 深度融合 / 世界系轨迹 / 法向 (numpy-only, 不拉 cv2)
===============================================================================
接口 (registry.LOCATORS): DotLocator(cfg, f, R1); f; same_rig(f, R1); update(fr, T_WC, t); extra(fr, T_WC, warn)。
地面 (或机上检测端) 只回像素量, 这里做所有"物理"的事 (claude/PLAN_dot_v1.md §3):

  1. 逐候选门限 (理由写进 fr.rejected): partial / fill / circ / axr / red / size / zmismatch / mono_weak
       · 形状: 填充率 [0.70, 0.86] (圆盘 π/4; 方块 ≈ 1)、圆度、长短轴比 —— 小目标 (< shape_px) 放宽
       · size: 双目 Z 算出的物理直径 D_st = Z·dmaj/f 必须在 [d_min, d_max] = 8–24 cm 里 (红桶 28 cm、红格子 2–3 cm 过不了; 不假设具体直径)
  2. 深度 (2026-11-05 改, **不假设红点直径**):
       · 双目 Z_st 是主测距。物理直径 D_st = Z_st·dmaj/f 只卡物理范围 [d_min, d_max]
       · 每条轨迹自己估直径 D_trk = 近处、静止双目帧 D_st 的中位数 (≥ cal_min_n 个才算数), 按世界位置记住
         (估计器重建后同一处的红点直接用, inherit())。D_trk 有了以后才融合尺寸测距 Z_sz = f·D_trk/dmaj
       · 两个 Z 差超过 consist_k·σ → 这帧只用双目; 相机在转 (rate_max_deg) → 不用尺寸 (模糊把长轴拉长)
       · 没有有效立体: 有 D_trk 用它; 没有就用 dot_d 当宽先验 (size_prior_rel, 只够发现, 锁不了);
         只接受高分 + 有白边的 (src = size / size_prior)
  3. 世界系: R_WR = R_WC·R1ᵀ (★漏乘 R1ᵀ = 5 m 处横向偏约 25 cm, 见 PLAN_refactor_v2 §5), 每个候选一条轨迹
     (常位置 + 随机游走, 马氏距离关联)。轨迹分 = 单帧像素分滑动平均 × 命中数 × 稳定度。
     2026-11-05: 观测协方差加下限 (obs_ang_floor_deg / obs_pos_floor: 外参 / VIO 误差), χ² 不过但在 track_radius
     以内也并进去, 两条轨迹靠得比 track_radius 近就合并; 选发布的轨迹时只看 lost_timeout 内更新过的。
     以前同一个红点换个视角就开一条新轨迹, 发布的却是不再更新的旧轨迹 (stage = coast / lost, aim 是旧值)。
  4. 法向: 椭圆 cosθ = dmin/dmaj + 短轴方向 → 每帧两个候选, 互为关于视线的镜像, **单个视角分不开**。
     每条轨迹按视线方向分桶 (normal_bin_deg) 存候选, 找"各桶都有一个候选落在 normal_inlier_deg 内"的一致解
     (圆盘没有正反面, 当轴处理, 符号翻成指向相机一侧)。只有同时满足才发 n (2026-11-05 改):
       · 一致的桶 ≥ normal_min_k, 且它们的视线跨度 ≥ normal_view_min_deg (视角不变时错解也"一致")
       · 替代假设 (离最佳解 > 2×inlier) 的拟合代价比最佳解大 ≥ normal_lr_min: 每个假设把各桶里离它近的候选
         取平均再算 Σ min(残差, inlier)²/σ²。数内点不够: 竖直扫描时错解只被拖开 ±2ε·cosθ, 也全落在 inlier 内
     n_sd_deg = 各帧 σ_i = 0.02/sinθ 的合并 (近正对时单帧法向误差大) 和一致性散布, 不低于 normal_floor_deg。
  5. (2026-11-08, PLAN_dot_v2) 轨迹位置 = 分桶后验 (_Track.solve): 视线方向 × 距离档分桶, 桶内加权平均 + 不随帧数
     下降的下限; 位置 p 和立体比例 m 联合解 (抵近时 m 可观测, 立体比例误差被 VIO 位移标掉)。报的协方差含 m 的不确定度。
  6. 选点 select(cmd, T_WC): 飞行脚本决定选哪个 (轨迹号 / 像素视线 / 取消), 这里负责关联: 选中后只发布这条轨迹,
     没挂上 / 丢了 → 不发布 (target 无效) 而不是换成别的红点; 轨迹删掉 / 合并 / 估计器重建 → 按记住的位置重新挂。
     选中视线附近的弱候选 (没有立体、没白边) 也放行 —— 远处置信度低时靠人点出来。
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from .config import DotConfig
from .frame import DotFrame, DotObs, TargetState

_I3 = np.eye(3)


def _r4(v):
    return None if v is None else round(float(v), 4)


def _cov_along_ray(u, s_lat, s_z):
    u = u / np.linalg.norm(u)
    return s_lat ** 2 * (_I3 - np.outer(u, u)) + s_z ** 2 * np.outer(u, u)


def pixel_score(c, cfg, s):
    """单帧像素分 0–1 (手调权重; 攒够日志后用逻辑回归拟合)。"""
    strict = c.dmaj >= cfg.shape_px * s
    t_fill = 1.0 - min(1.0, abs(c.fill - 0.785) / (0.12 if strict else 0.2))
    t_circ = min(1.0, max(0.0, (c.circ - 0.5) / 0.4))
    t_axr = min(1.0, max(0.0, (c.axr - 0.4) / 0.4))
    t_red = min(1.0, max(0.0, (c.red - 50.0) / 80.0))
    return 0.30 * t_fill + 0.20 * t_circ + 0.10 * t_axr + 0.15 * t_red + 0.25 * (c.nw / 8.0)


class _Track(object):
    """
    一条世界系轨迹 (一个红点)。位置不再用逐帧卡尔曼, 而是分桶后验 (见 config 的"分桶后验"一节):
      bins[key] = 加权和 (W, Σw·c, Σw·ρ·u, Σw·u, Σw·ρ, n, Σs_lat², t_last)   w = 1/σρ²
      solve() 解 [p; m]: 桶均值 c̄ + m·b̄ = p (线性), 先验 m ~ N(m0, sm0²), Huber 降权 → x, P (含 m 的不确定度)
    关联仍按帧: chi2_to(帧观测) 对 (x, P + 老化)。
    """

    def __init__(self, tid, t, info, m0=1.0, sm0=0.05):
        self.id = tid
        self.x, self.P = info["zw"].copy(), info["Rw"].copy()
        self.t, self.t_upd = t, t
        self.n_hit = 1
        self.ema = info["score"]
        self.chi2 = deque(maxlen=8)
        self.ns = {}                         # 视角桶 → (n1_w, n2_w, σ_deg, los_w, t)。按视角存: 悬停几百帧只占一桶
        self.last = info
        self.dsamp = deque(maxlen=60)        # 直径样本 (近处静止双目帧的 D_st, **立体单位**: 真直径 = 它 × m)
        self.D_seed = None                   # 按位置记住的直径 (立体单位)
        self.bins = {}                       # 位置桶 (见类说明)
        self.m0, self.sm0 = float(m0), float(sm0)   # 建轨迹时的比例先验 (本次运行学到的, 不含本轨迹自己)
        self.m, self.sm = self.m0, self.sm0
        self.n_dir = 0
        self.n_bins = 0
        self.robust_drop = 0

    def D_raw(self, cfg):
        """这条轨迹的直径 (立体单位, m) 或 None。dot_d_cal > 0 → 用它 (换回立体单位)。"""
        if cfg.dot_d_cal > 0:
            return cfg.dot_d_cal / max(self.m, 1e-6)
        if len(self.dsamp) >= cfg.cal_min_n:
            return float(np.median(self.dsamp))
        return self.D_seed

    def D(self, cfg):
        """真直径 (m) 或 None = 立体单位直径 × 比例 m。"""
        d = self.D_raw(cfg)
        return None if d is None else d * self.m

    def add_normal(self, n1, n2, sig, los, t, cfg):
        az = math.degrees(math.atan2(los[1], los[0]))
        el = math.degrees(math.asin(max(-1.0, min(1.0, float(los[2])))))
        b = cfg.normal_bin_deg
        self.ns[(int(round(az / b)), int(round(el / b)))] = (n1, n2, sig, los, t)
        if len(self.ns) > cfg.normal_buf:                  # 超了 → 丢最旧的桶
            del self.ns[min(self.ns, key=lambda k: self.ns[k][4])]

    # ---- 位置桶 ----
    def add_obs(self, o, t, cfg):
        """一帧观测进桶。o: cw (相机光心 world), uw (视线单位向量 world, 相机→红点), rho / srho (立体单位距离及 σ), s_lat。"""
        u, rho = o["uw"], o["rho"]
        az = math.degrees(math.atan2(-u[1], -u[0]))
        el = math.degrees(math.asin(max(-1.0, min(1.0, float(-u[2])))))
        b = cfg.pos_bin_deg
        # 距离档按轨迹当前估计的距离分, 不按这一帧的立体距离 —— 按带噪声的量分桶会让噪声大的帧跑到别的桶,
        # 桶均值有选择偏差 (合成: 8 m 处深度偏短 1 m、k 被拉到 1.10)
        r_key = float(np.linalg.norm(self.x - o["cw"]))
        kr = int(math.floor(math.log(max(r_key, 0.05)) / math.log(1.0 + cfg.pos_bin_rng)))
        key = (int(round(az / b)), int(round(el / b)), kr)
        w = 1.0 / max(o["srho"], 1e-4) ** 2
        bn = self.bins.get(key)
        if bn is None:
            bn = self.bins[key] = [0.0, np.zeros(3), np.zeros(3), np.zeros(3), 0.0, 0, 0.0, t]
        bn[0] += w
        bn[1] += w * o["cw"]
        bn[2] += w * rho * u
        bn[3] += w * u
        bn[4] += w * rho
        bn[5] += 1
        bn[6] += o["s_lat"] ** 2
        bn[7] = t
        if len(self.bins) > cfg.max_bins:
            del self.bins[min(self.bins, key=lambda k: self.bins[k][7])]

    def solve(self, t, cfg, cam=None):
        """桶 → [p; m] 后验。cam (当前相机位置) 给了 → 横向 σ 不低于 角度下限·距离 / √(独立方向数)。"""
        keys = list(self.bins)
        if not keys:
            return
        B = [self.bins[k] for k in keys]
        N = len(B)
        W = np.array([b[0] for b in B])
        cb = np.array([b[1] for b in B]) / W[:, None]
        bb = np.array([b[2] for b in B]) / W[:, None]
        ub = np.array([b[3] for b in B])
        ub = ub / np.maximum(np.linalg.norm(ub, axis=1), 1e-9)[:, None]
        rho = np.array([b[4] for b in B]) / W
        nn = np.array([b[5] for b in B], float)
        slat2 = np.array([b[6] for b in B]) / nn
        age = np.maximum(0.0, t - np.array([b[7] for b in B]))
        s_al2 = np.maximum(1.0 / W, (cfg.along_floor_rel * rho) ** 2)
        s_lt2 = (rho * math.radians(cfg.obs_ang_floor_deg)) ** 2 + slat2 / nn
        iso = cfg.obs_pos_floor ** 2 + cfg.drift_q ** 2 * age
        uu = ub[:, :, None] * ub[:, None, :]
        R = s_lt2[:, None, None] * (np.eye(3)[None] - uu) + s_al2[:, None, None] * uu + iso[:, None, None] * np.eye(3)[None]
        Ri = np.linalg.inv(R)
        wr = np.ones(N)
        theta = cov = None
        for _ in range(3):
            Rw = Ri * wr[:, None, None]
            A = np.zeros((4, 4))
            bv = np.zeros(4)
            A[:3, :3] = Rw.sum(0)
            Rb = np.einsum("nij,nj->ni", Rw, bb)                   # Ri·b̄
            A[:3, 3] = A[3, :3] = -Rb.sum(0)
            A[3, 3] = float(np.einsum("ni,ni->", bb, Rb)) + 1.0 / self.sm0 ** 2
            Rc = np.einsum("nij,nj->ni", Rw, cb)
            bv[:3] = Rc.sum(0)
            bv[3] = -float(np.einsum("ni,ni->", bb, Rc)) + self.m0 / self.sm0 ** 2
            try:
                cov = np.linalg.inv(A)
            except np.linalg.LinAlgError:
                return
            theta = cov @ bv
            e = cb - (theta[:3][None, :] - theta[3] * bb)
            r2 = np.einsum("ni,nij,nj->n", e, Ri, e)
            wn = np.where(r2 > cfg.robust_k ** 2, cfg.robust_k / np.sqrt(np.maximum(r2, 1e-12)), 1.0)
            if np.allclose(wn, wr):
                break
            wr = wn
        self.robust_drop = int(np.sum(wr < 0.999))
        m = float(np.clip(theta[3], cfg.scale_min, cfg.scale_max))
        x = theta[:3] if m == theta[3] else (cb + m * bb).T @ (wr * W) / float(np.sum(wr * W))
        P = cov[:3, :3]
        P = 0.5 * (P + P.T)
        self.n_bins = N
        self.n_dir = len({k[:2] for k in keys})
        if cam is not None:
            l = cam - x
            r = float(np.linalg.norm(l))
            if r > 1e-6:
                l = l / r
                L = np.eye(3) - np.outer(l, l)
                f2 = (r * math.radians(cfg.obs_ang_floor_deg)) ** 2 / max(self.n_dir, 1) + cfg.obs_pos_floor ** 2
                lat = 0.5 * float(np.trace(L @ P @ L))
                if lat < f2:
                    P = P + (f2 - lat) * L
        self.x, self.P = np.asarray(x, float), P
        self.m, self.sm = m, float(math.sqrt(max(cov[3, 3], 0.0)))
        self.t = t

    def absorb(self, other, cfg):
        """把另一条 (同一个红点被拆出来的) 轨迹并进来。"""
        for k, bo in other.bins.items():
            b = self.bins.get(k)
            if b is None:
                self.bins[k] = [bo[0], bo[1].copy(), bo[2].copy(), bo[3].copy(), bo[4], bo[5], bo[6], bo[7]]
            else:
                for i in range(1, 4):
                    b[i] = b[i] + bo[i]
                b[0] += bo[0]
                b[4] += bo[4]
                b[5] += bo[5]
                b[6] += bo[6]
                b[7] = max(b[7], bo[7])
        while len(self.bins) > cfg.max_bins:
            del self.bins[min(self.bins, key=lambda k: self.bins[k][7])]
        self.n_hit += other.n_hit
        self.ema = max(self.ema, other.ema)
        if other.t_upd > self.t_upd:
            self.t_upd, self.last = other.t_upd, other.last
        for k, v in other.ns.items():
            if k not in self.ns or v[4] > self.ns[k][4]:
                self.ns[k] = v
        self.dsamp.extend(other.dsamp)
        if self.D_seed is None:
            self.D_seed = other.D_seed

    def P_at(self, t, q):
        """关联用: 后验 + 自上次求解以来的放大。"""
        return self.P + (q ** 2) * max(0.0, t - self.t) * _I3

    def chi2_to(self, z, R, t=None, q=0.0):
        d = z - self.x
        P = self.P if t is None else self.P_at(t, q)
        return float(d @ np.linalg.solve(P + R, d))

    def hit(self, t, info, chi2):
        self.t_upd = t
        self.n_hit += 1
        self.ema = 0.4 * info["score"] + 0.6 * self.ema
        self.chi2.append(chi2)
        self.last = info

    @property
    def score(self):
        hits = min(1.0, self.n_hit / 5.0)
        m = float(np.mean(self.chi2)) if self.chi2 else 3.0
        return self.ema * hits / (1.0 + max(0.0, m - 6.0) / 6.0)


def _ang(a, b):
    return math.degrees(math.acos(min(1.0, abs(float(a @ b)))))


def _fit(seed, ns, inlier):
    """假设法向 ≈ seed: 各桶取离它近的候选, 加权平均两次 → (n, 代价 Σ min(残差, inlier)²/σ², 内点下标)。"""
    n = seed / np.linalg.norm(seed)
    for _ in range(2):
        acc = np.zeros(3)
        for a, b, sg, _l in ns:
            v = a if abs(float(a @ n)) >= abs(float(b @ n)) else b
            if _ang(v, n) < inlier:
                acc += (v if float(v @ n) >= 0 else -v) / sg ** 2
        if np.linalg.norm(acc) < 1e-12:
            break
        n = acc / np.linalg.norm(acc)
    cost, idx = 0.0, []
    for i, (a, b, sg, _l) in enumerate(ns):
        e = min(_ang(n, a), _ang(n, b))
        cost += min(e, inlier) ** 2 / sg ** 2
        if e < inlier:
            idx.append(i)
    return n, cost, idx


def _view_span(los_list):
    """视线方向两两之间的最大夹角 (°)。"""
    span = 0.0
    for i in range(len(los_list)):
        for j in range(i + 1, len(los_list)):
            span = max(span, math.degrees(math.acos(max(-1.0, min(1.0, float(los_list[i] @ los_list[j]))))))
    return span


def normal_consensus(ns, cfg, why=None):
    """
    ns: [(n1, n2, σ_deg, los)] (world 单位向量; los = 红点 → 相机) → (n, n_sd_deg, k, span_deg) 或 None。n 是轴 (符号任意)。
    why: 给一个 dict 就把不发 n 的原因写进 why["n_why"] (诊断用)。
    """
    def no(r, st):
        if why is not None:
            why["n_why"] = r
            why["n_state"] = st           # few / span / ambig: 飞行侧 Refine 按它决定怎么换视角
        return None
    if len(ns) < cfg.normal_min_k:
        return no("视角桶 %d < %d" % (len(ns), cfg.normal_min_k), "few")
    inl = cfg.normal_inlier_deg
    fits = [_fit(v, ns, inl) for x in ns for v in (x[0], x[1])]
    n, cost, idx = min(fits, key=lambda z: z[1])
    if len(idx) < cfg.normal_min_k:
        return no("一致的视角桶 %d < %d" % (len(idx), cfg.normal_min_k), "few")
    span = _view_span([ns[i][3] for i in idx])
    if span < cfg.normal_view_min_deg:
        return no("视角跨度 %.1f° < %.0f° (单个视角分不开两个解)" % (span, cfg.normal_view_min_deg), "span")
    alt = [z[1] for z in fits if _ang(z[0], n) > 2.0 * inl]
    lr = (min(alt) - cost) if alt else float("inf")      # 没有离得远的替代解 (近正对: 两个候选本来就挨着)
    if lr < cfg.normal_lr_min:
        return no("两个解都说得通 (代价差 %.1f < %.0f)" % (lr, cfg.normal_lr_min), "ambig")
    angs = [min(_ang(n, ns[i][0]), _ang(n, ns[i][1])) for i in idx]
    wi = [1.0 / ns[i][2] ** 2 for i in idx]
    spread = math.sqrt(float(np.mean(np.square(angs))))
    k = len(idx)
    n_sd = math.sqrt(cfg.normal_floor_deg ** 2 + max(1.0 / max(sum(wi), 1e-9), spread ** 2 / max(k, 1)))
    if not alt:
        # 近正对: 两个候选相距只有 2θ, 没有"离得远的替代解"可比; 轴比噪声又让小倾角被估大 (acos 在 1 附近有偏,
        # 合成: 真 5° → 估 11°)。n 只能说"大致沿视线", 不确定度至少是测得的倾角 → judge 按 n_sd 门限丢掉,
        # 飞行按视线方向进入 (正对时本来就对)
        tilt = float(np.median([_ang(ns[i][0], ns[i][1]) / 2.0 for i in idx]))
        n_sd = max(n_sd, tilt)
        if why is not None:
            why["n_why"] = "近正对 (倾角 ≈ %.0f°), n 不确定度按倾角报" % tilt
            why["n_state"] = "frontal"
            return n, n_sd, k, span
    if why is not None:
        why["n_why"] = ""
        why["n_state"] = "ok"
    return n, n_sd, k, span


class DotLocator(object):
    kind = "reddot"

    def __init__(self, cfg: DotConfig, f, R1):
        self.cfg = cfg
        self._f = float(f)
        self.R1 = np.asarray(R1, float).reshape(3, 3)
        self.s = self._f / cfg.ref_f
        self.tracks = []
        self.next_id = 1
        self.dmem = []                       # [(世界位置, 直径 立体单位)]: 估过直径的红点 (轨迹删掉 / 估计器重建后还在)
        self._prev = None                    # (R_WR, t) 上一帧, 算相机角速度
        self.m_glob, self.sm_glob = 1.0, cfg.scale_prior_sd    # 本次运行学到的立体比例 m = 1/k (相机性质, 所有轨迹共用)
        self.sel = None                      # 选点状态 (见 select)

    # ---- 接口 ----
    @property
    def f(self):
        return self._f

    def same_rig(self, f, R1, tol=1e-4):
        return abs(float(f) - self._f) <= tol * self._f and \
            np.allclose(np.asarray(R1, float).reshape(3, 3), self.R1, atol=tol)

    def T_WR(self, T_WC):
        """rect 系 → 世界: R_WR = R_WC·R1ᵀ"""
        if T_WC is None:
            return None, None
        T_WC = np.asarray(T_WC, float)
        return T_WC[:3, :3] @ self.R1.T, T_WC[:3, 3]

    def reset(self):
        self.tracks, self.next_id = [], 1
        self._prev = None                    # dmem / 比例 / 选点保留: 和估计器状态无关
        self._sel_lost()

    def inherit(self, old):
        """估计器重建时 (桥的 _stale 重置) 从旧的 Locator 带过来: 直径、立体比例、选点 (按位置重新挂)。轨迹不带。"""
        try:
            self._remember(old)
            self.dmem = list(old.dmem)[-16:]
            self.m_glob, self.sm_glob = old.m_glob, old.sm_glob
            if old.sel is not None:
                self.sel = dict(old.sel)
                self._sel_lost()
        except Exception:
            pass
        return self

    def _remember(self, src=None):
        """把估好直径的轨迹记进 dmem (按位置去重, 新的覆盖旧的)。"""
        src = src or self
        cfg = self.cfg
        for tr in src.tracks:
            if len(tr.dsamp) < cfg.cal_min_n:
                continue
            D = float(np.median(tr.dsamp))
            mem = [(x, d) for x, d in src.dmem if np.linalg.norm(x - tr.x) >= cfg.d_mem_radius]
            mem.append((tr.x.copy(), D))
            src.dmem = mem[-16:]

    def _recall(self, x):
        best = None
        for xm, d in self.dmem:
            dd = float(np.linalg.norm(xm - x))
            if dd < self.cfg.d_mem_radius and (best is None or dd < best[0]):
                best = (dd, d)
        return best[1] if best else None

    def _rate(self, R_WR, t):
        """相机角速度 (°/s), 由相邻两帧的位姿算; 间隔太长 / 第一帧 → 0。"""
        prev, self._prev = self._prev, (R_WR, t)
        if prev is None or not (0.0 < t - prev[1] < 0.5):
            return 0.0
        c = (float(np.trace(prev[0].T @ R_WR)) - 1.0) / 2.0
        return math.degrees(math.acos(max(-1.0, min(1.0, c)))) / (t - prev[1])

    def D_of(self, tr):
        """给显示 / 状态用: 轨迹真直径 (m) 或 None。"""
        return tr.D(self.cfg) if tr is not None else None

    def _prior(self):
        return self.m_glob, max(self.sm_glob, self.cfg.scale_floor_sd)

    # ---- 选点 (飞行脚本 → /gate/select → 桥 → 这里) ----
    def select(self, cmd, T_WC=None):
        """
        cmd: {"seq": int, "mode": "track" | "ray" | "none", "trk_id": int, "xn": float, "yn": float, "t_cap": float}
          track  选一条现有轨迹 (飞行脚本的自动选点 / 终端数字键)
          ray    按像素选 (网页点击): xn, yn = 校正左目归一化坐标; T_WC = 那一帧 (t_cap) 的相机位姿 (桥按 t_cap 插值)
          none   取消选点 (回到"发布分数最高的")
        → 处理结果字符串 (桥打日志)。状态在 extra()["sel"] 里回显, 飞行脚本按 seq 核对。
        """
        try:
            seq = int(cmd.get("seq", 0))
            mode = str(cmd.get("mode", ""))
        except Exception:
            return "坏指令"
        if mode == "none":
            self.sel = None
            return "取消选点 (seq %d)" % seq
        if mode == "track":
            tid = int(cmd.get("trk_id", -1))
            tr = next((x for x in self.tracks if x.id == tid), None)
            if tr is None:
                self.sel = dict(seq=seq, mode="track", trk_id=tid, x=None, dir=None, o=None, state="lost", t=None)
                return "选轨迹 %d: 现在没有这条 (seq %d)" % (tid, seq)
            self.sel = dict(seq=seq, mode="track", trk_id=tid, x=tr.x.copy(), dir=None, o=None, state="bound", t=None)
            return "选轨迹 %d (seq %d)" % (tid, seq)
        if mode == "ray":
            R_WR, t_WR = self.T_WR(T_WC)
            if R_WR is None:
                return "按像素选点: 那一帧没有位姿 (seq %d)" % seq
            try:
                r = R_WR @ np.array([float(cmd["xn"]), float(cmd["yn"]), 1.0])
            except Exception:
                return "按像素选点: 坐标不对 (seq %d)" % seq
            self.sel = dict(seq=seq, mode="ray", trk_id=None, x=None, dir=r / np.linalg.norm(r), o=t_WR.copy(),
                            state="pending", t=None)
            tr = self._ray_match()
            if tr is not None:
                self._bind(tr)
                return "按像素选点: 挂上轨迹 %d (seq %d)" % (tr.id, seq)
            return "按像素选点: 等这条视线上出现候选 (seq %d)" % seq
        return "不认识的选点模式 %r" % mode

    def _ray_dist(self, x):
        """点 x 到选点视线的 (夹角°, 垂直距离 m); 在视线背后 → None。"""
        sl = self.sel
        v = x - sl["o"]
        a = float(v @ sl["dir"])
        if a <= 0:
            return None
        perp = float(np.linalg.norm(v - a * sl["dir"]))
        return math.degrees(math.atan2(perp, a)), perp

    def _sel_bearing(self, x, cam):
        """点 x 离选点目标的方位差: 挂上之前按选点视线 (夹角, 垂直距离), 之后按"从当前光心 cam 看, x 和记住的位置"
        的夹角 / 横向距离。→ (夹角°, 横向 m) 或 None。按方位不按欧氏距离: 远处深度误差几十 cm 到几 m
        (没有立体的弱候选用标称直径的宽先验), 欧氏距离会把选中的红点判成别的、又把旁边的干扰判成它。"""
        sl = self.sel
        if sl is None:
            return None
        if sl.get("x") is None:
            return self._ray_dist(x) if sl.get("dir") is not None else None
        if cam is None:
            d = float(np.linalg.norm(x - sl["x"]))
            return (0.0 if d < self.cfg.sel_rebind else 90.0), d
        a, b = x - cam, sl["x"] - cam
        na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        if na < 1e-6 or nb < 1e-6:
            return None
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b) / (na * nb)))))
        lat = float(np.linalg.norm(a / na * nb - b))                       # x 的方向在目标距离处离目标多远
        return ang, lat

    def _near_sel(self, x, cam=None):
        d = self._sel_bearing(x, cam)
        return d is not None and (d[0] < self.cfg.sel_ang_deg or d[1] < self.cfg.sel_lat)

    def _ray_match(self, cam=None):
        best = None
        for tr in self.tracks:
            d = self._sel_bearing(tr.x, cam)
            if d is not None and (d[0] < self.cfg.sel_ang_deg or d[1] < self.cfg.sel_lat):
                if best is None or d[0] < best[0]:
                    best = (d[0], tr)
        return best[1] if best else None

    def _bind(self, tr):
        sl = self.sel
        sl.update(mode="track", trk_id=tr.id, x=tr.x.copy(), state="bound")

    def _sel_lost(self):
        if self.sel is not None and self.sel.get("state") == "bound":
            self.sel["state"] = "lost"

    def _sel_resolve(self, cam=None):
        """每帧更新后: 选中的轨迹还在不在; 不在 → 按方位 (记住的位置 / 选点视线) 重新挂。→ 选中的轨迹或 None。"""
        sl = self.sel
        if sl is None:
            return None
        tr = next((x for x in self.tracks if x.id == sl.get("trk_id")), None) if sl.get("trk_id") is not None else None
        if tr is None:
            tr = self._ray_match(cam)
        if tr is None:
            if sl["state"] == "bound":
                sl["state"] = "lost"
            return None
        self._bind(tr)
        return tr

    # ---- 单个候选 → 观测 (rect 系) 或 拒绝原因 ----
    def _obs(self, c, relax=False):
        """relax = 选点视线附近: 不要求立体 / 白边 / 高分 (远处置信度低时人点出来的)。"""
        cfg, s, f = self.cfg, self.s, self._f
        if c.partial:
            return None, "partial"
        strict = c.dmaj >= cfg.shape_px * s
        lo, hi, cm = (cfg.fill_lo, cfg.fill_hi, cfg.circ_min) if strict else (cfg.fill_lo_s, cfg.fill_hi_s, cfg.circ_min_s)
        if not (lo <= c.fill <= hi):
            return None, "fill"
        if c.circ < cm:
            return None, "circ"
        if c.axr < cfg.axr_min:
            return None, "axr"
        if c.red < cfg.red_min:
            return None, "red"
        score = pixel_score(c, cfg, s)
        # 立体
        disp = c.disp + cfg.disp_bias * s if c.disp == c.disp else float("nan")
        st_ok = (disp == disp and disp >= cfg.disp_min * s and c.pk >= cfg.match_pk_min and c.uniq <= cfg.uniq_max
                 and abs(c.dy) <= cfg.row_tol * s)
        Zst = sst = Dst = None
        if st_ok:
            Zst = c.fb / disp
            sst = max(Zst * Zst / c.fb * cfg.sigma_disp * s, 0.02 * Zst)
            Dst = Zst * c.dmaj / f
            # 物理直径门限 (2026-11-08: 按 σ 放宽, 用本次运行学到的比例把立体单位换成米)
            Dm = Dst * self.m_glob
            sD = Dm * math.sqrt((sst / Zst) ** 2 + (cfg.size_edge_px * s / c.dmaj) ** 2)
            if not (cfg.d_min - cfg.d_gate_k * sD <= Dm <= cfg.d_max + cfg.d_gate_k * sD):
                return None, "size"
            Z, sZ, src = Zst, sst, "stereo"               # 先只用双目; 轨迹有直径后在 _refine 里融合尺寸
        else:
            if not relax and (c.nw < cfg.white_min or score < cfg.mono_score_min):
                return None, "mono_weak"
            Z = f * cfg.dot_d / self.m_glob / c.dmaj        # 宽先验 (立体单位); 关联到有直径的轨迹后在 _refine 里换成轨迹直径
            sZ, src = Z * cfg.size_prior_rel, "size_prior"
        ray = np.array([c.xn, c.yn, 1.0])
        rn = float(np.linalg.norm(ray))
        o = dict(score=score, Zst=Zst, sst=sst, Zsz=None, Dst=Dst, d_px=c.dmaj, u=c.u, v=c.v,
                 disp=disp if st_ok else None, ray=ray, rn=rn, s_lat=Z * cfg.sigma_px * s / f, relax=relax)
        self._set_depth(o, Z, sZ, src)
        # 法向候选 (rect 系)
        o["nc"] = None
        if c.dmaj >= cfg.normal_min_px * s:
            th = math.acos(min(1.0, max(0.0, c.dmin / c.dmaj)))
            su = ray / rn
            t = np.array([c.mx, c.my, 0.0])
            t = t - su * float(t @ su)
            tn = float(np.linalg.norm(t))
            if tn > 1e-6:
                t = t / tn
                o["nc"] = (-math.cos(th) * su + math.sin(th) * t, -math.cos(th) * su - math.sin(th) * t)
                o["tilt"] = math.degrees(th)
                o["n_sig"] = max(1.0, math.degrees(max(0.02, cfg.normal_sig_r * s / c.dmaj) / max(math.sin(th), 0.1)))
        return o, "ok"

    def _set_depth(self, o, Z, sZ, src, s_mov=0.0, m=1.0):
        """按深度 Z (立体单位, σ sZ) 填观测的 rect 系位置和协方差 (含外参 / VIO 下限和转动项)。位置按比例 m 换成米。"""
        cfg, ray, rn = self.cfg, o["ray"], o["rn"]
        s_lat = Z * cfg.sigma_px * self.s / self._f
        Rc = _cov_along_ray(ray, s_lat, sZ * rn * m)
        s_ang = Z * rn * math.radians(cfg.obs_ang_floor_deg)              # 外参 / VIO 误差 (只用于关联和融合)
        lat = _I3 - np.outer(ray, ray) / rn ** 2
        Rc = Rc + (s_ang ** 2 + s_mov ** 2) * lat + cfg.obs_pos_floor ** 2 * _I3
        o.update(p=m * Z * ray, Rc=Rc, Z=Z, sZ=sZ, src=src, D=m * Z * o["d_px"] / self._f, s_lat=s_lat,
                 rho=Z * rn, srho=math.hypot(sZ * rn, s_mov), m=m)

    def _refine(self, o, tr, rate, R_WR, t_WR):
        """关联到轨迹 tr (或新轨迹, tr = None) 之后: 用这条轨迹的直径融合尺寸测距; 换成这条轨迹的比例; 攒直径样本。"""
        cfg, f = self.cfg, self._f
        moving = rate > cfg.rate_max_deg
        D = tr.D_raw(cfg) if tr is not None else None                   # 立体单位
        m = tr.m if tr is not None else self.m_glob
        s_mov = o["Z"] * o["rn"] * math.radians(rate) * cfg.motion_dt
        if D is not None:
            Zsz = f * D / o["d_px"]
            ssz = Zsz * math.sqrt(cfg.size_sigma_floor ** 2 + (cfg.size_edge_px * self.s / o["d_px"]) ** 2)
            o["Zsz"] = Zsz
            if o["Zst"] is not None:
                if moving or abs(o["Zst"] - Zsz) > cfg.consist_k * math.sqrt(o["sst"] ** 2 + ssz ** 2):
                    self._set_depth(o, o["Zst"], o["sst"], "stereo", s_mov, m)          # 只用双目
                else:
                    wst, wsz = 1.0 / o["sst"] ** 2, 1.0 / ssz ** 2
                    self._set_depth(o, (wst * o["Zst"] + wsz * Zsz) / (wst + wsz), 1.0 / math.sqrt(wst + wsz),
                                    "fused", s_mov, m)
            else:
                self._set_depth(o, Zsz, ssz * (2.0 if moving else 1.0), "size", s_mov, m)
        else:
            self._set_depth(o, o["Z"], o["sZ"], o["src"], s_mov, m)
        o["zw"] = R_WR @ o["p"] + t_WR
        o["Rw"] = R_WR @ o["Rc"] @ R_WR.T
        o["cw"] = np.asarray(t_WR, float)                 # 光心 (world); 不叫 c / u: u, v 是像素
        o["uw"] = R_WR @ o["ray"] / o["rn"]
        o["moving"] = moving
        o["D_trk"] = None if D is None else D * m
        # 攒直径样本: 要比"不用尺寸测距"更静 (半个 rate_max_deg) —— 慢转时的轻微模糊也会把长轴拉长
        if tr is not None and rate <= 0.5 * cfg.rate_max_deg and o["Dst"] is not None and o["disp"] is not None \
                and o["disp"] >= cfg.cal_disp_min * self.s:
            tr.dsamp.append(o["Dst"])

    # ---- 主入口 ----
    def update(self, fr: DotFrame, T_WC, t):
        cfg = self.cfg
        fr.used, fr.rejected, fr.dots = [], [], []
        obs, rej, weak = [], {}, []
        for c in fr.cands:
            o, why = self._obs(c)
            if o is None:
                rej[why] = rej.get(why, 0) + 1
                if why == "mono_weak":
                    weak.append(c)
            else:
                obs.append(o)
        R_WR, t_WR = self.T_WR(T_WC)
        # 选点视线 / 选中位置附近的弱候选也放行 (远处没有立体、没白边时靠人点出来)
        if R_WR is not None and self.sel is not None and weak:
            for c in weak:
                o, why = self._obs(c, relax=True)
                if o is None:
                    continue
                if self._near_sel(R_WR @ o["p"] * self.m_glob + t_WR, np.asarray(t_WR, float)):
                    obs.append(o)
                    rej["mono_weak"] -= 1
                    rej["sel_relax"] = rej.get("sel_relax", 0) + 1
            rej = {k: v for k, v in rej.items() if v > 0}
        fr.rejected = ["%s×%d" % (k, v) for k, v in sorted(rej.items())]
        fr.diag.update(n_cand=len(fr.cands), n_pass=len(obs))
        if R_WR is None:
            fr.target, fr.stage = TargetState(), "search"
            fr.dbg["dot"] = {}
            return fr
        rate = self._rate(R_WR, t)
        fr.diag["rate_dps"] = round(rate, 1)
        self._remember()
        self.tracks = [tr for tr in self.tracks if t - tr.t_upd <= cfg.track_timeout]
        for o in obs:
            o["zw"] = R_WR @ o["p"] * self.m_glob + t_WR
            o["Rw"] = R_WR @ o["Rc"] @ R_WR.T
        obs.sort(key=lambda o: -o["score"])
        updated = {}
        for o in obs:
            best = near = None
            for tr in self.tracks:
                if tr.id in updated:
                    continue
                zt = t_WR + (o["zw"] - t_WR) * (tr.m / self.m_glob)       # 按这条轨迹自己的立体比例换成米
                c2 = tr.chi2_to(zt, o["Rw"], t, cfg.q_pos)
                if c2 <= cfg.gate_chi2 and (best is None or c2 < best[0]):
                    best = (c2, tr)
                dd = float(np.linalg.norm(zt - tr.x))
                if dd < cfg.track_radius and (near is None or dd < near[0]):
                    near = (dd, tr, c2)
            if best is None and near is not None:      # χ² 不过但离得近: 同一个红点换了视角 (外参 / VIO 偏差)
                best = (min(near[2], cfg.gate_chi2), near[1])    # χ² 记成门限值: 不因偏差把轨迹分压没
            if best is not None:
                tr = best[1]
                self._refine(o, tr, rate, R_WR, t_WR)           # 用这条轨迹的直径 / 比例
                c2 = tr.chi2_to(o["zw"], o["Rw"], t, cfg.q_pos)
                tr.hit(t, o, min(c2, cfg.gate_chi2) if best[0] >= cfg.gate_chi2 else c2)
            else:
                if any(self._same_dot(o["zw"], o["Rw"], tr, t_WR, t) and tr.id in updated for tr in self.tracks):
                    continue                           # 已更新过的轨迹旁边的重复观测
                if len(self.tracks) >= cfg.max_tracks:
                    sel_id = (self.sel or {}).get("trk_id")
                    cand = [x for x in self.tracks if x.id != sel_id] or self.tracks
                    self.tracks.remove(min(cand, key=lambda x: x.score))
                m0, sm0 = self._prior()
                o["zw"] = R_WR @ o["p"] * m0 + t_WR
                tr = _Track(self.next_id, t, o, m0, sm0)
                tr.D_seed = self._recall(o["zw"])               # 这里以前估过直径 → 直接用
                self._refine(o, tr, rate, R_WR, t_WR)
                tr.x, tr.P, tr.last = o["zw"].copy(), o["Rw"].copy(), o
                self.next_id += 1
                self.tracks.append(tr)
            tr.add_obs(o, t, cfg)
            tr.solve(t, cfg, cam=np.asarray(t_WR, float))
            updated[tr.id] = o
            if o["nc"] is not None:
                los = t_WR - o["zw"]
                los = los / max(float(np.linalg.norm(los)), 1e-9)
                tr.add_normal(R_WR @ o["nc"][0], R_WR @ o["nc"][1], o["n_sig"], los, t, cfg)
        self._merge(updated, t, np.asarray(t_WR, float))
        # 立体比例: 某条轨迹标得够准 → 记成本次运行的 (以后新轨迹的先验)
        #   (只认"主要靠自己的数据"标出来的: σ 比它建轨迹时的先验小很多, 免得先验绕一圈又回来自我加强)
        for tr in self.tracks:
            if tr.sm < cfg.scale_learn_sd and tr.sm < self.sm_glob and tr.sm < 0.7 * tr.sm0:
                self.m_glob, self.sm_glob = tr.m, tr.sm
        valid = [tr for tr in self.tracks if tr.n_hit >= cfg.min_hits and tr.score >= cfg.min_score]
        fresh = [tr for tr in valid if t - tr.t_upd <= cfg.lost_timeout]
        sel_tr = self._sel_resolve(np.asarray(t_WR, float))
        if self.sel is not None:
            # 选了点: 只发布选中的那条 (分数不够也发 —— 是人 / 飞行脚本选的); 没挂上 / 丢了 → 不发布, 不换成别的红点
            best = sel_tr
        else:
            best = max(fresh or valid, key=lambda tr: tr.score) if valid else None
        info = {"tracks": sorted(self.tracks, key=lambda tr: -tr.score), "best": best, "updated": updated}
        fr.dbg["dot"] = info
        fr.diag["n_trk"] = len(self.tracks)
        if best is None:
            fr.target, fr.stage = TargetState(), ("lost" if self.tracks else "search")
            return fr
        o = updated.get(best.id)
        Db = best.D(cfg)
        fr.diag["D_trk_cm"] = round(Db * 100, 2) if Db else -1.0
        fr.diag["D_n"] = len(best.dsamp)
        age = t - best.t_upd
        tg = TargetState(valid=True, pos_w=best.x.copy(), cov_w=best.P_at(t, cfg.drift_q),
                         src="dot_" + best.last["src"], age=age, dist=float(np.linalg.norm(best.x - t_WR)))
        fr.target = tg
        if o is not None:
            fr.used.append("dot_" + o["src"])
            fr.dots.append(DotObs(ok=True, src="stereo" if o["src"] in ("fused", "stereo") else "mono", p=o["p"],
                                  pix=(o["u"], o["v"]), diam_px=o["d_px"], metric_d=o["D"],
                                  sigma_lat=o["s_lat"], sigma_z=o["sZ"]))
            fr.diag.update(trk_id=best.id, trk_hits=best.n_hit, score=round(best.score, 3), z=round(o["Z"] * o["m"], 3),
                           z_st=round(o["Zst"] * o["m"], 3) if o["Zst"] else -1.0,
                           z_sz=round(o["Zsz"] * o["m"], 3) if o["Zsz"] else -1.0,
                           d_est_cm=round(o["D"] * 100, 2), sel_u=round(o["u"], 1), sel_v=round(o["v"], 1))
            if "tilt" in o:
                fr.diag["tilt_deg"] = round(o["tilt"], 1)
        fr.diag.update(st_scale=round(1.0 / best.m, 4), st_scale_sd=round(best.sm, 4), n_bins=best.n_bins,
                       n_dir=best.n_dir)
        if tg.dist < cfg.blind_dist:
            fr.stage = "blind"
        elif age <= 1e-6:
            fr.stage = "far" if tg.dist > 6.0 else ("mid" if tg.dist > 3.0 else "near")
        else:
            fr.stage = "coast" if age <= cfg.lost_timeout else "lost"
        close = [tr for tr in valid if tr.id != best.id and tr.score >= 0.7 * best.score
                 and np.linalg.norm(tr.x - best.x) > 0.5]
        fr.diag["n_close"] = len(close)
        return fr

    def _same_dot(self, x, Px, tr, cam, t):
        """x (协方差 Px) 和轨迹 tr 是不是同一个红点: 欧氏距离 < track_radius, 或者 横向 (垂直视线) < track_radius
        且马氏距离过 gate_chi2 —— 远处深度误差几十 cm, 同一个红点沿视线拆成两条时只看欧氏距离永远合不上。"""
        cfg = self.cfg
        d = x - tr.x
        if float(np.linalg.norm(d)) < cfg.track_radius:
            return True
        l = cam - tr.x
        ln = float(np.linalg.norm(l))
        if ln < 1e-6:
            return False
        l = l / ln
        lat = float(np.linalg.norm(d - l * float(d @ l)))
        if lat >= cfg.track_radius:
            return False
        try:
            return float(d @ np.linalg.solve(Px + tr.P_at(t, cfg.q_pos), d)) <= cfg.gate_chi2
        except np.linalg.LinAlgError:
            return False

    def _merge(self, updated, t, cam):
        """同一个红点被拆开的轨迹合并 (_same_dot), 留命中多的那条的编号 (选中的那条优先留)。"""
        sel_id = (self.sel or {}).get("trk_id")
        changed = True
        while changed:
            changed = False
            for a in self.tracks:
                for b in self.tracks:
                    if a is b or b.id < a.id or not self._same_dot(a.x, a.P, b, cam, t):
                        continue
                    if a.id == sel_id or b.id == sel_id:
                        keep, drop = (a, b) if a.id == sel_id else (b, a)
                    else:
                        keep, drop = (a, b) if (a.n_hit, -a.id) >= (b.n_hit, -b.id) else (b, a)
                    keep.absorb(drop, self.cfg)
                    keep.solve(t, self.cfg, cam=cam)
                    if drop.id in updated:
                        o = updated.pop(drop.id)
                        updated.setdefault(keep.id, o)
                    self.tracks.remove(drop)
                    changed = True
                    break
                if changed:
                    break

    # ---- /gate/status 附加字段 ----
    def extra(self, fr, T_WC, warn=None):
        """kind / aim / aim_sd_cm / D_est / D_trk (D_eff) / D_n / score / trk_id / n_hit / n_cand / cands / n / n_sd_deg /
        n_state / st_scale (立体比例 k) / sel (选点状态)"""
        out = {"kind": "reddot"}
        try:
            if self.sel is not None:
                sl = self.sel
                out["sel"] = {"seq": int(sl["seq"]), "mode": sl["mode"], "state": sl["state"],
                              "trk_id": sl.get("trk_id")}
            else:
                out["sel"] = {"seq": None, "mode": "none", "state": "none", "trk_id": None}
            R_WR, t_WR = self.T_WR(T_WC)
            info = fr.dbg.get("dot") or {}
            out["cands"] = [{"id": int(tr.id), "aim": [round(float(v), 3) for v in tr.x],
                             "score": round(float(tr.score), 3), "D": round(float(tr.last["D"]), 4),
                             "Z_st": round(float(tr.last["Zst"] * tr.m), 3) if tr.last["Zst"] else None,
                             "Z_sz": round(float(tr.last["Zsz"] * tr.m), 3) if tr.last.get("Zsz") else None,
                             "D_trk": _r4(tr.D(self.cfg)), "d_px": round(float(tr.last["d_px"]), 1),
                             "uv": [round(float(tr.last["u"]), 1), round(float(tr.last["v"]), 1)],
                             "sd_cm": round(float(np.sqrt(max(np.trace(tr.P), 0.0) / 3.0)) * 100, 1),
                             "hits": int(tr.n_hit), "age": round(float(fr.t - tr.t_upd), 2)}
                            for tr in (info.get("tracks") or [])[:5]]
            best = info.get("best")
            if R_WR is None or best is None:
                return out
            tg = fr.target
            out.update(aim=[round(float(v), 4) for v in tg.pos_w],
                       aim_sd_cm=[round(float(v) * 100, 2) for v in np.sqrt(np.maximum(np.diag(tg.cov_w), 0))],
                       D_est=round(float(best.last["D"]), 4),
                       # D_trk = 这条轨迹估出的直径 (没估好 = None, 这时只用双目); D_eff 是旧名字, 同值 (gcs_view 在读)
                       D_trk=_r4(best.D(self.cfg)), D_eff=_r4(best.D(self.cfg)), D_n=len(best.dsamp),
                       score=round(float(best.score), 3), trk_id=int(best.id), n_hit=int(best.n_hit),
                       n_cand=len([tr for tr in info["tracks"] if tr.n_hit >= self.cfg.min_hits]),
                       st_scale=round(1.0 / best.m, 4), st_scale_sd=round(best.sm / best.m ** 2, 4),
                       n_bins=int(best.n_bins), n_dir=int(best.n_dir))
            why = {}
            cons = normal_consensus([v[:4] for v in best.ns.values()], self.cfg, why)
            out["n_vbins"] = len(best.ns)             # 法向的视角桶数 (gcs_view 读的 n_bins 现在是位置桶数)
            out["n_state"] = why.get("n_state", "few")
            if cons is not None:
                n, n_sd, k, span = cons
                if float((t_WR - best.x) @ n) < 0:
                    n = -n                                  # 指向相机一侧 (进近侧)
                out.update(n=[round(float(v), 5) for v in n], n_sd_deg=round(float(n_sd), 2), n_k=int(k),
                           n_span_deg=round(float(span), 1))
            if why.get("n_why"):
                out["n_why"] = why["n_why"]
        except Exception as e:                              # 附加字段出错不能影响主链路
            if warn is not None:
                warn("[reddot] 附加字段出错: %r" % (e,))
        return out
