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
    def __init__(self, tid, z, R, t, info):
        self.id = tid
        self.x, self.P = z.copy(), R.copy()
        self.t, self.t_upd = t, t
        self.n_hit = 1
        self.ema = info["score"]
        self.chi2 = deque(maxlen=8)
        self.ns = {}                         # 视角桶 → (n1_w, n2_w, σ_deg, los_w, t)。按视角存: 悬停几百帧只占一桶
        self.last = info
        self.dsamp = deque(maxlen=60)        # 直径样本 (近处静止双目帧的 D_st)
        self.D_seed = None                   # 按位置记住的直径 (估计器重建前 / 删掉的轨迹留下的)

    def D(self, cfg):
        """这条轨迹的直径 (m) 或 None。dot_d_cal > 0 → 用它; 样本够 → 中位数; 否则记住的值。"""
        if cfg.dot_d_cal > 0:
            return cfg.dot_d_cal
        if len(self.dsamp) >= cfg.cal_min_n:
            return float(np.median(self.dsamp))
        return self.D_seed

    def add_normal(self, n1, n2, sig, los, t, cfg):
        az = math.degrees(math.atan2(los[1], los[0]))
        el = math.degrees(math.asin(max(-1.0, min(1.0, float(los[2])))))
        b = cfg.normal_bin_deg
        self.ns[(int(round(az / b)), int(round(el / b)))] = (n1, n2, sig, los, t)
        if len(self.ns) > cfg.normal_buf:                  # 超了 → 丢最旧的桶
            del self.ns[min(self.ns, key=lambda k: self.ns[k][4])]

    def absorb(self, other):
        """把另一条 (同一个红点被拆出来的) 轨迹并进来。"""
        S = self.P + other.P
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ (other.x - self.x)
        self.P = 0.5 * ((_I3 - K) @ self.P + ((_I3 - K) @ self.P).T)
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

    def predict(self, t, q):
        if t > self.t:
            self.P = self.P + (q ** 2) * (t - self.t) * _I3
            self.t = t

    def chi2_to(self, z, R):
        d = z - self.x
        return float(d @ np.linalg.solve(self.P + R, d))

    def update(self, z, R, t, info, chi2):
        S = self.P + R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x)
        self.P = (_I3 - K) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
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
    def no(r):
        if why is not None:
            why["n_why"] = r
        return None
    if len(ns) < cfg.normal_min_k:
        return no("视角桶 %d < %d" % (len(ns), cfg.normal_min_k))
    inl = cfg.normal_inlier_deg
    fits = [_fit(v, ns, inl) for x in ns for v in (x[0], x[1])]
    n, cost, idx = min(fits, key=lambda z: z[1])
    if len(idx) < cfg.normal_min_k:
        return no("一致的视角桶 %d < %d" % (len(idx), cfg.normal_min_k))
    span = _view_span([ns[i][3] for i in idx])
    if span < cfg.normal_view_min_deg:
        return no("视角跨度 %.1f° < %.0f° (单个视角分不开两个解)" % (span, cfg.normal_view_min_deg))
    alt = [z[1] for z in fits if _ang(z[0], n) > 2.0 * inl]
    lr = (min(alt) - cost) if alt else float("inf")      # 没有离得远的替代解 (近正对: 两个候选本来就挨着)
    if lr < cfg.normal_lr_min:
        return no("两个解都说得通 (代价差 %.1f < %.0f)" % (lr, cfg.normal_lr_min))
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
            return n, n_sd, k, span
    if why is not None:
        why["n_why"] = ""
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
        self.dmem = []                       # [(世界位置, 直径)]: 估过直径的红点 (轨迹删掉 / 估计器重建后还在)
        self._prev = None                    # (R_WR, t) 上一帧, 算相机角速度

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
        self._prev = None                    # dmem 保留: 红点直径和估计器状态无关

    def inherit(self, old):
        """估计器重建时 (桥的 _stale 重置) 从旧的 Locator 带过来已经估好的直径。轨迹 / 位置不带。"""
        try:
            self._remember(old)
            self.dmem = list(old.dmem)[-16:]
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
        """给显示 / 状态用: 轨迹直径 (m) 或 None。"""
        return tr.D(self.cfg) if tr is not None else None

    # ---- 单个候选 → 观测 (rect 系) 或 拒绝原因 ----
    def _obs(self, c):
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
            if not (cfg.d_min <= Dst <= cfg.d_max):
                return None, "size"
            Z, sZ, src = Zst, sst, "stereo"               # 先只用双目; 轨迹有直径后在 _refine 里融合尺寸
        else:
            if c.nw < cfg.white_min or score < cfg.mono_score_min:
                return None, "mono_weak"
            Z = f * cfg.dot_d / c.dmaj                    # 宽先验; 关联到有直径的轨迹后在 _refine 里换成轨迹直径
            sZ, src = Z * cfg.size_prior_rel, "size_prior"
        ray = np.array([c.xn, c.yn, 1.0])
        rn = float(np.linalg.norm(ray))
        o = dict(score=score, Zst=Zst, sst=sst, Zsz=None, Dst=Dst, d_px=c.dmaj, u=c.u, v=c.v,
                 disp=disp if st_ok else None, ray=ray, rn=rn, s_lat=Z * cfg.sigma_px * s / f)
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

    def _set_depth(self, o, Z, sZ, src, s_mov=0.0):
        """按深度 Z (σ sZ) 填观测的 rect 系位置和协方差 (含外参 / VIO 下限和转动项)。"""
        cfg, ray, rn = self.cfg, o["ray"], o["rn"]
        s_lat = Z * cfg.sigma_px * self.s / self._f
        Rc = _cov_along_ray(ray, s_lat, sZ * rn)
        s_ang = Z * rn * math.radians(cfg.obs_ang_floor_deg)              # 外参 / VIO 误差 (只用于关联和融合)
        lat = _I3 - np.outer(ray, ray) / rn ** 2
        Rc = Rc + (s_ang ** 2 + s_mov ** 2) * lat + cfg.obs_pos_floor ** 2 * _I3
        o.update(p=Z * ray, Rc=Rc, Z=Z, sZ=sZ, src=src, D=Z * o["d_px"] / self._f, s_lat=s_lat)

    def _refine(self, o, tr, rate, R_WR, t_WR):
        """关联到轨迹 tr (或新轨迹, tr = None) 之后: 用这条轨迹的直径融合尺寸测距; 攒直径样本。"""
        cfg, f = self.cfg, self._f
        moving = rate > cfg.rate_max_deg
        D = tr.D(cfg) if tr is not None else None
        s_mov = o["Z"] * o["rn"] * math.radians(rate) * cfg.motion_dt
        if D is not None:
            Zsz = f * D / o["d_px"]
            ssz = Zsz * math.sqrt(cfg.size_sigma_floor ** 2 + (cfg.size_edge_px * self.s / o["d_px"]) ** 2)
            o["Zsz"] = Zsz
            if o["Zst"] is not None:
                if moving or abs(o["Zst"] - Zsz) > cfg.consist_k * math.sqrt(o["sst"] ** 2 + ssz ** 2):
                    self._set_depth(o, o["Zst"], o["sst"], "stereo", s_mov)          # 只用双目
                else:
                    wst, wsz = 1.0 / o["sst"] ** 2, 1.0 / ssz ** 2
                    self._set_depth(o, (wst * o["Zst"] + wsz * Zsz) / (wst + wsz), 1.0 / math.sqrt(wst + wsz),
                                    "fused", s_mov)
            else:
                self._set_depth(o, Zsz, ssz * (2.0 if moving else 1.0), "size", s_mov)
        elif s_mov > 0:
            self._set_depth(o, o["Z"], o["sZ"], o["src"], s_mov)
        o["zw"] = R_WR @ o["p"] + t_WR
        o["Rw"] = R_WR @ o["Rc"] @ R_WR.T
        o["moving"] = moving
        o["D_trk"] = D
        # 攒直径样本: 要比"不用尺寸测距"更静 (半个 rate_max_deg) —— 慢转时的轻微模糊也会把长轴拉长
        if tr is not None and rate <= 0.5 * cfg.rate_max_deg and o["Dst"] is not None and o["disp"] is not None \
                and o["disp"] >= cfg.cal_disp_min * self.s:
            tr.dsamp.append(o["Dst"])

    # ---- 主入口 ----
    def update(self, fr: DotFrame, T_WC, t):
        cfg = self.cfg
        fr.used, fr.rejected, fr.dots = [], [], []
        obs, rej = [], {}
        for c in fr.cands:
            o, why = self._obs(c)
            if o is None:
                rej[why] = rej.get(why, 0) + 1
            else:
                obs.append(o)
        fr.rejected = ["%s×%d" % (k, v) for k, v in sorted(rej.items())]
        fr.diag.update(n_cand=len(fr.cands), n_pass=len(obs))
        R_WR, t_WR = self.T_WR(T_WC)
        if R_WR is None:
            fr.target, fr.stage = TargetState(), "search"
            fr.dbg["dot"] = {}
            return fr
        rate = self._rate(R_WR, t)
        fr.diag["rate_dps"] = round(rate, 1)
        self._remember()
        self.tracks = [tr for tr in self.tracks if t - tr.t_upd <= cfg.track_timeout]
        for tr in self.tracks:
            tr.predict(t, cfg.q_pos)
        for o in obs:
            o["zw"] = R_WR @ o["p"] + t_WR
            o["Rw"] = R_WR @ o["Rc"] @ R_WR.T
        obs.sort(key=lambda o: -o["score"])
        updated = {}
        for o in obs:
            best = near = None
            for tr in self.tracks:
                if tr.id in updated:
                    continue
                c2 = tr.chi2_to(o["zw"], o["Rw"])
                if c2 <= cfg.gate_chi2 and (best is None or c2 < best[0]):
                    best = (c2, tr)
                dd = float(np.linalg.norm(o["zw"] - tr.x))
                if dd < cfg.track_radius and (near is None or dd < near[0]):
                    near = (dd, tr, c2)
            if best is None and near is not None:      # χ² 不过但离得近: 同一个红点换了视角 (外参 / VIO 偏差)
                best = (min(near[2], cfg.gate_chi2), near[1])    # χ² 记成门限值: 不因偏差把轨迹分压没
            if best is not None:
                tr = best[1]
                self._refine(o, tr, rate, R_WR, t_WR)           # 用这条轨迹的直径融合尺寸测距
                c2 = tr.chi2_to(o["zw"], o["Rw"])
                tr.update(o["zw"], o["Rw"], t, o, min(c2, cfg.gate_chi2) if best[0] >= cfg.gate_chi2 else c2)
            else:
                if any(np.linalg.norm(o["zw"] - tr.x) < cfg.track_radius and tr.id in updated for tr in self.tracks):
                    continue                           # 已更新过的轨迹旁边的重复观测
                if len(self.tracks) >= cfg.max_tracks:
                    self.tracks.remove(min(self.tracks, key=lambda x: x.score))
                tr = _Track(self.next_id, o["zw"], o["Rw"], t, o)
                tr.D_seed = self._recall(o["zw"])               # 这里以前估过直径 → 直接用
                self._refine(o, tr, rate, R_WR, t_WR)
                tr.x, tr.P, tr.last = o["zw"].copy(), o["Rw"].copy(), o
                self.next_id += 1
                self.tracks.append(tr)
            updated[tr.id] = o
            if o["nc"] is not None:
                los = t_WR - o["zw"]
                los = los / max(float(np.linalg.norm(los)), 1e-9)
                tr.add_normal(R_WR @ o["nc"][0], R_WR @ o["nc"][1], o["n_sig"], los, t, cfg)
        self._merge(updated)
        valid = [tr for tr in self.tracks if tr.n_hit >= cfg.min_hits and tr.score >= cfg.min_score]
        fresh = [tr for tr in valid if t - tr.t_upd <= cfg.lost_timeout]
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
        tg = TargetState(valid=True, pos_w=best.x.copy(),
                         cov_w=best.P + (cfg.q_pos ** 2) * max(0.0, t - best.t) * _I3,
                         src="dot_" + best.last["src"], age=age, dist=float(np.linalg.norm(best.x - t_WR)))
        fr.target = tg
        if o is not None:
            fr.used.append("dot_" + o["src"])
            fr.dots.append(DotObs(ok=True, src="stereo" if o["src"] in ("fused", "stereo") else "mono", p=o["p"],
                                  pix=(o["u"], o["v"]), diam_px=o["d_px"], metric_d=o["D"],
                                  sigma_lat=o["s_lat"], sigma_z=o["sZ"]))
            fr.diag.update(trk_id=best.id, trk_hits=best.n_hit, score=round(best.score, 3), z=round(o["Z"], 3),
                           z_st=round(o["Zst"], 3) if o["Zst"] else -1.0,
                           z_sz=round(o["Zsz"], 3) if o["Zsz"] else -1.0,
                           d_est_cm=round(o["D"] * 100, 2), sel_u=round(o["u"], 1), sel_v=round(o["v"], 1))
            if "tilt" in o:
                fr.diag["tilt_deg"] = round(o["tilt"], 1)
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

    def _merge(self, updated):
        """靠得比 track_radius 近的轨迹合并 (同一个红点被拆开的), 留命中多的那条的编号。"""
        r = self.cfg.track_radius
        changed = True
        while changed:
            changed = False
            for a in self.tracks:
                for b in self.tracks:
                    if a is b or float(np.linalg.norm(a.x - b.x)) >= r:
                        continue
                    keep, drop = (a, b) if (a.n_hit, -a.id) >= (b.n_hit, -b.id) else (b, a)
                    keep.absorb(drop)
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
        """kind / aim / aim_sd_cm / D_est / D_trk (D_eff) / D_n / score / trk_id / n_hit / n_cand / cands / n / n_sd_deg"""
        out = {"kind": "reddot"}
        try:
            R_WR, t_WR = self.T_WR(T_WC)
            info = fr.dbg.get("dot") or {}
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
                       cands=[{"id": int(tr.id), "aim": [round(float(v), 3) for v in tr.x],
                               "score": round(float(tr.score), 3), "D": round(float(tr.last["D"]), 4),
                               "Z_st": round(float(tr.last["Zst"]), 3) if tr.last["Zst"] else None,
                               "Z_sz": round(float(tr.last["Zsz"]), 3) if tr.last.get("Zsz") else None,
                               "D_trk": _r4(tr.D(self.cfg)), "d_px": round(float(tr.last["d_px"]), 1),
                               "uv": [round(float(tr.last["u"]), 1), round(float(tr.last["v"]), 1)],
                               "hits": int(tr.n_hit)} for tr in info["tracks"][:5]])
            why = {}
            cons = normal_consensus([v[:4] for v in best.ns.values()], self.cfg, why)
            out["n_bins"] = len(best.ns)
            if cons is not None:
                n, n_sd, k, span = cons
                if float((t_WR - best.x) @ n) < 0:
                    n = -n                                  # 指向相机一侧 (进近侧)
                out.update(n=[round(float(v), 5) for v in n], n_sd_deg=round(float(n_sd), 2), n_k=int(k),
                           n_span_deg=round(float(span), 1))
            elif why.get("n_why"):
                out["n_why"] = why["n_why"]
        except Exception as e:                              # 附加字段出错不能影响主链路
            if warn is not None:
                warn("[reddot] 附加字段出错: %r" % (e,))
        return out
