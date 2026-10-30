# -*- coding: utf-8 -*-
"""
vb09/perception/ring/fusion.py —— 世界系估计器 (TargetEstimator) + 融合外壳 (GateFuser)
===============================================================================
2026-10-30 从 gate_detector.py 原样搬来。只依赖 numpy (不 import cv2): 飞机侧 gate_gcs 模式
(vision_bridge.GateFuseWorker) 只导入这个文件, 不再拉起整个检测库。

★ 已核对过的事实 (PLAN_refactor_v2 §5, 不要"修"):
  GateFuser.T_WR 用 R_WR = R_WC·R1ᵀ —— 正确。校正旋转 R1 约 2.8°, 用校正系观测时必须乘 R1ᵀ,
  漏乘 = 5 m 处横向偏约 25 cm。q_pos 默认 0.02 (GateConfig)。
"""
from __future__ import annotations

from collections import deque

import numpy as np

from .config import GateConfig
from .frame import TargetState
from .geom3d import _angle

_N_FUSE = ("plane", "mono_plane", "mono_disp", "mono_est")     # 这几种法线才进估计器 (mono_mid 误差可到 ~10°, 只显示)


def _cov_along_ray(u, s_lat, s_z):
    u = u / np.linalg.norm(u)
    return s_lat ** 2 * (np.eye(3) - np.outer(u, u)) + s_z ** 2 * np.outer(u, u)


def _psd_max(P, F):
    """最小的 P' ⪰ P 且 P' ⪰ F (把 F−P 的正特征部分补上)。用来给弱观测的协方差设下限。"""
    D = 0.5 * ((F - P) + (F - P).T)
    lam, V = np.linalg.eigh(D)
    if lam.max() <= 0:
        return P
    return P + (V * np.maximum(lam, 0.0)) @ V.T


class TargetEstimator:
    """红点世界坐标（常位置 + 随机游走）；同时维护环心、环法线、红点相对环心的偏移、正反面。"""

    def __init__(self, cfg: GateConfig):
        self.cfg = cfg
        self.x = None
        self.P = None
        self.t = None
        self.t_upd = None
        self.src = ""
        self.n_rej = 0
        self.rej_z = None
        self.t_dot = None             # 上次被接受的红点观测 (dot_*) 时刻
        self.ring_c = None
        self.ring_c_w = 0.0
        self.ring_c_t = None
        self.ring_n_sum = None        # 角点平面 (双目) 法线
        self.ring_n_cnt = 0
        self.ring_nm = deque(maxlen=15)   # 最近的单目定过向法线 (mono_plane / mono_disp / mono_est), 远处用
        self.offset_sum = np.zeros(3)
        self.offset_cnt = 0
        self.side_votes = {"front": 0.0, "back": 0.0}
        self.side = None

    # ---- 状态 ----
    @property
    def ring_n_stereo(self):
        if self.ring_n_sum is None or self.ring_n_cnt < 3:
            return None
        return self.ring_n_sum / np.linalg.norm(self.ring_n_sum)

    @property
    def ring_n(self):
        """双目法线优先; 没有时用一致的单目定向法线 (2026-10-21: 以前单目法线算了但从没进来)。"""
        n = self.ring_n_stereo
        if n is not None:
            return n
        if len(self.ring_nm) < self.cfg.mono_n_min_cnt:
            return None
        S = np.sum(self.ring_nm, axis=0)
        L = float(np.linalg.norm(S))
        if L / len(self.ring_nm) < self.cfg.mono_n_min_rbar:
            return None                   # 最近这些帧彼此不一致 (有挑错的解) → 先不给
        return S / L

    @property
    def offset(self):
        return self.offset_sum / self.offset_cnt if self.offset_cnt >= 3 else None

    def predict(self, t):
        """协方差随时间膨胀（随机游走）。self.t = 协方差对应的时刻；self.t_upd = 上次被接受的观测时刻。"""
        if self.x is not None and t > self.t:
            self.P = self.P + (self.cfg.q_pos ** 2) * (t - self.t) * np.eye(3)
            self.t = t

    def update(self, z, R, src, t, strong, floor=None):
        """返回 True=接受。强观测连续被拒 reinit_after 次后重置（纸被吹走/初值错）。
        floor: 弱观测的协方差下限 (误差是系统性的, 多帧平均不掉) —— 更新后 P ⪰ floor。"""
        ok = self._update(z, R, src, t, strong)
        if ok:
            if floor is not None:
                self.P = _psd_max(self.P, floor)
            if src.startswith("dot"):
                self.t_dot = t
        return ok

    def _update(self, z, R, src, t, strong):
        if self.x is None:
            self.x, self.P, self.t, self.t_upd, self.src = z.copy(), R.copy(), t, t, src
            return True
        S = self.P + R
        d = z - self.x
        chi2 = float(d @ np.linalg.solve(S, d))
        if chi2 > self.cfg.gate_chi2:
            if strong:
                # 连续被拒、而且被拒的观测彼此一致（都指向同一个新位置）→ 目标真的移动了（纸被吹走）
                tol = 3 * np.sqrt(np.trace(R)) + 0.01
                if self.rej_z is not None and np.linalg.norm(z - self.rej_z) < tol:
                    self.n_rej += 1
                else:
                    self.n_rej = 1
                self.rej_z = z.copy()
                if self.n_rej >= self.cfg.reinit_after:
                    self.x, self.P, self.t, self.t_upd = z.copy(), R.copy(), t, t
                    self.src, self.n_rej, self.rej_z = src + "(reinit)", 0, None
                    return True
            return False
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ d
        self.P = (np.eye(3) - K) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        self.t_upd, self.src = t, src
        if strong:
            self.n_rej, self.rej_z = 0, None
        return True

    def update_ring(self, c_w, n_w, sigma, stereo, t=None):
        """
        环心 (加权平均) + 法线。n_w: 双目平面法线 (stereo=True) 或已定过向的单目法线 (stereo=False)。
        2026-10-21: ① 权重按 ring_c_halflife 衰减、上限 ring_c_w_max (以前只增不减到 1e6, VINS 漂移 >0.3m 后
                    新观测全当离群丢掉, 环心冻住);  ② 单目定向法线进 ring_nm (以前 stereo=False 时法线直接丢)。
        """
        cfg = self.cfg
        if t is not None and self.ring_c_t is not None and t > self.ring_c_t and cfg.ring_c_halflife > 0:
            self.ring_c_w *= 0.5 ** ((t - self.ring_c_t) / cfg.ring_c_halflife)
        if t is not None:
            self.ring_c_t = t if self.ring_c_t is None else max(self.ring_c_t, t)
        w = 1.0 / max(sigma, 1e-3) ** 2
        if self.ring_c is None or np.linalg.norm(c_w - self.ring_c) > max(0.3, 5 * sigma):
            if self.ring_c is not None and self.ring_c_w > 3 * w:
                return  # 离群，丢掉
            self.ring_c, self.ring_c_w = c_w.copy(), min(w, cfg.ring_c_w_max)
        else:
            self.ring_c = (self.ring_c * self.ring_c_w + c_w * w) / (self.ring_c_w + w)
            self.ring_c_w = min(self.ring_c_w + w, cfg.ring_c_w_max)
        if n_w is None:
            return
        n_w = n_w / np.linalg.norm(n_w)
        if stereo:
            if self.ring_n_sum is None:
                self.ring_n_sum = n_w.copy()
            else:
                ref = self.ring_n_sum / np.linalg.norm(self.ring_n_sum)
                if n_w @ ref < 0:
                    n_w = -n_w
                if n_w @ ref > np.cos(np.radians(15)) or self.ring_n_cnt < 3:
                    self.ring_n_sum = self.ring_n_sum + n_w
            self.ring_n_cnt += 1
        else:
            ref = self.ring_n if self.ring_n is not None else (self.ring_nm[-1] if self.ring_nm else None)
            if ref is not None and n_w @ ref < 0:
                n_w = -n_w
            self.ring_nm.append(n_w)

    def vote_side(self, side, conf):
        if side in self.side_votes and self.side is None:
            self.side_votes[side] += conf
            for s_, v in self.side_votes.items():
                if v >= self.cfg.side_lock_votes:
                    self.side = s_

    def state(self, t, cam_pos):
        ts = TargetState(side=self.side, ring_c_w=self.ring_c, ring_n_w=self.ring_n, offset_w=self.offset)
        if self.x is None:
            return ts
        ts.valid = True
        ts.pos_w = self.x.copy()
        ts.cov_w = self.P + (self.cfg.q_pos ** 2) * max(0.0, t - self.t) * np.eye(3)
        ts.src, ts.age = self.src, t - self.t_upd
        ts.dist = float(np.linalg.norm(self.x - cam_pos))
        return ts


# =============================================================================
# 世界系融合（估计器外壳）
# =============================================================================
class GateFuser:
    """
    把"相机(rect)系观测"融合成"红点世界坐标"。只需要校正后左目的 f 和 R1, 不碰图像。

      机上一体 (_mode:=gate)       GateDetector 内部持有一个, process() 结尾调用 update()
      地面检测 (_mode:=gate_gcs)   飞机上单独持有一个: 收到地面的观测 → frame_from_msg()
                                   → update(fr, T_WC(t_cap), t_cap)
                                   位姿始终在飞机上插值, 不过网络; 估计器状态也留在飞机上,
                                   断链/地面重启都不会把它清掉。
    """

    def __init__(self, cfg: GateConfig, f, R1):
        self.cfg = cfg
        self.f = float(f)
        self.R1 = np.asarray(R1, float).reshape(3, 3)
        self.est = TargetEstimator(cfg)

    def same_rig(self, f, R1, tol=1e-4):
        return abs(float(f) - self.f) <= tol * self.f and \
            np.allclose(np.asarray(R1, float).reshape(3, 3), self.R1, atol=tol)

    def T_WR(self, T_WC):
        """rect 系 → 世界: R_WR = R_WC·R1ᵀ"""
        if T_WC is None:
            return None, None
        T_WC = np.asarray(T_WC, float)
        return T_WC[:3, :3] @ self.R1.T, T_WC[:3, 3]

    def update(self, fr, T_WC, t):
        """fr 里的观测(rect 系) + 位姿 → fr.target / fr.used / fr.rejected / fr.stage。T_WC=None 只出阶段。"""
        R_WR, t_WR = self.T_WR(T_WC)
        if R_WR is not None:
            self._update_estimator(fr, R_WR, t_WR, t)
            fr.target = self.est.state(t, t_WR)
        else:
            fr.target = TargetState()
        fr.stage = self.stage(fr)
        return fr

    def reset(self):
        """整个估计器清零 (红点位置 / 环心 / 法线 / 偏移 / 正反面)。换纸、挪环、每次起飞前调用。"""
        self.est = TargetEstimator(self.cfg)

    def _update_estimator(self, fr, R_WR, t_WR, t):
        cfg, est, ring = self.cfg, self.est, fr.ring
        est.predict(t)
        to_w = lambda p: R_WR @ p + t_WR
        cov_w = lambda u, sl, sz: R_WR @ _cov_along_ray(u, sl, sz) @ R_WR.T
        # 环
        if ring.ok:
            if ring.ambiguous and ring.n_alt is not None and est.ring_n is not None:
                # 地面检测没有位姿, 分不清的单目两解到这里按已存的环法线再挑一次
                a = [_angle(R_WR @ ring.n, est.ring_n), _angle(R_WR @ ring.n_alt, est.ring_n)]
                if abs(a[0] - a[1]) >= self.cfg.normal_sep_deg and min(a) <= self.cfg.normal_ref_max_deg:
                    if a[1] < a[0]:
                        ring.n, ring.n_alt = ring.n_alt, ring.n
                        if ring.c_alt is not None:          # 两个解的环心也不同, 一起换
                            ring.c, ring.c_alt = ring.c_alt, ring.c
                    ring.ambiguous, ring.n_src = False, "mono_est"
            cw = to_w(ring.c)
            # 法线: 角点平面的, 或已经定过向的单目解 (远处比噪声大的角点平面还准) 都能用
            nw = R_WR @ ring.n if (ring.src == "stereo" or ring.n_src in _N_FUSE) else None
            est.update_ring(cw, nw, max(ring.sigma_lat, ring.sigma_z), ring.src == "stereo", t)
            if ring.side:
                est.vote_side(ring.side, ring.side_conf)
        # 红点：强观测。stereo 和 ray_plane 是同一个左目像素 (横向是同一份信息), 以前两个都融合 → 横向 σ
        # 低估 √2。2026-10-21: 只融合协方差小的那个。
        strong_obs = []
        for d in fr.dots:
            if d.src in ("stereo", "ray_plane"):
                strong_obs.append((d.src, to_w(d.p), cov_w(d.p, d.sigma_lat, d.sigma_z)))
        if len(strong_obs) > 1:
            strong_obs = [min(strong_obs, key=lambda o: float(np.trace(o[2])))]
        for src, z, Rw in strong_obs:
            (fr.used if est.update(z, Rw, "dot_" + src, t, True) else fr.rejected).append("dot_" + src)
        # 学偏移（红点与环心都由双目给出时）
        if ring.ok and ring.src == "stereo" and strong_obs and "dot_" + strong_obs[0][0] in fr.used:
            est.offset_sum += strong_obs[0][1] - to_w(ring.c)
            est.offset_cnt += 1
        if strong_obs:
            return
        # 单目 + 已存环平面 (只用双目定的平面: 单目环心的深度有 ~3%·Z 误差, 当平面用会把红点深度带偏)
        d = fr.dot("mono")
        n_st = est.ring_n_stereo
        if d is not None and n_st is not None and est.ring_c is not None:
            uw = R_WR @ d.p
            den = uw @ n_st
            if abs(den) > 1e-3:
                s = ((est.ring_c - t_WR) @ n_st) / den
                if s > 0:
                    z = t_WR + uw * s
                    ur = d.p
                    zr = s * ur[2] / np.linalg.norm(ur)
                    Rw = cov_w(ur, max(0.003, zr * cfg.sigma_px / self.f), 0.01 + 0.01 * zr)
                    (fr.used if est.update(z, Rw, "dot_mono_plane", t, False, floor=Rw)
                     else fr.rejected).append("dot_mono_plane")
                    return
        # 只有环
        if ring.ok:
            cw = to_w(ring.c)
            sr = max(ring.sigma_lat, ring.sigma_z)
            R_ring = cov_w(ring.c, ring.sigma_lat, ring.sigma_z)       # 本帧环心观测 (沿视线拉长)
            if est.offset is not None:
                s = np.sqrt(sr ** 2 + 0.02 ** 2)
                (fr.used if est.update(cw + est.offset, np.eye(3) * s ** 2, "ring_offset", t, False,
                                       floor=R_ring + np.eye(3) * 0.02 ** 2)
                 else fr.rejected).append("ring_offset")
            elif (est.x is None or est.src.startswith("ring")
                  or est.t_dot is None or t - est.t_dot > cfg.ring_takeover_s):
                # 2026-10-21: 红点 ring_takeover_s 没更新了 → 环接管 (以前有过一次红点观测后, 只看到环的帧
                #   整帧被忽略, 目标一直老化直到桥把它重置)
                s = np.sqrt(sr ** 2 + 0.3 ** 2)
                (fr.used if est.update(cw, np.eye(3) * s ** 2, "ring_prior", t, False,
                                       floor=R_ring + np.eye(3) * cfg.ring_prior_floor ** 2)
                 else fr.rejected).append("ring_prior")
            else:
                fr.diag["ring_wait"] = True     # 红点刚丢 (< ring_takeover_s), 这帧的环先不进估计器

    def stage(self, fr):
        cfg, ring = self.cfg, fr.ring
        dot_seen = any(d.ok for d in fr.dots)
        tg = fr.target
        if tg.valid and tg.dist == tg.dist and tg.dist < cfg.blind_dist:
            return "blind"
        if ring.ok and ring.src == "stereo":
            return "near" if ring.partial else "mid"
        if ring.ok:
            return "near" if ring.partial else "far"
        if dot_seen and (fr.used or not tg.valid):
            return "terminal"
        if fr.ring_seen:
            return "far"
        if tg.valid:
            return "coast" if tg.age <= cfg.lost_timeout else "lost"
        return "search"
