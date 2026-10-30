# -*- coding: utf-8 -*-
"""
vb09/perception/ring/detect.py —— 环 + 红点检测主类 GateDetector
===============================================================================
2026-10-30 从 gate_detector.py 搬来。process() 拆成几个阶段方法 (_find_ring_2d / _ring_diag /
_locate_ring / _find_dots / _remember_ref), 语句和顺序不变 (tools/baseline/det_replay 逐字段对拍)。
总体思路 / 观测种类 / 阶段 / 坐标系见 gate_detector.py 文件头 (那里保留了完整说明)。

  process(left_rect, right_rect, t, T_WC=None) → GateFrame
    T_WC=None (地面检测): 只出相机 (rect) 系观测, 估计器不动 —— 由飞机侧的 GateFuser 融合。
"""
from __future__ import annotations

import time

import numpy as np

from .config import GateConfig
from .det2d import (_el_points, _shift_el, apply_wb, find_corners, find_dot, find_gate, hue_side,
                    segment)
from .frame import DotObs, GateFrame
from .fusion import GateFuser
from .geom3d import (_ray, circle_pose_from_ellipse, ellipse_disparity_normal, fit_plane,
                     match_corners, pick_ring_normal, ring_center_in_plane, ring_tilt_deg)


class GateDetector:
    def __init__(self, rig, cfg: GateConfig = None, seed=0):
        self.rig = rig
        self.cfg = cfg or GateConfig()
        self.rng = np.random.default_rng(seed)
        self.fuser = GateFuser(self.cfg, rig.f, rig.R1)
        self.prior_el, self.prior_age = None, 0
        self.last_dot_pix, self.last_dot_z = None, None
        self.last_dist = None
        self.last_ref_t = None        # last_dot_* / last_dist 对应的帧时刻 (过期就不用, 见 last_dot_max_age)
        self.n_frame = 0
        self._ring_fail = {}          # 上一次真正搜过却没找到环时的诊断 (跳过搜索的帧沿用它)

    @property
    def est(self):
        return self.fuser.est

    def reset(self):
        self.__init__(self.rig, self.cfg)

    # ---------------- 坐标 ----------------
    def _T_WR(self, T_WC):
        return self.fuser.T_WR(T_WC)

    def _project(self, Pw, R_WR, t_WR):
        Pr = R_WR.T @ (Pw - t_WR)
        if Pr[2] <= 0.05:
            return None, Pr
        return (self.rig.f * Pr[0] / Pr[2] + self.rig.cx, self.rig.f * Pr[1] / Pr[2] + self.rig.cy), Pr

    # ---------------- 计划本帧算什么 ----------------
    def _plan(self, R_WR, t_WR, t=None):
        """
        没位姿 (地面检测) 时用上一帧红点的像素/深度 —— 末段 ROI 照样能开。
        2026-10-21: 这份"上一帧"超过 last_dot_max_age 就不用了。以前永不过期: 撞纸前最后几帧 <1m,
        之后红点没了 (纸破了 / 落地后没对着纸) → 每帧 skip_ring → 地面检测器再也不找环, 只能重启。
        """
        cfg, rig = self.cfg, self.rig
        dist, pix, z = None, None, None
        fresh = (self.last_ref_t is not None and t is not None
                 and abs(t - self.last_ref_t) <= cfg.last_dot_max_age)
        if R_WR is not None and self.est.x is not None:
            pix, Pr = self._project(self.est.x, R_WR, t_WR)
            z = Pr[2]
            dist = float(np.linalg.norm(Pr))
        elif fresh and self.last_dist is not None:
            pix, z, dist = self.last_dot_pix, self.last_dot_z, self.last_dist
        skip_ring = dist is not None and dist < cfg.ring_skip_dist
        roi = None
        if skip_ring and pix is not None and z and z > 0:
            pd = rig.f * cfg.dot_d / z
            hw = cfg.dot_roi_scale * pd + 20
            roi = (pix[0] - hw, pix[1] - hw, pix[0] + hw, pix[1] + hw)
        return skip_ring, roi, dist, z

    # ---------------- 每帧 ----------------
    def process(self, left, right, t=None, T_WC=None) -> GateFrame:
        """
        left/right: 校正后的左右目。T_WC: 原始左目相机→世界 (4x4)。
        T_WC=None (地面检测): 只出相机系观测, 估计器不动 —— 由飞机侧的 GateFuser 融合。
        2026-10-30: 拆成几个阶段方法, 语句和顺序不变。
        """
        cfg = self.cfg
        t = time.time() if t is None else float(t)
        fr = GateFrame(t=t)
        self.n_frame += 1
        t0 = time.perf_counter()
        L, R = apply_wb(left, cfg), apply_wb(right, cfg)
        R_WR, t_WR = self._T_WR(T_WC)
        skip_ring, roi, dist_pred, z_pred = self._plan(R_WR, t_WR, t)
        fr.skipped_ring, fr.dot_roi = skip_ring, roi
        gL, gR, segL, segR = self._find_ring_2d(fr, L, R, skip_ring, t0)     # 环: 左右目椭圆
        self._ring_diag(fr, gL, segL)                                         # 环找不到时卡在哪
        plane = self._locate_ring(fr, L, gL, gR, segL, segR, R_WR)            # 环: 角点平面 / 单目位姿
        self._find_dots(fr, L, R, gL, gR, roi, z_pred, plane)                 # 红点
        self._remember_ref(fr, t)                                             # 下一帧 _plan 用

        # ---------- 融合 (T_WC=None 时只出阶段) ----------
        self.fuser.update(fr, T_WC, t)
        fr.t_ms["total"] = (time.perf_counter() - t0) * 1e3
        return fr

    # ---------------- 环: 左右目 2D ----------------
    def _find_ring_2d(self, fr, L, R, skip_ring, t0):
        """→ (gL, gR, segL, segR)。顺带设 fr.ring_seen 和耗时。"""
        cfg, rig = self.cfg, self.rig
        # ---------- 环 ----------
        gL = gR = None
        segL = segR = None
        if not skip_ring:
            prior = self.prior_el if self.prior_age <= cfg.prior_max_age else None
            if prior is None and cfg.search_every > 1 and ((self.n_frame - 1) % int(cfg.search_every)) != 0:
                # 环丢了: 全图 RANSAC 全搜很贵, 每 search_every 帧才做一次。红点照常每帧找。
                gL = {"ok": False, "why": "search_skip"}
                fr.diag.update(self._ring_fail)
                fr.diag["search_skip"] = True
            else:
                segL = segment(L, cfg)
                gL = find_gate(segL, cfg, prior, self.rng)
            fr.t_ms["ringL"] = (time.perf_counter() - t0) * 1e3
            if gL["ok"]:
                self.prior_el, self.prior_age = gL["center"], 0
                ta = time.perf_counter()
                segR = segment(R, cfg)
                d_pred = rig.B * gL["center_major"] / (2 * cfg.r_center)
                gR = find_gate(segR, cfg, _shift_el(gL["center"], -d_pred), self.rng)
                fr.t_ms["ringR"] = (time.perf_counter() - ta) * 1e3
            else:
                self.prior_age += 1
        fr.t_ms["ring"] = (time.perf_counter() - t0) * 1e3
        fr.ring_seen = bool(gL is not None and gL["ok"])
        return gL, gR, segL, segR

    def _ring_diag(self, fr, gL, segL):
        if gL is not None and segL is not None:
            # 诊断: 饱和像素占比 / 方格估计 / 闭运算核 —— 环找不到时看这几个就知道卡在哪一步
            fr.diag["sat_frac"] = round(float(np.count_nonzero(segL["sat"])) / segL["sat"].size, 3)
            if gL.get("sq_est"):
                fr.diag["sq_est"] = round(gL["sq_est"], 1)
            fr.diag["close_k"] = gL.get("close_k")
            if not gL["ok"]:
                fr.diag["ring_why"] = gL.get("why")
                rj = gL.get("rej")
                if rj:
                    # 和门限比: onb ≥ min_onband, cov ≥ min_coverage, hole ≤ max_hole, w_ratio ∈ [0.5, 1.6]
                    fr.diag.update(rej_onb=round(rj["onb"], 2), rej_cov=round(rj["cov"], 2),
                                   rej_hole=round(rj["hole"], 2), rej_w=round(rj["w_ratio"], 2),
                                   rej_px=round(rj["major_px"]))
                self._ring_fail = {k: v for k, v in fr.diag.items()
                                   if k in ("ring_why", "sat_frac", "sq_est", "close_k", "rej_onb", "rej_cov",
                                            "rej_hole", "rej_w", "rej_px")}
            else:
                self._ring_fail = {}

    # ---------------- 环: 3D 定位 ----------------
    def _locate_ring(self, fr, L, gL, gR, segL, segR, R_WR):
        """→ 角点平面 (dict) 或 None。结果写进 fr.ring。"""
        cfg, rig = self.cfg, self.rig
        ring = fr.ring
        plane = None
        if fr.ring_seen:
            ring.partial = gL["partial"]
            ring.side, ring.side_conf = hue_side(segL, gL)
            z_ring = rig.f * 2 * cfg.r_center / max(gL["center_major"], 1e-6)   # 长轴不受倾斜影响
            fr.diag["z_ring"] = round(float(z_ring), 3)
            consistent = False
            if gR is not None and gR["ok"]:
                d_guess = gL["center"][0][0] - gR["center"][0][0]
                sr = gL["center_major"] / gR["center_major"]
                dy = gL["center"][0][1] - gR["center"][0][1]
                consistent = d_guess > 0 and abs(sr - 1) < (0.25 if (gL["partial"] or gR["partial"]) else 0.15) \
                    and (gL["partial"] or gR["partial"] or abs(dy) < 4 + 0.03 * gL["center_major"])
            fr.diag["lr_ok"] = bool(consistent)
            t1 = time.perf_counter()
            weak_plane = None                           # 不够当 stereo 用、但能给单目法线定向的平面
            if consistent and z_ring > max(cfg.corners_max_dist, cfg.normal_max_dist):
                fr.diag["corners_skip"] = True          # 太远: 平面连定向都不够, 不算
            elif consistent:
                cL, cR = find_corners(segL, gL, cfg), find_corners(segR, gR, cfg)
                P3, pixm = match_corners(cL, cR, d_guess, rig, cfg)
                ring.n_corners = len(P3)
                fr.dbg.update(cornersL=cL, matched=pixm)
                if len(P3) >= cfg.min_plane_corners:
                    zmed = float(np.median(P3[:, 2]))
                    thr = max(0.01, 3 * zmed ** 2 / (rig.f * rig.B) * cfg.sigma_disp)
                    pl = fit_plane(P3, thr=thr, rng=self.rng)
                    if pl is not None:
                        fr.diag["plane_rms"] = round(float(pl["rms"]), 4)    # 不管过没过门限都记, 用来调门限
                        fr.diag["plane_n"] = int(pl["n_pts"])
                        if pl["rms"] <= cfg.max_plane_rms and z_ring <= cfg.corners_max_dist:
                            plane = pl
                        elif pl["rms"] <= max(cfg.max_plane_rms,
                                              cfg.plane_rms_k * zmed ** 2 / (rig.f * rig.B) * cfg.sigma_disp):
                            weak_plane = pl
            fr.diag["corners"] = int(ring.n_corners)
            fr.t_ms["corners"] = (time.perf_counter() - t1) * 1e3
            if plane is not None:
                rc = ring_center_in_plane(gL["centerline_px"], plane, rig, cfg.r_center)
                if rc is not None:
                    Z = float(rc["c"][2])
                    ring.ok, ring.src, ring.c, ring.n, ring.plane = True, "stereo", rc["c"], plane["n"], plane
                    ring.plane_rms = float(plane["rms"])
                    ring.n_src = "plane"
                    ring.r_free_ratio = rc["r_free"] / cfg.r_center
                    ring.sigma_lat = max(0.005, Z * cfg.sigma_px / rig.f * 2)
                    ring.sigma_z = max(0.005, plane["rms"] / np.sqrt(plane["n_pts"]) + 0.003 * Z)
            # 单目位姿。2026-10-21: 以前 partial (外圈 1.6m 椭圆有一点出画) 一律不算 → 地面/低处看高处的环、
            #   或者 >3.5m 环在画面边上时, 视频里有环、却一个观测都不出 (阶段 far, 距离 None)。
            #   现在看中心线椭圆: 角度覆盖够、大部分在画面里就算, σ 放大 mono_partial_sigma_k 倍。
            mono_ok, mono_k = not gL["partial"], 1.0
            if not ring.ok and gL["partial"]:
                Hh_, Ww_ = L.shape[:2]
                ex_, ey_ = _el_points(gL["center"], 90)
                in_frac = float(((ex_ >= 0) & (ex_ < Ww_) & (ey_ >= 0) & (ey_ < Hh_)).mean())
                fr.diag["in_frac"] = round(in_frac, 2)
                if gL.get("coverage", 0.0) >= cfg.mono_partial_min_cov and in_frac >= cfg.mono_partial_min_in:
                    mono_ok, mono_k = True, cfg.mono_partial_sigma_k
                else:
                    fr.diag["ring_loc"] = "partial"      # 看到环但没定位: 出画太多 / 覆盖不够
            if not ring.ok and mono_ok:
                sols = circle_pose_from_ellipse(gL["center"], rig.K, cfg.r_center)
                if not sols:
                    fr.diag["ring_loc"] = "pose_fail"
                if sols:
                    ref = None
                    if self.est.ring_n is not None and R_WR is not None:
                        ref = R_WR.T @ self.est.ring_n
                    disp_n = None
                    if consistent and not gR["partial"] and z_ring <= cfg.eldisp_max_dist:
                        disp_n = ellipse_disparity_normal(gL["center"], gR["center"], rig)
                    j, n_, nsrc, sep = pick_ring_normal(sols, weak_plane["n"] if weak_plane is not None else None,
                                                        ref, cfg, disp_n)
                    c_ = sols[j][0]
                    Z = float(c_[2])
                    ring.ok, ring.src, ring.c, ring.n = True, "mono", c_, n_
                    ring.n_src = nsrc
                    ring.ambiguous = nsrc == "mono_amb"
                    if ring.ambiguous:
                        ring.n_alt = sols[1 - j][1]
                        ring.c_alt = sols[1 - j][0]
                    if weak_plane is not None:
                        ring.plane_rms = float(weak_plane["rms"])
                    fr.diag["n_sep"] = round(sep, 1)      # mono_plane/disp/est: 两解离参考的夹角差; mid/amb: 两解夹角
                    ring.sigma_lat = max(0.01, Z * 2 / rig.f) * mono_k
                    ring.sigma_z = max(0.03, 0.03 * Z) * mono_k
                    if mono_k > 1:
                        fr.diag["ring_loc"] = "mono_partial"
        if ring.ok:
            fr.diag["n_src"] = ring.n_src
            fr.diag["tilt"] = round(ring_tilt_deg(ring.c, ring.n), 1)
        fr.dbg.update(gL=gL, gR=gR)
        return plane

    # ---------------- 红点 ----------------
    def _find_dots(self, fr, L, R, gL, gR, roi, z_pred, plane):
        cfg, rig = self.cfg, self.rig
        ring = fr.ring
        # ---------- 红点 ----------
        t2 = time.perf_counter()
        pred = gL["outer_major"] * cfg.dot_d / cfg.outer_d if fr.ring_seen else None
        roiR = None
        ring_roi = False
        if roi is None and cfg.dot_in_ring and fr.ring_seen:
            # 纸贴在环中间: 红点一定在内圈里 → 只在内圈外接框里找; 右目按环的视差平移
            (icx, icy), (iw, ih), _ = gL["inner"]
            hw = 0.5 * max(iw, ih) + 4
            roi = (icx - hw, icy - hw, icx + hw, icy + hw)
            if gR is not None and gR["ok"]:
                dpx = gL["center"][0][0] - gR["center"][0][0]
            else:
                dpx = rig.B * gL["center_major"] / (2 * cfg.r_center)
            mg = 0.3 * abs(dpx) + 10
            roiR = (roi[0] - dpx - mg, roi[1], roi[2] - dpx + mg, roi[3])
            ring_roi = True
            fr.dot_roi = roi
        elif roi is not None and z_pred:
            dpx = rig.f * rig.B / z_pred
            roiR = (roi[0] - 1.5 * dpx - 10, roi[1], roi[2] + 10, roi[3])
        dL = find_dot(L, cfg, pred, roi)
        dR = find_dot(R, cfg, pred, roiR)
        if roi is not None and not ring_roi and not dL["cands"]:  # 末段 ROI 里没找到 → 全图再找一次
            dL, dR = find_dot(L, cfg, pred), find_dot(R, cfg, pred)
            fr.dot_roi = None
        elif ring_roi and not dL["cands"] and gL["partial"]:
            # 2026-10-21: 环部分出画时内圈框可能不准 (或红点所在的那块正好出了框) → 全图再找一次。
            #   环完整时不兜底 (远处红点本来就小, 每帧全图找太贵, 也容易把环外红色当红点)
            dL, dR = find_dot(L, cfg, pred), find_dot(R, cfg, pred)
            fr.dot_roi = None
        fr.dbg.update(dotL=dL, dotR=dR)
        best = None
        pair_why = None                     # 诊断: 双目没配上时, 最接近的一对卡在哪
        for a in dL["cands"]:
            for b in dR["cands"]:
                d = a["c"][0] - b["c"][0]
                if d <= 0:
                    pair_why = pair_why or ("pair_disp", round(float(d), 1))
                    continue
                if abs(a["c"][1] - b["c"][1]) > max(2.0, 0.15 * a["diam"]):
                    pair_why = ("pair_dy", round(float(a["c"][1] - b["c"][1]), 1))
                    continue
                if abs(a["diam"] / b["diam"] - 1) > 0.25:
                    pair_why = ("pair_size", round(float(a["diam"] / b["diam"]), 2))
                    continue
                Z = rig.f * rig.B / d
                err = abs(a["diam"] * Z / rig.f / cfg.dot_d - 1)
                if err <= cfg.dot_metric_tol and (best is None or err < best[0]):
                    best = (err, a, b, Z)
                elif best is None:
                    pair_why = ("metric", round(float(a["diam"] * Z / rig.f / cfg.dot_d - 1), 2))
        # 2026-10-23 诊断: 红点为什么没出双目观测 (diag.dot_why + 关键数) —— 实拍 3.5~4m 外一直没红点,
        #   合成图 (含模糊/两次 JPEG) 7m 都能出, 要看真实画面卡在哪一道
        if not best:
            if not dL["blobs"]:
                fr.diag["dot_why"] = "no_red"            # 红色掩码里没有够大的块 (red_thr / dot_min_px / ROI)
            elif not dL["cands"]:
                bl = max(dL["blobs"], key=lambda x: x["diam"])
                fr.diag["dot_why"] = "shape" if not bl["shape_ok"] else "surround"
                fr.diag.update(dot_d=round(bl["diam"], 1), dot_circ=round(bl["circ"], 2),
                               dot_fill=round(bl["fill"], 2), dot_touch=bool(bl["touch"]))
                if bl["surr_S"] == bl["surr_S"]:
                    fr.diag.update(dot_sS=round(bl["surr_S"]), dot_sV=round(bl["surr_V"]))
            elif not dR["cands"]:
                fr.diag["dot_why"] = "no_right"          # 左目有、右目没有 (右目 ROI / 曝光)
                fr.diag["dot_d"] = round(max(x["diam"] for x in dL["cands"]), 1)
            elif pair_why:
                fr.diag["dot_why"], fr.diag["dot_val"] = pair_why
            fr.diag["dot_nb"] = len(dL["blobs"])
        if best:
            _, a, b, Z = best
            u = _ray(rig, *a["c"])
            sz = max(0.002, Z ** 2 / (rig.f * rig.B) * cfg.sigma_disp)
            sl = max(0.002, Z * cfg.sigma_px / rig.f)
            fr.dots.append(DotObs(True, "stereo", u * Z, a["c"], a["diam"], a["diam"] * Z / rig.f, sl, sz))
        # 左目单独的最佳候选（双目没配上时用）
        mono_cand = None
        if best:
            mono_cand = best[1]
        elif pred is not None:
            mono_cand = dL["best"]  # 已按环预测的直径筛过
        elif z_pred and dL["cands"]:  # 没有环：按估计器预测的距离挑尺寸最像的，差太多就不要
            pd = rig.f * cfg.dot_d / z_pred
            c_ = min(dL["cands"], key=lambda bl: abs(bl["diam"] / pd - 1))
            if abs(c_["diam"] / pd - 1) < 0.4:
                mono_cand = c_
        # 都没有（无位姿、无环、双目又没配上）：不给单目观测，避免把胶带之类的当成红点
        if mono_cand is not None and plane is not None:
            u = _ray(rig, *mono_cand["c"])
            den = u @ plane["n"]
            if abs(den) > 1e-6:
                s = (plane["mu"] @ plane["n"]) / den
                if s > 0:
                    Z = s * u[2]
                    o = DotObs(True, "ray_plane", u * s, mono_cand["c"], mono_cand["diam"],
                               mono_cand["diam"] * Z / rig.f, max(0.002, Z * cfg.sigma_px / rig.f),
                               max(0.005, ring.sigma_z * 1.5))
                    if best:
                        o.rp_delta = float(best[3] - Z)
                        fr.dots[0].rp_delta = o.rp_delta
                    fr.dots.append(o)
        if mono_cand is not None and not best and plane is None:
            u = _ray(rig, *mono_cand["c"])
            fr.dots.append(DotObs(True, "mono", u / np.linalg.norm(u), mono_cand["c"], mono_cand["diam"]))
        fr.t_ms["dot"] = (time.perf_counter() - t2) * 1e3

    def _remember_ref(self, fr, t):
        """记下这一帧红点 (或环心) 的像素 / 深度 / 距离 —— 没位姿时下一帧 _plan 用。"""
        rig = self.rig
        ring = fr.ring
        d0 = fr.dot("stereo") or fr.dot("ray_plane")
        if d0 is not None:
            self.last_dot_pix, self.last_dot_z = d0.pix, float(d0.p[2])
            self.last_dist = float(np.linalg.norm(d0.p))
            self.last_ref_t = t
        elif ring.ok:
            # 只有环: 距离/像素都按环心 (红点贴在环中间), 不再混用很久以前的红点像素
            c_ = ring.c
            self.last_dot_pix = (rig.f * c_[0] / c_[2] + rig.cx, rig.f * c_[1] / c_[2] + rig.cy)
            self.last_dot_z = float(c_[2])
            self.last_dist = float(np.linalg.norm(c_))
            self.last_ref_t = t
