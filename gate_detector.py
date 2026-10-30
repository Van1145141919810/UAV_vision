"""
gate_detector.py — 格子环 + 红点 识别（正式版）

任务: 识别比赛环（外径 1.6m / 内径 1.2m，左红格右蓝格，上下渐变色带）及其中间纸上的红点(11cm)，
      输出红点在世界系下的位置，供最终冲撞。不依赖 ROS；双目校正复用 ring_detector.StereoRig。

整体思路
--------
红点自始至终是最终目标；环是"远处把它找到 + 中距离把它精确定位"的工具。
不按距离硬切逻辑，而是维护一个"红点世界坐标"估计器，每帧有什么观测就用什么，按精度加权：

  观测                 何时可用                         精度（0.5 倍率, f≈823, B=60mm）
  ring_prior          看得到环、还没锚定偏移              环心 + 大不确定度(0.3m)，远处给个方向
  ring_offset         看得到环、已学到"红点-环心"偏移     环心精度 + 偏移精度
  dot_ray_plane       角点平面 + 左目看到红点             深度来自平面，中距离最准
  dot_stereo          左右目都看到红点（物理直径≈11cm）    σZ = Z²/(fB)·σd，近处/末段主力
  dot_mono_plane      只有一只眼看到红点 + 已存环平面      末段兜底

阶段（由"本帧有哪些观测"决定，只用于显示/调度计算量）:
  search  什么都没有
  far     看到环，但没有能当 stereo 用的角点平面（环距离 > corners_max_dist 或残差 > max_plane_rms）。
          这时环法线来自单目椭圆 (两个解), 用远处的噪声角点平面 / 左右目椭圆视差 / 估计器给它定向,
          见 pick_ring_normal 和 diag.n_src (2026-10-15: 以前远处法线根本不出, 也不画)
  mid     角点平面 + 环完整                      ← 锚定红点、学偏移、存环平面
  near    角点平面/环还在，但环部分出画
  terminal 环已不找（太近）或看不到，只靠红点
  coast   本帧没观测，沿用估计（未超时）
  blind   估计距离 < blind_dist（最后 0.2–0.3m，红点占满画面/失焦）
  lost    超时

坐标系
------
  rect : 校正后左目（检测都在这里做）      cam : 原始左目相机系，P_cam = R1ᵀ · P_rect
  world: 你给的世界系（VIO）。process() 传入 T_WC（原始左目相机 → 世界，4x4）。
  不传 T_WC 时估计器不更新，只输出本帧相机系结果（离线看录像用）。

两种部署（2026-10-15）
---------------------
  机上一体   vision_bridge _mode:=gate      GateDetector.process(L, R, t, T_WC) 一步到位
  地面检测   vision_bridge _mode:=gate_gcs  地面 gcs_gate.py: process(L, R, t_cap, None) → frame_to_msg
             + gcs_gate.py                  飞机:            frame_from_msg → GateFuser.update(fr, T_WC(t_cap), t_cap)
             检测只出相机(rect)系观测; 位姿插值和估计器都在飞机上 (位姿不过网络, 断链不丢估计器状态)。

2026-10-21 修 "视频里有环, 却显示没检测到"
------------------------------------------
  1. 地面检测: _plan 的"上次红点距离"永不过期 → 撞纸后 (<1m) 红点没了就永远 skip_ring 不找环。
     现在超过 last_dot_max_age (0.5s) 就不用。
  2. 外圈椭圆有一点出画 (partial) 以前一律不算单目位姿 → 地面/低处看环、环在画面边上时只有 ring_seen 没观测。
     现在中心线椭圆覆盖 ≥ mono_partial_min_cov 且在画面内 ≥ mono_partial_min_in 就算, σ×mono_partial_sigma_k。
     没定位时 diag.ring_loc = partial / pose_fail, 定位了是 mono_partial。
  3. 估计器: 有过一次红点观测后, 只看到环的帧被整帧忽略 → 目标老化 → 桥重置。
     现在红点 ring_takeover_s (0.5s) 没更新就让环接管 (ring_prior); 等待中的帧 diag.ring_wait。
  4. 弱观测 (ring_prior / ring_offset / dot_mono_plane) 的误差是系统性的, 以前多帧平均到 σ≈3.5cm (假精度)。
     现在更新后 P ≥ 本帧观测协方差 (+ ring_prior_floor²)。
  5. 单目定过向的法线 (mono_plane/disp/est) 以前算了却没进估计器 → 远处 ring_n 一直空。现在进 ring_nm
     (最近 15 帧一致才用); dot_mono_plane 仍只用双目平面。
  6. 环心权重按时间衰减 + 上限 (以前只增不减, VINS 漂 >0.3m 后环心冻住)。
  7. 同一帧 dot_stereo + dot_ray_plane 只融合协方差小的一个 (同一像素, 以前横向 σ 低估 √2)。
  8. 缩小图 → 工作图坐标按像素中心换算 (以前环心固定偏左上 ~0.93px)。
  9. 环部分出画、内圈框里没找到红点 → 全图再找一次。
 10. 单目两解的另一个环心也发出去 (msg ring.c2), 飞机侧换法线时环心一起换; GateFuser.reset() 整个清零。
  消息格式向下兼容: 老飞机端读新消息只是忽略 c2 / 新 diag。

★ 标定单位: stereo_calib.yaml 的 T 是 mm。ring_detector.load_stereo_calib 现在会自动换成米
  (旧版直接当米 → 基线 60m → 双目深度大 1000 倍 → 角点平面 / 红点双目从来没成立过)。

接入示例（vision_bridge 里）
-------------------------
    import ring_detector as rd, gate_detector as gd
    rig = rd.StereoRig.from_file("stereo_calib.yaml", scale=0.5)
    det = gd.GateDetector(rig, gd.GateConfig.load("gate_params.json"))
    ...
    L, R = rig.rectify(*rig.split(sbs_frame))
    T_WC = T_WB_at(t_cap) @ T_BC            # 你的 VIO 位姿(曝光时刻) × 相机外参
    f = det.process(L, R, t_cap, T_WC)
    if f.target.valid: 目标 = f.target.pos_w, 协方差 f.target.cov_w, 来源 f.target.src, 阶段 f.stage
"""
from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import asdict, dataclass, field, fields
from typing import List, Optional

import cv2
import numpy as np

import ring_detector as rd


# =============================================================================
# 配置（字段名与 gate_tune.py 的 gate_params.json 兼容，可直接 load）
# =============================================================================
@dataclass
class GateConfig:
    # 物理尺寸 (m)
    outer_d: float = 1.6
    inner_d: float = 1.2
    square: float = 0.05
    dot_d: float = 0.11
    # 白平衡增益 (B, G, R)
    wb: list = field(default_factory=lambda: [1.0, 1.0, 1.0])
    # 饱和度分割
    s_min: int = 80
    v_min: int = 90                 # 2026-10-15 实拍: 60 → 90 (窗外夜景树 V≈80 被当成彩色)
    c_min: int = 50                 # 彩度 max(R,G,B)-min(R,G,B) 下限; 0 = 不用 (见 segment)。
                                    #   实拍 10 帧×双目: 旧参数环拟错/被拒 6 处, 新参数 20/20;
                                    #   画面整体压暗到 0.6 倍仍有 15/20 (只调 v_min=120 只剩 2/20)
    # 找环
    find_scale: float = 0.35
    close_k: int = 5
    ransac_iters: int = 300
    min_center_px: int = 30
    min_onband: float = 0.55
    min_coverage: float = 0.15
    max_hole: float = 0.25
    prior_max_age: int = 5
    # 红点
    red_thr: int = 40
    dot_circ_min: float = 0.80
    dot_fill_min: float = 0.85
    dot_size_tol: float = 0.35
    dot_metric_tol: float = 0.30
    dot_min_px: float = 6.0
    surround_v_min: int = 100
    # 角点
    gray_blur: float = 1.0
    corner_quality: float = 0.05
    corner_block: int = 5
    x_r_factor: float = 0.35
    mind_factor: float = 0.6
    min_square_px: float = 4.0
    v_dark: int = 60
    y_tol: float = 1.0
    # ---- 以下为正式版新增 ----
    min_plane_corners: int = 12     # 角点平面最少匹配点
    max_plane_rms: float = 0.03     # 角点平面残差上限 (m)
    sigma_px: float = 0.5           # 像素噪声（横向）
    sigma_disp: float = 0.15        # 视差噪声 (px)
    q_pos: float = 0.02             # 目标随机游走 m/√s（纸会被气流吹动）。2026-10-19: 0.05 → 0.02
                                    #   (环固定在架子上; 0.05 时只看到环的 σ 停在 6~7cm, 1s 没观测 σ 涨 5cm)
    gate_chi2: float = 16.0         # 观测检验阈值（3自由度 χ²，≈99.9%）
    reinit_after: int = 2           # 强观测连续 N 次被拒且彼此一致 → 认为目标移动了，重置
    ring_skip_dist: float = 1.0     # 估计距离 < 此值时不再找环（只找红点）
    dot_roi_scale: float = 3.0      # 末段红点 ROI 半宽 = 倍数 × 预测直径
    blind_dist: float = 0.3
    lost_timeout: float = 0.5       # s
    side_lock_votes: int = 5
    world_up: list = field(default_factory=lambda: [0.0, 0.0, 1.0])
    # ---- 2026-10-15 提速 ----
    corners_max_dist: float = 3.5   # 椭圆估的环距离 > 此值就不算角点平面 (远处平面残差必然超 max_plane_rms,
                                    #   算了也白算)。合成图(含真实校正/JPEG)实测: 1.5m rms 9mm ✓, 3m 36mm ✗, 5m 93mm ✗。
                                    #   现场按 /gate/status 的 diag.z_ring / diag.plane_rms 再调
    search_every: int = 3           # 没有先验 (环丢了) 时, 每 N 帧才做一次全图 RANSAC 全搜; 1 = 每帧都搜
    max_corners: int = 1200         # goodFeaturesToTrack 上限 (色带一圈 X 角点只有几百个)
    dot_in_ring: bool = True        # 看得到环时, 红点只在内圈椭圆的外接框里找 (中距离提速 + 去掉环外红色误检)
    # ---- 2026-10-15 法线 ----
    # 单目椭圆给的法线有两个解, 对的那个远处也只差 1~3°, 但光靠椭圆分不出是哪个。
    # 角点平面在远处残差大 (位置不能用), 法线却仍有几度精度 —— 足够在两解 (相差 30~70°) 里挑对的。
    normal_max_dist: float = 11.0   # 环距离 ≤ 此值都算角点, 平面不够准当 stereo 用时, 拿来给单目法线定向
                                    #   (远处色带小, 角点只剩几十个, 多花 ~10ms)
    plane_rms_k: float = 2.5        # 定向用平面的残差上限 = max(max_plane_rms, k·σZ),  σZ = Z²/(fB)·sigma_disp
    normal_sep_deg: float = 10.0    # 参考法线离两解的夹角要差这么多才算分清
    normal_ref_max_deg: float = 20.0  # 而且离选中的解不能超过这么多 (否则参考法线本身是坏的, 比如角点太少的平面)
    eldisp_max_dist: float = 6.0    # 环距离 ≤ 此值时, 用"左右目椭圆的视差随位置的变化"再算一个参考法线 (不要角点;
                                    #   合成图 ≤6m 误差 1~10°, 再远就不可靠)
    normal_mid_deg: float = 35.0    # 两解夹角 ≤ 此值 = 接近正对环 / 扁率被噪声主导: 取两解平分线, 只显示不进估计器。
                                    #   实拍 8~11m 的环扁率 0.97 左右、长轴方向乱跳 → 两解夹角 23~34° 全是噪声
                                    #   (环本身不圆几个 % 也会这样); 合成图真倾斜 20° 时两解夹角 35~40°
    # ---- 2026-10-21 修 "看得到环却不出观测 / 估计器不用环" ----
    last_dot_max_age: float = 0.5   # s。没位姿 (地面检测) 时 _plan 用"上次红点/环的距离"决定要不要找环;
                                    #   以前它永不过期 → 撞完纸 (最后 <1m) 红点没了就**永远不找环**, 重启才好
    mono_partial_min_cov: float = 0.5   # 环部分出画时, 中心线椭圆的角度覆盖 ≥ 此值才算单目位姿 (以前 partial 一律不算)
    mono_partial_min_in: float = 0.6    # 且中心线椭圆至少这么多在画面里
    mono_partial_sigma_k: float = 2.0   # partial 单目位姿的 σ 放大倍数
    ring_takeover_s: float = 0.5    # 红点 (dot_*) 这么久没更新 → 只看到环的帧重新可以更新估计 (ring_prior)。
                                    #   以前一旦有过红点观测, 只看到环的帧被**整帧忽略**, 目标老化 → 桥重置 → 没目标
    ring_prior_floor: float = 0.05  # m。弱观测 (ring_prior / ring_offset / dot_mono_plane) 的误差是系统性的,
                                    #   多帧平均不掉: 更新后 P 不低于"本帧观测协方差 (+ 此值²)"。以前 σ 会被平均到 ~3.5cm
    ring_c_halflife: float = 3.0    # s。存下的环心权重按时间衰减 (以前只增不减 → VINS 漂移 >0.3m 后环心冻住)
    ring_c_w_max: float = 1e5       # 环心权重上限 (≈ σ 3mm)
    mono_n_min_cnt: int = 5         # 只有单目定过向的法线时, 至少这么多帧 ...
    mono_n_min_rbar: float = 0.996  # ... 且彼此一致 (平均合向量长度, cos5°) 才给出 ring_n

    @property
    def r_center(self):
        return (self.outer_d + self.inner_d) / 4

    @property
    def hw_ratio(self):
        return (self.outer_d - self.inner_d) / (self.outer_d + self.inner_d)

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2, ensure_ascii=False)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})


# =============================================================================
# 单目：分割 / 找环 / 正反面 / 角点 / 红点   （与 gate_tune.py v2 相同的算法）
# =============================================================================
def apply_wb(img, cfg: GateConfig):
    g = np.asarray(cfg.wb, np.float32)
    if np.allclose(g, 1.0):
        return img
    return cv2.convertScaleAbs(img.astype(np.float32) * g[None, None, :])


def segment(img, cfg: GateConfig):
    """
    "彩色像素"掩码。只看 S (饱和度) 会把**暗处的杂色**也算进去 —— 夜里窗外的树 S≈90 但 V≈80,
    2026-10-15 实拍: 它和环粘成一片, 椭圆拟错/被 hole 判据拒掉。彩度 C = S·V/255 = max-min
    是"绝对鲜艳程度", 环 C 中位数 ~110, 窗外树 ~30, 用它分得开。
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    m = (S > cfg.s_min) & (V > cfg.v_min)
    if cfg.c_min > 0:
        b, g, r = cv2.split(img)
        C = cv2.subtract(cv2.max(cv2.max(b, g), r), cv2.min(cv2.min(b, g), r))
        m &= C > cfg.c_min
    sat = m.astype(np.uint8) * 255
    return {"H": H, "S": S, "V": V, "sat": sat, "gray": cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)}


def _scale_el(el, s, about_center=False):
    (cx, cy), (w, h), a = el
    if about_center:
        return ((cx, cy), (w * s, h * s), a)
    return ((cx * s, cy * s), (w * s, h * s), a)


def _el_small_to_full(el, fs):
    """缩小图 (find_scale) 的椭圆 → 工作图。像素中心对齐: x_full = (x_small + 0.5)/fs − 0.5。
    2026-10-21: 以前直接 x/fs, 环心固定往左上偏 (0.5/fs − 0.5) ≈ 0.93px (fs=0.35)。"""
    (cx, cy), (w, h), a = el
    return (((cx + 0.5) / fs - 0.5, (cy + 0.5) / fs - 0.5), (w / fs, h / fs), a)


def _el_full_to_small(el, fs):
    (cx, cy), (w, h), a = el
    return (((cx + 0.5) * fs - 0.5, (cy + 0.5) * fs - 0.5), (w * fs, h * fs), a)


def _shift_el(el, dx, dy=0.0):
    (cx, cy), (w, h), a = el
    return ((cx + dx, cy + dy), (w, h), a)


_TRIG = {}


def _el_points(el, n=90):
    (cx, cy), (w, h), ang = el
    if n not in _TRIG:
        t = np.linspace(0, 2 * np.pi, n, endpoint=False)
        _TRIG[n] = (np.cos(t), np.sin(t))
    ct, st = _TRIG[n]
    c, s = np.cos(np.radians(ang)), np.sin(np.radians(ang))
    return cx + w / 2 * ct * c - h / 2 * st * s, cy + w / 2 * ct * s + h / 2 * st * c


def _estimate_square(small):
    n, _, st, _ = cv2.connectedComponentsWithStats(small, connectivity=4)
    if n < 13:
        return None
    w, h, a = st[1:, 2].astype(float), st[1:, 3].astype(float), st[1:, 4].astype(float)
    sel = (a >= 4) & (w >= 2) & (h >= 2) & (np.maximum(w, h) <= 1.8 * np.minimum(w, h)) & (a >= 0.4 * w * h)
    if sel.sum() < 10:
        return None
    s = np.sort(np.sqrt(a[sel]))
    lo = np.searchsorted(s, s * 0.8, "left")
    hi = np.searchsorted(s, s * 1.25, "right")
    cnt = hi - lo
    i = int(np.argmax(cnt * s))
    return float(np.median(s[lo[i]:hi[i]])) if cnt[i] >= 10 else None


def _inliers(el, pts, w, cfg, shape):
    (cx, cy), (W, H), ang = el
    if not np.all(np.isfinite([cx, cy, W, H, ang])):
        return None
    a, b = W / 2.0, H / 2.0
    sh, sw = shape
    if min(a, b) < 3 or min(a, b) / max(a, b) < 0.25 or max(a, b) > 4 * max(sh, sw):
        return None
    t = np.radians(ang)
    c, s = np.cos(t), np.sin(t)
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    xp, yp = dx * c + dy * s, -dx * s + dy * c
    rn = np.sqrt((xp / a) ** 2 + (yp / b) ** 2)
    rad = np.sqrt(a * b)
    hw = cfg.hw_ratio * rad
    inl = (np.abs(rn - 1) * rad < max(1.5, 0.3 * hw)) & (w > 0.5 * hw) & (w < 1.6 * hw)
    return inl, xp, yp, a, b, hw


def _eval_hyp(el, pts, w, closed, cfg, quick=None):
    q = quick or _inliers(el, pts, w, cfg, closed.shape)
    if q is None:
        return None
    inl, xp, yp, a, b, hw = q
    n = int(inl.sum())
    if n < 10:
        return None
    sh, sw = closed.shape
    (cx, cy), (W, H), ang = el
    th = np.arctan2(yp[inl] / b, xp[inl] / a)
    cov = float((np.bincount(np.clip(((th + np.pi) / (2 * np.pi) * 36).astype(int), 0, 35), minlength=36) > 0).mean())
    ex, ey = _el_points(el, 120)
    inimg = (ex >= 0) & (ex < sw) & (ey >= 0) & (ey < sh)
    if inimg.sum() < 12:
        return None
    onb = float((closed[ey[inimg].astype(int), ex[inimg].astype(int)] > 0).mean())
    w_ratio = float(np.median(w[inl]) / hw)
    hv = []
    for k in (0.55, 0.7):
        hx, hy = _el_points(((cx, cy), (W * k, H * k), ang), 90)
        ok = (hx >= 0) & (hx < sw) & (hy >= 0) & (hy < sh)
        if ok.sum() >= 8:
            hv.append(closed[hy[ok].astype(int), hx[ok].astype(int)] > 0)
    hole = float(np.concatenate(hv).mean()) if hv else 0.0
    score = n * onb * np.exp(-((w_ratio - 1) / 0.3) ** 2) * np.sqrt(cov) * max(0.0, 1 - 2 * hole)
    return {"el": el, "n": n, "cov": cov, "onb": onb, "inl": inl, "w_ratio": w_ratio, "hole": hole,
            "score": float(score)}


def _refine(h, pts, w, closed, cfg, iters=3):
    for _ in range(iters):
        if h is None or h["n"] < 6:
            return h
        try:
            el = cv2.fitEllipse(pts[h["inl"]].astype(np.float32))
        except cv2.error:
            return h
        h2 = _eval_hyp(el, pts, w, closed, cfg)
        if h2 is None or h2["score"] < h["score"] * 0.98:
            return h
        h = h2
    return h


def _accept(h, cfg):
    return h is not None and h["onb"] >= cfg.min_onband and h["cov"] >= cfg.min_coverage and h["hole"] <= cfg.max_hole


def find_gate(seg, cfg: GateConfig, prior=None, rng=None):
    """色带中心线(距离变换脊线) + RANSAC 椭圆。返回 dict(ok, center/outer/inner 椭圆, band_roi, ...)。"""
    rng = rng or np.random.default_rng(0)
    fs = cfg.find_scale
    small = cv2.resize(seg["sat"], None, fx=fs, fy=fs, interpolation=cv2.INTER_AREA)
    small = (small > 64).astype(np.uint8) * 255
    sq = _estimate_square(small)
    if prior is not None:
        sq = max(sq or 0.0, max(prior[1]) * fs * ((cfg.outer_d / 2) / cfg.r_center) * cfg.square / cfg.outer_d)
    base = int(cfg.close_k) | 1
    ks = [min(61, max(base, int(round(1.6 * sq))) | 1)] if sq else sorted({base, 9, 15, 23})
    best = None
    rej = None                                   # 诊断: 所有核里得分最高的"被拒"假设
    for k in ks:
        g = _find_gate_k(seg, small, k, cfg, prior, rng)
        if g.get("rej") and (rej is None or g["rej"]["score"] > rej["score"]):
            rej = dict(g["rej"], k=k)
        if best is None or (g["ok"] and (not best["ok"] or g["score"] > best["score"])):
            best = g
        if g["ok"] and g["coverage"] >= 0.6 and 0.8 <= g["width_ratio"] <= 1.25 and g["hole"] <= 0.1:
            break
    best["tried_k"] = list(ks)
    best["sq_est"] = float(sq) if sq else None
    if not best["ok"] and rej is not None:
        best["rej"] = rej
    return best


def _find_gate_k(seg, small, k, cfg, prior, rng):
    Hh, Ww = seg["sat"].shape
    fs = cfg.find_scale
    closed = cv2.morphologyEx(small, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    out = {"ok": False, "why": "", "prior_used": False, "close_k": k, "score": 0.0}
    dt = cv2.distanceTransform(closed, cv2.DIST_L2, 3)
    ridge = (dt >= cv2.dilate(dt, np.ones((3, 3), np.uint8))) & (dt >= 1.0)
    ys, xs = np.nonzero(ridge)
    if len(xs) < 15:
        out["why"] = "no_ridge"
        return out
    if len(xs) > 1500:
        sel = rng.choice(len(xs), 1500, replace=False)
        xs, ys = xs[sel], ys[sel]
    pts = np.stack([xs, ys], 1).astype(np.float64)
    w = dt[ys, xs].astype(np.float64)
    # 先验够好就跳过 RANSAC（跟踪时大部分帧走这里，省一半以上时间）
    if prior is not None:
        ph = _refine(_eval_hyp(_el_full_to_small(prior, fs), pts, w, closed, cfg), pts, w, closed, cfg)
        if _accept(ph, cfg) and ph["cov"] >= 0.5 and 0.8 <= ph["w_ratio"] <= 1.25 and ph["hole"] <= 0.1:
            return _finish_gate(out, ph, pts, cfg, fs, Hh, Ww, prior_used=True)
    _, lab = cv2.connectedComponents(closed, connectivity=8)
    lab_pts = lab[ys, xs]
    comps, counts = np.unique(lab_pts, return_counts=True)
    probs = counts / counts.sum()
    by_comp = {c_: np.flatnonzero(lab_pts == c_) for c_ in comps}
    quick = []
    for it in range(cfg.ransac_iters):
        if it % 2 == 0:
            pool = by_comp[rng.choice(comps, p=probs)]
            if len(pool) < 5:
                continue
            idx = rng.choice(pool, 5, replace=False)
        else:
            idx = rng.choice(len(pts), 5, replace=False)
        try:
            el = cv2.fitEllipse(pts[idx].astype(np.float32))
        except cv2.error:
            continue
        q = _inliers(el, pts, w, cfg, closed.shape)
        if q is not None:
            nq = int(q[0].sum())
            if nq >= 10:
                quick.append((nq, el, q))
    quick.sort(key=lambda t_: -t_[0])
    hyps = [h for h in (_eval_hyp(el, pts, w, closed, cfg, quick=q) for _, el, q in quick[:30]) if h is not None]
    hyps.sort(key=lambda h: -h["score"])
    best = None
    for h in hyps[:8]:
        h = _refine(h, pts, w, closed, cfg)
        if _accept(h, cfg) and (best is None or h["score"] > best["score"]):
            best = h
    if prior is not None:
        ph = _refine(_eval_hyp(_el_full_to_small(prior, fs), pts, w, closed, cfg), pts, w, closed, cfg)
        if _accept(ph, cfg) and (best is None or ph["score"] >= 0.6 * best["score"]):
            best, out["prior_used"] = ph, True
    if best is None:
        out["why"] = "no_consensus" if hyps else "no_hypothesis"
        if hyps:
            h0 = _refine(hyps[0], pts, w, closed, cfg) or hyps[0]
            out["rej"] = {"score": float(h0["score"]), "onb": float(h0["onb"]), "cov": float(h0["cov"]),
                          "hole": float(h0["hole"]), "w_ratio": float(h0["w_ratio"]), "n": int(h0["n"]),
                          "major_px": float(max(h0["el"][1]) / fs)}
        return out
    return _finish_gate(out, best, pts, cfg, fs, Hh, Ww, out["prior_used"])


def _finish_gate(out, best, pts, cfg, fs, Hh, Ww, prior_used):
    out["prior_used"] = prior_used
    center = _el_small_to_full(best["el"], fs)
    if max(center[1]) < cfg.min_center_px:
        out["why"] = "too_small"
        return out
    ko, ki = (cfg.outer_d / 2) / cfg.r_center, (cfg.inner_d / 2) / cfg.r_center
    outer, inner = _scale_el(center, ko, True), _scale_el(center, ki, True)
    roi = np.zeros((Hh, Ww), np.uint8)
    cv2.ellipse(roi, _scale_el(outer, 1.05, True), 255, -1)
    cv2.ellipse(roi, _scale_el(inner, 0.95, True), 0, -1)
    ex, ey = _el_points(outer, 180)
    out.update(ok=True, why="ok", center=center, outer=outer, inner=inner, band_roi=roi,
               outer_major=float(max(outer[1])), center_major=float(max(center[1])),
               coverage=best["cov"], onband=best["onb"], width_ratio=best["w_ratio"], hole=best["hole"],
               score=best["score"], partial=bool(((ex < 0) | (ex >= Ww) | (ey < 0) | (ey >= Hh)).any()),
               centerline_px=(pts[best["inl"]] + 0.5) / fs - 0.5)  # 中心线内点（工作图像素）
    return out


def hue_side(seg, gate):
    H, sat = seg["H"], seg["sat"]
    (cx, cy), (w, h), _ = gate["center"]
    ys, xs = np.nonzero((gate["band_roi"] > 0) & (sat > 0))
    th = np.degrees(np.arctan2((ys - cy) / (h / 2), (xs - cx) / (w / 2)))
    L, R = np.abs(np.abs(th) - 180) <= 45, np.abs(th) <= 45
    hv = H[ys, xs]
    red, blue = (hv < 10) | (hv > 165), (hv > 95) & (hv < 130)
    f = lambda sel, c: float(c[sel].mean()) if sel.sum() >= 150 else float("nan")
    lr, lb, rr_, rb = f(L, red), f(L, blue), f(R, red), f(R, blue)

    def lab(r_, b_):
        if r_ == r_ and r_ > 0.6 and r_ > 2 * b_:
            return "red"
        if b_ == b_ and b_ > 0.6 and b_ > 2 * r_:
            return "blue"
        return None

    a, b = lab(lr, lb), lab(rr_, rb)
    if a == "red" and b == "blue":
        return "front", 1.0
    if a == "blue" and b == "red":
        return "back", 1.0
    if (a == "red" and b is None) or (b == "blue" and a is None):
        return "front", 0.5
    if (a == "blue" and b is None) or (b == "red" and a is None):
        return "back", 0.5
    return None, 0.0


def find_corners(seg, gate, cfg):
    sq = gate["outer_major"] * cfg.square / cfg.outer_d
    r = {"sq_px": float(sq), "pts": np.zeros((0, 2)), "xok": np.zeros(0, bool), "pol": np.zeros(0, np.int8)}
    if sq < cfg.min_square_px:
        return r
    # ★2026-10-15: 先裁到色带外接框。goodFeaturesToTrack 的特征值图是在**整幅图**上算的,
    #   mask 只在最后筛点 —— 不裁的话环再小也是全图开销。
    bx, by, bw, bh = cv2.boundingRect(gate["band_roi"])
    if bw < 8 or bh < 8:
        return r
    gray = seg["gray"][by:by + bh, bx:bx + bw]
    g = cv2.GaussianBlur(gray, (0, 0), cfg.gray_blur) if cfg.gray_blur > 0 else gray
    pts = cv2.goodFeaturesToTrack(g, maxCorners=int(cfg.max_corners), qualityLevel=max(cfg.corner_quality, 1e-3),
                                  minDistance=max(3.0, cfg.mind_factor * sq), mask=gate["band_roi"][by:by + bh, bx:bx + bw],
                                  blockSize=max(3, int(cfg.corner_block) | 1))
    if pts is None:
        return r
    win = max(2, int(round(0.35 * sq)))
    pts = cv2.cornerSubPix(g, pts.astype(np.float32), (win, win), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)).reshape(-1, 2)
    pts = pts + np.array([bx, by], np.float32)          # 回到工作图坐标
    sat = seg["sat"]
    Hh, Ww = sat.shape
    rad = max(3.0, cfg.x_r_factor * sq)
    t = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    xs = np.round(pts[:, 0:1] + rad * np.cos(t)[None]).astype(int)
    ys = np.round(pts[:, 1:2] + rad * np.sin(t)[None]).astype(int)
    inb = (xs >= 0).all(1) & (xs < Ww).all(1) & (ys >= 0).all(1) & (ys < Hh).all(1)
    s = sat[np.clip(ys, 0, Hh - 1), np.clip(xs, 0, Ww - 1)] > 0
    trans = (s != np.roll(s, 1, axis=1)).sum(1)
    q = s[:, [6, 18, 30, 42]]
    xok = inb & (trans == 4) & (q[:, 0] == q[:, 2]) & (q[:, 1] == q[:, 3]) & (q[:, 0] != q[:, 1])
    r.update(pts=pts, xok=xok, pol=q[:, 0].astype(np.int8))
    return r


def find_dot(img, cfg: GateConfig, pred_diam=None, roi=None):
    """白底包围的红色圆斑。roi=(x0,y0,x1,y1) 时只在 ROI 内找（末段省算力）。"""
    Hh, Ww = img.shape[:2]
    x0, y0, x1, y1 = (0, 0, Ww, Hh) if roi is None else (max(0, int(roi[0])), max(0, int(roi[1])),
                                                        min(Ww, int(roi[2])), min(Hh, int(roi[3])))
    out = {"cands": [], "blobs": [], "best": None, "pred_diam": pred_diam, "roi": (x0, y0, x1, y1)}
    if x1 - x0 < 8 or y1 - y0 < 8:
        return out
    crop = img[y0:y1, x0:x1]
    b, g, r = cv2.split(crop)
    m = (cv2.subtract(r, cv2.max(g, b)) > cfg.red_thr).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    S, V = hsv[..., 1], hsv[..., 2]
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    ch, cw = m.shape
    min_area = np.pi * (cfg.dot_min_px / 2) ** 2
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < min_area or w < 3 or h < 3:
            continue
        fill = a / max(np.pi * w * h / 4, 1)
        if fill < 0.5:
            continue
        cs = cv2.findContours((lab[y:y + h, x:x + w] == i).astype(np.uint8), cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_NONE)[-2]
        per = cv2.arcLength(max(cs, key=len), True)
        circ = 4 * np.pi * a / max(per * per, 1)
        diam = 2 * np.sqrt(a / np.pi)
        cx, cy = cen[i]
        bl = {"c": (float(cx + x0), float(cy + y0)), "diam": float(diam), "circ": float(circ), "fill": float(fill),
              "ratio": float(diam / pred_diam) if pred_diam else float("nan"),
              "surr_S": float("nan"), "surr_V": float("nan"), "shape_ok": False, "ok": False,
              "touch": bool(x == 0 or y == 0 or x + w >= cw or y + h >= ch)}
        bl["shape_ok"] = circ >= cfg.dot_circ_min and cfg.dot_fill_min <= fill <= 1.15 and not bl["touch"]
        if bl["shape_ok"]:
            rad = diam / 2
            ya, yb = int(max(0, cy - 2 * rad)), int(min(ch, cy + 2 * rad + 1))
            xa, xb = int(max(0, cx - 2 * rad)), int(min(cw, cx + 2 * rad + 1))
            yy, xx = np.ogrid[ya:yb, xa:xb]
            dd = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
            ann = (dd > 1.3 * rad) & (dd < 1.8 * rad)
            if ann.sum() > 8:
                bl["surr_S"] = float(S[ya:yb, xa:xb][ann].mean())
                bl["surr_V"] = float(V[ya:yb, xa:xb][ann].mean())
            bl["ok"] = bl["surr_S"] < cfg.s_min and bl["surr_V"] > cfg.surround_v_min
        out["blobs"].append(bl)
    out["cands"] = [bl for bl in out["blobs"] if bl["ok"]]
    if pred_diam:
        sized = [bl for bl in out["cands"] if abs(bl["ratio"] - 1) <= cfg.dot_size_tol]
        out["best"] = min(sized, key=lambda bl: abs(bl["ratio"] - 1)) if sized else None
    elif out["cands"]:
        out["best"] = max(out["cands"], key=lambda bl: bl["circ"] * bl["diam"])
    return out


# =============================================================================
# 双目 / 3D
# =============================================================================
def _ray(rig, u, v):
    return np.array([(u - rig.cx) / rig.f, (v - rig.cy) / rig.f, 1.0])


def match_corners(cL, cR, d_guess, rig, cfg):
    """同极性 X 角点按行配对 → 3D 点（rect 系）。"""
    PL, PR = cL["pts"][cL["xok"]], cR["pts"][cR["xok"]]
    if len(PL) == 0 or len(PR) == 0:
        return np.zeros((0, 3)), np.zeros((0, 2))
    polL, polR = cL["pol"][cL["xok"]], cR["pol"][cR["xok"]]
    tol = 0.45 * 2 * cL["sq_px"]
    used = {}
    for i, (x, y) in enumerate(PL):
        sel = np.flatnonzero((np.abs(PR[:, 1] - y) <= cfg.y_tol) & (polR == polL[i]))
        if not len(sel):
            continue
        dx = np.abs(PR[sel, 0] - (x - d_guess))
        j, e = sel[np.argmin(dx)], dx.min()
        if e <= tol and (j not in used or e < used[j][1]):
            used[j] = (i, e)
    if not used:
        return np.zeros((0, 3)), np.zeros((0, 2))
    ii = np.array([i for i, _ in used.values()])
    jj = np.array(list(used.keys()))
    xl, y, xr = PL[ii, 0], PL[ii, 1], PR[jj, 0]
    d = xl - xr
    g = d > 0
    Z = rig.f * rig.B / d[g]
    P = np.stack([(xl[g] - rig.cx) * Z / rig.f, (y[g] - rig.cy) * Z / rig.f, Z], 1)
    return P, PL[ii][g]


def fit_plane(P, iters=100, thr=0.02, rng=None):
    rng = rng or np.random.default_rng(0)
    n = len(P)
    if n < 6:
        return None
    best = None
    idx = rng.integers(0, n, size=(iters, 3))
    for a, b, c in idx:
        nv = np.cross(P[b] - P[a], P[c] - P[a])
        L = np.linalg.norm(nv)
        if L < 1e-9:
            continue
        nv /= L
        inl = np.abs((P - P[a]) @ nv) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 6:
        return None
    Q = P[best]
    mu = Q.mean(0)
    _, _, Vt = np.linalg.svd(Q - mu)
    nv = Vt[2]
    if nv @ mu > 0:
        nv = -nv  # 法线指向相机
    rms = float(np.sqrt(np.mean(((Q - mu) @ nv) ** 2)))
    return {"mu": mu, "n": nv, "rms": rms, "n_pts": int(best.sum())}


def _basis(n):
    a = np.array([0.0, 1.0, 0.0]) if abs(n[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, n)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(n, e1)


def ring_center_in_plane(px, plane, rig, radius):
    """中心线像素点 → 射线与角点平面求交 → 平面内拟合固定半径圆 → 环心。"""
    mu, n = plane["mu"], plane["n"]
    rays = np.stack([(px[:, 0] - rig.cx) / rig.f, (px[:, 1] - rig.cy) / rig.f, np.ones(len(px))], 1)
    den = rays @ n
    ok = np.abs(den) > 1e-6
    s = (mu @ n) / den[ok]
    P = rays[ok] * s[:, None]
    P = P[s > 0]
    if len(P) < 10:
        return None
    e1, e2 = _basis(n)
    uv = np.stack([(P - mu) @ e1, (P - mu) @ e2], 1)

    def fit(uv):
        A = np.stack([uv[:, 0], uv[:, 1], np.ones(len(uv))], 1)
        sol, *_ = np.linalg.lstsq(A, (uv ** 2).sum(1), rcond=None)
        c = sol[:2] / 2
        r_free = float(np.sqrt(max(sol[2] + c @ c, 1e-9)))
        for _ in range(10):  # 固定半径 GN
            d = uv - c
            rho = np.linalg.norm(d, axis=1) + 1e-12
            J = -d / rho[:, None]
            step, *_ = np.linalg.lstsq(J, -(rho - radius), rcond=None)
            c = c + step
            if np.linalg.norm(step) < 1e-6:
                break
        res = np.linalg.norm(uv - c, axis=1) - radius
        return c, r_free, res

    c, r_free, res = fit(uv)
    mad = np.median(np.abs(res)) * 1.4826 + 1e-4
    keep = np.abs(res) < max(3 * mad, 0.01)
    if keep.sum() >= 10 and keep.sum() < len(uv):
        c, r_free, res = fit(uv[keep])
    return {"c": mu + c[0] * e1 + c[1] * e2, "n": n, "r_free": r_free,
            "rms": float(np.sqrt(np.mean(res ** 2))), "n_pts": int(len(res))}


# =============================================================================
# 结果结构
def _angle(a, b):
    """两条法线的夹角 (度), 不分正反"""
    return float(np.degrees(np.arccos(np.clip(abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b)), 0, 1))))


def ring_tilt_deg(c, n):
    """环面相对视线的倾角 (度): 0 = 正对"""
    return _angle(np.asarray(c, float), np.asarray(n, float))


def _row_roots(C, y):
    A = C[0, 0]
    Bq = 2 * (C[0, 1] * y + C[0, 2])
    C0 = C[1, 1] * y * y + 2 * C[1, 2] * y + C[2, 2]
    D = Bq * Bq - 4 * A * C0
    s = np.sqrt(np.where(D > 0, D, 0))
    r1, r2 = (-Bq - s) / (2 * A), (-Bq + s) / (2 * A)
    return np.minimum(r1, r2), np.maximum(r1, r2), D > 0


def ellipse_disparity_normal(elL, elR, rig, frac=0.8, n=60):
    """
    左右目中心线椭圆 → 环平面法线, 不用角点。
    平面在校正图上的视差是 (u, v) 的仿射函数 d = a + b·u + c·v (近的一侧视差大),
    逐行取左右两段弧的视差, 最小二乘拟合 (a, b, c) → 法线 ∝ (b, c, (a + b·cx + c·cy)/f)。
    """
    CL, CR = rd.ellipse_to_conic(elL), rd.ellipse_to_conic(elR)
    yc, hh = 0.5 * (elL[0][1] + elR[0][1]), 0.5 * max(elL[1])
    ys = np.linspace(yc - frac * hh, yc + frac * hh, n)      # 避开上下顶点 (交点对行太敏感)
    l1, l2, ok1 = _row_roots(CL, ys)
    r1, r2, ok2 = _row_roots(CR, ys)
    ok = ok1 & ok2
    if ok.sum() < 8:
        return None
    u = np.r_[l1[ok], l2[ok]]
    v = np.r_[ys[ok], ys[ok]]
    d = np.r_[l1[ok] - r1[ok], l2[ok] - r2[ok]]
    (a, b, c), *_ = np.linalg.lstsq(np.c_[np.ones_like(u), u, v], d, rcond=None)
    nn = np.array([b, c, (a + b * rig.cx + c * rig.cy) / rig.f])
    nrm = float(np.linalg.norm(nn))
    return nn / nrm if nrm > 0 and np.isfinite(nrm) else None


def pick_ring_normal(sols, plane_n, ref_n, cfg: GateConfig, disp_n=None):
    """
    单目椭圆的两个 (c, n) 解里挑。返回 (下标 j, 法线 n, n_src, 区分度°)。

    单目椭圆: 倾斜 ≥ 15° 时对的那个解远处也只差 1~3°, 但两个解几何上等价, 光靠椭圆分不出;
             接近正对时扁率被噪声主导, 两解各偏 5~12°, 平分线 (≈视线方向) 反而准。
      1) 有角点平面 (远处残差大, 法线仍有几度精度) → 选离它近的, 要拉开 normal_sep_deg     mono_plane
         且离选中的解 ≤ normal_ref_max_deg (角点太少的坏平面会和两个解都差很远)
      2) 左右目椭圆视差算的法线 (≤ eldisp_max_dist)  → 同上                               mono_disp
      3) 有估计器里的环法线 (机上一体时)            → 同上                               mono_est
      4) 两解夹角 ≤ normal_mid_deg  → 近正对/扁率是噪声: 取平分线, 误差 ≲ 夹角/2, 只显示     mono_mid
      5) 都不行 → 老办法 "最正对" 那个, 标二义 (另一个解放 n_alt, 飞机侧有环法线时再挑)   mono_amb
    """
    if len(sols) == 1:
        return 0, sols[0][1], "mono_mid", 0.0
    n0, n1 = sols[0][1], sols[1][1]
    for ref, tag in ((plane_n, "mono_plane"), (disp_n, "mono_disp"), (ref_n, "mono_est")):
        if ref is None:
            continue
        a = [_angle(n0, ref), _angle(n1, ref)]
        k = int(np.argmin(a))
        if a[1 - k] - a[k] >= cfg.normal_sep_deg and a[k] <= cfg.normal_ref_max_deg:
            return k, sols[k][1], tag, a[1 - k] - a[k]
    sep = _angle(n0, n1)
    j = int(np.argmax([-(n_ @ (c_ / np.linalg.norm(c_))) for c_, n_ in sols]))
    if sep <= cfg.normal_mid_deg:
        m = n0 + (n1 if n0 @ n1 > 0 else -n1)
        return j, m / np.linalg.norm(m), "mono_mid", sep
    return j, sols[j][1], "mono_amb", sep


# =============================================================================
@dataclass
class RingObs:
    ok: bool = False
    src: str = ""                 # "stereo"（角点平面）/ "mono"（椭圆，法线有二义）
    c: Optional[np.ndarray] = None  # rect 系
    n: Optional[np.ndarray] = None
    sigma_lat: float = 0.0
    sigma_z: float = 0.0
    partial: bool = False
    side: Optional[str] = None
    side_conf: float = 0.0
    n_corners: int = 0
    plane: Optional[dict] = None    # 只在检测端有 (不过网络)
    r_free_ratio: float = float("nan")
    ambiguous: bool = False
    plane_rms: float = float("nan")   # 角点平面残差 (m), 只在 src="stereo" 时有值
    n_src: str = ""                   # 法线怎么来的: plane(角点平面) / mono_plane(单目两解按平面定向)
                                      #   / mono_disp(按左右目椭圆视差定向) / mono_est(按估计器里的环法线)
                                      #   / mono_mid(近正对, 两解平分线) / mono_amb(没分清)  —— 见 pick_ring_normal
    n_alt: Optional[np.ndarray] = None  # 单目没分清时的另一个解 (飞机侧融合器有环法线时再挑)
    c_alt: Optional[np.ndarray] = None  # 另一个解对应的环心 (两个解的环心也不一样, 换法线时一起换)


@dataclass
class DotObs:
    ok: bool = False
    src: str = ""                 # "stereo" / "ray_plane" / "mono"
    p: Optional[np.ndarray] = None  # rect 系 3D（mono 时为单位射线）
    pix: Optional[tuple] = None   # 左目像素
    diam_px: float = 0.0
    metric_d: float = float("nan")
    sigma_lat: float = 0.0
    sigma_z: float = 0.0
    rp_delta: float = float("nan")  # 视差深度 - 射线平面深度（≈0 表示纸与环共面）


@dataclass
class TargetState:
    valid: bool = False
    pos_w: Optional[np.ndarray] = None
    cov_w: Optional[np.ndarray] = None
    src: str = ""
    age: float = float("inf")
    dist: float = float("nan")
    side: Optional[str] = None
    ring_c_w: Optional[np.ndarray] = None
    ring_n_w: Optional[np.ndarray] = None
    offset_w: Optional[np.ndarray] = None


@dataclass
class GateFrame:
    t: float = 0.0
    stage: str = "search"
    ring: RingObs = field(default_factory=RingObs)
    dots: List[DotObs] = field(default_factory=list)
    target: TargetState = field(default_factory=TargetState)
    used: List[str] = field(default_factory=list)      # 本帧被估计器接受的观测
    rejected: List[str] = field(default_factory=list)  # 被检验拒掉的观测
    skipped_ring: bool = False
    dot_roi: Optional[tuple] = None
    ring_seen: bool = False                            # 左目找到了环 (哪怕没成功定位)
    diag: dict = field(default_factory=dict)           # 诊断小数据 (可序列化): z_ring / plane_rms / 跳过了什么
    t_ms: dict = field(default_factory=dict)
    dbg: dict = field(default_factory=dict)            # 画图用的单目结果 (只在检测端, 不过网络)

    def dot(self, src=None):
        for d in self.dots:
            if d.ok and (src is None or d.src == src):
                return d
        return None


# =============================================================================
# 世界系估计器
# =============================================================================
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


# =============================================================================
# 主类（检测）
# =============================================================================
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
        """
        cfg, rig = self.cfg, self.rig
        t = time.time() if t is None else float(t)
        fr = GateFrame(t=t)
        self.n_frame += 1
        t0 = time.perf_counter()
        L, R = apply_wb(left, cfg), apply_wb(right, cfg)
        R_WR, t_WR = self._T_WR(T_WC)
        skip_ring, roi, dist_pred, z_pred = self._plan(R_WR, t_WR, t)
        fr.skipped_ring, fr.dot_roi = skip_ring, roi

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
        ring = fr.ring
        plane = None
        fr.ring_seen = bool(gL is not None and gL["ok"])
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
                sols = rd.circle_pose_from_ellipse(gL["center"], rig.K, cfg.r_center)
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

        # ---------- 融合 (T_WC=None 时只出阶段) ----------
        self.fuser.update(fr, T_WC, t)
        fr.t_ms["total"] = (time.perf_counter() - t0) * 1e3
        return fr


# =============================================================================
# 网络序列化（地面检测 → 飞机融合）
# =============================================================================
MSG_VERSION = 1
_DOT_SRC = ("stereo", "ray_plane", "mono")
_RING_SRC = ("stereo", "mono")
_NSRC = ("plane", "mono_plane", "mono_disp", "mono_est", "mono_mid", "mono_amb")
_N_FUSE = ("plane", "mono_plane", "mono_disp", "mono_est")     # 这几种法线才进估计器 (mono_mid 误差可到 ~10°, 只显示)


def _vec(a):
    return None if a is None else [float(x) for x in np.asarray(a, float).ravel()]


def _num(x, nd=6):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return round(x, nd) if np.isfinite(x) else None


def _arr3(v):
    """网络里来的向量: 必须是 3 个有限数, 否则 None。"""
    if v is None:
        return None
    try:
        a = np.asarray(v, float).ravel()
    except (TypeError, ValueError):
        return None
    return a if a.size == 3 and np.all(np.isfinite(a)) else None


def _pos(x, default=0.0):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return default
    return x if np.isfinite(x) and x > 0 else default


def frame_to_msg(fr: GateFrame) -> dict:
    """检测结果 → 纯 Python 字典 (msgpack/JSON 都能装)。只带相机(rect)系观测, 不带图像/位姿。"""
    r = fr.ring
    return {
        "v": MSG_VERSION, "t": float(fr.t), "stage": fr.stage,
        "ring_seen": bool(fr.ring_seen), "skipped_ring": bool(fr.skipped_ring),
        "dot_roi": [float(v) for v in fr.dot_roi] if fr.dot_roi is not None else None,
        "ring": {"ok": bool(r.ok), "src": r.src, "c": _vec(r.c), "n": _vec(r.n),
                 "sl": _num(r.sigma_lat), "sz": _num(r.sigma_z), "partial": bool(r.partial),
                 "side": r.side, "side_conf": _num(r.side_conf), "nc": int(r.n_corners),
                 "rfr": _num(r.r_free_ratio), "amb": bool(r.ambiguous), "prms": _num(r.plane_rms),
                 "nsrc": r.n_src, "n2": _vec(r.n_alt), "c2": _vec(r.c_alt)},
        "dots": [{"src": d.src, "p": _vec(d.p), "pix": [float(d.pix[0]), float(d.pix[1])],
                  "d": _num(d.diam_px), "md": _num(d.metric_d), "sl": _num(d.sigma_lat),
                  "sz": _num(d.sigma_z), "rp": _num(d.rp_delta)}
                 for d in fr.dots if d.ok and d.p is not None],
        "diag": {k: v for k, v in fr.diag.items() if isinstance(v, (bool, int, float, str))},
        "t_ms": {k: round(float(v), 2) for k, v in fr.t_ms.items()},
    }


def frame_from_msg(m: dict) -> GateFrame:
    """frame_to_msg 的逆。网络输入当不可信数据: 字段缺失/非法的观测直接丢掉, 不抛异常。"""
    fr = GateFrame(t=float(m.get("t") or 0.0))
    fr.ring_seen = bool(m.get("ring_seen"))
    fr.skipped_ring = bool(m.get("skipped_ring"))
    roi = m.get("dot_roi")
    if isinstance(roi, (list, tuple)) and len(roi) == 4:
        fr.dot_roi = tuple(float(v) for v in roi)
    r = m.get("ring") or {}
    c, n = _arr3(r.get("c")), _arr3(r.get("n"))
    ring = fr.ring
    ring.partial = bool(r.get("partial"))
    ring.side = r.get("side") if r.get("side") in ("front", "back") else None
    ring.side_conf = _pos(r.get("side_conf"))
    ring.n_corners = int(r.get("nc") or 0)
    if r.get("ok") and r.get("src") in _RING_SRC and c is not None and n is not None and c[2] > 0:
        nn = float(np.linalg.norm(n))
        if nn > 1e-6:
            ring.ok, ring.src, ring.c, ring.n = True, r["src"], c, n / nn
            ring.sigma_lat = _pos(r.get("sl"), 0.05)
            ring.sigma_z = _pos(r.get("sz"), 0.1)
            ring.ambiguous = bool(r.get("amb"))
            ring.r_free_ratio = float(r["rfr"]) if r.get("rfr") is not None else float("nan")
            ring.plane_rms = float(r["prms"]) if r.get("prms") is not None else float("nan")
            ring.n_src = r.get("nsrc") if r.get("nsrc") in _NSRC else ""
            n2 = _arr3(r.get("n2"))
            if ring.ambiguous and n2 is not None and float(np.linalg.norm(n2)) > 1e-6:
                ring.n_alt = n2 / float(np.linalg.norm(n2))
                c2 = _arr3(r.get("c2"))            # 老版地面端没有 c2 → 换法线时环心不换 (同以前)
                if c2 is not None and c2[2] > 0:
                    ring.c_alt = c2
    for d in m.get("dots") or []:
        if not isinstance(d, dict) or d.get("src") not in _DOT_SRC:
            continue
        p = _arr3(d.get("p"))
        pix = d.get("pix")
        if p is None or p[2] <= 0 or not (isinstance(pix, (list, tuple)) and len(pix) == 2):
            continue
        if d["src"] == "mono":
            p = p / np.linalg.norm(p)
        o = DotObs(True, d["src"], p, (float(pix[0]), float(pix[1])), _pos(d.get("d")),
                   float(d["md"]) if d.get("md") is not None else float("nan"),
                   _pos(d.get("sl"), 0.01), _pos(d.get("sz"), 0.05))
        if d.get("rp") is not None:
            o.rp_delta = float(d["rp"])
        fr.dots.append(o)
    diag = m.get("diag")
    if isinstance(diag, dict):
        fr.diag = {k: v for k, v in diag.items() if isinstance(v, (bool, int, float, str))}
    tms = m.get("t_ms")
    if isinstance(tms, dict):
        fr.t_ms = {k: float(v) for k, v in tms.items() if isinstance(v, (int, float))}
    return fr


def rig_to_msg(rig) -> dict:
    """飞机侧融合只需要这几个数 (和地面检测用的是**同一份**标定)。"""
    return {"f": float(rig.f), "cx": float(rig.cx), "cy": float(rig.cy), "B": float(rig.B),
            "R1": _vec(rig.R1), "size": [int(rig.size[0]), int(rig.size[1])]}


# =============================================================================
# 可视化
# =============================================================================
STAGE_COLOR = {"search": (128, 128, 128), "far": (255, 200, 0), "mid": (0, 255, 0), "near": (0, 220, 255),
               "terminal": (0, 128, 255), "coast": (200, 200, 200), "blind": (0, 0, 255), "lost": (80, 80, 80)}


def draw_frame(img, fr: GateFrame, det: GateDetector, T_WC=None, extra_lines=None, text=True):
    """
    extra_lines: 额外显示的文字行。
    text=False: 只画椭圆 / 角点 / 红点 / 法线 (不写字) —— 地面 gcs_gate 用:
                网页的数字走 X-Det, 本地窗口的字由 HUD 在 1920×1080 画布上画 (同 gcs_aruco_det2)。
    """
    vis = apply_wb(img, det.cfg).copy()
    rig = det.rig
    gL = fr.dbg.get("gL")
    if gL is not None and gL["ok"]:
        cv2.ellipse(vis, gL["center"], (0, 255, 0), 1, cv2.LINE_AA)
        cv2.ellipse(vis, gL["outer"], (0, 255, 255), 1, cv2.LINE_AA)
        cv2.ellipse(vis, gL["inner"], (255, 255, 0), 1, cv2.LINE_AA)
    for x, y in fr.dbg.get("matched", np.zeros((0, 2))):
        cv2.circle(vis, (int(x), int(y)), 2, (0, 140, 255), -1, cv2.LINE_AA)
    if fr.dot_roi is not None:
        x0, y0, x1, y1 = (int(v) for v in fr.dot_roi)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 255), 1)
    d = fr.dot()
    if d is not None:
        cv2.circle(vis, (int(d.pix[0]), int(d.pix[1])), int(d.diam_px / 2) + 5, (255, 0, 0), 2, cv2.LINE_AA)
    if fr.ring.ok and fr.ring.n is not None:
        # 法线箭头 (长 = 环半径): 红 = 角点平面, 品红 = 单目解已定向, 橙 = 近正对取平分线, 灰 = 两解没分清 (两个都画)
        c = fr.ring.c
        L_ = det.cfg.outer_d / 2

        def px(p):
            return (int(rig.f * p[0] / p[2] + rig.cx), int(rig.f * p[1] / p[2] + rig.cy))
        q0 = px(c)
        cv2.drawMarker(vis, q0, (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
        ns = [fr.ring.n] + ([fr.ring.n_alt] if fr.ring.ambiguous and fr.ring.n_alt is not None else [])
        col = (0, 0, 255) if fr.ring.src == "stereo" else (
            (160, 160, 160) if fr.ring.ambiguous else ((0, 165, 255) if fr.ring.n_src == "mono_mid" else (255, 0, 255)))
        for n_ in ns:
            n_ = n_ if n_ @ c < 0 else -n_            # 统一画成指向相机这一侧
            p = c + L_ * n_
            if p[2] > 0.05:
                cv2.arrowedLine(vis, q0, px(p), col, 2, cv2.LINE_AA, tipLength=0.2)
    tg = fr.target
    if tg.valid and T_WC is not None:
        R_WR, t_WR = det._T_WR(T_WC)
        pix, _ = det._project(tg.pos_w, R_WR, t_WR)
        if pix is not None:
            cv2.drawMarker(vis, (int(pix[0]), int(pix[1])), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
    if not text:
        return vis
    col = STAGE_COLOR.get(fr.stage, (255, 255, 255))
    lines = [f"[{fr.stage.upper()}]  {fr.t_ms.get('total', 0):.0f}ms"
             + (" ring-skipped" if fr.skipped_ring else "")]
    r = fr.ring
    if r.ok:
        s = f"ring {r.src} c=({r.c[0]:+.2f},{r.c[1]:+.2f},{r.c[2]:.2f}) corners={r.n_corners} side={r.side}"
        if r.src == "stereo":
            s += f" rms={r.plane_rms * 1000:.0f}mm r/R={r.r_free_ratio:.3f}"
        if r.n is not None:
            s += f"  n:{r.n_src or '?'} tilt={ring_tilt_deg(r.c, r.n):.0f}deg"
        lines.append(s)
    for dd in fr.dots:
        if dd.src == "mono":
            lines.append(f"dot mono  pix=({dd.pix[0]:.0f},{dd.pix[1]:.0f}) d={dd.diam_px:.0f}px")
        else:
            lines.append(f"dot {dd.src:9s} Z={dd.p[2]:.3f}m  diam={dd.metric_d * 100:.1f}cm  "
                         f"sz={dd.sigma_z * 100:.1f}cm" + (f"  rp_delta={dd.rp_delta * 100:+.1f}cm"
                                                           if dd.rp_delta == dd.rp_delta else ""))
    if tg.valid:
        sd = np.sqrt(np.diag(tg.cov_w))
        lines.append(f"TARGET {tg.src} dist={tg.dist:.2f}m sd=({sd[0] * 100:.1f},{sd[1] * 100:.1f},{sd[2] * 100:.1f})cm "
                     f"age={tg.age * 1000:.0f}ms side={tg.side}")
    if fr.rejected:
        lines.append("rejected: " + ",".join(fr.rejected))
    dg = fr.diag
    if dg:
        lines.append("diag " + "  ".join("%s=%s" % (k, v) for k, v in dg.items()))
    lines += list(extra_lines or [])
    for i, s in enumerate(lines):
        y = 20 + 18 * i
        cv2.putText(vis, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, col if i == 0 else (255, 255, 255), 1,
                    cv2.LINE_AA)
    return vis
