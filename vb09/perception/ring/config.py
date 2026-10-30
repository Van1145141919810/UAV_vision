# -*- coding: utf-8 -*-
"""
vb09/perception/ring/config.py —— 环检测 + 估计器的全部参数 (GateConfig)
===============================================================================
2026-10-30 从 gate_detector.py 原样搬来, **默认值一个字都不改** (默认没有 gate_params.json,
两端都用这里的代码默认值 ⇒ 默认值就是现场参数)。字段名与 gate_tune.py 的 gate_params.json 兼容。
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields


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
