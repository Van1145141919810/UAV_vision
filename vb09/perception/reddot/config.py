# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/config.py —— 红点检测 + 机上定位的全部参数 (DotConfig)
===============================================================================
2026-11-05。接口和 ring/config.py 的 GateConfig 一样: dataclass, load(path) / save(path), lost_timeout 属性。
**默认值 = 现场参数**(默认没有 gate_params.json, 地面和飞机都用这里的代码默认值)。改默认值会触发版本检查
(common/wire.cfg_digest), 地面、飞机两边要一起更新。

数值来源 (claude/RESULT_reddot_p1_2026-11-05.md, reddot_exp.zip 15 帧, 1.8–5.9 m, 室内, 白卡片上的红点):
  * 红点在 r = R − max(G,B) 上平台值 90–145; 填充率 0.74–0.80 (正圆盘 π/4); 圆度 0.90–0.93
  * 泡沫块填充率 > 0.9 (方的), 红桶直径 28 cm → 物理直径 + 形状就能把它们分开
  * 双目视差: 比 VIO 射线交会真值偏小约 1 px (均值), 单帧噪声约 ±0.7 px —— 远比理想的 0.15 px 大
  * 用长轴算的物理直径 13.7 ± 0.3 cm (标称 15 cm, 差 9%, 原因没查清: 实物尺寸 / VIO 尺度 / 外参),
    所以尺寸测距用"近处双目自校准"的有效直径 D_eff (见 locate.py), 量到实物后填 dot_d_cal 就不再自校准
这些阈值是看着这批数据定的, **没有留出验证集** (见 RESULT 文档), 上场前要用新数据复核。
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields


@dataclass
class DotConfig:
    # ---- 物理 ----
    dot_d: float = 0.15             # m, 红点标称直径
    dot_d_cal: float = 0.0          # m, >0: 量到的有效直径, 直接用, 不再用近处双目自校准
    size_tol: float = 0.30          # 双目算出的物理直径离 dot_d 超过 ±30% → 不是红点

    # ---- 分割 (地面检测端) ----
    ref_f: float = 1626.6           # 参考焦距 (全分辨率校正左目, 2026-11-04 标定)。下面标 "(ref px)" 的门限 × (f / ref_f)
    red_thr: int = 50               # r = R − max(G,B) 分割门限
    min_d_px: float = 10.0          # (ref px) 最小等效直径。15 cm 红点 ≈ 14 m 处 17 px, 20 m 处 12 px
    max_d_px: float = 450.0         # (ref px) 最大等效直径 (≈ 0.5 m 内)
    max_cands: int = 10             # 每帧最多回传几个候选 (按像素分排)
    open_k: int = 3                 # 分割后开运算核

    # ---- 像素门限 (机上判定; 地面只做松的预筛) ----
    shape_px: float = 20.0          # (ref px) 直径 ≥ 此值用严格形状门限, 否则用松的 (小目标形状量不可信)
    fill_lo: float = 0.70
    fill_hi: float = 0.86           # 正圆盘 π/4 = 0.785
    circ_min: float = 0.78
    fill_lo_s: float = 0.55
    fill_hi_s: float = 0.95
    circ_min_s: float = 0.60
    axr_min: float = 0.50           # 短轴/长轴 ≥ 此值 (视线和法向夹角 ≲ 60°)
    red_min: float = 60.0           # 红度 (r 均值) 下限
    white_min: int = 6              # 周围白环扇区 ≥ 此值算"白边" (加分; 没有立体时必须有)

    # ---- 立体 ----
    disp_max: float = 260.0         # (ref px) 视差搜索上限 (≈ 0.38 m)
    disp_min: float = 3.0           # (ref px) 视差下限 (> 32 m 不信)
    match_pk_min: float = 0.85      # 红度图模板匹配峰值下限
    uniq_max: float = 0.92          # 第二峰 / 第一峰 上限 (红蓝格子之类重复纹理会出现两个一样高的峰)
    row_tol: float = 2.5            # (ref px) 左右目行差上限 (实测最大 0.8)
    disp_bias: float = 0.0          # (ref px) 视差零点修正 (加到测得视差上)。实测均值 −1.0±0.7 px 未采用 (外参 / VIO 时间戳也没定),
                                    #   要在 3/5/10 m 卷尺核对后再填
    sigma_disp: float = 0.7         # (ref px) 视差噪声 (实测)
    sigma_px: float = 0.4           # (ref px) 圆心像素噪声

    # ---- 尺寸测距 (直径固定 → 单目测距, 远场比视差好) ----
    size_sigma_floor: float = 0.02  # 尺寸测距相对误差下限
    size_edge_px: float = 0.6       # (ref px) 直径量测的边缘噪声 → 相对误差 = edge/d
    cal_disp_min: float = 18.0      # (ref px) 视差 ≥ 此值 (≈ 5.4 m 内) 的双目帧才用来自校准有效直径
    cal_min_n: int = 8              # 至少这么多样本才启用 D_eff
    cal_clamp: float = 0.20         # D_eff 限制在 dot_d·(1 ± 此值)
    consist_k: float = 4.0          # 双目 Z 和尺寸 Z 差超过 k·σ → 拒绝 (有一个不是红点)
    mono_score_min: float = 0.80    # 没有有效立体 (只有尺寸) 时, 单帧分至少这么高并且有白边才接受

    # ---- 世界系估计 (和 ring GateConfig 同名同义) ----
    q_pos: float = 0.02             # m/√s, 红点固定 → 小
    gate_chi2: float = 16.0
    track_radius: float = 0.30      # m, 新观测离已有轨迹超过这么远 (且 χ² 不过) → 新轨迹
    track_timeout: float = 3.0      # s, 轨迹这么久没更新就删
    max_tracks: int = 8
    min_hits: int = 2               # 轨迹至少这么多次命中才发布
    min_score: float = 0.30
    lost_timeout: float = 0.5       # s
    blind_dist: float = 0.3         # m

    # ---- 法向 (椭圆, 多帧一致) ----
    normal_min_px: float = 25.0     # (ref px) 直径小于此值不出法向 (椭圆扁率被模糊和噪声主导)
    normal_buf: int = 20            # 每条轨迹留多少帧的候选
    normal_inlier_deg: float = 12.0
    normal_min_k: int = 3           # 至少这么多帧一致
    normal_floor_deg: float = 2.0   # n_sd_deg 下限 (实测一致帧与多帧法向差 1–6°)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2, ensure_ascii=False)
