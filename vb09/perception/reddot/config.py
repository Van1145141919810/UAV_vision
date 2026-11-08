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
  * 实物直径 15 cm (卷尺)。旧的软权重矩长轴偏小 (28 px −4…−7%) → 反推 13.7 cm; 改成面积法后 14.1–15.1 cm
  * 2026-11-05 起**不假设直径**: 每条轨迹用自己的近处双目帧估直径 (locate.py), 估好之前只用双目;
    dot_d 只给"远处刚发现、没有双目"的帧当很宽的先验 (size_prior_rel)
这些阈值是看着这批数据定的, **没有留出验证集** (见 RESULT 文档), 上场前要用新数据复核。
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields


@dataclass
class DotConfig:
    # ---- 物理 ----
    dot_d: float = 0.15             # m, 标称直径: 只当没有双目时的宽先验 (σ = size_prior_rel), 不参与有双目时的测距
    dot_d_cal: float = 0.0          # m, >0: 所有轨迹都用这个直径 (不再按轨迹估)。一般不填
    d_min: float = 0.08             # m, 双目算出的物理直径必须在 [d_min, d_max] (红点 10–21 cm; 红桶 28 cm、红格子 2–3 cm 过不了)
    d_max: float = 0.24

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

    # ---- 尺寸测距 (按轨迹估直径 → 单目测距, 远场比视差好; 2026-11-05 改) ----
    # 每条轨迹自己攒近处 (视差 ≥ cal_disp_min)、静止 (角速度 < rate_max_deg) 双目帧的 D_st = Z_st·dmaj/f, 中位数 = 这条轨迹的直径。
    # 攒够 cal_min_n 之前: 有双目 → 只用双目; 没双目 → dot_d 宽先验 (锁不了, 只够"发现")。
    # 直径按世界位置记住 (d_mem_radius), 估计器重建 / 轨迹删掉后再看到同一处的红点直接用 —— 不再出现
    #   "丢失 1 s 后回到标称直径 → 距离偏长 30 cm 几秒" (2026-11-05 实测 4.4x m → 4.05 m)。
    size_sigma_floor: float = 0.02  # 尺寸测距相对误差下限
    size_edge_px: float = 0.6       # (ref px) 直径量测的边缘噪声 → 相对误差 = edge/d
    size_prior_rel: float = 0.40    # 没有轨迹直径时, 用 dot_d 当先验的相对 σ (10–21 cm 都在 ±1σ 左右)
    cal_disp_min: float = 18.0      # (ref px) 视差 ≥ 此值 (≈ 5.4 m 内) 的双目帧才用来估直径
    cal_min_n: int = 8              # 至少这么多样本才启用轨迹直径
    cal_buf: int = 60               # 每条轨迹最多留多少个直径样本
    d_mem_radius: float = 0.5       # m, 记住的直径按位置匹配的半径
    consist_k: float = 4.0          # 双目 Z 和尺寸 Z 差超过 k·σ → 这一帧只用双目 (不融合, 也不拒绝)
    mono_score_min: float = 0.80    # 没有有效立体 (只有尺寸) 时, 单帧分至少这么高并且有白边才接受
    rate_max_deg: float = 15.0      # °/s, 相机角速度 (由相邻帧位姿算) 超过 → 这帧算"在转": 不用尺寸测距、不攒直径
                                    #   (运动模糊把长轴拉长 → 尺寸距离偏短, 2026-11-05 转头回来时轨迹被拉近 ~5 cm)
    motion_dt: float = 0.05         # s, 转动时观测横向协方差加 (Z·ω·motion_dt)² (曝光 + 时间戳误差)

    # ---- 世界系估计: 分桶后验 + 立体比例 (2026-11-08, PLAN_dot_v2 §2) ----
    # 每条轨迹把观测按 (视线方向 pos_bin_deg × 距离 pos_bin_rng 的对数档) 分桶, 桶内按 1/σ² 加权平均。
    # 同一视点的几百帧只算"一次观测 + 不随帧数下降的下限" (外参 / VIO / 时间戳的误差在同一视点里是同一个),
    # 换视角 / 换距离才是新信息 → 报出来的 σ 和实际误差同量级 (以前 1.7–2.2 cm 报, 实际 4.8 cm)。
    # 位置 p 和立体比例 m = 1/k (k = 立体距离 / 真距离) 一起解 (线性最小二乘, 4 个未知数):
    #   桶均值 c̄ + m·b̄ = p, c̄ = 相机位置均值, b̄ = 立体距离 × 视线方向 的均值
    # 抵近 (距离变化大) 时 m 可观测 → 立体的比例误差 (reddot_exp 约 +3%) 被 VIO 位移标掉, 不用卷尺 / 不用直径。
    pos_bin_deg: float = 2.0        # °, 视线方向分桶
    pos_bin_rng: float = 0.15       # 距离分桶: 每档 ×(1+此值)
    max_bins: int = 48              # 每条轨迹最多留多少桶 (超了丢最久没更新的)
    along_floor_rel: float = 0.025  # 桶内沿视线 σ 下限 (相对距离): 同一视点的视差误差帧间相关。reddot_exp 跨视点随机部分 ≈ 2.5%
    drift_q: float = 0.01           # m/√s, 桶的"老化": VIO 慢漂移, 久没更新的桶协方差 += q²·Δt (远处早期的桶自然淡出)
    robust_k: float = 3.0           # 桶残差 > k·σ → 按 Huber 降权 (野值 / 误配)
    scale_prior_sd: float = 0.03    # m 的先验 σ (还没学到比例时)。先验均值 = 本次运行里已学到的比例 (没有 = 1)。
                                    #   标定验到 1–2%; 5% 时合成里一段连续偏短的视差会把 k 追过头 (4σ), 3% 时最差 3.4σ
    scale_floor_sd: float = 0.01    # 把学到的比例当别的轨迹的先验时, σ 至少这么大
    scale_learn_sd: float = 0.02    # 一条轨迹的 m 的 σ 小于它 → 记成本次运行的立体比例
    scale_min: float = 0.85
    scale_max: float = 1.15
    d_gate_k: float = 2.0           # 物理直径门限按 σ 放宽: D 在 [d_min − kσ_D, d_max + kσ_D] (远处视差噪声大, 21 cm 的红点 10 m 处会读到 24+ cm)
    # 选点 (飞行脚本经 /gate/select 发来; 桥里只做关联)
    sel_ang_deg: float = 2.0        # 按像素 (视线) 选点: 候选 / 轨迹离这条视线的夹角上限
    sel_lat: float = 0.25           # m, … 或者垂直距离上限 (近处用)
    sel_rebind: float = 0.5         # m, 选中的轨迹没了 (删掉 / 合并 / 估计器重建) → 在记住的位置这么近内重新挂上

    # ---- 世界系估计 (和 ring GateConfig 同名同义) ----
    q_pos: float = 0.02             # m/√s, 关联用: 轨迹 P 随时间放大 (后验本身的老化见 drift_q)
    gate_chi2: float = 16.0
    track_radius: float = 0.30      # m, χ² 不过但离轨迹在这么近以内 → 仍并进这条轨迹; 两条轨迹靠这么近 → 合并
    # 观测协方差下限 (2026-11-05): 像素噪声 (sigma_px) 只有零点几 cm, 但外参 / VIO / 时间戳带来的误差是 cm 级,
    #   且随视角变。只用像素噪声 → 换个视角 χ² 就不过 → 同一个红点被拆成好几条轨迹 (reddot_exp 实测 11 个视点 6–8 条)
    obs_ang_floor_deg: float = 0.5  # °, 横向: 相当于外参误差 (重标 T1 之前旧外参误差 ~6°, 只能靠 track_radius 兜)
    obs_pos_floor: float = 0.02     # m, 各向
    track_timeout: float = 3.0      # s, 轨迹这么久没更新就删
    max_tracks: int = 8
    min_hits: int = 2               # 轨迹至少这么多次命中才发布
    min_score: float = 0.30
    lost_timeout: float = 0.5       # s
    blind_dist: float = 0.3         # m

    # ---- 法向 (椭圆 + 多视角, 2026-11-05 改) ----
    # 单帧椭圆给两个候选, 互为关于视线的镜像, 单个视角分不开 (reddot_exp: 0004 / 0007 选错 73–84°)。
    # 视线方向变 δ, 错的候选移动 ≈ 2δ·cosθ, 对的不动 → 要求不同视角的样本里只有一个解一致。
    # 视角变化可以是竖直的 (停靠后上下扫描, flight 的 GATE_VSCAN_DZ): 3 m 处 ±0.3 m ≈ 11°。
    # reddot_exp 两帧配对: 视角差 ≥ 10° 时 25/25 对, 余量 ≥ 15°; 5–10° 时 6/6 对但余量只有 5–12°
    normal_min_px: float = 25.0     # (ref px) 直径小于此值不出法向 (椭圆扁率被模糊和噪声主导)
    normal_buf: int = 40            # 每条轨迹按视角分桶最多留多少桶 (每桶留最新一帧)
    normal_bin_deg: float = 1.5     # 视角分桶大小 (按视线方向; 悬停时的几百帧只占 1 桶, 不会把多视角挤掉)
    normal_inlier_deg: float = 12.0
    normal_min_k: int = 3           # 至少这么多个视角桶一致
    normal_view_min_deg: float = 10.0   # 一致的样本里视线方向的跨度至少这么大 (不够 → 不发 n)
    normal_lr_min: float = 16.0     # 两个假设的拟合代价差 (Σ 残差²/σ², ≈ 似然比 χ²) 至少这么大才发 n。
                                    #   竖直扫描时错解只被拖开 ±2ε·cosθ (3 m、±0.3 m ≈ ±9°), 数内点分不开, 要比残差
    normal_sig_r: float = 1.2       # (ref px) 单帧法向噪声: σ_cosθ = max(0.02, 此值/长轴), σ_θ = σ_cosθ / sinθ
    normal_floor_deg: float = 2.0   # n_sd_deg 下限 (实测一致帧与多帧法向差 1–6°)

    # 默认参数文件 (registry.load_config: 桥 / gcs_gate 没显式给别的文件时读它, 不读环的 gate_params.json)
    PARAMS_FILE = "reddot_params.json"

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2, ensure_ascii=False)
