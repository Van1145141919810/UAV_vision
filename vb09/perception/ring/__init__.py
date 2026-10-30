# -*- coding: utf-8 -*-
"""
vb09/perception/ring —— 格子环 + 红点 识别（正式版）

2026-10-30 从 gate_detector.py 拆成: config (GateConfig) / frame (结果结构) / det2d (单目 2D) /
geom3d (双目 3D、单目椭圆位姿) / fusion (估计器, 只依赖 numpy) / detect (GateDetector) / draw (画图) /
locate (飞机侧附加字段); 网络序列化在 vb09/common/wire.py。代码原样, 下面是原来的说明。

任务: 识别比赛环（外径 1.6m / 内径 1.2m，左红格右蓝格，上下渐变色带）及其中间纸上的红点(11cm)，
      输出红点在世界系下的位置，供最终冲撞。不依赖 ROS；双目校正用 vb09/perception/stereo_rig.StereoRig。

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

★ 标定单位: stereo_calib.yaml 的 T 是 mm。stereo_rig.load_stereo_calib 现在会自动换成米
  (旧版直接当米 → 基线 60m → 双目深度大 1000 倍 → 角点平面 / 红点双目从来没成立过)。

接入示例（vision_bridge 里）
-------------------------
    from vb09.perception.stereo_rig import StereoRig
    from vb09.perception.ring.config import GateConfig
    from vb09.perception.ring.detect import GateDetector
    rig = StereoRig.from_file("stereo_calib.yaml", scale=0.5)
    det = GateDetector(rig, GateConfig.load("gate_params.json"))
    ...
    L, R = rig.rectify(*rig.split(sbs_frame))
    T_WC = T_WB_at(t_cap) @ T_BC            # 你的 VIO 位姿(曝光时刻) × 相机外参
    f = det.process(L, R, t_cap, T_WC)
    if f.target.valid: 目标 = f.target.pos_w, 协方差 f.target.cov_w, 来源 f.target.src, 阶段 f.stage
"""
