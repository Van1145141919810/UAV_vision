# -*- coding: utf-8 -*-
"""
vb09/perception/ring/frame.py —— 一帧检测结果的数据结构 (RingObs / DotObs / TargetState / GateFrame)
===============================================================================
2026-10-30 从 gate_detector.py 原样搬来。检测 (detect.py)、融合 (fusion.py)、序列化 (vb09/common/wire.py)、
画图 (draw.py) 共用。坐标: rect = 校正后左目系; world = VIO odom。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


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
