# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/frame.py —— 红点一帧结果的数据结构 (Cand / DotFrame)
===============================================================================
Cand = 地面检测端回传的一个候选, **只有像素量** (PLAN_dot_v1 §1): 圆心、长短轴、短轴方向、形状特征、
左右视差。Z / 物理直径 / 世界系 / 门限 / 时序都在机上 (locate.py)。
为了让飞机侧 Locator(cfg, f, R1) 的构造签名不变, 每个候选自带归一化坐标 (xn, yn) 和 f·B (fb, px·m),
机上不需要 cx / cy / B。

DotFrame 的属性名和环的 GateFrame 对齐 (target / stage / ring / dots / used / rejected / skipped_ring / diag / t_ms),
这样 vision_bridge.GateOut.emit 不用改就能发布。ring 是一个永远 ok=False 的空对象。
numpy-only (飞行脚本 / 机上融合不拉 cv2)。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from vb09.perception.ring.frame import DotObs, RingObs, TargetState   # 通用小结构, 只依赖 numpy

NAN = float("nan")


@dataclass
class Cand:
    u: float = 0.0                # 左目 (校正后) 像素, 半高加权矩圆心
    v: float = 0.0
    xn: float = 0.0               # (u − cx)/f
    yn: float = 0.0               # (v − cy)/f
    fb: float = 0.0               # f·B (px·m): Z = fb / 视差
    dmaj: float = 0.0             # 长轴 (px) = 4·σ_major (半高掩膜), 等于圆盘直径 (与倾斜无关)
    dmin: float = 0.0             # 短轴 (px)
    mx: float = 0.0               # 短轴方向 (像面内单位向量)
    my: float = 1.0
    area: float = 0.0             # 红度 > 门限的连通域面积 (px)
    fill: float = 0.0
    circ: float = 0.0
    axr: float = 1.0              # 二阶矩 短轴/长轴
    red: float = 0.0              # 红度均值
    nw: int = 0                   # 1.3–1.8R 环带 8 扇区里"白"(亮且低饱和) 的个数
    nn: int = 0                   # 其中"不红"的个数
    sat: float = 0.0
    disp: float = NAN             # 视差 (px), 右目半高矩圆心之差; NaN = 没配上
    pk: float = 0.0               # 模板匹配峰值
    uniq: float = 1.0             # 第二峰 / 第一峰
    dy: float = 0.0               # 左右目圆心行差 (px)
    partial: bool = False         # 贴图像边缘 / 被切掉一部分
    ps: float = 0.0               # 地面的像素预筛分 (0–1), 只用来排序 / 截断


@dataclass
class DotFrame:
    t: float = 0.0
    stage: str = "search"
    cands: List[Cand] = field(default_factory=list)
    ring: RingObs = field(default_factory=RingObs)             # 空对象 (emit 要读)
    dots: List[DotObs] = field(default_factory=list)
    target: TargetState = field(default_factory=TargetState)
    used: List[str] = field(default_factory=list)
    rejected: List[str] = field(default_factory=list)
    skipped_ring: bool = False
    dot_roi: Optional[tuple] = None
    ring_seen: bool = False
    diag: dict = field(default_factory=dict)
    t_ms: dict = field(default_factory=dict)
    dbg: dict = field(default_factory=dict)

    def dot(self, src=None):
        for d in self.dots:
            if d.ok and (src is None or d.src == src):
                return d
        return None
