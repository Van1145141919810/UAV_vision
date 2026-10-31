# -*- coding: utf-8 -*-
"""
vb09/perception/ring/plugin.py —— 环插件: Detector / Locator 两个适配器 (registry.DETECTORS / LOCATORS)
===============================================================================
2026-11-01 接线。只是把原来散在 gcs_gate.py / vision_bridge.py 里对 GateDetector / GateFuser / draw_frame /
ring_extra 的直接调用收成两个固定接口, 算法和顺序都不变。

  RingDetector(rig, cfg)        地面检测 (不给 T_WC) / 机上检测 (给 T_WC, 检测器内部自带融合)
      process(L, R, t, T_WC=None) → GateFrame
      reset()                       整个检测器重建 (估计器旧状态清掉)
      draw(img, fr, T_WC=None, text=True) → 叠加图
      extra(fr, T_WC, warn=None)    机上检测时 /gate/status 的环附加字段 (ring_w / n / aim …)
  RingLocator(cfg, f, R1)       机上融合 (地面检测, gate_gcs 模式): 收相机系观测 → 世界系估计
      same_rig(f, R1) / f
      update(fr, T_WC, t)           fr 由 codec.frame_from_msg 得到; 更新后 fr.target 是融合结果
      extra(fr, T_WC, warn=None)    /gate/status 的环附加字段

★ 这个文件顶层不 import cv2 / 检测库: LOCATORS 只要 numpy (机上 gate_gcs 进程不拉起检测库)。
"""
from __future__ import annotations

from .locate import ring_extra


class RingDetector(object):
    kind = "ring"

    def __init__(self, rig, cfg):
        from .detect import GateDetector          # cv2 —— 只有检测端才 import
        self._cls = GateDetector
        self.rig, self.cfg = rig, cfg
        self.det = GateDetector(rig, cfg)

    def process(self, L, R, t, T_WC=None):
        return self.det.process(L, R, t, T_WC)

    def reset(self):
        self.det = self._cls(self.rig, self.cfg)

    def draw(self, img, fr, T_WC=None, text=True):
        from .draw import draw_frame
        if text:
            return draw_frame(img, fr, self.det, T_WC)
        return draw_frame(img, fr, self.det, T_WC, text=False)

    def extra(self, fr, T_WC, warn=None):
        return ring_extra(self.det.fuser, fr, T_WC, warn=warn)


class RingLocator(object):
    kind = "ring"

    def __init__(self, cfg, f, R1):
        from .fusion import GateFuser
        self.fuser = GateFuser(cfg, f, R1)

    @property
    def f(self):
        return self.fuser.f

    def same_rig(self, f, R1):
        return self.fuser.same_rig(f, R1)

    def update(self, fr, T_WC, t):
        return self.fuser.update(fr, T_WC, t)

    def extra(self, fr, T_WC, warn=None):
        return ring_extra(self.fuser, fr, T_WC, warn=warn)
