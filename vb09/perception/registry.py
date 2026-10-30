# -*- coding: utf-8 -*-
"""
vb09/perception/registry.py —— 感知插件注册表 (显式字典, 不自动扫描目录)
===============================================================================
  JUDGES[kind](C)   飞行脚本侧的单帧判定 (C = vb09.flight.config)
以后: DETECTORS[kind] (地面)、LOCATORS[kind] (机上, vision_bridge 里)。加一种目标 = 写插件 + 这里加一行。
"""
from vb09.perception.ring.judge import RingJudge

JUDGES = {
    "ring": RingJudge,
}
