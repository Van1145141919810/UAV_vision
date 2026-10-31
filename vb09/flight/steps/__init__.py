# -*- coding: utf-8 -*-
"""
vb09/flight/steps —— 飞行步骤 (可插拔)。组合成任务见 vb09/flight/modes.py。一个模式家族一个文件:
  base     Step / StepFail / StepBack           接口
  common   Acquire, move_to, turn_to, align_lock   各模式共用
  dash     make_plan / PlanDash, Reposition, Align, Commit, Dash   冲刺穿环
  shadow   AlignHere, Shadow                     影子
  dock     StandOff, Hold                        停靠
"""
