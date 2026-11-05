# -*- coding: utf-8 -*-
"""
vb09/flight/modes.py —— 任务 = 步骤列表 (显式字典, 不扫描目录)
===============================================================================
run_mission(ctx, MODES[mode]) 依次执行; 放弃 / 回手动段 / 收尾都在 runner 里, 和步骤无关。
  dash    冲刺穿环 (原 phase_gate 的 else 分支)
  shadow  影子: 识别 / 锁定 / 规划照做, 不摆位不冲刺, 计划发网页, 回手动段 (原 GATE_SHADOW 分支)
  dock    停靠: 识别 → 飞到目标前 GATE_DOCK_D 停住对正 → (上下扫描 ±GATE_VSCAN_DZ 定法向) → 悬停 GATE_DOCK_HOLD_S
          记录检测 → 回手动段 (第 7 步; 上下扫描 2026-11-05)
以后: 红点的 Detect / Select / Approach / Confirm。
"""
from vb09.flight.steps.common import Acquire
from vb09.flight.steps.dash import Align, Commit, Dash, PlanDash, Reposition
from vb09.flight.steps.dock import Hold, StandOff, VScan
from vb09.flight.steps.shadow import AlignHere, Shadow

MODES = {
    "dash": [Acquire(), PlanDash("粗规划"), Reposition(), Align(), Commit(), Dash()],
    "shadow": [Acquire(), PlanDash("粗规划"), AlignHere(), Shadow()],
    "dock": [Acquire(), StandOff(), VScan(), Hold()],     # VScan: GATE_VSCAN_DZ = 0 时什么都不做
}
