# -*- coding: utf-8 -*-
"""
vb09/flight/modes.py —— 任务 = 步骤列表 (显式字典, 不扫描目录)
===============================================================================
run_mission(ctx, MODES[mode]) 依次执行; 放弃 / 回手动段 / 收尾都在 runner 里, 和步骤无关。
  dash    冲刺穿环 (原 phase_gate 的 else 分支)
  shadow  影子: 识别 / 锁定 / 规划照做, 不摆位不冲刺, 计划发网页, 回手动段 (原 GATE_SHADOW 分支)
以后: dock (停靠, 第 7 步) / 红点的 Detect / Select / Approach / Confirm。
"""
from vb09.flight.steps.acquire import Acquire
from vb09.flight.steps.align import Align, AlignHere
from vb09.flight.steps.commit import Commit
from vb09.flight.steps.dash import Dash
from vb09.flight.steps.plan_dash import PlanDash
from vb09.flight.steps.reposition import Reposition
from vb09.flight.steps.shadow import Shadow

MODES = {
    "dash": [Acquire(), PlanDash("粗规划"), Reposition(), Align(), Commit(), Dash()],
    "shadow": [Acquire(), PlanDash("粗规划"), AlignHere(), Shadow()],
}
