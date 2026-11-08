# -*- coding: utf-8 -*-
"""
vb09/flight/modes.py —— 任务 = 步骤列表 (显式字典, 不扫描目录)
===============================================================================
run_mission(ctx, steps_for(mode, tracked)) 依次执行; 放弃 / 回手动段 / 收尾都在 runner 里, 和步骤无关。
  dash    冲刺穿环 (原 phase_gate 的 else 分支)
  shadow  影子: 识别 / 锁定 / 规划照做, 不摆位不冲刺, 计划发网页, 回手动段 (原 GATE_SHADOW 分支)
  dock    停靠: 识别 → 飞到目标前 GATE_DOCK_D 停住对正 → (上下扫描 ±GATE_VSCAN_DZ 定法向) → 悬停 GATE_DOCK_HOLD_S
          记录检测 → 回手动段 (第 7 步; 上下扫描 2026-11-05)

MODES_TRACKED (2026-11-08, claude/PLAN_dot_v2.md): 判定插件声明 TRACKED = True 的目标 (红点) 用这一套 ——
桥里一直有诚实的世界系后验, 步骤每拍读最新值 (边飞边估), 不再"悬停取样 → 冻结"。
  dock    Detect (发现 + 选点) → ApproachDock (抵近到 dock_dist) → Refine (够准就锁 / 换视角) → Hold
  dash    Detect → ApproachDash (抵近到最小起跑距离) → Refine → PlanLocked → CommitTrack (按 G 时变了就重锁) → Dash
  shadow  Detect → PlanDash → AlignHere → Shadow (原地, 不抵近)
mode_spec.MODE_SPECS 的 steps / steps_tracked 要和这里一致 (tools/mode_isolation_check.py 查)。
"""
from vb09.flight.steps.common import Acquire
from vb09.flight.steps.dash import Align, Commit, Dash, PlanDash, Reposition
from vb09.flight.steps.dock import Hold, StandOff, VScan
from vb09.flight.steps.shadow import AlignHere, Shadow
from vb09.flight.steps.track import ApproachDash, ApproachDock, CommitTrack, Detect, PlanLocked, Refine

MODES = {
    "dash": [Acquire(), PlanDash("粗规划"), Reposition(), Align(), Commit(), Dash()],
    "shadow": [Acquire(), PlanDash("粗规划"), AlignHere(), Shadow()],
    "dock": [Acquire(), StandOff(), VScan(), Hold()],     # VScan: GATE_VSCAN_DZ = 0 时什么都不做
}

MODES_TRACKED = {
    "dash": [Detect(), ApproachDash(), Refine(), PlanLocked(), CommitTrack(), Dash()],
    "shadow": [Detect(), PlanDash("粗规划"), AlignHere(), Shadow()],
    "dock": [Detect(), ApproachDock(), Refine(), Hold()],
}


def steps_for(mode, tracked=False):
    """模式 + 目标是不是跟踪类 → 步骤列表。"""
    return (MODES_TRACKED if tracked else MODES)[mode]
