# -*- coding: utf-8 -*-
"""
vb09/flight/steps/shadow.py —— 影子档的收尾: 锁定后规划 → 计划发网页 → 原地悬停 3s → 回手动段
===============================================================================
2026-10-30 第 4.4 步: 原 phase_gate 里 `if GATE_SHADOW:` 分支的后半段, 原样搬来。
"""
import rospy

from vb09.flight.steps.base import Step
from vb09.flight.steps.plan_dash import make_plan


class Shadow(Step):
    name = "hold"
    vel_lim = "pre"

    def run(self, ctx, lock):
        make_plan(ctx, ctx.locked, ctx.pos(), "锁定后规划 (影子)")
        ctx.set_seg("hold", "影子模式: 不摆位不冲刺, 计划已发网页")
        t_end = rospy.Time.now().to_sec() + 3.0
        hp = ctx.pos()
        while rospy.Time.now().to_sec() < t_end:
            ctx.hold(hp)
            ctx.tick()
        ctx.outcome = "shadow"
        ctx.again("影子模式做完了 (计划在网页上) —— 换个位置 T 再来, 或 L 降落")
        return lock
