# -*- coding: utf-8 -*-
"""
vb09/flight/steps/shadow.py —— 影子模式的步骤 (MODES["shadow"])
===============================================================================
2026-11-01 由 align.py (AlignHere) / shadow.py 合并 (代码原样)。
  AlignHere   原地 (当前位置) 取样锁定, 不摆位
  Shadow      锁定后规划 → 计划发网页 → 原地悬停 3s → 回手动段
"""
import rospy
from vb09.flight.steps.base import Step
from vb09.flight.steps.common import align_lock
from vb09.flight.steps.dash import make_plan


class AlignHere(Step):
    """影子档: 原地 (当前位置) 取样锁定, 不摆位。原 phase_gate: run.align(run.pos())。"""
    name = "align"
    vel_lim = "pre"

    def run(self, ctx, lock):
        return align_lock(ctx, ctx.pos())


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
