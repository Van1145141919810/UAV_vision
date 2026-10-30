# -*- coding: utf-8 -*-
"""
vb09/flight/steps/plan_dash.py —— PLAN: gate_plan.plan_dash (进入方向 / 起跑点 S / 制动终点 E)
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.make_plan 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
"""

import math

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step


def make_plan(ctx, est, p_from, label):
    aim = est.get("aim") or est["c"]
    P = gp.plan_dash(aim, est["c"], est.get("n"), p_from, ctx.origin, ctx.params,
                     virtual_offset=C.GATE_VIRTUAL_OFFSET)
    if not P.ok:
        ctx.fail(f"{label}: " + "; ".join(P.errors))
    ctx.plan = P
    ctx.plan_seq += 1
    rospy.loginfo(f"[gate] {label}: {P.summary()}")
    for w in P.notes + P.warnings:
        rospy.logwarn(f"[gate]   ⚠ {w}")
    S = P.S
    if est.get("c") is not None and P.dir is not None:
        # 靶面上看: 目标 (红点/环心) 和机体中心瞄点各在环心的哪里
        r_, u_, _ = gp.entry_axes(P.dir)
        c0 = est["c"]
        t_h, t_v = gp.vdot(gp.vsub(P.aim_tool, c0), r_), gp.vdot(gp.vsub(P.aim_tool, c0), u_)
        a_h, a_v = gp.vdot(gp.vsub(P.aim, c0), r_), gp.vdot(gp.vsub(P.aim, c0), u_)
        rospy.loginfo(f"[gate]   靶面 (相对环心, 右/上 cm): 目标 ({t_h*100:+.0f},{t_v*100:+.0f})  "
                      f"机体中心瞄 ({a_h*100:+.0f},{a_v*100:+.0f})  "
                      f"针尖预计 ({(a_h+C.NEEDLE_RIGHT)*100:+.0f},{(a_v+C.NEEDLE_UP)*100:+.0f})"
                      + (f"  ★偏离目标 {P.tool_miss*100:.0f}cm" if P.aim_clamped else ""))
    rospy.loginfo(f"[gate]   起跑点 S=({S[0]:+.2f},{S[1]:+.2f},{S[2]:+.2f})  "
                  f"制动终点 E=({P.E[0]:+.2f},{P.E[1]:+.2f},{P.E[2]:+.2f})  "
                  f"进入方向 {math.degrees(gp.yaw_of(P.dir)):+.0f}°")
    return P


class PlanDash(Step):
    """用当前 Lock 规划一次 (原 phase_gate 里的 run.make_plan(run.acq, run.pos(), "粗规划"))。"""
    name = "wait"            # 规划在识别段里做, 不换段名
    vel_lim = "pre"

    def __init__(self, label="粗规划"):
        self.label = label

    def run(self, ctx, lock):
        make_plan(ctx, lock, ctx.pos(), self.label)
        return lock
