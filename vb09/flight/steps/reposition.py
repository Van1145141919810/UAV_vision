# -*- coding: utf-8 -*-
"""
vb09/flight/steps/reposition.py —— REPOSITION: 直线慢飞到起跑点 S, 再转头对正进入方向
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.move_to / turn_to 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
"""

import math

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.telemetry import wrap_pi


def move_to(ctx, S, yaw_mode, timeout, label):
    """
    setpoint 以 GATE_REPO_V 斜坡走直线到 S (3D, 带牵引绳)。yaw_mode:
      "ring" = 机头盯环心 (保持环在视野里)   数值 = 固定朝向
    全程查"飞机在环前 ≥1.5m" (环系 x ≤ REPO_X_MAX)。
    """
    ctx.set_seg("align", label)
    P = ctx.plan
    fwd, _ = gp._ring_frame(P.dir, (ctx.used() or {}).get("n"))
    sp = ctx.pos()
    t_end = rospy.Time.now().to_sec() + timeout
    reach = 0
    need = max(1, int(round(0.5 * C.GATE_LOOP_HZ)))
    dt = 0.0
    while True:
        now = rospy.Time.now().to_sec()
        p = ctx.pos()
        d = gp.vsub(S, sp)
        dn = gp.vnorm(d)
        step = C.GATE_REPO_V * dt
        sp = S if dn <= step else gp.vadd(sp, gp.vmul(d, step / max(dn, 1e-9)))
        lead = gp.vsub(sp, p)
        ln = gp.vnorm(lead)
        if ln > C.GATE_REPO_LEASH:
            sp = gp.vadd(p, gp.vmul(lead, C.GATE_REPO_LEASH / ln))
        if yaw_mode == "ring":
            c = (ctx.used() or {}).get("c") or P.aim
            if math.hypot(c[0] - p[0], c[1] - p[1]) > 0.5:
                ctx.yaw_step(math.atan2(c[1] - p[1], c[0] - p[0]), dt)
        else:
            ctx.yaw_step(yaw_mode, dt)
        vff = gp.vmul(d, C.GATE_REPO_V / dn) if dn > step and ln <= C.GATE_REPO_LEASH else (0.0, 0.0, 0.0)
        ctx.setpoint.set_zhold_mode(sp[0], sp[1], sp[2], ctx.yaw_sp, vff[0], vff[1])
        x_ring = gp.vdot(gp.vsub(p, P.aim), fwd)
        if x_ring > C.REPO_X_MAX + 0.3:
            ctx.fail(f"摆位中离环平面只剩 {-x_ring:.2f}m (< {-C.REPO_X_MAX:.1f}m)")
        err = gp.vnorm(gp.vsub(S, p))
        reach = reach + 1 if (err < C.GATE_REPO_TOL and sp == S) else 0
        if reach >= need:
            return
        if now > t_end:
            ctx.fail(f"{label}: {timeout:.0f}s 没到位 (还差 {err:.2f}m)")
        if now - ctx.t_log > 1.0:
            ctx.t_log = now
            rospy.loginfo(f"    [{label}] 离起跑点 {err:.2f}m  离环平面 {-x_ring:.2f}m")
        dt = ctx.tick()


def turn_to(ctx, yaw_target, hold_p, label, timeout=15.0):
    t_end = rospy.Time.now().to_sec() + timeout
    dt = 0.0
    while True:
        err = ctx.yaw_step(yaw_target, dt)
        ctx.hold(hold_p)
        if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) and \
                abs(wrap_pi(yaw_target - ctx.yaw_sp)) < 1e-3:
            return
        if rospy.Time.now().to_sec() > t_end:
            ctx.fail(f"{label}: 转头 {timeout:.0f}s 没到位 (差 {math.degrees(err):+.0f}°)")
        dt = ctx.tick()


class Reposition(Step):
    """原 phase_gate: move_to(P.S, "ring", GATE_REPO_TIMEOUT, "摆位") + turn_to(yaw_of(P.dir), P.S, "对正进入方向")。"""
    name = "align"           # move_to 设的段名就是 align (遥测不变)
    vel_lim = "pre"

    def run(self, ctx, lock):
        P = ctx.plan
        move_to(ctx, P.S, "ring", C.GATE_REPO_TIMEOUT, "摆位")
        turn_to(ctx, gp.yaw_of(P.dir), P.S, "对正进入方向")
        return lock
