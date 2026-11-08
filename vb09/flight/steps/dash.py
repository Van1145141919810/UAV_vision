# -*- coding: utf-8 -*-
"""
vb09/flight/steps/dash.py —— 冲刺穿环的步骤 (MODES["dash"])
===============================================================================
2026-11-01 由 plan_dash.py / reposition.py / align.py / commit.py / dash.py 合并 (代码原样)。
  make_plan / PlanDash   gate_plan.plan_dash + 不可行时降速
  Reposition             摆位到起跑点 S + 对正进入方向
  Align                  在 S 取样锁定 → 锁定后规划 → 起跑点挪了就微调
  Commit                 等 G / Y, 停稳
  Dash                   按时间表冲刺 + 制动 + 卡纸处置 (committed = True)
"""
import math
import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.steps.common import align_lock, move_to, turn_to


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


class Reposition(Step):
    """原 phase_gate: move_to(P.S, "ring", GATE_REPO_TIMEOUT, "摆位") + turn_to(yaw_of(P.dir), P.S, "对正进入方向")。"""
    name = "align"           # move_to 设的段名就是 align (遥测不变)
    vel_lim = "pre"

    def run(self, ctx, lock):
        P = ctx.plan
        move_to(ctx, P.S, "ring", C.GATE_REPO_TIMEOUT, "摆位")
        turn_to(ctx, gp.yaw_of(P.dir), P.S, "对正进入方向")
        return lock


class Align(Step):
    """
    冲刺档 (原 phase_gate 的 else 分支):
      align(P.S) → make_plan(locked, pos, "锁定后规划") → 起跑点挪了 > GATE_START_TOL 就微调过去
    """
    name = "align"
    vel_lim = "pre"

    def run(self, ctx, lock):
        lock = align_lock(ctx, ctx.plan.S)
        P2 = make_plan(ctx, ctx.locked, ctx.pos(), "锁定后规划")
        gap = gp.vnorm(gp.vsub(P2.S, ctx.pos()))
        if gap > C.GATE_START_TOL:
            if gap > 0.5:
                rospy.logwarn(f"[gate] 锁定后起跑点挪了 {gap:.2f}m —— 再挪过去 (不重新取样)")
            move_to(ctx, P2.S, gp.yaw_of(P2.dir), C.GATE_REPO_TIMEOUT, "微调起跑点")
        return lock


class Commit(Step):
    name = "commit"
    vel_lim = "pre"

    def on_go(self, ctx):
        """按了 G / Y (或不用确认) 之后、冲刺之前的最后检查。返回 False = 这次不算, 再等一次确认 (计划可能已经换了,
        循环里按 ctx.plan 读)。环: 总是 True (行为不变)。跟踪类目标: track.CommitTrack 按最新估计重锁。"""
        return True

    def run(self, ctx, lock):
        P = ctx.plan
        ctx.set_seg("commit", "等确认")
        rospy.logwarn("=" * 64)
        rospy.logwarn(f"[gate] 计划: {P.summary()}")
        if P.v < C.DASH_V - 1e-9:
            rospy.logwarn(f"[gate] ★ 冲刺速度被场地压到 {P.v:.2f} m/s (设定 {C.DASH_V:.2f}) —— 原因见上面"
                          f"'场地限制'那行; 不接受就按 M 回手动段")
        if C.GATE_VIRTUAL_OFFSET > 0:
            rospy.logwarn(f"[gate] ★ 虚拟环: 会停在真纸面前约 {C.GATE_VIRTUAL_OFFSET - P.s_stop:.2f}m")
        need_y = bool(P.need_confirm)     # 降速了 → 只认 Y, G 不算
        go_key = 'y' if need_y else 'g'
        if need_y:
            rospy.logerr(f"[gate] ★★★ 降速: 计划 {P.v:.2f} m/s, 设定 {C.DASH_V:.2f} m/s —— 按 {C.DASH_V:.2f} 卡在: {P.slow_why}")
            rospy.logerr(f"[gate] ★★★ 接受 {P.v:.2f} m/s 就按 **Y** 冲刺 (G 不算)  |  不接受按 M 回手动段, 挪位置后 T 重来")
        if C.GATE_CONFIRM_DASH or need_y:
            if ctx.keys is None:
                ctx.fail("要按键确认冲刺, 但没有终端键盘")
            back = "回手动段" if (C.GATE_ABORT_TO_MANUAL and ctx.back_ok) else "降落"
            rospy.logwarn(f"[gate] ★★ 按 G 冲刺 (之后不能中止, 只有 X 停桨)  |  M / 方向键 = 不冲了, 回手动段  "
                          f"|  L = 降落  |  {C.GATE_CONFIRM_TIMEOUT:.0f}s 不按 = {back}")
        rospy.logwarn("=" * 64)
        t_end = rospy.Time.now().to_sec() + C.GATE_CONFIRM_TIMEOUT
        while C.GATE_CONFIRM_DASH or need_y:
            P = ctx.plan
            ctx.hold(P.S)
            ctx.tick()
            if go_key in ctx.flags:
                if not self.on_go(ctx):
                    t_end = rospy.Time.now().to_sec() + C.GATE_CONFIRM_TIMEOUT
                    P = ctx.plan
                    need_y = need_y or bool(P.need_confirm)
                    go_key = 'y' if need_y else 'g'
                    rospy.logwarn(f"[gate] ★★ 计划已更新: {P.summary()} —— 再按 {go_key.upper()} 冲刺 | M = 回手动段")
                    continue
                rospy.logwarn(f"[gate] {go_key.upper()} → 冲刺 {P.v:.2f} m/s")
                break
            if need_y and 'g' in ctx.flags:
                rospy.logwarn(f"[gate] 这次降速了 ({P.v:.2f} < {C.DASH_V:.2f} m/s), G 不算 —— 接受就按 Y, 不接受按 M")
            now = rospy.Time.now().to_sec()
            if now > t_end:
                ctx.fail(f"{C.GATE_CONFIRM_TIMEOUT:.0f}s 没按 {go_key.upper()}")
            if now - ctx.t_log > 5.0:
                ctx.t_log = now
                lv = ctx.gate_in.latest_valid()
                if lv is not None and ctx.locked:
                    d = gp.vnorm(gp.vsub(lv["c"], ctx.locked["c"])) * 100
                    rospy.loginfo(f"    [commit] 等 G … 最新识别和锁定值差 {d:.0f}cm")
        if not (C.GATE_CONFIRM_DASH or need_y):
            self.on_go(ctx)                       # 不用确认: 照样做最后检查 (跟踪类目标可能重锁), 结果不拦冲刺
        P = ctx.plan
        # 停稳在起跑点
        t_end = rospy.Time.now().to_sec() + C.GATE_START_WAIT
        while True:
            p, v = ctx.pos(), ctx.vel()
            e = gp.vnorm(gp.vsub(p, P.S))
            if e < C.GATE_START_TOL and gp.vnorm(v) < C.GATE_START_V_MAX:
                return lock
            if rospy.Time.now().to_sec() > t_end:
                ctx.fail(f"起跑前没停稳: 离 S {e:.2f}m, 速度 {gp.vnorm(v):.2f}m/s")
            ctx.hold(P.S)
            ctx.tick()


class Dash(Step):
    """
    committed = True: ctx.enter 放开速度包络到冲刺档、关掉回手动段、停桨线换成 GATE_DASH_TILT_DEG;
    ctx.tick 推迟 LAND 到刹停后。
    """
    name = "dash"
    committed = True

    def run(self, ctx, lock):
        P = ctx.plan
        tr = P.traj
        aim, d = P.aim, P.dir
        yaw_d = gp.yaw_of(d)
        ctx.yaw_sp = yaw_d

        def sampler(t):
            s, v, a = tr.at(t)
            return gp.vadd(aim, gp.vmul(d, s)), gp.vmul(d, v), gp.vmul(d, a)

        # 速度包络放开到冲刺档 / 不能再回手动段 / 停桨线换成 GATE_DASH_TILT_DEG:
        #   本步骤 committed = True, 由 ctx.enter(step) 设 (平台白名单属性, 步骤只声明)
        ctx.t0 = rospy.Time.now().to_sec() + 0.05
        ctx.setpoint.set_traj_mode(sampler, ctx.t0, yaw_d, C.GATE_ACC_FF)
        ctx.set_seg("dash", f"t0, {P.v:.2f}m/s, 预计 {tr.T:.2f}s 后停在 s={P.s_stop:+.2f}")
        stuck_since = None
        t_wait_end = tr.T + 3.0 * C.T_LAG
        warned_dev = False
        while True:
            now = rospy.Time.now().to_sec()
            t = now - ctx.t0
            s_p, v_p, _ = tr.at(t)
            p, v = ctx.pos(), ctx.vel()
            s_e = gp.vdot(gp.vsub(p, aim), d)
            v_e = gp.vdot(v, d)
            ctx.dash_state = (t, s_p, v_p, s_e, v_e)
            if t >= tr.t2 and ctx.seg == "dash":
                ctx.set_seg("brake", f"s_plan={s_p:+.2f} s_est={s_e:+.2f} v={v_e:.2f}")
            if not ctx.passed and s_e > 0.0:
                ctx.passed = True
                rospy.logwarn(f"[gate] 过纸面 (估计) v={v_e:.2f}m/s  t={t:.2f}s")
            if abs(s_e - s_p) > C.GATE_DEV_WARN and not warned_dev:
                warned_dev = True
                rospy.logwarn(f"[gate] 估计 s 和规划差 {s_e - s_p:+.2f}m (只记录, 不改时间表)")
            # 卡纸: 机尾还没过纸面就被顶住。两种样子:
            #   ① 还在冲: 实测比规划慢 > DV, 且落后 > DS
            #   ② 规划已经慢下来 (刹车段后半): 落后 > DS 且实测几乎停了
            lag_s = s_p - s_e
            if C.GATE_STUCK_ENABLE and t > 0.3 and s_e < C.L_R and lag_s > C.GATE_STUCK_DS and \
                    (v_p - v_e > C.GATE_STUCK_DV or v_e < C.GATE_STOP_V):
                stuck_since = stuck_since if stuck_since is not None else now
                if now - stuck_since >= C.GATE_STUCK_HOLD:
                    ctx.lvl = 1
                    ctx.outcome = "stuck"
                    ctx.hold(p)
                    rospy.logerr(f"[gate] ★ 卡纸: 规划 s={s_p:+.2f} v={v_p:.2f} / 实测 s={s_e:+.2f} "
                                 f"v={v_e:.2f} —— 停止前推, 设定点 = 当前位置")
                    return lock
            else:
                stuck_since = None
            if t >= t_wait_end and math.hypot(v[0], v[1]) < C.GATE_STOP_V:
                break
            if t >= tr.T + C.GATE_STOP_EXTRA:
                rospy.logwarn(f"[gate] 轨迹结束 {C.GATE_STOP_EXTRA:.1f}s 还没停稳 "
                              f"(v={math.hypot(v[0], v[1]):.2f}m/s) —— 照样转悬停")
                break
            ctx.tick()                          # committed: LAND 推迟到刹停后
        ctx.outcome = ctx.outcome or "done"
        rospy.logwarn(f"[gate] 刹停: 规划停在 s={P.s_stop:+.2f}, 估计 s={s_e:+.2f} "
                      f"(差 {s_e - P.s_stop:+.2f}m)  过纸面={'是' if ctx.passed else '否'}")
        return lock
