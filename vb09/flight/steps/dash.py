# -*- coding: utf-8 -*-
"""
vb09/flight/steps/dash.py —— DASH / BRAKE: 按时间表执行冲刺轨迹 (位置+速度+加速度前馈), 卡纸检测
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.dash 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
"""

import math

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step


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
