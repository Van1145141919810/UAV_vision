# -*- coding: utf-8 -*-
"""
vb09/flight/steps/commit.py —— COMMIT: 打印计划, 等操作员按 G (降速时只认 Y), 再停稳在起跑点
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.commit 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
"""

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step


class Commit(Step):
    name = "commit"
    vel_lim = "pre"

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
            ctx.hold(P.S)
            ctx.tick()
            if go_key in ctx.flags:
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
