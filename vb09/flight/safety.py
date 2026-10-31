# -*- coding: utf-8 -*-
"""
vb09/flight/safety.py —— 安全监控线程 (FATAL 停桨 / LAND 缓降判据, 去抖)
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import math

import numpy as np
import rospy
from vb09.flight import config as C

from vb09.flight.ctx import F
from vb09.flight.ctx import _f, allowed_modes, get_envelope, request_abort, request_land, telem_put, tilt_limit
from vb09.flight.devices import tilt_deg_from_quat


# =============================================================================
# ============================   Safety Monitor   =============================
# =============================================================================
class Debounce:
    """
    "连续超限"确认器 —— 安全判据去抖。

    语义: violated 必须**连续**保持 hold_s 才返回 True;
          中间任何一次 False 立即清零 (毛刺自愈)。
    这样"50ms 的 VIO 毛刺"永远够不到 hold_s, 而"真故障"会稳定累积到阈值。

    用法:
        db = Debounce(0.25)
        if db.check(cond, now):
            ...升级...
    """

    def __init__(self, hold_s):
        self.hold = float(hold_s)
        self._since = None
        self.n = 0            # 当前连续超限次数 (诊断用)

    def check(self, violated, now):
        if not violated:
            self._since = None
            self.n = 0
            return False
        self.n += 1
        if self._since is None:
            self._since = now
            return False
        return (now - self._since).to_sec() >= self.hold


def safety_monitor():
    """
    判据分级 + 去抖。

    FATAL (停桨): 姿态失控 (tilt) / MAVROS 断链 —— "根本没法再控制"的定义。
    LAND (缓降):  围栏越界 (水平/高度, FENCE_VIOLATION_LANDS) / VIO 老化 / VIO 跳变 /
                 视觉断供 / 速度超限 / 电压低 / 模式被改 (同时记 takeover)。

    围栏和倾角要求连续超限 FATAL_CONFIRM_S 才升级, 一次 VIO 毛刺够不到。
    MAVROS 断链不去抖: 断了就是断了。
    """
    rate = rospy.Rate(C.MONITOR_HZ)
    deb_height = Debounce(C.FATAL_CONFIRM_S)
    deb_horiz = Debounce(C.FATAL_CONFIRM_S)
    deb_tilt = Debounce(C.FATAL_CONFIRM_S)
    deb_tilt_dash = Debounce(C.GATE_DASH_TILT_CONFIRM_S)
    warned = {}

    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        state, pose, pose_time, vel, batt, _, _, zf = F.tele.snap()
        now = rospy.Time.now()
        horiz_lim, vel_lim = get_envelope(F.phase_ref[0])

        # ---------- FATAL ----------
        if state is not None and not state.connected:
            request_abort("MAVROS lost FCU connection")
            break

        if pose is not None:
            z_use = zf if zf is not None else pose.pose.position.z
            dz = z_use - F.INITIAL_Z

            dx = pose.pose.position.x - F.INITIAL_X
            dy = pose.pose.position.y - F.INITIAL_Y
            horiz = float(np.hypot(dx, dy))

            tilt = tilt_deg_from_quat(pose.pose.orientation)

            # 高度围栏 (去抖后 → LAND)
            if deb_height.check(dz > C.MAX_HEIGHT, now):
                msg = (f"height {dz:.2f}m > {C.MAX_HEIGHT}m "
                       f"(持续{C.FATAL_CONFIRM_S:.2f}s)")
                if C.FENCE_VIOLATION_LANDS:
                    request_land(msg)
                else:
                    request_abort(msg)
                    break

            # 水平围栏 (去抖后 → LAND)
            if deb_horiz.check(horiz > horiz_lim, now):
                msg = (f"horiz drift {horiz:.2f}m > {horiz_lim:.2f}m "
                       f"(phase={F.phase_ref[0]}, 持续{C.FATAL_CONFIRM_S:.2f}s)")
                if C.FENCE_VIOLATION_LANDS:
                    request_land(msg)
                else:
                    request_abort(msg)
                    break

            # 姿态 —— 始终 FATAL (缓降在这种情况下也执行不了)。
            # 冲刺段 (按 G 之后) 换成 GATE_DASH_TILT_DEG: 急刹 + 撞纸的短时大倾角不该停桨
            t_lim, t_hold = tilt_limit()
            db_on, db_off = (deb_tilt_dash, deb_tilt) if F.dash_tilt_on[0] else (deb_tilt, deb_tilt_dash)
            db_off.check(False, now)
            if db_on.check(tilt > t_lim, now):
                request_abort(f"tilt {tilt:.1f}deg > {t_lim:.0f}deg "
                              f"(持续{t_hold:.2f}s{', 冲刺段' if F.dash_tilt_on[0] else ''})")
                break

            # 观察期提示: 第一次越界打一行, 恢复再打一行。
            # 这段日志很重要 —— 去抖之后"擦线但恢复"不会再留下任何痕迹,
            # 不打印的话你永远不知道围栏其实被碰过。
            for nm, db, cur in (("高度", deb_height, dz),
                                ("水平", deb_horiz, horiz),
                                ("姿态", db_on, tilt)):
                if db.n == 1 and not warned.get(nm):
                    rospy.logwarn(f"[safety] {nm}越界 ({cur:.2f}) — 进入 "
                                  f"{(t_hold if nm == '姿态' else C.FATAL_CONFIRM_S):.2f}s 观察期, 恢复则忽略")
                    warned[nm] = True
                elif db.n == 0 and warned.get(nm):
                    rospy.loginfo(f"[safety] {nm}已回到带内 — 观察期解除")
                    warned[nm] = False

        # ---------- LAND ----------
        if pose_time is not None:
            age = (now - pose_time).to_sec()
            if age > C.POSE_STALE_SEC:
                request_land(f"VIO stale {age:.2f}s")

        # 视觉源断供 (见 VISION_LOST_LAND_S 注释)
        # 上面那条看的是 EKF2 的输出, VINS 死了它也不会 stale;
        # 这条看的是 VINS 自己还发不发 odometry —— 这才是"视觉真的没了"。
        # 已解锁才判: 没解锁时视觉没起来无所谓, preflight 已经报过。
        if C.VISION_LOST_ENABLE and state is not None and state.armed:
            _, odom_time = F.tele.snap_vins()
            if odom_time is not None:
                odom_age = (now - odom_time).to_sec()
                if odom_age > C.VISION_LOST_LAND_S:
                    request_land(
                        f"视觉断供 {odom_age:.2f}s (VINS 已停止输出 odometry) — "
                        f"无外部位置观测, 位置环会追惯性漂移, 主动缓降")

        jump = F.tele.take_jump_max()
        if jump > C.POSE_JUMP_MAX:
            request_land(f"VIO jump {jump*100:.0f}cm")

        if vel is not None:
            v = vel.twist.linear
            horiz_v = math.hypot(v.x, v.y)
            if horiz_v > vel_lim:
                request_land(f"horiz vel {horiz_v:.2f}m/s > {vel_lim:.2f}m/s "
                             f"(phase={F.phase_ref[0]})")

        if batt is not None and batt.voltage > 0:
            fly_min = C.CELL_MIN_FLY * C.cell_count[0]
            if batt.voltage < fly_min:
                request_land(f"battery {batt.voltage:.2f}V < {fly_min:.2f}V")

        if (state is not None and state.armed
                and state.mode not in allowed_modes()):
            # 模式被改: 可能是飞手切到 POSCTL 救场, 也可能是 PX4 失效保护。
            # 别停桨，别抢控制权: 记下接管, 各阶段看到 takeover 就停手, 交给
            # phase_handover。脚本自己切到 AUTO.LAND 之后飞手再切走, 同样算接管。
            if F.takeover[0] is None:
                F.takeover[0] = state.mode
                rospy.logerr(f"*** 控制权被接管: 飞控模式 → {state.mode} "
                             f"(不是脚本切的) — 脚本不再切模式、不再停桨 ***")
            request_land(f"mode changed to {state.mode} — 控制权被接管")

        # 网页"安全余量"面板。只读上面已经拿到的快照, 自己重算一遍显示值,
        # 不碰任何判定逻辑; 整段 try, 出错只丢这一帧遥测。
        try:
            _t = {"horiz_lim": _f(horiz_lim), "vel_lim": _f(vel_lim),
                  "max_height": _f(C.MAX_HEIGHT), "max_tilt": _f(tilt_limit()[0]),
                  "max_tilt_base": _f(C.MAX_TILT_DEG), "max_tilt_dash": _f(C.GATE_DASH_TILT_DEG),
                  "dash_tilt": bool(F.dash_tilt_on[0]),
                  "pose_stale": _f(C.POSE_STALE_SEC), "vision_lost": _f(C.VISION_LOST_LAND_S),
                  "jump_max": _f(C.POSE_JUMP_MAX), "jump": _f(jump),
                  "deb": {"height": deb_height.n, "horiz": deb_horiz.n,
                          "tilt": (deb_tilt_dash if F.dash_tilt_on[0] else deb_tilt).n},
                  "confirm_s": _f(tilt_limit()[1])}
            if pose is not None:
                _zu = zf if zf is not None else pose.pose.position.z
                _t["dz"] = _f(_zu - F.INITIAL_Z)
                _t["horiz"] = _f(math.hypot(pose.pose.position.x - F.INITIAL_X,
                                            pose.pose.position.y - F.INITIAL_Y))
                _t["tilt"] = _f(tilt_deg_from_quat(pose.pose.orientation))
            if pose_time is not None:
                _t["pose_age"] = _f((now - pose_time).to_sec())
            _, _ot = F.tele.snap_vins()
            if _ot is not None:
                _t["odom_age"] = _f((now - _ot).to_sec())
            if vel is not None:
                _t["horiz_v"] = _f(math.hypot(vel.twist.linear.x, vel.twist.linear.y))
            if batt is not None and batt.voltage > 0:
                _t["batt_v"] = _f(batt.voltage)
                _t["fly_min_v"] = _f(C.CELL_MIN_FLY * C.cell_count[0])
            telem_put("safety", _t)
        except Exception:
            pass

        rate.sleep()
