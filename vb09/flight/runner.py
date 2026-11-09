# -*- coding: utf-8 -*-
"""
vb09/flight/runner.py —— 飞行主序列 main(): 订阅 / 发布 / 服务 → 预检 → prime → OFFBOARD → 解锁 → 起飞 → 任务 → 降落 → cleanup
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import signal
import threading
import time
import traceback

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import EstimatorStatus
from mavros_msgs.msg import StatusText
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import ParamGet
from mavros_msgs.srv import SetMode
try:
    from mavros_msgs.srv import ParamPull
except ImportError:                       # 很老的 mavros_msgs
    ParamPull = None
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from vb09.flight import config as C

from vb09.flight.ctx import F
from vb09.flight.ctx import SEV_FATAL, SEV_LAND, check_abort, set_phase, statustext_cb
from vb09.flight.log import drain_gate_queue, drain_timing_queue, ekf_logger_loop, fsync_all_logs, fsync_loop, gate_logger_loop, init_log_dir, last_fsync, last_zdrop_trigger, main_logger_loop, publish_config, publisher_loop, status_loop, telemetry_loop, timing_logger_loop, vins_logger_loop
from vb09.flight.mission import phase_gate
from vb09.flight.phases import arm_with_confirm, cleanup_decision, emergency_land, force_disarm_with_timeout, on_ground, phase_auto_land, phase_descend, phase_handover, phase_takeoff, pose_cb, restore_ground_mode, single_instance_guard
from vb09.flight.preflight import preflight, print_config_summary
from vb09.flight.safety import safety_monitor


# =============================================================================
# =================================   Main   ==================================
# =============================================================================
def main():
    single_instance_guard()
    # disable_signals=True: Ctrl+C 不再让 rospy 先把 ROS 关掉, 而是在主线程抛
    # KeyboardInterrupt → 走下面的 except/finally, 这时服务还能调 (切 AUTO.LAND /
    # 还原模式才发得出去)。进程退出前在 __main__ 里 signal_shutdown。
    rospy.init_node(C.NODE_NAME, disable_signals=True)
    # disable_signals 同时关掉了 rospy 对 SIGTERM 的处理; 让 SIGTERM (kill) 和 Ctrl+C
    # 走同一条路, 否则进程会被直接杀掉、cleanup 一行都不执行。
    def _sigterm(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)
    init_log_dir()

    rospy.Subscriber("/mavros/state", State, F.tele.set_state)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local",
                     TwistStamped, F.tele.set_velocity)
    rospy.Subscriber("/mavros/battery", BatteryState, F.tele.set_battery)
    rospy.Subscriber("/mavros/target_actuator_control",
                     ActuatorControl, F.tele.set_actuator)
    rospy.Subscriber("/mavros/setpoint_raw/target_local",
                     PositionTarget, F.tele.set_target_local)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, F.tele.set_vins_odom)
    rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                     F.tele.set_estimator_status)
    rospy.Subscriber("/mavros/extended_state", ExtendedState,
                     F.tele.set_extended_state)
    rospy.Subscriber(C.GATE_STATUS_TOPIC, String, F.gate_in.on_status, queue_size=5)
    rospy.Subscriber("/flight_ctrl/select", String, F.gate_in.on_web_select, queue_size=5)   # 网页选点 (2026-11-08)
    F.gate_in.init_select()            # /gate/select 发布器先连上; 跟踪类目标清掉桥里上一次的选点 (2026-11-09)
    rospy.Subscriber(C.CAM_STATUS_TOPIC, String, F.gate_in.on_cam, queue_size=1)
    rospy.Subscriber("/mavros/statustext/recv", StatusText, statustext_cb, queue_size=50)

    pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget,
                          queue_size=10)
    status_pub = rospy.Publisher(C.STATUS_TOPIC, String, queue_size=1)
    # 网页监控。遥测线程最先起来 —— preflight 阶段网页就能看到状态。
    telem_pub = rospy.Publisher(C.TELEMETRY_TOPIC, String, queue_size=1)
    config_pub = rospy.Publisher(C.CONFIG_TOPIC, String, queue_size=1, latch=True)
    threading.Thread(target=telemetry_loop, args=(telem_pub,), daemon=True).start()
    publish_config(config_pub)

    rospy.loginfo("Waiting for services...")
    for name in ["/mavros/set_mode", "/mavros/cmd/arming",
                 "/mavros/cmd/command", "/mavros/param/get"]:
        rospy.wait_for_service(name, timeout=10.0)
    set_mode  = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arm_srv   = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    cmd_long  = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
    param_get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
    param_pull = None
    if ParamPull is not None:
        try:
            rospy.wait_for_service("/mavros/param/pull", timeout=2.0)
            param_pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
        except Exception as e:
            rospy.logwarn(f"/mavros/param/pull 不可用 ({e}) —— 参数缓存没就绪时只能拒绝起飞")

    last_zdrop_trigger[0] = rospy.Time(0)
    last_fsync[0] = rospy.Time.now()

    F.phase_ref[0] = "preflight"          # 网页显示用 (safety_monitor 此时还没起)
    if not preflight(param_get, set_mode, param_pull):
        F.phase_ref[0] = "preflight_failed"
        rospy.sleep(0.5)                # 让网页来得及看到"preflight 失败"
        return 1

    publish_config(config_pub)          # 原点已确定, 再发一次
    print_config_summary()

    F.setpoint.set_velocity_mode(0.0, 0.0, 0.0, 0.0)

    rospy.loginfo("=" * 64)
    rospy.loginfo(f"Preflight OK. Log dir: {F.log_dir}")
    rospy.loginfo("2s 后开始。Ctrl+C 中止。")
    rospy.loginfo("=" * 64)
    rospy.sleep(2.0)

    threads = [
        threading.Thread(target=publisher_loop, args=(pub,), daemon=True),
        threading.Thread(target=status_loop, args=(status_pub,), daemon=True),
        threading.Thread(target=safety_monitor, daemon=True),
        threading.Thread(target=main_logger_loop, daemon=True),
        threading.Thread(target=vins_logger_loop, daemon=True),
        threading.Thread(target=ekf_logger_loop, daemon=True),
        threading.Thread(target=timing_logger_loop, daemon=True),
        threading.Thread(target=gate_logger_loop, daemon=True),
        threading.Thread(target=fsync_loop, daemon=True),
    ]
    for t in threads:
        t.start()

    entered_offboard = False

    try:
        set_phase("prime")
        rospy.loginfo(f"[prime] flooding setpoint ({C.PRIME_SECONDS}s @ "
                      f"{C.SP_PUB_HZ}Hz = {int(C.PRIME_SECONDS*C.SP_PUB_HZ)} 条)")
        rospy.sleep(C.PRIME_SECONDS)

        set_phase("offboard_switch")
        rospy.loginfo("[offboard] switching")
        r = set_mode(custom_mode="OFFBOARD")
        if not r.mode_sent:
            raise RuntimeError("SetMode OFFBOARD not sent")
        t_end = rospy.Time.now() + rospy.Duration(C.OFFBOARD_TIMEOUT)
        while rospy.Time.now() < t_end:
            check_abort()
            if F.tele.state and F.tele.state.mode == "OFFBOARD":
                entered_offboard = True
                break
            rospy.sleep(0.05)
        if not entered_offboard:
            cur = F.tele.state.mode if F.tele.state else "?"
            raise RuntimeError(f"OFFBOARD not entered, mode={cur}")
        rospy.loginfo("[offboard] OK")

        set_phase("arm")
        arm_with_confirm(arm_srv)
        rospy.logwarn("*** ARMED ***")

        # ---------------- 飞行序列 ----------------
        z_top = phase_takeoff()
        gx, gy = F.tele.xy()
        gz, gyaw = z_top, F.INITIAL_YAW

        if not F.land_flag.is_set():
            check_abort()
            gx, gy, gz, gyaw = phase_gate(z_top)

        check_abort()

        # 穿环之后不返航: 不管成功/放弃/LAND, 都在当前位置原地降落
        if F.takeover[0] is None:
            if F.land_flag.is_set():
                rospy.logwarn(f"[main] LAND 路径: {F.land_reason[0]}")
                emergency_land(set_mode, gz, gyaw)
            else:
                phase_descend(gz, gyaw, gx, gy)
                phase_auto_land(set_mode)
        # 任何阶段 (包括上面的降落过程中) 被接管 → 只看不动
        if F.takeover[0] is not None:
            phase_handover()

    except KeyboardInterrupt:
        # disable_signals=True 后 Ctrl+C 落到这里, ROS 仍活着 → 下面 cleanup 的
        # 切 AUTO.LAND / 还原模式都能真正发出去。和普通异常同等处理: 不停桨。
        rospy.logerr("[EXCEPTION] Ctrl+C / SIGTERM")
        F.exc_reason[0] = "Ctrl+C / SIGTERM"

    except Exception as e:
        rospy.logerr(f"[EXCEPTION] {e}")
        rospy.logerr(traceback.format_exc())       # 完整堆栈 → events.log
        # 普通异常不是 FATAL (FATAL 只认 request_abort), 不停桨。
        F.exc_reason[0] = str(e)

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)     # cleanup 期间再按 Ctrl+C 不能打断它
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        fatal = F.abort_flag.is_set()          # 只有 request_abort 会置它 (在下面置位之前读)
        st = F.tele.state
        armed = bool(st and st.armed)
        mode = st.mode if st else "?"
        grounded = on_ground()
        action = cleanup_decision(fatal, F.takeover[0], armed, grounded, mode)
        # 穿环后多半落在离起飞点几米外: cleanup 不再按起飞点查水平围栏 (否则落地后还报越界)
        set_phase("cleanup", horiz=1e9, vel=1e9)

        if fatal:
            rospy.logerr(f"ABORT PATH [{SEV_FATAL}]: {F.abort_reason[0]} — motors stop NOW")
        elif F.takeover[0] is not None:
            rospy.logwarn(f"TAKEOVER PATH: 控制权在 {F.takeover[0]}")
        elif F.exc_reason[0] is not None:
            rospy.logerr(f"EXCEPTION PATH: {F.exc_reason[0]}")
        elif F.land_reason[0] is not None:
            rospy.logwarn(f"LAND PATH: {F.land_reason[0]}")
        rospy.logwarn(f"cleanup: armed={armed} 模式={mode} 在地上={grounded} → {action}")

        # 请求 AUTO.LAND 必须在停发设定点之前: 停发后 PX4 会走 OFFBOARD 丢失保护
        if action == "auto_land":
            F.voluntary_land[0] = True
            try:
                r = set_mode(custom_mode="AUTO.LAND")
                sent = bool(r.mode_sent)
            except Exception as e:
                sent = False
                rospy.logerr(f"cleanup: 切 AUTO.LAND 异常: {e}")
            t_end = time.time() + 2.0
            while time.time() < t_end and not (F.tele.state and F.tele.state.mode == "AUTO.LAND"):
                time.sleep(0.05)
            if F.tele.state and F.tele.state.mode == "AUTO.LAND":
                rospy.logwarn("cleanup: ★ 飞机仍在空中, 已交给 PX4 AUTO.LAND 自己降落 "
                              "(脚本不停桨)。盯着它, 必要时用 QGC。")
            else:
                rospy.logerr(f"cleanup: ★ 切 AUTO.LAND 未确认 (sent={sent}) —— 停发设定点后 "
                             f"PX4 会按 COM_OBL_ACT 处理 (preflight 已确认是悬停或降落)。"
                             f"立刻用 QGC 切 Land!")

        # 地上已上锁 (arm 被拒 / 解锁前出错 / 正常降落后): 停发设定点**之前**切回开局模式。
        # 否则停在 OFFBOARD 停发设定点 → 地上触发 offboard 丢失保护 → 下一次开局模式乱跳。
        if action == "none":
            try:
                restore_ground_mode(set_mode, grounded)
            except Exception as e:
                rospy.logwarn(f"cleanup: 还原模式异常 (忽略): {e}")

        F.abort_flag.set()                     # 停 setpoint 发布 / 监控 / 日志线程
        time.sleep(0.1)                      # 不用 rospy.sleep: 不依赖 ROS 时钟
        drain_timing_queue()
        drain_gate_queue()

        if action == "disarm":
            rospy.logwarn("Executing force disarm (STOP)")
            ok = force_disarm_with_timeout(cmd_long, overall_timeout=2.0)
            if not ok:
                rospy.logerr("!!! FORCE DISARM FAILED OR TIMED OUT !!!")
                rospy.logerr("!!! Pull battery or hit QGC E-STOP NOW !!!")
            elif grounded:
                # 地上停桨后同样还原模式。等 /mavros/state 反映出已上锁再切。
                t_end = time.time() + 1.5
                while time.time() < t_end and F.tele.state and F.tele.state.armed:
                    time.sleep(0.05)
                try:
                    restore_ground_mode(set_mode, grounded)
                except Exception as e:
                    rospy.logwarn(f"cleanup: 还原模式异常 (忽略): {e}")
        elif action == "leave":
            rospy.logwarn(f"cleanup: 不动电机 —— 当前由 {F.takeover[0] or mode} 控制")

        if fatal:
            rospy.logerr(f"Terminated [{SEV_FATAL}]: {F.abort_reason[0]}")
        elif F.takeover[0] is not None:
            rospy.logwarn(f"Terminated [TAKEOVER]: 控制权在 {F.takeover[0]}")
        elif F.exc_reason[0] is not None:
            rospy.logerr(f"Terminated [EXCEPTION]: {F.exc_reason[0]}")
        elif F.land_reason[0]:
            rospy.logwarn(f"Terminated [{SEV_LAND}]: {F.land_reason[0]}")
        else:
            rospy.loginfo("*** FLIGHT COMPLETE ***")

        for f in (F.csv_main, F.csv_vins, F.csv_ekf, F.csv_timing, F.csv_gate):
            if f:
                try:
                    f.flush()
                except Exception:
                    pass
        fsync_all_logs()                     # close() 只到内核缓存; 撞机后马上拔电池也要留住
        for f in (F.csv_main, F.csv_vins, F.csv_ekf, F.csv_timing, F.csv_gate):
            if f:
                try:
                    f.close()
                except Exception:
                    pass
        rospy.loginfo(f"All logs closed (fsync 过, 现在可以断电). Dir: {F.log_dir}")
        fsync_all_logs()                     # 把上面这行 events.log 也落盘

        # 恢复 Python 默认的 Ctrl+C 行为 (抛 KeyboardInterrupt)。SIG_DFL 会直接杀进程。
        signal.signal(signal.SIGINT, signal.default_int_handler)

    ok_end = (F.abort_reason[0] is None and F.exc_reason[0] is None and F.takeover[0] is None)
    return 0 if ok_end else 1
