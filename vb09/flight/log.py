# -*- coding: utf-8 -*-
"""
vb09/flight/log.py —— 飞行日志目录、各 csv 记录线程、z 掉高检测、fsync; 遥测 / 状态 / 配置话题的发布
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import os
import csv
import json
import logging
import math
import re
import threading
import time
from collections import deque
from datetime import datetime

import rospy
from std_msgs.msg import String
from vb09.flight import config as C

from vb09.flight.ctx import F
from vb09.flight.ctx import _f, all_params, get_envelope
from vb09.flight.devices import SetpointState, rpy_deg_from_quat


csv_lock_main = threading.Lock()
csv_lock_vins = threading.Lock()
csv_lock_ekf = threading.Lock()
csv_lock_timing = threading.Lock()
csv_lock_gate = threading.Lock()
zdrop_lock = threading.Lock()
last_zdrop_trigger = [None]
last_fsync = [None]
last_vins_stamp = [None]

# pose 回调只往这里塞元组，落盘交给后台线程。eMMC 一次几百 ms 的写阻塞
# 不能出现在 pose 回调里 (会把下一帧 pose 顶出 POSE_STALE_SEC → 缓降)。
timing_queue = deque(maxlen=4096)
timing_queue_lock = threading.Lock()
# ★2026-10-20: gate.csv 同理 —— tick() 跑在主线程 (状态机 / 卡纸判断), 只往队列里放一行,
#   写盘由 gate_logger_loop 做。这样 fsync 线程刷盘时, 主线程一个系统调用都不碰磁盘。
gate_queue = deque(maxlen=4096)

# z 残差突降滑窗: (t_sec, residual)。
# 长度约束: maxlen >= LOG_HZ * ZDROP_SLOW_WINDOW
z_history = deque(maxlen=max(64, int(C.LOG_HZ * C.ZDROP_SLOW_WINDOW * 3)))
z_history_lock = threading.Lock()


# =============================================================================
# =============================   日志初始化   ================================
# =============================================================================
# gate.csv 的列。前 21 列 = INTERFACE_gate.md §6, 后面是 PLAN_gate_v1.md §9 的加列 + 诊断。
GATE_CSV_COLS = [
    "t", "seg", "along", "off_h", "off_v", "pred_h", "pred_v", "ttc", "v_along", "v_cmd",
    "gate_age_ms", "c_x", "c_y", "c_z", "n_x", "n_y", "n_z", "aim_x", "aim_y", "aim_z", "locked",
    "mode", "lvl", "s_plan", "v_plan", "s_est", "v_est", "sp_x", "sp_y", "sp_z",
    "px", "py", "pz", "yaw_deg", "tilt", "det_dist", "det_x", "det_y", "det_z", "det_age_ms",
    "n_valid", "n_rej", "land_deferred",
    # 2026-10-19: 最新有效帧的 σ 拆到靶面 (cm), 手动段标记, 当前停桨线
    "sig_h", "sig_v", "sig_along", "tilt_lim", "det_src",
    # 2026-10-21: 桥每帧的环心观测 ring_w (没滤波) 单独记 —— 和 det_* (红点/融合目标) 同一帧对比高度
    #   (flight_0023/0024: 红点估计比环心低 ~8-10cm, 0024 按红点穿, 实际比红点低 ~10cm 被纸从上方吸进去)
    "ring_x", "ring_y", "ring_z",
    # 2026-10-23: 环有没有部分出画 + 检测端 diag 原样 (JSON: ring_loc/in_frac/z_ring/plane_rms/n_src/dot_why/...)
    "ring_partial", "diag",
]


def _next_flight_seq(base):
    """
    日志目录编号 (2026-10-21): flight_0001, flight_0002 ...
    飞机每次重启时钟都会重置 (日志里的 ROS 时间是 8 月), 日期不能用来区分飞行。
    编号 = max(已有目录编号, .flight_seq 计数文件) + 1; 删掉旧目录也不会复用编号。
    老的 flight_YYYYMMDD_HHMMSS 目录 (8 位日期) 不是 1~5 位纯数字, 自动忽略。
    """
    os.makedirs(base, exist_ok=True)
    seq_file = os.path.join(base, ".flight_seq")
    seen = 0
    for n in os.listdir(base):
        m = re.fullmatch(r"flight_(\d{1,5})", n)
        if m:
            seen = max(seen, int(m.group(1)))
    try:
        with open(seq_file) as f:
            seen = max(seen, int(f.read().strip() or 0))
    except Exception:
        pass
    return seen + 1, seq_file


def _make_flight_dir(base):
    """建 flight_NNNN。两个进程同时起飞时 mkdir 抢不到就编号 +1 重试。→ (目录, 编号)"""
    seq, seq_file = _next_flight_seq(base)
    for _ in range(50):
        d = os.path.join(base, "flight_%04d" % seq)
        try:
            os.makedirs(d, exist_ok=False)
            break
        except FileExistsError:
            seq += 1
    else:
        raise RuntimeError("建不出日志目录 (flight_NNNN 全被占用?)")
    try:
        with open(seq_file, "w") as f:
            f.write(str(seq))
            f.flush()
            os.fsync(f.fileno())
    except Exception:
        pass
    return d, seq


def init_log_dir():
    pass
    pass
    pass

    base = os.path.expanduser(C.LOG_ROOT)
    F.log_dir, flight_seq = _make_flight_dir(base)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")      # 飞机时钟, 重启后会重置, 只作参考
    try:
        up = float(open("/proc/uptime").read().split()[0])
    except Exception:
        up = float("nan")
    try:
        with open(os.path.join(F.log_dir, "run_info.txt"), "w", encoding="utf-8") as f:
            f.write("flight_seq   = %d\n" % flight_seq)
            f.write("script       = %s\n" % os.path.basename(__file__))
            f.write("clock_local  = %s   # 飞机时钟, 重启后会重置, 不能当真实日期\n" % ts)
            f.write("ros_time     = %.3f\n" % rospy.Time.now().to_sec())
            f.write("uptime_s     = %.0f   # 本次开机至今; 同一次开机里的几次飞行可按它排先后\n" % up)
            f.flush()
            os.fsync(f.fileno())
    except Exception as _e:
        rospy.logwarn(f"写 run_info.txt 失败(不影响飞行): {_e}")

    F.csv_main = open(os.path.join(F.log_dir, "flight_log.csv"), "w", newline="")
    F.writer_main = csv.writer(F.csv_main)
    F.writer_main.writerow([
        "t", "phase",
        "px", "py", "pz", "pz_filt",
        "vx", "vy", "vz",
        "roll", "pitch", "yaw",
        "sp_px", "sp_py", "sp_pz",
        "sp_vx", "sp_vy", "sp_vz",
        "sp_yaw", "sp_yaw_rate",
        "act_roll", "act_pitch", "act_yaw", "thrust",
        "batt_v", "mode", "armed", "pose_age_ms",
    ])

    F.csv_vins = open(os.path.join(F.log_dir, "vins_raw.csv"), "w", newline="")
    F.writer_vins = csv.writer(F.csv_vins)
    F.writer_vins.writerow([
        "t_wall", "t_stamp",
        "vins_x", "vins_y", "vins_z",
        "vins_vx", "vins_vy", "vins_vz",
        "vins_qx", "vins_qy", "vins_qz", "vins_qw",
        "mavros_z", "diff_z",
    ])

    F.csv_ekf = open(os.path.join(F.log_dir, "ekf2_health.csv"), "w", newline="")
    F.writer_ekf = csv.writer(F.csv_ekf)
    F.writer_ekf.writerow([
        "t_wall",
        "attitude_status_flag", "velocity_horiz_flag", "velocity_vert_flag",
        "pos_horiz_rel_flag", "pos_horiz_abs_flag",
        "pos_vert_abs_flag", "pos_vert_agl_flag",
        "const_pos_mode_flag", "pred_pos_horiz_rel_flag",
        "pred_pos_horiz_abs_flag",
        "gps_glitch_flag", "accel_error_flag",
        "landed_state", "vtol_state",
    ])

    F.csv_timing = open(os.path.join(F.log_dir, "timing.csv"), "w", newline="")
    F.writer_timing = csv.writer(F.csv_timing)
    F.writer_timing.writerow([
        "t_wall", "pose_stamp", "delay_ms",
        "interval_ms",
        "stamp_backward_count_total",
        "last_jump_cm",
    ])

    # 本次生效的现场配置写进日志目录 —— 复盘时必须知道当时用的是哪套限制。
    try:
        with open(os.path.join(F.log_dir, "site_config.txt"), "w", encoding="utf-8") as f:
            if F.site_report:
                f.write(F.fp.site_config_text(F.site_report, all_params()))
            else:
                # 只有 flight_profile.py 不在时才走这里: 把所有大写数值/开关参数写下来。
                f.write("# 本次未使用现场档案（flight_profile.py 不在）—— 未经任何参数校验\n"
                        "# 以下为全部生效参数\n")
                _g = all_params()
                for _k in sorted(_g):
                    _v = _g[_k]
                    if (_k.isupper() and not _k.startswith("_")
                            and isinstance(_v, (bool, int, float))):
                        f.write("%s = %s\n" % (_k, _v))
                f.write("PHASE_ENVELOPE = %s\n" % (C.PHASE_ENVELOPE,))
    except Exception as _e:
        rospy.logwarn(f"写 site_config.txt 失败(不影响飞行): {_e}")

    # 穿环段每拍一行 (GATE_LOOP_HZ)。t 和 flight_log.csv 同一个时钟 (ROS 秒), 回放页按它对齐。
    F.csv_gate = open(os.path.join(F.log_dir, "gate.csv"), "w", newline="")
    F.writer_gate = csv.writer(F.csv_gate)
    F.writer_gate.writerow(GATE_CSV_COLS)

    F.zdrop_log_path = os.path.join(F.log_dir, "z_drop_events.log")
    with open(F.zdrop_log_path, "w") as f:
        f.write(f"# z residual drop snapshots. flight_{flight_seq:04d}, 飞机时钟 {ts} (可能未校时)\n")
        f.write("# residual = z_filt - z_setpoint  (不是 z 本身)\n")
        f.write(f"# slow: {C.ZDROP_SLOW_THRESH*100:.0f}cm within {C.ZDROP_SLOW_WINDOW}s\n")
        f.write(f"# fast: {C.ZDROP_FAST_THRESH*100:.0f}cm within {C.ZDROP_FAST_WINDOW}s\n\n")

    # 文字日志落到运行目录。rospy.log* 走的是 Python 的 "rosout" logger, 挂个文件
    # handler 就全收到了 —— 包括 ROS 关闭之后、以及 rosout 聚合丢掉的行。
    # 不能因为写日志失败而不飞, 所以整段 try。
    try:
        fh = logging.FileHandler(os.path.join(F.log_dir, "events.log"), encoding="utf-8")
        fh.setFormatter(logging.Formatter("%(created).3f %(levelname)-5s %(message)s"))
        logging.getLogger("rosout").addHandler(fh)
        F.events_fh = fh
    except Exception as _e:
        rospy.logwarn(f"开 events.log 失败(不影响飞行): {_e}")

    rospy.loginfo(f"Log dir: {F.log_dir}  (编号 {flight_seq})")
    fsync_all_logs()            # 表头 / site_config.txt 先落盘


def fsync_all_logs():
    """
    ★2026-10-20: 所有日志 fsync 到盘。之前只有 flight_log.csv 每秒 fsync, 其余只 flush 到内核缓存;
    撞机后很快拔电池 (内核默认 ~30s 才回写, ext4 延迟分配) → gate.csv / events.log / vins_raw.csv
    全是 0 字节, 即使程序已经正常 close。fsync 不需要拿各文件的写锁 (各写线程每行自己 flush 过)。
    """
    fds = [f for f in (F.csv_main, F.csv_vins, F.csv_ekf, F.csv_timing, F.csv_gate) if f is not None]
    st = getattr(F.events_fh, "stream", None) if F.events_fh is not None else None
    if st is not None:
        fds.append(st)
    for f in fds:
        try:
            if not f.closed:
                os.fsync(f.fileno())
        except (OSError, ValueError):
            pass
    for path in (F.zdrop_log_path, os.path.join(F.log_dir, "site_config.txt") if F.log_dir else None):
        if path and os.path.exists(path):
            try:
                fd = os.open(path, os.O_RDONLY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            except OSError:
                pass


def fsync_loop():
    """每 FSYNC_INTERVAL_S 把全部日志 fsync 一次。单独线程: eMMC 的 fsync 可能卡几百 ms, 不能卡别人。"""
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        fsync_all_logs()
        time.sleep(C.FSYNC_INTERVAL_S)


# =============================================================================
# ==============================   各路日志   =================================
# =============================================================================
def log_main_row():
    if F.writer_main is None:
        return
    try:
        with csv_lock_main:
            state, pose, pose_time, vel, batt, act, tgt, zf = F.tele.snap()
            now = rospy.Time.now()
            t = now.to_sec()
            nan = float('nan')

            px = py = pz = pz_filt = nan
            roll = pitch = yaw = nan
            if pose:
                px = pose.pose.position.x
                py = pose.pose.position.y
                pz = pose.pose.position.z
                roll, pitch, yaw = rpy_deg_from_quat(pose.pose.orientation)
            if zf is not None:
                pz_filt = zf

            vx = vy = vz = nan
            if vel:
                vx = vel.twist.linear.x
                vy = vel.twist.linear.y
                vz = vel.twist.linear.z

            sp_px = sp_py = sp_pz = nan
            sp_vx = sp_vy = sp_vz = nan
            sp_yaw = sp_yaw_rate = nan
            if tgt:
                sp_px = tgt.position.x
                sp_py = tgt.position.y
                sp_pz = tgt.position.z
                sp_vx = tgt.velocity.x
                sp_vy = tgt.velocity.y
                sp_vz = tgt.velocity.z
                sp_yaw = math.degrees(tgt.yaw)
                sp_yaw_rate = math.degrees(tgt.yaw_rate)

            act_r = act_p = act_y = thrust = nan
            if act and len(act.controls) >= 4:
                act_r = act.controls[0]
                act_p = act.controls[1]
                act_y = act.controls[2]
                thrust = act.controls[3]

            batt_v = batt.voltage if batt else nan
            mode = state.mode if state else "?"
            armed = state.armed if state else False
            age_ms = (now - pose_time).to_sec() * 1000 if pose_time else nan

            F.writer_main.writerow([
                f"{t:.3f}", F.phase_ref[0],
                f"{px:.3f}", f"{py:.3f}", f"{pz:.3f}", f"{pz_filt:.3f}",
                f"{vx:.3f}", f"{vy:.3f}", f"{vz:.3f}",
                f"{roll:.1f}", f"{pitch:.1f}", f"{yaw:.1f}",
                f"{sp_px:.3f}", f"{sp_py:.3f}", f"{sp_pz:.3f}",
                f"{sp_vx:.3f}", f"{sp_vy:.3f}", f"{sp_vz:.3f}",
                f"{sp_yaw:.1f}", f"{sp_yaw_rate:.1f}",
                f"{act_r:.3f}", f"{act_p:.3f}", f"{act_y:.3f}", f"{thrust:.3f}",
                f"{batt_v:.2f}", mode, armed, f"{age_ms:.0f}",
            ])
            F.csv_main.flush()
            # fsync 按 FSYNC_INTERVAL_S 节流: 每行 fsync 会和其他 IO 叠加成阻塞源。
            if last_fsync[0] is None or (now - last_fsync[0]).to_sec() >= C.FSYNC_INTERVAL_S:
                try:
                    os.fsync(F.csv_main.fileno())
                except OSError:
                    pass
                last_fsync[0] = now
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"main csv exception: {e}")


def log_vins_row():
    """
    按 stamp 去重: 轮询频率高于 VINS 发布频率, 不去重的话同一个样本会写多次,
    拿 vins_raw.csv 做差分会得到一半是零的序列。
    """
    if F.writer_vins is None:
        return
    try:
        with csv_lock_vins:
            odom, odom_time = F.tele.snap_vins()
            if odom is None:
                return
            stamp = odom.header.stamp
            if last_vins_stamp[0] is not None and stamp == last_vins_stamp[0]:
                return
            last_vins_stamp[0] = stamp

            _, pose, _, _, _, _, _, _ = F.tele.snap()
            now = rospy.Time.now()
            nan = float('nan')

            t_stamp = stamp.to_sec()
            vx = odom.pose.pose.position.x
            vy = odom.pose.pose.position.y
            vz = odom.pose.pose.position.z
            vvx = odom.twist.twist.linear.x
            vvy = odom.twist.twist.linear.y
            vvz = odom.twist.twist.linear.z
            q = odom.pose.pose.orientation

            mav_z = pose.pose.position.z if pose else nan
            diff = (mav_z - vz) if pose else nan

            F.writer_vins.writerow([
                f"{now.to_sec():.3f}", f"{t_stamp:.3f}",
                f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}",
                f"{vvx:.3f}", f"{vvy:.3f}", f"{vvz:.3f}",
                f"{q.x:.4f}", f"{q.y:.4f}", f"{q.z:.4f}", f"{q.w:.4f}",
                f"{mav_z:.4f}", f"{diff:.4f}",
            ])
            F.csv_vins.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"vins csv exception: {e}")


def log_ekf_row():
    if F.writer_ekf is None:
        return
    try:
        with csv_lock_ekf:
            est, ext = F.tele.snap_ekf()
            now = rospy.Time.now()

            def gf(obj, name, default=-1):
                try:
                    v = getattr(obj, name, default)
                    return int(v) if v is not None else default
                except Exception:
                    return default

            if est is None:
                att = vh = vv = phr = pha = pva = pval = cpm = ppphr = ppha = 0
                gps = acc = 0
            else:
                att   = gf(est, "attitude_status_flag", 0)
                vh    = gf(est, "velocity_horiz_status_flag", 0)
                vv    = gf(est, "velocity_vert_status_flag", 0)
                phr   = gf(est, "pos_horiz_rel_status_flag", 0)
                pha   = gf(est, "pos_horiz_abs_status_flag", 0)
                pva   = gf(est, "pos_vert_abs_status_flag", 0)
                pval  = gf(est, "pos_vert_agl_status_flag", 0)
                cpm   = gf(est, "const_pos_mode_status_flag", 0)
                ppphr = gf(est, "pred_pos_horiz_rel_status_flag", 0)
                ppha  = gf(est, "pred_pos_horiz_abs_status_flag", 0)
                gps   = gf(est, "gps_glitch_status_flag", 0)
                acc   = gf(est, "accel_error_status_flag", 0)

            landed = gf(ext, "landed_state", -1) if ext else -1
            vtol   = gf(ext, "vtol_state", -1) if ext else -1

            F.writer_ekf.writerow([
                f"{now.to_sec():.3f}",
                att, vh, vv, phr, pha, pva, pval, cpm, ppphr, ppha, gps, acc,
                landed, vtol,
            ])
            F.csv_ekf.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"ekf csv exception: {e}")


def drain_gate_queue():
    """gate.csv: 把队列里攒的行写盘。只在 gate_logger_loop / cleanup 里调。"""
    if F.writer_gate is None or not gate_queue:
        return
    rows = []
    try:
        while True:
            rows.append(gate_queue.popleft())
    except IndexError:
        pass
    try:
        with csv_lock_gate:
            F.writer_gate.writerows(rows)
            F.csv_gate.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"gate csv exception: {e}")


def gate_logger_loop():
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        drain_gate_queue()
        time.sleep(0.1)
    drain_gate_queue()


def drain_timing_queue():
    """后台线程批量落盘。pose 回调只负责 append。"""
    if F.writer_timing is None:
        return
    with timing_queue_lock:
        if not timing_queue:
            return
        batch = list(timing_queue)
        timing_queue.clear()
    try:
        with csv_lock_timing:
            for (t_wall, stamp, delay_ms, interval_ms, back_cnt, jump) in batch:
                F.writer_timing.writerow([
                    f"{t_wall:.3f}", f"{stamp:.3f}", f"{delay_ms:.1f}",
                    f"{interval_ms:.1f}" if interval_ms == interval_ms else "nan",
                    back_cnt, f"{jump*100:.1f}",
                ])
            F.csv_timing.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"timing csv exception: {e}")


def dump_zdrop_snapshot(reason, slow_drop, fast_drop):
    if F.zdrop_log_path is None:
        return
    try:
        with zdrop_lock:
            state, pose, pose_time, vel, batt, act, tgt, zf = F.tele.snap()
            odom, odom_time = F.tele.snap_vins()
            est, ext = F.tele.snap_ekf()
            now = rospy.Time.now()
            z_sp = F.setpoint.z_setpoint()

            with open(F.zdrop_log_path, "a") as f:
                f.write(f"\n=== Z_DROP EVENT @ t={now.to_sec():.3f} "
                        f"phase={F.phase_ref[0]} ===\n")
                f.write(f"reason: {reason}\n")
                f.write(f"slow_drop_{C.ZDROP_SLOW_WINDOW}s: {slow_drop*100:.1f}cm  "
                        f"fast_drop_{C.ZDROP_FAST_WINDOW}s: {fast_drop*100:.1f}cm\n")
                f.write(f"z_setpoint: {z_sp if z_sp is None else f'{z_sp:.3f}'}\n")
                if pose:
                    p = pose.pose.position
                    f.write(f"mavros pose: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} "
                            f"z_filt={zf:.3f}\n")
                    age_ms = (now - pose_time).to_sec() * 1000 if pose_time else -1
                    f.write(f"pose_age_ms: {age_ms:.0f}\n")
                if odom:
                    op = odom.pose.pose.position
                    ov = odom.twist.twist.linear
                    odom_age = (now - odom_time).to_sec() * 1000 if odom_time else -1
                    f.write(f"vins raw: x={op.x:.3f} y={op.y:.3f} z={op.z:.3f}  "
                            f"v=({ov.x:+.2f},{ov.y:+.2f},{ov.z:+.2f})  "
                            f"age_ms={odom_age:.0f}\n")
                    if pose:
                        f.write(f"z_diff (mavros - vins): "
                                f"{pose.pose.position.z - op.z:+.3f}m\n")
                else:
                    f.write("vins raw: NO DATA\n")
                if vel:
                    v = vel.twist.linear
                    f.write(f"mavros vel: ({v.x:+.2f},{v.y:+.2f},{v.z:+.2f})\n")
                if tgt:
                    f.write(f"setpoint: p=({tgt.position.x:.2f},"
                            f"{tgt.position.y:.2f},{tgt.position.z:.2f}) "
                            f"v=({tgt.velocity.x:+.2f},{tgt.velocity.y:+.2f},"
                            f"{tgt.velocity.z:+.2f}) "
                            f"yaw={math.degrees(tgt.yaw):+.1f}\n")
                if act and len(act.controls) >= 4:
                    f.write(f"actuator: roll={act.controls[0]:+.2f} "
                            f"pitch={act.controls[1]:+.2f} "
                            f"yaw={act.controls[2]:+.2f} "
                            f"thrust={act.controls[3]:.2f}\n")
                if batt:
                    f.write(f"battery: {batt.voltage:.2f}V\n")
                if state:
                    f.write(f"state: mode={state.mode} armed={state.armed}\n")
                if est:
                    def gf(o, n):
                        return getattr(o, n, "?")
                    f.write(f"ekf2 flags: "
                            f"pos_vert_abs={gf(est, 'pos_vert_abs_status_flag')} "
                            f"const_pos={gf(est, 'const_pos_mode_status_flag')} "
                            f"accel_err={gf(est, 'accel_error_status_flag')} "
                            f"gps_glitch={gf(est, 'gps_glitch_status_flag')}\n")
                if ext:
                    f.write(f"landed_state: {ext.landed_state}\n")
                f.write(f"stamp_backward_count: {F.tele.stamp_backward_count}\n")
                f.write("=" * 60 + "\n")

            rospy.logwarn(f"[zdrop] {reason} — slow={slow_drop*100:.1f}cm "
                          f"fast={fast_drop*100:.1f}cm, snapshot dumped")
    except Exception as e:
        rospy.logwarn(f"zdrop dump exception: {e}")


def check_z_drop():
    """
    看 (z_filt - z_setpoint) 残差，不看 z 本身: 主动下降 (跟踪下降 / descend)
    时指令和实际同步下降, 残差不变, 不误触发; VIO 真的掉高时残差立刻变化。
    z_drop_events.log 是排查真实 VIO 掉高的唯一线索, 不能被正常下降刷满。
    """
    now = rospy.Time.now()
    if last_zdrop_trigger[0] is not None and \
            (now - last_zdrop_trigger[0]).to_sec() < C.ZDROP_REFRACTORY:
        return

    with z_history_lock:
        if len(z_history) < C.ZDROP_MIN_SAMPLES:
            return
        history = list(z_history)

    t_now, r_now = history[-1]

    slow_max = r_now
    for t, r in reversed(history):
        if (t_now - t) > C.ZDROP_SLOW_WINDOW:
            break
        if r > slow_max:
            slow_max = r
    slow_drop = slow_max - r_now

    fast_max = r_now
    n_fast = 0
    for t, r in reversed(history):
        if (t_now - t) > C.ZDROP_FAST_WINDOW:
            break
        n_fast += 1
        if r > fast_max:
            fast_max = r
    fast_drop = fast_max - r_now

    triggered = False
    reason = ""
    if n_fast >= 2 and fast_drop > C.ZDROP_FAST_THRESH:
        triggered = True
        reason = f"fast: {fast_drop*100:.1f}cm residual in {C.ZDROP_FAST_WINDOW}s"
    elif slow_drop > C.ZDROP_SLOW_THRESH:
        triggered = True
        reason = f"slow: {slow_drop*100:.1f}cm residual in {C.ZDROP_SLOW_WINDOW}s"

    if triggered:
        last_zdrop_trigger[0] = now
        dump_zdrop_snapshot(reason, slow_drop, fast_drop)


# =============================================================================
# ============================   后台线程   ===================================
# =============================================================================
def main_logger_loop():
    rate = rospy.Rate(C.LOG_HZ)
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        log_main_row()
        zf = F.tele.z_filtered()
        z_sp = F.setpoint.z_setpoint()
        if zf is not None and z_sp is not None:
            with z_history_lock:
                z_history.append((rospy.Time.now().to_sec(), zf - z_sp))
        elif zf is None or z_sp is None:
            # MODE_VELOCITY 期间没有位置设定点，残差无意义 —— 清空滑窗，
            # 避免切回 ZHOLD 时跨模式做差分。
            with z_history_lock:
                z_history.clear()
        check_z_drop()
        rate.sleep()


def vins_logger_loop():
    # 轮询频率高于 VINS 发布频率即可，去重在 log_vins_row 里做
    rate = rospy.Rate(C.VINS_LOG_HZ)
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        log_vins_row()
        rate.sleep()


def ekf_logger_loop():
    rate = rospy.Rate(C.EKF_LOG_HZ)
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        log_ekf_row()
        rate.sleep()


def timing_logger_loop():
    rate = rospy.Rate(C.TIMING_LOG_HZ)
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        drain_timing_queue()
        rate.sleep()
    drain_timing_queue()


def publisher_loop(pub):
    rate = rospy.Rate(C.SP_PUB_HZ)
    while not rospy.is_shutdown() and not F.abort_flag.is_set():
        try:
            pub.publish(F.setpoint.build_msg())
        except Exception as e:
            rospy.logerr(f"publisher exception: {e}")
        rate.sleep()


def status_loop(spub):
    """阶段状态 → vision_bridge → 地面站 HUD。纯观测，不参与控制。"""
    rate = rospy.Rate(C.STATUS_HZ)
    while not rospy.is_shutdown():
        try:
            spub.publish(String(json.dumps({
                "phase": F.phase_ref[0],
                "sub_mode": F.sub_mode_ref[0],     # 穿环段名 (见 sub_mode_ref)
                "air":   bool(F.in_air[0]),
                "abort": bool(F.abort_flag.is_set()),
                "land":  bool(F.land_flag.is_set()),
                "shadow": bool(C.GATE_SHADOW),
                "z_track": False,
                "task": C.TASK,
            }, ensure_ascii=False)))
        except Exception:
            pass
        rate.sleep()


def build_telemetry():
    """网页用的完整快照。只读全局, 不改任何状态。"""
    with F.telem_lock:
        sections = {k: dict(v) for k, v in F.telem_data.items()}
    mode, p, v, a, yaw = F.setpoint.snapshot()
    sp = {"mode": {SetpointState.MODE_VELOCITY: "VEL", SetpointState.MODE_ZHOLD: "ZHOLD",
                   SetpointState.MODE_TRAJ: "TRAJ"}.get(mode, "?"),
          "px": _f(p[0]), "py": _f(p[1]), "pz": _f(p[2]),
          "yaw": _f(yaw), "vx": _f(v[0]), "vy": _f(v[1]), "vz": _f(v[2]),
          "ax": _f(a[0]), "ay": _f(a[1]), "az": _f(a[2])}
    h, vl = get_envelope(F.phase_ref[0])
    return {
        "t": rospy.Time.now().to_sec(),
        "node": rospy.get_name(),             # 网页靠它过滤 /rosout 事件
        "task": C.TASK,
        "schema": 2,
        "phase": F.phase_ref[0],
        "sub_mode": F.sub_mode_ref[0],
        "air": bool(F.in_air[0]),
        "abort": bool(F.abort_flag.is_set()),
        "land": bool(F.land_flag.is_set()),
        "abort_reason": F.abort_reason[0],
        "land_reason": F.land_reason[0],
        "takeover": F.takeover[0],              # 被接管时 = 飞控模式名
        "t_requested": False,
        "shadow": bool(C.GATE_SHADOW),
        "mode": C.MISSION_MODE,               # dash / shadow / dock (档案 mode)
        "sim_drive": bool(C.GATE_ALLOW_SIM_DRIVE),
        "z_enable": False,
        "origin": [_f(F.INITIAL_X), _f(F.INITIAL_Y), _f(F.INITIAL_Z), _f(F.INITIAL_YAW)],
        "setpoint": sp,
        "z_filt": _f(F.tele.z_filtered()),
        "cells": C.cell_count[0],
        "fly_min_v": _f(C.CELL_MIN_FLY * C.cell_count[0]),
        "envelope": [_f(h), _f(vl)],
        "log_dir": F.log_dir,
        "track": {},
        "gates": {},
        "safety": sections.get("safety", {}),
        "gate": sections.get("gate", {}),
    }


def telemetry_loop(tpub):
    """
    TELEMETRY_TOPIC。提交/冲刺/制动段 TELEMETRY_HZ_FAST (冲刺只有 1~2 秒, 5Hz 在网页上
    只有几个点), 其余 TELEMETRY_HZ。纯观测, 出错只跳过这一帧。
    """
    while not rospy.is_shutdown():
        t0 = time.time()
        try:
            tpub.publish(String(json.dumps(build_telemetry(), ensure_ascii=False)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[telemetry] 跳过一帧: {e}")
        fast = F.phase_ref[0] == "gate" and F.sub_mode_ref[0] in ("commit", "dash", "brake", "hold")
        dt = 1.0 / (C.TELEMETRY_HZ_FAST if fast else C.TELEMETRY_HZ)
        time.sleep(max(0.0, dt - (time.time() - t0)))


def publish_config(cpub):
    """
    CONFIG_TOPIC (latched)。网页上的围栏/刻度全部按这份画,
    网页里不写任何阈值 —— 保证"页面显示的就是脚本实际用的"。
    preflight 前发一次(原点未定), preflight 后再发一次(原点确定)。
    """
    try:
        params = {k: v for k, v in all_params().items()
                  if k.isupper() and not k.startswith("_")
                  and isinstance(v, (bool, int, float))}
        if F.fp is not None:
            derived = F.fp.derived_snapshot(vars(C))
        else:
            derived = {"phase_envelope": {k: list(v) for k, v in C.PHASE_ENVELOPE.items()}}
        rep = F.site_report or {}
        cpub.publish(String(json.dumps({
            "t": rospy.Time.now().to_sec(),
            "node": rospy.get_name(),
            "task": C.TASK,
            "src": rep.get("src", "（flight_profile.py 不在，未经校验）"),
            "knobs": rep.get("knobs", {}),
            "lines": rep.get("lines", []),
            "warnings": rep.get("warnings", []),
            "derived": derived,
            "params": params,
            "origin": [_f(F.INITIAL_X), _f(F.INITIAL_Y), _f(F.INITIAL_Z), _f(F.INITIAL_YAW)],
            "log_dir": F.log_dir,
        }, ensure_ascii=False)))
    except Exception as e:
        rospy.logwarn(f"[telemetry] 发布配置失败(不影响飞行): {e}")
