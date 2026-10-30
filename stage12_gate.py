#!/usr/bin/env python3
"""
stage13_gate.py —— 穿环 (撞环) 实飞脚本 v1: 悬停定位 → 冻结 → 按预定轨迹冲刺 → 原地降落
===============================================================================
方案: 项目文档 claude/PLAN_gate_v1.md (2026-10-19 讨论后简化, 见下面"和方案的差别")
接口: 项目文档 claude/INTERFACE_gate.md (遥测 gate 段 / 配置 derived.gate / gate.csv)
规划: 同目录 gate_plan.py (纯函数, 可离线跑)

启动 (档案与参数见 flight_profile.py; 档案里写 task = 穿环)
  python3 stage13_gate.py --site <档案> --dry-run   只打印生效包络, 不需要 ROS
  python3 stage13_gate.py --site <档案>             真飞: 必须在**普通** SSH 终端里

文件名约定 (2026-10-19): stage<版本号>_<任务>.py —— 编号是版本, 后缀是任务
  (_gate 穿环 / _track 停靠跟踪; 老名字 stage11_real.py 也算跟踪)。flight_monitor 按任务各取编号最大的。

前提
  视觉链路: python3 run_vision.py gate_gcs (地面检测) 或 run_vision.py gate --drive (机上检测) —— 预检会查
  PX4: MPC_TILTMAX_AIR ≤ MAX_TILT_DEG − 5 (=25°), MPC_XY_VEL_MAX ≥ 1.1×DASH_V —— 预检会查

ROS 接口
  节点   /flight_ctrl   (和 stage11 同名, 不能同时跑)
  订阅   /gate/status   String JSON (vision_bridge 每帧一条: target=环心, 可选 n / aim / n_sd_deg)
         /vision_bridge/cam  String JSON (图传模式, 预检用)
         /mavros/...  /vins_estimator/odometry  (同 stage11)
  发布   /mavros/setpoint_raw/local   PositionTarget  SP_PUB_HZ (冲刺段带加速度前馈)
         /flight_ctrl/status      JSON 2Hz   (同 stage11 + task)
         /flight_ctrl/telemetry   JSON 5Hz, 提交/冲刺/制动段 20Hz  (+ task, schema, gate 段)
         /flight_ctrl/config      JSON latched (+ task, derived.gate, derived.speed_chain)

飞行流程 (phase 全程 = "gate", 段名在遥测 gate.seg)
  preflight → prime → OFFBOARD → arm → takeoff
    → MANUAL   (manual) 手动段 (GATE_MANUAL_START): 键和 stage11 一样, 把飞机开到想起跑的地方,
                        按 T 开始下面这一次。GATE_MANUAL_TIMEOUT 一个键都没按 → 降落
    → ACQUIRE  (wait)   原地悬停, 机头慢慢转向环, 收 HOVER_ACQ 秒有效识别; 8s 看不到 → ±30° 扫一次
    → PLAN     (wait)   gate_plan.plan_dash: 进入方向 / 起跑点 S / 制动终点 E / 撞纸速度; 不可行先降速
                        起跑点动态: 离环在 [最小助跑, D_RUN_MAX] 之间就地起跑, 近了后退、远了前进
    → REPOSITION(align) 直线慢飞到 S (≤ GATE_REPO_V), 全程在环前 1.5m 以外, 机头盯环
    → ALIGN    (align)  在 S 悬停 HOVER_ALIGN 秒取样 → 收敛判定 → **锁定** → 用锁定值重新规划
    → COMMIT   (commit) 打印计划, 等操作员按 G (GATE_CONFIRM_DASH)。之后不能中止
    → DASH / BRAKE      按时间表执行轨迹 (位置+速度+加速度前馈, 全程 P 模式), 不看识别
    → HOLD     (hold)   以**当前位置**重锚悬停 GATE_HOLD_S → 原地缓降 → AUTO.LAND
  冲刺前任何一步放弃 (锁不住 / 不可行 / G 超时 / 找不到环) 或按 M / 方向键 → 回 MANUAL, 可以再 T
    (GATE_ABORT_TO_MANUAL; LAND 请求 / 接管 / 手动段超时照常降落)
  影子 GATE_SHADOW=True: ACQUIRE → PLAN → 在原地做 ALIGN 锁定 → 发计划给网页 → 不摆位不冲刺 → 回 MANUAL
  虚拟环 GATE_VIRTUAL_OFFSET=d: 把环平面往进近侧挪 d 米, 真纸面当墙 → 真冲刺但停在纸前 (T3)

键盘
  手动段   W前 S后 A左 D右 | R升 F降 | Q左转 E右转 | 空格悬停 | T = 开始找环/规划
  冲刺前   M 或任一方向键 = 回手动段   L = 就地降落   X = 停桨   G = 确认冲刺 (只在 COMMIT 段有效)
  冲刺中   X = 停桨 (立刻)   L / SSH 断连 = 记下, **刹停之后**再降落 (冲刺中缓降比刹完更危险)

安全分级 (沿用 stage11)
  FATAL (空中停桨)   倾角 > MAX_TILT_DEG(30°) 持续 FATAL_CONFIRM_S / MAVROS 断链 / 按 X
                    按 G 后到刹停悬停结束: 倾角 > GATE_DASH_TILT_DEG(45°) 持续 0.3s (急刹 + 撞纸)
  LAND  (缓降)       围栏、VIO 老化/跳变、视觉断供、速度超限、低电压、SSH 断连 …
                    —— 冲刺/制动段收到的 LAND **推迟到刹停后**执行 (见上)
  卡纸              冲刺中 (过纸面前) 实测速度比规划低 > GATE_STUCK_DV 且落后规划 > GATE_STUCK_DS
                    持续 GATE_STUCK_HOLD → 设定点改成当前位置 (停止前推) → 原地降落
  接管 / 脚本异常    同 stage11 (phase_handover / cleanup_decision)

和方案 PLAN_gate_v1.md 的差别 (2026-10-19 定的"先简单方案")
  · 全程 P 模式 (盲区不切速度模式), 不做 vision_gate (VINS 照常喂 EKF)
  · 不做 kill_watchdog / 姿态模式 (L2) / D-att: 倾角停桨沿用 stage11 的 safety_monitor,
    只是按 G 之后到刹停悬停结束换成 GATE_DASH_TILT_DEG (默认 45°/0.3s)
  · 刹车起点 (2026-10-21 改): 机头碰纸后再冲 PASS_RUN 才刹, s_b = −L_F + PASS_RUN + v·T_LAG
    (滞后补偿); 停止点要离环后墙 ≥ M_BACK, 放不下就降速
  · 助跑距离 ≥ D_ALIGN_MIN (起跑点要看得到整个环)
  · 穿环后不返航: 刹停 → 原地降落

2026-10-19 首飞后的修改 (项目文档 claude/ISSUE_gate_run_2026-10-19.md)
  · §1 识别 σ / 环心离散都按靶面拆成 横 h / 竖 v / 深度 分开判 (双目深度 σ ∝ 距离²)
  · §2 预检: 桥里的环估计 age_ms > 1s (旧状态) → 拒绝起飞 (新版 vision_bridge 会自己重置)
  · §3 预检: PX4 参数读不到 → 先 param/pull 等缓存, 还读不到就拒绝起飞; 读到的值写进 site_config.txt
  · §5 cleanup: 落地后停在 AUTO.LAND 就不动了 (不再去切 MANUAL / LOITER)
  · 手动段 + T 开始、冲刺前放弃回手动段、动态起跑点、冲刺段停桨线 45°

日志  ~/uav_logs/flight_NNNN/   (编号递增, 不用日期: 飞机每次重启时钟都会重置; 目录里有 run_info.txt)
  flight_log.csv (phase=gate)   gate.csv (穿环段每拍一行, 列见 init_log_dir)
  vins_raw.csv  ekf2_health.csv  timing.csv  z_drop_events.log  site_config.txt  events.log
===============================================================================
"""

import os
import sys
import csv
import json
import logging
import math
import re
import signal
import statistics
import threading
import time
import traceback
from collections import deque
from datetime import datetime

# numpy / rospy / mavros 消息在"现场档案校验"之后才 import (搜 "ROS 相关 import"),
# 所以 --dry-run 不需要 ROS。CONFIG 和派生量区不许用 ROS 的东西。


# =============================================================================
# ############################   CONFIG   ######################################
# =============================================================================
# 2026-10-30: 全部参数搬到 vb09/flight/config.py, 统一写 C.名字。现场请用 `--site <档案>`。
from vb09.flight import config as C  # noqa: E402

# =============================================================================
# ####################   现场档案（必须在派生量之前套用!）   ##################
# =============================================================================
# 下面"派生量"区是用上面的参数算出来的。档案必须在这里 (module 级、派生量之前)
# 写进全局, 否则派生量拿的是旧值。两段式:
#   setup()    在这里: 解析档案 + --set, 写进全局, 只拦"读不懂"的错误
#   finalize() 在派生量之后: 对最终生效值做全部一致性 + 安全检查; --dry-run 在那里退出
_SITE_REPORT = None
_ORIG_ARGV = list(getattr(sys, "argv", []))   # setup() 会改 sys.argv, 先留一份给提示用
try:
    import flight_profile as _fp
except ImportError:
    _fp = None
    # 没有 flight_profile 时 --dry-run 会被无视 → 本来只想看一眼却真飞了。
    # 所以只要带了任何 "-" 开头的参数就拒绝运行。
    if any(a.startswith("-") for a in _ORIG_ARGV[1:]):
        print("[site] [!] 命令行带了 %s, 但没找到 flight_profile.py "
              "—— 拒绝运行。把 flight_profile.py 和本文件放同一目录。"
              % " ".join(a for a in _ORIG_ARGV[1:] if a.startswith("-")))
        sys.exit(1)
    print("[site] [!] 没找到 flight_profile.py —— 本次用**代码默认值**飞, "
          "且**没有**任何参数校验。把 flight_profile.py 和本文件放同一目录即可。")
if _fp is not None:
    _SITE_REPORT = _fp.setup(sys.argv, vars(C))

# =============================================================================
# ##########################   派生量 (不要手改)   ###########################
# =============================================================================
C.derive()

# =============================================================================
# ##############   现场档案第二段: 对最终生效值做全部校验   #################
# =============================================================================
# 必须在派生量**之后**: 这里检查的是最终值(代码默认 + 档案 + --set)和派生量
# (PHASE_ENVELOPE / 停靠壳 / 可用带)。不过 → 拒绝起飞; --dry-run → 打印后退出。
if _fp is not None:
    _SITE_REPORT = _fp.finalize(_SITE_REPORT, vars(C))


# =============================================================================
# ##################   终端检查: 只能在普通 SSH 会话里飞   ###################
# =============================================================================
# 放在 finalize 之后: --dry-run 已经在上面退出了, 所以网页预览(子进程)和
# 在 tmux 里做 dry-run 都不受影响 —— 只拦"真飞"。
#
# 为什么: SSH 断连保护靠的是"断连后 stdin 读到 EOF → 立即缓降"(见 KeyReader)。
#   tmux / screen / VSCode 内置终端在 SSH 断开后**继续替你保持伪终端**,
#   stdin 永远不会 EOF:
#     · 穿环段的识别/摆位/等 G 都会一直等下去, 直到各自超时
#     · RC 的 SWC/SWD 是坏的 —— 键盘是唯一人工急停, 断了就只剩 QGC
#   stdin 不是终端(nohup / 后台 / 网页启动)更糟: X 急停不存在, 也按不了 G。
#   所以这几种情况一律拒绝起飞。VINS / mavros / flight_monitor 照常放 tmux 里。
def _terminal_problems():
    probs = []
    if os.environ.get("TMUX"):
        probs.append("在 tmux 里运行")
    if os.environ.get("STY"):
        probs.append("在 screen 里运行")
    if os.environ.get("TERM_PROGRAM") == "vscode" or os.environ.get("VSCODE_IPC_HOOK_CLI"):
        probs.append("在 VSCode 内置终端里运行")
    try:
        if not sys.stdin.isatty():
            probs.append("stdin 不是终端 (后台/nohup/重定向启动)")
    except Exception:
        probs.append("stdin 不可用")
    return probs


if __name__ == "__main__":
    _probs = _terminal_problems()
    if _probs:
        print("\n" + "!" * 78)
        print("拒绝起飞: 检测到 %s" % "、".join(_probs))
        print("  这样运行时 SSH 断开不会让脚本收到 EOF, 断连自动降落保护失效;")
        print("  RC 急停开关是坏的, 键盘是唯一人工急停通道。")
        print("")
        print("  请在笔记本上新开一个**普通**终端窗口(PowerShell / Windows Terminal):")
        print("      ssh visbot@<飞机IP>")
        print("      cd %s" % os.path.dirname(os.path.abspath(__file__)))
        print("      python3 %s %s" % (os.path.basename(__file__), " ".join(_ORIG_ARGV[1:])))
        print("  不要套 tmux, 不要用 VSCode 内置终端。")
        print("  (--dry-run 不受此限制, 在哪里跑都行)")
        print("!" * 78 + "\n")
        sys.exit(1)


# =============================================================================
# ##########################   ROS 相关 import   #############################
# =============================================================================
# 放在校验之后: --dry-run 到不了这里, 所以 dry-run 不需要 ROS。
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import (
    ActuatorControl, PositionTarget, State, ExtendedState, EstimatorStatus,
    StatusText,
)
from mavros_msgs.srv import CommandBool, CommandLong, ParamGet, SetMode
try:
    from mavros_msgs.srv import ParamPull
except ImportError:                       # 很老的 mavros_msgs
    ParamPull = None
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

import gate_plan as gp      # 同目录, 纯函数 (flight_profile 的 finalize 已经确认它在)
from vb09.common import geom


# =============================================================================
# ==============================   小函数   ===================================
# =============================================================================
def all_params():
    """
    以前直接用 globals() 的地方 (site_config.txt 的全部参数 / /flight_ctrl/config 的 params):
    = C (配置 + 派生量) + 本脚本运行时的大写量 (INITIAL_* / PX4_PULL_TIMEOUT), 和搬家前同一个集合。
    """
    g = dict(vars(C))
    g.update({k: v for k, v in globals().items() if k.isupper() and not k.startswith("_")})
    return g


def cells_from_voltage(v):
    """
    电池电压 → 串数。读不到 (v 为 None 或 <=0) 返回 None, 调用方保持默认并大声警告 ——
    绝不硬猜 6S: 4S 机被误判成 6S, 门槛变成 21.0V 而 4S 满电只有 16.8V → 一解锁就缓降。
    """
    if v is None or v <= 0:
        return None
    return 6 if v >= C.CELL_V_SPLIT else 4

# abort 分级
SEV_FATAL = "FATAL"   # 空中停桨
SEV_LAND  = "LAND"    # 冻结 → 缓降 → AUTO.LAND


# =============================================================================

# ==============================   Telemetry   ================================
# =============================================================================
class Telemetry:
    """mavros/VINS 遥测汇聚。所有 setter 由 ROS 回调线程调用，必须极快。"""

    def __init__(self):
        self.state = None
        self.pose = None
        self.pose_time = None
        self.pose_stamp_prev = None
        self.velocity = None
        self.velocity_time = None
        self.battery = None
        self.actuator = None
        self.target_local = None

        # 诊断
        self.vins_odom = None
        self.vins_odom_time = None
        self.estimator_status = None
        self.extended_state = None

        self._z_filt = None
        self._yaw = 0.0
        self._lock = threading.Lock()

        # 跳变锁存"自上次读取以来的最大值": monitor 20Hz 采样 30Hz 的 pose,
        # 覆盖式写入会漏掉三分之一的帧。
        self._jump_max = 0.0
        self.last_jump = 0.0          # 兼容字段: 最近一帧的瞬时值
        self.stamp_backward_count = 0

    def set_state(self, m):
        with self._lock:
            self.state = m

    def set_pose(self, m):
        with self._lock:
            prev = self.pose
            prev_stamp = self.pose_stamp_prev
            self.pose = m
            self.pose_time = rospy.Time.now()

            if prev_stamp is not None and m.header.stamp < prev_stamp:
                self.stamp_backward_count += 1
            self.pose_stamp_prev = m.header.stamp

            z = m.pose.position.z
            if self._z_filt is None:
                self._z_filt = z
            else:
                self._z_filt = (C.Z_FILTER_ALPHA * z
                                + (1.0 - C.Z_FILTER_ALPHA) * self._z_filt)

            self._yaw = yaw_from_quat(m.pose.orientation)

            if prev is not None:
                dx = m.pose.position.x - prev.pose.position.x
                dy = m.pose.position.y - prev.pose.position.y
                dz = z - prev.pose.position.z
                j = math.sqrt(dx * dx + dy * dy + dz * dz)
            else:
                j = 0.0
            self.last_jump = j
            if j > self._jump_max:
                self._jump_max = j

    def take_jump_max(self):
        """读取并清零。monitor 用，保证不漏帧。"""
        with self._lock:
            j = self._jump_max
            self._jump_max = 0.0
            return j

    def set_velocity(self, m):
        with self._lock:
            self.velocity = m
            self.velocity_time = rospy.Time.now()

    def set_battery(self, m):
        with self._lock:
            self.battery = m

    def set_actuator(self, m):
        with self._lock:
            self.actuator = m

    def set_target_local(self, m):
        with self._lock:
            self.target_local = m

    def set_vins_odom(self, m):
        with self._lock:
            self.vins_odom = m
            self.vins_odom_time = rospy.Time.now()

    def set_estimator_status(self, m):
        with self._lock:
            self.estimator_status = m

    def set_extended_state(self, m):
        with self._lock:
            self.extended_state = m

    def z_filtered(self):
        with self._lock:
            return self._z_filt

    def yaw_measured(self):
        with self._lock:
            return self._yaw

    def xy(self):
        """当前 xy，没有 pose 时回落到 origin。"""
        with self._lock:
            if self.pose is None:
                return INITIAL_X, INITIAL_Y
            p = self.pose.pose.position
            return p.x, p.y

    def snap(self):
        with self._lock:
            return (self.state, self.pose, self.pose_time, self.velocity,
                    self.battery, self.actuator, self.target_local,
                    self._z_filt)

    def snap_vins(self):
        with self._lock:
            return self.vins_odom, self.vins_odom_time

    def snap_ekf(self):
        with self._lock:
            return self.estimator_status, self.extended_state


# =============================================================================
# ==============================   Setpoint   =================================
# =============================================================================
class SetpointState:
    """
    三种发布模式:
      MODE_VELOCITY  vx/vy/vz + yaw_rate       (仅 prime 阶段用)
      MODE_ZHOLD     (px, py, pz) 位置目标 + (vx, vy) 速度前馈 + 绝对 yaw
      MODE_TRAJ      冲刺轨迹: 每次发布时按 (现在 − t0) 采样 位置 + 速度 + 加速度 (xyz)
                     —— 采样在发布线程里做 (SP_PUB_HZ), 不受状态机循环节奏影响

    ZHOLD 的 z 通道由 vz_ff 决定:
      vz_ff is None → 置 IGNORE_VZ, 纯位置 z
      vz_ff 是数    → 下发垂直速度前馈 (摆位时用)
    """
    MODE_VELOCITY = 0
    MODE_ZHOLD = 1
    MODE_TRAJ = 2

    def __init__(self):
        self._lock = threading.Lock()
        self.mode = self.MODE_VELOCITY
        self.vx = self.vy = self.vz = 0.0
        self.px = self.py = self.pz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.vz_ff = None
        self.sampler = None           # MODE_TRAJ: f(t) → ((px,py,pz), (vx,vy,vz), (ax,ay,az))
        self.t0 = 0.0
        self.acc_ff = True

    def set_traj_mode(self, sampler, t0, yaw, acc_ff=True):
        with self._lock:
            self.mode = self.MODE_TRAJ
            self.sampler, self.t0, self.yaw, self.acc_ff = sampler, t0, yaw, bool(acc_ff)
            self._sample(rospy.Time.now().to_sec())

    def _sample(self, now):
        (self.px, self.py, self.pz), (self.vx, self.vy, self.vz), (self.ax, self.ay, self.az) = \
            self.sampler(now - self.t0)

    def mode_name(self):
        with self._lock:
            return {self.MODE_VELOCITY: "VEL", self.MODE_ZHOLD: "ZHOLD",
                    self.MODE_TRAJ: "TRAJ"}.get(self.mode, "?")

    def snapshot(self):
        """(mode, p, v, a, yaw) —— 日志/遥测用。MODE_TRAJ 时是最近一次发布时采的样。"""
        with self._lock:
            return (self.mode, (self.px, self.py, self.pz), (self.vx, self.vy, self.vz),
                    (self.ax, self.ay, self.az), self.yaw)

    def set_velocity_mode(self, vx, vy, vz, yaw_rate=0.0):
        with self._lock:
            self.mode = self.MODE_VELOCITY
            self.vx, self.vy, self.vz = vx, vy, vz
            self.yaw_rate = yaw_rate

    def set_zhold_mode(self, x_target, y_target, z_target, yaw,
                       vx_ff=0.0, vy_ff=0.0, vz_ff=None):
        with self._lock:
            self.mode = self.MODE_ZHOLD
            self.px, self.py, self.pz = x_target, y_target, z_target
            self.vx, self.vy = vx_ff, vy_ff
            self.vz = vz_ff if vz_ff is not None else 0.0
            self.ax = self.ay = self.az = 0.0
            self.vz_ff = vz_ff
            self.yaw = yaw
            self.sampler = None

    def z_setpoint(self):
        """给 z 残差突降检测器用。MODE_VELOCITY 期间返回 None。"""
        with self._lock:
            return self.pz if self.mode in (self.MODE_ZHOLD, self.MODE_TRAJ) else None

    def build_msg(self):
        with self._lock:
            msg = PositionTarget()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            if self.mode == self.MODE_VELOCITY:
                msg.type_mask = (
                    PositionTarget.IGNORE_PX
                    | PositionTarget.IGNORE_PY
                    | PositionTarget.IGNORE_PZ
                    | PositionTarget.IGNORE_AFX
                    | PositionTarget.IGNORE_AFY
                    | PositionTarget.IGNORE_AFZ
                    | PositionTarget.IGNORE_YAW
                )
                msg.velocity.x = self.vx
                msg.velocity.y = self.vy
                msg.velocity.z = self.vz
                msg.yaw_rate = self.yaw_rate
            elif self.mode == self.MODE_TRAJ:
                self._sample(msg.header.stamp.to_sec())
                msg.type_mask = PositionTarget.IGNORE_YAW_RATE
                if not self.acc_ff:
                    msg.type_mask |= (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
                                      | PositionTarget.IGNORE_AFZ)
                msg.position.x, msg.position.y, msg.position.z = self.px, self.py, self.pz
                msg.velocity.x, msg.velocity.y, msg.velocity.z = self.vx, self.vy, self.vz
                msg.acceleration_or_force.x = self.ax
                msg.acceleration_or_force.y = self.ay
                msg.acceleration_or_force.z = self.az
                msg.yaw = self.yaw
            else:
                msg.type_mask = (
                    PositionTarget.IGNORE_AFX
                    | PositionTarget.IGNORE_AFY
                    | PositionTarget.IGNORE_AFZ
                    | PositionTarget.IGNORE_YAW_RATE
                )
                if self.vz_ff is None:
                    msg.type_mask |= PositionTarget.IGNORE_VZ        # 纯位置 (默认)
                else:
                    msg.velocity.z = self.vz_ff                      # 垂直速度前馈
                msg.position.x = self.px
                msg.position.y = self.py
                msg.position.z = self.pz
                msg.velocity.x = self.vx
                msg.velocity.y = self.vy
                msg.yaw = self.yaw
            return msg


# =============================================================================
# ================================   全局   ===================================
# =============================================================================
tele = Telemetry()
setpoint = SetpointState()

abort_flag = threading.Event()      # FATAL: 立即停桨
land_flag = threading.Event()       # LAND:  冻结 → 缓降 → AUTO.LAND
abort_reason = [None]
land_reason = [None]
exc_reason = [None]                 # 飞行序列里的普通异常 (不是 FATAL, 不停桨)

in_air = [False]
voluntary_land = [False]
# preflight [1/6] 读到的飞控模式。退出时 (地上已上锁) 还原成它, 见 restore_ground_mode。
start_mode = [None]

# 飞控 STATUSTEXT (PX4 的 "Preflight Fail: ..." / "Arming denied ..." 就在这里)。
# 元素 (t_ros_sec, severity, text)。arm 被拒时按时间窗取出来打印。
fcu_msgs = deque(maxlen=50)
fcu_msgs_lock = threading.Lock()
_MAV_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
               3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}


def statustext_cb(msg):
    try:
        with fcu_msgs_lock:
            fcu_msgs.append((rospy.Time.now().to_sec(), msg.severity, msg.text))
        # severity: 0 EMERGENCY … 3 ERROR, 4 WARNING, 6 INFO, 7 DEBUG
        if msg.severity <= 4:
            rospy.logwarn(f"[FCU] {msg.text}")
        elif msg.severity <= 6:
            rospy.loginfo(f"[FCU] {msg.text}")
    except Exception:
        pass


def fcu_texts_since(t0):
    with fcu_msgs_lock:
        return [txt for (ts, _sev, txt) in fcu_msgs if ts >= t0]
# 控制权被接管。值 = 接管时的飞控模式 (如 "POSCTL"), None = 没被接管。
#   已解锁后, 飞控模式变成**不是我们切的**模式 → 飞手(QGC/遥控)或 PX4 失效保护
#   拿走了控制权。从这一刻起脚本:
#     · 不再切任何模式 (否则会把飞手的控制权抢回来)
#     · 不再 force_disarm
#     · 只保持发布"原地悬停"设定点, 等飞机落地上锁 (见 phase_handover)
#   唯一例外: 操作员按 X —— 那是明确的人工停桨指令。
takeover = [None]


def allowed_modes():
    """脚本自己切过去的模式。已解锁时飞控处于这之外的模式 = 被接管。"""
    return ("OFFBOARD", "AUTO.LAND") if voluntary_land[0] else ("OFFBOARD",)
phase_ref = ["init"]
# 穿环段名 (wait / align / commit / dash / brake / hold / abort), 随 status 发给地面站 HUD
sub_mode_ref = [""]
envelope_dyn = [None]               # (horiz_lim, vel_lim) 或 None
# 冲刺段停桨线开关: GateRun 在按 G 之后置 True, 刹停悬停结束 / 放弃时置回 False
dash_tilt_on = [False]


def tilt_limit():
    """→ (停桨倾角 °, 去抖 s)。冲刺 / 制动 / 过环悬停用 GATE_DASH_TILT_DEG, 其余 MAX_TILT_DEG。"""
    if dash_tilt_on[0]:
        return C.GATE_DASH_TILT_DEG, C.GATE_DASH_TILT_CONFIRM_S
    return C.MAX_TILT_DEG, C.FATAL_CONFIRM_S

# 网页遥测快照。各处只做"整段替换"(一次赋值一个 dict), 读的一方拿到的永远是
# 某一时刻完整的一段。telem_put 吞掉一切异常 —— 遥测出错绝不能影响控制环和安全监控。
telem_lock = threading.Lock()
telem_data = {"safety": {}, "gate": {}}


def telem_put(section, d):
    try:
        d = dict(d)
        d["t"] = rospy.Time.now().to_sec()
        with telem_lock:
            telem_data[section] = d
    except Exception:
        pass


def telem_merge(section, **kw):
    try:
        with telem_lock:
            cur = dict(telem_data.get(section) or {})
            cur.update(kw)
            cur["t"] = rospy.Time.now().to_sec()
            telem_data[section] = cur
    except Exception:
        pass


def _f(x):
    """遥测里的数: nan/inf → None (JSON 不认 nan)。"""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None

INITIAL_X = INITIAL_Y = INITIAL_Z = INITIAL_YAW = 0.0

log_dir = None
csv_main = csv_vins = csv_ekf = csv_timing = csv_gate = None
writer_main = writer_vins = writer_ekf = writer_timing = writer_gate = None
zdrop_log_path = None
events_fh = None            # events.log 的 logging.FileHandler (fsync 用)
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


def request_abort(reason):
    """FATAL —— 空中停桨。只给姿态失控 / MAVROS 断链 / 操作员按 X 用。"""
    if not abort_flag.is_set():
        rospy.logerr(f"*** ABORT [{SEV_FATAL}]: {reason} — 立即停桨 ***")
        abort_reason[0] = reason
        abort_flag.set()


def request_land(reason):
    """
    LAND —— 可恢复级。冻结当前 setpoint，走 descend + AUTO.LAND (就地, 不回原点)。
    """
    if not land_flag.is_set() and not abort_flag.is_set():
        rospy.logerr(f"*** ABORT [{SEV_LAND}]: {reason} — 转入降落 ***")
        land_reason[0] = reason
        land_flag.set()


def stop_requested():
    return abort_flag.is_set() or land_flag.is_set()


def check_abort():
    """FATAL 才抛异常中断飞行序列；LAND 由各 phase 自己提前 return。"""
    if abort_flag.is_set():
        raise RuntimeError(abort_reason[0])


def set_phase(name, horiz=None, vel=None):
    """
    切阶段。horiz/vel 非 None 时动态覆盖该阶段的包络 —— 用于 descend /
    auto_land: 它们的基础水平硬限比 return_home 小, 而 return 超时时飞机
    还没回到原点, 不放宽的话进 descend 第一个 monitor tick 就越界。
    """
    if horiz is None and vel is None:
        envelope_dyn[0] = None
    else:
        h0, v0 = C.PHASE_ENVELOPE.get(name, C.PHASE_ENVELOPE["_default"])
        envelope_dyn[0] = (h0 if horiz is None else horiz,
                           v0 if vel is None else vel)
    phase_ref[0] = name


def get_envelope(phase):
    dyn = envelope_dyn[0]
    if dyn is not None:
        return dyn
    return C.PHASE_ENVELOPE.get(phase, C.PHASE_ENVELOPE["_default"])


# =============================================================================
# ===============================   Helpers   =================================
# =============================================================================
# 四元数 / 角度小函数: 2026-10-30 起公式在 vb09/common/geom.py (和监控、地面共用), 这里只是按 msg 取分量
def yaw_from_quat(q):
    return geom.yaw_from_quat(q.x, q.y, q.z, q.w)


wrap_pi = geom.wrap_pi
clamp = geom.clamp


def rpy_deg_from_quat(q):
    r, p, y = geom.rpy_from_quat(q.x, q.y, q.z, q.w, eps=1e-6)
    return math.degrees(r), math.degrees(p), math.degrees(y)


def tilt_deg_from_quat(q):
    return math.degrees(geom.tilt_from_quat(q.x, q.y, q.z, q.w, eps=1e-6))


def force_disarm_with_timeout(cmd_long, overall_timeout=2.0):
    result = {"ok": False}

    def _worker():
        for i in range(3):
            try:
                r = cmd_long(broadcast=False, command=400, confirmation=0,
                             param1=0.0, param2=21196.0,
                             param3=0.0, param4=0.0,
                             param5=0.0, param6=0.0, param7=0.0)
                rospy.logwarn(f"force_disarm attempt {i+1}: success={r.success}")
                if r.success:
                    result["ok"] = True
                    return
            except Exception as e:
                rospy.logerr(f"force_disarm attempt {i+1} exception: {e}")
            rospy.sleep(0.1)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    t.join(timeout=overall_timeout)
    if t.is_alive():
        rospy.logerr(f"force_disarm timed out after {overall_timeout}s")
    return result["ok"]


# =============================================================================
# ============================   键盘输入处理   ===============================
# =============================================================================
class KeyReader:
    """
    cbreak 模式读键盘。两条要点:

    1) 每轮读空缓冲区, 方向键只保留最后一个。终端按键重复约 30-40 字符/秒,
       和 30Hz 控制环基本持平; 每轮只读 1 字节的话一次调度打嗝就开始积压,
       松开 W 之后飞机还会按 W 飞几秒, dead-man 也永远不触发。

    2) SSH 断连后 read(1) 返回**空串**而不是抛异常 → 识别为 EOF, 调用方立即降落。
       RC 的 SWC/SWD 是坏的, 键盘是唯一的人工急停通道。
    """

    MOVE_KEYS = ('w', 's', 'a', 'd', 'r', 'f', 'q', 'e', ' ')

    def __init__(self):
        import termios
        import tty
        import select
        self._termios = termios
        self._tty = tty
        self._select = select
        self.fd = sys.stdin.fileno()
        self._old = None

    def __enter__(self):
        self._old = self._termios.tcgetattr(self.fd)
        self._tty.setcbreak(self.fd)
        self.drain()
        return self

    def __exit__(self, *exc):
        try:
            self._termios.tcsetattr(self.fd, self._termios.TCSADRAIN, self._old)
        except Exception as e:
            rospy.logwarn(f"[key] 恢复 tty 失败: {e}")
        return False

    def drain(self):
        while self._select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                if sys.stdin.read(1) == "":
                    return
            except Exception:
                return

    def poll(self):
        """
        读空缓冲区。返回 (move_key, flags, eof, n_read)
          move_key: 本批里最后一个方向/悬停键，没有则 None
          flags:    集合，含本批出现过的 'x' / 'l' / 't' / 'g' / 'm'
          eof:      stdin 已关闭 (SSH 断连)
        """
        move = None
        flags = set()
        eof = False
        n = 0
        while self._select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if ch == "":
                eof = True
                break
            n += 1
            k = ch.lower()
            if k in self.MOVE_KEYS:
                move = k
            elif k in ('x', 'l', 't', 'g', 'm', 'y'):
                flags.add(k)
            if n > 256:          # 极端积压，别把这一轮卡死
                break
        return move, flags, eof, n


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
    global log_dir, csv_main, csv_vins, csv_ekf, csv_timing, csv_gate
    global writer_main, writer_vins, writer_ekf, writer_timing, writer_gate
    global zdrop_log_path, events_fh

    base = os.path.expanduser(C.LOG_ROOT)
    log_dir, flight_seq = _make_flight_dir(base)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")      # 飞机时钟, 重启后会重置, 只作参考
    try:
        up = float(open("/proc/uptime").read().split()[0])
    except Exception:
        up = float("nan")
    try:
        with open(os.path.join(log_dir, "run_info.txt"), "w", encoding="utf-8") as f:
            f.write("flight_seq   = %d\n" % flight_seq)
            f.write("script       = %s\n" % os.path.basename(__file__))
            f.write("clock_local  = %s   # 飞机时钟, 重启后会重置, 不能当真实日期\n" % ts)
            f.write("ros_time     = %.3f\n" % rospy.Time.now().to_sec())
            f.write("uptime_s     = %.0f   # 本次开机至今; 同一次开机里的几次飞行可按它排先后\n" % up)
            f.flush()
            os.fsync(f.fileno())
    except Exception as _e:
        rospy.logwarn(f"写 run_info.txt 失败(不影响飞行): {_e}")

    csv_main = open(os.path.join(log_dir, "flight_log.csv"), "w", newline="")
    writer_main = csv.writer(csv_main)
    writer_main.writerow([
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

    csv_vins = open(os.path.join(log_dir, "vins_raw.csv"), "w", newline="")
    writer_vins = csv.writer(csv_vins)
    writer_vins.writerow([
        "t_wall", "t_stamp",
        "vins_x", "vins_y", "vins_z",
        "vins_vx", "vins_vy", "vins_vz",
        "vins_qx", "vins_qy", "vins_qz", "vins_qw",
        "mavros_z", "diff_z",
    ])

    csv_ekf = open(os.path.join(log_dir, "ekf2_health.csv"), "w", newline="")
    writer_ekf = csv.writer(csv_ekf)
    writer_ekf.writerow([
        "t_wall",
        "attitude_status_flag", "velocity_horiz_flag", "velocity_vert_flag",
        "pos_horiz_rel_flag", "pos_horiz_abs_flag",
        "pos_vert_abs_flag", "pos_vert_agl_flag",
        "const_pos_mode_flag", "pred_pos_horiz_rel_flag",
        "pred_pos_horiz_abs_flag",
        "gps_glitch_flag", "accel_error_flag",
        "landed_state", "vtol_state",
    ])

    csv_timing = open(os.path.join(log_dir, "timing.csv"), "w", newline="")
    writer_timing = csv.writer(csv_timing)
    writer_timing.writerow([
        "t_wall", "pose_stamp", "delay_ms",
        "interval_ms",
        "stamp_backward_count_total",
        "last_jump_cm",
    ])

    # 本次生效的现场配置写进日志目录 —— 复盘时必须知道当时用的是哪套限制。
    try:
        with open(os.path.join(log_dir, "site_config.txt"), "w", encoding="utf-8") as f:
            if _SITE_REPORT:
                f.write(_fp.site_config_text(_SITE_REPORT, all_params()))
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
    csv_gate = open(os.path.join(log_dir, "gate.csv"), "w", newline="")
    writer_gate = csv.writer(csv_gate)
    writer_gate.writerow(GATE_CSV_COLS)

    zdrop_log_path = os.path.join(log_dir, "z_drop_events.log")
    with open(zdrop_log_path, "w") as f:
        f.write(f"# z residual drop snapshots. flight_{flight_seq:04d}, 飞机时钟 {ts} (可能未校时)\n")
        f.write("# residual = z_filt - z_setpoint  (不是 z 本身)\n")
        f.write(f"# slow: {C.ZDROP_SLOW_THRESH*100:.0f}cm within {C.ZDROP_SLOW_WINDOW}s\n")
        f.write(f"# fast: {C.ZDROP_FAST_THRESH*100:.0f}cm within {C.ZDROP_FAST_WINDOW}s\n\n")

    # 文字日志落到运行目录。rospy.log* 走的是 Python 的 "rosout" logger, 挂个文件
    # handler 就全收到了 —— 包括 ROS 关闭之后、以及 rosout 聚合丢掉的行。
    # 不能因为写日志失败而不飞, 所以整段 try。
    try:
        fh = logging.FileHandler(os.path.join(log_dir, "events.log"), encoding="utf-8")
        fh.setFormatter(logging.Formatter("%(created).3f %(levelname)-5s %(message)s"))
        logging.getLogger("rosout").addHandler(fh)
        events_fh = fh
    except Exception as _e:
        rospy.logwarn(f"开 events.log 失败(不影响飞行): {_e}")

    rospy.loginfo(f"Log dir: {log_dir}  (编号 {flight_seq})")
    fsync_all_logs()            # 表头 / site_config.txt 先落盘


def fsync_all_logs():
    """
    ★2026-10-20: 所有日志 fsync 到盘。之前只有 flight_log.csv 每秒 fsync, 其余只 flush 到内核缓存;
    撞机后很快拔电池 (内核默认 ~30s 才回写, ext4 延迟分配) → gate.csv / events.log / vins_raw.csv
    全是 0 字节, 即使程序已经正常 close。fsync 不需要拿各文件的写锁 (各写线程每行自己 flush 过)。
    """
    fds = [f for f in (csv_main, csv_vins, csv_ekf, csv_timing, csv_gate) if f is not None]
    st = getattr(events_fh, "stream", None) if events_fh is not None else None
    if st is not None:
        fds.append(st)
    for f in fds:
        try:
            if not f.closed:
                os.fsync(f.fileno())
        except (OSError, ValueError):
            pass
    for path in (zdrop_log_path, os.path.join(log_dir, "site_config.txt") if log_dir else None):
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
    while not rospy.is_shutdown() and not abort_flag.is_set():
        fsync_all_logs()
        time.sleep(C.FSYNC_INTERVAL_S)


# =============================================================================
# ==============================   各路日志   =================================
# =============================================================================
def log_main_row():
    if writer_main is None:
        return
    try:
        with csv_lock_main:
            state, pose, pose_time, vel, batt, act, tgt, zf = tele.snap()
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

            writer_main.writerow([
                f"{t:.3f}", phase_ref[0],
                f"{px:.3f}", f"{py:.3f}", f"{pz:.3f}", f"{pz_filt:.3f}",
                f"{vx:.3f}", f"{vy:.3f}", f"{vz:.3f}",
                f"{roll:.1f}", f"{pitch:.1f}", f"{yaw:.1f}",
                f"{sp_px:.3f}", f"{sp_py:.3f}", f"{sp_pz:.3f}",
                f"{sp_vx:.3f}", f"{sp_vy:.3f}", f"{sp_vz:.3f}",
                f"{sp_yaw:.1f}", f"{sp_yaw_rate:.1f}",
                f"{act_r:.3f}", f"{act_p:.3f}", f"{act_y:.3f}", f"{thrust:.3f}",
                f"{batt_v:.2f}", mode, armed, f"{age_ms:.0f}",
            ])
            csv_main.flush()
            # fsync 按 FSYNC_INTERVAL_S 节流: 每行 fsync 会和其他 IO 叠加成阻塞源。
            if last_fsync[0] is None or (now - last_fsync[0]).to_sec() >= C.FSYNC_INTERVAL_S:
                try:
                    os.fsync(csv_main.fileno())
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
    if writer_vins is None:
        return
    try:
        with csv_lock_vins:
            odom, odom_time = tele.snap_vins()
            if odom is None:
                return
            stamp = odom.header.stamp
            if last_vins_stamp[0] is not None and stamp == last_vins_stamp[0]:
                return
            last_vins_stamp[0] = stamp

            _, pose, _, _, _, _, _, _ = tele.snap()
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

            writer_vins.writerow([
                f"{now.to_sec():.3f}", f"{t_stamp:.3f}",
                f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}",
                f"{vvx:.3f}", f"{vvy:.3f}", f"{vvz:.3f}",
                f"{q.x:.4f}", f"{q.y:.4f}", f"{q.z:.4f}", f"{q.w:.4f}",
                f"{mav_z:.4f}", f"{diff:.4f}",
            ])
            csv_vins.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"vins csv exception: {e}")


def log_ekf_row():
    if writer_ekf is None:
        return
    try:
        with csv_lock_ekf:
            est, ext = tele.snap_ekf()
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

            writer_ekf.writerow([
                f"{now.to_sec():.3f}",
                att, vh, vv, phr, pha, pva, pval, cpm, ppphr, ppha, gps, acc,
                landed, vtol,
            ])
            csv_ekf.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"ekf csv exception: {e}")


def drain_gate_queue():
    """gate.csv: 把队列里攒的行写盘。只在 gate_logger_loop / cleanup 里调。"""
    if writer_gate is None or not gate_queue:
        return
    rows = []
    try:
        while True:
            rows.append(gate_queue.popleft())
    except IndexError:
        pass
    try:
        with csv_lock_gate:
            writer_gate.writerows(rows)
            csv_gate.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"gate csv exception: {e}")


def gate_logger_loop():
    while not rospy.is_shutdown() and not abort_flag.is_set():
        drain_gate_queue()
        time.sleep(0.1)
    drain_gate_queue()


def drain_timing_queue():
    """后台线程批量落盘。pose 回调只负责 append。"""
    if writer_timing is None:
        return
    with timing_queue_lock:
        if not timing_queue:
            return
        batch = list(timing_queue)
        timing_queue.clear()
    try:
        with csv_lock_timing:
            for (t_wall, stamp, delay_ms, interval_ms, back_cnt, jump) in batch:
                writer_timing.writerow([
                    f"{t_wall:.3f}", f"{stamp:.3f}", f"{delay_ms:.1f}",
                    f"{interval_ms:.1f}" if interval_ms == interval_ms else "nan",
                    back_cnt, f"{jump*100:.1f}",
                ])
            csv_timing.flush()
    except Exception as e:
        rospy.logwarn_throttle(C.THROTTLE_S, f"timing csv exception: {e}")


def dump_zdrop_snapshot(reason, slow_drop, fast_drop):
    if zdrop_log_path is None:
        return
    try:
        with zdrop_lock:
            state, pose, pose_time, vel, batt, act, tgt, zf = tele.snap()
            odom, odom_time = tele.snap_vins()
            est, ext = tele.snap_ekf()
            now = rospy.Time.now()
            z_sp = setpoint.z_setpoint()

            with open(zdrop_log_path, "a") as f:
                f.write(f"\n=== Z_DROP EVENT @ t={now.to_sec():.3f} "
                        f"phase={phase_ref[0]} ===\n")
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
                f.write(f"stamp_backward_count: {tele.stamp_backward_count}\n")
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
    while not rospy.is_shutdown() and not abort_flag.is_set():
        log_main_row()
        zf = tele.z_filtered()
        z_sp = setpoint.z_setpoint()
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
    while not rospy.is_shutdown() and not abort_flag.is_set():
        log_vins_row()
        rate.sleep()


def ekf_logger_loop():
    rate = rospy.Rate(C.EKF_LOG_HZ)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        log_ekf_row()
        rate.sleep()


def timing_logger_loop():
    rate = rospy.Rate(C.TIMING_LOG_HZ)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        drain_timing_queue()
        rate.sleep()
    drain_timing_queue()


def publisher_loop(pub):
    rate = rospy.Rate(C.SP_PUB_HZ)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        try:
            pub.publish(setpoint.build_msg())
        except Exception as e:
            rospy.logerr(f"publisher exception: {e}")
        rate.sleep()


def status_loop(spub):
    """阶段状态 → vision_bridge → 地面站 HUD。纯观测，不参与控制。"""
    rate = rospy.Rate(C.STATUS_HZ)
    while not rospy.is_shutdown():
        try:
            spub.publish(String(json.dumps({
                "phase": phase_ref[0],
                "sub_mode": sub_mode_ref[0],     # 穿环段名 (见 sub_mode_ref)
                "air":   bool(in_air[0]),
                "abort": bool(abort_flag.is_set()),
                "land":  bool(land_flag.is_set()),
                "shadow": bool(C.GATE_SHADOW),
                "z_track": False,
                "task": C.TASK,
            }, ensure_ascii=False)))
        except Exception:
            pass
        rate.sleep()


def build_telemetry():
    """网页用的完整快照。只读全局, 不改任何状态。"""
    with telem_lock:
        sections = {k: dict(v) for k, v in telem_data.items()}
    mode, p, v, a, yaw = setpoint.snapshot()
    sp = {"mode": {SetpointState.MODE_VELOCITY: "VEL", SetpointState.MODE_ZHOLD: "ZHOLD",
                   SetpointState.MODE_TRAJ: "TRAJ"}.get(mode, "?"),
          "px": _f(p[0]), "py": _f(p[1]), "pz": _f(p[2]),
          "yaw": _f(yaw), "vx": _f(v[0]), "vy": _f(v[1]), "vz": _f(v[2]),
          "ax": _f(a[0]), "ay": _f(a[1]), "az": _f(a[2])}
    h, vl = get_envelope(phase_ref[0])
    return {
        "t": rospy.Time.now().to_sec(),
        "node": rospy.get_name(),             # 网页靠它过滤 /rosout 事件
        "task": C.TASK,
        "schema": 2,
        "phase": phase_ref[0],
        "sub_mode": sub_mode_ref[0],
        "air": bool(in_air[0]),
        "abort": bool(abort_flag.is_set()),
        "land": bool(land_flag.is_set()),
        "abort_reason": abort_reason[0],
        "land_reason": land_reason[0],
        "takeover": takeover[0],              # 被接管时 = 飞控模式名
        "t_requested": False,
        "shadow": bool(C.GATE_SHADOW),
        "sim_drive": bool(C.GATE_ALLOW_SIM_DRIVE),
        "z_enable": False,
        "origin": [_f(INITIAL_X), _f(INITIAL_Y), _f(INITIAL_Z), _f(INITIAL_YAW)],
        "setpoint": sp,
        "z_filt": _f(tele.z_filtered()),
        "cells": C.cell_count[0],
        "fly_min_v": _f(C.CELL_MIN_FLY * C.cell_count[0]),
        "envelope": [_f(h), _f(vl)],
        "log_dir": log_dir,
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
        fast = phase_ref[0] == "gate" and sub_mode_ref[0] in ("commit", "dash", "brake", "hold")
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
        if _fp is not None:
            derived = _fp.derived_snapshot(vars(C))
        else:
            derived = {"phase_envelope": {k: list(v) for k, v in C.PHASE_ENVELOPE.items()}}
        rep = _SITE_REPORT or {}
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
            "origin": [_f(INITIAL_X), _f(INITIAL_Y), _f(INITIAL_Z), _f(INITIAL_YAW)],
            "log_dir": log_dir,
        }, ensure_ascii=False)))
    except Exception as e:
        rospy.logwarn(f"[telemetry] 发布配置失败(不影响飞行): {e}")


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

    while not rospy.is_shutdown() and not abort_flag.is_set():
        state, pose, pose_time, vel, batt, _, _, zf = tele.snap()
        now = rospy.Time.now()
        horiz_lim, vel_lim = get_envelope(phase_ref[0])

        # ---------- FATAL ----------
        if state is not None and not state.connected:
            request_abort("MAVROS lost FCU connection")
            break

        if pose is not None:
            z_use = zf if zf is not None else pose.pose.position.z
            dz = z_use - INITIAL_Z

            dx = pose.pose.position.x - INITIAL_X
            dy = pose.pose.position.y - INITIAL_Y
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
                       f"(phase={phase_ref[0]}, 持续{C.FATAL_CONFIRM_S:.2f}s)")
                if C.FENCE_VIOLATION_LANDS:
                    request_land(msg)
                else:
                    request_abort(msg)
                    break

            # 姿态 —— 始终 FATAL (缓降在这种情况下也执行不了)。
            # 冲刺段 (按 G 之后) 换成 GATE_DASH_TILT_DEG: 急刹 + 撞纸的短时大倾角不该停桨
            t_lim, t_hold = tilt_limit()
            db_on, db_off = (deb_tilt_dash, deb_tilt) if dash_tilt_on[0] else (deb_tilt, deb_tilt_dash)
            db_off.check(False, now)
            if db_on.check(tilt > t_lim, now):
                request_abort(f"tilt {tilt:.1f}deg > {t_lim:.0f}deg "
                              f"(持续{t_hold:.2f}s{', 冲刺段' if dash_tilt_on[0] else ''})")
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
            _, odom_time = tele.snap_vins()
            if odom_time is not None:
                odom_age = (now - odom_time).to_sec()
                if odom_age > C.VISION_LOST_LAND_S:
                    request_land(
                        f"视觉断供 {odom_age:.2f}s (VINS 已停止输出 odometry) — "
                        f"无外部位置观测, 位置环会追惯性漂移, 主动缓降")

        jump = tele.take_jump_max()
        if jump > C.POSE_JUMP_MAX:
            request_land(f"VIO jump {jump*100:.0f}cm")

        if vel is not None:
            v = vel.twist.linear
            horiz_v = math.hypot(v.x, v.y)
            if horiz_v > vel_lim:
                request_land(f"horiz vel {horiz_v:.2f}m/s > {vel_lim:.2f}m/s "
                             f"(phase={phase_ref[0]})")

        if batt is not None and batt.voltage > 0:
            fly_min = C.CELL_MIN_FLY * C.cell_count[0]
            if batt.voltage < fly_min:
                request_land(f"battery {batt.voltage:.2f}V < {fly_min:.2f}V")

        if (state is not None and state.armed
                and state.mode not in allowed_modes()):
            # 模式被改: 可能是飞手切到 POSCTL 救场, 也可能是 PX4 失效保护。
            # 别停桨，别抢控制权: 记下接管, 各阶段看到 takeover 就停手, 交给
            # phase_handover。脚本自己切到 AUTO.LAND 之后飞手再切走, 同样算接管。
            if takeover[0] is None:
                takeover[0] = state.mode
                rospy.logerr(f"*** 控制权被接管: 飞控模式 → {state.mode} "
                             f"(不是脚本切的) — 脚本不再切模式、不再停桨 ***")
            request_land(f"mode changed to {state.mode} — 控制权被接管")

        # 网页"安全余量"面板。只读上面已经拿到的快照, 自己重算一遍显示值,
        # 不碰任何判定逻辑; 整段 try, 出错只丢这一帧遥测。
        try:
            _t = {"horiz_lim": _f(horiz_lim), "vel_lim": _f(vel_lim),
                  "max_height": _f(C.MAX_HEIGHT), "max_tilt": _f(tilt_limit()[0]),
                  "max_tilt_base": _f(C.MAX_TILT_DEG), "max_tilt_dash": _f(C.GATE_DASH_TILT_DEG),
                  "dash_tilt": bool(dash_tilt_on[0]),
                  "pose_stale": _f(C.POSE_STALE_SEC), "vision_lost": _f(C.VISION_LOST_LAND_S),
                  "jump_max": _f(C.POSE_JUMP_MAX), "jump": _f(jump),
                  "deb": {"height": deb_height.n, "horiz": deb_horiz.n,
                          "tilt": (deb_tilt_dash if dash_tilt_on[0] else deb_tilt).n},
                  "confirm_s": _f(tilt_limit()[1])}
            if pose is not None:
                _zu = zf if zf is not None else pose.pose.position.z
                _t["dz"] = _f(_zu - INITIAL_Z)
                _t["horiz"] = _f(math.hypot(pose.pose.position.x - INITIAL_X,
                                            pose.pose.position.y - INITIAL_Y))
                _t["tilt"] = _f(tilt_deg_from_quat(pose.pose.orientation))
            if pose_time is not None:
                _t["pose_age"] = _f((now - pose_time).to_sec())
            _, _ot = tele.snap_vins()
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


# =============================================================================
# ============================   参数摘要打印   ===============================
# =============================================================================
def gp_r_lim(axis):
    """机体中心最多能偏离环心多远 (只偏一个方向时): axis 0 = 左右, 1 = 上下。"""
    r = C.R_IN - C.AIM_MARGIN
    a, b = (C.H_HALF, C.W_HALF) if axis == 0 else (C.W_HALF, C.H_HALF)
    return max(0.0, math.sqrt(max(r * r - a * a, 0.0)) - b)


def print_config_summary():
    """arm 之前把生效参数念一遍。让程序自己把最终状态报出来, 比靠人读注释可靠。"""
    L = rospy.loginfo
    W = rospy.logwarn
    pv = gp.static_preview(gp.params_from_globals(vars(C)))
    L("=" * 64)
    L("生效参数摘要 (穿环)")
    L("-" * 64)
    L(f"  机体: 机头 {C.L_F:.2f} 机尾 {C.L_R:.2f} 半宽 {C.W_HALF:.2f} 半高 {C.H_HALF:.2f} m  → 机体中心离环心 "
      f"左右 ≤ {gp_r_lim(0):.2f} / 上下 ≤ {gp_r_lim(1):.2f} m")
    if C.NEEDLE_RIGHT or C.NEEDLE_UP or C.AIM_TRIM_RIGHT or C.AIM_TRIM_UP:
        W(f"  撞击点: 针尖在机体中心 右 {C.NEEDLE_RIGHT*100:+.0f} / 上 {C.NEEDLE_UP*100:+.0f} cm,  "
          f"经验修正 右 {C.AIM_TRIM_RIGHT*100:+.0f} / 上 {C.AIM_TRIM_UP*100:+.0f} cm  "
          f"→ 机体中心瞄在目标 右 {(C.AIM_TRIM_RIGHT-C.NEEDLE_RIGHT)*100:+.0f} / 上 {(C.AIM_TRIM_UP-C.NEEDLE_UP)*100:+.0f} cm")
    else:
        L("  撞击点: 机体中心对准目标 (NEEDLE_* / AIM_TRIM_* 都是 0)")
    L(f"  场地: 环后 {C.D_BACK:.2f}  起飞点后方 {C.D_REAR:.2f}  两侧 {C.D_SIDE:.2f} m  "
      f"水平范围 {C.GATE_XY_MAX:.2f} m")
    L(f"  冲刺: ≤{C.DASH_V:.2f} m/s (最低降到 {C.DASH_V_MIN:.2f})  加 {C.DASH_ACC:.1f} / 减 {C.BRAKE_ACC:.1f} m/s²  "
      f"T_LAG {C.T_LAG:.2f}s")
    vp = pv.get("v_plan", C.DASH_V)
    (W if vp < C.DASH_V - 1e-9 or pv.get("v_room") is None else L)(
        f"  正对进入时: {vp:.2f} m/s" + ("" if vp >= C.DASH_V - 1e-9 else
                                      f" (环后空间限速; 按 {C.DASH_V:.2f} 冲要 D_BACK ≥ {pv['d_back_need']:.2f} m)")
        + f"  助跑 {pv['d_align']:.2f} m")
    L(f"             过纸冲刺 {C.PASS_RUN:.2f} m (+滞后补偿 {pv.get('pass_lag', 0):.2f})  "
      f"刹车起点 s={pv.get('s_brake', float('nan')):+.2f}  停止 s={pv['overshoot']:+.2f}  "
      f"含超调 {pv.get('brake_over', 0):.2f} m 后机头离墙 {C.M_BACK + pv['tol']:.2f} m (要求 ≥ {C.M_BACK:.2f})")
    L("             场地不够时: " + {0: "不降速, 规划失败回手动段", 1: "降速, 冲刺前要按 Y 确认",
                                   2: "自动降速"}.get(int(C.GATE_SLOW_POLICY), "?"))
    if "v_hit" in pv:
        L(f"             撞纸 {pv['v_hit']:.2f} m/s (撕纸下限 {C.V_TEAR_MIN:.2f})  冲刺 {pv['t_dash']:.2f}s")
    L(f"  识别: 距离 [{C.GATE_D_MIN:.1f}, {C.GATE_D_MAX:.1f}] m  单帧σ 横<{C.GATE_SD_MAX_CM:.0f}/深<{C.GATE_SD_ALONG_MAX_CM:.0f}cm (环心观测放宽到 {C.GATE_SD_ALONG_PER_M_CM:.1f}cm×距离)  "
      f"锁定σ 横<{C.GATE_LOCK_SD_LAT_CM:.0f}/深<{C.GATE_LOCK_SD_ALONG_CM:.0f}cm  "
      f"锁定离散 横<{C.GATE_ALIGN_SPREAD_CM:.0f}/深<{C.GATE_ALIGN_SPREAD_ALONG_CM:.0f}cm  "
      f"{'必须有法向' if C.GATE_REQUIRE_NORMAL else '没有法向就按视线方向进入'}")
    L("-" * 64)
    if C.GATE_SHADOW:
        L("  ✓ 影子 GATE_SHADOW=True —— 识别/锁定/规划照做, 不摆位、不冲刺")
    else:
        W("  ✗ 影子 GATE_SHADOW=False —— !!! 锁定后会摆位、冲刺 !!!")
    if C.GATE_VIRTUAL_OFFSET > 0:
        W(f"  ★ 虚拟环: 环平面前移 {C.GATE_VIRTUAL_OFFSET:.2f} m, 停在纸前")
    if C.GATE_CONFIRM_DASH:
        L("  ✓ 冲刺前要按 G 确认")
    else:
        W("  ✗ GATE_CONFIRM_DASH=False —— 锁定后直接冲刺, 不等确认")
    if C.GATE_ALLOW_SIM_DRIVE:
        W("  ✗ GATE_ALLOW_SIM_DRIVE=True —— !!! sim 标记的识别也能驱动冲刺 !!!")
    L("-" * 64)
    for name in ("takeoff", "gate", "descend", "auto_land"):
        h, v = C.PHASE_ENVELOPE[name]
        L(f"  包络 {name:<10s} 水平<{h:.2f}m  速度<{v:.2f}m/s")
    L(f"  冲刺前 gate 段速度包络收紧到 {C.GATE_PRE_VEL_LIM:.2f} m/s")
    L(f"  高度硬顶 {C.MAX_HEIGHT:.2f}m  倾角停桨 {C.MAX_TILT_DEG:.0f}° (持续 {C.FATAL_CONFIRM_S:.2f}s)  "
      f"冲刺段 {C.GATE_DASH_TILT_DEG:.0f}° (持续 {C.GATE_DASH_TILT_CONFIRM_S:.2f}s)")
    if _SITE_REPORT:
        L("-" * 64)
        L(f"  现场档案: {_SITE_REPORT['src']}")
        for _ln in _SITE_REPORT["lines"]:
            L(f"    · {_ln}")
        for _w in _SITE_REPORT["warnings"]:
            W(f"    ⚠️ {_w}")
    else:
        W("  现场档案: flight_profile.py 不在 —— 用的是代码默认值, 且未经任何校验")
    L(f"  电池 {C.cell_count[0]}S  低压门槛 {C.CELL_MIN_FLY * C.cell_count[0]:.1f}V")
    L("=" * 64)


# =============================================================================
# ==============================   Preflight   ================================
# =============================================================================
PX4_CHECK_PARAMS = ("COM_OBL_ACT", "MPC_TILTMAX_AIR", "MPC_XY_VEL_MAX")
PX4_LOG_PARAMS = ("MPC_ACC_HOR_MAX", "MPC_JERK_AUTO", "MPC_XY_P", "MPC_XY_VEL_P_ACC")
PX4_PULL_TIMEOUT = 30.0      # s，等 mavros 参数缓存就绪


def _fmt_px4(v):
    return ("%.3f" % v).rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def _param_value(r):
    """mavros ParamGet 回复 → 数。整数参数在 integer, 浮点参数在 real (另一个是 0)。"""
    v = r.value
    return float(v.real) if float(v.real) != 0.0 else int(v.integer)


def read_px4_params(param_get, param_pull, names, optional=PX4_LOG_PARAMS):
    """
    读 PX4 参数。任何一个 names 读不到 → 先 param/pull (不强制; 缓存空就拉, 正在拉就等)
    再读一遍; 还读不到 → 抛 RuntimeError。optional 只记日志, 读不到不管。→ {名字: 值}
    """
    def get_all(ns):
        got, miss = {}, []
        for k in ns:
            try:
                r = param_get(param_id=k)
                ok = bool(r.success)
            except Exception:
                ok = False
            if ok:
                got[k] = _param_value(r)
            else:
                miss.append(k)
        return got, miss

    got, miss = get_all(names)
    if miss:
        rospy.logwarn(f"    参数缓存还没就绪 (读不到 {', '.join(miss)}) —— param/pull, "
                      f"最多等 {PX4_PULL_TIMEOUT:.0f}s ...")
        if param_pull is None:
            raise RuntimeError("读不到 %s, 也没有 /mavros/param/pull" % ", ".join(miss))
        t0 = time.time()
        n = 0
        try:
            r = param_pull(force_pull=False)
            n = int(getattr(r, "param_received", 0))
        except Exception as e:
            rospy.logwarn(f"    param/pull 出错: {e}")
        while time.time() - t0 < PX4_PULL_TIMEOUT:
            got2, miss = get_all(miss)
            got.update(got2)
            if not miss:
                break
            time.sleep(1.0)
        if miss:
            raise RuntimeError("pull 后仍读不到 %s (param_received=%d)" % (", ".join(miss), n))
        rospy.loginfo(f"    参数缓存就绪 (param_received={n}, 等了 {time.time() - t0:.1f}s)")
    if optional:
        extra, _ = get_all(optional)
        got.update(extra)
    return got


def preflight_plan(st, age):
    """
    [4/6] 的一部分 (2026-10-21): 地面上已经看到环 → 先按 DASH_V 规划一遍, 不用飞起来才发现放不下。
    放得下 → 打印一行; 放不下 → 说清楚卡在哪、要按设定速度冲需要改成多少, 让操作员选
      c = 照样起飞 (allow_slow 是"确认"/"是" 时空中会降到能飞的速度; "否" 时空中会规划失败)
      q = 退出去改档案
    看不到环 / 识别太旧 → 跳过 (返回 True)。返回 False = 操作员选了退出。
    """
    try:
        rw = st.get("ring_w") or st.get("target")
        if rw is None or age > 1.0:
            rospy.loginfo("    (地面看不到环 —— 冲刺速度要起飞后才知道放不放得下)")
            return True
        c = tuple(float(v) for v in rw[:3])
        aim = tuple(float(v) for v in st["aim"][:3]) if st.get("aim") else c
        n = tuple(float(v) for v in st["n"][:3]) if st.get("n") else None
        r = gp.preflight_check(aim, c, n, (INITIAL_X, INITIAL_Y, INITIAL_Z),
                               gp.params_from_globals(vars(C)), C.TAKEOFF_HEIGHT,
                               virtual_offset=C.GATE_VIRTUAL_OFFSET)
    except Exception as e:
        rospy.logwarn(f"    起飞前预规划出错 (跳过, 起飞后照常规划): {e}")
        return True
    dist = math.hypot(c[0] - INITIAL_X, c[1] - INITIAL_Y)
    telem_put("gate", {"seg": "preflight", "pre": {
        "ok_full": r["ok_full"], "v_set": _f(C.DASH_V), "v_room": _f(r["v_room"]), "why": r["why"],
        "xy_need": _f(r["xy_need"]), "d_back_need": _f(r["d_back_need"]), "ring_dist": _f(dist)}})
    if r["ok_full"]:
        rospy.loginfo(f"    ✓ 地面看到环 (水平 {dist:.1f} m): 按 {C.DASH_V:.2f} m/s 放得下 —— {r['summary']}")
        return True
    vr = r["v_room"]
    rospy.logerr("    " + "!" * 66)
    rospy.logerr(f"    ★ 地面看到环 (水平 {dist:.1f} m), 按设定 {C.DASH_V:.2f} m/s 放不下: {r['why']}")
    rospy.logerr("      场地最高能冲 " + (f"{vr:.2f} m/s" if vr is not None else "—— 降到最低速也不行"))
    fix = []
    if r["xy_need"] is not None and r["xy_need"] > C.GATE_XY_MAX:
        fix.append(f"xy_radius ≥ {r['xy_need']:.1f} (现在 {C.GATE_XY_MAX:.1f}), 或起飞点往环挪 {r['xy_need'] - C.GATE_XY_MAX:.1f} m")
    dbn = r["d_back_need"]
    if dbn > (C.GATE_VIRTUAL_OFFSET if C.GATE_VIRTUAL_OFFSET > 0 else C.D_BACK) + 1e-6:
        fix.append(f"{'虚拟环偏移' if C.GATE_VIRTUAL_OFFSET > 0 else 'd_back'} ≥ {dbn:.2f}")
    if vr is not None:
        fix.append(f"或 dash_v 改成 {vr:.2f}")
    rospy.logerr(f"      要按 {C.DASH_V:.2f} 冲: " + ";  ".join(fix))
    act = {0: "空中会规划失败、回手动段", 1: f"空中会降到约 {vr or 0:.2f} m/s, 冲刺前要按 Y 确认",
           2: f"空中会自动降到约 {vr or 0:.2f} m/s"}.get(int(C.GATE_SLOW_POLICY), "")
    rospy.logerr(f"      (地面识别位置可能差 10~20 cm, 起飞后会按空中识别重算。照样起飞的话: {act})")
    rospy.logerr("    " + "!" * 66)
    _flush_stdin()
    try:
        ans = input("    输入 c 回车 = 照样起飞,  其它 = 退出去改档案: ").strip().lower()
    except EOFError:
        ans = ""
    if ans != "c":
        rospy.logerr("    操作员选择退出 —— 改好档案再起飞")
        return False
    rospy.logwarn("    操作员选择照样起飞")
    return True


def preflight(param_get, set_mode, param_pull=None):
    global INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_YAW
    rospy.loginfo("=" * 64)
    rospy.loginfo("Preflight")
    rospy.loginfo("=" * 64)

    rospy.loginfo("[1/6] MAVROS link...")
    t_end = rospy.Time.now() + rospy.Duration(5.0)
    state = None
    while rospy.Time.now() < t_end:
        state = tele.state
        if state and state.connected:
            break
        rospy.sleep(0.1)
    if state is None or not state.connected:
        rospy.logerr("    not connected")
        return False
    rospy.loginfo(f"    connected  mode={state.mode}  armed={state.armed}")
    if state.armed:
        rospy.logerr("    already armed")
        return False
    start_mode[0] = state.mode
    # 开局就是 OFFBOARD / AUTO.LAND = 上一次运行没正常收尾 (被强杀/断电/崩溃) 留下的,
    # PX4 里可能还挂着 offboard 丢失保护。地上未解锁, 切模式没有风险; 切不过去也不拦
    # (后面马上就切 OFFBOARD), 只记一笔。
    # AUTO.LAND 是正常降落后的收尾模式 (§5: 室内没遥控、没全局位置时 PX4 不肯切回 MANUAL /
    # LOITER, 脚本就把它留在 AUTO.LAND) —— 不算遗留, 不切。
    if state.mode == "AUTO.LAND":
        rospy.loginfo("    开局模式 AUTO.LAND (上次正常降落后留下的), 不用切")
    if state.mode == "OFFBOARD":
        rospy.logwarn(f"    ★ 开局模式 {state.mode} 是上一次运行遗留的 (上次没正常收尾) "
                      f"→ 先切 {C.PARK_MODE}")
        try:
            set_mode(custom_mode=C.PARK_MODE)
            t_end = rospy.Time.now() + rospy.Duration(1.5)
            while rospy.Time.now() < t_end:
                if tele.state and tele.state.mode == C.PARK_MODE:
                    break
                rospy.sleep(0.05)
            cur = tele.state.mode if tele.state else "?"
            if cur == C.PARK_MODE:
                rospy.loginfo(f"    已切到 {C.PARK_MODE}")
            else:
                rospy.logwarn(f"    没切成 (当前 {cur}) —— 不影响起飞, 后面会切 OFFBOARD")
        except Exception as e:
            rospy.logwarn(f"    切 {C.PARK_MODE} 异常 (不影响起飞): {e}")

    # 电池串数自动识别 (见 CELL_V_SPLIT)。写在 [1/6] 里, 不打乱 [n/6] 编号。
    t_end = rospy.Time.now() + rospy.Duration(3.0)
    bv = 0.0
    while rospy.Time.now() < t_end:
        _b = tele.snap()[4]                 # snap() 第 5 个 = battery
        if _b is not None and _b.voltage > 0:
            bv = _b.voltage
            break
        rospy.sleep(0.1)
    _n = cells_from_voltage(bv)
    if _n is None:
        rospy.logwarn(f"    ★ 读不到电池电压 → 串数按默认 {C.N_CELLS}S, "
                      f"低压门槛 {C.CELL_MIN_FLY * C.N_CELLS:.1f}V")
        rospy.logwarn("      ★ 若这架不是 4S 电池, 低压保护会失效, 请手动确认 ★")
    else:
        C.cell_count[0] = _n
        rospy.loginfo(f"    电池 {bv:.2f}V → 判为 {C.cell_count[0]}S "
                      f"(≈{bv / C.cell_count[0]:.2f}V/芯)  低压门槛 "
                      f"{C.CELL_MIN_FLY * C.cell_count[0]:.1f}V")

    rospy.loginfo("[2/6] VIO pose (/mavros/local_position/pose)...")
    t_end = rospy.Time.now() + rospy.Duration(C.POSE_WAIT_TIMEOUT)
    while rospy.Time.now() < t_end and tele.pose is None:
        rospy.sleep(0.1)
    if tele.pose is None:
        rospy.logerr("    no pose. is VINS running?")
        return False
    rospy.loginfo("    pose received")

    rospy.loginfo("[3/6] VINS raw odometry (/vins_estimator/odometry)...")
    t_end = rospy.Time.now() + rospy.Duration(3.0)
    while rospy.Time.now() < t_end and tele.vins_odom is None:
        rospy.sleep(0.1)
    if tele.vins_odom is None:
        rospy.logwarn("    NO vins_estimator/odometry. 诊断日志不完整，飞行可继续。")
        rospy.logwarn("    检查: rostopic list | grep vins")
    else:
        rospy.loginfo("    vins odometry received")

    # 先确定 origin —— 目标 z 门限需要 INITIAL_Z
    p0 = tele.pose.pose.position
    INITIAL_X, INITIAL_Y, INITIAL_Z = p0.x, p0.y, p0.z
    INITIAL_YAW = yaw_from_quat(tele.pose.pose.orientation)

    # ---------------- 图传模式 + 识别门禁 ----------------
    rospy.loginfo(f"[4/6] 图传模式 ({C.CAM_STATUS_TOPIC}) + 环识别 ({C.GATE_STATUS_TOPIC})...")
    t_end = rospy.Time.now() + rospy.Duration(3.0)
    while rospy.Time.now() < t_end and gate_in.cam_mode() is None:
        rospy.sleep(0.1)
    mode = gate_in.cam_mode()
    if mode not in ("gate", "gate_gcs"):
        rospy.logerr(f"    图传模式 = {mode or '收不到 /vision_bridge/cam (桥没在跑?)'} —— 穿环要 "
                     f"gate_gcs (地面检测) 或 gate (机上检测)。拒绝起飞")
        rospy.logerr("    重起: python3 run_vision.py gate_gcs --gcs <地面站IP>")
        return False
    rospy.loginfo(f"    图传模式 {mode}")
    t_end = rospy.Time.now() + rospy.Duration(2.0)
    while rospy.Time.now() < t_end and gate_in.n_msgs == 0:
        rospy.sleep(0.1)
    st, age = gate_in.latest()
    if st is None:
        rospy.logwarn("    还没收到 /gate/status —— 起飞后再找环 (地面 gcs_gate.py 在跑吗?)")
    else:
        rospy.loginfo(f"    /gate/status: 阶段 {st.get('stage')}  距离 {st.get('dist')}  "
                      f"sim={st.get('sim')}  法向 {'有' if st.get('n') is not None else '没有'}  "
                      f"红点 {'有' if st.get('aim') is not None else '没有'}")
        # 桥里的环估计是旧的 (首飞问题 §2: 落地后滤波器只 predict 不 update, 年龄几百秒、
        # σ 上米还在发) → 拒绝。新版 vision_bridge 会自己重置; 老版要重启桥。
        if age > C.GATE_STATUS_STALE_S:
            rospy.logwarn(f"    ★ /gate/status 最新一条是 {age:.1f}s 前收到的 (> {C.GATE_STATUS_STALE_S:.1f}s) —— "
                          f"地面检测停了 / 桥卡住了? 起飞后找环会一直等")
        t_age = st.get("age_ms")
        if st.get("target") is not None and isinstance(t_age, (int, float)) and \
                t_age > C.GATE_PREFLIGHT_TARGET_AGE_MS:
            rospy.logerr(f"    桥里的环估计已经 {t_age / 1000:.1f}s 没更新 (σ {st.get('sd_cm')} cm) —— "
                         f"旧状态会带进这次飞行。重启桥 (run_vision.py) 或换新版 vision_bridge.py "
                         f"(会自动重置)。拒绝起飞")
            return False
        d0 = st.get("dist")
        if (st.get("target") is not None and isinstance(d0, (int, float))
                and age < 1.0 and d0 < C.D_ALIGN_MIN - 0.3):
            if C.GATE_MANUAL_START:
                rospy.logwarn(f"    ★ 起飞点离环只有 {d0:.2f} m —— 起飞后先在手动段往后退到 ≥ "
                              f"{C.D_ALIGN_MIN:.1f} m 再按 T")
            else:
                rospy.logerr(f"    起飞点离环只有 {d0:.2f} m (< {C.D_ALIGN_MIN - 0.3:.2f}) —— "
                             f"往后挪, 拒绝起飞")
                return False
        if st.get("sim") and not (C.GATE_SHADOW or C.GATE_ALLOW_SIM_DRIVE):
            rospy.logwarn("    ★ 识别结果标了 sim (地面 gcs_gate --mode 1 / 机上 _gate_sim:=true) —— "
                          "这样的帧不会用来冲刺, 起飞后会一直等不到有效识别。"
                          "要真飞: gate_gcs 地面用 --mode 2; gate (机上检测) 用 run_vision.py gate --drive")
        if st.get("n") is None and C.GATE_REQUIRE_NORMAL:
            rospy.logerr("    GATE_REQUIRE_NORMAL=True 但识别结果里没有法向 n —— 拒绝起飞")
            return False
        if not preflight_plan(st, age):
            return False

    rospy.loginfo(f"[5/6] Static drift ({C.STATIC_CHECK_S}s, "
                  f"xy_std<{C.STATIC_XY_STD_MAX*100:.0f}cm "
                  f"z_std<{C.STATIC_Z_STD_MAX*100:.0f}cm)...")
    xs, ys, zs, tilts = [], [], [], []
    t_end = rospy.Time.now() + rospy.Duration(C.STATIC_CHECK_S)
    while rospy.Time.now() < t_end:
        if tele.pose:
            p = tele.pose.pose.position
            xs.append(p.x); ys.append(p.y); zs.append(p.z)
            tilts.append(tilt_deg_from_quat(tele.pose.pose.orientation))
        rospy.sleep(0.05)
    if len(xs) < 10:
        rospy.logerr("    not enough samples")
        return False
    if max(tilts) > C.STATIC_TILT_MAX_DEG:
        rospy.logerr(f"    ground tilt {max(tilts):.1f}deg")
        return False
    xy_std = math.hypot(statistics.stdev(xs), statistics.stdev(ys))
    z_std = statistics.stdev(zs)
    rospy.loginfo(f"    xy_std={xy_std*100:.2f}cm  z_std={z_std*100:.2f}cm  "
                  f"(z peak-to-peak={((max(zs)-min(zs))*100):.1f}cm)")
    if xy_std > C.STATIC_XY_STD_MAX:
        rospy.logerr(f"    xy_std > {C.STATIC_XY_STD_MAX*100:.1f}cm")
        return False
    if z_std > C.STATIC_Z_STD_MAX:
        rospy.logerr(f"    z_std > {C.STATIC_Z_STD_MAX*100:.1f}cm — check VIO stability")
        return False

    rospy.loginfo("[6/6] PX4 参数 (COM_OBL_ACT / MPC_TILTMAX_AIR / MPC_XY_VEL_MAX)...")
    # 读不到 = 拒绝起飞 (首飞问题 §3)。mavros 连上飞控后要十几秒才把参数表拉完, 这之前
    # param/get 一律失败 ("PR: Unknown parameter to get")。以前 COM_OBL_ACT 没看 success,
    # 读失败时 value 是 0 → 被当成"0 = Position"放行。现在先 pull (等缓存就绪) 再读。
    try:
        vals = read_px4_params(param_get, param_pull, PX4_CHECK_PARAMS)
    except Exception as e:
        rospy.logerr(f"    ✗ PX4 参数读不到: {e} —— 拒绝起飞")
        rospy.logerr("      查: rosservice call /mavros/param/pull \"force_pull: true\"   (param_received 应为几百)")
        return False
    rospy.loginfo("    " + "  ".join(f"{k}={_fmt_px4(v)}" for k, v in vals.items()))
    val = int(vals["COM_OBL_ACT"])
    if val not in (0, 2):
        rospy.logerr(f"    COM_OBL_ACT={val}, want 0 (Position) or 2 (Land)")
        return False
    tmax = float(vals["MPC_TILTMAX_AIR"])
    lim = C.MAX_TILT_DEG - C.PX4_TILT_MARGIN_DEG
    if tmax > lim + 1e-6:
        rospy.logerr(f"    MPC_TILTMAX_AIR={tmax:.0f}° > {lim:.0f}°: 急刹时 PX4 自己就可能把飞机倾过"
                     f"脚本的停桨线 {C.MAX_TILT_DEG:.0f}° —— QGC 里改到 ≤ {lim:.0f}°, 拒绝起飞")
        return False
    vmax = float(vals["MPC_XY_VEL_MAX"])
    v_need = C.DASH_V
    if vmax < 1.1 * v_need:
        rospy.logerr(f"    MPC_XY_VEL_MAX={vmax:.2f} < 1.1×冲刺速度 {1.1 * v_need:.2f}: "
                     f"PX4 会把速度削掉 —— 改参数或降 dash_v, 拒绝起飞")
        return False
    rospy.loginfo(f"    COM_OBL_ACT={val} MPC_TILTMAX_AIR={tmax:.0f}° MPC_XY_VEL_MAX={vmax:.1f} OK")
    try:
        if log_dir:
            with open(os.path.join(log_dir, "site_config.txt"), "a", encoding="utf-8") as f:
                f.write("\n# PX4 参数 (预检读到)\n" +
                        "".join(f"{k} = {_fmt_px4(v)}\n" for k, v in vals.items()))
    except Exception:
        pass

    # origin 重取一次 (静态检查期间可能微动)
    p0 = tele.pose.pose.position
    INITIAL_X, INITIAL_Y, INITIAL_Z = p0.x, p0.y, p0.z
    INITIAL_YAW = yaw_from_quat(tele.pose.pose.orientation)
    rospy.loginfo(f"Origin ({INITIAL_X:+.3f}, {INITIAL_Y:+.3f}, {INITIAL_Z:+.3f})  "
                  f"yaw={math.degrees(INITIAL_YAW):+.1f}deg")
    return True


# =============================================================================
# ================================   Phases   =================================
# =============================================================================
def phase_takeoff():
    """ZHOLD 位置模式爬升。xy 目标锁在 origin。"""
    set_phase("takeoff")
    target_z = INITIAL_Z + C.TAKEOFF_HEIGHT
    rospy.loginfo(f"[takeoff] ZHOLD xy=origin z={target_z:.2f} "
                  f"(dz=+{C.TAKEOFF_HEIGHT}) timeout={C.TAKEOFF_TIMEOUT}s")
    setpoint.set_zhold_mode(INITIAL_X, INITIAL_Y, target_z, INITIAL_YAW, 0.0, 0.0)

    reach_frames_target = max(1, int(round(C.TAKEOFF_REACH_S * C.PHASE_LOOP_HZ)))
    t_end = rospy.Time.now() + rospy.Duration(C.TAKEOFF_TIMEOUT)
    t_log = rospy.Time.now()
    reach_count = 0
    rate = rospy.Rate(C.PHASE_LOOP_HZ)

    while rospy.Time.now() < t_end:
        check_abort()
        if land_flag.is_set():
            rospy.logwarn("[takeoff] LAND 请求 — 停止爬升")
            return target_z
        zf = tele.z_filtered()
        if zf is not None:
            dz = zf - INITIAL_Z
            if not in_air[0] and dz > C.TAKEOFF_LIFTOFF_DZ:
                in_air[0] = True
                rospy.logwarn(f"[takeoff] LIFTOFF DETECTED at dz={dz:.2f}")

            if abs(zf - target_z) < C.TAKEOFF_REACH_TOL:
                reach_count += 1
                if reach_count >= reach_frames_target:
                    rospy.loginfo(f"[takeoff] REACHED z={zf:.2f} "
                                  f"(target {target_z:.2f})")
                    return target_z
            else:
                reach_count = 0

            if (rospy.Time.now() - t_log).to_sec() >= 0.2:
                px, py = tele.xy()
                v = tele.velocity.twist.linear if tele.velocity else None
                dx, dy = px - INITIAL_X, py - INITIAL_Y
                if v:
                    rospy.loginfo(f"    d=({dx:+.2f},{dy:+.2f},{dz:+.2f}) "
                                  f"v=({v.x:+.2f},{v.y:+.2f},{v.z:+.2f}) "
                                  f"err_z={target_z-zf:+.2f} "
                                  f"reach_cnt={reach_count}")
                else:
                    rospy.loginfo(f"    d=({dx:+.2f},{dy:+.2f},{dz:+.2f}) "
                                  f"reach_cnt={reach_count}")
                t_log = rospy.Time.now()
        rate.sleep()

    zf = tele.z_filtered()
    dz_final = (zf - INITIAL_Z) if zf is not None else 0.0
    if in_air[0]:
        rospy.logwarn(f"[takeoff] TIMEOUT but in air at dz={dz_final:+.2f}")
        return target_z
    rospy.logerr(f"[takeoff] TIMEOUT without liftoff (dz={dz_final:+.2f})")
    raise RuntimeError("takeoff timeout without liftoff")


# =============================================================================
# ==========================   穿环: 识别输入   ===============================
# =============================================================================
class GateInput:
    """
    /gate/status (vision_bridge 每帧一条 JSON) 和 /vision_bridge/cam 的接收。
    回调里当场按当时的飞机位姿判一次能不能用 (judge), 能用的进滑窗 self.win。
    读的一方只拿拷贝, 回调和状态机互不阻塞。

    字段 (INTERFACE_gate.md §1):
      target = 环心 c (world)          —— 现在的 bridge 就有
      n / aim / n_sd_deg / cov         —— 约定里的新增字段, bridge 补上之前是 None
                                          (cov: 2026-10-19 的 vision_bridge 已经发)
      t = 拍摄时刻 t_cap, sim, pose ("ok" = 位姿插值成功), dist, sd_cm, published
    """
    REASONS = ("unpub", "pose", "sim", "old", "dist", "sd", "sd_along", "n_sd", "view", "no_n", "bad")

    def __init__(self):
        self.lock = threading.Lock()
        self.last = None
        self.last_recv = None
        self.n_msgs = 0
        self.cam = None
        self.cam_recv = None
        self.win = deque(maxlen=900)          # 有效样本 (dict)
        self.rej = {k: 0 for k in self.REASONS}
        self.n_valid = 0
        self.n_aim_far = 0

    # ---- 回调 ----
    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except Exception:
            with self.lock:
                self.rej["bad"] += 1
            return
        now = rospy.Time.now().to_sec()
        p = tele.pose
        p_now = (p.pose.position.x, p.pose.position.y, p.pose.position.z) if p else None
        smp, why = self.judge(st, now, p_now)
        with self.lock:
            self.last, self.last_recv = st, now
            self.n_msgs += 1
            if smp is not None:
                self.win.append(smp)
                self.n_valid += 1
            elif why in self.rej:
                self.rej[why] += 1

    def on_cam(self, msg):
        try:
            c = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.cam, self.cam_recv = c, rospy.Time.now().to_sec()

    # ---- 判定 ----
    def judge(self, st, now, p_now):
        """→ (样本 dict | None, 拒绝原因)"""
        try:
            # ★按环心穿 (2026-10-19): 新版桥每帧发环心观测 ring_w (+ ring_cov)。target 是红点的
            #   融合估计, 红点没定位上时只是"环心 + 0.3m 先验" (σ 停在 6~7cm) —— 有 ring_w 就用它当 c,
            #   红点 (aim) 有就瞄红点, 没有就瞄环心。老版桥没有 ring_w → 照旧用 target。
            rw = st.get("ring_w")
            raw = rw is not None
            tsrc = str(st.get("src") or "")
            t_age = st.get("age_ms")
            if (C.GATE_DOT_PREFER and st.get("target") is not None and st.get("published", True)
                    and tsrc.startswith(("dot_stereo", "dot_ray_plane"))
                    and isinstance(t_age, (int, float)) and t_age <= C.GATE_DOT_MAX_AGE_MS):
                raw = False                        # 双目红点: 比 3.5m 外的单目环心准一个量级 → 用桥的融合估计
            if raw:
                c = tuple(float(v) for v in rw[:3])
            else:
                c = st.get("target")
                if c is None or not st.get("published", True):
                    return None, "unpub"
                c = tuple(float(v) for v in c[:3])
            if st.get("pose", "ok") != "ok":
                return None, "pose"
            if st.get("sim") and not (C.GATE_SHADOW or C.GATE_ALLOW_SIM_DRIVE):
                return None, "sim"
            t_cap = float(st.get("t", now))
            if now - t_cap > C.GATE_CAP_MAX_AGE:
                return None, "old"
            dist = None if raw else st.get("dist")
            if not isinstance(dist, (int, float)):
                dist = gp.vnorm(gp.vsub(c, p_now)) if p_now else float("nan")
            if not (C.GATE_D_MIN <= dist <= C.GATE_D_MAX):
                return None, "dist"
            n = st.get("n")
            if n is not None:
                n = gp.vunit(tuple(float(v) for v in n[:3]))
                nsd = st.get("n_sd_deg")
                if isinstance(nsd, (int, float)) and nsd > C.GATE_N_SD_MAX_DEG:
                    return None, "n_sd"
                if p_now is not None:
                    los = gp.vsub(p_now, c)
                    if gp.vdot(los, n) < 0:
                        n = gp.vmul(n, -1.0)      # 约定: 指向进近侧
                    if gp.ang_deg(los, n) > C.GATE_VIEW_ANG_MAX:
                        return None, "view"
            elif C.GATE_REQUIRE_NORMAL:
                return None, "no_n"
            # σ 拆到靶面 (§1): 进入方向 ≈ −n, 没有法向就用视线方向 (和规划一致)
            d_in = gp.hunit(gp.vmul(n, -1.0)) if n is not None else None
            if d_in is None and p_now is not None:
                d_in = gp.hunit(gp.vsub(c, p_now))
            sd = st.get("sd_cm") or []
            cov_c = st.get("ring_cov") if raw else st.get("cov")
            sig = gp.sigma_hva(d_in, cov_c, None if raw else sd) if d_in is not None else None
            if sig is None:                        # 拆不了 (没有 σ / 正上方) → 按最大分量当横向
                sd_max = max(float(v) for v in sd) if sd else float("nan")
                sig = (sd_max, sd_max, sd_max)
            sd_lat = max(sig[0], sig[1])
            if not (sd_lat <= C.GATE_SD_MAX_CM):
                return None, "sd"
            lim_al = C.GATE_SD_ALONG_MAX_CM
            if raw and dist == dist:
                lim_al = max(lim_al, C.GATE_SD_ALONG_PER_M_CM * dist)
            if not (sig[2] <= lim_al):
                return None, "sd_along"
            aim = st.get("aim")
            if aim is not None:
                aim = tuple(float(v) for v in aim[:3])
                if gp.vnorm(gp.vsub(aim, c)) > C.R_IN - 0.05:
                    aim = None                     # 红点跑到环外 = 红点识别有问题, 按环心瞄
                    self.n_aim_far += 1
            return {"t": t_cap, "recv": now, "c": c, "aim": aim, "n": n, "dist": float(dist),
                    "sd": sd_lat, "sig": sig, "n_sd": st.get("n_sd_deg"),
                    "raw": raw, "src": "ring_" + str(st.get("ring_src")) if raw else st.get("src"),
                    "p": p_now}, "ok"
        except Exception:
            return None, "bad"

    # ---- 读 ----
    def valid_since(self, t_recv):
        with self.lock:
            return [s for s in self.win if s["recv"] >= t_recv]

    def latest(self):
        """→ (最新一条原始 JSON, 收到至今秒数)"""
        with self.lock:
            st, t = self.last, self.last_recv
        if st is None:
            return None, float("inf")
        return st, rospy.Time.now().to_sec() - t

    def latest_valid(self):
        with self.lock:
            return self.win[-1] if self.win else None

    def cam_mode(self):
        with self.lock:
            c, t = self.cam, self.cam_recv
        if c is None or rospy.Time.now().to_sec() - t > 5.0:
            return None
        return (c.get("cfg") or {}).get("mode") or "aruco"

    def counters(self):
        with self.lock:
            return self.n_valid, sum(self.rej.values()), dict(self.rej)

    REJ_TXT = {"unpub": "桥没发布", "pose": "位姿不对", "sim": "假数据", "old": "帧太旧", "dist": "距离超范围",
               "sd": "横向σ太大", "sd_along": "深度σ太大", "n_sd": "法向不稳", "view": "斜视角太大",
               "no_n": "没法向", "bad": "消息坏"}

    def rej_since(self, snap):
        """snap = 上次的 (n_valid, rej dict)。→ (新快照, 这段时间的拒绝说明 或 None)。
        2026-10-21: 以前识别到了但每帧都被拒时, 日志只写"没看到环", 看不出是被哪道门卡住。"""
        nv, _, rej = self.counters()
        if snap is None:
            return (nv, rej), None
        d = {k: v - snap[1].get(k, 0) for k, v in rej.items() if v - snap[1].get(k, 0) > 0}
        if not d:
            return (nv, rej), None
        with self.lock:
            st = self.last
        extra = ""
        if d.get("sd_along") or d.get("sd") or d.get("dist"):
            try:
                rw = (st or {}).get("ring_w") or (st or {}).get("target")
                if rw is not None and tele.pose is not None:
                    q = tele.pose.pose.position
                    extra = f" (最新一帧环心距 {math.dist(rw[:3], (q.x, q.y, q.z)):.1f}m)"
            except Exception:
                pass
        txt = "识别到了但单帧被拒: " + ", ".join(f"{self.REJ_TXT.get(k, k)}×{v}" for k, v in
                                          sorted(d.items(), key=lambda kv: -kv[1])) + extra
        return (nv, rej), txt


gate_in = GateInput()


# =============================================================================
# ==========================   穿环: 状态机   =================================
# =============================================================================
class GateStop(Exception):
    """
    冲刺前的正常退出 (放弃 / LAND / 接管)。原因在 args[0]。
    retry=True  = 这一次不行 (锁不住 / 不可行 / G 超时 …), GATE_ABORT_TO_MANUAL 时回手动段重来
    retry=False = LAND 请求 / 接管 / 手动段超时: 不重来, 降落
    """

    def __init__(self, why, retry=True):
        super().__init__(why)
        self.retry = retry


class GateBack(GateStop):
    """操作员要回手动段 (冲刺前按 M 或任一方向键)。"""


def _l3(v):
    return [_f(x) for x in v] if v is not None else None


class GateRun:
    """
    一次穿环。phase_gate() 创建它并依次调用各段; 每段都是"循环 + tick()"。
    tick() 统一做: 键盘、冲刺前的停止条件、遥测 gate 段、gate.csv、按 GATE_LOOP_HZ 节拍。
    """

    def __init__(self, z_hold):
        self.z_hold = z_hold
        self.seg = ""
        p = self.pos()
        self.p_hover = p
        self.yaw_sp = tele.yaw_measured()
        self.acq = None          # ACQUIRE 粗估 (gate_plan.fuse_samples 的结果)
        self.locked = None       # ALIGN 锁定值 (+ t_lock)
        self.plan = None
        self.plan_seq = 0
        self.t0 = None           # 冲刺起点 (ROS 秒)
        self.lvl = 0             # 0 正常 / 1 卡纸处置 (L1)
        self.passed = False
        self.dash_state = None   # (t, s_plan, v_plan, s_est, v_est)
        self.land_deferred = None
        self.outcome = ""
        self.keys = None
        self.flags = set()
        self.move = None         # 本拍最后一个方向/悬停键 (手动段用)
        self.nkeys = 0           # 本拍读到几个字符 (手动段: 有键 = 重新计时)
        self.back_ok = False     # True = 冲刺前的段, M / 方向键 → 回手动段
        self.n_try = 0           # 第几次 (按 T 开始一次 +1)
        self.last_why = ""       # 上一次放弃的原因 (遥测 gate.why)
        self.pre = None          # 手动段预规划结果 (遥测 gate.pre)
        self.rate = rospy.Rate(C.GATE_LOOP_HZ)
        self.last_t = rospy.Time.now().to_sec()
        self.t_log = 0.0
        self.origin = (INITIAL_X, INITIAL_Y, INITIAL_Z)
        self.params = gp.params_from_globals(vars(C))

    # ---------------- 小工具 ----------------
    def pos(self):
        p = tele.pose
        if p is None:
            return (INITIAL_X, INITIAL_Y, self.z_hold if hasattr(self, "z_hold") else INITIAL_Z)
        q = p.pose.position
        return (q.x, q.y, q.z)

    def vel(self):
        v = tele.velocity
        if v is None:
            return (0.0, 0.0, 0.0)
        l = v.twist.linear
        return (l.x, l.y, l.z)

    def set_seg(self, name, msg=None):
        if name != self.seg:
            self.seg = name
            sub_mode_ref[0] = name
            rospy.loginfo(f"[gate] ── 段 {name}" + (f": {msg}" if msg else ""))

    def yaw_step(self, target, dt):
        """yaw 设定值以 GATE_YAW_RATE 朝 target 走一步。→ 剩余误差 (rad)。"""
        err = wrap_pi(target - self.yaw_sp)
        step = C.GATE_YAW_RATE * dt
        self.yaw_sp = wrap_pi(self.yaw_sp + clamp(err, -step, step))
        return wrap_pi(target - tele.yaw_measured())

    def hold(self, p):
        setpoint.set_zhold_mode(p[0], p[1], p[2], self.yaw_sp, 0.0, 0.0)

    def fail(self, why):
        raise GateStop(why)

    @staticmethod
    def _raw_sigma(f, smp):
        """
        样本是桥每帧的环心观测 (ring_w, 没经过桥的滤波) 时, 多帧中值的 σ ≈ 单帧 σ / √k。
        样本是桥的融合估计 (老版桥 / target) 时, 每帧的 σ 已经是融合后的, 不再除。
        """
        sg = f.get("sig_hva")
        if sg and smp and all(s.get("raw") for s in smp):
            k = max(1, len(smp))
            f["raw"] = True
            f["sig_hva_frame"] = sg
            # 多帧中值的标准误: 用实际散布 (p90−p10 ≈ 2.56σ, 中值效率 ≈ 1.25) 和桥报的单帧 σ 取大的
            sp = f.get("spread_hva") or (0.0, 0.0, 0.0)
            f["sig_hva"] = tuple(max(x, 1.25 * y / 2.56) / math.sqrt(k) for x, y in zip(sg, sp))

    @staticmethod
    def _sd_fields(f):
        """融合结果里给遥测用的 σ 字段。sd_hva_cm = 靶面 (横 h, 竖 v, 深度) σ 中位数。"""
        sg = f.get("sig_hva")
        if sg:
            f["sd_hva_cm"] = [round(x, 2) for x in sg]
            f["sd_cm_xyz"] = [round(max(sg[0], sg[1]), 2)] * 3

    @staticmethod
    def _sig_txt(f):
        sg = f.get("sig_hva")
        return f"横{sg[0]:.1f}/竖{sg[1]:.1f}/深{sg[2]:.1f}cm" if sg else "?"

    def used(self):
        """遥测 / 规划用的环: 锁定值 > 粗估 > 最新一帧。→ dict 或 None"""
        if self.locked:
            return self.locked
        if self.acq:
            return self.acq
        s = gate_in.latest_valid()
        if s is None:
            return None
        return {"c": s["c"], "aim": s["aim"], "n": s["n"], "k": 1, "spread_cm": None,
                "sig": s.get("sig")}

    def n_used(self, u, p):
        """→ (法向, 来源)。没有法向时用"−进入方向"或视线方向代替 (网页靠它画靶面)。"""
        if u and u.get("n") is not None:
            return u["n"], "bridge"
        if self.plan is not None and self.plan.dir is not None:
            return gp.vmul(self.plan.dir, -1.0), "plan_dir"
        if u:
            d = gp.hunit(gp.vsub(p, u["c"]))
            if d is not None:
                return d, "los"
        return None, None

    # ---------------- 每拍 ----------------
    def tick(self, dash=False):
        now = rospy.Time.now().to_sec()
        dt = min(max(now - self.last_t, 0.0), 0.2)
        self.last_t = now
        if self.keys is not None:
            move, flags, eof, nk = self.keys.poll()
            if 'x' in flags:
                request_abort(f"gate X 键 (段 {self.seg})")
            if eof:
                request_land("gate stdin EOF (SSH 断连?)")
            if 'l' in flags:
                request_land("gate L 键")
            self.flags, self.move, self.nkeys = flags, move, nk
        else:
            self.flags, self.move, self.nkeys = set(), None, 0
        check_abort()
        if takeover[0] is not None:
            raise GateStop("控制权被接管", retry=False)
        if land_flag.is_set():
            if not dash:
                raise GateStop(f"LAND 请求: {land_reason[0]}", retry=False)
            if self.land_deferred is None:
                self.land_deferred = land_reason[0]
                rospy.logwarn(f"[gate] LAND 请求 ({land_reason[0]}) —— 冲刺中, 推迟到刹停后执行")
        if not dash and self.back_ok and ('m' in self.flags or self.move is not None):
            raise GateBack("操作员按了 %s" % ("M" if 'm' in self.flags else "方向键"))
        try:
            self._telem_and_log(now)
        except Exception as e:
            rospy.logwarn_throttle(C.THROTTLE_S, f"[gate] 遥测/日志出错 (不影响飞行): {e}")
        self.rate.sleep()
        return dt

    def _telem_and_log(self, now):
        p, v = self.pos(), self.vel()
        u = self.used()
        n, n_src = self.n_used(u, p)
        g = {"seg": self.seg}
        along = off = pred = ttc = v_al = None
        if u is not None and n is not None:
            h_, v_, along = gp.to_plane(p, u["c"], n)
            off = [h_, v_]
            right, up = gp.gate_axes(n)
            v_al = -gp.vdot(v, n)
            if self.seg in ("dash", "brake") and v_al > 0.2 and along > 0:
                ttc = along / v_al
                pred = [h_ + gp.vdot(v, right) * ttc, v_ + gp.vdot(v, up) * ttc]
            elif self.plan is not None and self.plan.ok:
                ah, av, _ = gp.to_plane(self.plan.aim, u["c"], n)
                pred = [ah, av]                       # 计划从瞄点穿过
            else:
                pred = list(off)
            sd = u.get("sd_cm_xyz") or [C.GATE_SD_MAX_CM] * 3
            hva = u.get("sd_hva_cm") or (list(u["sig"]) if u.get("sig") else None)
            sdh, sdv = (hva[0], hva[1]) if hva else (max(sd), max(sd))
            g.update({
                "gate_used": {
                    "c": _l3(u["c"]), "n": _l3(n), "n_src": n_src,
                    "aim": _l3(u.get("aim") or u["c"]),
                    "sd_cm": _l3(sd), "sd_hva_cm": _l3(hva), "n_sd_deg": _f(u.get("n_spread_deg")),
                    "locked": bool(self.locked), "t_lock": (self.locked or {}).get("t_lock"),
                    "r_in": C.R_IN, "r_out": C.R_OUT, "spread_cm": _f(u.get("spread_cm")),
                    "k": u.get("k")},
                "along": _f(along), "off": _l3(off), "pred_cross": _l3(pred),
                "pred_sd_cm": [_f(math.hypot(sdh, 5.0)), _f(math.hypot(sdv, 5.0))],
                "ttc": _f(ttc), "v_along_meas": _f(v_al),
            })
        ds = self.dash_state
        g["v_along_cmd"] = _f(ds[2]) if ds and self.seg in ("dash", "brake") else 0.0
        lv = gate_in.latest_valid()
        g["gate_age"] = _f(now - lv["t"]) if lv else None
        if self.plan is not None and self.plan.ok:
            P = self.plan
            g["commit"] = {"d": _f(P.d_run), "passed": bool(self.passed),
                           "abort_ok": self.seg in ("manual", "wait", "align", "commit", "abort")}
            g["plan"] = {"seq": self.plan_seq,
                         "pts": [_l3(self.p_hover), _l3(P.S), _l3(P.aim), _l3(P.E)],
                         "labels": ["悬停点", "起跑点", "穿越点", "制动终点"],
                         "mode": "dash",
                         "v": _f(P.v), "summary": P.summary(), "dir_why": P.dir_why,
                         "v_set": _f(C.DASH_V),
                         "slow": bool(P.v < C.DASH_V - 1e-9),
                         "slow_why": P.slow_why,
                         "need_confirm": bool(P.need_confirm),
                         "v_hit": _f(P.v_hit_nom), "v_hit_worst": _f(P.v_hit_worst),
                         "virtual": C.GATE_VIRTUAL_OFFSET > 0}
            g["brake"] = {"end": _l3(P.E), "dist": _f(P.brake_dist), "acc": _f(C.BRAKE_ACC),
                          "s_b": _f(P.traj.s_b), "s_stop": _f(P.s_stop),
                          "pass_run": _f(C.PASS_RUN),
                          "pass_lag": _f(P.pass_lag), "pass_actual": _f(P.pass_actual),
                          "wall_left": _f((P.tol or 0.0) + C.M_BACK)}
        nv, nr, rej = gate_in.counters()
        g["det"] = {"n_valid": nv, "n_rej": nr, "rej": rej}
        if ds:
            g["dash"] = {"t": _f(ds[0]), "s_plan": _f(ds[1]), "v_plan": _f(ds[2]),
                         "s_est": _f(ds[3]), "v_est": _f(ds[4])}
        g["lvl"] = self.lvl
        g["why"] = self.last_why        # 上一次放弃的原因 (回手动段后网页提示行显示)
        if self.seg == "manual" and self.pre:
            pr = self.pre
            g["pre"] = {"ok_full": bool(pr["ok_full"]), "v_set": _f(C.DASH_V), "v_room": _f(pr["v_room"]),
                        "why": pr["why"], "xy_need": _f(pr["xy_need"]), "d_back_need": _f(pr["d_back_need"])}
        g["land_deferred"] = self.land_deferred
        g["shadow"] = bool(C.GATE_SHADOW)
        telem_put("gate", g)

        # ---- gate.csv
        if writer_gate is None:
            return
        st, age = gate_in.latest()
        det = (st or {}).get("target") if st else None
        mode, spp, _, _, yaw_sp = setpoint.snapshot()
        nan = float("nan")

        def fx(x, fmt="%.3f"):
            return (fmt % x) if isinstance(x, (int, float)) and x == x else "nan"
        cc = (u or {}).get("c") or (nan,) * 3
        aa = ((u or {}).get("aim") or cc) if u else (nan,) * 3
        nn = n or (nan,) * 3
        q = tele.pose.pose.orientation if tele.pose else None
        sg_ = (lv or {}).get("sig") or (nan,) * 3
        rw_ = (st or {}).get("ring_w") or (nan,) * 3
        row = [
            "%.3f" % now, self.seg, fx(along), fx(off[0] if off else None),
            fx(off[1] if off else None), fx(pred[0] if pred else None),
            fx(pred[1] if pred else None), fx(ttc), fx(v_al), fx(g.get("v_along_cmd")),
            fx((now - lv["t"]) * 1000 if lv else None, "%.0f"),
            fx(cc[0]), fx(cc[1]), fx(cc[2]), fx(nn[0], "%.4f"), fx(nn[1], "%.4f"), fx(nn[2], "%.4f"),
            fx(aa[0]), fx(aa[1]), fx(aa[2]), int(bool(self.locked)),
            {0: "V", 1: "P", 2: "T"}.get(mode, "?"), self.lvl,
            fx(ds[1] if ds else None), fx(ds[2] if ds else None),
            fx(ds[3] if ds else None), fx(ds[4] if ds else None),
            fx(spp[0]), fx(spp[1]), fx(spp[2]), fx(p[0]), fx(p[1]), fx(p[2]),
            fx(math.degrees(tele.yaw_measured()), "%.1f"),
            fx(tilt_deg_from_quat(q) if q else None, "%.1f"),
            fx((st or {}).get("dist") if st else None),
            fx(det[0] if det else None), fx(det[1] if det else None), fx(det[2] if det else None),
            fx(age * 1000 if st else None, "%.0f"), nv, nr, int(self.land_deferred is not None),
            fx(sg_[0], "%.2f"), fx(sg_[1], "%.2f"), fx(sg_[2], "%.2f"),
            fx(tilt_limit()[0], "%.0f"),
            "%s|%s" % ((st or {}).get("src") or "", (st or {}).get("ring_src") or ""),
            fx(rw_[0]), fx(rw_[1]), fx(rw_[2]),
            int(bool(((st or {}).get("ring") or {}).get("partial"))) if st else "nan",
            json.dumps((st or {}).get("diag") or {}, ensure_ascii=False, separators=(",", ":"))
            if isinstance((st or {}).get("diag"), dict) else "",
        ]
        gate_queue.append(row)           # deque.append 线程安全, 不碰磁盘

    # ---------------- ACQUIRE ----------------
    # ---------------- MANUAL (手动段) ----------------
    def manual(self, why=""):
        """
        手动段: 和 stage11 的 teleop 一样的键 (机体系 WASD 平移 / R F 升降 / Q E 转向 / 空格悬停),
        按 T 返回 = 开始这一次 (找环 → 规划 → …)。所有按键都是"给速度, 按 dt 积分", 松键
        TELEOP_DEADMAN 后速度归零、位置冻结。GATE_MANUAL_TIMEOUT 内一个键都没按 → 降落。
        没有终端键盘 → 直接返回 (= 老行为: 起飞后马上找环)。
        """
        self.back_ok = False
        dash_tilt_on[0] = False
        set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
        self.set_seg("manual", why or "按 T 开始找环/规划")
        if self.keys is None:
            rospy.logwarn("[gate] 没有终端键盘 —— 跳过手动段, 直接找环")
            return
        self.keys.drain()                          # 丢掉回手动段之前按着的键
        self.flags, self.move, self.nkeys = set(), None, 0
        if why:
            rospy.logwarn(f"[gate] ── 手动段: {why}")
        rospy.logwarn("[gate] ★ 手动段: W前 S后 A左 D右 | R升 F降 | Q左转 E右转 | 空格悬停 | "
                      "T = 开始找环规划 | L = 降落 | X = 停桨")
        p = self.pos()
        tx, ty, tz = p
        yaw = self.yaw_sp
        vbx = vby = vz = yr = 0.0
        t0 = rospy.Time.now().to_sec()
        t_key = t0
        dt = 0.0
        while True:
            now = rospy.Time.now().to_sec()
            if 't' in self.flags:
                rospy.logwarn("[gate] ★ T → 开始找环/规划")
                break
            if self.nkeys > 0:
                t_key = now
            mv = self.move
            if mv is not None:
                vbx = vby = vz = yr = 0.0
                if mv == 'w':
                    vbx = C.TELEOP_XY_SPEED
                elif mv == 's':
                    vbx = -C.TELEOP_XY_SPEED
                elif mv == 'a':
                    vby = C.TELEOP_XY_SPEED
                elif mv == 'd':
                    vby = -C.TELEOP_XY_SPEED
                elif mv == 'r':
                    vz = C.TELEOP_Z_SPEED
                elif mv == 'f':
                    vz = -C.TELEOP_Z_SPEED
                elif mv == 'q':
                    yr = C.TELEOP_YAW_RATE
                elif mv == 'e':
                    yr = -C.TELEOP_YAW_RATE
            idle = now - t_key
            if idle > C.TELEOP_DEADMAN:
                vbx = vby = vz = yr = 0.0
            if idle > C.GATE_MANUAL_TIMEOUT:
                raise GateStop(f"手动段 {C.GATE_MANUAL_TIMEOUT:.0f}s 没有按键", retry=False)
            cy, sy = math.cos(yaw), math.sin(yaw)
            vx, vy = vbx * cy - vby * sy, vbx * sy + vby * cy
            tx += vx * dt
            ty += vy * dt
            yaw = wrap_pi(yaw + yr * dt)
            ddx, ddy = tx - INITIAL_X, ty - INITIAL_Y
            r = math.hypot(ddx, ddy)
            if r > C.TELEOP_XY_MAX:
                tx, ty = INITIAL_X + ddx * C.TELEOP_XY_MAX / r, INITIAL_Y + ddy * C.TELEOP_XY_MAX / r
                vx = vy = 0.0
            tz = INITIAL_Z + clamp(tz + vz * dt - INITIAL_Z, C.TELEOP_DZ_MIN, C.TELEOP_DZ_MAX)
            self.yaw_sp = yaw
            setpoint.set_zhold_mode(tx, ty, tz, yaw, vx, vy)
            if now - self.t_log > 1.0:
                self.t_log = now
                pn = self.pos()
                lv = gate_in.latest_valid()
                self._rej_snap, rtxt = gate_in.rej_since(getattr(self, "_rej_snap", None))
                if lv is not None and now - lv["recv"] < 1.0:
                    dc = math.hypot(lv["c"][0] - pn[0], lv["c"][1] - pn[1])
                    dmin = gp.d_run_for(C.DASH_V, self.params)
                    hint = ("就地起跑" if dmin <= dc <= max(dmin, C.D_RUN_MAX) else
                            ("会后退到 %.1fm" % dmin) if dc < dmin else ("会前进到 %.1fm" % max(dmin, C.D_RUN_MAX)))
                    seen = f"看到环: 水平 {dc:.2f}m 高差 {lv['c'][2] - pn[2]:+.2f}m ({hint})"
                    seen += "  " + self.manual_precheck(pn, now)
                else:
                    seen = rtxt or "没看到环"
                    self.pre = None
                rospy.loginfo(f"    [manual] d=({pn[0]-INITIAL_X:+.2f},{pn[1]-INITIAL_Y:+.2f},"
                              f"{pn[2]-INITIAL_Z:+.2f}) yaw={math.degrees(yaw):+.0f}°  {seen}  "
                              f"(无按键 {idle:.0f}s / {C.GATE_MANUAL_TIMEOUT:.0f}s)")
            dt = self.tick()
        setpoint.set_zhold_mode(tx, ty, tz, yaw, 0.0, 0.0)

    def manual_precheck(self, pn, now):
        """
        手动段里 (2026-10-21) 每 2s 按飞机当前位置、最近 1s 的识别中值预规划一次:
        按 T 之前就知道 设定速度放不放得下 / 瞄点高度在不在带内, 不用 T 了才发现。
        → 一段日志文字; 结果放进 self.pre (遥测 gate.pre, 网页提示行显示)。
        """
        if self.pre is not None and now - self.pre.get("_t", 0) < 2.0:
            return self.pre.get("_txt", "")
        smp = gate_in.valid_since(now - 1.0)
        if len(smp) < 3:
            return ""
        try:
            c = tuple(gp.median([s["c"][i] for s in smp]) for i in range(3))
            aims = [s["aim"] for s in smp if s.get("aim")]
            aim = tuple(gp.median([a[i] for a in aims]) for i in range(3)) if aims else c
            r = gp.preflight_check(aim, c, None, self.origin, self.params, 0.0,
                                   virtual_offset=C.GATE_VIRTUAL_OFFSET, p_now=pn)
        except Exception as e:
            return f"(预规划出错: {e})"
        if r["ok_full"]:
            txt = f"按 {C.DASH_V:.1f} m/s: ✓ 放得下"
        else:
            txt = (f"按 {C.DASH_V:.1f} m/s: ✗ {r['why'] or '不可行'} → 最高 "
                   + (f"{r['v_room']:.1f} m/s" if r["v_room"] is not None else "—")
                   + (f", 满速要 xy_radius ≥ {r['xy_need']:.1f}" if r["xy_need"] and r["xy_need"] > C.GATE_XY_MAX else ""))
            if not (self.pre or {}).get("_warned"):
                rospy.logwarn(f"[gate] ★ 预规划 (按 T 之前): {txt}")
                r["_warned"] = True
            elif self.pre:
                r["_warned"] = True
        r.update(_t=now, _txt=txt, v_set=C.DASH_V)
        self.pre = r
        return txt

    def new_attempt(self):
        """按 T 之后: 清掉上一次的估计/计划, 以当前位置为悬停点开始。"""
        p = self.pos()
        self.p_hover = p
        self.z_hold = p[2]
        self.acq = self.locked = self.plan = None
        self.lvl = 0
        self.passed = False
        self.dash_state = None
        self.outcome = ""
        self.n_try += 1
        self.last_why = ""
        self.back_ok = self.keys is not None
        if self.n_try > 1:
            rospy.logwarn(f"[gate] ── 第 {self.n_try} 次")

    # ---------------- ACQUIRE ----------------
    def acquire(self):
        self.set_seg("wait", f"悬停识别 (窗口 {C.HOVER_ACQ:.0f}s, 至少 {C.GATE_ACQ_MIN_N} 帧)")
        set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
        t_start = rospy.Time.now().to_sec()
        yaw0 = self.yaw_sp
        scan = [yaw0 + math.radians(C.GATE_SCAN_DEG), yaw0 - math.radians(C.GATE_SCAN_DEG), yaw0]
        scan_i = None                      # None = 还没开始扫; len(scan) = 扫完了
        hp = (self.p_hover[0], self.p_hover[1], self.z_hold)
        self.p_hover = hp
        dt = 0.0
        tol = math.radians(C.GATE_YAW_TOL_DEG)
        last_why = "还没看到环"
        rej_snap, _ = gate_in.rej_since(None)
        while True:
            now = rospy.Time.now().to_sec()
            p = self.pos()
            # 只用进入 ACQUIRE 之后收到的帧 (起飞爬升中的帧不算)
            smp = gate_in.valid_since(max(t_start, now - C.HOVER_ACQ))
            fresh = [s for s in smp if now - s["recv"] < C.GATE_STATUS_MAX_AGE]
            scanning = False
            if fresh:                      # 看得到: 机头转向环心
                c = fresh[-1]["c"]
                target = math.atan2(c[1] - p[1], c[0] - p[0])
            elif not smp and now - t_start > C.GATE_ACQ_SCAN_AFTER and (scan_i or 0) < len(scan):
                if scan_i is None:
                    scan_i = 0
                    rospy.logwarn(f"[gate] {C.GATE_ACQ_SCAN_AFTER:.0f}s 没看到环 —— "
                                  f"±{C.GATE_SCAN_DEG:.0f}° 慢扫一次")
                target, scanning = scan[scan_i], True
            else:
                target = self.yaw_sp
            err = self.yaw_step(target, dt)
            if scanning and abs(wrap_pi(target - self.yaw_sp)) < 1e-3 and abs(err) < tol:
                scan_i += 1
            self.hold(hp)
            if len(smp) >= C.GATE_ACQ_MIN_N and fresh and abs(err) < math.radians(C.GATE_YAW_TOL_DEG):
                d_los = gp.hunit(gp.vsub(fresh[-1]["c"], p)) or (math.cos(self.yaw_sp), math.sin(self.yaw_sp), 0.0)
                f = gp.fuse_samples(smp, p, d=d_los)
                self._raw_sigma(f, smp)
                sh, sv, sa = f["spread_hva"]
                if f.get("raw"):                     # 每帧原始环心: 散布本来就大, 看中值的标准误
                    sh, sv, sa = (x * 2.56 / 1.25 for x in f["sig_hva"])
                if max(sh, sv) <= C.GATE_ACQ_SPREAD_CM and sa <= 2.0 * C.GATE_ACQ_SPREAD_CM:
                    self._sd_fields(f)
                    self.acq = f
                    rospy.loginfo(f"[gate] 粗估: 环心 ({f['c'][0]:+.2f},{f['c'][1]:+.2f},{f['c'][2]:+.2f}) "
                                  f"k={f['k']} 离散 横{max(sh, sv):.1f}/深{sa:.1f}cm  σ {self._sig_txt(f)}  "
                                  f"法向 {'有' if f.get('n') else '无'}  红点 {'有' if f.get('aim') else '无'}")
                    return
                last_why = (f"样本离散 横{max(sh, sv):.1f}/深{sa:.1f}cm > "
                            f"{C.GATE_ACQ_SPREAD_CM:.0f}/{2 * C.GATE_ACQ_SPREAD_CM:.0f}cm")
            elif smp:
                last_why = f"有效帧 {len(smp)}/{C.GATE_ACQ_MIN_N}" + (
                    "" if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) else
                    f", 机头还差 {math.degrees(err):+.0f}°")
            if now - t_start > C.GATE_ACQ_TIMEOUT:
                nv, nr, rej = gate_in.counters()
                self.fail(f"{C.GATE_ACQ_TIMEOUT:.0f}s 内没拿到稳定识别 ({last_why}; "
                          f"拒绝原因 {', '.join(f'{k}={v}' for k, v in rej.items() if v)})")
            if now - self.t_log > 1.0:
                self.t_log = now
                rej_snap, rtxt = gate_in.rej_since(rej_snap)
                if not smp and rtxt:
                    last_why = rtxt
                rospy.loginfo(f"    [wait] {last_why}")
            dt = self.tick()

    # ---------------- PLAN ----------------
    def make_plan(self, est, p_from, label):
        aim = est.get("aim") or est["c"]
        P = gp.plan_dash(aim, est["c"], est.get("n"), p_from, self.origin, self.params,
                         virtual_offset=C.GATE_VIRTUAL_OFFSET)
        if not P.ok:
            self.fail(f"{label}: " + "; ".join(P.errors))
        self.plan = P
        self.plan_seq += 1
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

    # ---------------- REPOSITION ----------------
    def move_to(self, S, yaw_mode, timeout, label):
        """
        setpoint 以 GATE_REPO_V 斜坡走直线到 S (3D, 带牵引绳)。yaw_mode:
          "ring" = 机头盯环心 (保持环在视野里)   数值 = 固定朝向
        全程查"飞机在环前 ≥1.5m" (环系 x ≤ REPO_X_MAX)。
        """
        self.set_seg("align", label)
        P = self.plan
        fwd, _ = gp._ring_frame(P.dir, (self.used() or {}).get("n"))
        sp = self.pos()
        t_end = rospy.Time.now().to_sec() + timeout
        reach = 0
        need = max(1, int(round(0.5 * C.GATE_LOOP_HZ)))
        dt = 0.0
        while True:
            now = rospy.Time.now().to_sec()
            p = self.pos()
            d = gp.vsub(S, sp)
            dn = gp.vnorm(d)
            step = C.GATE_REPO_V * dt
            sp = S if dn <= step else gp.vadd(sp, gp.vmul(d, step / max(dn, 1e-9)))
            lead = gp.vsub(sp, p)
            ln = gp.vnorm(lead)
            if ln > C.GATE_REPO_LEASH:
                sp = gp.vadd(p, gp.vmul(lead, C.GATE_REPO_LEASH / ln))
            if yaw_mode == "ring":
                c = (self.used() or {}).get("c") or P.aim
                if math.hypot(c[0] - p[0], c[1] - p[1]) > 0.5:
                    self.yaw_step(math.atan2(c[1] - p[1], c[0] - p[0]), dt)
            else:
                self.yaw_step(yaw_mode, dt)
            vff = gp.vmul(d, C.GATE_REPO_V / dn) if dn > step and ln <= C.GATE_REPO_LEASH else (0.0, 0.0, 0.0)
            setpoint.set_zhold_mode(sp[0], sp[1], sp[2], self.yaw_sp, vff[0], vff[1])
            x_ring = gp.vdot(gp.vsub(p, P.aim), fwd)
            if x_ring > C.REPO_X_MAX + 0.3:
                self.fail(f"摆位中离环平面只剩 {-x_ring:.2f}m (< {-C.REPO_X_MAX:.1f}m)")
            err = gp.vnorm(gp.vsub(S, p))
            reach = reach + 1 if (err < C.GATE_REPO_TOL and sp == S) else 0
            if reach >= need:
                return
            if now > t_end:
                self.fail(f"{label}: {timeout:.0f}s 没到位 (还差 {err:.2f}m)")
            if now - self.t_log > 1.0:
                self.t_log = now
                rospy.loginfo(f"    [{label}] 离起跑点 {err:.2f}m  离环平面 {-x_ring:.2f}m")
            dt = self.tick()

    def turn_to(self, yaw_target, hold_p, label, timeout=15.0):
        t_end = rospy.Time.now().to_sec() + timeout
        dt = 0.0
        while True:
            err = self.yaw_step(yaw_target, dt)
            self.hold(hold_p)
            if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) and \
                    abs(wrap_pi(yaw_target - self.yaw_sp)) < 1e-3:
                return
            if rospy.Time.now().to_sec() > t_end:
                self.fail(f"{label}: 转头 {timeout:.0f}s 没到位 (差 {math.degrees(err):+.0f}°)")
            dt = self.tick()

    # ---------------- ALIGN ----------------
    def align(self, hold_p):
        self.set_seg("align", f"在 {'原地 (影子)' if C.GATE_SHADOW else '起跑点'} 取样 {C.HOVER_ALIGN:.0f}s → 锁定")
        t_al = rospy.Time.now().to_sec() + 0.5          # 转头/刹停后先稳 0.5s
        last_why = ""
        while True:
            now = rospy.Time.now().to_sec()
            self.hold(hold_p)
            if now - t_al >= C.HOVER_ALIGN:
                p, v = self.pos(), self.vel()
                smp = gate_in.valid_since(max(t_al, now - C.HOVER_ALIGN))
                d_in = self.plan.dir if self.plan is not None and self.plan.dir is not None else \
                    (gp.hunit(gp.vsub(smp[-1]["c"], p)) if smp else None)
                f = gp.fuse_samples(smp, p, d=d_in) if d_in is not None else gp.fuse_samples(smp, p)
                self._raw_sigma(f, smp)
                why = []
                if f["k"] < C.GATE_ACQ_MIN_N:
                    why.append(f"有效帧 {f['k']} < {C.GATE_ACQ_MIN_N}")
                else:
                    # 横向 (h, v) 和深度分开判 (首飞问题 §1)
                    sh, sv, sa = f.get("spread_hva") or (f["spread_cm"],) * 3
                    if f.get("raw"):
                        # 每帧原始环心: 单帧散布大是正常的 (几 cm ~ 深度十几 cm), 精度看中值的标准误
                        # (下面的 σ 判定已经用了实际散布); 这里只拦散得离谱的 (野值 / 两个解来回跳)
                        sh, sv, sa = sh / 4.0, sv / 4.0, sa / 4.0
                    if max(sh, sv) > C.GATE_ALIGN_SPREAD_CM:
                        why.append(f"环心横向离散 {max(sh, sv):.1f}cm > {C.GATE_ALIGN_SPREAD_CM:.0f}")
                    if sa > C.GATE_ALIGN_SPREAD_ALONG_CM:
                        why.append(f"环心深度离散 {sa:.1f}cm > {C.GATE_ALIGN_SPREAD_ALONG_CM:.0f}")
                    ah_, av_, _ = f.get("aim_spread_hva") or (f.get("aim_spread_cm", 0.0),) * 3
                    if max(ah_, av_) > C.GATE_ALIGN_SPREAD_CM:
                        why.append(f"红点横向离散 {max(ah_, av_):.1f}cm > {C.GATE_ALIGN_SPREAD_CM:.0f}")
                    if self.acq and self.acq.get("n") and f.get("n"):
                        a = gp.ang_deg(self.acq["n"], f["n"])
                        if a > C.GATE_N_AGREE_DEG:
                            why.append(f"两个视角的法向差 {a:.1f}° > {C.GATE_N_AGREE_DEG:.0f}° (翻转解?)")
                    if gp.vnorm(v) > C.GATE_START_V_MAX:
                        why.append(f"飞机还在动 {gp.vnorm(v):.2f}m/s")
                    sg = f.get("sig_hva") or (gp.median([s["sd"] for s in smp]),) * 3
                    if max(sg[0], sg[1]) > C.GATE_LOCK_SD_LAT_CM:
                        why.append(f"识别横向 σ 中位数 {max(sg[0], sg[1]):.1f}cm > {C.GATE_LOCK_SD_LAT_CM:.0f}")
                    if sg[2] > C.GATE_LOCK_SD_ALONG_CM:
                        why.append(f"识别深度 σ 中位数 {sg[2]:.1f}cm > {C.GATE_LOCK_SD_ALONG_CM:.0f}")
                if not why:
                    self._sd_fields(f)
                    f["t_lock"] = now
                    f["locked"] = True
                    self.locked = f
                    c = f["c"]
                    shift = gp.vnorm(gp.vsub(c, self.acq["c"])) if self.acq else float("nan")
                    rospy.logwarn(f"[gate] ★ 锁定: 环心 ({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f}) "
                                  f"k={f['k']} 单帧散布 横{max(f['spread_hva'][:2]):.1f}/深{f['spread_hva'][2]:.1f}cm  "
                                  f"σ {self._sig_txt(f)}  比粗估挪了 {shift*100:.0f}cm"
                                  + (f"  红点离环心 {gp.vnorm(gp.vsub(f['aim'], c))*100:.0f}cm"
                                     if f.get("aim") else "  (没有红点, 瞄环心)"))
                    return
                last_why = "; ".join(why)
                if now - self.t_log > 1.0:
                    self.t_log = now
                    rospy.loginfo(f"    [align] 还不能锁定: {last_why}")
                if now - t_al > C.GATE_ALIGN_MAX_S:
                    self.fail(f"取样 {C.GATE_ALIGN_MAX_S:.0f}s 仍不收敛: {last_why}")
            self.tick()

    # ---------------- COMMIT ----------------
    def commit(self):
        P = self.plan
        self.set_seg("commit", "等确认")
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
            if self.keys is None:
                self.fail("要按键确认冲刺, 但没有终端键盘")
            back = "回手动段" if (C.GATE_ABORT_TO_MANUAL and self.back_ok) else "降落"
            rospy.logwarn(f"[gate] ★★ 按 G 冲刺 (之后不能中止, 只有 X 停桨)  |  M / 方向键 = 不冲了, 回手动段  "
                          f"|  L = 降落  |  {C.GATE_CONFIRM_TIMEOUT:.0f}s 不按 = {back}")
        rospy.logwarn("=" * 64)
        t_end = rospy.Time.now().to_sec() + C.GATE_CONFIRM_TIMEOUT
        while C.GATE_CONFIRM_DASH or need_y:
            self.hold(P.S)
            self.tick()
            if go_key in self.flags:
                rospy.logwarn(f"[gate] {go_key.upper()} → 冲刺 {P.v:.2f} m/s")
                break
            if need_y and 'g' in self.flags:
                rospy.logwarn(f"[gate] 这次降速了 ({P.v:.2f} < {C.DASH_V:.2f} m/s), G 不算 —— 接受就按 Y, 不接受按 M")
            now = rospy.Time.now().to_sec()
            if now > t_end:
                self.fail(f"{C.GATE_CONFIRM_TIMEOUT:.0f}s 没按 {go_key.upper()}")
            if now - self.t_log > 5.0:
                self.t_log = now
                lv = gate_in.latest_valid()
                if lv is not None and self.locked:
                    d = gp.vnorm(gp.vsub(lv["c"], self.locked["c"])) * 100
                    rospy.loginfo(f"    [commit] 等 G … 最新识别和锁定值差 {d:.0f}cm")
        # 停稳在起跑点
        t_end = rospy.Time.now().to_sec() + C.GATE_START_WAIT
        while True:
            p, v = self.pos(), self.vel()
            e = gp.vnorm(gp.vsub(p, P.S))
            if e < C.GATE_START_TOL and gp.vnorm(v) < C.GATE_START_V_MAX:
                return
            if rospy.Time.now().to_sec() > t_end:
                self.fail(f"起跑前没停稳: 离 S {e:.2f}m, 速度 {gp.vnorm(v):.2f}m/s")
            self.hold(P.S)
            self.tick()

    # ---------------- DASH / BRAKE ----------------
    def dash(self):
        P = self.plan
        tr = P.traj
        aim, d = P.aim, P.dir
        yaw_d = gp.yaw_of(d)
        self.yaw_sp = yaw_d

        def sampler(t):
            s, v, a = tr.at(t)
            return gp.vadd(aim, gp.vmul(d, s)), gp.vmul(d, v), gp.vmul(d, a)

        set_phase("gate")                                  # 速度包络放开到冲刺档
        self.back_ok = False
        dash_tilt_on[0] = True                             # 停桨线换成 GATE_DASH_TILT_DEG
        self.t0 = rospy.Time.now().to_sec() + 0.05
        setpoint.set_traj_mode(sampler, self.t0, yaw_d, C.GATE_ACC_FF)
        self.set_seg("dash", f"t0, {P.v:.2f}m/s, 预计 {tr.T:.2f}s 后停在 s={P.s_stop:+.2f}")
        stuck_since = None
        t_wait_end = tr.T + 3.0 * C.T_LAG
        warned_dev = False
        while True:
            now = rospy.Time.now().to_sec()
            t = now - self.t0
            s_p, v_p, _ = tr.at(t)
            p, v = self.pos(), self.vel()
            s_e = gp.vdot(gp.vsub(p, aim), d)
            v_e = gp.vdot(v, d)
            self.dash_state = (t, s_p, v_p, s_e, v_e)
            if t >= tr.t2 and self.seg == "dash":
                self.set_seg("brake", f"s_plan={s_p:+.2f} s_est={s_e:+.2f} v={v_e:.2f}")
            if not self.passed and s_e > 0.0:
                self.passed = True
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
                    self.lvl = 1
                    self.outcome = "stuck"
                    self.hold(p)
                    rospy.logerr(f"[gate] ★ 卡纸: 规划 s={s_p:+.2f} v={v_p:.2f} / 实测 s={s_e:+.2f} "
                                 f"v={v_e:.2f} —— 停止前推, 设定点 = 当前位置")
                    return
            else:
                stuck_since = None
            if t >= t_wait_end and math.hypot(v[0], v[1]) < C.GATE_STOP_V:
                break
            if t >= tr.T + C.GATE_STOP_EXTRA:
                rospy.logwarn(f"[gate] 轨迹结束 {C.GATE_STOP_EXTRA:.1f}s 还没停稳 "
                              f"(v={math.hypot(v[0], v[1]):.2f}m/s) —— 照样转悬停")
                break
            self.tick(dash=True)
        self.outcome = self.outcome or "done"
        rospy.logwarn(f"[gate] 刹停: 规划停在 s={P.s_stop:+.2f}, 估计 s={s_e:+.2f} "
                      f"(差 {s_e - P.s_stop:+.2f}m)  过纸面={'是' if self.passed else '否'}")


def phase_gate(z_hold):
    """
    穿环。返回 (x, y, z, yaw) —— 结束时的悬停点, main 从这里原地降落。

    流程 (2026-10-19): 手动段 ─T→ 找环 → 规划 → 摆位 → 锁定 → 等 G → 冲刺 → 刹停悬停
      冲刺前放弃 (GateStop.retry) 或按 M / 方向键 (GateBack) → 回手动段, 可以 T 重来;
      LAND / 接管 / 手动段超时 → 原地悬停后返回 (降落)。影子模式做完也回手动段。
      FATAL (X 键 / 倾角 / 断链) 照常抛 RuntimeError 给 main 的 cleanup。
    """
    set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
    run = GateRun(z_hold)
    if sys.stdin.isatty():
        try:
            run.keys = KeyReader()
            run.keys.__enter__()
        except Exception as e:
            rospy.logwarn(f"[gate] 键盘不可用 ({e})")
            run.keys = None
    rospy.loginfo("[gate] 键盘: 手动段 WASD/RF/QE/空格 + T = 开始 | 冲刺前 M 或方向键 = 回手动段 | "
                  "G = 确认冲刺 | L = 降落 | X = 停桨")
    can_retry = run.keys is not None and C.GATE_ABORT_TO_MANUAL
    manual_on = C.GATE_MANUAL_START
    why_manual = ""
    try:
        while True:
            try:
                if manual_on:
                    run.manual(why_manual)
                run.new_attempt()
                run.acquire()
                run.make_plan(run.acq, run.pos(), "粗规划")
                if C.GATE_SHADOW:
                    run.align(run.pos())
                    run.make_plan(run.locked, run.pos(), "锁定后规划 (影子)")
                    run.set_seg("hold", "影子模式: 不摆位不冲刺, 计划已发网页")
                    t_end = rospy.Time.now().to_sec() + 3.0
                    hp = run.pos()
                    while rospy.Time.now().to_sec() < t_end:
                        run.hold(hp)
                        run.tick()
                    run.outcome = "shadow"
                    if can_retry:
                        manual_on = True
                        why_manual = "影子模式做完了 (计划在网页上) —— 换个位置 T 再来, 或 L 降落"
                        continue
                else:
                    P = run.plan
                    run.move_to(P.S, "ring", C.GATE_REPO_TIMEOUT, "摆位")
                    run.turn_to(gp.yaw_of(P.dir), P.S, "对正进入方向")
                    run.align(P.S)
                    P2 = run.make_plan(run.locked, run.pos(), "锁定后规划")
                    gap = gp.vnorm(gp.vsub(P2.S, run.pos()))
                    if gap > C.GATE_START_TOL:
                        if gap > 0.5:
                            rospy.logwarn(f"[gate] 锁定后起跑点挪了 {gap:.2f}m —— 再挪过去 (不重新取样)")
                        run.move_to(P2.S, gp.yaw_of(P2.dir), C.GATE_REPO_TIMEOUT, "微调起跑点")
                    run.commit()
                    run.dash()
                break
            except GateStop as e:
                back = isinstance(e, GateBack)
                if (back or (e.retry and can_retry)) and takeover[0] is None and not land_flag.is_set():
                    run.hold(run.pos())
                    if back:
                        rospy.logwarn(f"[gate] {e.args[0]} —— 回手动段")
                        why_manual = "操作员中止这一次"
                    else:
                        rospy.logerr(f"[gate] 这一次放弃: {e.args[0]} —— 回手动段 (T 重来 / L 降落)")
                        why_manual = f"上一次放弃: {e.args[0]}"
                    run.last_why = str(e.args[0])
                    run.outcome = ""
                    manual_on = True
                    continue
                run.outcome = run.outcome or "abort"
                run.set_seg("abort", str(e.args[0]))
                rospy.logerr(f"[gate] 放弃: {e.args[0]} —— 原地悬停后降落")
                if takeover[0] is None:
                    run.hold(run.pos())
                break
        run.back_ok = False
        # 收尾: 以当前位置重锚 (冲刺中 VIO 可能跳过, 不回规划的 E 点)
        if takeover[0] is None and not abort_flag.is_set():
            hp = run.pos()
            run.hold(hp)
            if run.seg not in ("abort",):
                run.set_seg("hold", f"在当前位置悬停 {C.GATE_HOLD_S:.1f}s 后降落")
            set_phase("gate")                     # 刹停后可能还有残速, 用冲刺档包络
            t_end = rospy.Time.now().to_sec() + C.GATE_HOLD_S
            while rospy.Time.now().to_sec() < t_end and not stop_requested():
                run.hold(hp)
                try:
                    run.tick(dash=True)           # dash=True: LAND 不再抛 GateStop, 反正马上降
                except GateStop:
                    break
        dash_tilt_on[0] = False                   # 降落段回到平时的停桨线
        if run.land_deferred:
            rospy.logwarn(f"[gate] 冲刺中推迟的 LAND: {run.land_deferred} —— 现在降落")
        rospy.loginfo(f"[gate] 结束: {run.outcome or 'abort'}  (共 {run.n_try} 次)")
        p = run.pos()
        return p[0], p[1], p[2], run.yaw_sp
    finally:
        dash_tilt_on[0] = False
        if run.keys is not None:
            run.keys.__exit__(None, None, None)


def phase_descend(z_from, yaw_hold, hold_x, hold_y):
    """
    ZHOLD 缓降。xy 锁在 hold_x/hold_y (不一定是 origin)。

    · 包络按进入时的实际位置动态放宽 —— 见 set_phase 的注释。
    · TRACK_Z_MIN 的钳位只在 track 阶段生效，这里不适用: 降落本来就要穿过那条线。
    """
    px, py = tele.xy()
    horiz_now = math.hypot(px - INITIAL_X, py - INITIAL_Y)
    horiz_lim = max(C.PHASE_ENVELOPE["descend"][0], horiz_now + C.ENVELOPE_MARGIN)
    set_phase("descend", horiz=horiz_lim)

    z_target_final = INITIAL_Z + C.DESCEND_Z_TARGET_DZ
    rospy.loginfo(f"[descend] ZHOLD ramp z: {z_from:.2f} → {z_target_final:.2f} "
                  f"({C.DESCEND_SPEED}m/s), xy=({hold_x:+.2f},{hold_y:+.2f}) "
                  f"包络 {horiz_lim:.2f}m")

    # 从"指令高度"和"实际高度"里取低的那个起步。LAND 路径下 z_from 可能
    # 是个很旧的值 (比如起飞阶段就触发了 LAND)，从那里往下斜坡会白等几秒。
    zf0 = tele.z_filtered()
    target_z = min(z_from, zf0) if zf0 is not None else z_from
    rate = rospy.Rate(20)
    last = rospy.Time.now()
    t_end = rospy.Time.now() + rospy.Duration(C.DESCEND_TIMEOUT)
    t_log = rospy.Time.now()

    while rospy.Time.now() < t_end:
        check_abort()
        if takeover[0] is not None:
            rospy.logwarn("[descend] 控制权被接管 — 停止下降, 不再切 AUTO.LAND")
            return
        now = rospy.Time.now()
        dt = min((now - last).to_sec(), 0.1)
        last = now

        target_z = max(target_z - C.DESCEND_SPEED * dt, z_target_final)
        setpoint.set_zhold_mode(hold_x, hold_y, target_z, yaw_hold, 0.0, 0.0)

        if (now - t_log).to_sec() >= 0.3:
            zf = tele.z_filtered()
            dz_real = (zf - INITIAL_Z) if zf is not None else float('nan')
            rospy.loginfo(f"    target_dz={target_z-INITIAL_Z:+.2f}  "
                          f"real_dz={dz_real:+.2f}")
            t_log = now

        if target_z <= z_target_final + 1e-3:
            zf = tele.z_filtered()
            if zf is not None and abs(zf - z_target_final) < C.DESCEND_REACH_TOL:
                rospy.loginfo("[descend] 到达低高度，准备 auto_land")
                return
        rate.sleep()

    rospy.logwarn("[descend] TIMEOUT — 强行进入 auto_land")


def auto_land_timeout():
    """
    AUTO.LAND 等待时间, 按当前高度放大 (下限 AUTO_LAND_TIMEOUT)。
    descend 超时后可能从高处切 AUTO.LAND, 写死 15s 会在 PX4 还没降完时就超时。
    """
    zf = tele.z_filtered()
    h = max(0.0, (zf - INITIAL_Z)) if zf is not None else C.MAX_HEIGHT
    return max(C.AUTO_LAND_TIMEOUT, h / C.AUTO_LAND_SPEED_EST * 1.5 + 10.0)


def phase_auto_land(set_mode):
    px, py = tele.xy()
    horiz_now = math.hypot(px - INITIAL_X, py - INITIAL_Y)
    horiz_lim = max(C.PHASE_ENVELOPE["auto_land"][0], horiz_now + C.ENVELOPE_MARGIN)
    set_phase("auto_land", horiz=horiz_lim)

    # 被接管了就不能再切模式 —— 那是在抢飞手的控制权
    if takeover[0] is not None:
        rospy.logwarn(f"[auto_land] 控制权在 {takeover[0]} — 不切 AUTO.LAND")
        return False

    voluntary_land[0] = True  # 必须先设 flag 再切 mode
    rospy.loginfo("[auto_land] 切 AUTO.LAND")
    try:
        r = set_mode(custom_mode="AUTO.LAND")
        if not r.mode_sent:
            rospy.logwarn("[auto_land] SetMode not sent")
            return False
    except Exception as e:
        rospy.logerr(f"[auto_land] exception: {e}")
        return False

    tmo = auto_land_timeout()
    rospy.loginfo(f"[auto_land] 等待 PX4 落地上锁 (最长 {tmo:.0f}s)")
    t_end = rospy.Time.now() + rospy.Duration(tmo)
    while rospy.Time.now() < t_end:
        if abort_flag.is_set():
            return False
        if takeover[0] is not None:
            # AUTO.LAND 之后飞手又切走了 (safety_monitor 现在也看这种情况)
            rospy.logwarn(f"[auto_land] 降落中控制权被接管 ({takeover[0]}) — 停手")
            return False
        if tele.state and not tele.state.armed:
            rospy.loginfo("[auto_land] PX4 已在着陆时 disarm")
            in_air[0] = False
            return True
        rospy.sleep(0.3)
    rospy.logwarn("[auto_land] TIMEOUT — PX4 未在窗口内 disarm "
                  "(不会因此停桨: cleanup 看到仍是 AUTO.LAND 会让 PX4 自己降完)")
    return False


def phase_handover():
    """
    控制权被接管后的唯一阶段。

    脚本在这里**什么都不主动做**:
      · 不切模式, 不停桨
      · 飞控不在 OFFBOARD 时, 把设定点持续更新成"当前位置" —— 这样如果飞手
        又切回 OFFBOARD, 飞机原地悬停, 而不是冲向一个几秒前的旧设定点;
        回到 OFFBOARD 后设定点冻结在那一刻 (不跟着漂)
      · 等飞机上锁 (飞手/PX4 已经降落) 后正常退出
    键盘: 只有 X 有效 (人工停桨)。Ctrl+C 退出脚本**不动电机**。
    """
    set_phase("handover", horiz=1e9, vel=1e9)   # 包络不再适用: 飞机不归脚本管
    sub_mode_ref[0] = ""
    rospy.logerr("=" * 64)
    rospy.logerr(f"[handover] 控制权在 {takeover[0]} —— 脚本只看不动, 等飞机落地上锁")
    rospy.logerr("[handover] X = 紧急停桨  |  Ctrl+C = 退出脚本(不动电机)")
    rospy.logerr("[handover] 若飞手切回 OFFBOARD: 飞机原地悬停, 用 QGC 切 Land 降落")
    rospy.logerr("=" * 64)

    keys = None
    if sys.stdin.isatty():
        try:
            keys = KeyReader()
            keys.__enter__()
        except Exception:
            keys = None
    rate = rospy.Rate(C.PHASE_LOOP_HZ)
    t_log = rospy.Time.now()
    try:
        while not rospy.is_shutdown():
            check_abort()
            st = tele.state
            if st is not None and not st.armed:
                in_air[0] = False
                rospy.loginfo(f"[handover] 飞机已上锁 (模式 {st.mode}) — 退出")
                return
            if keys is not None:
                _, flags, eof, _ = keys.poll()
                if 'x' in flags:
                    request_abort("handover X 键")
                    check_abort()
                # EOF 在这里没有动作可做: 降落本来就不归脚本管
            if st is not None and st.mode != "OFFBOARD" and tele.pose is not None:
                p = tele.pose.pose.position
                setpoint.set_zhold_mode(p.x, p.y, p.z, tele.yaw_measured(), 0.0, 0.0)
            now = rospy.Time.now()
            if (now - t_log).to_sec() >= 3.0:
                zf = tele.z_filtered()
                dz = (zf - INITIAL_Z) if zf is not None else float('nan')
                rospy.logwarn(f"[handover] 模式 {st.mode if st else '?'}  dz={dz:+.2f}  "
                              f"等待落地上锁…")
                t_log = now
            rate.sleep()
    finally:
        if keys is not None:
            keys.__exit__(None, None, None)


def on_ground():
    """
    cleanup 用的"飞机是不是已经在地上"。
    优先信 PX4 的 landed_state; 读不到时才退回看 VIO 高度
    (只看 VIO 高度不可靠: 它在空中也可能往下漂 —— z_drop_events.log 就是为这个记的)。
    """
    if not in_air[0]:
        return True                       # 从没检测到离地
    _, ext = tele.snap_ekf()
    ls = getattr(ext, "landed_state", 0) if ext is not None else 0
    if ls == 1:                           # LANDED_STATE_ON_GROUND
        return True
    if ls in (2, 3, 4):                   # IN_AIR / TAKEOFF / LANDING
        return False
    zf = tele.z_filtered()
    return zf is not None and (zf - INITIAL_Z) < C.TAKEOFF_LIFTOFF_DZ


def cleanup_decision(fatal, takeover_mode, armed, grounded, mode):
    """
    脚本退出时对电机做什么 (纯函数)。

    返回:
      "none"      已上锁, 什么都不用做
      "disarm"    force_disarm (停桨)
      "leave"     什么都不做, 让当前控制方(飞手 / PX4 的 AUTO.LAND)继续
      "auto_land" 请求 AUTO.LAND, 不停桨

    空中停桨只有一个来源: FATAL (姿态失控 / MAVROS 断链 / 操作员按 X)。
    代码异常、Ctrl+C、AUTO.LAND 等太久、飞手接管都不停桨。
    """
    if not armed:
        return "none"
    if fatal:
        return "disarm"
    if grounded:
        return "disarm"                   # 地上: 停桨无害, 也是必须的
    if takeover_mode is not None:
        return "leave"                    # 飞手/PX4 在控制, 不碰
    if mode == "AUTO.LAND":
        return "leave"                    # PX4 正在自己降, 让它降完
    return "auto_land"


def restore_ground_mode(set_mode, grounded):
    """
    退出前整理飞控模式, 让下一次运行从干净状态开始。

    只在 "地上 + 已上锁 + 没被接管 + MAVROS 在线" 时做 —— 空中切模式会改变飞行,
    被接管时切模式是抢飞手的控制权。不满足就什么都不做。
    2026-10-19 (首飞问题 §5): 只有停在 **OFFBOARD** 才需要切走 (停发设定点后地上会触发
    offboard 丢失保护)。正常降落后是 AUTO.LAND, 保持不动 —— 室内没遥控、没全局位置时
    PX4 本来就不接受 MANUAL / AUTO.LOITER, 以前每次都白报两条警告。
    OFFBOARD 时依次试: 开局模式 → PARK_MODE → AUTO.LAND (PX4 只要有本地位置就接受)。
    返回最终模式 (没动则 None)。
    """
    st = tele.state
    if st is None or not st.connected or st.armed or not grounded:
        return None
    if takeover[0] is not None:
        return None
    if st.mode != "OFFBOARD":
        rospy.loginfo(f"cleanup: 模式 {st.mode}, 保持不动")
        return st.mode
    tried = []
    for m in (start_mode[0], C.PARK_MODE, "AUTO.LAND"):
        if not m or m == "OFFBOARD" or m in tried:
            continue
        tried.append(m)
        try:
            r = set_mode(custom_mode=m)
            if not r.mode_sent:
                rospy.loginfo(f"cleanup: 切 {m} 没发出去")
                continue
        except Exception as e:
            rospy.loginfo(f"cleanup: 切 {m} 异常: {e}")
            continue
        t_end = time.time() + 1.0          # time.time: 不依赖 ROS 时钟
        while time.time() < t_end:
            if tele.state and tele.state.mode == m:
                rospy.loginfo(f"cleanup: 模式 OFFBOARD → {m}")
                return m
            time.sleep(0.05)
        rospy.loginfo(f"cleanup: PX4 没接受 {m}")
    cur = tele.state.mode if tele.state else "?"
    rospy.logwarn(f"cleanup: 停在 {cur} 切不走 (试过 {', '.join(tried)}) —— 下一次开局会先切 {C.PARK_MODE}")
    return None


def _flush_stdin():
    """丢掉终端里积压的按键, 免得之前误按的回车直接把确认提示答掉。"""
    try:
        import termios
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass


def arm_with_confirm(arm_srv):
    """
    解锁。被拒时打印 MAV_RESULT + 飞控 STATUSTEXT 原文, 然后**等操作员回车确认**再重试。

    不自动重试: 被拒后现场的人可能已经走到飞机旁边看情况, 这时自动重试一旦成功,
    飞机会立刻解锁并直接进入起飞。等待确认期间 setpoint 照常发布, 模式保持 OFFBOARD。
    """
    for attempt in range(1, C.ARM_MAX_TRIES + 1):
        check_abort()
        cur = tele.state.mode if tele.state else "?"
        if cur != "OFFBOARD":
            raise RuntimeError(f"arm 前模式已不是 OFFBOARD (当前 {cur})")

        t0 = rospy.Time.now().to_sec()
        rospy.loginfo(f"[arm] arming (第 {attempt}/{C.ARM_MAX_TRIES} 次)")
        r = arm_srv(True)

        if r.success:
            t_end = rospy.Time.now() + rospy.Duration(C.ARM_TIMEOUT)
            while rospy.Time.now() < t_end:
                check_abort()
                if tele.state and tele.state.armed:
                    # 正常解锁时 PX4 会发 "Armed by external command"。一条都没有
                    # 说明 STATUSTEXT 没通 —— 下次被拒就看不到原因, 提前喊出来。
                    # 放后台线程里查, 不耽误起飞。
                    def _check_fcu_link(t_ref=t0):
                        if not fcu_texts_since(t_ref - 0.2):
                            rospy.logwarn("[arm] 解锁后 2s 内没收到任何飞控消息 —— STATUSTEXT "
                                          "链路不通, 下次被拒时看不到原因 "
                                          "(查 rostopic echo /mavros/statustext/recv)")
                    _tm = threading.Timer(2.0, _check_fcu_link)
                    _tm.daemon = True
                    _tm.start()
                    return
                rospy.sleep(0.05)
            raise RuntimeError("arm not confirmed")

        # STATUSTEXT 通常比 COMMAND_ACK 晚几十 ms 到, 等一下再取
        rospy.sleep(1.0)
        res = getattr(r, "result", -1)
        rospy.logerr(f"[arm] 第 {attempt} 次被拒  MAV_RESULT={res} "
                     f"({_MAV_RESULT.get(res, '?')})")
        why = fcu_texts_since(t0 - 0.2)
        if why:
            for txt in why:
                rospy.logerr(f"[arm]   飞控: {txt}")
        else:
            rospy.logerr("[arm]   没收到任何飞控消息 —— STATUSTEXT 链路不通, 看不到拒绝原因。"
                         "看 QGC 消息栏, 或在 QGC MAVLink 控制台跑 commander check")

        if attempt >= C.ARM_MAX_TRIES:
            break
        _flush_stdin()
        ans = input(f"\n[arm] 确认桨周围无人后按回车重试 ({attempt + 1}/{C.ARM_MAX_TRIES}), "
                    f"输入 q 回车退出: ").strip().lower()
        if ans == "q":
            break

    raise RuntimeError("arm rejected")


def single_instance_guard():
    """
    已有飞控脚本在跑就拒绝启动。
      · 同名 /flight_ctrl: ROS master 会直接关掉**正在飞的那个** (它的 cleanup 也发不出指令)
      · 旧版 /stage9_track: 不会被关, 两个脚本同时往飞控发设定点
    master 里登记着但进程已死 (上次被强杀) → 只警告, 新节点会顶掉旧登记。
    """
    try:
        import rosgraph
        import socket
        import xmlrpc.client
        master = rosgraph.Master("/flight_ctrl_precheck")
        for name in ("/" + C.NODE_NAME, "/stage9_track"):
            try:
                uri = master.lookupNode(name)
            except rosgraph.MasterError:
                continue                   # 没登记
            old = socket.getdefaulttimeout()
            socket.setdefaulttimeout(1.0)
            try:
                xmlrpc.client.ServerProxy(uri).getPid("/flight_ctrl_precheck")
                alive = True
            except Exception:
                alive = False
            finally:
                socket.setdefaulttimeout(old)
            if alive:
                print("\n" + "!" * 78)
                print(f"拒绝启动: 飞控脚本 {name} 正在运行 ({uri})")
                print("  同名节点会把正在飞的那个直接关掉; 旧版脚本则会和本脚本同时发设定点。")
                print("  先结束那个脚本 (它的终端里按 L 降落 / Ctrl+C), 再启动。")
                print("!" * 78 + "\n")
                sys.exit(1)
            print(f"[guard] {name} 在 master 里有登记但进程已不在 (上次被强杀?) —— 继续")
    except SystemExit:
        raise
    except Exception as e:
        # roscore 都连不上时 init_node 自己会卡住/报错, 这里不替它拦
        print(f"[guard] 查重失败 (跳过检查): {e}")


def emergency_land(set_mode, z_now, yaw_hold):
    """
    LAND 级 abort 的降落路径: 就地冻结 xy → 缓降 → AUTO.LAND。
    不回原点 —— 触发 LAND 的原因往往正是"位置/速度不可信"。
    """
    px, py = tele.xy()
    rospy.logwarn(f"[emergency_land] 就地降落 @ ({px:+.2f},{py:+.2f})")
    try:
        phase_descend(z_now, yaw_hold, px, py)
    except Exception as e:
        rospy.logerr(f"[emergency_land] descend 异常: {e}")
    if abort_flag.is_set():
        return False
    return phase_auto_land(set_mode)


# =============================================================================
# ==========================   Pose callback   ================================
# =============================================================================
def pose_cb(msg):
    """
    回调里只做两件极快的事: 更新 telemetry、往队列塞一个元组。
    ROS 单订阅者的回调是串行的, 这里一次几百 ms 的文件 IO 就会把下一帧
    pose 顶到 POSE_STALE_SEC 之外。timing.csv 由后台线程落盘。
    """
    prev_stamp = tele.pose_stamp_prev
    tele.set_pose(msg)
    try:
        now = rospy.Time.now().to_sec()
        stamp = msg.header.stamp.to_sec()
        if prev_stamp is not None and prev_stamp.to_sec() > 0:
            interval_ms = (stamp - prev_stamp.to_sec()) * 1000.0
        else:
            interval_ms = float('nan')
        with timing_queue_lock:
            timing_queue.append((now, stamp, (now - stamp) * 1000.0,
                                 interval_ms, tele.stamp_backward_count,
                                 tele.last_jump))
    except Exception:
        pass


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

    rospy.Subscriber("/mavros/state", State, tele.set_state)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local",
                     TwistStamped, tele.set_velocity)
    rospy.Subscriber("/mavros/battery", BatteryState, tele.set_battery)
    rospy.Subscriber("/mavros/target_actuator_control",
                     ActuatorControl, tele.set_actuator)
    rospy.Subscriber("/mavros/setpoint_raw/target_local",
                     PositionTarget, tele.set_target_local)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, tele.set_vins_odom)
    rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                     tele.set_estimator_status)
    rospy.Subscriber("/mavros/extended_state", ExtendedState,
                     tele.set_extended_state)
    rospy.Subscriber(C.GATE_STATUS_TOPIC, String, gate_in.on_status, queue_size=5)
    rospy.Subscriber(C.CAM_STATUS_TOPIC, String, gate_in.on_cam, queue_size=1)
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

    phase_ref[0] = "preflight"          # 网页显示用 (safety_monitor 此时还没起)
    if not preflight(param_get, set_mode, param_pull):
        phase_ref[0] = "preflight_failed"
        rospy.sleep(0.5)                # 让网页来得及看到"preflight 失败"
        return 1

    publish_config(config_pub)          # 原点已确定, 再发一次
    print_config_summary()

    setpoint.set_velocity_mode(0.0, 0.0, 0.0, 0.0)

    rospy.loginfo("=" * 64)
    rospy.loginfo(f"Preflight OK. Log dir: {log_dir}")
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
            if tele.state and tele.state.mode == "OFFBOARD":
                entered_offboard = True
                break
            rospy.sleep(0.05)
        if not entered_offboard:
            cur = tele.state.mode if tele.state else "?"
            raise RuntimeError(f"OFFBOARD not entered, mode={cur}")
        rospy.loginfo("[offboard] OK")

        set_phase("arm")
        arm_with_confirm(arm_srv)
        rospy.logwarn("*** ARMED ***")

        # ---------------- 飞行序列 ----------------
        z_top = phase_takeoff()
        gx, gy = tele.xy()
        gz, gyaw = z_top, INITIAL_YAW

        if not land_flag.is_set():
            check_abort()
            gx, gy, gz, gyaw = phase_gate(z_top)

        check_abort()

        # 穿环之后不返航: 不管成功/放弃/LAND, 都在当前位置原地降落
        if takeover[0] is None:
            if land_flag.is_set():
                rospy.logwarn(f"[main] LAND 路径: {land_reason[0]}")
                emergency_land(set_mode, gz, gyaw)
            else:
                phase_descend(gz, gyaw, gx, gy)
                phase_auto_land(set_mode)
        # 任何阶段 (包括上面的降落过程中) 被接管 → 只看不动
        if takeover[0] is not None:
            phase_handover()

    except KeyboardInterrupt:
        # disable_signals=True 后 Ctrl+C 落到这里, ROS 仍活着 → 下面 cleanup 的
        # 切 AUTO.LAND / 还原模式都能真正发出去。和普通异常同等处理: 不停桨。
        rospy.logerr("[EXCEPTION] Ctrl+C / SIGTERM")
        exc_reason[0] = "Ctrl+C / SIGTERM"

    except Exception as e:
        rospy.logerr(f"[EXCEPTION] {e}")
        rospy.logerr(traceback.format_exc())       # 完整堆栈 → events.log
        # 普通异常不是 FATAL (FATAL 只认 request_abort), 不停桨。
        exc_reason[0] = str(e)

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)     # cleanup 期间再按 Ctrl+C 不能打断它
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        fatal = abort_flag.is_set()          # 只有 request_abort 会置它 (在下面置位之前读)
        st = tele.state
        armed = bool(st and st.armed)
        mode = st.mode if st else "?"
        grounded = on_ground()
        action = cleanup_decision(fatal, takeover[0], armed, grounded, mode)
        # 穿环后多半落在离起飞点几米外: cleanup 不再按起飞点查水平围栏 (否则落地后还报越界)
        set_phase("cleanup", horiz=1e9, vel=1e9)

        if fatal:
            rospy.logerr(f"ABORT PATH [{SEV_FATAL}]: {abort_reason[0]} — motors stop NOW")
        elif takeover[0] is not None:
            rospy.logwarn(f"TAKEOVER PATH: 控制权在 {takeover[0]}")
        elif exc_reason[0] is not None:
            rospy.logerr(f"EXCEPTION PATH: {exc_reason[0]}")
        elif land_reason[0] is not None:
            rospy.logwarn(f"LAND PATH: {land_reason[0]}")
        rospy.logwarn(f"cleanup: armed={armed} 模式={mode} 在地上={grounded} → {action}")

        # 请求 AUTO.LAND 必须在停发设定点之前: 停发后 PX4 会走 OFFBOARD 丢失保护
        if action == "auto_land":
            voluntary_land[0] = True
            try:
                r = set_mode(custom_mode="AUTO.LAND")
                sent = bool(r.mode_sent)
            except Exception as e:
                sent = False
                rospy.logerr(f"cleanup: 切 AUTO.LAND 异常: {e}")
            t_end = time.time() + 2.0
            while time.time() < t_end and not (tele.state and tele.state.mode == "AUTO.LAND"):
                time.sleep(0.05)
            if tele.state and tele.state.mode == "AUTO.LAND":
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

        abort_flag.set()                     # 停 setpoint 发布 / 监控 / 日志线程
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
                while time.time() < t_end and tele.state and tele.state.armed:
                    time.sleep(0.05)
                try:
                    restore_ground_mode(set_mode, grounded)
                except Exception as e:
                    rospy.logwarn(f"cleanup: 还原模式异常 (忽略): {e}")
        elif action == "leave":
            rospy.logwarn(f"cleanup: 不动电机 —— 当前由 {takeover[0] or mode} 控制")

        if fatal:
            rospy.logerr(f"Terminated [{SEV_FATAL}]: {abort_reason[0]}")
        elif takeover[0] is not None:
            rospy.logwarn(f"Terminated [TAKEOVER]: 控制权在 {takeover[0]}")
        elif exc_reason[0] is not None:
            rospy.logerr(f"Terminated [EXCEPTION]: {exc_reason[0]}")
        elif land_reason[0]:
            rospy.logwarn(f"Terminated [{SEV_LAND}]: {land_reason[0]}")
        else:
            rospy.loginfo("*** FLIGHT COMPLETE ***")

        for f in (csv_main, csv_vins, csv_ekf, csv_timing, csv_gate):
            if f:
                try:
                    f.flush()
                except Exception:
                    pass
        fsync_all_logs()                     # close() 只到内核缓存; 撞机后马上拔电池也要留住
        for f in (csv_main, csv_vins, csv_ekf, csv_timing, csv_gate):
            if f:
                try:
                    f.close()
                except Exception:
                    pass
        rospy.loginfo(f"All logs closed (fsync 过, 现在可以断电). Dir: {log_dir}")
        fsync_all_logs()                     # 把上面这行 events.log 也落盘

        # 恢复 Python 默认的 Ctrl+C 行为 (抛 KeyboardInterrupt)。SIG_DFL 会直接杀进程。
        signal.signal(signal.SIGINT, signal.default_int_handler)

    ok_end = (abort_reason[0] is None and exc_reason[0] is None and takeover[0] is None)
    return 0 if ok_end else 1


if __name__ == "__main__":
    _rc = 1
    try:
        _rc = main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        # preflight 阶段 (还没发任何设定点、没切模式) 按的 Ctrl+C 会到这里
        rospy.logwarn("Interrupted before flight sequence")
        _rc = 1
    finally:
        # disable_signals=True 后 rospy 不会自己关, 退出前手动关
        try:
            rospy.signal_shutdown("flight script exit")
        except Exception:
            pass
    sys.exit(_rc)

