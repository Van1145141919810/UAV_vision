# -*- coding: utf-8 -*-
"""
vb09/flight/devices.py —— 飞机的输入输出: 遥测汇聚 (Telemetry) / 设定点 (SetpointState) / 终端键盘 (KeyReader)
===============================================================================
2026-11-01 由 telemetry.py / setpoint.py / keys.py 合并 (代码原样; 这三个都在 2026-10-30 从 stage12_gate.py 原样搬出)。
共享状态是 F.名字 (vb09/flight/ctx.py), 参数是 C.名字。
  Telemetry      mavros / VINS 回调的数据 (位姿、速度、状态、电池 …) + 四元数小函数 yaw_from_quat / rpy_deg_from_quat / tilt_deg_from_quat
  SetpointState  三种发布模式 (速度 / 定高 / 轨迹) 和 PositionTarget 组包
  KeyReader      cbreak 读键盘, EOF = SSH 断连; _flush_stdin
"""
import math
import sys
import threading

import rospy
from mavros_msgs.msg import PositionTarget

from vb09.common import geom
from vb09.flight import config as C
from vb09.flight.ctx import F


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
                return F.INITIAL_X, F.INITIAL_Y
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


def _flush_stdin():
    """丢掉终端里积压的按键, 免得之前误按的回车直接把确认提示答掉。"""
    try:
        import termios
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass
