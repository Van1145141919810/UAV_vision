# -*- coding: utf-8 -*-
"""
vb09/flight/setpoint.py —— setpoint 状态 (速度 / 定高 / 轨迹三种模式) 和 PositionTarget 组包
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import threading

import rospy
from mavros_msgs.msg import PositionTarget


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
