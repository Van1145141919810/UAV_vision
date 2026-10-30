# -*- coding: utf-8 -*-
"""
vb09/flight/telemetry.py —— mavros / VINS 遥测汇聚 (Telemetry) + 四元数小函数
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import math
import threading

import rospy
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
