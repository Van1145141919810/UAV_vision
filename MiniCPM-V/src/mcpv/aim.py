# -*- coding: utf-8 -*-
"""像素 → 飞行方向：把人点的那个点，变成"往哪飞、飞多远"。

这里全部是几何，**不调用任何模型**。

坐标约定
    相机系：x 右、y 下、z 前（OpenCV 习惯，和 stereo_calib.yaml 一致）
    机体系：x 前、y 左、z 上（MAVROS / ROS FLU）
    相机装反了/歪了，用 pitch/roll/yaw 三个安装角补，写在 config 里。

用法
    from mcpv.aim import Aimer
    aim = Aimer.from_config(image_width, image_height)
    cmd = aim.command(u, v, step_m=3.0)
    print(cmd.describe())     # "前 2.99 m  左 0.12 m  上 0.05 m"
"""

from dataclasses import dataclass
import math

import numpy as np

from . import config as C


@dataclass
class MoveCommand:
    """机体系位移指令（米）。FLU：前/左/上为正。"""

    forward: float
    left: float
    up: float
    distance_m: float | None = None   # 如果估出了目标距离
    yaw_deg: float = 0.0              # 建议机头偏转（正 = 左转）
    arrived: bool = False             # 已经够近，不该再飞
    range_unknown: bool = True        # 距离没估出来（单目看不到深度）

    def describe(self) -> str:
        def fmt(v, pos, neg):
            return f"{pos} {abs(v):.2f} m" if v >= 0 else f"{neg} {abs(v):.2f} m"
        parts = [fmt(self.forward, "前", "后"),
                 fmt(self.left, "左", "右"),
                 fmt(self.up, "上", "下")]
        s = "  ".join(parts)
        if self.distance_m:
            s += f"   （目标距离约 {self.distance_m:.1f} m）"
        elif self.range_unknown:
            s += "   （距离未知，按固定步长走）"
        if self.arrived:
            s += "   ★ 已到位，建议停"
        return s

    def to_dict(self) -> dict:
        return {
            "forward_m": round(self.forward, 3),
            "left_m": round(self.left, 3),
            "up_m": round(self.up, 3),
            "distance_m": None if self.distance_m is None else round(self.distance_m, 2),
            "yaw_deg": round(self.yaw_deg, 2),
        }


class Aimer:
    """相机内参 + 安装角 → 像素到方向的换算。"""

    def __init__(self, fx, fy, cx, cy,
                 pitch_down_deg=0.0, roll_deg=0.0, yaw_deg=0.0):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.R_bc = self._r_body_cam(pitch_down_deg, roll_deg, yaw_deg)

    # ---------- 构造 ----------
    @classmethod
    def from_config(cls, width: int, height: int):
        """按图像尺寸换算内参。

        只有"同一台相机换分辨率"时才能直接等比缩放（此时长宽比一致）。
        长宽比不一致 ⇒ 这多半不是识别相机拍的（比如手机照片），
        内参只能近似：按宽度缩放，主点按画面中心估。
        """
        s = width / C.CALIB_W
        same_aspect = abs((width / height) - (C.CALIB_W / C.CALIB_H)) < 0.03
        cx = C.CX * s
        cy = C.CY * s if same_aspect else height / 2.0
        return cls(fx=C.FX * s, fy=C.FY * s, cx=cx, cy=cy,
                   pitch_down_deg=C.CAM_PITCH_DOWN_DEG,
                   roll_deg=C.CAM_ROLL_DEG,
                   yaw_deg=C.CAM_YAW_DEG)

    @staticmethod
    def _r_body_cam(pitch_deg, roll_deg, yaw_deg) -> np.ndarray:
        """相机系 → 机体系的旋转。

        先做固定的轴变换（相机 x右/y下/z前 → 机体 x前/y左/z上），
        再叠加安装误差角：绕机体 Y 轴（俯仰）、X 轴（横滚）、Z 轴（偏航）。
        """
        # 固定轴变换
        R0 = np.array([[0, 0, 1],
                       [-1, 0, 0],
                       [0, -1, 0]], dtype=float)
        th = math.radians(pitch_deg)
        ro = math.radians(roll_deg)
        ya = math.radians(yaw_deg)
        Ry = np.array([[math.cos(th), 0, math.sin(th)],
                       [0, 1, 0],
                       [-math.sin(th), 0, math.cos(th)]])
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(ro), -math.sin(ro)],
                       [0, math.sin(ro), math.cos(ro)]])
        Rz = np.array([[math.cos(ya), -math.sin(ya), 0],
                       [math.sin(ya), math.cos(ya), 0],
                       [0, 0, 1]])
        return Rz @ Rx @ Ry @ R0

    # ---------- 核心 ----------
    def ray_body(self, u: float, v: float) -> np.ndarray:
        """像素 (u,v) → 机体系单位方向向量（x 前、y 左、z 上）。"""
        d_cam = np.array([(u - self.cx) / self.fx,
                          (v - self.cy) / self.fy,
                          1.0])
        d_cam /= np.linalg.norm(d_cam)
        d_body = self.R_bc @ d_cam
        return d_body / np.linalg.norm(d_body)

    @staticmethod
    def range_from_width(width_px, real_m, f_px):
        return Aimer.range_from_size(width_px, real_m, f_px)

    @staticmethod
    def range_from_size(size_px: float, real_m: float, f_px: float) -> float:
        """已知实物尺寸，单目测距：Z = f·W / w"""
        if size_px <= 0:
            return float("inf")
        return f_px * real_m / size_px

    def command(self, u: float, v: float,
                step_m: float = 3.0,
                distance_m: float | None = None,
                standoff_m: float = 1.0) -> MoveCommand:
        """朝 (u,v) 这个方向飞 step_m 米。

        distance_m 给了就先算"还能靠近多少"，避免冲过头。
        """
        n = self.ray_body(u, v)

        step = step_m
        arrived = False
        if distance_m is not None:
            room = max(0.0, distance_m - standoff_m)
            if room < C.MIN_STEP_M:
                arrived = True
                step = 0.0
            else:
                step = min(step, room)

        d = n * step
        return MoveCommand(forward=float(d[0]), left=float(d[1]), up=float(d[2]),
                           distance_m=distance_m,
                           yaw_deg=math.degrees(math.atan2(n[1], n[0])),
                           arrived=arrived,
                           range_unknown=(distance_m is None))

    def project_hint(self, u: float, v: float, step_m: float) -> tuple[int, int]:
        """把这个位移在画面上画出来：返回位移终点的像素坐标（仅用于可视化）。"""
        n = self.ray_body(u, v)
        # 把位移向量转回相机系，再乘一个"视觉缩放"，让箭头长度可控
        d_cam = np.linalg.inv(self.R_bc) @ (n * step_m)
        if d_cam[2] <= 1e-6:
            return int(u), int(v)
        u2 = self.cx + self.fx * d_cam[0] / d_cam[2]
        v2 = self.cy + self.fy * d_cam[1] / d_cam[2]
        return int(u2), int(v2)
