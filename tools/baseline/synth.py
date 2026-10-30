# -*- coding: utf-8 -*-
"""
synth.py —— 等价验证用的合成场景 (确定性): 双目校正图 + 飞机位姿 + 环/红点真值
===============================================================================
没有实录 (gcs_gate --record) 时, det_replay / fuse_replay / judge_replay 都用这里的序列。
不追求逼真, 只追求: 同一份代码 → 同一份输出; 并且能走到检测/融合/判定的主要分支
(远处单目环、中距离角点平面、红点双目、环部分出画、没有红点的帧、环丢失)。

坐标: world = ENU。环心 C, 法向 N 指向进近侧。靶面轴 d = −N, right = norm(d × z), up = right × d。
相机: T_WC = T_WB · T_BC (原始左目光学系); 校正系 P_rect = R1 · P_cam。
"""
import math

import cv2
import numpy as np

# ---- 场景 ----
C = np.array([6.0, 0.30, 1.40])
N = np.array([-1.0, -0.12, 0.0]); N /= np.linalg.norm(N)
DOT_OFF = (0.08, -0.05)            # 红点相对环心 (right, up) m
R_IN, R_OUT, SQ, DOT_R = 0.60, 0.80, 0.05, 0.055
CAM = {"cam_pitch_deg": 5.23, "cam_roll_deg": 3.40, "cam_yaw_deg": -0.73,
       "cam_x": 0.080, "cam_y": 0.030, "cam_z": -0.115}
HZ = 15.0
T0 = 1000.0                        # 序列起点 (秒, 任意)


def target_axes():
    d = -N
    right = np.cross(d, [0.0, 0.0, 1.0]); right /= np.linalg.norm(right)
    up = np.cross(right, d)
    return right, up


def aim_world():
    right, up = target_axes()
    return C + DOT_OFF[0] * right + DOT_OFF[1] * up


def make_rig():
    """1920×1080 全分辨率内参 + 60mm 基线 + 小旋转 (校正旋转 R1 ≈ 2.4°), 工作分辨率 0.5。"""
    import _compat
    K = np.array([[1579.0, 0, 959.5], [0, 1579.0, 539.5], [0, 0, 1.0]])
    D = np.zeros(5)
    R, _ = cv2.Rodrigues(np.array([0.002, -0.004, 0.001]))
    T = np.array([-0.06008, 0.0004, -0.0025])
    return _compat.StereoRig()(K, D, K, D, R, T, (1920, 1080), 0.5)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def R_from_quat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def make_T_BC():
    R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    p, r, yw = np.radians([CAM["cam_pitch_deg"], CAM["cam_roll_deg"], CAM["cam_yaw_deg"]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Rz = np.array([[np.cos(yw), -np.sin(yw), 0], [np.sin(yw), np.cos(yw), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Rx @ Ry @ R0
    T[:3, 3] = [CAM["cam_x"], CAM["cam_y"], CAM["cam_z"]]
    return T


def trajectory():
    """
    → [(t, p_body(3), q_body(xyzw), flags)]。flags: dot (红点在纸上吗), visible (渲染环吗)。
    段: 远处悬停 (8.5 m) → 抵近到 2.6 m → 横移让环部分出画 → 回中 → 丢环几帧 (不渲染) → 回来。
    """
    out = []
    k = 0
    def add(p, yaw, pitch=0.0, roll=0.0, dot=True, visible=True):
        nonlocal k
        out.append((T0 + k / HZ, np.array(p, float), quat_from_rpy(roll, pitch, yaw),
                    {"dot": dot, "visible": visible}))
        k += 1
    start = C + 8.5 * N
    start[2] = 1.5
    def yaw_to(p):
        v = C - p
        return math.atan2(v[1], v[0])
    for i in range(10):                                    # 远处悬停
        p = start + np.array([0.0, 0.01 * math.sin(i), 0.0])
        add(p, yaw_to(p) + 0.01 * math.sin(0.7 * i))
    end = C + 2.6 * N
    end[2] = 1.45
    for i in range(30):                                    # 抵近 (带一点俯仰)
        a = (i + 1) / 30.0
        p = start + a * (end - start)
        add(p, yaw_to(p), pitch=0.04 * math.sin(0.5 * i), dot=not (12 <= i < 16))
    right, _ = target_axes()
    for i in range(10):                                    # 横移 → 环部分出画
        p = end + (0.12 * (i + 1)) * right
        add(p, yaw_to(end) + 0.02 * i)
    for i in range(6):                                     # 丢环 (不渲染)
        add(end + 1.2 * right, yaw_to(end) + 0.2, visible=False)
    for i in range(10):                                    # 回中
        p = end + (1.2 * (1 - (i + 1) / 10.0)) * right
        add(p, yaw_to(p))
    return out


def _texture(h, w, seed):
    rng = np.random.default_rng(seed)
    small = rng.integers(70, 150, size=(h // 24 + 2, w // 24 + 2, 3)).astype(np.uint8)
    big = cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC)
    return cv2.GaussianBlur(big, (0, 0), 6)


def render_pair(rig, T_WC, dot=True, visible=True, seed=0):
    """校正后的左右目 (BGR, rig.size)。"""
    W, H = rig.size
    f, cx, cy, B = rig.f, rig.cx, rig.cy, rig.B
    R_CW = T_WC[:3, :3].T
    t_CW = -R_CW @ T_WC[:3, 3]
    R1 = rig.R1
    c_r = R1 @ (R_CW @ C + t_CW)
    n_r = R1 @ (R_CW @ N)
    right_w, up_w = target_axes()
    rt_r, up_r = R1 @ (R_CW @ right_w), R1 @ (R_CW @ up_w)
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    dirs = np.stack([(uu - cx) / f, (vv - cy) / f, np.ones_like(uu)], -1)
    imgs = []
    for eye, ox in (("L", 0.0), ("R", B)):
        o = np.array([ox, 0.0, 0.0])
        bg = _texture(H, W, seed * 2 + (eye == "R"))
        img = bg.astype(np.float32)
        if visible:
            den = dirs @ n_r
            with np.errstate(divide="ignore", invalid="ignore"):
                tt = ((c_r - o) @ n_r) / den
            hit = np.isfinite(tt) & (tt > 0)
            q = o + tt[..., None] * dirs - c_r
            h = q @ rt_r
            v = q @ up_r
            r = np.hypot(h, v)
            paper = hit & (r < R_IN)
            band = hit & (r >= R_IN) & (r <= R_OUT)
            img[paper] = (222, 226, 228)
            if dot:
                dm = paper & (np.hypot(h - DOT_OFF[0], v - DOT_OFF[1]) < DOT_R)
                img[dm] = (40, 38, 205)
            chk = ((np.floor(h / SQ) + np.floor(v / SQ)) % 2) == 0
            white = np.array([232, 232, 232], np.float32)
            red = np.array([35, 35, 200], np.float32)
            blue = np.array([200, 80, 30], np.float32)
            lm = band & (h < -0.25)
            rm = band & (h > 0.25)
            gm = band & ~lm & ~rm
            img[lm & chk] = red
            img[lm & ~chk] = white
            img[rm & chk] = blue
            img[rm & ~chk] = white
            a = np.clip((h[gm] + 0.25) / 0.5, 0, 1)[:, None]
            img[gm] = (1 - a) * red + a * blue
        img = cv2.GaussianBlur(img, (0, 0), 0.7)
        rng = np.random.default_rng(seed * 7 + (eye == "R"))
        img = img + rng.normal(0, 2.5, img.shape)
        img = np.clip(img, 0, 255).astype(np.uint8)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        imgs.append(cv2.imdecode(buf, cv2.IMREAD_COLOR))
    return imgs[0], imgs[1]


def sequence(rig):
    """→ [(t, T_WC, p, q, flags)]"""
    T_BC = make_T_BC()
    out = []
    for t, p, q, fl in trajectory():
        T_WB = np.eye(4)
        T_WB[:3, :3] = R_from_quat(q)
        T_WB[:3, 3] = p
        out.append((t, T_WB @ T_BC, p, q, fl))
    return out
