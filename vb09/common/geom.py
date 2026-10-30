# -*- coding: utf-8 -*-
"""
vb09/common/geom.py —— 纯 Python 小几何 (不依赖 numpy / ROS), 飞机 + 地面 + 网页后端共用
===============================================================================
2026-10-30 从 gate_plan.py / flight_monitor.py / stage12_gate.py / gcs_gate.py / vision_bridge.py
里的重复实现合并而来。**公式逐字搬过来, 运算顺序不变** —— 等价验证 (tools/baseline) 按 float repr 比。

坐标约定 (INTERFACE_gate.md §0, monitor.html gateAxes / toPlane 同一套; tools/geom_check.py 对拍)
-------------------------------------------------------------------------------
  world  = VIO odom (ENU, 米)
  n      = 环平面单位法向, **指向进近侧** (飞机过来的那一边)
  靶面   = 站在进近侧看向环: d = −n, right = normalize(d × z), up = right × d
           h = (p − c)·right, v = (p − c)·up
  along  = (p − c)·n          离环越近越小, 穿过环平面 = 0, 过环后为负

四元数 (x, y, z, w), mavros local pose: ENU 世界系 + FLU 机体系。
"""
import math


# =============================================================================
# 3 维向量 (元组进元组出; 也接受 list)
# =============================================================================
def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vmul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def vnorm(a):
    return math.sqrt(vdot(a, a))


def vunit(a):
    n = vnorm(a)
    return vmul(a, 1.0 / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def hunit(a):
    """水平分量单位化 (z 置 0)。太短返回 None。"""
    h = (a[0], a[1], 0.0)
    n = vnorm(h)
    return vmul(h, 1.0 / n) if n > 1e-9 else None


def ang_deg(a, b):
    c = max(-1.0, min(1.0, vdot(vunit(a), vunit(b))))
    return math.degrees(math.acos(c))


def yaw_of(d):
    return math.atan2(d[1], d[0])


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =============================================================================
# 靶面坐标
# =============================================================================
def gate_axes(n):
    """靶面坐标轴 (right, up)。n 指向进近侧; 从进近侧看向环, right = 右, up = 上。"""
    d = vmul(n, -1.0)
    right = vunit(vcross(d, (0.0, 0.0, 1.0)))
    return right, vcross(right, d)


def to_plane(p, c, n):
    """→ (h, v, along)。见文件头。"""
    r, u = gate_axes(n)
    q = vsub(p, c)
    return vdot(q, r), vdot(q, u), vdot(q, n)


def entry_axes(d):
    """进入方向 d (朝环飞为正) → (right, up, d)。和 gate_axes(n = −d) 同一套靶面轴。"""
    right, up = gate_axes(vmul(d, -1.0))
    return right, up, d


def pct_range(xs, lo=0.1, hi=0.9):
    """p90 − p10 (比 max−min 抗单个野值)。"""
    s = sorted(xs)
    if len(s) < 2:
        return 0.0
    i0 = int(round(lo * (len(s) - 1)))
    i1 = int(round(hi * (len(s) - 1)))
    return s[i1] - s[i0]


def sigma_hva(d, cov=None, sd_cm=None):
    """
    环心位置 σ 投到靶面: → (σ_h, σ_v, σ_along) cm (首飞问题 §1)。
      cov    9 个数 (m², 行主序 3×3, world) —— /gate/status 的 cov
      sd_cm  3 个数 (cm, world xyz) —— 没有 cov 时当对角协方差用
    都没有 → None
    """
    if cov is not None and len(cov) >= 9:
        C = [[float(cov[3 * i + j]) * 1e4 for j in range(3)] for i in range(3)]
    elif sd_cm is not None and len(sd_cm) >= 3:
        C = [[float(sd_cm[i]) ** 2 if i == j else 0.0 for j in range(3)] for i in range(3)]
    else:
        return None
    out = []
    for a in entry_axes(d):
        q = sum(a[i] * C[i][j] * a[j] for i in range(3) for j in range(3))
        out.append(math.sqrt(max(q, 0.0)))
    return tuple(out)


def spread_hva(points, d):
    """一组世界点按靶面轴的 p90−p10 → (h, v, along) cm。"""
    if len(points) < 2:
        return (0.0, 0.0, 0.0)
    return tuple(pct_range([vdot(q, a) for q in points]) * 100.0 for a in entry_axes(d))


# =============================================================================
# 四元数 (x, y, z, w)
# =============================================================================
def yaw_from_quat(x, y, z, w):
    """不归一化 (atan2 对整体缩放不敏感)。"""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def rpy_from_quat(x, y, z, w, eps=1e-9):
    """→ (roll, pitch, yaw) rad。pitch 是 FLU 原始符号 (正 = 机头向下)。模长 < eps → (0, 0, 0)。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < eps:
        return 0.0, 0.0, 0.0
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def tilt_from_quat(x, y, z, w, eps=1e-9):
    """合成倾角 rad (机体 z 轴和竖直的夹角)。模长 < eps → 0。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < eps:
        return 0.0
    x, y = x / n, y / n
    c = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.acos(c)


def quat_to_R(x, y, z, w):
    """→ 3×3 旋转矩阵 (嵌套 list, 行主序)。先归一化。"""
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def quat_from_euler(roll, pitch, yaw):
    """ZYX 欧拉角 (rad, FLU 符号) → (qx, qy, qz, qw)。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)
