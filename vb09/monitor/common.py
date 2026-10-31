# -*- coding: utf-8 -*-
"""
vb09/monitor/common.py —— 常量、飞行脚本发现 (stageNN_<任务>.py)、clean / att_from_quat 小工具
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py)。
"""

import math
import os
import re
import socket
import sys
from vb09.common import geom


# 程序根目录 (= flight_monitor.py 所在目录; 飞行脚本 / monitor.html / 档案都按它找)
HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
try:
    import flight_profile as fp
except ImportError:
    fp = None

SCRIPT_NODE = "/flight_ctrl"           # 飞行脚本的节点名 (stageNN_*.py 的 NODE_NAME)
STATUS_TOPIC = SCRIPT_NODE + "/status"  # vision_bridge 应当订阅它 (转给地面站 HUD)
SETPOINT_PREFIX = "/mavros/setpoint_"  # 任何人往这些话题发数据都要在起飞前发现
HOSTNAME = socket.gethostname()        # 页头显示: 同时开两条隧道时一眼看出是哪架飞机

# ---- /api/stream: 遥测和环状态的环形缓冲 (按接收时刻保留)
TEL_BUF_N = 3000                       # 20Hz × 150s
GATE_BUF_N = 1500                      # 30Hz × 50s
STREAM_INIT_TEL_S = 60.0               # 网页第一次拉 (since=0) 给多久的历史
STREAM_INIT_GATE_S = 15.0
STREAM_MAX_TEL = 1500                  # 一次最多回多少条 (标签页在后台被节流后一次性补拉)
STREAM_MAX_GATE = 600

# ---- PX4 机动参数: 飞控已连接、未解锁时从 mavros 参数缓存里读 (不读 = 不占链路)
PX4_PARAMS = ("MPC_XY_VEL_MAX", "MPC_XY_CRUISE", "MPC_ACC_HOR", "MPC_ACC_HOR_MAX",
              "MPC_JERK_AUTO", "MPC_TILTMAX_AIR", "MPC_XY_P", "MPC_XY_VEL_P_ACC",
              "MPC_Z_VEL_MAX_UP", "COM_OBL_ACT")

# ---- 飞行日志 (网页「回放」, 只读)
LOG_DIR_RE = re.compile(r"^flight_[\w\-]+$")
LOG_FILE_RE = re.compile(r"^[\w\-.]+\.(csv|txt|log)$")
LOG_MAX_BYTES = 40 * 1024 * 1024
TASK_LABEL = {"gate": "穿环"}


def _abs_node(n):
    n = str(n or "").strip()
    return n if n.startswith("/") else "/" + n


# 2026-10-30: 入口改名 stage14_mission.py (任务模式在档案里选); 老的 stageNN_gate.py 也认 (编号大的优先)
SCRIPT_RE = re.compile(r"^stage(\d+)_(gate|mission)\.py$")
SUFFIX_TASK = {"gate": "gate", "mission": "gate"}
KNOB_TASK = {"穿环": "gate"}


def discover_scripts(d=HERE):
    """
    ★2026-10-19 同目录的飞行脚本, 按任务各取编号最大的一个 → {task: 绝对路径}。
    文件名 stage<版本号>_<任务>.py (SCRIPT_RE)。
    """
    best = {}
    for f in os.listdir(d):
        m = SCRIPT_RE.match(f)
        if not m:
            continue
        task = SUFFIX_TASK[m.group(2)]
        key = (int(m.group(1)),)
        if task not in best or key > best[task][0]:
            best[task] = (key, os.path.join(d, f))
    return {t: v[1] for t, v in best.items()}


def task_of_script(path):
    """按文件名后缀判断任务 (软链接看指向的文件名)。认不出 → None。"""
    for p in (path, os.path.realpath(path)):
        m = SCRIPT_RE.match(os.path.basename(p))
        if m:
            return SUFFIX_TASK[m.group(2)]
    return None


def script_for(args, knobs=None):
    """档案里写了任务就用那个任务的脚本, 否则用默认任务 args.task。→ (task, 路径或 None)"""
    task = KNOB_TASK.get((knobs or {}).get("task")) or args.task
    return task, args.scripts.get(task)


def _no_script_msg(args, task):
    return ("没有「%s」任务的飞行脚本 —— 同目录放 stageNN_%s.py (现有: %s)"
            % (TASK_LABEL.get(task, task), task,
               ", ".join(os.path.basename(v) for v in args.scripts.values()) or "无"))


# =============================================================================
# 小工具
# =============================================================================
def clean(x):
    """递归把 nan/inf 变成 None —— JSON 不认 nan, 浏览器会解析失败。"""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    return x


def att_from_quat(qx, qy, qz, qw):
    """
    返回 (roll°, pitch_up°, yaw rad, tilt°)。
    mavros local pose: ENU 世界系 + FLU 机体系 (前/左/上)。
      roll  正 = 右侧向下 (和航空习惯同号, 不用改)
      pitch 四元数解出来正 = 机头向下 → 取负, pitch_up 正 = 机头上仰 (航空习惯)
      yaw   相对 VIO 世界系 (VINS 初始化时的机头方向), 不是罗盘航向
      tilt  合成倾角 (飞行脚本判停桨用的就是它)
    """
    roll, pitch, yaw = geom.rpy_from_quat(qx, qy, qz, qw, eps=1e-9)
    return math.degrees(roll), -math.degrees(pitch), yaw, math.degrees(geom.tilt_from_quat(qx, qy, qz, qw, eps=1e-9))
