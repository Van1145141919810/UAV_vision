# -*- coding: utf-8 -*-
"""
split_stage12.py —— 一次性: 把 stage12_gate.py 的平台层按顶层定义切到 vb09/flight/*.py (第 4.2 步)
  python3 tools/dev/split_stage12.py            (在仓库根目录跑; 改写 vb09/flight/ 和 stage12_gate.py)
做法:
  · 每个顶层节点 (连同它前面的注释) 原样搬到目标模块
  · 共享可变状态 (STATE) 收进 FlightContext 单例 F (vb09/flight/ctx.py), 所有引用改成 F.名字
    —— 只有 init_log_dir / preflight 会重新绑定 (global), 现在都是给 F 的属性赋值, 各模块看到同一份
  · 跨模块用到的函数 / 常量: 按实际用到的名字自动生成 `from vb09.flight.xxx import ...`
  · 标准库 / ROS 导入: 按用到的名字从 HEADER_IMPORTS 里挑
保留在入口 (stage12_gate.py) 的: 档案 setup / derive / finalize、终端检查、main 调用。
"""
import ast
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qualify import qualify  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "stage12_gate.py")

# ---- FlightContext 里的状态 (旧全局名 → F 的属性名)
STATE = """tele setpoint abort_flag land_flag abort_reason land_reason exc_reason in_air voluntary_land
start_mode fcu_msgs fcu_msgs_lock takeover phase_ref sub_mode_ref envelope_dyn dash_tilt_on telem_lock
telem_data INITIAL_X INITIAL_Y INITIAL_Z INITIAL_YAW log_dir csv_main csv_vins csv_ekf csv_timing csv_gate
writer_main writer_vins writer_ekf writer_timing writer_gate zdrop_log_path events_fh gate_in""".split()
RENAME = {k: k for k in STATE}
RENAME.update({"_SITE_REPORT": "site_report", "_fp": "fp"})

MODULES = {
    "ctx": "all_params cells_from_voltage SEV_FATAL SEV_LAND _MAV_RESULT statustext_cb fcu_texts_since "
           "allowed_modes tilt_limit telem_put telem_merge _f request_abort request_land stop_requested "
           "check_abort set_phase get_envelope",
    "telemetry": "Telemetry yaw_from_quat wrap_pi clamp rpy_deg_from_quat tilt_deg_from_quat",
    "setpoint": "SetpointState",
    "keys": "KeyReader _flush_stdin",
    "log": "GATE_CSV_COLS _next_flight_seq _make_flight_dir init_log_dir fsync_all_logs fsync_loop "
           "log_main_row log_vins_row log_ekf_row drain_gate_queue gate_logger_loop drain_timing_queue "
           "dump_zdrop_snapshot check_z_drop main_logger_loop vins_logger_loop ekf_logger_loop "
           "timing_logger_loop publisher_loop status_loop build_telemetry telemetry_loop publish_config "
           "csv_lock_main csv_lock_vins csv_lock_ekf csv_lock_timing csv_lock_gate zdrop_lock "
           "last_zdrop_trigger last_fsync last_vins_stamp timing_queue timing_queue_lock gate_queue "
           "z_history z_history_lock",
    "safety": "Debounce safety_monitor",
    "preflight": "gp_r_lim print_config_summary PX4_CHECK_PARAMS PX4_LOG_PARAMS PX4_PULL_TIMEOUT _fmt_px4 "
                 "_param_value read_px4_params preflight_plan preflight",
    "target_in": "GateInput gate_in",
    "mission": "GateStop GateBack _l3 GateRun phase_gate",
    "phases": "force_disarm_with_timeout phase_takeoff phase_descend auto_land_timeout phase_auto_land "
              "phase_handover on_ground cleanup_decision restore_ground_mode arm_with_confirm "
              "single_instance_guard emergency_land pose_cb",
    "runner": "main",
}
OWNER = {n: m for m, ns in MODULES.items() for n in ns.split()}

DOCS = {
    "ctx": "飞行上下文: 共享可变状态单例 F (FlightContext) + 停止 / 降落请求、阶段与包络、遥测分段等小函数",
    "telemetry": "mavros / VINS 遥测汇聚 (Telemetry) + 四元数小函数",
    "setpoint": "setpoint 状态 (速度 / 定高 / 轨迹三种模式) 和 PositionTarget 组包",
    "keys": "终端键盘 (cbreak, 读空缓冲, EOF = SSH 断连)",
    "log": "飞行日志目录、各 csv 记录线程、z 掉高检测、fsync; 遥测 / 状态 / 配置话题的发布",
    "safety": "安全监控线程 (FATAL 停桨 / LAND 缓降判据, 去抖)",
    "preflight": "起飞前检查 (MAVROS / 位姿 / VINS / 视觉 / 静止漂移 / PX4 参数) + 参数摘要 + 地面预规划",
    "target_in": "/gate/status、/vision_bridge/cam 的接收 + 单帧判定窗口 (GateInput)",
    "mission": "穿环任务: 一次尝试的状态机 (GateRun) 和 phase_gate (第 4.4 步换成 Step + run_mission)",
    "phases": "起飞 / 下降 / AUTO.LAND / 接管 / cleanup 判定 / 解锁 / 单实例 / 紧急降落 / 位姿回调",
    "runner": "飞行主序列 main(): 订阅 / 发布 / 服务 → 预检 → prime → OFFBOARD → 解锁 → 起飞 → 任务 → 降落 → cleanup",
}

HEADER_IMPORTS = [
    ("os", "import os"), ("sys", "import sys"), ("csv", "import csv"), ("json", "import json"),
    ("logging", "import logging"), ("math", "import math"), ("re", "import re"),
    ("signal", "import signal"), ("statistics", "import statistics"), ("threading", "import threading"),
    ("time", "import time"), ("traceback", "import traceback"),
    ("deque", "from collections import deque"), ("datetime", "from datetime import datetime"),
    ("np", "import numpy as np"), ("rospy", "import rospy"),
    ("PoseStamped", "from geometry_msgs.msg import PoseStamped"),
    ("TwistStamped", "from geometry_msgs.msg import TwistStamped"),
    ("ActuatorControl", "from mavros_msgs.msg import ActuatorControl"),
    ("PositionTarget", "from mavros_msgs.msg import PositionTarget"),
    ("State", "from mavros_msgs.msg import State"),
    ("ExtendedState", "from mavros_msgs.msg import ExtendedState"),
    ("EstimatorStatus", "from mavros_msgs.msg import EstimatorStatus"),
    ("StatusText", "from mavros_msgs.msg import StatusText"),
    ("CommandBool", "from mavros_msgs.srv import CommandBool"),
    ("CommandLong", "from mavros_msgs.srv import CommandLong"),
    ("ParamGet", "from mavros_msgs.srv import ParamGet"),
    ("SetMode", "from mavros_msgs.srv import SetMode"),
    ("ParamPull", "try:\n    from mavros_msgs.srv import ParamPull\nexcept ImportError:                       # 很老的 mavros_msgs\n    ParamPull = None"),
    ("Odometry", "from nav_msgs.msg import Odometry"),
    ("BatteryState", "from sensor_msgs.msg import BatteryState"),
    ("String", "from std_msgs.msg import String"),
    ("gp", "import gate_plan as gp      # 同目录, 纯函数"),
    ("geom", "from vb09.common import geom"),
    ("C", "from vb09.flight import config as C"),
]
STD_NAMES = {k for k, _ in HEADER_IMPORTS}


def main():
    src = open(SRC, encoding="utf-8").read()
    lines = src.splitlines(keepends=True)
    t = ast.parse(src)
    body = t.body
    # 第一个要搬的节点: all_params
    i0 = next(i for i, n in enumerate(body) if isinstance(n, ast.FunctionDef) and n.name == "all_params")
    i_main = next(i for i, n in enumerate(body) if isinstance(n, ast.If) and "__main__" in ast.unparse(n.test)
                  and i > i0)
    # 每个节点的源码 = 上一个节点结束到本节点结束 (含前面的注释)
    chunks = []
    prev_end = body[i0 - 1].end_lineno
    # i0 前面紧挨的注释 (所有搬走的从 i0 开始)
    for n in body[i0:i_main]:
        text = "".join(lines[prev_end:n.end_lineno])
        prev_end = n.end_lineno
        if isinstance(n, (ast.FunctionDef, ast.ClassDef)):
            names = [n.name]
        elif isinstance(n, ast.Assign):
            names = [x.id for tg in n.targets for x in ast.walk(tg) if isinstance(x, ast.Name)]
        else:
            raise SystemExit("unexpected node at %d: %s" % (n.lineno, ast.dump(n)[:80]))
        chunks.append((names, text, n))
    entry_head = "".join(lines[:body[i0 - 1].end_lineno])
    entry_tail = "".join(lines[body[i_main - 1].end_lineno:])

    per = {m: [] for m in MODULES}
    state_init = []
    for names, text, n in chunks:
        if all(x in STATE for x in names):
            state_init.append((names, text, n))
            continue
        owners = {OWNER.get(x) for x in names}
        if len(owners) != 1 or None in owners:
            raise SystemExit("no owner for %s" % names)
        per[owners.pop()].append((names, text, n))

    # ---- ctx.py: FlightContext.__init__ 收状态
    init_lines = []
    for names, text, n in state_init:
        seg = ast.get_source_segment(src, n)
        cm = [ln for ln in text.splitlines() if ln.strip().startswith("#")]
        trail = ""
        m = re.search(r"(#.*)$", lines[n.end_lineno - 1].rstrip("\n"))
        if m and m.group(1) not in seg:
            trail = "  " + m.group(1)
        val = seg.split("=", len(names))[-1].strip() if isinstance(n, ast.Assign) else seg
        for c in cm:
            if c.strip() not in ("",) and c.strip() not in [x.strip() for x in init_lines]:
                init_lines.append("        " + c.strip())
        tgt = " = ".join("self.%s" % x for x in names)
        init_lines.append("        %s = %s%s" % (tgt, val, trail))
    ctx_class = '''class FlightContext(object):
    """
    飞行脚本的共享可变状态 (原 stage12 的模块级全局)。整个进程只有一个实例 F (下面创建),
    各模块 `from vb09.flight.ctx import F` 后用 F.名字 —— 对象本身不会被重新绑定, 所以不会出现
    "某个模块拿着旧副本"的问题 (PLAN_refactor_v2 §7)。以前用 global 重新赋值的
    (INITIAL_* / log_dir / csv_* / writer_* …) 现在都是给 F 的属性赋值。
    tele / setpoint / gate_in 由入口在导入完成后挂上 (见 bind())。
    """

    def __init__(self):
        self.site_report = None      # flight_profile 的 report (入口 setup/finalize 之后挂上)
        self.fp = None               # flight_profile 模块 (没有 = None)
        self.tele = None             # telemetry.Telemetry()
        self.setpoint = None         # setpoint.SetpointState()
        self.gate_in = None          # target_in.GateInput()
''' + "\n".join(x for x in init_lines if "Telemetry()" not in x and "SetpointState()" not in x
               and "GateInput()" not in x) + '''


F = FlightContext()
'''

    def module_text(m, items, extra_top=""):
        code = "".join(t for _, t, _ in items).strip("\n") + "\n"
        code = extra_top + code
        out, n = qualify(code, RENAME, "F")
        if out is None:
            raise SystemExit("%s: %s" % (m, n))
        tree = ast.parse(out)
        defined = set()
        for nd in tree.body:
            if isinstance(nd, (ast.FunctionDef, ast.ClassDef)):
                defined.add(nd.name)
            elif isinstance(nd, ast.Assign):
                defined |= {x.id for tg in nd.targets for x in ast.walk(tg) if isinstance(x, ast.Name)}
        used = {x.id for x in ast.walk(tree) if isinstance(x, ast.Name)}
        imports = [imp for k, imp in HEADER_IMPORTS if k in used and k not in defined]
        cross = {}
        for u in sorted(used - defined):
            o = OWNER.get(u)
            if o and o != m:
                cross.setdefault(o, []).append(u)
        cl = []
        for o in sorted(cross):
            cl.append("from vb09.flight.%s import %s" % (o, ", ".join(sorted(cross[o]))))
        need_F = "F" in used and m != "ctx"
        head = ['# -*- coding: utf-8 -*-', '"""', 'vb09/flight/%s.py —— %s' % (m, DOCS[m]),
                '=' * 79,
                '2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);',
                '共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。', '"""']
        std = [i for i in imports if not i.startswith(("import rospy", "from geometry", "from mavros", "try:",
                                                       "from nav_msgs", "from sensor_msgs", "from std_msgs",
                                                       "import numpy", "import gate_plan", "from vb09"))]
        third = [i for i in imports if i not in std]
        parts = ["\n".join(head), "\n".join(std), "\n".join(third)]
        local = ([("from vb09.flight.ctx import F")] if need_F else []) + cl
        parts.append("\n".join(local))
        return "\n\n".join(p for p in parts if p) + "\n\n\n" + out, cross

    outputs, graph = {}, {}
    for m, items in per.items():
        extra = ctx_class + "\n\n" if m == "ctx" else ""
        if m == "target_in":
            items = [it for it in items if it[0] != ["gate_in"]]
        txt, cross = module_text(m, items, extra)
        if m == "target_in":
            txt += "\n\nF.gate_in = GateInput()\n"
        outputs[m] = txt
        graph[m] = sorted(cross)
    # 环检查
    def cyc(m, stack):
        for d in graph[m]:
            if d in stack:
                return stack[stack.index(d):] + [d]
            r = cyc(d, stack + [d])
            if r:
                return r
        return None
    for m in graph:
        c = cyc(m, [m])
        if c:
            print("import cycle:", " -> ".join(c))
    for m, txt in outputs.items():
        open(os.path.join(ROOT, "vb09", "flight", m + ".py"), "w", encoding="utf-8").write(txt)
        print("vb09/flight/%s.py  deps %s" % (m, graph[m]))
    open(os.path.join(ROOT, "tools", "dev", "_entry_head.txt"), "w", encoding="utf-8").write(entry_head)
    open(os.path.join(ROOT, "tools", "dev", "_entry_tail.txt"), "w", encoding="utf-8").write(entry_tail)


if __name__ == "__main__":
    main()
