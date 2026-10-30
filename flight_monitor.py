#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_monitor.py —— 飞机上的网页监控站（只读监控 + 起飞配置）
===============================================================================
在飞机上常驻一个小网页服务, 笔记本用 SSH 端口映射来看。不传视频。

  飞机:   nice -n 10 python3 flight_monitor.py      (放 tmux 里, 先起 roscore/mavros)
  笔记本: ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
              -L 8080:127.0.0.1:8080 visbot@192.168.2.20
          (ExitOnForwardFailure: 本地 8080 被占时直接退出 —— 默认 ssh 只打一行警告继续连着,
           浏览器打开的其实是旧隧道, 可能是另一架飞机)
          多台电脑同时看: 每台各开一条, 互不冲突; 同一台电脑看两架飞机: -L 8081:127.0.0.1:8080
          浏览器打开 http://localhost:8080   → 「帮助」页有完整起飞流程
  (VSCode Remote-SSH 会自动转发端口, 在"端口"面板点一下也行)

  离线试页面(不需要 ROS, 笔记本上就能跑):
          python3 flight_monitor.py --fake

  飞行脚本: 文件名 stage<版本号>_<任务>.py, 编号 = 版本, 取编号最大的。
           2026-10-30 起只有穿环 (_gate); 停靠跟踪 (_track / _real) 已删除。
           --script 指定单个脚本时只用它 (同目录 flight.py 软链接同理)。

为什么这样设计
-------------------------------------------------------------------------------
1. **独立进程**。它崩了飞行不受影响; 它只订阅话题, 从不发布任何控制量。
2. **只绑 127.0.0.1**。必须先 SSH 登录飞机才能打开; 同一 WiFi 下别人打不开,
   也就不可能误改档案。
3. **纯标准库** (http.server)。外场没网 pip 装不上; 页面也不加载任何 CDN。
4. **页面上的阈值全部来自脚本发布的 /flight_ctrl/config**, 网页里不写任何阈值。
   脚本没启动前, 用 `<飞行脚本> --dry-run --json`(代码默认值)作参考。
5. **配置页只写档案、只跑 dry-run、只生成启动脚本 (~/fly.sh), 不启动飞行**。
   飞行必须在普通 SSH 终端里启动 (运行 ~/fly.sh 或手打命令):
   脚本依赖 TTY (键盘急停 / stdin EOF 检测), 后台启动会让这些全部失效;
   飞行脚本自己也会拒绝在 tmux / VSCode 终端 / 非 TTY 里起飞。
6. 校验逻辑只有一份 —— 飞机上的飞行脚本 + flight_profile.py。
   网页只把它的 JSON 结果原样显示。

数据来源
-------------------------------------------------------------------------------
  /flight_ctrl/telemetry  脚本内部状态 (阶段/穿环段/安全余量)   5Hz
  /flight_ctrl/config     生效配置 + 派生量 + 原点 (latched)
  /rosout                 脚本的告警/事件 (按节点名过滤; 节点名从遥测里学, 不写死)
  mavros 话题              位姿/速度/电池/模式 (脚本没启动也有)
  VINS 话题               只统计频率和延迟
  ROS master              机况检查: geometric_controller、谁在发设定点、版本是否配套、
                          有没有旧的抓帧节点/旧版桥还在跑
  /vision_bridge/cam      图传桥自己报的新相机状态 (1Hz JSON: 帧率/分辨率/原样转发比例/重开次数)

节点名 /flight_ctrl 是固定的, 不随飞行脚本文件的版本号变。

单实例 / 端口 (2026-10-10)
-------------------------------------------------------------------------------
  **先占 8080, 再碰 ROS**。以前顺序反了: 第二个实例先用同名节点 init_node →
  ROS master 把**旧实例**的节点踢掉 → 新实例再绑 8080 失败退出 → 旧实例的网页还占着
  8080 但 ROS 已死, 页面"已连接"却一直"无数据"。现在第二个实例在绑端口这一步就退出,
  并打印是谁占着; 本实例的 ROS 节点被关掉时 (被同名节点顶掉 / master 退出),
  网页服务也跟着退出, 把端口让出来, 不留僵尸。

姿态 (2026-10-10)
-------------------------------------------------------------------------------
  /api/state 的 pose 里多了 roll / pitch_up (度), 给网页姿态球用。
  pitch_up 已换成航空习惯 (机头上仰为正): mavros local pose 是 ENU/FLU,
  四元数直接解出来的 pitch 是"机头向下为正"。flight_log.csv 和地面站 HUD 里的
  pitch 仍是 FLU 原始符号, 比对时差一个负号, 不是 bug。

穿环 (2026-10-19) —— 字段约定见同目录 INTERFACE_gate.md
-------------------------------------------------------------------------------
  新增数据源   /gate/status (vision_bridge 每帧 JSON, gate / gate_gcs 模式才有)
  新增接口     /api/state 里的 gate / px4 / stream_seq
               /api/stream?since=N   遥测 + 环状态的增量 (不丢点: 冲刺只有 1~2 秒,
                                     网页 5Hz 拉"最新一条"会漏掉大部分)
               /api/logs  /api/log?dir=&file=   只读飞行日志 (网页「回放」页)
  机况检查     任务 / 图传配套、环识别、PX4 机动参数 (MPC_TILTMAX_AIR 等,
               只在飞控已连接且**未解锁**时读 mavros 的参数缓存)
  布局         穿环布局 (2026-10-30 停靠跟踪布局已删除)
  离线试       python3 flight_monitor.py --fake --fake-task gate
               (带一个假的识别画面 /fake/snapshot.jpg, 用来核对画面叠加的投影)
===============================================================================
"""

import argparse
import collections
import errno
import json
import math
import os
import re
import shutil
import socket
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from vb09.common import geom  # noqa: E402
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


SCRIPT_RE = re.compile(r"^stage(\d+)_(gate)\.py$")
SUFFIX_TASK = {"gate": "gate"}
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


class RateMeter:
    """滑动窗口频率计 + 最近一次到达时刻。"""

    def __init__(self, window=2.0):
        self.window = window
        self.ts = collections.deque(maxlen=400)

    def tick(self, t=None):
        self.ts.append(time.time() if t is None else t)

    def age(self, now):
        return (now - self.ts[-1]) if self.ts else None

    def rate(self, now):
        while self.ts and now - self.ts[0] > self.window:
            self.ts.popleft()
        if len(self.ts) < 2 or now - self.ts[-1] > 1.0:
            return 0.0
        span = self.ts[-1] - self.ts[0]
        return (len(self.ts) - 1) / span if span > 1e-6 else 0.0


# =============================================================================
# 共享状态 (ROS 回调 / fake 线程写, HTTP 线程读)
# =============================================================================
class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = "ros"
        self.mavros = {}
        self.t_state = None
        self.pose = None
        self.vel = None
        self.batt_v = None
        self.t_batt = None
        self.landed = None
        self.pose_hist = collections.deque(maxlen=600)       # (t, x, y, z)
        self.meters = {k: RateMeter() for k in
                       ("pose", "vins", "vision_pose", "telemetry", "gate")}
        self.cam = None                    # 图传桥 /vision_bridge/cam 的最新一条 (dict)
        self.t_cam = None
        self.gate = None                   # /gate/status 最新一条 (dict, bridge 原样)
        self.t_gate = None
        # /api/stream: 遥测 + 环状态共用一个递增序号, 网页按序号增量拉, 一条不漏
        self.seq = 0
        self.tel_buf = collections.deque(maxlen=TEL_BUF_N)    # (seq, 接收时刻, telemetry)
        self.gate_buf = collections.deque(maxlen=GATE_BUF_N)  # (seq, 接收时刻, gate_status)
        self.px4 = {"params": {}, "t": None, "t_try": None, "error": None}
        self.telemetry = None
        self.t_telemetry = None
        self.config = None
        self.t_config = None
        self.config_fallback = None
        self.events = collections.deque(maxlen=600)
        self.event_id = 0
        self.graph = {"ok": False, "t": None, "nodes": [], "setpoint_pubs": {},
                      "status_subs": [],
                      "error": None, "master_restarted": False,
                      # 识别相机 (见 run_checks「识别相机」)
                      "bridge_img_subs": [],    # vision_bridge 订的图像话题 (非空 = 旧版桥)
                      "det_cam_pubs": []}       # 谁在发 /det_cam/image_raw (旧抓帧节点 = 抢相机)
        self.script_node = SCRIPT_NODE     # 从遥测/配置的 "node" 字段学到的实际节点名
        self._last_phase = None
        self._script_alive = False
        self._seen_once = set()            # 每次飞行脚本启动时清空

    # ---------------- 写 ----------------
    def set_state(self, connected, armed, mode):
        with self.lock:
            self.mavros = {"connected": bool(connected), "armed": bool(armed),
                           "mode": mode}
            self.t_state = time.time()

    def set_pose(self, x, y, z, qx, qy, qz, qw):
        now = time.time()
        roll, pitch_up, yaw, tilt = att_from_quat(qx, qy, qz, qw)
        with self.lock:
            self.pose = {"x": x, "y": y, "z": z, "yaw": yaw, "tilt": tilt,
                         "roll": roll, "pitch_up": pitch_up, "t": now}
            self.pose_hist.append((now, x, y, z))
            self.meters["pose"].tick(now)

    def set_vel(self, vx, vy, vz):
        with self.lock:
            self.vel = {"x": vx, "y": vy, "z": vz, "t": time.time()}

    def set_batt(self, v):
        with self.lock:
            self.batt_v = v
            self.t_batt = time.time()

    def set_landed(self, s):
        with self.lock:
            self.landed = s

    def tick(self, name):
        with self.lock:
            self.meters[name].tick()

    def set_cam(self, d):
        with self.lock:
            self.cam = d
            self.t_cam = time.time()

    def set_telemetry(self, d):
        ph = d.get("phase")
        with self.lock:
            if d.get("node"):
                self.script_node = _abs_node(d["node"])
            self.telemetry = d
            self.t_telemetry = time.time()
            self.meters["telemetry"].tick()
            self.seq += 1
            self.tel_buf.append((self.seq, self.t_telemetry, d))
            prev = self._last_phase
            self._last_phase = ph
        if ph != prev and ph is not None:
            self.add_event("PHASE", "阶段 → %s%s" % (
                ph, ("（%s）" % d.get("sub_mode")) if d.get("sub_mode") else ""),
                "monitor")
        if d.get("abort") and d.get("abort_reason"):
            self._once("abort:" + str(d["abort_reason"]), "ERROR",
                       "★ FATAL: %s" % d["abort_reason"])
        if d.get("takeover"):
            self._once("takeover:" + str(d["takeover"]), "WARN",
                       "★ 控制权被接管 (飞控模式 %s) —— 脚本不切模式、不停桨" % d["takeover"])
        if d.get("land") and d.get("land_reason"):
            self._once("land:" + str(d["land_reason"]), "WARN",
                       "★ LAND: %s" % d["land_reason"])

    def _once(self, key, level, msg):
        if key in self._seen_once:
            return
        self._seen_once.add(key)
        self.add_event(level, msg, "monitor")

    def set_gate(self, d):
        """/gate/status 一条 (vision_bridge 每帧发)。原样存, 网页自己挑字段。"""
        if not isinstance(d, dict):
            return
        now = time.time()
        with self.lock:
            self.gate = d
            self.t_gate = now
            self.meters["gate"].tick(now)
            self.seq += 1
            self.gate_buf.append((self.seq, now, d))

    def set_px4(self, params, error=None):
        now = time.time()
        with self.lock:
            self.px4["t_try"] = now
            self.px4["error"] = error
            if params:
                self.px4["params"] = dict(params)
                self.px4["t"] = now

    def set_config(self, d):
        with self.lock:
            if d.get("node"):
                self.script_node = _abs_node(d["node"])
            self.config = d
            self.t_config = time.time()

    def add_event(self, level, msg, src="script"):
        with self.lock:
            self.event_id += 1
            self.events.append({"id": self.event_id, "t": time.time(),
                                "level": level, "msg": msg, "src": src})

    def set_graph(self, **kw):
        with self.lock:
            self.graph.update(kw)
            self.graph["t"] = time.time()

    # ---------------- 读 ----------------
    def script_alive(self, now):
        return self.t_telemetry is not None and now - self.t_telemetry < 1.5

    def watch_script(self):
        """脚本启动/退出也记成事件 (由后台线程每秒调一次)。"""
        now = time.time()
        alive = self.script_alive(now)
        if alive != self._script_alive:
            self._script_alive = alive
            if alive:
                self._seen_once.clear()
            self.add_event("PHASE", "飞行脚本已启动" if alive else
                           "飞行脚本已停止输出遥测（退出或卡死）", "monitor")

    def snapshot(self):
        now = time.time()
        with self.lock:
            m = self.meters
            snap = {
                "now": now,
                "mode": self.mode,
                "host": HOSTNAME,
                "mavros": dict(self.mavros,
                               state_age=(now - self.t_state) if self.t_state else None),
                "pose": dict(self.pose, age=now - self.pose["t"]) if self.pose else None,
                "vel": dict(self.vel, age=now - self.vel["t"]) if self.vel else None,
                "batt_v": self.batt_v,
                "batt_age": (now - self.t_batt) if self.t_batt else None,
                "landed": self.landed,
                "rates": {k: round(v.rate(now), 1) for k, v in m.items()},
                "ages": {k: v.age(now) for k, v in m.items()},
                "telemetry": self.telemetry,
                "telemetry_age": (now - self.t_telemetry) if self.t_telemetry else None,
                "script_alive": self.script_alive(now),
                "config_t": (self.config or self.config_fallback or {}).get("t"),
                "config_source": "live" if self.config else
                                 ("default" if self.config_fallback else None),
                "event_last_id": self.event_id,
                "script_node": self.script_node,
                "cam": dict(self.cam, age=now - self.t_cam) if self.cam else None,
                # 穿环 (2026-10-19)。gate.age = 本程序收到它到现在的秒数;
                # gate.age_ms (bridge 原有字段) = 估计本身的年龄
                "gate": dict(self.gate, age=now - self.t_gate) if self.gate else None,
                "px4": {"params": self.px4["params"], "error": self.px4["error"],
                        "age": (now - self.px4["t"]) if self.px4["t"] else None},
                "stream_seq": self.seq,
            }
        return clean(snap)

    def stream(self, since):
        """
        /api/stream: 序号 > since 的遥测和环状态。since <= 0 (网页刚打开) → 最近一段历史。
        网页靠它画冲刺段曲线: 冲刺只有 1~2 秒, /api/state 只给"最新一条", 5Hz 拉会丢大部分点。
        """
        now = time.time()
        with self.lock:
            if since > 0:
                tel = [e for e in self.tel_buf if e[0] > since]
                gate = [e for e in self.gate_buf if e[0] > since]
            else:
                tel = [e for e in self.tel_buf if now - e[1] <= STREAM_INIT_TEL_S]
                gate = [e for e in self.gate_buf if now - e[1] <= STREAM_INIT_GATE_S]
            seq = self.seq
        return clean({"seq": seq, "now": now,
                      "tel": [{"s": s, "tm": t, "d": d} for s, t, d in tel[-STREAM_MAX_TEL:]],
                      "gate": [{"s": s, "tm": t, "d": d} for s, t, d in gate[-STREAM_MAX_GATE:]]})

    def get_config(self):
        with self.lock:
            if self.config:
                return {"source": "live", "age": time.time() - self.t_config,
                        "config": self.config}
            if self.config_fallback:
                return {"source": "default", "age": None,
                        "config": self.config_fallback}
            return {"source": None, "config": None}

    def get_events(self, since):
        with self.lock:
            return [e for e in self.events if e["id"] > since]

    def static_drift(self, window=3.0):
        now = time.time()
        with self.lock:
            pts = [p for p in self.pose_hist if now - p[0] <= window]
        if len(pts) < 10:
            return None
        xs = [p[1] for p in pts]
        ys = [p[2] for p in pts]
        zs = [p[3] for p in pts]
        return {"xy_std": math.hypot(statistics.pstdev(xs), statistics.pstdev(ys)),
                "z_std": statistics.pstdev(zs), "n": len(pts)}


STORE = Store()


# =============================================================================
# 机况检查
# =============================================================================
def current_task():
    """
    当前任务: 脚本在跑 → 它遥测里的 task; 否则 → 配置里的 task; 都没有 = 穿环。
    """
    now = time.time()
    with STORE.lock:
        t = STORE.telemetry if STORE.script_alive(now) else None
        c = STORE.config or STORE.config_fallback or {}
    return (t or {}).get("task") or c.get("task") or "gate"


def _bridge_mode(s):
    cam = s.get("cam")
    if not cam or cam.get("age") is None or cam["age"] > 5.0:
        return None
    return (cam.get("cfg") or {}).get("mode") or "aruco"


def gate_checks(add, s):
    """「任务 / 图传配套」+「环识别 /gate/status」。"""
    mode = _bridge_mode(s)
    gate_mode = mode in ("gate", "gate_gcs")
    task = current_task()
    tl = TASK_LABEL.get(task, task)

    if mode is None:
        add("任务 / 图传配套", "info", "任务 %s · 图传没在跑" % tl)
    elif task == "gate" and not gate_mode:
        add("任务 / 图传配套", "fail", "任务 = 穿环, 图传 = %s" % mode,
            "穿环要 gate_gcs (地面检测) 或 gate (机上检测) —— 重起: python3 run_vision.py gate_gcs")
    else:
        add("任务 / 图传配套", "ok", "任务 %s · 图传 %s" % (tl, mode))

    gs = s.get("gate")
    if not gate_mode and not gs:
        return
    r = (s.get("rates") or {}).get("gate") or 0.0
    if not gs or gs.get("age") is None or gs["age"] > 2.0:
        hint = {"gate_gcs": "地面 gcs_gate.py 在跑吗? (--mode 0 不检测, 也就没有 /gate/status)",
                "gate": "机上检测线程出错? 看 /tmp/bridge.log"}.get(mode, "")
        add("环识别 /gate/status", "fail" if gate_mode else "warn",
            "无数据" + (" (%.0fs 前)" % gs["age"] if gs and gs.get("age") else ""), hint)
        return
    parts = ["%.1f Hz" % r, "阶段 %s" % gs.get("stage")]
    if gs.get("dist") is not None:
        parts.append("距离 %.2f m" % gs["dist"])
    sd = gs.get("sd_cm")
    if isinstance(sd, list) and sd and all(isinstance(v, (int, float)) for v in sd):
        parts.append("σ %.1f cm" % max(sd))
    st, notes = "ok", []
    if gs.get("pose") not in (None, "ok"):
        st = "warn"
        notes.append("位姿插值失败: %s" % gs.get("pose"))
    if gs.get("sim", True):
        notes.append("发布为 odom_sim: 不会驱动飞机 (gate_gcs 下地面 --mode 2 才驱动)")
    if gs.get("n") is None:
        notes.append("没有法向 n —— 俯视图/靶面视图只能画环中心点 (bridge 要按 INTERFACE_gate.md 补 n)")
    if gs.get("aim") is None:
        notes.append("没有红点位置 aim —— 瞄准点按环中心画")
    add("环识别 /gate/status", st, "  ·  ".join(parts), "; ".join(notes))


def _fmt_param(v):
    if isinstance(v, float):
        return ("%.2f" % v).rstrip("0").rstrip(".")
    return str(v)


def px4_check(add, s):
    """
    「PX4 机动参数」。关键是 MPC_TILTMAX_AIR: PX4 允许的最大倾角比脚本的停桨线还大时,
    急刹 / 冲刺这种合法机动就可能被脚本当成"姿态失控"而空中停桨。
    """
    px = s.get("px4") or {}
    P = px.get("params") or {}
    if not P:
        add("PX4 机动参数", "info", "未读到",
            px.get("error") or "飞控连上且未解锁时自动读 (mavros 参数要先拉完, 开机后约半分钟)")
        return
    cfg = STORE.get_config().get("config") or {}
    prm = cfg.get("params") or {}
    gate = (cfg.get("derived") or {}).get("gate") or {}
    tilt, confirm = prm.get("MAX_TILT_DEG"), prm.get("FATAL_CONFIRM_S")
    val = "  ".join("%s=%s" % (k, _fmt_param(P[k])) for k in PX4_PARAMS if k in P)
    st, notes = "ok", []
    ta = P.get("MPC_TILTMAX_AIR")
    if isinstance(ta, (int, float)) and isinstance(tilt, (int, float)) and ta > tilt - 3:
        st = "warn"
        notes.append("PX4 允许倾到 %.0f°, 脚本倾角 > %.0f° 持续 %ss 就**空中停桨** —— 急刹/冲刺时 "
                     "PX4 自己就可能把飞机倾过停桨线。冲刺前把 MPC_TILTMAX_AIR 调到 ≤ %.0f°"
                     % (ta, tilt, _fmt_param(confirm) if confirm else "?", tilt - 5))
    vmax, dash = P.get("MPC_XY_VEL_MAX"), gate.get("dash_v")
    if isinstance(vmax, (int, float)) and isinstance(dash, (int, float)) and vmax < dash * 1.1:
        st = "warn"
        notes.append("MPC_XY_VEL_MAX=%s < 冲刺速度 %s × 1.1: PX4 会把冲刺速度削掉"
                     % (_fmt_param(vmax), _fmt_param(dash)))
    if px.get("error"):
        notes.append(px["error"])
    if px.get("age") and px["age"] > 120:
        notes.append("%.0f 分钟前读的 (解锁后不再读)" % (px["age"] / 60))
    add("PX4 机动参数", st, val, "; ".join(notes))


def camera_check(add, g, s, bridges, args):
    """
    「识别相机」: 新相机直连版图传桥 (同目录 vision_bridge.py, 用 run_vision.py 起) 好不好。

    数据来自桥自己发的 /vision_bridge/cam (1Hz JSON)。另外从 ROS 图里抓两种冲突:
      · 旧抓帧节点 det_cam_node 还在发 /det_cam/image_raw → 会和桥抢同一个相机
      · 桥订阅了图像话题 → 跑的是**旧版桥** (新版不订任何图像话题)
    """
    cam = s.get("cam")
    subs = g.get("bridge_img_subs") or []
    legacy = list(g.get("det_cam_pubs") or []) + [
        n for n in g["nodes"] if n.startswith("/det_cam_watchdog")]
    cfg = (STORE.get_config().get("config") or {}).get("params") or {}
    f_det = cfg.get("F_DETECT")
    run = "cd %s && python3 run_vision.py %s --gcs <地面站IP>" % (
        _tilde(HERE), "gate_gcs" if current_task() == "gate" else "aruco")

    if not bridges:
        add("识别相机", "info" if not legacy else "warn", "图传桥没在跑",
            run + ("  (它会先停掉旧的 %s)" % ", ".join(legacy) if legacy else ""))
        return
    if subs:
        add("识别相机", "warn", "跑的是旧版桥 (订 %s)" % ", ".join(subs),
            "新版桥自己开新相机、不订图像话题。Ctrl+C 掉它, 换成: " + run)
        return
    if not cam or cam.get("age") is None or cam["age"] > 3.0:
        add("识别相机", "fail", "桥在跑, 但没收到 /vision_bridge/cam",
            "桥卡死了? 看 tmux 里桥的窗口 / /tmp/bridge.log")
        return

    notes, st = [], "ok"
    if legacy:
        st = "fail"
        notes.append("旧的 %s 还在跑, 会和桥抢相机 —— 重新 python3 run_vision.py <模式> (会先停它们)"
                     % ", ".join(legacy))
    if not cam.get("ok"):
        add("识别相机", "fail", "相机不出图: %s" % (cam.get("err") or "?"),
            "; ".join(notes + ["桥会每秒自动重开; 已重开 %s 次" % cam.get("reopens", 0)]))
        return
    w, h = cam.get("w") or 0, cam.get("h") or 0
    fps, tx = cam.get("cam_fps") or 0.0, cam.get("tx_hz") or 0.0
    mode = (cam.get("cfg") or {}).get("mode") or "aruco"
    gate_mode = mode in ("gate", "gate_gcs")
    exp_wh = (3840, 1080) if gate_mode else (1920, 1080)
    if (w, h) != exp_wh:
        st = "warn" if st == "ok" else st
        notes.append("分辨率 %dx%d ≠ %dx%d (%s 模式) —— %s" % (
            w, h, exp_wh[0], exp_wh[1], mode,
            "双目标定对不上" if gate_mode else "gcs_aruco_det2 的内参对不上"))
    g = cam.get("gate") or {}
    det_hz = (g.get("hz") or 0.0) if gate_mode else tx       # gate 模式看检测输出频率, 不看上行
    if f_det and abs(det_hz - float(f_det)) > max(1.5, 0.2 * float(f_det)):
        st = "warn" if st == "ok" else st
        notes.append("%s %.1f Hz ≠ 飞行脚本 F_DETECT=%g (参与年龄降级)"
                     % ("检测输出" if gate_mode else "上行", det_hz, f_det))
    if gate_mode:
        if g.get("pose") not in (None, "ok"):
            st = "warn" if st == "ok" else st
            notes.append("融合拿不到位姿: %s" % g.get("pose"))
        notes.append("gate%s: %.1f Hz %s ms · 阶段 %s · 距离 %s · 来源 %s" % (
            "(地面检测)" if mode == "gate_gcs" else "(机上检测)", g.get("hz") or 0.0, g.get("ms"),
            g.get("stage"), g.get("dist"), g.get("src")))
    if cam.get("reopens"):
        st = "warn" if st == "ok" else st
        notes.append("相机已自动重开 %d 次 (USB 接触?) 原因: 见 /tmp/bridge.log" % cam["reopens"])
    if cam.get("raw_supported") is False:
        notes.append("驱动不给原始 MJPG, 每帧在飞机上重编码 (费 CPU)")
    if (cam.get("drop") or 0) > 0:
        notes.append("近 1 秒上行丢 %d 帧 (链路跟不上, 可减 _raw_max_kb 让它重编码成小图)" % cam["drop"])
    raw = cam.get("raw_pct")
    val = "新相机 %dx%d  采集 %.1f fps → 上行 %.1f Hz  ·  %s%s  ·  %.1f Mbit/s" % (
        w, h, fps, tx,
        ("原样转发 %d%%" % raw) if raw is not None else "",
        ("  %.0f KB/张" % cam["kb"]) if cam.get("kb") else "",
        cam.get("mbps") or 0.0)
    add("识别相机", st, val, "; ".join(notes))


def run_checks(args):
    s = STORE.snapshot()
    now = time.time()
    out = []

    def add(name, status, value="", detail=""):
        out.append({"name": name, "status": status, "value": value, "detail": detail})

    # ---- ROS master / 图
    with STORE.lock:
        g = dict(STORE.graph)
    if s["mode"] == "fake":
        add("数据源", "info", "FAKE 模拟数据", "页面演示用, 不连接飞机")
    if g.get("master_restarted"):
        add("ROS master", "fail", "master 重启过",
            "ROS1 节点不会自动重新注册: 本页的所有话题数据都已失效。"
            "去 tmux 里 Ctrl+C 重启 flight_monitor.py")
    elif g["t"] is None or now - g["t"] > 6:
        add("ROS master", "fail", "无响应", g.get("error") or "roscore 没起来?")
    elif not g["ok"]:
        add("ROS master", "fail", "查询失败", g.get("error") or "")
    else:
        add("ROS master", "ok", "%d 个节点" % len(g["nodes"]))

        gc = [n for n in g["nodes"] if "geometric_controller" in n]
        if gc:
            add("geometric_controller", "fail", "在运行: " + ", ".join(gc),
                "开机自启, 会注入竞争设定点 (历史 z 轴不稳的根因)。"
                "执行: rosnode kill /geometric_controller")
        else:
            add("geometric_controller", "ok", "未运行")

        # mavros 自己会发布 /mavros/setpoint_raw/target_* (飞控回报的"实际在执行的
        # 设定点"), 不算别人在发设定点。
        me = s.get("script_node") or SCRIPT_NODE
        others = {}
        for topic, nodes in g["setpoint_pubs"].items():
            if "/target_" in topic:
                continue
            o = [n for n in nodes if n not in (me, SCRIPT_NODE, "/flight_monitor")
                 and not n.startswith("/mavros")]
            if o:
                others[topic] = o
        if others:
            add("设定点话题", "fail",
                "; ".join("%s ← %s" % (t, ", ".join(n)) for t, n in others.items()),
                "除了飞行脚本, 还有别的节点在往 /mavros/setpoint_* 发数据 —— 必须先停掉")
        else:
            add("设定点话题", "ok", "无其他发布者")

        # ---- 版本配套: 飞行脚本 / vision_bridge / 本程序用的话题名必须一致
        bridges = [n for n in g["nodes"] if "vision_bridge" in n]
        if bridges and not any(b in (g.get("status_subs") or []) for b in bridges):
            add("版本配套", "warn", "vision_bridge 没订阅 %s" % STATUS_TOPIC,
                "地面站 HUD 看不到阶段。检查 vision_bridge 的 _status_topic 参数")
        else:
            add("版本配套", "ok",
                "话题前缀 %s/" % SCRIPT_NODE + ("  vision_bridge 已订阅" if bridges else ""))

        camera_check(add, g, s, bridges, args)

    # ---- MAVROS
    mv = s["mavros"]
    if not mv or mv.get("state_age") is None or mv["state_age"] > 3:
        add("MAVROS", "fail", "无 /mavros/state", "mavros 没起来?")
    elif not mv.get("connected"):
        add("MAVROS", "fail", "未连上飞控")
    else:
        add("MAVROS", "ok", "已连接  模式 %s  %s" % (
            mv.get("mode"), "已解锁" if mv.get("armed") else "未解锁"))

    # ---- 频率
    def rate_check(name, key, ok_hz, warn_hz, hint):
        r = s["rates"].get(key) or 0.0
        a = s["ages"].get(key)
        val = "%.1f Hz" % r + ("  (%.1fs 前)" % a if a is not None and r == 0 else "")
        if r >= ok_hz:
            add(name, "ok", val)
        elif r >= warn_hz:
            add(name, "warn", val, hint)
        else:
            add(name, "fail", val, hint)

    rate_check("位姿 local_position", "pose", 15, 5, "VINS 没起 → 没有位置, 无法 OFFBOARD")
    rate_check("VINS odometry", "vins", 8, 3,
               "VINS 自启会崩, 需手动: tmux 里 roslaunch vins stereo120.launch (见帮助页)")
    rate_check("vision_pose → 飞控", "vision_pose", 15, 5, "VINS 到 mavros 的桥")

    gate_checks(add, s)                  # 视觉链路: /gate/status (原 /track/target 的检查并进这里)
    px4_check(add, s)

    # ---- 电池
    v = s["batt_v"]
    if v is None or v <= 0:
        add("电池", "warn", "读不到电压", "低压保护会按默认 4S 判")
    else:
        cells = 6 if v >= 18.0 else 4
        pc = v / cells
        st = "ok" if pc >= 3.8 else ("warn" if pc >= 3.6 else "fail")
        add("电池", st, "%.2f V  %dS  %.2f V/芯" % (v, cells, pc),
            "" if st == "ok" else "电压偏低, 起飞后很快会碰到低压缓降")

    # ---- 静止漂移
    if mv.get("armed"):
        add("静止漂移", "info", "已解锁, 不适用")
    else:
        dr = STORE.static_drift()
        if dr is None:
            add("静止漂移", "warn", "样本不足")
        else:
            st = "ok"
            if dr["xy_std"] > 0.015 or dr["z_std"] > 0.03:
                st = "fail"
            add("静止漂移 (3s)", st, "xy %.1f cm  z %.1f cm"
                % (dr["xy_std"] * 100, dr["z_std"] * 100),
                "preflight 门限 xy<1.5cm z<3cm; 不过就会拒绝起飞" if st != "ok" else "")

    # ---- 脚本 / 档案
    if s["script_alive"]:
        t = s["telemetry"] or {}
        add("飞行脚本", "info", "运行中  阶段 %s" % t.get("phase"))
    else:
        add("飞行脚本", "info", "未运行",
            "在**普通** SSH 窗口里启动 (不要 tmux / VSCode 终端, 脚本会拒绝)")
    if fp is None:
        add("flight_profile.py", "fail", "找不到", "放到 %s" % HERE)
    else:
        n = len(fp.list_sites(args.sites_dir))
        add("现场档案", "ok" if n else "warn", "%d 个" % n, args.sites_dir)
    if not args.scripts:
        add("飞行脚本文件", "fail", "找不到", "同目录放 stageNN_gate.py")
    else:
        add("飞行脚本文件", "ok" if args.script else "warn",
            "  ".join("%s %s" % (TASK_LABEL.get(t, t), os.path.basename(p))
                      for t, p in sorted(args.scripts.items())),
            "默认任务 %s (%s)" % (TASK_LABEL.get(args.task, args.task), args.task_src))
    return out


# =============================================================================
# 配置: dry-run / 写档案
# =============================================================================
_preview_lock = threading.Lock()
# 档案名: 中文/字母/数字/_ - . 都行 (★2026-10-21 加了 "."; 以前"穿环快速2.5"这种名字在网页里读不出来,
#   表单被静默清空成代码默认值)。不许: 路径分隔符、空白、引号等 shell 特殊字符、以 . 或 - 开头、".."
NAME_RE = re.compile(r"^(?![.\-])(?!.*\.\.)[\w一-鿿\-.]{1,40}$")
SET_RE = re.compile(r"^[A-Z][A-Z0-9_]*=[^\s]+$")


def normalize_knobs(raw):
    """网页表单 → 旋钮 dict。空值 = 不给 (用代码默认值)。"""
    if not isinstance(raw, dict):
        raise ValueError("knobs 必须是对象")
    out = {}
    for k, v in raw.items():
        spec = fp.KNOB_INDEX.get(k)
        if spec is None:
            raise ValueError("未知旋钮 %r" % k)
        if v is None or (isinstance(v, str) and v.strip() == ""):
            continue
        if spec["type"] == "float":
            try:
                f = float(v)
            except (TypeError, ValueError):
                raise ValueError("%s 需要数字, 得到 %r" % (k, v))
            if not math.isfinite(f):
                raise ValueError("%s 需要有限数字, 得到 %r" % (k, v))
            out[k] = f
        else:
            s = str(v).strip()
            if "#" in s or "\n" in s or "=" in s:
                raise ValueError("%s 不能包含 # = 或换行" % k)
            out[k] = s
    return out


def normalize_sets(raw):
    if raw is None:
        return []
    if isinstance(raw, str):
        raw = raw.splitlines()
    out = []
    for line in raw:
        line = str(line).strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("--set"):
            line = line[5:].strip()
        if line.startswith("set."):
            line = line[4:].replace(" ", "")
        if not SET_RE.match(line):
            raise ValueError("--set 行格式应为 KEY=VALUE (大写参数名, 值不含空格): %r"
                             % line)
        out.append(line)
    return out


def dry_run(args, knobs, sets):
    """
    把旋钮 + set 写进临时档案, 跑 <飞行脚本> --dry-run --json。
    set 也写进档案 (而不是拼 --set), 预览走的就是保存后真正起飞时的同一条路径。
    """
    tmp = tempfile.mkdtemp(prefix="fm_preview_")
    try:
        with open(os.path.join(tmp, "__preview__.conf"), "w", encoding="utf-8") as f:
            f.write(fp.format_conf(knobs, header="flight_monitor 预览", sets=sets))
        task, script = script_for(args, knobs)
        if not script or not os.path.exists(script):
            return {"ok": False, "errors": [_no_script_msg(args, task)]}
        cmd = [sys.executable, script, "--sites-dir", tmp,
               "--site", "__preview__", "--dry-run", "--json"]
        with _preview_lock:
            p = subprocess.run(cmd, cwd=os.path.dirname(script),
                               capture_output=True, text=True, timeout=30)
        try:
            res = json.loads(p.stdout)
        except ValueError:
            return {"ok": False, "errors": ["dry-run 没有输出 JSON (退出码 %d)"
                                            % p.returncode],
                    "stderr": (p.stderr or p.stdout)[-3000:]}
        res["stderr"] = (p.stderr or "")[-2000:]
        res["exit_code"] = p.returncode
        res["script"] = os.path.basename(script)
        return res
    except subprocess.TimeoutExpired:
        return {"ok": False, "errors": ["dry-run 超时"]}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def script_cmd(args, tail="", script=None):
    """飞行脚本的启动命令(带 cd, 复制到任何目录下的 SSH 窗口里都能直接跑)。"""
    script = script or args.script
    if not script:
        return "cd ~/<目录> && python3 stageNN_<任务>.py"
    d = os.path.dirname(script)
    home = os.path.expanduser("~")
    if d == home or d.startswith(home + os.sep):
        d = "~" + d[len(home):]
    return ("cd %s && python3 %s %s" % (d, os.path.basename(script), tail)).rstrip()


def default_config(args):
    """代码默认值的 dry-run 结果, 脚本启动前给页面作参考。"""
    if fp is None or not args.script or not os.path.exists(args.script):
        return None
    cmd = [sys.executable, args.script, "--dry-run", "--json",
           "--sites-dir", args.sites_dir]
    try:
        p = subprocess.run(cmd, cwd=os.path.dirname(args.script),
                           capture_output=True, text=True, timeout=30)
        res = json.loads(p.stdout)
    except Exception as e:
        print("[monitor] 取代码默认配置失败: %s" % e)
        return None
    return {"t": time.time(), "src": "代码默认值 (脚本还没启动, 仅供参考)",
            "task": res.get("task"),     # 脚本没跑时网页靠它选布局 (INTERFACE_gate §4)
            "knobs": {}, "lines": res.get("lines", []),
            "warnings": res.get("warnings", []) + res.get("errors", []),
            "derived": res.get("derived"), "params": res.get("params", {}),
            "origin": None, "log_dir": None}


def save_site(args, body):
    name = str(body.get("name", "")).strip()
    if not NAME_RE.match(name) or name.startswith("__"):
        return 400, {"ok": False, "error": "档案名只能用中文/字母/数字/_/-, 1~40 字"}
    s = STORE.snapshot()
    armed = bool(s["mavros"].get("armed")) or bool(
        s["script_alive"] and (s["telemetry"] or {}).get("air"))
    if armed:
        return 409, {"ok": False, "error": "飞机已解锁/在空中 —— 飞行中不写档案"}
    try:
        knobs = normalize_knobs(body.get("knobs") or {})
        sets = normalize_sets(body.get("sets"))
    except ValueError as e:
        return 400, {"ok": False, "error": str(e)}
    res = dry_run(args, knobs, sets)
    if not res.get("ok"):
        return 422, {"ok": False, "error": "校验不通过, 不保存",
                     "errors": res.get("errors", [])}
    d = os.path.expanduser(args.sites_dir)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, name + ".conf")
    if os.path.exists(path):
        if not body.get("overwrite"):
            return 409, {"ok": False, "error": "档案 %s 已存在 (勾选覆盖, 或换个名字)"
                                               % name, "exists": True}
        shutil.copy2(path, path + ".bak")
    header = "由 flight_monitor 网页生成 %s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(fp.format_conf(knobs, header=header, sets=sets))
    os.replace(tmp, path)
    # set 已经在档案里了 → 启动命令只有 --site, 不会因为少复制一段而变成另一套配置
    script = script_for(args, knobs)[1]
    cmd = script_cmd(args, "--site %s" % name, script)
    STORE.add_event("INFO", "已保存现场档案 %s" % name, "monitor")
    out = {"ok": True, "path": path, "command": cmd, "dry_run_command": cmd + " --dry-run"}
    if body.get("launcher"):
        out["launcher"] = write_launcher(args, name, script=script)
    return 200, out


# ★2026-10-12: 「只生成启动脚本, 不存档案」—— 临时测试用。
#   配置写到档案目录**外面**的一个固定位置 (每次覆盖), 不会在档案列表里多出一堆副本。
#   fly.sh 的自查照旧: 这份临时配置被别人 (另一台电脑再点一次) 覆盖了 → sha 对不上 → 要 yes。
TEMP_SITE = "临时配置"
TEMP_DIR = os.path.expanduser("~/.flight_monitor/tmp_site")


def _flying():
    s = STORE.snapshot()
    return bool(s["mavros"].get("armed")) or bool(
        s["script_alive"] and (s["telemetry"] or {}).get("air"))


def temp_launcher(args, body):
    if _flying():
        return 409, {"ok": False, "error": "飞机已解锁/在空中 —— 飞行中不生成启动脚本"}
    try:
        knobs = normalize_knobs(body.get("knobs") or {})
        sets = normalize_sets(body.get("sets"))
    except ValueError as e:
        return 400, {"ok": False, "error": str(e)}
    res = dry_run(args, knobs, sets)
    if not res.get("ok"):
        return 422, {"ok": False, "error": "校验不通过, 不生成",
                     "errors": res.get("errors", [])}
    os.makedirs(TEMP_DIR, exist_ok=True)
    path = os.path.join(TEMP_DIR, TEMP_SITE + ".conf")
    base = str(body.get("base") or "").strip()
    header = ("临时配置 —— 网页「只生成启动脚本」写的, 没有存为档案 (下次生成会覆盖)\n"
              "%s%s" % (datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        ("  基于档案 %s" % base) if base else "  基于代码默认值"))
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(fp.format_conf(knobs, header=header, sets=sets))
    os.replace(tmp, path)
    return 200, {"ok": True, "path": path,
                 "launcher": write_launcher(args, TEMP_SITE, conf=path, temp=True,
                                            script=script_for(args, knobs)[1])}


# =============================================================================
# 启动脚本 (~/fly.sh): 网页生成, 在普通 SSH 终端里直接运行
# =============================================================================
# 运行时先自查三件事, 任何一件不对就打印警告 + 档案内容, 要求输入 yes 才起飞:
#   · 生成超过 --launcher-max-age 秒 (默认 60) —— 防止拿上一场 / 早上那套配置飞
#   · 档案在生成之后被改过 (sha256 对不上) —— 现在生效的不是网页上预览过的那套
#   · 飞行脚本在生成之后被改过 (更新过代码、flight.py 软链接换了目标)
# 档案或飞行脚本不存在 → 直接拒绝。最后用 exec 把终端交给 python:
# SSH 断开时飞行脚本要能收到 stdin EOF 才会自动降落, 中间不能隔一层 bash。
# 时间戳用的是飞机自己的时钟 (生成和运行都在飞机上), 不需要和笔记本对时。
LAUNCHER_TMPL = r"""#!/usr/bin/env bash
# =============================================================================
# 飞行启动脚本 —— 由 flight_monitor 网页生成于 @@GEN_STR@@，不要手改 (改了也会被下次生成覆盖)
#   档案       @@SITE@@   (@@CONF@@)
#   飞行脚本   @@SCRIPT_PATH@@
# 用法: 笔记本上新开的**普通** SSH 终端 (不要 tmux / VSCode 终端), 运行
#       @@LAUNCHER@@
# 生成超过 @@MAX_AGE@@ 秒、档案或飞行脚本在生成后被改过 → 先警告, 输入 yes 才起飞。
# =============================================================================
GEN_TS=@@GEN_TS@@
MAX_AGE=@@MAX_AGE@@
SITE=@@Q_SITE@@
CONF=@@Q_CONF@@
SITES_DIR=@@Q_SITES@@
CONF_SHA=@@CONF_SHA@@
SCRIPT_DIR=@@Q_DIR@@
SCRIPT=@@Q_SCRIPT@@
SCRIPT_SHA=@@SCRIPT_SHA@@
TEMP=@@TEMP@@

R=$'\e[1;31m'; Y=$'\e[1;33m'; G=$'\e[1;32m'; N=$'\e[0m'
fmt_age() {
  local s=$1
  if [ "$s" -ge 3600 ]; then echo "$((s / 3600)) 小时 $((s % 3600 / 60)) 分"
  elif [ "$s" -ge 60 ]; then echo "$((s / 60)) 分 $((s % 60)) 秒"
  else echo "$s 秒"; fi
}
sha() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

if [ ! -f "$CONF" ]; then
  echo "${R}档案不存在: $CONF —— 拒绝起飞。回网页「起飞配置」重新生成。${N}"; exit 1
fi
if [ ! -f "$SCRIPT_DIR/$SCRIPT" ]; then
  echo "${R}飞行脚本不存在: $SCRIPT_DIR/$SCRIPT —— 拒绝起飞。${N}"; exit 1
fi

age=$(( $(date +%s) - GEN_TS ))
warn=()
if [ "$age" -lt -5 ]; then
  warn+=("生成时间比现在晚 $(( -age )) 秒 —— 飞机时钟跳过, 判断不了这个脚本新不新")
elif [ "$age" -gt "$MAX_AGE" ]; then
  warn+=("这个启动脚本是 $(fmt_age "$age")前生成的 (超过 ${MAX_AGE} 秒) —— 确认它就是这次要飞的配置")
fi
[ "$(sha "$CONF")" = "$CONF_SHA" ] || \
  warn+=("档案 $SITE 在生成之后被改过 —— 现在生效的已不是网页上预览过的那套")
[ "$(sha "$SCRIPT_DIR/$SCRIPT")" = "$SCRIPT_SHA" ] || \
  warn+=("飞行脚本 $SCRIPT 在生成之后被改过 (更新过代码 / flight.py 换了目标?)")

if [ ${#warn[@]} -gt 0 ]; then
  line="$(printf '!%.0s' $(seq 1 72))"
  echo "${Y}${line}${N}"
  echo "${Y}  启动前请确认:${N}"
  for w in "${warn[@]}"; do echo "${Y}  ⚠️  $w${N}"; done
  echo
  echo "  档案 $CONF 当前内容:"
  sed -e 's/[[:space:]]*#.*$//' -e '/^[[:space:]]*$/d' -e 's/^/      /' "$CONF"
  echo "${Y}${line}${N}"
  read -r -p "  确认用这套配置起飞, 输入 yes 回车 (其它任何输入 = 取消): " ans || exit 1
  if [ "$ans" != "yes" ]; then
    echo "已取消。回网页「起飞配置」重新生成, 或核对后再运行。"; exit 1
  fi
else
  echo "${G}档案 $SITE  ·  启动脚本 $(fmt_age "$age")前生成${N}"
  if [ "$TEMP" = "1" ]; then
    # 临时配置没有档案名可核对 → 每次都把内容摊开给飞手看一眼
    echo "  临时配置 (没有存为档案) 内容:"
    body="$(sed -e 's/[[:space:]]*#.*$//' -e '/^[[:space:]]*$/d' -e 's/^/      /' "$CONF")"
    if [ -n "$body" ]; then echo "$body"; else echo "      (空 = 全部代码默认值)"; fi
  fi
fi

cd "$SCRIPT_DIR" || exit 1
# exec: python 直接接管这个终端 —— SSH 断开时它要收到 stdin EOF 才能自动降落
exec python3 "$SCRIPT" --sites-dir "$SITES_DIR" --site "$SITE"
"""


def _sha256(path):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _tilde(p):
    home = os.path.expanduser("~")
    return "~" + p[len(home):] if p == home or p.startswith(home + os.sep) else p


def write_launcher(args, name, conf=None, temp=False, script=None):
    """按档案 name 生成启动脚本。调用方负责先校验档案 (dry-run 通过)。
    conf/temp: 临时配置 (不在档案目录里, 见 temp_launcher)。
    script: 用哪个飞行脚本 (按档案的任务选, 见 script_for); 不给 = 默认任务的脚本。"""
    import shlex
    script = script or args.script
    conf = conf or os.path.join(os.path.expanduser(args.sites_dir), name + ".conf")
    path = os.path.expanduser(args.launcher)
    now = time.time()
    vals = {
        "GEN_STR": datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S"),
        "GEN_TS": str(int(now)),
        "MAX_AGE": str(int(args.launcher_max_age)),
        "SITE": name, "CONF": conf, "SCRIPT_PATH": script,
        "LAUNCHER": _tilde(path),
        "Q_SITE": shlex.quote(name), "Q_CONF": shlex.quote(conf),
        "Q_SITES": shlex.quote(os.path.dirname(conf)),
        "CONF_SHA": _sha256(conf), "SCRIPT_SHA": _sha256(script),
        "Q_DIR": shlex.quote(os.path.dirname(script)),
        "Q_SCRIPT": shlex.quote(os.path.basename(script)),
        "TEMP": "1" if temp else "0",
    }
    text = LAUNCHER_TMPL
    for k, v in vals.items():
        text = text.replace("@@%s@@" % k, v)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    os.chmod(tmp, 0o755)
    os.replace(tmp, path)
    STORE.add_event("INFO", "已生成启动脚本 %s (%s)" % (
        _tilde(path), "临时配置, 未存档案" if temp else "档案 " + name), "monitor")
    return {"path": path, "command": _tilde(path), "gen_ts": now, "site": name, "temp": temp}


def launcher_status(args):
    """读出现有启动脚本的元数据, 并像它运行时一样判断是否会警告。"""
    path = os.path.expanduser(args.launcher)
    st = {"exists": False, "path": path, "command": _tilde(path),
          "max_age": args.launcher_max_age, "now": time.time()}
    if not os.path.exists(path):
        return st
    meta = {}
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                m = re.match(r"^(GEN_TS|SITE|CONF|CONF_SHA|SCRIPT_DIR|SCRIPT|SCRIPT_SHA|TEMP)=(.*)$",
                             line.rstrip("\n"))
                if m:
                    meta[m.group(1)] = m.group(2).strip("'")
    except OSError as e:
        st["error"] = str(e)
        return st
    if "GEN_TS" not in meta:
        st["error"] = "不是网页生成的启动脚本"
        return st
    st.update(exists=True, site=meta.get("SITE"), gen_ts=float(meta["GEN_TS"]),
              temp=meta.get("TEMP") == "1")
    conf = meta.get("CONF", "")
    scr = os.path.join(meta.get("SCRIPT_DIR", ""), meta.get("SCRIPT", ""))
    st["conf_missing"] = not os.path.exists(conf)
    st["conf_changed"] = (not st["conf_missing"]) and _sha256(conf) != meta.get("CONF_SHA")
    st["script_missing"] = not os.path.exists(scr)
    st["script_changed"] = (not st["script_missing"]) and _sha256(scr) != meta.get("SCRIPT_SHA")
    st["script"] = scr
    return st


def regen_launcher(args, body):
    """按已保存的档案重新生成启动脚本 (先重新跑一遍 dry-run 校验)。"""
    name = str(body.get("name", "")).strip()
    if not NAME_RE.match(name) or name.startswith("__"):
        return 400, {"ok": False, "error": "档案名不合法"}
    if _flying():
        return 409, {"ok": False, "error": "飞机已解锁/在空中 —— 飞行中不生成启动脚本"}
    temp = bool(body.get("temp"))
    knobs, sets, path, errs = fp.load_site(name, TEMP_DIR if temp else args.sites_dir)
    if errs:
        return 422, {"ok": False, "error": "档案读不懂", "errors": errs}
    res = dry_run(args, knobs or {}, sets or [])
    if not res.get("ok"):
        return 422, {"ok": False, "error": "校验不通过, 不生成", "errors": res.get("errors", [])}
    return 200, {"ok": True, "launcher": write_launcher(
        args, name, conf=path if temp else None, temp=temp,
        script=script_for(args, knobs or {})[1])}


# =============================================================================
# 文件与路径 (机况检查页「文件与路径」表)
# =============================================================================
def paths_info(args):
    """
    把本程序实际用到的每个路径都列出来 (展开 ~、解析软链接、看在不在)。
    所有文件放在同一个文件夹里时, 只有 档案目录 / 启动脚本 / 飞行日志 在文件夹外面。
    """
    rows = []

    def add(name, path, note="", kind="file"):
        p = os.path.expanduser(path) if path else ""
        ok = bool(p) and (os.path.isdir(p) if kind == "dir" else os.path.isfile(p))
        real = os.path.realpath(p) if ok else ""
        rows.append({"name": name, "path": p, "real": real if real and real != p else "",
                     "exists": ok, "kind": kind, "note": note})

    add("监控程序目录", HERE, "flight_monitor.py 所在目录; 下面同目录的文件都从这里找", "dir")
    add("monitor.html", os.path.join(HERE, "monitor.html"), "网页本身, 必须和 flight_monitor.py 同目录")
    fp_path = (getattr(fp, "__file__", "") if fp else "") or os.path.join(HERE, "flight_profile.py")
    add("flight_profile.py", fp_path,
        "现场档案的解析/校验。监控和飞行脚本各自从**自己所在目录** import 它")
    for t, sp in sorted(args.scripts.items()) or [("gate", "")]:
        add("飞行脚本 · %s" % TASK_LABEL.get(t, t), sp,
            getattr(args, "script_src", "") + ("  ★默认任务" if t == args.task else ""))
        if not sp:
            continue
        sdir = os.path.dirname(os.path.realpath(sp))
        if sdir != os.path.realpath(HERE):
            rows[-1]["note"] += ("  ⚠️ 不在监控目录: 飞行脚本会从 %s 找 flight_profile.py, "
                                 "和网页用的可能不是同一份" % sdir)
    n = len(fp.list_sites(args.sites_dir)) if fp else 0
    add("现场档案目录", args.sites_dir,
        "%d 个 .conf  · 来源: %s" % (n, getattr(args, "sites_src", "")), "dir")
    add("启动脚本", args.launcher, "网页「生成启动脚本」写到这里; 在普通 SSH 终端里运行它")
    add("  └ 临时配置", os.path.join(TEMP_DIR, TEMP_SITE + ".conf"),
        "「只生成启动脚本 (不存档案)」时写这里, 每次覆盖, 不进档案列表")

    # 飞行日志: 脚本在跑 → 用它报的 log_dir; 否则找 ~/uav_logs 下最新的一次
    with STORE.lock:
        live = (STORE.config or {}).get("log_dir") or (STORE.telemetry or {}).get("log_dir")
    root = os.path.expanduser(args.log_root)
    add("飞行日志根目录", root, "飞行脚本的 LOG_ROOT; 每次飞行一个 flight_NNNN 子目录 (编号递增)"
        " (网页「回放」读这里)", "dir")
    last = live
    if not last and os.path.isdir(root):
        subs = sorted(d for d in os.listdir(root) if d.startswith("flight_"))
        last = os.path.join(root, subs[-1]) if subs else ""
    if last:
        add("本次/最近一次飞行日志", last,
            "飞行中 (飞行脚本报的)" if live else "最近一次 (按目录名)", "dir")
        add("  └ site_config.txt", os.path.join(last, "site_config.txt"),
            "那次生效的全部参数 —— 复盘先看它")
    # 视觉链路: 新相机直连版桥 (和本程序同目录)
    add("图传桥 vision_bridge.py", os.path.join(HERE, "vision_bridge.py"),
        "自己打开新相机 (by-id), 掉线自动重开; 老相机只给 VINS")
    add("共用代码包 vb09/", os.path.join(HERE, "vb09"),
        "2026-10-30 起: 几何 / 网络格式 / 环检测与融合 / 规划都在这里, 和 .py 入口放同一个文件夹", "dir")
    add("起视觉链路 run_vision.py", os.path.join(HERE, "run_vision.py"),
        "python3 run_vision.py gate_gcs|gate|aruco [--gcs IP]  —— 会先停旧抓帧节点/看门狗/旧桥")
    add("  └ 桥日志", "/tmp/bridge.log", "")
    old = [os.path.expanduser("~/tools/" + f) for f in
           ("det_cam_node.py", "det_cam_watchdog.py", "prep_det_cam.sh", "run_bridge11.sh")]
    left = [o for o in old if os.path.exists(o)]
    if left:
        rows.append({"name": "旧新相机管线 (已不需要)", "path": "~/tools/ 下 %d 个文件" % len(left),
                     "real": "", "exists": None, "kind": "note",
                     "note": "det_cam_node / watchdog / prep_det_cam.sh / run_bridge11.sh —— "
                             "不要再跑, 会和新版桥抢相机; 可以删掉"})
    rows.append({"name": "地面站 (不在飞机上)",
                 "path": "gcs_aruco_det2.py (aruco 模式) / gcs_gate.py (gate_gcs 模式) + preview_server.py",
                 "real": "", "exists": None, "kind": "note",
                 "note": "放识别笔记本的同一个目录; gcs_gate.py 还要 vb09/common + vb09/perception 和 "
                         "stereo_calib.yaml; 预览地址 http://127.0.0.1:8091/snapshot.jpg"})
    return {"rows": rows, "host": HOSTNAME, "t": time.time()}


# =============================================================================
# 飞行日志 (网页「回放」页, 只读)
# =============================================================================
def _live_log_name():
    with STORE.lock:
        live = (STORE.config or {}).get("log_dir") or (STORE.telemetry or {}).get("log_dir")
    return os.path.basename(str(live).rstrip("/")) if live else None


def list_logs(args, limit=80):
    """~/uav_logs 下的飞行目录 (新的在前), 每个目录里能读的 csv/txt/log 文件。"""
    root = os.path.expanduser(args.log_root)
    out = {"root": root, "flights": [], "live": _live_log_name()}
    if not os.path.isdir(root):
        return out
    names = sorted((d for d in os.listdir(root)
                    if LOG_DIR_RE.match(d) and os.path.isdir(os.path.join(root, d))),
                   reverse=True)[:limit]
    for n in names:
        p = os.path.join(root, n)
        files = []
        try:
            for f in sorted(os.listdir(p)):
                fpath = os.path.join(p, f)
                if LOG_FILE_RE.match(f) and os.path.isfile(fpath):
                    files.append({"name": f, "size": os.path.getsize(fpath)})
            out["flights"].append({"name": n, "mtime": os.path.getmtime(p), "files": files})
        except OSError:
            continue
    return out


def read_log(args, d, f):
    """→ (http 状态码, bytes 或 dict)。只认 flight_* 目录下的 csv/txt/log, 不跟出根目录。"""
    root = os.path.realpath(os.path.expanduser(args.log_root))
    if not LOG_DIR_RE.match(d or "") or not LOG_FILE_RE.match(f or ""):
        return 400, {"ok": False, "error": "目录名/文件名不合法"}
    p = os.path.realpath(os.path.join(root, d, f))
    if not p.startswith(root + os.sep) or not os.path.isfile(p):
        return 404, {"ok": False, "error": "没有这个文件"}
    if os.path.getsize(p) > LOG_MAX_BYTES:
        return 413, {"ok": False, "error": "文件太大 (> %d MB), 用 scp 拷出来看"
                     % (LOG_MAX_BYTES // 1024 // 1024)}
    with open(p, "rb") as fh:
        return 200, fh.read()


# =============================================================================
# HTTP
# =============================================================================
class Handler(BaseHTTPRequestHandler):
    server_version = "flight_monitor/1.0"
    args = None

    def log_message(self, fmt, *a):          # 不刷屏
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
        if not isinstance(body, (bytes, bytearray)):
            body = json.dumps(clean(body), ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n > 65536:
            raise ValueError("请求太大")
        raw = self.rfile.read(n) if n else b"{}"
        return json.loads(raw.decode("utf-8") or "{}")

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        a = self.args
        try:
            if u.path in ("/", "/index.html"):
                with open(os.path.join(HERE, "monitor.html"), "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if u.path == "/api/state":
                return self._send(200, STORE.snapshot())
            if u.path == "/api/config":
                return self._send(200, STORE.get_config())
            if u.path == "/api/events":
                since = int((q.get("since") or ["0"])[0])
                return self._send(200, {"events": STORE.get_events(since)})
            if u.path == "/api/checks":
                return self._send(200, {"checks": run_checks(a), "t": time.time()})
            if u.path == "/api/paths":
                return self._send(200, paths_info(a))
            if u.path == "/api/stream":
                try:
                    since = int((q.get("since") or ["0"])[0])
                except ValueError:
                    since = 0
                return self._send(200, STORE.stream(since))
            if u.path == "/api/logs":
                return self._send(200, list_logs(a))
            if u.path == "/api/log":
                code, body = read_log(a, (q.get("dir") or [""])[0], (q.get("file") or [""])[0])
                if isinstance(body, (bytes, bytearray)):
                    return self._send(code, body, "text/plain; charset=utf-8")
                return self._send(code, body)
            if u.path == "/fake/snapshot.jpg":
                r = FAKE_CAM.get("render")
                if STORE.mode != "fake" or r is None:
                    return self._send(503, b"fake camera only with --fake --fake-task gate",
                                      "text/plain; charset=utf-8")
                img, meta = r()
                return self._send(200, img, "image/svg+xml", {
                    "X-Frame-Age": "%.3f" % meta.pop("_age", 0.0),
                    "X-Det": json.dumps(clean(meta), ensure_ascii=True, separators=(",", ":"))})
            if fp is None and u.path.startswith(("/api/schema", "/api/site")):
                return self._send(503, {"ok": False, "error": "flight_profile.py 不在"})
            if u.path == "/api/schema":
                return self._send(200, fp.schema_json())
            if u.path == "/api/launcher":
                return self._send(200, launcher_status(a))
            if u.path == "/api/sites":
                d = os.path.expanduser(a.sites_dir)
                items = []
                bad = []
                for n in fp.list_sites(a.sites_dir):
                    p = os.path.join(d, n + ".conf")
                    # ★2026-10-21 一个读不了的 (断开的软链接 / 刚被删 / 没权限) 不能让整个列表报错
                    try:
                        items.append({"name": n, "mtime": os.path.getmtime(p)})
                    except OSError as e:
                        bad.append({"name": n, "error": "%s: %s" % (type(e).__name__, e.strerror or e)})
                return self._send(200, {"sites": items, "dir": d, "bad": bad,
                                        "script": os.path.basename(a.script or ""),
                                        "script_cmd": script_cmd(a),
                                        "task": a.task,
                                        "scripts": {t: {"name": os.path.basename(sp),
                                                        "cmd": script_cmd(a, script=sp)}
                                                    for t, sp in a.scripts.items()},
                                        "launcher": _tilde(os.path.expanduser(a.launcher)),
                                        "monitor_dir": HERE})
            if u.path == "/api/site":
                name = (q.get("name") or [""])[0]
                if not NAME_RE.match(name):
                    return self._send(400, {"ok": False, "error": "档案名不合法"})
                knobs, sets, path, errs = fp.load_site(name, a.sites_dir)
                text = ""
                if os.path.exists(path):
                    with open(path, encoding="utf-8") as f:
                        text = f.read()
                return self._send(200, {"ok": not errs, "name": name, "path": path,
                                        "knobs": knobs or {}, "sets": sets or [],
                                        "errors": errs, "text": text})
            return self._send(404, {"ok": False, "error": "not found"})
        except BrokenPipeError:
            pass
        except Exception as e:
            try:
                self._send(500, {"ok": False, "error": "%s: %s" % (type(e).__name__, e)})
            except Exception:
                pass

    def do_POST(self):
        u = urlparse(self.path)
        a = self.args
        try:
            if fp is None:
                return self._send(503, {"ok": False, "error": "flight_profile.py 不在"})
            body = self._body()
            if u.path == "/api/preview":
                try:
                    knobs = normalize_knobs(body.get("knobs") or {})
                    sets = normalize_sets(body.get("sets"))
                except ValueError as e:
                    return self._send(200, {"ok": False, "errors": [str(e)]})
                return self._send(200, dry_run(a, knobs, sets))
            if u.path == "/api/save":
                code, res = save_site(a, body)
                return self._send(code, res)
            if u.path == "/api/launcher":
                code, res = regen_launcher(a, body)
                return self._send(code, res)
            if u.path == "/api/launcher_temp":
                code, res = temp_launcher(a, body)
                return self._send(code, res)
            return self._send(404, {"ok": False, "error": "not found"})
        except BrokenPipeError:
            pass
        except Exception as e:
            try:
                self._send(500, {"ok": False, "error": "%s: %s" % (type(e).__name__, e)})
            except Exception:
                pass


# =============================================================================
# ROS 侧
# =============================================================================
def start_ros(args, srv=None):
    import rospy
    import rosgraph
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State, ExtendedState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Log

    rospy.init_node("flight_monitor", anonymous=False, disable_signals=True)

    # 本节点被关 (同名节点顶掉 / master 退出 / rosnode kill) → 网页服务也退出, 让出端口。
    # 否则会留下"网页还在、ROS 已死"的僵尸: 页面显示已连接, 数据却一直不更新。
    if srv is not None:
        def _on_ros_shutdown():
            print("[monitor] ROS 节点已关闭 (被同名节点顶掉 / master 退出?) —— 网页服务一起退出")
            threading.Thread(target=srv.shutdown, daemon=True).start()
        rospy.on_shutdown(_on_ros_shutdown)

    def on_state(m):
        STORE.set_state(m.connected, m.armed, m.mode)

    def on_pose(m):
        p, q = m.pose.position, m.pose.orientation
        STORE.set_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_vel(m):
        v = m.twist.linear
        STORE.set_vel(v.x, v.y, v.z)

    def on_json(setter):
        def cb(m):
            try:
                setter(json.loads(m.data))
            except ValueError:
                pass
        return cb

    def on_rosout(m):
        if m.name != STORE.script_node:
            return
        msg = (m.msg or "").rstrip()
        if m.level >= Log.WARN:
            lvl = "ERROR" if m.level >= Log.ERROR else "WARN"
        elif m.level == Log.INFO:
            # 周期性状态行以空格缩进开头、分隔线以 = / - 开头 → 不进事件流
            if not msg or msg[0] in " =-" or len(msg) > 300:
                return
            lvl = "INFO"
        else:
            return
        STORE.add_event(lvl, msg, "script")

    S = rospy.Subscriber
    S("/mavros/state", State, on_state, queue_size=1)
    S("/mavros/local_position/pose", PoseStamped, on_pose, queue_size=1)
    S("/mavros/local_position/velocity_local", TwistStamped, on_vel, queue_size=1)
    S("/mavros/battery", BatteryState, lambda m: STORE.set_batt(m.voltage), queue_size=1)
    S("/mavros/extended_state", ExtendedState,
      lambda m: STORE.set_landed(m.landed_state), queue_size=1)
    S("/vins_estimator/odometry", Odometry, lambda m: STORE.tick("vins"), queue_size=1)
    S("/mavros/vision_pose/pose", PoseStamped,
      lambda m: STORE.tick("vision_pose"), queue_size=1)
    S(args.telemetry_topic, String, on_json(STORE.set_telemetry), queue_size=1)
    S(args.config_topic, String, on_json(STORE.set_config), queue_size=1)
    S("/rosout", Log, on_rosout, queue_size=50)
    # 新相机状态: 图传桥自己报 (1Hz JSON); 本程序不碰相机、不订任何图像
    S(args.cam_status_topic, String, on_json(STORE.set_cam), queue_size=1)
    # 环识别: 桥 gate / gate_gcs 模式每帧发一条 (15~30Hz)。queue_size 给大一点: 进 /api/stream 不丢帧
    S(args.gate_status_topic, String, on_json(STORE.set_gate), queue_size=10)

    def px4_loop():
        """
        PX4 机动参数 (MPC_TILTMAX_AIR 等): 只在飞控已连接且**未解锁**时读。
        读的是 mavros 自己的参数缓存 (开机后 mavros 会整表拉一遍), 不额外占 MAVLink 链路。
        """
        from mavros_msgs.srv import ParamGet
        try:
            from mavros_msgs.srv import ParamPull
        except ImportError:
            ParamPull = None
        proxy = pull = None
        while not rospy.is_shutdown():
            time.sleep(2.0)
            with STORE.lock:
                mv = dict(STORE.mavros)
                t_ok, t_try = STORE.px4.get("t"), STORE.px4.get("t_try")
            if not mv.get("connected") or mv.get("armed"):
                continue
            now = time.time()
            if t_ok is not None and now - t_ok < args.px4_refresh:
                continue
            if t_try is not None and now - t_try < 10.0 and t_ok is None:
                continue                                  # 上次失败了, 10s 后再试
            try:
                if proxy is None:
                    rospy.wait_for_service("/mavros/param/get", timeout=1.0)
                    proxy = rospy.ServiceProxy("/mavros/param/get", ParamGet)
                # ★2026-10-19 (穿环首飞问题 §3): 先确认 mavros 参数缓存就绪再逐个 get。
                #   缓存没拉完时每个 get 都失败, mavros 每次刷一行 "PR: Unknown parameter to get"。
                #   pull(force_pull=False): 缓存空就开始拉、正在拉就等它拉完、已就绪立刻返回。
                if ParamPull is not None and t_ok is None:
                    if pull is None:
                        rospy.wait_for_service("/mavros/param/pull", timeout=1.0)
                        pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
                    STORE.set_px4(None, "参数未就绪: 等 mavros 拉完参数表 …")
                    pr = pull(force_pull=False)
                    if not pr.success or int(pr.param_received) == 0:
                        STORE.set_px4(None, "参数未就绪 (mavros 拉参数表没完成, 收到 %d 个) —— 10s 后再试"
                                      % int(pr.param_received))
                        continue
                vals, miss = {}, []
                for p in PX4_PARAMS:
                    r = proxy(param_id=p)
                    if r.success:
                        v = r.value
                        vals[p] = float(v.real) if v.real != 0.0 else int(v.integer)
                    else:
                        miss.append(p)
                STORE.set_px4(vals, ("没读到 %s (mavros 参数还没拉完?)" % ", ".join(miss))
                              if miss else None)
            except Exception as e:
                proxy = None
                STORE.set_px4(None, "读参数失败: %s" % e)

    threading.Thread(target=px4_loop, daemon=True).start()

    def graph_loop():
        master = rosgraph.Master("/flight_monitor")
        # 记住 master 的 PID。ROS1 节点在 master 重启后不会自动重新注册, 本程序的
        # 订阅会全部悄悄失效 (页面显示"无数据")。PID 一变就在事件流和机况检查里说出来。
        pid0 = None
        while not rospy.is_shutdown():
            try:
                pid = master.getPid()
                if pid0 is None:
                    pid0 = pid
                elif pid != pid0 and not STORE.graph.get("master_restarted"):
                    STORE.set_graph(master_restarted=True)
                    STORE.add_event("ERROR", "ROS master 重启过 (PID %s → %s) —— 本页数据已失效,"
                                    " 请在 tmux 里重启 flight_monitor.py" % (pid0, pid),
                                    "monitor")
                    print("[monitor] ★ ROS master 重启过, 本程序的订阅已失效, 请重启我")
                pubs, subs, _srvs = master.getSystemState()
                nodes = set()
                sp, ssub = {}, []
                img_subs, det_pubs = [], []
                for topic, ns in pubs:
                    nodes.update(ns)
                    if topic.startswith(SETPOINT_PREFIX):
                        sp[topic] = list(ns)
                    if topic == args.det_image_topic:
                        det_pubs = list(ns)
                for topic, ns in subs:
                    nodes.update(ns)
                    if topic == args.status_topic:
                        ssub = list(ns)
                    # 桥订了图像话题 = 旧版桥 (新版自己开相机, 不订图像)
                    if any("vision_bridge" in n for n in ns) and (
                            "image" in topic or topic in (args.det_image_topic, args.old_image_topic)):
                        img_subs.append(topic)
                STORE.set_graph(ok=True, nodes=sorted(nodes), setpoint_pubs=sp,
                                status_subs=ssub,
                                bridge_img_subs=sorted(img_subs), det_cam_pubs=det_pubs,
                                error=None)
            except Exception as e:
                STORE.set_graph(ok=False, error=str(e))
            STORE.watch_script()
            time.sleep(1.0)

    threading.Thread(target=graph_loop, daemon=True).start()


# =============================================================================
# FAKE 侧 —— 笔记本上离线试页面
# =============================================================================
quat_from_euler = geom.quat_from_euler          # FAKE 模式造数据用


# -----------------------------------------------------------------------------
# FAKE 穿环 (--fake --fake-task gate)
#   一架假飞机看着一个假环: 起飞 → 悬停识别 → 对准 → 冲刺穿环 → 制动 → 返航。
#   按 INTERFACE_gate.md 的字段发 /gate/status 和遥测 gate 段 (网页按真接口收)。
#   另外画一张假识别画面 /fake/snapshot.jpg (SVG): 画面里的环用和网页叠加**同一套**
#   约定投影 (T_WC 行主序, 相机光学系 x右 y下 z前) —— 网页按感知结果叠加的环应当和
#   画面里的环基本重合 (差几厘米噪声), 用来核对投影和坐标约定。
#   红点故意不在环正中, 冲刺中故意向右漂 5cm, 看靶面视图和预计穿越点。
# -----------------------------------------------------------------------------
FAKE_CAM = {}           # {"render": fn}  —— /fake/snapshot.jpg 用
FAKE_PX4 = {"MPC_XY_VEL_MAX": 12.0, "MPC_XY_CRUISE": 5.0, "MPC_ACC_HOR": 3.0,
            "MPC_ACC_HOR_MAX": 5.0, "MPC_JERK_AUTO": 4.0, "MPC_TILTMAX_AIR": 45.0,
            "MPC_XY_P": 0.95, "MPC_XY_VEL_P_ACC": 1.8, "MPC_Z_VEL_MAX_UP": 3.0,
            "COM_OBL_ACT": 2}



# 小向量 / 靶面轴: 2026-10-30 起和 gate_plan / 网页共用 vb09/common/geom.py (返回元组)
_va, _vs, _vm, _vd, _vx, _vu = geom.vadd, geom.vsub, geom.vmul, geom.vdot, geom.vcross, geom.vunit
gate_axes = geom.gate_axes


def cam_R_WC(yaw, pitch_down):
    """
    相机光学系 (x右 y下 z前) → world 的旋转。机体只有 yaw + 低头角 (FLU, 正 = 机头向下);
    相机安装同 vision_bridge.make_T_BC 全 0 (朝正前方、水平)。
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch_down), math.sin(pitch_down)
    Rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    Ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    R0 = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]   # 列 = 相机 x/y/z 轴在机体系

    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mm(mm(Rz, Ry), R0)


def start_fake_gate(args):
    import random
    STORE.mode = "fake"
    STORE.set_px4(FAKE_PX4)
    base = json.loads(json.dumps(STORE.config_fallback or {"params": {}, "derived": {}}))
    P = base.get("params") or {}
    der = base.setdefault("derived", {}) or {}
    origin = [0.0, 0.0, 0.05, 0.0]
    C = [5.5, 0.8, 1.65]                                    # 环中心 (world)
    N = _vu([-1.0, -0.15, 0.0])                             # 法向, 指向进近侧
    RIGHT, UP = gate_axes(N)
    AIM = _va(_va(C, _vm(RIGHT, 0.12)), _vm(UP, -0.08))    # 红点: 偏右 12cm、偏下 8cm
    G = {"r_in": 0.6, "r_out": 0.8, "dot_d": 0.11, "dash_v": 2.5, "dash_acc": 4.0,
         "brake_acc": 3.0, "d_align": 3.0, "d_commit": 3.0, "overshoot": 0.8}
    V, a1, a2 = G["dash_v"], G["dash_acc"], G["brake_acc"]
    G["brake_dist"] = round(V * V / (2 * a2), 3)
    hard = der.get("xy_hard") or 11.0
    der["gate"] = G
    der.setdefault("phase_envelope", {})["gate"] = [hard, 4.0]
    base["task"] = "gate"
    STORE.config_fallback = base

    S0 = [origin[0], origin[1], origin[2] + 1.5]            # 悬停点 (= 起飞高度)
    A = _va(AIM, _vm(N, G["d_align"]))                       # 对准点
    t1, s1 = V / a1, V * V / (2 * a1)
    s_brake = G["d_align"] + G["overshoot"]                  # 过环后多冲 overshoot 才开始刹
    tc = (s_brake - s1) / V
    t2, s2 = V / a2, V * V / (2 * a2)
    E = _va(A, _vm(N, -(s_brake + s2)))                      # 制动终点
    segs = [("align", 5.0), ("commit", 0.6), ("dash", t1 + tc), ("brake", t2), ("hold", 2.5)]
    timeline = [("idle", 3), ("preflight", 2), ("prime", 1), ("takeoff", 3), ("hover", 6),
                ("gate", sum(d for _, d in segs)), ("return_home", 7), ("descend", 4),
                ("auto_land", 2), ("cleanup", 2)]
    total = sum(d for _, d in timeline)
    yaw_face = math.atan2(-N[1], -N[0])
    plan = {"seq": 1, "pts": [S0, A, AIM, E], "labels": ["悬停点", "对准点", "穿越点", "制动终点"]}
    rnd = random.Random(7)

    def phase_at(t):
        t = t % total
        for name, d in timeline:
            if t < d:
                return name, t
            t -= d
        return "idle", 0.0

    def seg_at(tin):
        for name, d in segs:
            if tin < d:
                return name, tin
            tin -= d
        return "hold", segs[-1][1]

    def smooth(u):
        u = max(0.0, min(1.0, u))
        return u * u * (3 - 2 * u), 6 * u * (1 - u)

    def dash_s(tau):
        if tau < t1:
            return 0.5 * a1 * tau * tau, a1 * tau, a1
        if tau < t1 + tc:
            return s1 + V * (tau - t1), V, 0.0
        tb = min(tau - t1 - tc, t2)
        return s_brake + V * tb - 0.5 * a2 * tb * tb, V - a2 * tb, -a2

    def gate_kin(tin):
        """→ (位置, 速度, 沿前进方向的加速度, 段名)"""
        seg, ts = seg_at(tin)
        if seg == "align":
            f, df = smooth(ts / 5.0)
            d = _vs(A, S0)
            return _va(S0, _vm(d, f)), _vm(d, df / 5.0), 0.0, seg
        if seg == "commit":
            return list(A), [0.0, 0.0, 0.0], 0.0, seg
        tau = ts if seg == "dash" else segs[2][1] + (ts if seg == "brake" else segs[3][1])
        s, v, acc = dash_s(tau)
        drift = 0.05 * min(1.0, tau / 1.2)
        p = _va(_va(A, _vm(N, -s)), _vm(RIGHT, drift))
        if seg == "hold":
            v, acc = 0.0, 0.0
        return p, _vm(N, -v), acc, seg

    def perceive(now, p, yaw, pitch):
        R = cam_R_WC(yaw, pitch)
        fwd = [R[0][2], R[1][2], R[2][2]]
        rel = _vs(AIM, p)
        zc = _vd(rel, fwd)
        dist = math.sqrt(_vd(rel, rel))
        along = _vd(_vs(p, C), N)
        if along < 0.05:
            stage = "lost"
        elif zc < 0.25:
            stage = "search"
        elif dist > 4.0:
            stage = "far"
        elif dist > 2.2:
            stage = "mid"
        elif dist > 1.3:
            stage = "near"
        elif dist > 0.45:
            stage = "terminal"
        else:
            stage = "blind"
        ring_ok = stage in ("far", "mid", "near")
        st = {"t": now, "stage": stage, "pose": "ok", "ms": 38.0,
              "det_ms": {"ring": 22.0, "dot": 9.0, "total": 38.0},
              "ring": {"ok": ring_ok, "src": "checker" if dist < 3.5 else "color",
                       "partial": stage == "near",
                       "corners": int(max(0, 40 - 6 * dist)) if ring_ok else 0, "side": "both"},
              "dots": ["red"] if stage in ("mid", "near", "terminal") else [],
              "used": ["ring", "dot"] if stage in ("mid", "near") else ["ring"],
              "rejected": [], "skip_ring": stage == "terminal", "diag": {},
              "sim": False, "ground_sim": False, "ground_stage": stage}
        if stage != "search":
            sd = [0.6 + 1.0 * dist, 0.4 + 0.5 * dist, 0.4 + 0.5 * dist]       # cm, x/y/z

            def nse(i):
                return rnd.gauss(0.0, sd[i] / 100.0)
            st.update(
                target=[round(C[i] + nse(i), 4) for i in range(3)],
                dist=round(dist, 3), sd_cm=[round(v, 1) for v in sd],
                src="plane" if dist < 3.5 else "mono_disp", age_ms=110, side="both",
                published=stage in ("far", "mid", "near", "terminal"),
                n=[round(v, 4) for v in _vu([N[0] + rnd.gauss(0, 0.015),
                                            N[1] + rnd.gauss(0, 0.015),
                                            N[2] + rnd.gauss(0, 0.008)])],
                n_sd_deg=round(0.6 + 0.4 * dist, 2),
                cov=[(sd[0] / 100) ** 2, 0, 0, 0, (sd[1] / 100) ** 2, 0, 0, 0, (sd[2] / 100) ** 2],
                aim=[round(AIM[i] + nse(i), 4) for i in range(3)],
                aim_sd_cm=[round(v, 1) for v in sd], r_in=G["r_in"], r_out=G["r_out"])
        return st

    cur = {}                       # 给 /fake/snapshot.jpg: 最新一帧的相机位姿 + 感知

    def render():
        c = cur.get("v")
        W, Hh = 960, 540
        K = [787.0, 787.0, 480.0, 270.0]
        head = ('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">'
                '<rect width="100%%" height="100%%" fill="#2a3138"/>' % (W, Hh, W, Hh))
        if not c:
            return (head + "</svg>").encode(), {"_age": 0.0}
        R, t = cam_R_WC(c["yaw"], c["pitch"]), c["p"]

        def proj(pw):
            d = _vs(pw, t)
            xc = R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2]
            yc = R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2]
            zc = R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2]
            if zc < 0.05:
                return None
            return K[0] * xc / zc + K[2], K[1] * yc / zc + K[3]

        def loop_path(pts):
            pp = [proj(q) for q in pts]
            if any(q is None for q in pp):
                return None
            return "M" + "L".join("%.1f,%.1f" % q for q in pp) + "Z"

        def circle(center, r, n=72):
            return loop_path([_va(center, _va(_vm(RIGHT, r * math.cos(2 * math.pi * k / n)),
                                              _vm(UP, r * math.sin(2 * math.pi * k / n))))
                              for k in range(n)])
        out = [head]
        # 地面网格 (z=0), 只为有个空间感
        for gx in range(-2, 12):
            for (a_, b_) in (([gx, -5, 0], [gx, 5, 0]),):
                pa, pb = proj(a_), proj(b_)
                if pa and pb:
                    out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#3b444d"/>'
                               % (pa + pb))
        for gy in range(-5, 6):
            pa, pb = proj([0.3, gy, 0]), proj([12, gy, 0])
            if pa and pb:
                out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#3b444d"/>'
                           % (pa + pb))
        o, i = circle(C, G["r_out"]), circle(C, G["r_in"])
        if o and i:
            out.append('<path d="%s %s" fill="#b8323a" fill-rule="evenodd" stroke="#eee" '
                       'stroke-width="1"/>' % (o, i))
        h = 0.18
        paper = loop_path([_va(AIM, _va(_vm(RIGHT, sx * h), _vm(UP, sy_ * h)))
                           for sx, sy_ in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        if paper:
            out.append('<path d="%s" fill="#f1f1ea"/>' % paper)
        dot = circle(AIM, G["dot_d"] / 2, 36)
        if dot:
            out.append('<path d="%s" fill="#e0262b"/>' % dot)
        out.append('<text x="12" y="26" fill="#ddd" font-size="18" font-family="sans-serif">'
                   'FAKE 识别画面 · %s</text></svg>' % c["st"].get("stage"))
        meta = dict(c["st"])                      # bridge /gate/status 原样格式 (网页要能兼容)
        meta["rtt"] = 160.0
        meta["proj"] = {"K": K, "size": [W, Hh], "approx": False, "t": c["t"],
                        "T_WC": [R[0][0], R[0][1], R[0][2], t[0],
                                 R[1][0], R[1][1], R[1][2], t[1],
                                 R[2][0], R[2][1], R[2][2], t[2], 0.0, 0.0, 0.0, 1.0]}
        meta["_age"] = time.time() - c["t"]
        return "".join(out).encode("utf-8"), meta

    FAKE_CAM["render"] = render

    def loop():
        t0 = time.time()
        last_ph, last_tel, last_cam = None, 0.0, 0.0
        locked = None
        prev_v = [0.0, 0.0, 0.0]
        while True:
            now = time.time()
            ph, tin = phase_at(now - t0)
            seg, acc = None, 0.0
            if ph in ("idle", "preflight", "prime", "cleanup", "auto_land"):
                p, v, yaw = [origin[0], origin[1], origin[2]], [0.0, 0.0, 0.0], \
                    (yaw_face if ph in ("auto_land", "cleanup") else 0.0)
                if ph == "auto_land":
                    p = [S0[0], S0[1], origin[2]]
            elif ph == "takeoff":
                f, df = smooth(tin / 3.0)
                p, v, yaw = [S0[0], S0[1], origin[2] + 1.45 * f], [0, 0, 1.45 * df / 3.0], 0.0
            elif ph == "hover":
                f, _ = smooth(tin / 4.0)
                p, v, yaw, seg = list(S0), [0.0, 0.0, 0.0], yaw_face * f, "wait"
            elif ph == "gate":
                p, v, acc, seg = gate_kin(tin)
                yaw = yaw_face
            elif ph == "return_home":
                f, df = smooth(tin / 7.0)
                d = _vs(S0, E)
                p, v, yaw = _va(E, _vm(d, f)), _vm(d, df / 7.0), yaw_face
            else:                                                   # descend
                f, df = smooth(tin / 4.0)
                p = [S0[0], S0[1], S0[2] - (S0[2] - origin[2]) * f]
                v, yaw = [0.0, 0.0, -(S0[2] - origin[2]) * df / 4.0], yaw_face
            flying = ph in ("takeoff", "hover", "gate", "return_home", "descend")
            pitch = math.atan(acc / 9.81) if ph == "gate" else 0.0      # 加速低头, 刹车抬头
            if ph != "gate":
                a_fwd = (_vd(v, [math.cos(yaw), math.sin(yaw), 0]) -
                         _vd(prev_v, [math.cos(yaw), math.sin(yaw), 0])) / 0.05
                pitch = max(-0.3, min(0.3, math.atan(a_fwd / 9.81)))
            prev_v = v
            roll = math.radians(1.2 * math.sin(now * 1.7))
            STORE.set_pose(p[0], p[1], p[2], *quat_from_euler(roll, pitch, yaw))
            STORE.set_vel(v[0], v[1], v[2])
            STORE.set_batt(16.4 - 0.004 * ((now - t0) % total))
            armed = flying or ph == "prime"
            STORE.set_state(True, armed, "OFFBOARD" if armed else "POSCTL")
            STORE.tick("vins")
            STORE.tick("vision_pose")

            # ---- 感知 (bridge 每帧发一条 /gate/status)
            st = perceive(now, p, yaw, pitch)
            STORE.set_gate(st)
            cur["v"] = {"p": p, "yaw": yaw, "pitch": pitch, "st": st, "t": now}
            if now - last_cam >= 1.0:
                last_cam = now
                STORE.set_cam({"t": now, "ok": True, "err": "", "dev": "/dev/v4l/by-id/FAKE-STYT",
                               "w": 3840, "h": 1080, "cam_fps": 15.0, "tx_hz": 15.0,
                               "raw_pct": None, "kb": 95.0, "mbps": 11.1, "drop": 0,
                               "rx_hz": 15.0, "reopens": 0, "raw_supported": True,
                               "frame_age": 0.02,
                               "gate": {"hz": 20.0, "ms": 38.0, "stage": st["stage"],
                                        "dist": st.get("dist"), "pose": "ok", "src": st.get("src")},
                               "cfg": {"mode": "gate_gcs", "cam_fps": 15.0, "tx_hz": 15.0,
                                       "tx_max_w": 1920, "raw_max_kb": 260, "jpeg_q": 85}})

            # ---- 脚本遥测: 穿环段 20Hz, 其余 5Hz
            alive = ph != "idle"
            hz = 20.0 if ph in ("hover", "gate") else 5.0
            if alive and now - last_tel >= 1.0 / hz - 1e-3:
                last_tel = now
                env = (der.get("phase_envelope") or {}).get(ph) or \
                    (der.get("phase_envelope") or {}).get("_default") or [0.8, 1.0]
                gsec = {}
                if seg is not None:
                    if seg in ("wait", "align") or locked is None:
                        if st.get("published"):
                            locked = {"c": st["target"], "n": st["n"], "aim": st["aim"],
                                      "sd_cm": st["sd_cm"], "n_sd_deg": st["n_sd_deg"],
                                      "locked": False, "t_lock": None}
                    elif seg == "commit" and not locked.get("locked"):
                        locked = dict(locked, locked=True, t_lock=now)
                    if locked:
                        c, n = locked["c"], _vu(locked["n"])
                        rr, uu = gate_axes(n)
                        q = _vs(p, c)
                        along = _vd(q, n)
                        off = [_vd(q, rr), _vd(q, uu)]
                        v_al = -_vd(v, n)
                        ttc = along / v_al if (v_al > 0.2 and along > 0) else None
                        pred = ([off[0] + _vd(v, rr) * ttc, off[1] + _vd(v, uu) * ttc]
                                if ttc is not None else list(off))
                        sdc = locked["sd_cm"]
                        gsec = {
                            "seg": seg, "gate_used": locked,
                            "along": along, "off": off, "pred_cross": pred,
                            "pred_sd_cm": [math.hypot(sdc[1], 2.0), math.hypot(sdc[2], 2.0)],
                            "ttc": ttc, "v_along_meas": v_al,
                            "v_along_cmd": (V if seg == "dash" else max(0.0, v_al)),
                            "gate_age": 0.11,
                            "commit": {"d": G["d_commit"], "passed": seg in ("dash", "brake", "hold"),
                                       "abort_ok": seg in ("wait", "align", "commit")},
                        }
                        if seg != "wait":
                            gsec["plan"] = plan
                            gsec["brake"] = {"end": E, "dist": s2, "acc": a2}
                    else:
                        gsec = {"seg": seg}
                tele = {
                    "t": now, "node": SCRIPT_NODE, "task": "gate", "schema": 2,
                    "phase": ph, "sub_mode": "", "air": p[2] - origin[2] > 0.15,
                    "abort": False, "land": False, "abort_reason": None, "land_reason": None,
                    "t_requested": False, "shadow": P.get("GATE_SHADOW", False),
                    "sim_drive": P.get("GATE_ALLOW_SIM_DRIVE", False),
                    "z_enable": False, "origin": origin,
                    "setpoint": {"mode": "ZHOLD", "px": p[0], "py": p[1], "pz": p[2],
                                 "yaw": yaw, "vx": v[0], "vy": v[1]},
                    "z_filt": p[2], "cells": 4, "fly_min_v": 14.0, "envelope": env,
                    "log_dir": "~/uav_logs/flight_FAKE",
                    "track": {}, "gates": {}, "gate": gsec,
                    "safety": {} if not flying else {
                        "horiz_lim": env[0], "vel_lim": env[1], "max_height": 11.0,
                        "max_tilt": 30.0, "pose_stale": 0.15, "vision_lost": 0.8,
                        "jump_max": 0.2, "jump": 0.01, "confirm_s": 0.25,
                        "deb": {"height": 0, "horiz": 0, "tilt": 0},
                        "dz": p[2] - origin[2], "horiz": math.hypot(p[0], p[1]),
                        "tilt": abs(math.degrees(pitch)), "pose_age": 0.03, "odom_age": 0.08,
                        "horiz_v": math.hypot(v[0], v[1]), "batt_v": 16.3, "fly_min_v": 14.0},
                }
                for sec in ("gate", "safety"):
                    if tele[sec]:
                        tele[sec]["t"] = now
                STORE.set_telemetry(tele)
                if ph != last_ph:
                    if ph == "preflight":
                        c = json.loads(json.dumps(base))
                        c.update(t=now, origin=origin, src="FAKE 穿环: " + str(base.get("src")))
                        STORE.set_config(c)
                        locked = None
                    if ph == "gate":
                        STORE.add_event("WARN", "[gate] 对准 → 提交 → 冲刺 %.1f m/s" % V)
                    last_ph = ph
            if not alive and last_ph not in (None, "idle"):
                last_ph = "idle"
                with STORE.lock:
                    STORE.config = None
            if int(now) != int(now - 0.05):
                STORE.set_graph(ok=True,
                                nodes=["/mavros", "/vins_estimator", "/rosout", "/vision_bridge"]
                                + ([SCRIPT_NODE] if alive else []),
                                setpoint_pubs={"/mavros/setpoint_raw/local":
                                               [SCRIPT_NODE] if alive else []},
                                status_subs=["/vision_bridge"],
                                bridge_img_subs=[], det_cam_pubs=[])
            STORE.watch_script()
            time.sleep(0.05)

    threading.Thread(target=loop, daemon=True).start()


# =============================================================================
# main
# =============================================================================
def bind_or_die(args):
    """
    先占端口。被占 → 打印是谁占着, 退出 (不碰 ROS, 不会把正在运行的那个实例顶掉)。
    """
    try:
        srv = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as e:
        if e.errno != errno.EADDRINUSE:
            raise
        print("[monitor] ✗ 端口 %d 已被占用 —— 多半是上一个 flight_monitor 还活着"
              " (另一个 tmux 窗口 / 会话?)" % args.port)
        try:
            out = subprocess.run(["ss", "-ltnp", "sport = :%d" % args.port],
                                 capture_output=True, text=True, timeout=3).stdout.strip()
            if out:
                print("    占用者:\n" + "\n".join("      " + l for l in out.splitlines()))
        except Exception:
            pass
        print("    找它:  tmux ls   →  进那个窗口 Ctrl+C")
        print("    或者:  pkill -f flight_monitor.py   (只会结束监控, 不影响飞行脚本)")
        print("    确实要两个并存 (比如一个 --fake): 另一个加 --port 8081")
        sys.exit(1)
    srv.daemon_threads = True
    return srv


def main():
    ap = argparse.ArgumentParser(description="飞机上的网页监控站")
    ap.add_argument("--host", default="127.0.0.1",
                    help="监听地址 (默认只本机, 通过 SSH 端口映射访问)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--script", default=None,
                    help="只用这一个飞行脚本 (默认: 同目录编号最大的 stageNN_gate.py; "
                         "有 flight.py 软链接就只用它)")
    ap.add_argument("--task", choices=("auto", "gate"), default="auto",
                    help="档案没写任务时用哪种任务的脚本 (只剩 gate)")
    ap.add_argument("--sites-dir", default=None,
                    help="现场档案目录 (默认: flight_profile.DEFAULT_SITES_DIR)")
    ap.add_argument("--node", default=SCRIPT_NODE,
                    help="飞行脚本节点名 (脚本启动后以遥测里的为准)")
    ap.add_argument("--telemetry-topic", default=SCRIPT_NODE + "/telemetry")
    ap.add_argument("--config-topic", default=SCRIPT_NODE + "/config")
    ap.add_argument("--status-topic", default=STATUS_TOPIC,
                    help="vision_bridge 应订阅的状态话题 (只用于版本配套检查)")
    ap.add_argument("--cam-status-topic", default="/vision_bridge/cam",
                    help="图传桥报相机状态的话题 (1Hz JSON)")
    ap.add_argument("--det-image-topic", default="/det_cam/image_raw",
                    help="旧抓帧节点的话题: 有人发 = 它还在跑, 会和桥抢相机 (本程序不订它)")
    ap.add_argument("--old-image-topic", default="/stereo_cam/left/image_raw",
                    help="老相机图像话题: 桥订它 = 旧版桥")
    ap.add_argument("--gate-status-topic", default="/gate/status",
                    help="环识别状态 (vision_bridge gate / gate_gcs 模式每帧 JSON)")
    ap.add_argument("--log-root", default="~/uav_logs",
                    help="飞行日志根目录 (网页「回放」只读这里, 默认 ~/uav_logs)")
    ap.add_argument("--px4-refresh", type=float, default=30.0,
                    help="未解锁时多久重读一次 PX4 机动参数 (秒, 默认 30)")
    ap.add_argument("--launcher", default="~/fly.sh",
                    help="网页生成的启动脚本放哪 (默认 ~/fly.sh)")
    ap.add_argument("--launcher-max-age", type=float, default=60.0,
                    help="启动脚本生成超过多少秒, 运行时要求输入 yes 确认 (默认 60)")
    ap.add_argument("--nice", type=int, default=10, help="降低进程优先级 (默认 10)")
    ap.add_argument("--fake", action="store_true", help="不连 ROS, 用模拟数据试页面")
    ap.add_argument("--fake-task", choices=("gate",), default="gate",
                    help="--fake 时模拟哪种任务 (只剩 gate = 穿环, 带假识别画面)")
    args = ap.parse_args()
    # 记下每个路径是怎么来的 (「机况检查 → 文件与路径」里显示)
    # ★2026-10-19 飞行脚本按任务找 (见 discover_scripts)
    one = args.script or (os.path.join(HERE, "flight.py")
                          if os.path.exists(os.path.join(HERE, "flight.py")) else None)
    if one:
        one = os.path.abspath(one)
        t = task_of_script(one) or "gate"
        args.scripts = {t: one}
        args.script_src = "来源: --script 参数" if args.script else "来源: 同目录的 flight.py"
    else:
        args.scripts = discover_scripts()
        args.script_src = "来源: 同目录 stage<版本>_<任务>.py 编号最大的"
    if args.task == "auto":
        args.task = "gate"
        args.task_src = "--task auto"
    else:
        args.task_src = "--task 参数"
    args.script = args.scripts.get(args.task)
    if args.sites_dir:
        args.sites_src = "--sites-dir 参数"
    elif fp is not None:
        args.sites_dir = fp.DEFAULT_SITES_DIR
        args.sites_src = "flight_profile.DEFAULT_SITES_DIR (飞行脚本不带 --sites-dir 时也用它)"
    else:
        args.sites_dir = os.path.expanduser("~/flight_sites")
        args.sites_src = "flight_profile.py 不在, 退回 ~/flight_sites"
    STORE.script_node = _abs_node(args.node)
    Handler.args = args

    if args.nice:
        try:
            os.nice(args.nice)
        except (AttributeError, OSError):
            pass
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print("[monitor] ⚠️ 监听 %s —— 同网段的人都能打开并写档案。"
              "正常请用默认 127.0.0.1 + SSH 端口映射。" % args.host)
    if fp is None:
        print("[monitor] ⚠️ 找不到 flight_profile.py —— 配置页不可用")

    # ① 先占端口: 第二个实例在这里就退出, 不会用同名节点把正在运行的那个顶掉
    srv = bind_or_die(args)
    # ② 再做耗时的 dry-run 和 ROS 初始化 (这期间网页已能打开, 只是数据还没来)
    STORE.config_fallback = default_config(args)
    if args.fake:
        start_fake_gate(args)
    else:
        try:
            start_ros(args, srv)
        except ImportError as e:
            print("[monitor] 没有 ROS 环境 (%s)。想离线试页面请加 --fake" % e)
            srv.server_close()
            sys.exit(1)

    print("[monitor] http://%s:%d  (%s)  脚本 %s  默认任务 %s  档案目录 %s"
          % (args.host, args.port, "FAKE" if args.fake else "ROS",
             "  ".join("%s=%s" % (t, os.path.basename(p)) for t, p in sorted(args.scripts.items()))
             or "(没找到)", args.task, args.sites_dir))
    if not args.fake:
        print("[monitor] 笔记本: ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 "
              "-L %d:127.0.0.1:%d visbot@<飞机IP>  然后打开 "
              "http://localhost:%d" % (args.port, args.port, args.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
