# -*- coding: utf-8 -*-
"""
vb09/monitor/store.py —— 话题数据汇总 (Store 单例 STORE) + 频率计 + 飞行日志读取 (网页「回放」)
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py); 2026-11-01 并入 logs.py (list_logs / read_log)。
"""

import collections
import math
import os
import statistics
import threading
import time

from vb09.monitor.common import GATE_BUF_N, HOSTNAME, SCRIPT_NODE, STREAM_INIT_GATE_S, STREAM_INIT_TEL_S, STREAM_MAX_GATE, STREAM_MAX_TEL, TEL_BUF_N, _abs_node, att_from_quat, clean
from vb09.monitor.common import LOG_DIR_RE, LOG_FILE_RE, LOG_MAX_BYTES


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
