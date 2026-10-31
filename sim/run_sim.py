#!/usr/bin/env python3
"""
stage14_mission.py (原 stage12_gate.py) 离线仿真: 桩 rospy + 一个粗糙的 PX4 位置环 + 假识别 (延迟/噪声) + 纸。
  python3 run_sim.py normal|stuck|shadow|virtual|nonormal|oblique|noring|lowbatt|dock|dock_nonormal  [--set K=V ...]
不是飞行动力学仿真 —— 只用来跑通状态机、检查时间表/停止点/遥测字段和日志。
"""
import json
import math
import os
import random
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "stubs"))
sys.path.insert(1, os.path.dirname(HERE))

import rospy                                      # noqa: E402  (桩)
from mavros_msgs.msg import State, ExtendedState  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry                 # noqa: E402
from sensor_msgs.msg import BatteryState          # noqa: E402
from std_msgs.msg import String                   # noqa: E402

G = 9.81
scenario = sys.argv[1] if len(sys.argv) > 1 else "normal"
extra = sys.argv[2:]
random.seed(3)


def unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return [x / n for x in a]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


# ---------------- 场景 ----------------
C = [5.0, 0.4, 1.35]                       # 环心
N = unit([-1.0, -0.15, 0.0])               # 法向 (指向进近侧)
if scenario == "oblique":
    N = unit([-1.0, -0.6, 0.0])            # 斜 31°: 进入方向会被限到 20°
d_ = [-N[0], -N[1], -N[2]]                 # 靶面轴: right = normalize(d × z), up = right × d
RIGHT = unit([d_[1], -d_[0], 0.0])
UP = [RIGHT[1] * d_[2] - RIGHT[2] * d_[1], RIGHT[2] * d_[0] - RIGHT[0] * d_[2],
      RIGHT[0] * d_[1] - RIGHT[1] * d_[0]]
AIM = [C[i] + 0.08 * RIGHT[i] - 0.05 * UP[i] for i in range(3)]
D_BACK, L_F, L_R = 4.0, 0.30, 0.30
V_TEAR = 0.6 if scenario != "stuck" else 99.0

sites = tempfile.mkdtemp()
with open(os.path.join(sites, "sim.conf"), "w") as f:
    f.write("task=穿环\nscene=室内\ntakeoff_h=1.5\nz_low=0.8\nz_high=2.2\nxy_radius=10\n"
            "dash_v=2.5\nd_back=%.2f\nd_rear=2.0\nd_side=2.0\nl_f=%.2f\nl_r=%.2f\nw_half=0.25\n"
            % (D_BACK, L_F, L_R))
    if scenario.startswith("dock"):
        f.write("mode=停靠\ndock_hold=6\n")          # 停靠: 字符串旋钮只能写在档案里
os.environ["HOME"] = tempfile.mkdtemp()
sets = ["--set", "GATE_CONFIRM_DASH=False"]
if scenario == "shadow":
    sets += ["--set", "GATE_SHADOW=True"]
if scenario == "virtual":
    sets += ["--set", "GATE_VIRTUAL_OFFSET=2.6"]
if scenario == "noring":
    sets += ["--set", "GATE_ACQ_TIMEOUT=12"]
sys.argv = ["stage14_mission.py", "--sites-dir", sites, "--site", "sim"] + sets + extra


# ---------------- 飞机 ----------------
class Vehicle(object):
    KP, KV, TAU_A = 1.0, 3.0, 0.08
    TILT_MAX = math.radians(25.0)

    def __init__(self):
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.a = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.armed = False
        self.mode = "POSCTL"
        self.sp = None
        self.sp_t = 0.0
        self.lock = threading.Lock()
        self.hist = []                  # (t, p, yaw) 给识别用
        self.paper = "intact"           # intact / torn / stuck
        self.cross = None               # 穿纸时的 (速度, h, v)
        self.max_past = -9.0            # 机头过纸面最远多少
        self.max_tilt = 0.0
        self.disarm_air = False
        self.log = []

    def on_sp(self, m):
        with self.lock:
            self.sp, self.sp_t = m, time.time()

    def step(self, dt):
        with self.lock:
            sp, fresh = self.sp, time.time() - self.sp_t < 0.5
        a_cmd = [0.0, 0.0, 0.0]
        if self.armed and self.mode == "OFFBOARD" and fresh and sp is not None:
            tm = sp.type_mask
            pos = [sp.position.x, sp.position.y, sp.position.z]
            vff = [0.0 if tm & 8 else sp.velocity.x, 0.0 if tm & 16 else sp.velocity.y,
                   0.0 if tm & 32 else sp.velocity.z]
            aff = [0.0, 0.0, 0.0] if tm & 448 else [sp.acceleration_or_force.x,
                                                    sp.acceleration_or_force.y,
                                                    sp.acceleration_or_force.z]
            if not tm & 7:
                vc = [vff[i] + self.KP * (pos[i] - self.p[i]) for i in range(3)]
            else:
                vc = [sp.velocity.x, sp.velocity.y, sp.velocity.z]
            a_cmd = [aff[i] + self.KV * (vc[i] - self.v[i]) for i in range(3)]
            if not tm & 1024:
                e = math.atan2(math.sin(sp.yaw - self.yaw), math.cos(sp.yaw - self.yaw))
                self.yaw += max(-2.0 * dt, min(2.0 * dt, e))
        elif self.armed and self.mode == "AUTO.LAND":
            a_cmd = [self.KV * (0 - self.v[0]), self.KV * (0 - self.v[1]),
                     self.KV * (-0.7 - self.v[2])]
        ah = math.hypot(a_cmd[0], a_cmd[1])
        amax = G * math.tan(self.TILT_MAX)
        if ah > amax:
            a_cmd[0] *= amax / ah
            a_cmd[1] *= amax / ah
        for i in range(3):
            self.a[i] += (a_cmd[i] - self.a[i]) * dt / self.TAU_A
        if not self.armed:
            self.a = [0.0, 0.0, 0.0]
            self.v = [0.0, 0.0, 0.0]
        # ---- 纸
        along = dot([self.p[i] - C[i] for i in range(3)], N)
        nose = along - L_F                   # 机头相对纸面 (正 = 还没到)
        vn = -dot(self.v, N)
        if self.paper == "intact" and nose <= 0.0 and vn > 0:
            q = [self.p[i] - C[i] for i in range(3)]
            h, vv = dot(q, RIGHT), dot(q, UP)
            self.cross = (vn, h - dot([AIM[i] - C[i] for i in range(3)], RIGHT),
                          vv - dot([AIM[i] - C[i] for i in range(3)], UP))
            if vn >= V_TEAR:
                self.paper = "torn"
                for i in range(3):
                    self.v[i] += 0.15 * N[i]   # 撕纸掉一点速度
            else:
                self.paper = "stuck"
        if self.paper == "stuck" and nose <= 0.0:
            # 纸顶住: 沿 −N 的速度/加速度清零
            vn = -dot(self.v, N)
            if vn > 0:
                for i in range(3):
                    self.v[i] += vn * N[i]
            an = -dot(self.a, N)
            if an > 0:
                for i in range(3):
                    self.a[i] += an * N[i]
        for i in range(3):
            self.v[i] += self.a[i] * dt
            self.p[i] += self.v[i] * dt
        if self.p[2] <= 0.0:
            self.p[2] = 0.0
            self.v[2] = max(self.v[2], 0.0)
            if self.armed and self.mode == "AUTO.LAND":
                self.armed = False
        self.max_past = max(self.max_past, -nose)
        tilt = math.atan(math.hypot(self.a[0], self.a[1]) / G)
        if self.armed and self.p[2] > 0.1:
            self.max_tilt = max(self.max_tilt, math.degrees(tilt))

    def quat(self):
        ah = [self.a[0], self.a[1], 0.0]
        n = math.hypot(ah[0], ah[1])
        th = math.atan(n / G)
        k = [-ah[1] / n, ah[0] / n, 0.0] if n > 1e-6 else [1.0, 0.0, 0.0]   # z × a_h
        s, c = math.sin(th / 2), math.cos(th / 2)
        qt = (k[0] * s, k[1] * s, 0.0, c)
        qy = (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        x1, y1, z1, w1 = qt
        x2, y2, z2, w2 = qy
        return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


veh = Vehicle()
rospy.Subscriber("/mavros/setpoint_raw/local", None, veh.on_sp)


def set_mode(custom_mode=""):
    class R: mode_sent = True
    if custom_mode == "OFFBOARD" and time.time() - veh.sp_t > 0.5:
        R.mode_sent = False
    else:
        veh.mode = custom_mode
    return R


def arming(value):
    class R: success = True; result = 0
    if veh.mode != "OFFBOARD":
        R.success = False
        return R
    veh.armed = bool(value)
    return R


def command(**kw):
    class R: success = True
    if kw.get("command") == 400 and kw.get("param2") == 21196.0:
        if veh.p[2] > 0.15:
            veh.disarm_air = True
        veh.armed = False
    return R


PARAMS = {"COM_OBL_ACT": ("i", 2), "MPC_TILTMAX_AIR": ("r", 25.0), "MPC_XY_VEL_MAX": ("r", 5.0)}


def param_get(param_id=""):
    class V: integer = 0; real = 0.0
    class R: success = True; value = V()
    if param_id not in PARAMS:
        R.success = False
        return R
    t, v = PARAMS[param_id]
    if t == "i":
        R.value.integer = v
    else:
        R.value.real = v
    return R


rospy._services.update({"/mavros/set_mode": set_mode, "/mavros/cmd/arming": arming,
                        "/mavros/cmd/command": command, "/mavros/param/get": param_get})

telem = []
rospy.Subscriber("/flight_ctrl/telemetry", None, lambda m: telem.append(json.loads(m.data)))
config = []
rospy.Subscriber("/flight_ctrl/config", None, lambda m: config.append(json.loads(m.data)))


def sim_loop():
    dt = 0.005
    nxt = time.time()
    k = 0
    det_q = []
    while not rospy.is_shutdown():
        veh.step(dt)
        now = time.time()
        k += 1
        q = veh.quat()
        veh.hist.append((now, list(veh.p), veh.yaw))
        if len(veh.hist) > 2000:
            del veh.hist[:1000]
        if k % 7 == 0:                                   # ~30Hz pose / vel
            m = PoseStamped()
            m.header.stamp = rospy.Time(now)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (
                veh.p[0] + random.gauss(0, 0.001), veh.p[1] + random.gauss(0, 0.001),
                veh.p[2] + random.gauss(0, 0.001))
            (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
             m.pose.orientation.w) = q
            rospy._pub("/mavros/local_position/pose", m)
            t = TwistStamped()
            t.twist.linear.x, t.twist.linear.y, t.twist.linear.z = veh.v
            rospy._pub("/mavros/local_position/velocity_local", t)
        if k % 20 == 0:                                  # 10Hz vins
            o = Odometry()
            o.header.stamp = rospy.Time(now)
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = veh.p
            rospy._pub("/vins_estimator/odometry", o)
        if k % 40 == 0:
            s = State()
            s.armed, s.mode = veh.armed, veh.mode
            rospy._pub("/mavros/state", s)
            e = ExtendedState()
            e.landed_state = 1 if veh.p[2] < 0.05 else 2
            rospy._pub("/mavros/extended_state", e)
        if scenario == "lowbatt" and math.hypot(veh.v[0], veh.v[1]) > 2.0:
            b = BatteryState()
            b.voltage = 13.0                              # 冲刺中掉电压 → LAND 请求
            rospy._pub("/mavros/battery", b)
        elif k % 200 == 0:
            rospy._pub("/mavros/battery", BatteryState())
            rospy._pub("/vision_bridge/cam", String(json.dumps({"cfg": {"mode": "gate_gcs"}})))
        if k % 13 == 0 and scenario != "noring":         # ~15Hz 识别, 延迟 0.25s
            p, yaw = list(veh.p), veh.yaw
            rel = [C[i] - p[i] for i in range(3)]
            dist = math.sqrt(dot(rel, rel))
            brg = math.atan2(rel[1], rel[0])
            vis = (dot([-r for r in rel], N) > 0.3 and 0.8 < dist < 9.0
                   and abs(math.atan2(math.sin(brg - yaw), math.cos(brg - yaw))) < math.radians(40))
            st = {"t": now, "stage": "mid" if vis else "search", "pose": "ok", "sim": False}
            if vis:
                sd = 0.8 + 0.25 * dist
                nn = unit([N[i] + random.gauss(0, 0.01) for i in range(3)])
                st.update(target=[C[i] + random.gauss(0, sd / 200) for i in range(3)],
                          dist=round(dist, 3), sd_cm=[sd, sd, sd], published=True,
                          aim=[AIM[i] + random.gauss(0, sd / 200) for i in range(3)],
                          n_sd_deg=1.0)
                if scenario not in ("nonormal", "dock_nonormal"):
                    st["n"] = nn
            det_q.append((now + 0.25 + random.uniform(-0.03, 0.08), st))
        while det_q and det_q[0][0] <= now:
            rospy._pub("/gate/status", String(json.dumps(det_q.pop(0)[1])))
        if k % 40 == 0:
            veh.log.append((now, list(veh.p), list(veh.v), veh.mode, veh.armed, veh.paper))
        nxt += dt
        d = nxt - time.time()
        if d > 0:
            time.sleep(d)
        else:
            nxt = time.time()


rospy.PRINT[0] = os.environ.get("SIM_QUIET") != "1"
import stage14_mission as S                       # noqa: E402  (档案 setup 在 import 时)
threading.Thread(target=sim_loop, daemon=True).start()
t_start = time.time()
rc = S.main()
rospy.signal_shutdown("done")
time.sleep(0.1)

# ---------------- 结果 ----------------
print("\n" + "=" * 70)
print("场景 %s  rc=%s  用时 %.0fs  墙在纸面后 %.2fm" % (scenario, rc, time.time() - t_start, D_BACK))
print("纸: %s  穿纸时 %s" % (veh.paper, ("v=%.2fm/s 离瞄点 h=%+.3f v=%+.3f m" % veh.cross)
                          if veh.cross else "-"))
print("机头最远过纸面 %.2fm (墙 %.2fm → 余 %.2fm)   最大倾角 %.1f°   空中停桨 %s   最后 armed=%s mode=%s"
      % (veh.max_past, D_BACK, D_BACK - veh.max_past, veh.max_tilt, veh.disarm_air,
         veh.armed, veh.mode))
segs = []
for t in telem:
    g = t.get("gate") or {}
    s = g.get("seg")
    if s and (not segs or segs[-1] != s):
        segs.append(s)
print("遥测 task=%s schema=%s  段序列 %s  帧数 %d" % (telem[-1].get("task"), telem[-1].get("schema"),
                                                 " → ".join(segs), len(telem)))
need = ["seg", "gate_used", "along", "off", "pred_cross", "pred_sd_cm", "ttc", "v_along_meas",
        "v_along_cmd", "gate_age", "commit", "plan", "brake", "t"]
lastg = None
for t in telem:
    if (t.get("gate") or {}).get("plan"):
        lastg = t["gate"]
if lastg:
    print("gate 段缺字段: %s" % [k for k in need if k not in lastg])
    print("plan.pts=%s" % [[round(x, 2) for x in p] for p in lastg["plan"]["pts"]])
dk = None
for t in telem:
    if (t.get("gate") or {}).get("dock"):
        dk = t["gate"]["dock"]
if dk:
    print("停靠: S=%s 距离 %.2fm 方向按%s  误差 中位 %s / 最大 %s cm  aim 抖动 %s  done=%s ok=%s  遥测 mode=%s"
          % (dk.get("S"), dk.get("d"), dk.get("dir_src"), dk.get("err_cm"), dk.get("err_max_cm"),
             dk.get("aim_sd_cm"), dk.get("done"), dk.get("ok"), telem[-1].get("mode")))
cfg = config[-1] if config else {}
print("config task=%s  derived.gate=%s" % (cfg.get("task"), bool((cfg.get("derived") or {}).get("gate"))))
# 冲刺段的 20Hz
ts = [t["t"] for t in telem if (t.get("gate") or {}).get("seg") in ("dash", "brake")]
if len(ts) > 2:
    print("冲刺/制动段遥测 %.1f Hz" % ((len(ts) - 1) / (ts[-1] - ts[0])))
logs = os.path.join(os.environ["HOME"], "uav_logs")
d = sorted(os.listdir(logs))[-1]
with open(os.path.join(logs, d, "gate.csv")) as f:
    rows = f.read().splitlines()
print("gate.csv %d 行, 列 %d" % (len(rows) - 1, len(rows[0].split(","))))
print("日志目录", os.path.join(logs, d))
with open(os.path.join(logs, d, "sim_config_msg.json"), "w") as f:      # 等价验证用 (tools/baseline)
    json.dump(config[-1] if config else None, f, ensure_ascii=False, indent=1, sort_keys=True)
