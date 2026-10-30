# -*- coding: utf-8 -*-
"""
vb09/flight/ctx.py —— 飞行上下文: 共享可变状态单例 F (FlightContext) + 停止 / 降落请求、阶段与包络、遥测分段等小函数
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import math
import threading
from collections import deque

import rospy
from vb09.flight import config as C


class FlightContext(object):
    """
    飞行脚本的共享可变状态 (原 stage12 的模块级全局)。整个进程只有一个实例 F (下面创建),
    各模块 `from vb09.flight.ctx import F` 后用 F.名字 —— 对象本身不会被重新绑定, 所以不会出现
    "某个模块拿着旧副本"的问题 (PLAN_refactor_v2 §7)。以前用 global 重新赋值的
    (INITIAL_* / log_dir / csv_* / writer_* …) 现在都是给 F 的属性赋值。
    tele / setpoint / gate_in 由入口在导入完成后挂上 (见 bind_defaults())。
    """

    def __init__(self):
        self.site_report = None      # flight_profile 的 report (入口 setup/finalize 之后挂上)
        self.fp = None               # flight_profile 模块 (没有 = None)
        self.tele = None             # telemetry.Telemetry()
        self.setpoint = None         # setpoint.SetpointState()
        self.gate_in = None          # target_in.GateInput()
        # =============================================================================
        # ================================   全局   ===================================
        self.abort_flag = threading.Event()  # FATAL: 立即停桨
        self.land_flag = threading.Event()  # LAND:  冻结 → 缓降 → AUTO.LAND
        self.abort_reason = [None]
        self.land_reason = [None]
        self.exc_reason = [None]  # 飞行序列里的普通异常 (不是 FATAL, 不停桨)
        self.in_air = [False]
        self.voluntary_land = [False]
        # preflight [1/6] 读到的飞控模式。退出时 (地上已上锁) 还原成它, 见 restore_ground_mode。
        self.start_mode = [None]
        # 飞控 STATUSTEXT (PX4 的 "Preflight Fail: ..." / "Arming denied ..." 就在这里)。
        # 元素 (t_ros_sec, severity, text)。arm 被拒时按时间窗取出来打印。
        self.fcu_msgs = deque(maxlen=50)
        self.fcu_msgs_lock = threading.Lock()
        # 控制权被接管。值 = 接管时的飞控模式 (如 "POSCTL"), None = 没被接管。
        #   已解锁后, 飞控模式变成**不是我们切的**模式 → 飞手(QGC/遥控)或 PX4 失效保护
        #   拿走了控制权。从这一刻起脚本:
        #     · 不再切任何模式 (否则会把飞手的控制权抢回来)
        #     · 不再 force_disarm
        #     · 只保持发布"原地悬停"设定点, 等飞机落地上锁 (见 phase_handover)
        #   唯一例外: 操作员按 X —— 那是明确的人工停桨指令。
        self.takeover = [None]
        self.phase_ref = ["init"]
        # 穿环段名 (wait / align / commit / dash / brake / hold / abort), 随 status 发给地面站 HUD
        self.sub_mode_ref = [""]
        self.envelope_dyn = [None]  # (horiz_lim, vel_lim) 或 None
        # 冲刺段停桨线开关: GateRun 在按 G 之后置 True, 刹停悬停结束 / 放弃时置回 False
        self.dash_tilt_on = [False]
        # 网页遥测快照。各处只做"整段替换"(一次赋值一个 dict), 读的一方拿到的永远是
        # 某一时刻完整的一段。telem_put 吞掉一切异常 —— 遥测出错绝不能影响控制环和安全监控。
        self.telem_lock = threading.Lock()
        self.telem_data = {"safety": {}, "gate": {}}
        self.INITIAL_X = self.INITIAL_Y = self.INITIAL_Z = self.INITIAL_YAW = 0.0
        self.log_dir = None
        self.csv_main = self.csv_vins = self.csv_ekf = self.csv_timing = self.csv_gate = None
        self.writer_main = self.writer_vins = self.writer_ekf = self.writer_timing = self.writer_gate = None
        self.zdrop_log_path = None
        self.events_fh = None  # events.log 的 logging.FileHandler (fsync 用)


F = FlightContext()


def bind_defaults():
    """入口在 import 完成后调用一次: 建遥测 / setpoint / 环识别接收器 (以前是 stage12 的模块级实例)。"""
    from vb09.flight.setpoint import SetpointState
    from vb09.flight.target_in import GateInput
    from vb09.flight.telemetry import Telemetry
    if F.tele is None:
        F.tele = Telemetry()
    if F.setpoint is None:
        F.setpoint = SetpointState()
    if F.gate_in is None:
        F.gate_in = GateInput()


# =============================================================================
# ==============================   小函数   ===================================
# =============================================================================
def all_params():
    """
    以前直接用 globals() 的地方 (site_config.txt 的全部参数 / /flight_ctrl/config 的 params):
    = C (配置 + 派生量) + 本脚本运行时的大写量 (INITIAL_* / PX4_PULL_TIMEOUT), 和搬家前同一个集合。
    """
    from vb09.flight.preflight import PX4_PULL_TIMEOUT
    g = dict(vars(C))
    g.update({"INITIAL_X": F.INITIAL_X, "INITIAL_Y": F.INITIAL_Y, "INITIAL_Z": F.INITIAL_Z,
              "INITIAL_YAW": F.INITIAL_YAW, "PX4_PULL_TIMEOUT": PX4_PULL_TIMEOUT})
    return g


def cells_from_voltage(v):
    """
    电池电压 → 串数。读不到 (v 为 None 或 <=0) 返回 None, 调用方保持默认并大声警告 ——
    绝不硬猜 6S: 4S 机被误判成 6S, 门槛变成 21.0V 而 4S 满电只有 16.8V → 一解锁就缓降。
    """
    if v is None or v <= 0:
        return None
    return 6 if v >= C.CELL_V_SPLIT else 4

# abort 分级
SEV_FATAL = "FATAL"   # 空中停桨
SEV_LAND  = "LAND"    # 冻结 → 缓降 → AUTO.LAND
_MAV_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
               3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}


def statustext_cb(msg):
    try:
        with F.fcu_msgs_lock:
            F.fcu_msgs.append((rospy.Time.now().to_sec(), msg.severity, msg.text))
        # severity: 0 EMERGENCY … 3 ERROR, 4 WARNING, 6 INFO, 7 DEBUG
        if msg.severity <= 4:
            rospy.logwarn(f"[FCU] {msg.text}")
        elif msg.severity <= 6:
            rospy.loginfo(f"[FCU] {msg.text}")
    except Exception:
        pass


def fcu_texts_since(t0):
    with F.fcu_msgs_lock:
        return [txt for (ts, _sev, txt) in F.fcu_msgs if ts >= t0]


def allowed_modes():
    """脚本自己切过去的模式。已解锁时飞控处于这之外的模式 = 被接管。"""
    return ("OFFBOARD", "AUTO.LAND") if F.voluntary_land[0] else ("OFFBOARD",)


def tilt_limit():
    """→ (停桨倾角 °, 去抖 s)。冲刺 / 制动 / 过环悬停用 GATE_DASH_TILT_DEG, 其余 MAX_TILT_DEG。"""
    if F.dash_tilt_on[0]:
        return C.GATE_DASH_TILT_DEG, C.GATE_DASH_TILT_CONFIRM_S
    return C.MAX_TILT_DEG, C.FATAL_CONFIRM_S


def telem_put(section, d):
    try:
        d = dict(d)
        d["t"] = rospy.Time.now().to_sec()
        with F.telem_lock:
            F.telem_data[section] = d
    except Exception:
        pass


def telem_merge(section, **kw):
    try:
        with F.telem_lock:
            cur = dict(F.telem_data.get(section) or {})
            cur.update(kw)
            cur["t"] = rospy.Time.now().to_sec()
            F.telem_data[section] = cur
    except Exception:
        pass


def _f(x):
    """遥测里的数: nan/inf → None (JSON 不认 nan)。"""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def request_abort(reason):
    """FATAL —— 空中停桨。只给姿态失控 / MAVROS 断链 / 操作员按 X 用。"""
    if not F.abort_flag.is_set():
        rospy.logerr(f"*** ABORT [{SEV_FATAL}]: {reason} — 立即停桨 ***")
        F.abort_reason[0] = reason
        F.abort_flag.set()


def request_land(reason):
    """
    LAND —— 可恢复级。冻结当前 setpoint，走 descend + AUTO.LAND (就地, 不回原点)。
    """
    if not F.land_flag.is_set() and not F.abort_flag.is_set():
        rospy.logerr(f"*** ABORT [{SEV_LAND}]: {reason} — 转入降落 ***")
        F.land_reason[0] = reason
        F.land_flag.set()


def stop_requested():
    return F.abort_flag.is_set() or F.land_flag.is_set()


def check_abort():
    """FATAL 才抛异常中断飞行序列；LAND 由各 phase 自己提前 return。"""
    if F.abort_flag.is_set():
        raise RuntimeError(F.abort_reason[0])


def set_phase(name, horiz=None, vel=None):
    """
    切阶段。horiz/vel 非 None 时动态覆盖该阶段的包络 —— 用于 descend /
    auto_land: 它们的基础水平硬限比 return_home 小, 而 return 超时时飞机
    还没回到原点, 不放宽的话进 descend 第一个 monitor tick 就越界。
    """
    if horiz is None and vel is None:
        F.envelope_dyn[0] = None
    else:
        h0, v0 = C.PHASE_ENVELOPE.get(name, C.PHASE_ENVELOPE["_default"])
        F.envelope_dyn[0] = (h0 if horiz is None else horiz,
                           v0 if vel is None else vel)
    F.phase_ref[0] = name


def get_envelope(phase):
    dyn = F.envelope_dyn[0]
    if dyn is not None:
        return dyn
    return C.PHASE_ENVELOPE.get(phase, C.PHASE_ENVELOPE["_default"])
