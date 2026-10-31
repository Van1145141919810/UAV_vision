# -*- coding: utf-8 -*-
"""
vb09/flight/preflight.py —— 起飞前检查 (MAVROS / 位姿 / VINS / 视觉 / 静止漂移 / PX4 参数) + 参数摘要 + 地面预规划
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import os
import math
import statistics
import time

import rospy
import gate_plan as gp      # 同目录, 纯函数
from vb09.flight import config as C

from vb09.flight.ctx import F
from vb09.flight.ctx import _f, cells_from_voltage, telem_put
from vb09.flight.devices import _flush_stdin, tilt_deg_from_quat, yaw_from_quat


# =============================================================================
# ============================   参数摘要打印   ===============================
# =============================================================================
def gp_r_lim(axis):
    """机体中心最多能偏离环心多远 (只偏一个方向时): axis 0 = 左右, 1 = 上下。"""
    r = C.R_IN - C.AIM_MARGIN
    a, b = (C.H_HALF, C.W_HALF) if axis == 0 else (C.W_HALF, C.H_HALF)
    return max(0.0, math.sqrt(max(r * r - a * a, 0.0)) - b)


def print_config_summary():
    """arm 之前把生效参数念一遍。让程序自己把最终状态报出来, 比靠人读注释可靠。"""
    L = rospy.loginfo
    W = rospy.logwarn
    pv = gp.static_preview(gp.params_from_globals(vars(C)))
    L("=" * 64)
    L("生效参数摘要 (穿环)")
    L("-" * 64)
    L(f"  机体: 机头 {C.L_F:.2f} 机尾 {C.L_R:.2f} 半宽 {C.W_HALF:.2f} 半高 {C.H_HALF:.2f} m  → 机体中心离环心 "
      f"左右 ≤ {gp_r_lim(0):.2f} / 上下 ≤ {gp_r_lim(1):.2f} m")
    if C.NEEDLE_RIGHT or C.NEEDLE_UP or C.AIM_TRIM_RIGHT or C.AIM_TRIM_UP:
        W(f"  撞击点: 针尖在机体中心 右 {C.NEEDLE_RIGHT*100:+.0f} / 上 {C.NEEDLE_UP*100:+.0f} cm,  "
          f"经验修正 右 {C.AIM_TRIM_RIGHT*100:+.0f} / 上 {C.AIM_TRIM_UP*100:+.0f} cm  "
          f"→ 机体中心瞄在目标 右 {(C.AIM_TRIM_RIGHT-C.NEEDLE_RIGHT)*100:+.0f} / 上 {(C.AIM_TRIM_UP-C.NEEDLE_UP)*100:+.0f} cm")
    else:
        L("  撞击点: 机体中心对准目标 (NEEDLE_* / AIM_TRIM_* 都是 0)")
    L(f"  场地: 环后 {C.D_BACK:.2f}  起飞点后方 {C.D_REAR:.2f}  两侧 {C.D_SIDE:.2f} m  "
      f"水平范围 {C.GATE_XY_MAX:.2f} m")
    L(f"  冲刺: ≤{C.DASH_V:.2f} m/s (最低降到 {C.DASH_V_MIN:.2f})  加 {C.DASH_ACC:.1f} / 减 {C.BRAKE_ACC:.1f} m/s²  "
      f"T_LAG {C.T_LAG:.2f}s")
    vp = pv.get("v_plan", C.DASH_V)
    (W if vp < C.DASH_V - 1e-9 or pv.get("v_room") is None else L)(
        f"  正对进入时: {vp:.2f} m/s" + ("" if vp >= C.DASH_V - 1e-9 else
                                      f" (环后空间限速; 按 {C.DASH_V:.2f} 冲要 D_BACK ≥ {pv['d_back_need']:.2f} m)")
        + f"  助跑 {pv['d_align']:.2f} m")
    L(f"             过纸冲刺 {C.PASS_RUN:.2f} m (+滞后补偿 {pv.get('pass_lag', 0):.2f})  "
      f"刹车起点 s={pv.get('s_brake', float('nan')):+.2f}  停止 s={pv['overshoot']:+.2f}  "
      f"含超调 {pv.get('brake_over', 0):.2f} m 后机头离墙 {C.M_BACK + pv['tol']:.2f} m (要求 ≥ {C.M_BACK:.2f})")
    L("             场地不够时: " + {0: "不降速, 规划失败回手动段", 1: "降速, 冲刺前要按 Y 确认",
                                   2: "自动降速"}.get(int(C.GATE_SLOW_POLICY), "?"))
    if "v_hit" in pv:
        L(f"             撞纸 {pv['v_hit']:.2f} m/s (撕纸下限 {C.V_TEAR_MIN:.2f})  冲刺 {pv['t_dash']:.2f}s")
    L(f"  识别: 距离 [{C.GATE_D_MIN:.1f}, {C.GATE_D_MAX:.1f}] m  单帧σ 横<{C.GATE_SD_MAX_CM:.0f}/深<{C.GATE_SD_ALONG_MAX_CM:.0f}cm (环心观测放宽到 {C.GATE_SD_ALONG_PER_M_CM:.1f}cm×距离)  "
      f"锁定σ 横<{C.GATE_LOCK_SD_LAT_CM:.0f}/深<{C.GATE_LOCK_SD_ALONG_CM:.0f}cm  "
      f"锁定离散 横<{C.GATE_ALIGN_SPREAD_CM:.0f}/深<{C.GATE_ALIGN_SPREAD_ALONG_CM:.0f}cm  "
      f"{'必须有法向' if C.GATE_REQUIRE_NORMAL else '没有法向就按视线方向进入'}")
    L("-" * 64)
    if C.GATE_SHADOW:
        L("  ✓ 影子 GATE_SHADOW=True —— 识别/锁定/规划照做, 不摆位、不冲刺")
    else:
        W("  ✗ 影子 GATE_SHADOW=False —— !!! 锁定后会摆位、冲刺 !!!")
    if C.GATE_VIRTUAL_OFFSET > 0:
        W(f"  ★ 虚拟环: 环平面前移 {C.GATE_VIRTUAL_OFFSET:.2f} m, 停在纸前")
    if C.GATE_CONFIRM_DASH:
        L("  ✓ 冲刺前要按 G 确认")
    else:
        W("  ✗ GATE_CONFIRM_DASH=False —— 锁定后直接冲刺, 不等确认")
    if C.GATE_ALLOW_SIM_DRIVE:
        W("  ✗ GATE_ALLOW_SIM_DRIVE=True —— !!! sim 标记的识别也能驱动冲刺 !!!")
    L("-" * 64)
    for name in ("takeoff", "gate", "descend", "auto_land"):
        h, v = C.PHASE_ENVELOPE[name]
        L(f"  包络 {name:<10s} 水平<{h:.2f}m  速度<{v:.2f}m/s")
    L(f"  冲刺前 gate 段速度包络收紧到 {C.GATE_PRE_VEL_LIM:.2f} m/s")
    L(f"  高度硬顶 {C.MAX_HEIGHT:.2f}m  倾角停桨 {C.MAX_TILT_DEG:.0f}° (持续 {C.FATAL_CONFIRM_S:.2f}s)  "
      f"冲刺段 {C.GATE_DASH_TILT_DEG:.0f}° (持续 {C.GATE_DASH_TILT_CONFIRM_S:.2f}s)")
    if F.site_report:
        L("-" * 64)
        L(f"  现场档案: {F.site_report['src']}")
        for _ln in F.site_report["lines"]:
            L(f"    · {_ln}")
        for _w in F.site_report["warnings"]:
            W(f"    ⚠️ {_w}")
    else:
        W("  现场档案: flight_profile.py 不在 —— 用的是代码默认值, 且未经任何校验")
    L(f"  电池 {C.cell_count[0]}S  低压门槛 {C.CELL_MIN_FLY * C.cell_count[0]:.1f}V")
    L("=" * 64)


# =============================================================================
# ==============================   Preflight   ================================
# =============================================================================
PX4_CHECK_PARAMS = ("COM_OBL_ACT", "MPC_TILTMAX_AIR", "MPC_XY_VEL_MAX")
PX4_LOG_PARAMS = ("MPC_ACC_HOR_MAX", "MPC_JERK_AUTO", "MPC_XY_P", "MPC_XY_VEL_P_ACC")
PX4_PULL_TIMEOUT = 30.0      # s，等 mavros 参数缓存就绪


def _fmt_px4(v):
    return ("%.3f" % v).rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def _param_value(r):
    """mavros ParamGet 回复 → 数。整数参数在 integer, 浮点参数在 real (另一个是 0)。"""
    v = r.value
    return float(v.real) if float(v.real) != 0.0 else int(v.integer)


def read_px4_params(param_get, param_pull, names, optional=PX4_LOG_PARAMS):
    """
    读 PX4 参数。任何一个 names 读不到 → 先 param/pull (不强制; 缓存空就拉, 正在拉就等)
    再读一遍; 还读不到 → 抛 RuntimeError。optional 只记日志, 读不到不管。→ {名字: 值}
    """
    def get_all(ns):
        got, miss = {}, []
        for k in ns:
            try:
                r = param_get(param_id=k)
                ok = bool(r.success)
            except Exception:
                ok = False
            if ok:
                got[k] = _param_value(r)
            else:
                miss.append(k)
        return got, miss

    got, miss = get_all(names)
    if miss:
        rospy.logwarn(f"    参数缓存还没就绪 (读不到 {', '.join(miss)}) —— param/pull, "
                      f"最多等 {PX4_PULL_TIMEOUT:.0f}s ...")
        if param_pull is None:
            raise RuntimeError("读不到 %s, 也没有 /mavros/param/pull" % ", ".join(miss))
        t0 = time.time()
        n = 0
        try:
            r = param_pull(force_pull=False)
            n = int(getattr(r, "param_received", 0))
        except Exception as e:
            rospy.logwarn(f"    param/pull 出错: {e}")
        while time.time() - t0 < PX4_PULL_TIMEOUT:
            got2, miss = get_all(miss)
            got.update(got2)
            if not miss:
                break
            time.sleep(1.0)
        if miss:
            raise RuntimeError("pull 后仍读不到 %s (param_received=%d)" % (", ".join(miss), n))
        rospy.loginfo(f"    参数缓存就绪 (param_received={n}, 等了 {time.time() - t0:.1f}s)")
    if optional:
        extra, _ = get_all(optional)
        got.update(extra)
    return got


def preflight_plan(st, age):
    """
    [4/6] 的一部分 (2026-10-21): 地面上已经看到环 → 先按 DASH_V 规划一遍, 不用飞起来才发现放不下。
    放得下 → 打印一行; 放不下 → 说清楚卡在哪、要按设定速度冲需要改成多少, 让操作员选
      c = 照样起飞 (allow_slow 是"确认"/"是" 时空中会降到能飞的速度; "否" 时空中会规划失败)
      q = 退出去改档案
    看不到环 / 识别太旧 → 跳过 (返回 True)。返回 False = 操作员选了退出。
    """
    try:
        rw = st.get("ring_w") or st.get("target")
        if rw is None or age > 1.0:
            rospy.loginfo("    (地面看不到环 —— 冲刺速度要起飞后才知道放不放得下)")
            return True
        c = tuple(float(v) for v in rw[:3])
        aim = tuple(float(v) for v in st["aim"][:3]) if st.get("aim") else c
        n = tuple(float(v) for v in st["n"][:3]) if st.get("n") else None
        r = gp.preflight_check(aim, c, n, (F.INITIAL_X, F.INITIAL_Y, F.INITIAL_Z),
                               gp.params_from_globals(vars(C)), C.TAKEOFF_HEIGHT,
                               virtual_offset=C.GATE_VIRTUAL_OFFSET)
    except Exception as e:
        rospy.logwarn(f"    起飞前预规划出错 (跳过, 起飞后照常规划): {e}")
        return True
    dist = math.hypot(c[0] - F.INITIAL_X, c[1] - F.INITIAL_Y)
    telem_put("gate", {"seg": "preflight", "pre": {
        "ok_full": r["ok_full"], "v_set": _f(C.DASH_V), "v_room": _f(r["v_room"]), "why": r["why"],
        "xy_need": _f(r["xy_need"]), "d_back_need": _f(r["d_back_need"]), "ring_dist": _f(dist)}})
    if r["ok_full"]:
        rospy.loginfo(f"    ✓ 地面看到环 (水平 {dist:.1f} m): 按 {C.DASH_V:.2f} m/s 放得下 —— {r['summary']}")
        return True
    vr = r["v_room"]
    rospy.logerr("    " + "!" * 66)
    rospy.logerr(f"    ★ 地面看到环 (水平 {dist:.1f} m), 按设定 {C.DASH_V:.2f} m/s 放不下: {r['why']}")
    rospy.logerr("      场地最高能冲 " + (f"{vr:.2f} m/s" if vr is not None else "—— 降到最低速也不行"))
    fix = []
    if r["xy_need"] is not None and r["xy_need"] > C.GATE_XY_MAX:
        fix.append(f"xy_radius ≥ {r['xy_need']:.1f} (现在 {C.GATE_XY_MAX:.1f}), 或起飞点往环挪 {r['xy_need'] - C.GATE_XY_MAX:.1f} m")
    dbn = r["d_back_need"]
    if dbn > (C.GATE_VIRTUAL_OFFSET if C.GATE_VIRTUAL_OFFSET > 0 else C.D_BACK) + 1e-6:
        fix.append(f"{'虚拟环偏移' if C.GATE_VIRTUAL_OFFSET > 0 else 'd_back'} ≥ {dbn:.2f}")
    if vr is not None:
        fix.append(f"或 dash_v 改成 {vr:.2f}")
    rospy.logerr(f"      要按 {C.DASH_V:.2f} 冲: " + ";  ".join(fix))
    act = {0: "空中会规划失败、回手动段", 1: f"空中会降到约 {vr or 0:.2f} m/s, 冲刺前要按 Y 确认",
           2: f"空中会自动降到约 {vr or 0:.2f} m/s"}.get(int(C.GATE_SLOW_POLICY), "")
    rospy.logerr(f"      (地面识别位置可能差 10~20 cm, 起飞后会按空中识别重算。照样起飞的话: {act})")
    rospy.logerr("    " + "!" * 66)
    _flush_stdin()
    try:
        ans = input("    输入 c 回车 = 照样起飞,  其它 = 退出去改档案: ").strip().lower()
    except EOFError:
        ans = ""
    if ans != "c":
        rospy.logerr("    操作员选择退出 —— 改好档案再起飞")
        return False
    rospy.logwarn("    操作员选择照样起飞")
    return True


def preflight(param_get, set_mode, param_pull=None):
    pass
    rospy.loginfo("=" * 64)
    rospy.loginfo("Preflight")
    rospy.loginfo("=" * 64)

    rospy.loginfo("[1/6] MAVROS link...")
    t_end = rospy.Time.now() + rospy.Duration(5.0)
    state = None
    while rospy.Time.now() < t_end:
        state = F.tele.state
        if state and state.connected:
            break
        rospy.sleep(0.1)
    if state is None or not state.connected:
        rospy.logerr("    not connected")
        return False
    rospy.loginfo(f"    connected  mode={state.mode}  armed={state.armed}")
    if state.armed:
        rospy.logerr("    already armed")
        return False
    F.start_mode[0] = state.mode
    # 开局就是 OFFBOARD / AUTO.LAND = 上一次运行没正常收尾 (被强杀/断电/崩溃) 留下的,
    # PX4 里可能还挂着 offboard 丢失保护。地上未解锁, 切模式没有风险; 切不过去也不拦
    # (后面马上就切 OFFBOARD), 只记一笔。
    # AUTO.LAND 是正常降落后的收尾模式 (§5: 室内没遥控、没全局位置时 PX4 不肯切回 MANUAL /
    # LOITER, 脚本就把它留在 AUTO.LAND) —— 不算遗留, 不切。
    if state.mode == "AUTO.LAND":
        rospy.loginfo("    开局模式 AUTO.LAND (上次正常降落后留下的), 不用切")
    if state.mode == "OFFBOARD":
        rospy.logwarn(f"    ★ 开局模式 {state.mode} 是上一次运行遗留的 (上次没正常收尾) "
                      f"→ 先切 {C.PARK_MODE}")
        try:
            set_mode(custom_mode=C.PARK_MODE)
            t_end = rospy.Time.now() + rospy.Duration(1.5)
            while rospy.Time.now() < t_end:
                if F.tele.state and F.tele.state.mode == C.PARK_MODE:
                    break
                rospy.sleep(0.05)
            cur = F.tele.state.mode if F.tele.state else "?"
            if cur == C.PARK_MODE:
                rospy.loginfo(f"    已切到 {C.PARK_MODE}")
            else:
                rospy.logwarn(f"    没切成 (当前 {cur}) —— 不影响起飞, 后面会切 OFFBOARD")
        except Exception as e:
            rospy.logwarn(f"    切 {C.PARK_MODE} 异常 (不影响起飞): {e}")

    # 电池串数自动识别 (见 CELL_V_SPLIT)。写在 [1/6] 里, 不打乱 [n/6] 编号。
    t_end = rospy.Time.now() + rospy.Duration(3.0)
    bv = 0.0
    while rospy.Time.now() < t_end:
        _b = F.tele.snap()[4]                 # snap() 第 5 个 = battery
        if _b is not None and _b.voltage > 0:
            bv = _b.voltage
            break
        rospy.sleep(0.1)
    _n = cells_from_voltage(bv)
    if _n is None:
        rospy.logwarn(f"    ★ 读不到电池电压 → 串数按默认 {C.N_CELLS}S, "
                      f"低压门槛 {C.CELL_MIN_FLY * C.N_CELLS:.1f}V")
        rospy.logwarn("      ★ 若这架不是 4S 电池, 低压保护会失效, 请手动确认 ★")
    else:
        C.cell_count[0] = _n
        rospy.loginfo(f"    电池 {bv:.2f}V → 判为 {C.cell_count[0]}S "
                      f"(≈{bv / C.cell_count[0]:.2f}V/芯)  低压门槛 "
                      f"{C.CELL_MIN_FLY * C.cell_count[0]:.1f}V")

    rospy.loginfo("[2/6] VIO pose (/mavros/local_position/pose)...")
    t_end = rospy.Time.now() + rospy.Duration(C.POSE_WAIT_TIMEOUT)
    while rospy.Time.now() < t_end and F.tele.pose is None:
        rospy.sleep(0.1)
    if F.tele.pose is None:
        rospy.logerr("    no pose. is VINS running?")
        return False
    rospy.loginfo("    pose received")

    rospy.loginfo("[3/6] VINS raw odometry (/vins_estimator/odometry)...")
    t_end = rospy.Time.now() + rospy.Duration(3.0)
    while rospy.Time.now() < t_end and F.tele.vins_odom is None:
        rospy.sleep(0.1)
    if F.tele.vins_odom is None:
        rospy.logwarn("    NO vins_estimator/odometry. 诊断日志不完整，飞行可继续。")
        rospy.logwarn("    检查: rostopic list | grep vins")
    else:
        rospy.loginfo("    vins odometry received")

    # 先确定 origin —— 目标 z 门限需要 INITIAL_Z
    p0 = F.tele.pose.pose.position
    F.INITIAL_X, F.INITIAL_Y, F.INITIAL_Z = p0.x, p0.y, p0.z
    F.INITIAL_YAW = yaw_from_quat(F.tele.pose.pose.orientation)

    # ---------------- 图传模式 + 识别门禁 ----------------
    rospy.loginfo(f"[4/6] 图传模式 ({C.CAM_STATUS_TOPIC}) + 环识别 ({C.GATE_STATUS_TOPIC})...")
    t_end = rospy.Time.now() + rospy.Duration(3.0)
    while rospy.Time.now() < t_end and F.gate_in.cam_mode() is None:
        rospy.sleep(0.1)
    mode = F.gate_in.cam_mode()
    if mode not in ("gate", "gate_gcs"):
        rospy.logerr(f"    图传模式 = {mode or '收不到 /vision_bridge/cam (桥没在跑?)'} —— 穿环要 "
                     f"gate_gcs (地面检测) 或 gate (机上检测)。拒绝起飞")
        rospy.logerr("    重起: python3 run_vision.py gate_gcs --gcs <地面站IP>")
        return False
    rospy.loginfo(f"    图传模式 {mode}")
    t_end = rospy.Time.now() + rospy.Duration(2.0)
    while rospy.Time.now() < t_end and F.gate_in.n_msgs == 0:
        rospy.sleep(0.1)
    st, age = F.gate_in.latest()
    if st is None:
        rospy.logwarn("    还没收到 /gate/status —— 起飞后再找环 (地面 gcs_gate.py 在跑吗?)")
    else:
        rospy.loginfo(f"    /gate/status: 阶段 {st.get('stage')}  距离 {st.get('dist')}  "
                      f"sim={st.get('sim')}  法向 {'有' if st.get('n') is not None else '没有'}  "
                      f"红点 {'有' if st.get('aim') is not None else '没有'}")
        # 桥里的环估计是旧的 (首飞问题 §2: 落地后滤波器只 predict 不 update, 年龄几百秒、
        # σ 上米还在发) → 拒绝。新版 vision_bridge 会自己重置; 老版要重启桥。
        if age > C.GATE_STATUS_STALE_S:
            rospy.logwarn(f"    ★ /gate/status 最新一条是 {age:.1f}s 前收到的 (> {C.GATE_STATUS_STALE_S:.1f}s) —— "
                          f"地面检测停了 / 桥卡住了? 起飞后找环会一直等")
        t_age = st.get("age_ms")
        if st.get("target") is not None and isinstance(t_age, (int, float)) and \
                t_age > C.GATE_PREFLIGHT_TARGET_AGE_MS:
            rospy.logerr(f"    桥里的环估计已经 {t_age / 1000:.1f}s 没更新 (σ {st.get('sd_cm')} cm) —— "
                         f"旧状态会带进这次飞行。重启桥 (run_vision.py) 或换新版 vision_bridge.py "
                         f"(会自动重置)。拒绝起飞")
            return False
        d0 = st.get("dist")
        if (st.get("target") is not None and isinstance(d0, (int, float))
                and age < 1.0 and d0 < C.D_ALIGN_MIN - 0.3):
            if C.GATE_MANUAL_START:
                rospy.logwarn(f"    ★ 起飞点离环只有 {d0:.2f} m —— 起飞后先在手动段往后退到 ≥ "
                              f"{C.D_ALIGN_MIN:.1f} m 再按 T")
            else:
                rospy.logerr(f"    起飞点离环只有 {d0:.2f} m (< {C.D_ALIGN_MIN - 0.3:.2f}) —— "
                             f"往后挪, 拒绝起飞")
                return False
        if st.get("sim") and not (C.GATE_SHADOW or C.GATE_ALLOW_SIM_DRIVE):
            rospy.logwarn("    ★ 识别结果标了 sim (地面 gcs_gate --mode 1 / 机上 _gate_sim:=true) —— "
                          "这样的帧不会用来冲刺, 起飞后会一直等不到有效识别。"
                          "要真飞: gate_gcs 地面用 --mode 2; gate (机上检测) 用 run_vision.py gate --drive")
        if st.get("n") is None and C.GATE_REQUIRE_NORMAL:
            rospy.logerr("    GATE_REQUIRE_NORMAL=True 但识别结果里没有法向 n —— 拒绝起飞")
            return False
        if not preflight_plan(st, age):
            return False

    rospy.loginfo(f"[5/6] Static drift ({C.STATIC_CHECK_S}s, "
                  f"xy_std<{C.STATIC_XY_STD_MAX*100:.0f}cm "
                  f"z_std<{C.STATIC_Z_STD_MAX*100:.0f}cm)...")
    xs, ys, zs, tilts = [], [], [], []
    t_end = rospy.Time.now() + rospy.Duration(C.STATIC_CHECK_S)
    while rospy.Time.now() < t_end:
        if F.tele.pose:
            p = F.tele.pose.pose.position
            xs.append(p.x); ys.append(p.y); zs.append(p.z)
            tilts.append(tilt_deg_from_quat(F.tele.pose.pose.orientation))
        rospy.sleep(0.05)
    if len(xs) < 10:
        rospy.logerr("    not enough samples")
        return False
    if max(tilts) > C.STATIC_TILT_MAX_DEG:
        rospy.logerr(f"    ground tilt {max(tilts):.1f}deg")
        return False
    xy_std = math.hypot(statistics.stdev(xs), statistics.stdev(ys))
    z_std = statistics.stdev(zs)
    rospy.loginfo(f"    xy_std={xy_std*100:.2f}cm  z_std={z_std*100:.2f}cm  "
                  f"(z peak-to-peak={((max(zs)-min(zs))*100):.1f}cm)")
    if xy_std > C.STATIC_XY_STD_MAX:
        rospy.logerr(f"    xy_std > {C.STATIC_XY_STD_MAX*100:.1f}cm")
        return False
    if z_std > C.STATIC_Z_STD_MAX:
        rospy.logerr(f"    z_std > {C.STATIC_Z_STD_MAX*100:.1f}cm — check VIO stability")
        return False

    rospy.loginfo("[6/6] PX4 参数 (COM_OBL_ACT / MPC_TILTMAX_AIR / MPC_XY_VEL_MAX)...")
    # 读不到 = 拒绝起飞 (首飞问题 §3)。mavros 连上飞控后要十几秒才把参数表拉完, 这之前
    # param/get 一律失败 ("PR: Unknown parameter to get")。以前 COM_OBL_ACT 没看 success,
    # 读失败时 value 是 0 → 被当成"0 = Position"放行。现在先 pull (等缓存就绪) 再读。
    try:
        vals = read_px4_params(param_get, param_pull, PX4_CHECK_PARAMS)
    except Exception as e:
        rospy.logerr(f"    ✗ PX4 参数读不到: {e} —— 拒绝起飞")
        rospy.logerr("      查: rosservice call /mavros/param/pull \"force_pull: true\"   (param_received 应为几百)")
        return False
    rospy.loginfo("    " + "  ".join(f"{k}={_fmt_px4(v)}" for k, v in vals.items()))
    val = int(vals["COM_OBL_ACT"])
    if val not in (0, 2):
        rospy.logerr(f"    COM_OBL_ACT={val}, want 0 (Position) or 2 (Land)")
        return False
    tmax = float(vals["MPC_TILTMAX_AIR"])
    lim = C.MAX_TILT_DEG - C.PX4_TILT_MARGIN_DEG
    if tmax > lim + 1e-6:
        rospy.logerr(f"    MPC_TILTMAX_AIR={tmax:.0f}° > {lim:.0f}°: 急刹时 PX4 自己就可能把飞机倾过"
                     f"脚本的停桨线 {C.MAX_TILT_DEG:.0f}° —— QGC 里改到 ≤ {lim:.0f}°, 拒绝起飞")
        return False
    vmax = float(vals["MPC_XY_VEL_MAX"])
    v_need = C.DASH_V
    if vmax < 1.1 * v_need:
        rospy.logerr(f"    MPC_XY_VEL_MAX={vmax:.2f} < 1.1×冲刺速度 {1.1 * v_need:.2f}: "
                     f"PX4 会把速度削掉 —— 改参数或降 dash_v, 拒绝起飞")
        return False
    rospy.loginfo(f"    COM_OBL_ACT={val} MPC_TILTMAX_AIR={tmax:.0f}° MPC_XY_VEL_MAX={vmax:.1f} OK")
    try:
        if F.log_dir:
            with open(os.path.join(F.log_dir, "site_config.txt"), "a", encoding="utf-8") as f:
                f.write("\n# PX4 参数 (预检读到)\n" +
                        "".join(f"{k} = {_fmt_px4(v)}\n" for k, v in vals.items()))
    except Exception:
        pass

    # origin 重取一次 (静态检查期间可能微动)
    p0 = F.tele.pose.pose.position
    F.INITIAL_X, F.INITIAL_Y, F.INITIAL_Z = p0.x, p0.y, p0.z
    F.INITIAL_YAW = yaw_from_quat(F.tele.pose.pose.orientation)
    rospy.loginfo(f"Origin ({F.INITIAL_X:+.3f}, {F.INITIAL_Y:+.3f}, {F.INITIAL_Z:+.3f})  "
                  f"yaw={math.degrees(F.INITIAL_YAW):+.1f}deg")
    return True
