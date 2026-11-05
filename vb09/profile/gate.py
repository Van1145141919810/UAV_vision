# -*- coding: utf-8 -*-
"""
vb09/profile/gate.py —— 现场档案的穿环部分: 穿环旋钮 / 旋钮→参数 / 校验 / 派生量 / 打印
===============================================================================
2026-10-30 第 5 步从 flight_profile.py 原样搬来; 通用部分 (表结构、_fmt、SCENE_TABLE …) 用 _c. 引 core。
★ core 在文件**末尾**才 import: core 导入时要用这里的 GATE_KNOBS, 两边谁先被 import 都能成立。
"""
import json
import math
import os
import re
import sys



GATE_KNOBS = [
    # ---------------- 穿环 (stage12) ----------------
    dict(key="dash_v", tasks=["gate"], type="float", unit="m/s", label="冲刺速度", group="穿环",
         min=0.8, max=4.0, warn_hi=(3.0, "超过 3 m/s: 先在 T3 虚拟环逐级验证过再用"),
         help="冲刺巡航速度上限; 场地放不下时起飞后自动降速 (每次 0.3, 最低 1.2)"),
    dict(key="d_back", tasks=["gate"], type="float", unit="m", label="环后空间", group="穿环场地",
         min=1.0, max=50.0,
         help="纸面 → 环后最近障碍 (墙) 的距离, 卷尺量。冲刺要过纸后再冲一段才刹车, 需要 ≈ "
              "1.3 + 0.2v + v²/5 m (2.5 m/s 约 3.05, 2.0 约 2.5, 1.3 约 1.9); 不够会自动降速"),
    dict(key="back_margin", tasks=["gate"], type="float", unit="m", label="环后墙余量", group="穿环场地",
         min=0.1, max=2.0,
         warn_lo=(0.2, "低于 20cm: 纸面深度估计误差 (锁定时最大约 10cm) 和刹车超调都要从这里出"),
         help="冲刺刹停后机头离环后墙至少多远 (默认 0.3)。只管环后这一面; 两侧 / 起飞点后方的余量不变"),
    dict(key="d_rear", tasks=["gate"], type="float", unit="m", label="起飞点后方空地", group="穿环场地",
         min=0.3, max=50.0,
         help="起飞点往后 (背离环) 还有多少空地。起跑点可能在起飞点后面"),
    dict(key="d_side", tasks=["gate"], type="float", unit="m", label="两侧空间", group="穿环场地",
         min=0.6, max=50.0,
         help="环心 → 左右两侧最近障碍的距离 (取小的那边)"),
    dict(key="l_f", tasks=["gate"], type="float", unit="m", label="机头长", group="机体",
         min=0.05, max=1.0, help="机体中心 → 机头最前端 (含护桨, 不含针), 实测"),
    dict(key="l_r", tasks=["gate"], type="float", unit="m", label="机尾长", group="机体",
         min=0.05, max=1.0, help="机体中心 → 机尾最后端 (含护桨), 实测"),
    dict(key="w_half", tasks=["gate"], type="float", unit="m", label="半宽", group="机体",
         min=0.05, max=0.55, help="机体中心 → 最外侧 (含护桨), 实测。决定瞄点能偏离环心多远"),
    dict(key="mode", tasks=["gate"], type="choice", unit="", label="任务模式", group="穿环",
         choices=["冲刺", "影子", "停靠"],
         help="冲刺 = 锁定后摆位冲刺穿环; 影子 = 识别 / 锁定 / 规划照做, 不摆位不冲刺, 计划发网页; "
              "停靠 = 识别后飞到目标前方 (停靠距离) 停住对正, 悬停 (停靠悬停) 记录检测, 不冲刺 —— 测检测稳定性用 "
              "(不填 = 冲刺; 旧写法 set.GATE_SHADOW = True 等同于 影子)"),
    dict(key="target", tasks=["gate"], type="choice", unit="", label="目标类型", group="穿环",
         choices=["环", "红点"],
         help="飞行脚本按哪种目标判定识别结果 (→ TARGET_KIND, vb09/perception/registry.py)。"
              "桥 (run_vision.py --target) 要用同一种, 不一致会被强制 SIM。不填 = 环"),
    dict(key="dock_dist", tasks=["gate"], type="float", unit="m", label="停靠距离", group="穿环",
         min=1.8, max=8.0,
         help="mode = 停靠: 停靠点离目标多远 (水平, 沿环法向; 没有法向沿视线)。不填 = max(3.0, 最小助跑)"),
    dict(key="dock_hold", tasks=["gate"], type="float", unit="s", label="停靠悬停", group="穿环",
         min=3.0, max=300.0,
         help="mode = 停靠: 在停靠点悬停多久 (默认 20), 期间 gate.csv 照常记录; 结束打印停靠误差和 aim 抖动"),
    dict(key="vscan", tasks=["gate"], type="float", unit="m", label="上下扫描", group="穿环",
         min=0.0, max=0.6,
         help="mode = 停靠: 到停靠点后先降、再升这么多、回停靠点 (只改高度, 不横移), 给识别换视角定法向 "
              "(红点必须: 单个视角法向有两个解)。3 m 处 0.3 ≈ 视角跨度 11°, 至少要 10°。不填 / 0 = 不扫"),
    dict(key="allow_slow", tasks=["gate"], type="choice", unit="", label="场地不够时降速", group="穿环",
         choices=["否", "确认", "是"],
         help="场地放不下设定的冲刺速度时: 否 = 规划失败回手动段 (说明卡在哪、最高能冲多快); "
              "确认 (默认) = 降到能飞的速度, 冲刺前必须按 Y 确认; 是 = 自动降速 (会标出来)"),
    dict(key="brake_over", tasks=["gate"], adv=True, type="float", unit="s", label="刹车超调系数",
         group="穿环", min=0.0, max=1.0,
         help="刹停实际比规划多冲 ≈ 系数 × 冲刺速度 (默认 0.3: 10-21 实测 2.0 m/s 多冲 0.59 m)。"
              "离墙和水平范围按含超调的停止点查; 轨迹本身不变"),
    dict(key="pass_run", tasks=["gate"], type="float", unit="m", label="过纸冲刺距离", group="穿环",
         min=0.2, max=3.0,
         help="冲刺: 机头碰到纸后保持冲刺速度再往前冲多远才开始刹车 (默认 1.0), 让机身整个穿过纸、不被纸缠住。"
              "控制滞后已自动补偿; 越大需要的环后空间越大"),
    dict(key="run_max", tasks=["gate"], type="float", unit="m", label="最远起跑距离", group="穿环",
         min=0.0, max=8.0, warn_hi=(5.0, "超过 5 m: 远处锁定的横向 σ 和盲冲中的 VIO 漂移都会变大"),
         help="开始规划时飞机离环在 [最小助跑, 它] 之间就地起跑; 更远就前进到它, 更近就后退到最小助跑。"
              "0 = 总是先飞到最小助跑处 (约 3 m) 再起跑 —— 要从按 T 的位置直接起跑, 设成 ≥ 那里离环的距离"),
    dict(key="dash_acc", tasks=["gate"], adv=True, type="float", unit="m/s²", label="冲刺加速度",
         group="穿环", min=0.5, max=4.0, help="加速段加速度 (2.0 ≈ 前倾 11.5°)"),
    dict(key="brake_acc", tasks=["gate"], adv=True, type="float", unit="m/s²", label="制动减速度",
         group="穿环", min=0.5, max=4.0, help="刹车段减速度 (2.5 ≈ 后仰 14.3°)"),
    dict(key="t_lag", tasks=["gate"], adv=True, type="float", unit="s", label="控制滞后",
         group="穿环", min=0.0, max=1.0, help="只用于估计撞纸速度; T2 实测后替换"),
    dict(key="e_along", tasks=["gate"], adv=True, type="float", unit="m", label="沿法向误差预算",
         group="穿环", min=0.05, max=1.0, help="纸面位置估计最多差多少; 刹停窗口半宽必须 ≥ 它"),
    dict(key="v_tear_min", tasks=["gate"], adv=True, type="float", unit="m/s", label="撕纸最低速度",
         group="穿环", min=0.3, max=4.0, help="撕纸测试测到的最低速度 × 2; 最坏撞纸速度低于它 → 降不了速就放弃"),
    dict(key="entry_ang_max", tasks=["gate"], adv=True, type="float", unit="°", label="进入偏角上限",
         group="穿环", min=0.0, max=40.0, help="进入方向和环法向的最大夹角"),
    dict(key="d_straight", tasks=["gate"], adv=True, type="float", unit="m", label="稳定直线段",
         group="穿环", min=0.0, max=5.0, help="加速完到纸面之间至少留多少匀速直线"),

]

# 任务模式旋钮 → C.MISSION_MODE (vb09/flight/modes.py 的键)
MODE_NAMES = {"冲刺": "dash", "影子": "shadow", "停靠": "dock"}
# 目标类型旋钮 → C.TARGET_KIND (registry 的键)。2026-11-05: 以前档案里设不了 (set. 只认数值 / 开关)
TARGET_NAMES = {"环": "ring", "红点": "reddot"}

# 穿环旋钮 → 参数 (一对一)
GATE_KNOB_PARAM = {
    "dash_v": "DASH_V", "d_back": "D_BACK", "d_rear": "D_REAR", "d_side": "D_SIDE",
    "l_f": "L_F", "l_r": "L_R", "w_half": "W_HALF", "dash_acc": "DASH_ACC",
    "brake_acc": "BRAKE_ACC", "t_lag": "T_LAG", "e_along": "E_ALONG",
    "v_tear_min": "V_TEAR_MIN", "entry_ang_max": "ENTRY_ANG_MAX", "d_straight": "D_STRAIGHT",
    "run_max": "D_RUN_MAX",
    "pass_run": "PASS_RUN", "back_margin": "M_BACK", "brake_over": "BRAKE_OVER_T",
    "dock_dist": "GATE_DOCK_D_STOP", "dock_hold": "GATE_DOCK_HOLD_S", "vscan": "GATE_VSCAN_DZ",
}

# 穿环 (stage12) 才有的安全项, 只在 _check_gate 里查
GATE_SAFETY_RULES = [
    ("GATE_DASH_TILT_DEG", "range", (30.0, 50.0), "FATAL",
     "冲刺段 (按 G 后到刹停悬停) 的停桨倾角。急刹 + 撞纸有短时大倾角; 低于 30° 比平时还紧, "
     "高于 50° 已接近翻机才停"),
    ("GATE_DASH_TILT_CONFIRM_S", "range", (0.15, 0.50), "FATAL", "冲刺段停桨去抖时间"),
]


# =============================================================================
# 旋钮 → 参数覆盖
# =============================================================================
def _build_gate_overrides(knobs, view):
    """穿环的旋钮 → 参数。"""
    ov, rep, warns = {}, [], []

    def cur(k):
        if k in ov:
            return ov[k]
        if k in view:
            return view[k]
        return _c._STANDALONE_DEFAULTS.get(k)

    tk = knobs.get("takeoff_h")
    if tk is not None:
        ov["TAKEOFF_HEIGHT"] = tk
        ov["TAKEOFF_TIMEOUT"] = max(10.0, math.ceil(tk / 0.5) + 5.0)
        rep.append("起飞高度 %.2f m (超时 %.0f s)" % (tk, ov["TAKEOFF_TIMEOUT"]))

    zlo, zhi = knobs.get("z_low"), knobs.get("z_high")
    if zlo is not None and zhi is not None:
        ov["GATE_Z_MIN_DZ"], ov["GATE_Z_MAX_DZ"] = zlo, zhi
        top = max(zhi, cur("TAKEOFF_HEIGHT"))
        ov["MAX_HEIGHT"] = top + 1.0
        rep.append("瞄点高度带 [%.2f, %.2f] m / 安全硬顶 %.2f m" % (zlo, zhi, top + 1.0))
    elif tk is not None and tk + 1.0 > cur("MAX_HEIGHT"):
        ov["MAX_HEIGHT"] = tk + 1.0
        rep.append("起飞高度高于原硬顶 → 抬高 MAX_HEIGHT")
    if "MAX_HEIGHT" in ov:
        dsp = cur("DESCEND_SPEED")
        ov["DESCEND_TIMEOUT"] = max(45.0, math.ceil(ov["MAX_HEIGHT"] / dsp * 2) + 5.0)

    r = knobs.get("xy_radius")
    if r is not None:
        ov["GATE_XY_MAX"] = r
        margin = min(2.0, max(0.5, 0.15 * r))
        ov["ENVELOPE_MARGIN"] = margin
        rep.append("水平范围 %.2f m (起跑点/制动终点都要在内) / 硬围栏 %.2f m" % (r, r + margin))

    given = [(k, GATE_KNOB_PARAM[k]) for k in GATE_KNOB_PARAM if knobs.get(k) is not None]
    for k, pname in given:
        ov[pname] = knobs[k]
    if given:
        rep.append("穿环: " + "  ".join("%s=%s" % (pn, _c._fmt(knobs[k])) for k, pn in given))
    if knobs.get("allow_slow") is not None:
        ov["GATE_SLOW_POLICY"] = {"否": 0, "确认": 1, "是": 2}[knobs["allow_slow"]]
        rep.append("场地不够时降速: %s" % knobs["allow_slow"])
    tg = knobs.get("target")
    if tg is not None:
        ov["TARGET_KIND"] = TARGET_NAMES[tg]
        rep.append("目标类型: %s (TARGET_KIND = %s; 桥要 run_vision.py --target %s)"
                   % (tg, TARGET_NAMES[tg], TARGET_NAMES[tg]))
    # ★2026-10-30 任务模式。影子同时置 GATE_SHADOW (识别接受 sim 帧 / 摘要 / 网页开关都按它)
    md = knobs.get("mode")
    if md is not None:
        ov["MISSION_MODE"] = MODE_NAMES[md]
        if md == "影子":
            ov["GATE_SHADOW"] = True
        rep.append("任务模式: %s" % md)
        if md == "冲刺" and view.get("GATE_SHADOW"):
            warns.append("mode = 冲刺, 但 set.GATE_SHADOW = True —— 两个说法矛盾, 按 影子 飞 (不冲刺)。"
                         "要冲刺就删掉 GATE_SHADOW 那一行")
        if md == "停靠" and view.get("GATE_SHADOW"):
            warns.append("mode = 停靠, 但 set.GATE_SHADOW = True —— 两个说法矛盾, 按 影子 飞 (不停靠)。"
                         "要停靠就删掉 GATE_SHADOW 那一行")
    elif view.get("GATE_SHADOW"):
        warns.append("GATE_SHADOW = True 是旧写法, 现在等同于 mode = 影子 (档案里建议改写成 mode = 影子)")
    if "DASH_V" in ov and ov["DASH_V"] < cur("DASH_V_MIN"):
        ov["DASH_V_MIN"] = ov["DASH_V"]          # 冲刺速度本身比降速下限还低: 不降速
    miss = [k for k in ("d_back", "d_rear", "d_side", "l_f", "l_r", "w_half")
            if knobs.get(k) is None]
    if miss:
        warns.append("档案没给 %s —— 用的是代码默认值 (%s)。这些是**现场/实测量**, 请在档案里填写"
                     % ("、".join(_c.KNOB_INDEX[k]["label"] for k in miss),
                        ", ".join("%s=%s" % (GATE_KNOB_PARAM[k], _c._fmt(cur(GATE_KNOB_PARAM[k])))
                                  for k in miss)))

    sc = knobs.get("scene")
    if sc in _c.SCENE_TABLE:
        for k, v in _c.SCENE_TABLE[sc].items():
            ov[k] = v
        rep.append("场景 %s (%s)" % (sc, ", ".join(
            "%s=%g" % kv for kv in sorted(_c.SCENE_TABLE[sc].items()))))
    return ov, rep, warns


def _gate_preview(g):
    """穿环的静态派生量 (gate_plan.static_preview)。gate_plan.py 不在 → 抛 ImportError。"""
    import gate_plan
    return gate_plan.static_preview(gate_plan.params_from_globals(g)), gate_plan


def _check_gate(g):
    """穿环 (stage12)。场地/机体在起飞后按环的实际位置再算一次 (stage12 的 PLAN 段),
    这里只查"正对进入"时就放不下的情况, 和与飞控/安全线的配合。"""
    E, W = [], []
    get = g.get
    _c._check_safety(g, E)
    _c._check_safety(g, E, GATE_SAFETY_RULES)
    if get("GATE_DASH_TILT_DEG", 0) < get("MAX_TILT_DEG", 0):
        E.append("冲刺段停桨倾角 %.0f° 比平时 %.0f° 还小" % (g["GATE_DASH_TILT_DEG"], g["MAX_TILT_DEG"]))

    # ---------------- 高度 ----------------
    tk, top = g["TAKEOFF_HEIGHT"], g["MAX_HEIGHT"]
    zlo, zhi = g["GATE_Z_MIN_DZ"], g["GATE_Z_MAX_DZ"]
    if tk < 0.5:
        E.append("起飞高度 %.2f < 0.5：离地判定不可靠" % tk)
    if tk > top - 0.5 + 1e-9:
        E.append("起飞高度 %.2f 离安全硬顶 %.2f 太近(<0.5m)" % (tk, top))
    if g["TAKEOFF_TIMEOUT"] < tk / 0.5 + 3.0:
        W.append("起飞超时 %.0fs 偏短(升 %.1fm 按 0.5m/s 估要 %.0fs)"
                 % (g["TAKEOFF_TIMEOUT"], tk, tk / 0.5))
    if zlo < 0.5 - 1e-9:
        E.append("瞄点高度下限 %.2f < 0.50：地效 + 桨风, 冲刺高度太低" % zlo)
    if not zlo < zhi:
        E.append("瞄点高度带 [%.2f, %.2f] 为空" % (zlo, zhi))
    if zhi > top - 0.5 + 1e-9:
        E.append("瞄点高度上限 %.2f 离安全硬顶 %.2f 太近(<0.5m)" % (zhi, top))

    # ---------------- 水平 ----------------
    hard = g["GATE_XY_MAX"] + g["ENVELOPE_MARGIN"]
    base = g["BASE_DRIFT_LIM"]
    if base < 0.30:
        E.append("BASE_DRIFT_LIM=%.2f < 0.30：正常悬停漂移就会触发缓降" % base)
    if base + 0.40 > hard + 1e-9:
        E.append("起降阶段水平限制(最大 %.2f m)比水平硬围栏 %.2f m 还大" % (base + 0.40, hard))

    # ---------------- 冲刺 ----------------
    v, vmin = g["DASH_V"], g["DASH_V_MIN"]
    if vmin > v + 1e-9:
        E.append("DASH_V_MIN=%.2f > DASH_V=%.2f" % (vmin, v))
    if g["V_TEAR_MIN"] > vmin + 1e-9:
        W.append("撕纸最低速度 %.2f > 降速下限 %.2f: 场地逼得降速时会先因为撞纸速度不够而放弃"
                 % (g["V_TEAR_MIN"], vmin))
    if g.get("M_BACK", 0.3) < 0.1:
        E.append("环后墙余量 M_BACK=%.2f m < 0.1 m" % g["M_BACK"])
    tilt_plan = math.degrees(math.atan(max(g["DASH_ACC"], g["BRAKE_ACC"]) / 9.81))
    if tilt_plan > g["MAX_TILT_DEG"] - 10.0:
        E.append("规划倾角 %.1f° (加/减速度 %.1f m/s²) 离停桨线 %.0f° 不到 10°"
                 % (tilt_plan, max(g["DASH_ACC"], g["BRAKE_ACC"]), g["MAX_TILT_DEG"]))
    if get("D_RUN_MAX", 0) > get("GATE_D_MAX", 99) - 1.0:
        W.append("最远起跑距离 %.1f m 离识别上限 %.1f m 不到 1 m: 起跑点上锁定会很难收敛"
                 % (g["D_RUN_MAX"], g["GATE_D_MAX"]))
    if g["D_ALIGN_MIN"] < 2.4:
        W.append("D_ALIGN_MIN=%.2f < 2.4: 起跑点离环太近时识别相机看不到整个环, 锁定会用残缺的环"
                 % g["D_ALIGN_MIN"])
    try:
        pv, gp = _gate_preview(g)
    except ImportError:
        E.append("找不到 gate_plan.py —— 放到飞行脚本同目录")
        pv, gp = None, None
    if pv is not None:
        if gp.r_safe(gp.params_from_globals(g)) <= 0.0:
            E.append("半宽 %.2f m: 机体放不进内环 (内半径 %.2f, 留 %.2f 余量)"
                     % (g["W_HALF"], g["R_IN"], g["AIM_MARGIN"]))
        # ★2026-10-21 刹车点 = 机头过纸 PASS_RUN 之后 (+滞后补偿)。环后空间决定能冲多快
        gpp = gp.params_from_globals(g)
        vmin_need = gp.dash_stop(g["DASH_V_MIN"], gpp)["d_back_need"]
        pol = int(get("GATE_SLOW_POLICY", 1))
        slow_ok = pol >= 1
        if pv.get("v_room") is None:
            E.append("环后空间 D_BACK=%.2f m 不够: 过纸冲刺 %.2f m + 刹车超调, 降到 %.2f m/s 也要 ≥ %.2f m "
                     "—— 加大环后空间, 或减小 pass_run (过纸冲刺) / back_margin (环后墙余量)"
                     % (g["D_BACK"], g["PASS_RUN"], g["DASH_V_MIN"], vmin_need))
        elif pv["v_room"] < g["DASH_V"] - 1e-9:
            msg = ("环后空间 D_BACK=%.2f m 只够冲 %.2f m/s, 设定 %.2f。按设定速度冲需要 D_BACK ≥ %.2f m"
                   % (g["D_BACK"], pv["v_room"], g["DASH_V"], pv["d_back_need"]))
            if pol == 2:
                W.append(msg + " (allow_slow = 是: 起飞后会自动降到 %.2f)" % pv["v_room"])
            elif pol == 1:
                W.append(msg + " (allow_slow = 确认: 起飞后降到 %.2f, 冲刺前要按 Y 确认)" % pv["v_room"])
            else:
                E.append(msg + " —— 不允许降速 (allow_slow = 否): 把 dash_v 改成 %.2f, "
                               "或加大环后空间, 或 allow_slow = 确认" % pv["v_room"])
        ps = gp.pass_short(gpp)
        if ps:
            W.append(ps)
        vo = get("GATE_VIRTUAL_OFFSET") or 0
        if vo > 0:
            vv = gp.dash_v_room(gpp, d_back=vo)
            if vv is None:
                E.append("虚拟环偏移 %.2f m (真纸面当墙) 放不下冲刺: 降到 %.2f m/s 也要偏移 ≥ %.2f m"
                         % (vo, g["DASH_V_MIN"], vmin_need))
            else:
                if vv >= g["DASH_V"] - 1e-9:
                    W.append("虚拟环偏移 %.2f m 够按 %.2f m/s 冲" % (vo, g["DASH_V"]))
                elif slow_ok:
                    W.append("虚拟环偏移 %.2f m 只够冲 %.2f m/s (会降速%s)"
                             % (vo, vv, ", 冲刺前按 Y 确认" if pol == 1 else ""))
                else:
                    E.append("虚拟环偏移 %.2f m 只够冲 %.2f m/s, 设定 %.2f 且不允许自动降速 —— "
                             "加大偏移, 或 dash_v 改成 %.2f" % (vo, vv, g["DASH_V"], vv))
        if pv.get("error"):
            E.append("%s轨迹放不下: %s" % ("冲刺", pv["error"]))
        elif pv["v_hit_worst"] < g["V_TEAR_MIN"] - 1e-3:     # 滞后模型渐近到 v, 留数值余量
            W.append("正对进入时最坏撞纸速度 %.2f < 撕纸下限 %.2f m/s —— 起飞后的规划会拒绝"
                     % (pv["v_hit_worst"], g["V_TEAR_MIN"]))
    pe = get("PHASE_ENVELOPE")
    if isinstance(pe, dict) and "gate" in pe:
        if pe["gate"][1] < 1.7 * v:
            E.append("gate 速度包络 %.2f m/s < 1.7×%s速度 %.2f m/s：一%s就会触发缓降"
                     % (pe["gate"][1], "冲刺", 1.7 * v, "冲刺"))
    elif isinstance(pe, dict):
        E.append("PHASE_ENVELOPE 里没有 gate —— 冲刺会落到默认包络 (1 m/s) 一冲就缓降")

    # ---------------- 目标类型 (2026-11-01 接线) ----------------
    tk = get("TARGET_KIND", "ring")
    try:
        from vb09.perception import registry
        if tk not in registry.JUDGES:
            E.append("目标类型 TARGET_KIND=%r 没有判定插件 (vb09/perception/registry.py JUDGES 里只有: %s)"
                     % (tk, ", ".join(registry.kinds(registry.JUDGES))))
    except ImportError:
        pass

    # ---------------- 开关 ----------------
    if get("GATE_ALLOW_SIM_DRIVE"):
        W.append("GATE_ALLOW_SIM_DRIVE=True —— 标了 sim 的识别结果也能驱动冲刺！只在台架测试时用")
    if get("GATE_SHADOW"):
        W.append("GATE_SHADOW=True: 影子模式 —— 识别、锁定、规划照做, 不摆位、不冲刺; 做完回手动段 (可换位置再 T), L 降落")
    elif get("MISSION_MODE") == "dock":
        dd = get("GATE_DOCK_D_STOP") or 0.0
        d_eff = dd if dd > 0 else max(3.0, get("D_ALIGN_MIN") or 0.0)
        W.append("停靠模式 (mode = 停靠): 识别 → 飞到目标前 %.2f m%s 停住对正 → 悬停 %.0f s 记录检测 → 回手动段; "
                 "不冲刺 (冲刺相关的检查照做, 但这一趟用不到)"
                 % (d_eff, "" if dd > 0 else " (自动)", get("GATE_DOCK_HOLD_S") or 0.0))
    dd = get("GATE_DOCK_D_STOP") or 0.0
    if dd < 0:
        E.append("停靠距离 GATE_DOCK_D_STOP=%.2f < 0 (0 = 自动)" % dd)
    if 0 < dd < -get("REPO_X_MAX", -1.5) + 0.3:
        E.append("停靠距离 %.2f m 太近: 摆位时要在目标前 ≥ %.1f m (REPO_X_MAX), 至少给 %.1f m"
                 % (dd, -get("REPO_X_MAX", -1.5), -get("REPO_X_MAX", -1.5) + 0.3))
    vs = get("GATE_VSCAN_DZ") or 0.0
    if vs < 0 or vs > 0.6:
        E.append("上下扫描 GATE_VSCAN_DZ=%.2f m 不在 [0, 0.6]" % vs)
    elif vs > 0:
        if get("MISSION_MODE") != "dock" or get("GATE_SHADOW"):
            W.append("上下扫描 vscan=%.2f m 只在停靠模式里做, 这一趟用不到" % vs)
        else:
            th = get("TAKEOFF_HEIGHT") or 0.0
            lo, hi = get("TELEOP_DZ_MIN") or 0.0, (get("MAX_HEIGHT") or 0.0) - 0.5
            dd = get("GATE_DOCK_D_STOP") or 0.0
            d_eff = dd if dd > 0 else max(3.0, get("D_ALIGN_MIN") or 0.0)
            z_lo, z_hi = max(th - vs, lo), min(th + vs, hi)
            span = math.degrees(2.0 * math.atan2(max(z_hi - z_lo, 0.0) / 2.0, d_eff))
            W.append("停靠后上下扫描: 高度约 %.2f → %.2f m (起飞高度 %.2f ± %.2f%s), 离目标 %.2f m → 视角跨度约 %.1f° "
                     "(扫完拿到法向就按法向, 拿不到按视线)"
                     % (z_lo, z_hi, th, vs, ", 被高度带截了" if (z_lo, z_hi) != (th - vs, th + vs) else "",
                        d_eff, span))
            if span < (get("GATE_VSCAN_SPAN_DEG") or 10.0):
                W.append("上下扫描视角跨度约 %.1f° < %.0f°: 法向多半定不下来 —— 加大 vscan 或缩短停靠距离"
                         % (span, get("GATE_VSCAN_SPAN_DEG") or 10.0))
    if (get("GATE_VIRTUAL_OFFSET") or 0) > 0:
        W.append("虚拟环 GATE_VIRTUAL_OFFSET=%.2f m: 真冲刺, 停在纸前 (T3 测试)"
                 % g["GATE_VIRTUAL_OFFSET"])
    if not get("GATE_CONFIRM_DASH", True):
        W.append("GATE_CONFIRM_DASH=False: 锁定后不等按 G, 直接冲刺")

    # ---------------- 时间 / 频率 ----------------
    need_d = (top - g["DESCEND_Z_TARGET_DZ"]) / g["DESCEND_SPEED"] * 1.2 + 3
    if g["DESCEND_TIMEOUT"] < need_d:
        W.append("降落超时 %.0fs < 从硬顶降下来约需 %.0fs：超时会直接切 AUTO.LAND"
                 % (g["DESCEND_TIMEOUT"], need_d))
    if g["SP_PUB_HZ"] <= 2:
        E.append("SP_PUB_HZ=%s ≤ 2：PX4 会退出 OFFBOARD" % g["SP_PUB_HZ"])
    return E, W


def _derived_gate(g):
    """穿环的 derived (INTERFACE_gate.md §4)。停靠跟踪专用的字段一律不给 (null)。"""
    pe = g.get("PHASE_ENVELOPE") or {}
    try:
        pv, gp = _gate_preview(g)
    except ImportError:
        pv = {}
    gate = {"r_in": g["R_IN"], "r_out": g.get("R_OUT", 0.8), "dot_d": g.get("DOT_D", 0.11)}
    gate.update(pv)
    env_v = (pe.get("gate") or [None, None])[1]
    chain = [["DASH_V", g["DASH_V"]]]
    if env_v:
        chain.append(["gate 速度硬限 ÷1.7", env_v / 1.7])
    lo, hi = g["GATE_Z_MIN_DZ"], g["GATE_Z_MAX_DZ"]
    return {
        "task": "gate",
        "z_band": [lo, hi],
        "z_clamp": [lo, hi],
        "teleop_dz": None,
        "max_height": g["MAX_HEIGHT"],
        "takeoff_height": g["TAKEOFF_HEIGHT"],
        "xy_soft": g["GATE_XY_MAX"],
        "xy_hard": g["GATE_XY_MAX"] + g["ENVELOPE_MARGIN"],
        "base_drift": g["BASE_DRIFT_LIM"],
        "standoff_shell": None, "detect_range": None, "entry_dist": None,
        "approach_band": None, "leash": None,
        "phase_envelope": {k: list(v) for k, v in pe.items()},
        "timeouts": {k: g[k] for k in ("TAKEOFF_TIMEOUT", "DESCEND_TIMEOUT")
                     if k in g},
        "speeds": {k: g[k] for k in ("DASH_V", "DASH_ACC", "BRAKE_ACC", "GATE_REPO_V",
                                     "DESCEND_SPEED") if k in g},
        "safety": {p: g.get(p) for p, _, _, _, _ in _c.SAFETY_RULES + GATE_SAFETY_RULES},
        "switches": {k: g.get(k) for k in ("GATE_SHADOW", "GATE_ALLOW_SIM_DRIVE",
                                           "GATE_CONFIRM_DASH", "GATE_VIRTUAL_OFFSET",
                                           "GATE_REQUIRE_NORMAL", "GATE_MANUAL_START",
                                           "GATE_ABORT_TO_MANUAL",
                                           "GATE_SLOW_POLICY")},
        "site": {k: g.get(k) for k in ("D_BACK", "D_REAR", "D_SIDE", "L_F", "L_R", "W_HALF",
                                       "D_RUN_MAX", "PASS_RUN", "M_BACK")},
        "gate": gate,
        "speed_chain": chain,
    }


def _start_line(gt):
    """起跑点说明。run_max ≤ 最小助跑 (比如 0) 时不是"动态", 而是总会飞到离环最小助跑处。"""
    lo, hi = gt["d_align"], gt.get("d_run_max", gt["d_align"])
    if hi <= lo + 1e-6:
        return ("起跑点        固定: 按 T 后总会飞到离环 %.2f m 处起跑 (run_max ≤ 最小助跑; "
                "想就地起跑把 run_max 设成 ≥ 起飞点离环的距离)" % lo)
    return "起跑点        动态: 离环 [%.2f, %.2f] m 就地起跑, 近了后退 / 远了前进" % (lo, hi)


def _gate_lines(g, d):
    """穿环包络核对表 (dry-run 文本 / site_config.txt 共用)。"""
    gt = d.get("gate") or {}
    pe = d.get("phase_envelope") or {}
    L = []
    L.append("任务          穿环 (模式见下面 开关 / 档案 mode)")
    L.append("高度          瞄点带 [%.2f, %.2f] m   起飞 %.2f m   安全硬顶 %.2f m"
             % (d["z_band"][0], d["z_band"][1], d["takeoff_height"], d["max_height"]))
    L.append("水平          范围 %.2f m / 硬围栏 %.2f m / 起降漂移 %.2f m"
             % (d["xy_soft"], d["xy_hard"], d["base_drift"]))
    st = d.get("site") or {}
    L.append("场地          环后 %.2f m  起飞点后方 %.2f m  两侧 %.2f m"
             % (st.get("D_BACK"), st.get("D_REAR"), st.get("D_SIDE")))
    L.append("机体          机头 %.2f m  机尾 %.2f m  半宽 %.2f m  → 瞄点离环心 ≤ %.2f m"
             % (st.get("L_F"), st.get("L_R"), st.get("W_HALF"), gt.get("r_safe", float("nan"))))
    if "s_brake" in gt:
        vp = gt.get("v_plan", gt["dash_v"])
        L.append("冲刺(正对)    %.2f m/s%s  加 %.1f / 减 %.1f m/s²  助跑 %.2f m  用时 %.2f s"
                 % (vp, ("" if abs(vp - gt["dash_v"]) < 1e-9 else
                         " (环后空间限速, 设定 %.2f)" % gt["dash_v"]),
                    gt["dash_acc"], gt["brake_acc"], gt["d_align"], gt["t_dash"]))
        L.append("过纸冲刺      机头碰纸后再冲 %.2f m 才刹车 (滞后补偿 %.2f m)  刹车起点 s=%+.2f  停止 s=%+.2f"
                 % (gt["pass_run"], gt["pass_lag"], gt["s_brake"], gt["overshoot"]))
        L.append("环后墙        含刹车超调 %.2f m 后机头离墙 %.2f m (要求 ≥ %.2f, 再富余 %.2f)   按 %.2f m/s 冲要 D_BACK ≥ %.2f m"
                 % (gt.get("brake_over", 0.0), gt["back_margin"] + gt["tol"], gt["back_margin"], gt["tol"],
                    gt["dash_v"], gt["d_back_need"]))
        L.append("降速          %s" % {0: "否: 场地放不下设定速度时规划失败、回手动段",
                                       1: "确认: 场地放不下时降到能飞的速度, 冲刺前要按 Y",
                                       2: "是: 场地放不下时自动降速 (会告警)"}.get(gt.get("slow_policy", 1), "?")
                 + "   (起飞前 preflight 会用地面看到的环先算一遍)")
        L.append(_start_line(gt))
    if "gate" in pe:
        L.append("阶段包络      gate %.2f m / %.2f m/s" % tuple(pe["gate"]))
    s = d["safety"]
    L.append("安全          倾角停桨 %.0f°  去抖 %.2fs  (冲刺段 %s° / %ss)  视觉断供 %.2fs 后缓降"
             % (s["MAX_TILT_DEG"], s["FATAL_CONFIRM_S"], _c._fmt(s.get("GATE_DASH_TILT_DEG")),
                _c._fmt(s.get("GATE_DASH_TILT_CONFIRM_S")), s["VISION_LOST_LAND_S"]))
    sw = d["switches"]
    L.append("开关          影子 %s  假数据驱动 %s  冲刺前按 G %s  虚拟环 %s m  必须有法向 %s"
             % (sw.get("GATE_SHADOW"), sw.get("GATE_ALLOW_SIM_DRIVE"), sw.get("GATE_CONFIRM_DASH"),
                sw.get("GATE_VIRTUAL_OFFSET"), sw.get("GATE_REQUIRE_NORMAL")))
    L.append("              手动段起步 %s  放弃后回手动段 %s"
             % (sw.get("GATE_MANUAL_START"), sw.get("GATE_ABORT_TO_MANUAL")))
    return L


def _print_envelope_gate(report, g, d):
    print("=" * 78)
    print("现场档案: %s" % report["src"])
    for l in report["lines"]:
        print("  · " + l)
    if report["applied"]:
        print("  · 实际覆盖参数 %d 个:" % len(report["applied"]))
        for k, v in report["applied"]:
            print("      %-22s = %s" % (k, _c._fmt(v)))
    print("-" * 78)
    print("生效包络核对表:")
    for l in _gate_lines(g, d):
        print("    " + l)
    if report["warnings"]:
        print("-" * 78)
        for w in report["warnings"]:
            print("  ⚠️ " + w)
    print("=" * 78)
    print("校验通过。")


def _site_config_text_gate(report, g, d, extra=None):
    L = ["# 本次飞行生效的现场配置（自动生成，复盘用）",
         "# 档案来源: %s" % report["src"], ""]
    if report["knobs"]:
        L.append("[旋钮]")
        for k in sorted(report["knobs"]):
            L.append("  %-14s = %s" % (k, report["knobs"][k]))
        L.append("")
    if report.get("sets"):
        L.append("[原始参数覆盖 (档案 set. + 命令行 --set, 后者优先)]")
        for s in report["sets"]:
            L.append("  " + s)
        L.append("")
    L.append("[推导出的参数覆盖]")
    for k, v in report["applied"]:
        L.append("  %-22s = %s" % (k, _c._fmt(v)))
    L.append("")
    L.append("[生效包络]")
    L += ["  " + x for x in _gate_lines(g, d)]
    L.append("")
    L.append("[全部生效参数]")
    for k, v in _c._params_dict(g).items():
        L.append("  %-26s = %s" % (k, _c._fmt(v)))
    if report["warnings"]:
        L.append("")
        L.append("[警告]")
        for w in report["warnings"]:
            L.append("  ⚠️ " + w)
    if extra:
        L.append("")
        L += extra
    return "\n".join(L) + "\n"


from vb09.profile import core as _c  # noqa: E402  (见文件头)
