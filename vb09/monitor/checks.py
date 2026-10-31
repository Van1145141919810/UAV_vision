# -*- coding: utf-8 -*-
"""
vb09/monitor/checks.py —— 机况检查 (run_checks) + 文件与路径 (paths_info)
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py)。
"""

import os
import time

from vb09.monitor.common import HERE, HOSTNAME, PX4_PARAMS, SCRIPT_NODE, STATUS_TOPIC, TASK_LABEL, fp
from vb09.monitor.launcher import TEMP_DIR, TEMP_SITE, _tilde
from vb09.monitor.store import STORE


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
        add("飞行脚本文件", "fail", "找不到", "同目录放 stageNN_mission.py")
    else:
        add("飞行脚本文件", "ok" if args.script else "warn",
            "  ".join("%s %s" % (TASK_LABEL.get(t, t), os.path.basename(p))
                      for t, p in sorted(args.scripts.items())),
            "默认任务 %s (%s)" % (TASK_LABEL.get(args.task, args.task), args.task_src))
    return out


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
    add("网页 vb09/web/", os.path.join(HERE, "vb09", "web", "monitor.html"),
        "网页本身 (monitor.html + monitor.css + *.js), 在 flight_monitor.py 同目录的 vb09/web/ 下")
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
