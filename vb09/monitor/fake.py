# -*- coding: utf-8 -*-
"""
vb09/monitor/fake.py —— 离线假数据 (--fake): 一架假飞机看一个假环, 带假识别画面
===============================================================================
2026-10-30 第 5 步从 flight_monitor.py 原样搬来 (tools/dev/split_monitor.py)。
"""

import json
import math
import threading
import time
from vb09.common import geom

from vb09.monitor.common import SCRIPT_NODE
from vb09.monitor.store import STORE


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


def _fake_reddot(st, now):
    """--fake-kind reddot: 在环的假 /gate/status 上加红点 (reddot/locate.py extra) 的字段。"""
    aim = st.get("aim") or st.get("target")
    if aim is None:
        st.update(kind="reddot", cands=[], sel={"seq": None, "mode": "none", "state": "none", "trk_id": None},
                  n_state="few")
        return
    st.update(kind="reddot", trk_id=1, n_hit=60, score=0.93, st_scale=1.024, st_scale_sd=0.012, n_bins=9, n_dir=4,
              n_state="ok", sel={"seq": 1, "mode": "track", "state": "bound", "trk_id": 1},
              cands=[{"id": 1, "aim": aim, "score": 0.93, "hits": 60, "D": 0.148, "D_trk": 0.149, "Z_st": st.get("dist"),
                      "d_px": 40.0, "uv": [480.0, 270.0], "sd_cm": 2.4, "age": 0.0},
                     {"id": 3, "aim": [aim[0] + 0.2, aim[1] - 1.1, aim[2] + 0.3], "score": 0.41, "hits": 22,
                      "D": 0.12, "D_trk": None, "Z_st": st.get("dist"), "d_px": 31.0, "uv": [300.0, 250.0],
                      "sd_cm": 6.0, "age": 0.3}])


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
            if getattr(args, "fake_kind", "ring") == "reddot":
                _fake_reddot(st, now)          # 2026-11-08: 红点字段 (网页红点区 / 点选冒烟测试)
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
                if getattr(args, "fake_kind", "ring") == "reddot" and seg is not None:
                    gsec["track"] = {"step": {"wait": "detect", "align": "approach"}.get(seg, "locked"),
                                     "sig": [2.3, 2.2, 8.5], "rounds": 1 if seg != "wait" else 0,
                                     "dist_cmd": 3.2, "dir_src": "法向", "why": None,
                                     "sel": st.get("sel")}
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
