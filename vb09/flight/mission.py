# -*- coding: utf-8 -*-
"""
vb09/flight/mission.py —— 穿环任务: 一次尝试的状态机 (GateRun) 和 phase_gate (第 4.4 步换成 Step + run_mission)
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import sys
import json
import math

import rospy
import gate_plan as gp      # 同目录, 纯函数
from vb09.flight import config as C

from vb09.flight.ctx import F
from vb09.flight.ctx import _f, check_abort, request_abort, request_land, set_phase, stop_requested, telem_put, tilt_limit
from vb09.flight.keys import KeyReader
from vb09.flight.log import gate_queue
from vb09.flight.steps.base import StepBack, StepFail
from vb09.flight.telemetry import clamp, tilt_deg_from_quat, wrap_pi


# =============================================================================
# ==========================   穿环: 状态机   =================================
# =============================================================================
# 2026-10-30 (第 4.4 步): GateStop / GateBack 改名 StepFail / StepBack, 定义在 steps/base.py; 老名字留着
GateStop, GateBack = StepFail, StepBack


def _l3(v):
    return [_f(x) for x in v] if v is not None else None


class MissionCtx:
    """
    步骤看到的 ctx (原 GateRun)。run_mission() 创建它, 依次 ctx.enter(step); lock = step.run(ctx, lock)。
    步骤只能通过它下指令: pos / vel / hold / yaw_step / set_seg / fail / tick / setpoint / gate_in / again。
    tick() 统一做: 键盘、冲刺前的停止条件、遥测 gate 段、gate.csv、按 GATE_LOOP_HZ 节拍。
    任务状态 (acq / locked / plan / passed / dash_state …) 也在这里, 遥测 gate 段按它们拼。
    """

    def __init__(self, z_hold):
        self.z_hold = z_hold
        self.seg = ""
        p = self.pos()
        self.p_hover = p
        self.yaw_sp = F.tele.yaw_measured()
        self.acq = None          # ACQUIRE 粗估 (gate_plan.fuse_samples 的结果)
        self.locked = None       # ALIGN 锁定值 (+ t_lock)
        self.plan = None
        self.plan_seq = 0
        self.t0 = None           # 冲刺起点 (ROS 秒)
        self.lvl = 0             # 0 正常 / 1 卡纸处置 (L1)
        self.passed = False
        self.dash_state = None   # (t, s_plan, v_plan, s_est, v_est)
        self.land_deferred = None
        self.outcome = ""
        self.dock = None         # 停靠: {S, d, d_in_src, err_cm, aim_sd_cm, t_end} (遥测 gate.dock)
        self.keys = None
        self.flags = set()
        self.move = None         # 本拍最后一个方向/悬停键 (手动段用)
        self.nkeys = 0           # 本拍读到几个字符 (手动段: 有键 = 重新计时)
        self.back_ok = False     # True = 冲刺前的段, M / 方向键 → 回手动段
        self.n_try = 0           # 第几次 (按 T 开始一次 +1)
        self.last_why = ""       # 上一次放弃的原因 (遥测 gate.why)
        self.pre = None          # 手动段预规划结果 (遥测 gate.pre)
        self.rate = rospy.Rate(C.GATE_LOOP_HZ)
        self.last_t = rospy.Time.now().to_sec()
        self.t_log = 0.0
        self.origin = (F.INITIAL_X, F.INITIAL_Y, F.INITIAL_Z)
        self.params = gp.params_from_globals(vars(C))
        self.step = None         # 当前步骤 (enter 设)
        self._again = None       # 步骤要求"做完回手动段"的原因 (影子档), 见 again()

    # ---------------- 步骤 ----------------
    def enter(self, step):
        """
        进入一个步骤。平台白名单属性在这里生效 (步骤只声明):
          committed (冲刺) → 速度包络放开到冲刺档, 不能再回手动段, 停桨线换成 GATE_DASH_TILT_DEG;
                             tick() 推迟 LAND 到刹停后
          vel_lim "pre"    → 冲刺前的速度包络 GATE_PRE_VEL_LIM
        """
        self.step = step
        if step.committed:
            set_phase("gate")                                  # 速度包络放开到冲刺档
            self.back_ok = False
            F.dash_tilt_on[0] = True                             # 停桨线换成 GATE_DASH_TILT_DEG
        elif step.vel_lim == "pre":
            set_phase("gate", vel=C.GATE_PRE_VEL_LIM)

    def again(self, why):
        """步骤做完后回手动段 (能回的话), why 显示在手动段提示里。"""
        self._again = why

    @property
    def setpoint(self):
        return F.setpoint

    @property
    def gate_in(self):
        return F.gate_in

    # ---------------- 小工具 ----------------
    def pos(self):
        p = F.tele.pose
        if p is None:
            return (F.INITIAL_X, F.INITIAL_Y, self.z_hold if hasattr(self, "z_hold") else F.INITIAL_Z)
        q = p.pose.position
        return (q.x, q.y, q.z)

    def vel(self):
        v = F.tele.velocity
        if v is None:
            return (0.0, 0.0, 0.0)
        l = v.twist.linear
        return (l.x, l.y, l.z)

    def set_seg(self, name, msg=None):
        if name != self.seg:
            self.seg = name
            F.sub_mode_ref[0] = name
            rospy.loginfo(f"[gate] ── 段 {name}" + (f": {msg}" if msg else ""))

    def yaw_step(self, target, dt):
        """yaw 设定值以 GATE_YAW_RATE 朝 target 走一步。→ 剩余误差 (rad)。"""
        err = wrap_pi(target - self.yaw_sp)
        step = C.GATE_YAW_RATE * dt
        self.yaw_sp = wrap_pi(self.yaw_sp + clamp(err, -step, step))
        return wrap_pi(target - F.tele.yaw_measured())

    def hold(self, p):
        F.setpoint.set_zhold_mode(p[0], p[1], p[2], self.yaw_sp, 0.0, 0.0)

    def fail(self, why):
        raise GateStop(why)

    @staticmethod
    def _raw_sigma(f, smp):
        """
        样本是桥每帧的环心观测 (ring_w, 没经过桥的滤波) 时, 多帧中值的 σ ≈ 单帧 σ / √k。
        样本是桥的融合估计 (老版桥 / target) 时, 每帧的 σ 已经是融合后的, 不再除。
        """
        sg = f.get("sig_hva")
        if sg and smp and all(s.get("raw") for s in smp):
            k = max(1, len(smp))
            f["raw"] = True
            f["sig_hva_frame"] = sg
            # 多帧中值的标准误: 用实际散布 (p90−p10 ≈ 2.56σ, 中值效率 ≈ 1.25) 和桥报的单帧 σ 取大的
            sp = f.get("spread_hva") or (0.0, 0.0, 0.0)
            f["sig_hva"] = tuple(max(x, 1.25 * y / 2.56) / math.sqrt(k) for x, y in zip(sg, sp))

    @staticmethod
    def _sd_fields(f):
        """融合结果里给遥测用的 σ 字段。sd_hva_cm = 靶面 (横 h, 竖 v, 深度) σ 中位数。"""
        sg = f.get("sig_hva")
        if sg:
            f["sd_hva_cm"] = [round(x, 2) for x in sg]
            f["sd_cm_xyz"] = [round(max(sg[0], sg[1]), 2)] * 3

    @staticmethod
    def _sig_txt(f):
        sg = f.get("sig_hva")
        return f"横{sg[0]:.1f}/竖{sg[1]:.1f}/深{sg[2]:.1f}cm" if sg else "?"

    def used(self):
        """遥测 / 规划用的环: 锁定值 > 粗估 > 最新一帧。→ dict 或 None"""
        if self.locked:
            return self.locked
        if self.acq:
            return self.acq
        s = F.gate_in.latest_valid()
        if s is None:
            return None
        return {"c": s["c"], "aim": s["aim"], "n": s["n"], "k": 1, "spread_cm": None,
                "sig": s.get("sig")}

    def n_used(self, u, p):
        """→ (法向, 来源)。没有法向时用"−进入方向"或视线方向代替 (网页靠它画靶面)。"""
        if u and u.get("n") is not None:
            return u["n"], "bridge"
        if self.plan is not None and self.plan.dir is not None:
            return gp.vmul(self.plan.dir, -1.0), "plan_dir"
        if u:
            d = gp.hunit(gp.vsub(p, u["c"]))
            if d is not None:
                return d, "los"
        return None, None

    # ---------------- 每拍 ----------------
    def tick(self, dash=None):
        """dash=None: 按当前步骤 committed 决定 (冲刺中 LAND 推迟、不响应 M)。"""
        if dash is None:
            dash = bool(self.step is not None and self.step.committed)
        now = rospy.Time.now().to_sec()
        dt = min(max(now - self.last_t, 0.0), 0.2)
        self.last_t = now
        if self.keys is not None:
            move, flags, eof, nk = self.keys.poll()
            if 'x' in flags:
                request_abort(f"gate X 键 (段 {self.seg})")
            if eof:
                request_land("gate stdin EOF (SSH 断连?)")
            if 'l' in flags:
                request_land("gate L 键")
            self.flags, self.move, self.nkeys = flags, move, nk
        else:
            self.flags, self.move, self.nkeys = set(), None, 0
        check_abort()
        if F.takeover[0] is not None:
            raise GateStop("控制权被接管", retry=False)
        if F.land_flag.is_set():
            if not dash:
                raise GateStop(f"LAND 请求: {F.land_reason[0]}", retry=False)
            if self.land_deferred is None:
                self.land_deferred = F.land_reason[0]
                rospy.logwarn(f"[gate] LAND 请求 ({F.land_reason[0]}) —— 冲刺中, 推迟到刹停后执行")
        if not dash and self.back_ok and ('m' in self.flags or self.move is not None):
            raise GateBack("操作员按了 %s" % ("M" if 'm' in self.flags else "方向键"))
        try:
            self._telem_and_log(now)
        except Exception as e:
            rospy.logwarn_throttle(C.THROTTLE_S, f"[gate] 遥测/日志出错 (不影响飞行): {e}")
        self.rate.sleep()
        return dt

    def _telem_and_log(self, now):
        p, v = self.pos(), self.vel()
        u = self.used()
        n, n_src = self.n_used(u, p)
        g = {"seg": self.seg}
        along = off = pred = ttc = v_al = None
        if u is not None and n is not None:
            h_, v_, along = gp.to_plane(p, u["c"], n)
            off = [h_, v_]
            right, up = gp.gate_axes(n)
            v_al = -gp.vdot(v, n)
            if self.seg in ("dash", "brake") and v_al > 0.2 and along > 0:
                ttc = along / v_al
                pred = [h_ + gp.vdot(v, right) * ttc, v_ + gp.vdot(v, up) * ttc]
            elif self.plan is not None and self.plan.ok:
                ah, av, _ = gp.to_plane(self.plan.aim, u["c"], n)
                pred = [ah, av]                       # 计划从瞄点穿过
            else:
                pred = list(off)
            sd = u.get("sd_cm_xyz") or [C.GATE_SD_MAX_CM] * 3
            hva = u.get("sd_hva_cm") or (list(u["sig"]) if u.get("sig") else None)
            sdh, sdv = (hva[0], hva[1]) if hva else (max(sd), max(sd))
            g.update({
                "gate_used": {
                    "c": _l3(u["c"]), "n": _l3(n), "n_src": n_src,
                    "aim": _l3(u.get("aim") or u["c"]),
                    "sd_cm": _l3(sd), "sd_hva_cm": _l3(hva), "n_sd_deg": _f(u.get("n_spread_deg")),
                    "locked": bool(self.locked), "t_lock": (self.locked or {}).get("t_lock"),
                    "r_in": C.R_IN, "r_out": C.R_OUT, "spread_cm": _f(u.get("spread_cm")),
                    "k": u.get("k")},
                "along": _f(along), "off": _l3(off), "pred_cross": _l3(pred),
                "pred_sd_cm": [_f(math.hypot(sdh, 5.0)), _f(math.hypot(sdv, 5.0))],
                "ttc": _f(ttc), "v_along_meas": _f(v_al),
            })
        ds = self.dash_state
        g["v_along_cmd"] = _f(ds[2]) if ds and self.seg in ("dash", "brake") else 0.0
        lv = F.gate_in.latest_valid()
        g["gate_age"] = _f(now - lv["t"]) if lv else None
        if self.plan is not None and self.plan.ok:
            P = self.plan
            g["commit"] = {"d": _f(P.d_run), "passed": bool(self.passed),
                           "abort_ok": self.seg in ("manual", "wait", "align", "commit", "abort")}
            g["plan"] = {"seq": self.plan_seq,
                         "pts": [_l3(self.p_hover), _l3(P.S), _l3(P.aim), _l3(P.E)],
                         "labels": ["悬停点", "起跑点", "穿越点", "制动终点"],
                         "mode": "dash",
                         "v": _f(P.v), "summary": P.summary(), "dir_why": P.dir_why,
                         "v_set": _f(C.DASH_V),
                         "slow": bool(P.v < C.DASH_V - 1e-9),
                         "slow_why": P.slow_why,
                         "need_confirm": bool(P.need_confirm),
                         "v_hit": _f(P.v_hit_nom), "v_hit_worst": _f(P.v_hit_worst),
                         "virtual": C.GATE_VIRTUAL_OFFSET > 0}
            g["brake"] = {"end": _l3(P.E), "dist": _f(P.brake_dist), "acc": _f(C.BRAKE_ACC),
                          "s_b": _f(P.traj.s_b), "s_stop": _f(P.s_stop),
                          "pass_run": _f(C.PASS_RUN),
                          "pass_lag": _f(P.pass_lag), "pass_actual": _f(P.pass_actual),
                          "wall_left": _f((P.tol or 0.0) + C.M_BACK)}
        nv, nr, rej = F.gate_in.counters()
        g["det"] = {"n_valid": nv, "n_rej": nr, "rej": rej}
        if ds:
            g["dash"] = {"t": _f(ds[0]), "s_plan": _f(ds[1]), "v_plan": _f(ds[2]),
                         "s_est": _f(ds[3]), "v_est": _f(ds[4])}
        g["lvl"] = self.lvl
        g["why"] = self.last_why        # 上一次放弃的原因 (回手动段后网页提示行显示)
        if self.seg == "manual" and self.pre:
            pr = self.pre
            g["pre"] = {"ok_full": bool(pr["ok_full"]), "v_set": _f(C.DASH_V), "v_room": _f(pr["v_room"]),
                        "why": pr["why"], "xy_need": _f(pr["xy_need"]), "d_back_need": _f(pr["d_back_need"])}
        g["land_deferred"] = self.land_deferred
        g["shadow"] = bool(C.GATE_SHADOW)
        g["mode"] = C.MISSION_MODE
        if self.dock is not None:
            g["dock"] = dict(self.dock)           # 拷贝: 遥测线程序列化时步骤还在改它
        telem_put("gate", g)

        # ---- gate.csv
        if F.writer_gate is None:
            return
        st, age = F.gate_in.latest()
        det = (st or {}).get("target") if st else None
        mode, spp, _, _, yaw_sp = F.setpoint.snapshot()
        nan = float("nan")

        def fx(x, fmt="%.3f"):
            return (fmt % x) if isinstance(x, (int, float)) and x == x else "nan"
        cc = (u or {}).get("c") or (nan,) * 3
        aa = ((u or {}).get("aim") or cc) if u else (nan,) * 3
        nn = n or (nan,) * 3
        q = F.tele.pose.pose.orientation if F.tele.pose else None
        sg_ = (lv or {}).get("sig") or (nan,) * 3
        rw_ = (st or {}).get("ring_w") or (nan,) * 3
        row = [
            "%.3f" % now, self.seg, fx(along), fx(off[0] if off else None),
            fx(off[1] if off else None), fx(pred[0] if pred else None),
            fx(pred[1] if pred else None), fx(ttc), fx(v_al), fx(g.get("v_along_cmd")),
            fx((now - lv["t"]) * 1000 if lv else None, "%.0f"),
            fx(cc[0]), fx(cc[1]), fx(cc[2]), fx(nn[0], "%.4f"), fx(nn[1], "%.4f"), fx(nn[2], "%.4f"),
            fx(aa[0]), fx(aa[1]), fx(aa[2]), int(bool(self.locked)),
            {0: "V", 1: "P", 2: "T"}.get(mode, "?"), self.lvl,
            fx(ds[1] if ds else None), fx(ds[2] if ds else None),
            fx(ds[3] if ds else None), fx(ds[4] if ds else None),
            fx(spp[0]), fx(spp[1]), fx(spp[2]), fx(p[0]), fx(p[1]), fx(p[2]),
            fx(math.degrees(F.tele.yaw_measured()), "%.1f"),
            fx(tilt_deg_from_quat(q) if q else None, "%.1f"),
            fx((st or {}).get("dist") if st else None),
            fx(det[0] if det else None), fx(det[1] if det else None), fx(det[2] if det else None),
            fx(age * 1000 if st else None, "%.0f"), nv, nr, int(self.land_deferred is not None),
            fx(sg_[0], "%.2f"), fx(sg_[1], "%.2f"), fx(sg_[2], "%.2f"),
            fx(tilt_limit()[0], "%.0f"),
            "%s|%s" % ((st or {}).get("src") or "", (st or {}).get("ring_src") or ""),
            fx(rw_[0]), fx(rw_[1]), fx(rw_[2]),
            int(bool(((st or {}).get("ring") or {}).get("partial"))) if st else "nan",
            json.dumps((st or {}).get("diag") or {}, ensure_ascii=False, separators=(",", ":"))
            if isinstance((st or {}).get("diag"), dict) else "",
        ]
        gate_queue.append(row)           # deque.append 线程安全, 不碰磁盘

    # ---------------- ACQUIRE ----------------
    # ---------------- MANUAL (手动段) ----------------
    def manual(self, why=""):
        """
        手动段: 键 (机体系 WASD 平移 / R F 升降 / Q E 转向 / 空格悬停),
        按 T 返回 = 开始这一次 (找环 → 规划 → …)。所有按键都是"给速度, 按 dt 积分", 松键
        TELEOP_DEADMAN 后速度归零、位置冻结。GATE_MANUAL_TIMEOUT 内一个键都没按 → 降落。
        没有终端键盘 → 直接返回 (= 老行为: 起飞后马上找环)。
        """
        self.back_ok = False
        F.dash_tilt_on[0] = False
        set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
        self.set_seg("manual", why or "按 T 开始找环/规划")
        if self.keys is None:
            rospy.logwarn("[gate] 没有终端键盘 —— 跳过手动段, 直接找环")
            return
        self.keys.drain()                          # 丢掉回手动段之前按着的键
        self.flags, self.move, self.nkeys = set(), None, 0
        if why:
            rospy.logwarn(f"[gate] ── 手动段: {why}")
        rospy.logwarn("[gate] ★ 手动段: W前 S后 A左 D右 | R升 F降 | Q左转 E右转 | 空格悬停 | "
                      "T = 开始找环规划 | L = 降落 | X = 停桨")
        p = self.pos()
        tx, ty, tz = p
        yaw = self.yaw_sp
        vbx = vby = vz = yr = 0.0
        t0 = rospy.Time.now().to_sec()
        t_key = t0
        dt = 0.0
        while True:
            now = rospy.Time.now().to_sec()
            if 't' in self.flags:
                rospy.logwarn("[gate] ★ T → 开始找环/规划")
                break
            if self.nkeys > 0:
                t_key = now
            mv = self.move
            if mv is not None:
                vbx = vby = vz = yr = 0.0
                if mv == 'w':
                    vbx = C.TELEOP_XY_SPEED
                elif mv == 's':
                    vbx = -C.TELEOP_XY_SPEED
                elif mv == 'a':
                    vby = C.TELEOP_XY_SPEED
                elif mv == 'd':
                    vby = -C.TELEOP_XY_SPEED
                elif mv == 'r':
                    vz = C.TELEOP_Z_SPEED
                elif mv == 'f':
                    vz = -C.TELEOP_Z_SPEED
                elif mv == 'q':
                    yr = C.TELEOP_YAW_RATE
                elif mv == 'e':
                    yr = -C.TELEOP_YAW_RATE
            idle = now - t_key
            if idle > C.TELEOP_DEADMAN:
                vbx = vby = vz = yr = 0.0
            if idle > C.GATE_MANUAL_TIMEOUT:
                raise GateStop(f"手动段 {C.GATE_MANUAL_TIMEOUT:.0f}s 没有按键", retry=False)
            cy, sy = math.cos(yaw), math.sin(yaw)
            vx, vy = vbx * cy - vby * sy, vbx * sy + vby * cy
            tx += vx * dt
            ty += vy * dt
            yaw = wrap_pi(yaw + yr * dt)
            ddx, ddy = tx - F.INITIAL_X, ty - F.INITIAL_Y
            r = math.hypot(ddx, ddy)
            if r > C.TELEOP_XY_MAX:
                tx, ty = F.INITIAL_X + ddx * C.TELEOP_XY_MAX / r, F.INITIAL_Y + ddy * C.TELEOP_XY_MAX / r
                vx = vy = 0.0
            tz = F.INITIAL_Z + clamp(tz + vz * dt - F.INITIAL_Z, C.TELEOP_DZ_MIN, C.TELEOP_DZ_MAX)
            self.yaw_sp = yaw
            F.setpoint.set_zhold_mode(tx, ty, tz, yaw, vx, vy)
            if now - self.t_log > 1.0:
                self.t_log = now
                pn = self.pos()
                lv = F.gate_in.latest_valid()
                self._rej_snap, rtxt = F.gate_in.rej_since(getattr(self, "_rej_snap", None))
                if lv is not None and now - lv["recv"] < 1.0:
                    dc = math.hypot(lv["c"][0] - pn[0], lv["c"][1] - pn[1])
                    dmin = gp.d_run_for(C.DASH_V, self.params)
                    hint = ("就地起跑" if dmin <= dc <= max(dmin, C.D_RUN_MAX) else
                            ("会后退到 %.1fm" % dmin) if dc < dmin else ("会前进到 %.1fm" % max(dmin, C.D_RUN_MAX)))
                    seen = f"看到环: 水平 {dc:.2f}m 高差 {lv['c'][2] - pn[2]:+.2f}m ({hint})"
                    seen += "  " + self.manual_precheck(pn, now)
                else:
                    seen = rtxt or "没看到环"
                    self.pre = None
                rospy.loginfo(f"    [manual] d=({pn[0]-F.INITIAL_X:+.2f},{pn[1]-F.INITIAL_Y:+.2f},"
                              f"{pn[2]-F.INITIAL_Z:+.2f}) yaw={math.degrees(yaw):+.0f}°  {seen}  "
                              f"(无按键 {idle:.0f}s / {C.GATE_MANUAL_TIMEOUT:.0f}s)")
            dt = self.tick()
        F.setpoint.set_zhold_mode(tx, ty, tz, yaw, 0.0, 0.0)

    def manual_precheck(self, pn, now):
        """
        手动段里 (2026-10-21) 每 2s 按飞机当前位置、最近 1s 的识别中值预规划一次:
        按 T 之前就知道 设定速度放不放得下 / 瞄点高度在不在带内, 不用 T 了才发现。
        → 一段日志文字; 结果放进 self.pre (遥测 gate.pre, 网页提示行显示)。
        """
        if self.pre is not None and now - self.pre.get("_t", 0) < 2.0:
            return self.pre.get("_txt", "")
        smp = F.gate_in.valid_since(now - 1.0)
        if len(smp) < 3:
            return ""
        try:
            c = tuple(gp.median([s["c"][i] for s in smp]) for i in range(3))
            aims = [s["aim"] for s in smp if s.get("aim")]
            aim = tuple(gp.median([a[i] for a in aims]) for i in range(3)) if aims else c
            r = gp.preflight_check(aim, c, None, self.origin, self.params, 0.0,
                                   virtual_offset=C.GATE_VIRTUAL_OFFSET, p_now=pn)
        except Exception as e:
            return f"(预规划出错: {e})"
        if r["ok_full"]:
            txt = f"按 {C.DASH_V:.1f} m/s: ✓ 放得下"
        else:
            txt = (f"按 {C.DASH_V:.1f} m/s: ✗ {r['why'] or '不可行'} → 最高 "
                   + (f"{r['v_room']:.1f} m/s" if r["v_room"] is not None else "—")
                   + (f", 满速要 xy_radius ≥ {r['xy_need']:.1f}" if r["xy_need"] and r["xy_need"] > C.GATE_XY_MAX else ""))
            if not (self.pre or {}).get("_warned"):
                rospy.logwarn(f"[gate] ★ 预规划 (按 T 之前): {txt}")
                r["_warned"] = True
            elif self.pre:
                r["_warned"] = True
        r.update(_t=now, _txt=txt, v_set=C.DASH_V)
        self.pre = r
        return txt

    def new_attempt(self):
        """按 T 之后: 清掉上一次的估计/计划, 以当前位置为悬停点开始。"""
        p = self.pos()
        self.p_hover = p
        self.z_hold = p[2]
        self.acq = self.locked = self.plan = None
        self.dock = None
        self.lvl = 0
        self.passed = False
        self.dash_state = None
        self.outcome = ""
        self.n_try += 1
        self.last_why = ""
        self.back_ok = self.keys is not None
        self.step = None
        self._again = None
        if self.n_try > 1:
            rospy.logwarn(f"[gate] ── 第 {self.n_try} 次")

def run_mission(z_hold, steps):
    """
    执行一个任务 (MODES[mode] 的步骤列表)。返回 (x, y, z, yaw) —— 结束时的悬停点, main 从这里原地降落。
    2026-10-30: 原 phase_gate, 中间写死的段序列换成 `for step in steps: ctx.enter(step); lock = step.run(ctx, lock)`。

    流程 (2026-10-19): 手动段 ─T→ 找环 → 规划 → 摆位 → 锁定 → 等 G → 冲刺 → 刹停悬停
      冲刺前放弃 (GateStop.retry) 或按 M / 方向键 (GateBack) → 回手动段, 可以 T 重来;
      LAND / 接管 / 手动段超时 → 原地悬停后返回 (降落)。影子模式做完也回手动段。
      FATAL (X 键 / 倾角 / 断链) 照常抛 RuntimeError 给 main 的 cleanup。
    """
    set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
    run = MissionCtx(z_hold)
    if sys.stdin.isatty():
        try:
            run.keys = KeyReader()
            run.keys.__enter__()
        except Exception as e:
            rospy.logwarn(f"[gate] 键盘不可用 ({e})")
            run.keys = None
    rospy.loginfo("[gate] 键盘: 手动段 WASD/RF/QE/空格 + T = 开始 | 冲刺前 M 或方向键 = 回手动段 | "
                  "G = 确认冲刺 | L = 降落 | X = 停桨")
    can_retry = run.keys is not None and C.GATE_ABORT_TO_MANUAL
    manual_on = C.GATE_MANUAL_START
    why_manual = ""
    try:
        while True:
            try:
                if manual_on:
                    run.manual(why_manual)
                run.new_attempt()
                lock = None
                for step in steps:
                    run.enter(step)
                    lock = step.run(run, lock)
                if run._again is not None and can_retry:
                    manual_on = True
                    why_manual = run._again
                    continue
                break
            except GateStop as e:
                back = isinstance(e, GateBack)
                if (back or (e.retry and can_retry)) and F.takeover[0] is None and not F.land_flag.is_set():
                    run.hold(run.pos())
                    if back:
                        rospy.logwarn(f"[gate] {e.args[0]} —— 回手动段")
                        why_manual = "操作员中止这一次"
                    else:
                        rospy.logerr(f"[gate] 这一次放弃: {e.args[0]} —— 回手动段 (T 重来 / L 降落)")
                        why_manual = f"上一次放弃: {e.args[0]}"
                    run.last_why = str(e.args[0])
                    run.outcome = ""
                    manual_on = True
                    continue
                run.outcome = run.outcome or "abort"
                run.set_seg("abort", str(e.args[0]))
                rospy.logerr(f"[gate] 放弃: {e.args[0]} —— 原地悬停后降落")
                if F.takeover[0] is None:
                    run.hold(run.pos())
                break
        run.back_ok = False
        # 收尾: 以当前位置重锚 (冲刺中 VIO 可能跳过, 不回规划的 E 点)
        if F.takeover[0] is None and not F.abort_flag.is_set():
            hp = run.pos()
            run.hold(hp)
            if run.seg not in ("abort",):
                run.set_seg("hold", f"在当前位置悬停 {C.GATE_HOLD_S:.1f}s 后降落")
            set_phase("gate")                     # 刹停后可能还有残速, 用冲刺档包络
            t_end = rospy.Time.now().to_sec() + C.GATE_HOLD_S
            while rospy.Time.now().to_sec() < t_end and not stop_requested():
                run.hold(hp)
                try:
                    run.tick(dash=True)           # dash=True: LAND 不再抛 GateStop, 反正马上降
                except GateStop:
                    break
        F.dash_tilt_on[0] = False                   # 降落段回到平时的停桨线
        if run.land_deferred:
            rospy.logwarn(f"[gate] 冲刺中推迟的 LAND: {run.land_deferred} —— 现在降落")
        rospy.loginfo(f"[gate] 结束: {run.outcome or 'abort'}  (共 {run.n_try} 次)")
        p = run.pos()
        return p[0], p[1], p[2], run.yaw_sp
    finally:
        F.dash_tilt_on[0] = False
        if run.keys is not None:
            run.keys.__exit__(None, None, None)


def phase_gate(z_hold):
    """穿环: 按 C.MISSION_MODE (档案 mode; 旧写法 GATE_SHADOW = True → shadow) 选步骤列表。"""
    from vb09.flight.modes import MODES
    return run_mission(z_hold, MODES[C.MISSION_MODE])


GateRun = MissionCtx          # 老名字
