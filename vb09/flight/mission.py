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
from vb09.flight.telemetry import clamp, tilt_deg_from_quat, wrap_pi


# =============================================================================
# ==========================   穿环: 状态机   =================================
# =============================================================================
class GateStop(Exception):
    """
    冲刺前的正常退出 (放弃 / LAND / 接管)。原因在 args[0]。
    retry=True  = 这一次不行 (锁不住 / 不可行 / G 超时 …), GATE_ABORT_TO_MANUAL 时回手动段重来
    retry=False = LAND 请求 / 接管 / 手动段超时: 不重来, 降落
    """

    def __init__(self, why, retry=True):
        super().__init__(why)
        self.retry = retry


class GateBack(GateStop):
    """操作员要回手动段 (冲刺前按 M 或任一方向键)。"""


def _l3(v):
    return [_f(x) for x in v] if v is not None else None


class GateRun:
    """
    一次穿环。phase_gate() 创建它并依次调用各段; 每段都是"循环 + tick()"。
    tick() 统一做: 键盘、冲刺前的停止条件、遥测 gate 段、gate.csv、按 GATE_LOOP_HZ 节拍。
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
    def tick(self, dash=False):
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
        手动段: 和 stage11 的 teleop 一样的键 (机体系 WASD 平移 / R F 升降 / Q E 转向 / 空格悬停),
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
        self.lvl = 0
        self.passed = False
        self.dash_state = None
        self.outcome = ""
        self.n_try += 1
        self.last_why = ""
        self.back_ok = self.keys is not None
        if self.n_try > 1:
            rospy.logwarn(f"[gate] ── 第 {self.n_try} 次")

    # ---------------- ACQUIRE ----------------
    def acquire(self):
        self.set_seg("wait", f"悬停识别 (窗口 {C.HOVER_ACQ:.0f}s, 至少 {C.GATE_ACQ_MIN_N} 帧)")
        set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
        t_start = rospy.Time.now().to_sec()
        yaw0 = self.yaw_sp
        scan = [yaw0 + math.radians(C.GATE_SCAN_DEG), yaw0 - math.radians(C.GATE_SCAN_DEG), yaw0]
        scan_i = None                      # None = 还没开始扫; len(scan) = 扫完了
        hp = (self.p_hover[0], self.p_hover[1], self.z_hold)
        self.p_hover = hp
        dt = 0.0
        tol = math.radians(C.GATE_YAW_TOL_DEG)
        last_why = "还没看到环"
        rej_snap, _ = F.gate_in.rej_since(None)
        while True:
            now = rospy.Time.now().to_sec()
            p = self.pos()
            # 只用进入 ACQUIRE 之后收到的帧 (起飞爬升中的帧不算)
            smp = F.gate_in.valid_since(max(t_start, now - C.HOVER_ACQ))
            fresh = [s for s in smp if now - s["recv"] < C.GATE_STATUS_MAX_AGE]
            scanning = False
            if fresh:                      # 看得到: 机头转向环心
                c = fresh[-1]["c"]
                target = math.atan2(c[1] - p[1], c[0] - p[0])
            elif not smp and now - t_start > C.GATE_ACQ_SCAN_AFTER and (scan_i or 0) < len(scan):
                if scan_i is None:
                    scan_i = 0
                    rospy.logwarn(f"[gate] {C.GATE_ACQ_SCAN_AFTER:.0f}s 没看到环 —— "
                                  f"±{C.GATE_SCAN_DEG:.0f}° 慢扫一次")
                target, scanning = scan[scan_i], True
            else:
                target = self.yaw_sp
            err = self.yaw_step(target, dt)
            if scanning and abs(wrap_pi(target - self.yaw_sp)) < 1e-3 and abs(err) < tol:
                scan_i += 1
            self.hold(hp)
            if len(smp) >= C.GATE_ACQ_MIN_N and fresh and abs(err) < math.radians(C.GATE_YAW_TOL_DEG):
                d_los = gp.hunit(gp.vsub(fresh[-1]["c"], p)) or (math.cos(self.yaw_sp), math.sin(self.yaw_sp), 0.0)
                f = gp.fuse_samples(smp, p, d=d_los)
                self._raw_sigma(f, smp)
                sh, sv, sa = f["spread_hva"]
                if f.get("raw"):                     # 每帧原始环心: 散布本来就大, 看中值的标准误
                    sh, sv, sa = (x * 2.56 / 1.25 for x in f["sig_hva"])
                if max(sh, sv) <= C.GATE_ACQ_SPREAD_CM and sa <= 2.0 * C.GATE_ACQ_SPREAD_CM:
                    self._sd_fields(f)
                    self.acq = f
                    rospy.loginfo(f"[gate] 粗估: 环心 ({f['c'][0]:+.2f},{f['c'][1]:+.2f},{f['c'][2]:+.2f}) "
                                  f"k={f['k']} 离散 横{max(sh, sv):.1f}/深{sa:.1f}cm  σ {self._sig_txt(f)}  "
                                  f"法向 {'有' if f.get('n') else '无'}  红点 {'有' if f.get('aim') else '无'}")
                    return
                last_why = (f"样本离散 横{max(sh, sv):.1f}/深{sa:.1f}cm > "
                            f"{C.GATE_ACQ_SPREAD_CM:.0f}/{2 * C.GATE_ACQ_SPREAD_CM:.0f}cm")
            elif smp:
                last_why = f"有效帧 {len(smp)}/{C.GATE_ACQ_MIN_N}" + (
                    "" if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) else
                    f", 机头还差 {math.degrees(err):+.0f}°")
            if now - t_start > C.GATE_ACQ_TIMEOUT:
                nv, nr, rej = F.gate_in.counters()
                self.fail(f"{C.GATE_ACQ_TIMEOUT:.0f}s 内没拿到稳定识别 ({last_why}; "
                          f"拒绝原因 {', '.join(f'{k}={v}' for k, v in rej.items() if v)})")
            if now - self.t_log > 1.0:
                self.t_log = now
                rej_snap, rtxt = F.gate_in.rej_since(rej_snap)
                if not smp and rtxt:
                    last_why = rtxt
                rospy.loginfo(f"    [wait] {last_why}")
            dt = self.tick()

    # ---------------- PLAN ----------------
    def make_plan(self, est, p_from, label):
        aim = est.get("aim") or est["c"]
        P = gp.plan_dash(aim, est["c"], est.get("n"), p_from, self.origin, self.params,
                         virtual_offset=C.GATE_VIRTUAL_OFFSET)
        if not P.ok:
            self.fail(f"{label}: " + "; ".join(P.errors))
        self.plan = P
        self.plan_seq += 1
        rospy.loginfo(f"[gate] {label}: {P.summary()}")
        for w in P.notes + P.warnings:
            rospy.logwarn(f"[gate]   ⚠ {w}")
        S = P.S
        if est.get("c") is not None and P.dir is not None:
            # 靶面上看: 目标 (红点/环心) 和机体中心瞄点各在环心的哪里
            r_, u_, _ = gp.entry_axes(P.dir)
            c0 = est["c"]
            t_h, t_v = gp.vdot(gp.vsub(P.aim_tool, c0), r_), gp.vdot(gp.vsub(P.aim_tool, c0), u_)
            a_h, a_v = gp.vdot(gp.vsub(P.aim, c0), r_), gp.vdot(gp.vsub(P.aim, c0), u_)
            rospy.loginfo(f"[gate]   靶面 (相对环心, 右/上 cm): 目标 ({t_h*100:+.0f},{t_v*100:+.0f})  "
                          f"机体中心瞄 ({a_h*100:+.0f},{a_v*100:+.0f})  "
                          f"针尖预计 ({(a_h+C.NEEDLE_RIGHT)*100:+.0f},{(a_v+C.NEEDLE_UP)*100:+.0f})"
                          + (f"  ★偏离目标 {P.tool_miss*100:.0f}cm" if P.aim_clamped else ""))
        rospy.loginfo(f"[gate]   起跑点 S=({S[0]:+.2f},{S[1]:+.2f},{S[2]:+.2f})  "
                      f"制动终点 E=({P.E[0]:+.2f},{P.E[1]:+.2f},{P.E[2]:+.2f})  "
                      f"进入方向 {math.degrees(gp.yaw_of(P.dir)):+.0f}°")
        return P

    # ---------------- REPOSITION ----------------
    def move_to(self, S, yaw_mode, timeout, label):
        """
        setpoint 以 GATE_REPO_V 斜坡走直线到 S (3D, 带牵引绳)。yaw_mode:
          "ring" = 机头盯环心 (保持环在视野里)   数值 = 固定朝向
        全程查"飞机在环前 ≥1.5m" (环系 x ≤ REPO_X_MAX)。
        """
        self.set_seg("align", label)
        P = self.plan
        fwd, _ = gp._ring_frame(P.dir, (self.used() or {}).get("n"))
        sp = self.pos()
        t_end = rospy.Time.now().to_sec() + timeout
        reach = 0
        need = max(1, int(round(0.5 * C.GATE_LOOP_HZ)))
        dt = 0.0
        while True:
            now = rospy.Time.now().to_sec()
            p = self.pos()
            d = gp.vsub(S, sp)
            dn = gp.vnorm(d)
            step = C.GATE_REPO_V * dt
            sp = S if dn <= step else gp.vadd(sp, gp.vmul(d, step / max(dn, 1e-9)))
            lead = gp.vsub(sp, p)
            ln = gp.vnorm(lead)
            if ln > C.GATE_REPO_LEASH:
                sp = gp.vadd(p, gp.vmul(lead, C.GATE_REPO_LEASH / ln))
            if yaw_mode == "ring":
                c = (self.used() or {}).get("c") or P.aim
                if math.hypot(c[0] - p[0], c[1] - p[1]) > 0.5:
                    self.yaw_step(math.atan2(c[1] - p[1], c[0] - p[0]), dt)
            else:
                self.yaw_step(yaw_mode, dt)
            vff = gp.vmul(d, C.GATE_REPO_V / dn) if dn > step and ln <= C.GATE_REPO_LEASH else (0.0, 0.0, 0.0)
            F.setpoint.set_zhold_mode(sp[0], sp[1], sp[2], self.yaw_sp, vff[0], vff[1])
            x_ring = gp.vdot(gp.vsub(p, P.aim), fwd)
            if x_ring > C.REPO_X_MAX + 0.3:
                self.fail(f"摆位中离环平面只剩 {-x_ring:.2f}m (< {-C.REPO_X_MAX:.1f}m)")
            err = gp.vnorm(gp.vsub(S, p))
            reach = reach + 1 if (err < C.GATE_REPO_TOL and sp == S) else 0
            if reach >= need:
                return
            if now > t_end:
                self.fail(f"{label}: {timeout:.0f}s 没到位 (还差 {err:.2f}m)")
            if now - self.t_log > 1.0:
                self.t_log = now
                rospy.loginfo(f"    [{label}] 离起跑点 {err:.2f}m  离环平面 {-x_ring:.2f}m")
            dt = self.tick()

    def turn_to(self, yaw_target, hold_p, label, timeout=15.0):
        t_end = rospy.Time.now().to_sec() + timeout
        dt = 0.0
        while True:
            err = self.yaw_step(yaw_target, dt)
            self.hold(hold_p)
            if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) and \
                    abs(wrap_pi(yaw_target - self.yaw_sp)) < 1e-3:
                return
            if rospy.Time.now().to_sec() > t_end:
                self.fail(f"{label}: 转头 {timeout:.0f}s 没到位 (差 {math.degrees(err):+.0f}°)")
            dt = self.tick()

    # ---------------- ALIGN ----------------
    def align(self, hold_p):
        self.set_seg("align", f"在 {'原地 (影子)' if C.GATE_SHADOW else '起跑点'} 取样 {C.HOVER_ALIGN:.0f}s → 锁定")
        t_al = rospy.Time.now().to_sec() + 0.5          # 转头/刹停后先稳 0.5s
        last_why = ""
        while True:
            now = rospy.Time.now().to_sec()
            self.hold(hold_p)
            if now - t_al >= C.HOVER_ALIGN:
                p, v = self.pos(), self.vel()
                smp = F.gate_in.valid_since(max(t_al, now - C.HOVER_ALIGN))
                d_in = self.plan.dir if self.plan is not None and self.plan.dir is not None else \
                    (gp.hunit(gp.vsub(smp[-1]["c"], p)) if smp else None)
                f = gp.fuse_samples(smp, p, d=d_in) if d_in is not None else gp.fuse_samples(smp, p)
                self._raw_sigma(f, smp)
                why = []
                if f["k"] < C.GATE_ACQ_MIN_N:
                    why.append(f"有效帧 {f['k']} < {C.GATE_ACQ_MIN_N}")
                else:
                    # 横向 (h, v) 和深度分开判 (首飞问题 §1)
                    sh, sv, sa = f.get("spread_hva") or (f["spread_cm"],) * 3
                    if f.get("raw"):
                        # 每帧原始环心: 单帧散布大是正常的 (几 cm ~ 深度十几 cm), 精度看中值的标准误
                        # (下面的 σ 判定已经用了实际散布); 这里只拦散得离谱的 (野值 / 两个解来回跳)
                        sh, sv, sa = sh / 4.0, sv / 4.0, sa / 4.0
                    if max(sh, sv) > C.GATE_ALIGN_SPREAD_CM:
                        why.append(f"环心横向离散 {max(sh, sv):.1f}cm > {C.GATE_ALIGN_SPREAD_CM:.0f}")
                    if sa > C.GATE_ALIGN_SPREAD_ALONG_CM:
                        why.append(f"环心深度离散 {sa:.1f}cm > {C.GATE_ALIGN_SPREAD_ALONG_CM:.0f}")
                    ah_, av_, _ = f.get("aim_spread_hva") or (f.get("aim_spread_cm", 0.0),) * 3
                    if max(ah_, av_) > C.GATE_ALIGN_SPREAD_CM:
                        why.append(f"红点横向离散 {max(ah_, av_):.1f}cm > {C.GATE_ALIGN_SPREAD_CM:.0f}")
                    if self.acq and self.acq.get("n") and f.get("n"):
                        a = gp.ang_deg(self.acq["n"], f["n"])
                        if a > C.GATE_N_AGREE_DEG:
                            why.append(f"两个视角的法向差 {a:.1f}° > {C.GATE_N_AGREE_DEG:.0f}° (翻转解?)")
                    if gp.vnorm(v) > C.GATE_START_V_MAX:
                        why.append(f"飞机还在动 {gp.vnorm(v):.2f}m/s")
                    sg = f.get("sig_hva") or (gp.median([s["sd"] for s in smp]),) * 3
                    if max(sg[0], sg[1]) > C.GATE_LOCK_SD_LAT_CM:
                        why.append(f"识别横向 σ 中位数 {max(sg[0], sg[1]):.1f}cm > {C.GATE_LOCK_SD_LAT_CM:.0f}")
                    if sg[2] > C.GATE_LOCK_SD_ALONG_CM:
                        why.append(f"识别深度 σ 中位数 {sg[2]:.1f}cm > {C.GATE_LOCK_SD_ALONG_CM:.0f}")
                if not why:
                    self._sd_fields(f)
                    f["t_lock"] = now
                    f["locked"] = True
                    self.locked = f
                    c = f["c"]
                    shift = gp.vnorm(gp.vsub(c, self.acq["c"])) if self.acq else float("nan")
                    rospy.logwarn(f"[gate] ★ 锁定: 环心 ({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f}) "
                                  f"k={f['k']} 单帧散布 横{max(f['spread_hva'][:2]):.1f}/深{f['spread_hva'][2]:.1f}cm  "
                                  f"σ {self._sig_txt(f)}  比粗估挪了 {shift*100:.0f}cm"
                                  + (f"  红点离环心 {gp.vnorm(gp.vsub(f['aim'], c))*100:.0f}cm"
                                     if f.get("aim") else "  (没有红点, 瞄环心)"))
                    return
                last_why = "; ".join(why)
                if now - self.t_log > 1.0:
                    self.t_log = now
                    rospy.loginfo(f"    [align] 还不能锁定: {last_why}")
                if now - t_al > C.GATE_ALIGN_MAX_S:
                    self.fail(f"取样 {C.GATE_ALIGN_MAX_S:.0f}s 仍不收敛: {last_why}")
            self.tick()

    # ---------------- COMMIT ----------------
    def commit(self):
        P = self.plan
        self.set_seg("commit", "等确认")
        rospy.logwarn("=" * 64)
        rospy.logwarn(f"[gate] 计划: {P.summary()}")
        if P.v < C.DASH_V - 1e-9:
            rospy.logwarn(f"[gate] ★ 冲刺速度被场地压到 {P.v:.2f} m/s (设定 {C.DASH_V:.2f}) —— 原因见上面"
                          f"'场地限制'那行; 不接受就按 M 回手动段")
        if C.GATE_VIRTUAL_OFFSET > 0:
            rospy.logwarn(f"[gate] ★ 虚拟环: 会停在真纸面前约 {C.GATE_VIRTUAL_OFFSET - P.s_stop:.2f}m")
        need_y = bool(P.need_confirm)     # 降速了 → 只认 Y, G 不算
        go_key = 'y' if need_y else 'g'
        if need_y:
            rospy.logerr(f"[gate] ★★★ 降速: 计划 {P.v:.2f} m/s, 设定 {C.DASH_V:.2f} m/s —— 按 {C.DASH_V:.2f} 卡在: {P.slow_why}")
            rospy.logerr(f"[gate] ★★★ 接受 {P.v:.2f} m/s 就按 **Y** 冲刺 (G 不算)  |  不接受按 M 回手动段, 挪位置后 T 重来")
        if C.GATE_CONFIRM_DASH or need_y:
            if self.keys is None:
                self.fail("要按键确认冲刺, 但没有终端键盘")
            back = "回手动段" if (C.GATE_ABORT_TO_MANUAL and self.back_ok) else "降落"
            rospy.logwarn(f"[gate] ★★ 按 G 冲刺 (之后不能中止, 只有 X 停桨)  |  M / 方向键 = 不冲了, 回手动段  "
                          f"|  L = 降落  |  {C.GATE_CONFIRM_TIMEOUT:.0f}s 不按 = {back}")
        rospy.logwarn("=" * 64)
        t_end = rospy.Time.now().to_sec() + C.GATE_CONFIRM_TIMEOUT
        while C.GATE_CONFIRM_DASH or need_y:
            self.hold(P.S)
            self.tick()
            if go_key in self.flags:
                rospy.logwarn(f"[gate] {go_key.upper()} → 冲刺 {P.v:.2f} m/s")
                break
            if need_y and 'g' in self.flags:
                rospy.logwarn(f"[gate] 这次降速了 ({P.v:.2f} < {C.DASH_V:.2f} m/s), G 不算 —— 接受就按 Y, 不接受按 M")
            now = rospy.Time.now().to_sec()
            if now > t_end:
                self.fail(f"{C.GATE_CONFIRM_TIMEOUT:.0f}s 没按 {go_key.upper()}")
            if now - self.t_log > 5.0:
                self.t_log = now
                lv = F.gate_in.latest_valid()
                if lv is not None and self.locked:
                    d = gp.vnorm(gp.vsub(lv["c"], self.locked["c"])) * 100
                    rospy.loginfo(f"    [commit] 等 G … 最新识别和锁定值差 {d:.0f}cm")
        # 停稳在起跑点
        t_end = rospy.Time.now().to_sec() + C.GATE_START_WAIT
        while True:
            p, v = self.pos(), self.vel()
            e = gp.vnorm(gp.vsub(p, P.S))
            if e < C.GATE_START_TOL and gp.vnorm(v) < C.GATE_START_V_MAX:
                return
            if rospy.Time.now().to_sec() > t_end:
                self.fail(f"起跑前没停稳: 离 S {e:.2f}m, 速度 {gp.vnorm(v):.2f}m/s")
            self.hold(P.S)
            self.tick()

    # ---------------- DASH / BRAKE ----------------
    def dash(self):
        P = self.plan
        tr = P.traj
        aim, d = P.aim, P.dir
        yaw_d = gp.yaw_of(d)
        self.yaw_sp = yaw_d

        def sampler(t):
            s, v, a = tr.at(t)
            return gp.vadd(aim, gp.vmul(d, s)), gp.vmul(d, v), gp.vmul(d, a)

        set_phase("gate")                                  # 速度包络放开到冲刺档
        self.back_ok = False
        F.dash_tilt_on[0] = True                             # 停桨线换成 GATE_DASH_TILT_DEG
        self.t0 = rospy.Time.now().to_sec() + 0.05
        F.setpoint.set_traj_mode(sampler, self.t0, yaw_d, C.GATE_ACC_FF)
        self.set_seg("dash", f"t0, {P.v:.2f}m/s, 预计 {tr.T:.2f}s 后停在 s={P.s_stop:+.2f}")
        stuck_since = None
        t_wait_end = tr.T + 3.0 * C.T_LAG
        warned_dev = False
        while True:
            now = rospy.Time.now().to_sec()
            t = now - self.t0
            s_p, v_p, _ = tr.at(t)
            p, v = self.pos(), self.vel()
            s_e = gp.vdot(gp.vsub(p, aim), d)
            v_e = gp.vdot(v, d)
            self.dash_state = (t, s_p, v_p, s_e, v_e)
            if t >= tr.t2 and self.seg == "dash":
                self.set_seg("brake", f"s_plan={s_p:+.2f} s_est={s_e:+.2f} v={v_e:.2f}")
            if not self.passed and s_e > 0.0:
                self.passed = True
                rospy.logwarn(f"[gate] 过纸面 (估计) v={v_e:.2f}m/s  t={t:.2f}s")
            if abs(s_e - s_p) > C.GATE_DEV_WARN and not warned_dev:
                warned_dev = True
                rospy.logwarn(f"[gate] 估计 s 和规划差 {s_e - s_p:+.2f}m (只记录, 不改时间表)")
            # 卡纸: 机尾还没过纸面就被顶住。两种样子:
            #   ① 还在冲: 实测比规划慢 > DV, 且落后 > DS
            #   ② 规划已经慢下来 (刹车段后半): 落后 > DS 且实测几乎停了
            lag_s = s_p - s_e
            if C.GATE_STUCK_ENABLE and t > 0.3 and s_e < C.L_R and lag_s > C.GATE_STUCK_DS and \
                    (v_p - v_e > C.GATE_STUCK_DV or v_e < C.GATE_STOP_V):
                stuck_since = stuck_since if stuck_since is not None else now
                if now - stuck_since >= C.GATE_STUCK_HOLD:
                    self.lvl = 1
                    self.outcome = "stuck"
                    self.hold(p)
                    rospy.logerr(f"[gate] ★ 卡纸: 规划 s={s_p:+.2f} v={v_p:.2f} / 实测 s={s_e:+.2f} "
                                 f"v={v_e:.2f} —— 停止前推, 设定点 = 当前位置")
                    return
            else:
                stuck_since = None
            if t >= t_wait_end and math.hypot(v[0], v[1]) < C.GATE_STOP_V:
                break
            if t >= tr.T + C.GATE_STOP_EXTRA:
                rospy.logwarn(f"[gate] 轨迹结束 {C.GATE_STOP_EXTRA:.1f}s 还没停稳 "
                              f"(v={math.hypot(v[0], v[1]):.2f}m/s) —— 照样转悬停")
                break
            self.tick(dash=True)
        self.outcome = self.outcome or "done"
        rospy.logwarn(f"[gate] 刹停: 规划停在 s={P.s_stop:+.2f}, 估计 s={s_e:+.2f} "
                      f"(差 {s_e - P.s_stop:+.2f}m)  过纸面={'是' if self.passed else '否'}")


def phase_gate(z_hold):
    """
    穿环。返回 (x, y, z, yaw) —— 结束时的悬停点, main 从这里原地降落。

    流程 (2026-10-19): 手动段 ─T→ 找环 → 规划 → 摆位 → 锁定 → 等 G → 冲刺 → 刹停悬停
      冲刺前放弃 (GateStop.retry) 或按 M / 方向键 (GateBack) → 回手动段, 可以 T 重来;
      LAND / 接管 / 手动段超时 → 原地悬停后返回 (降落)。影子模式做完也回手动段。
      FATAL (X 键 / 倾角 / 断链) 照常抛 RuntimeError 给 main 的 cleanup。
    """
    set_phase("gate", vel=C.GATE_PRE_VEL_LIM)
    run = GateRun(z_hold)
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
                run.acquire()
                run.make_plan(run.acq, run.pos(), "粗规划")
                if C.GATE_SHADOW:
                    run.align(run.pos())
                    run.make_plan(run.locked, run.pos(), "锁定后规划 (影子)")
                    run.set_seg("hold", "影子模式: 不摆位不冲刺, 计划已发网页")
                    t_end = rospy.Time.now().to_sec() + 3.0
                    hp = run.pos()
                    while rospy.Time.now().to_sec() < t_end:
                        run.hold(hp)
                        run.tick()
                    run.outcome = "shadow"
                    if can_retry:
                        manual_on = True
                        why_manual = "影子模式做完了 (计划在网页上) —— 换个位置 T 再来, 或 L 降落"
                        continue
                else:
                    P = run.plan
                    run.move_to(P.S, "ring", C.GATE_REPO_TIMEOUT, "摆位")
                    run.turn_to(gp.yaw_of(P.dir), P.S, "对正进入方向")
                    run.align(P.S)
                    P2 = run.make_plan(run.locked, run.pos(), "锁定后规划")
                    gap = gp.vnorm(gp.vsub(P2.S, run.pos()))
                    if gap > C.GATE_START_TOL:
                        if gap > 0.5:
                            rospy.logwarn(f"[gate] 锁定后起跑点挪了 {gap:.2f}m —— 再挪过去 (不重新取样)")
                        run.move_to(P2.S, gp.yaw_of(P2.dir), C.GATE_REPO_TIMEOUT, "微调起跑点")
                    run.commit()
                    run.dash()
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
