# -*- coding: utf-8 -*-
"""
vb09/flight/steps/track.py —— 跟踪类目标 (判定插件 TRACKED = True, 现在是红点) 的步骤 (2026-11-08, claude/PLAN_dot_v2.md)
===============================================================================
感知侧 (桥里的 Locator) 一直在更新目标的世界系后验 (位置 + 诚实的协方差 + 法向 + 选点状态); 这里的步骤**每拍读最新值**,
不再"悬停取样 → 冻结 → 按冻结值飞"。步骤里没有目标类型判断, 只用样本的通用字段 (aim / n / sig / n_state / sel)。

  Detect      悬停, 机头转向目标; 等有候选 → 选点 (自动: 分数领先; 分不清: 终端数字键 1–5 / 网页点选) → 等桥回显
  Approach    抵近到停靠距离 D (停靠 = dock_dist; 冲刺 = 最小起跑距离): 每拍 S = aim − d·(D + k·σ_深度)
                d 从视线逐渐转到 −n (法向可信后, ≤ GATE_APPR_TURN_DPS); 远处深度不准 → 停在估计的近端, 越近越往前挪
                没估计 > GATE_APPR_LOST_S → 悬停等, > GATE_APPR_FAIL_S → 放弃; 估计一拍跳 > GATE_APPR_JUMP → 悬停 1 s
  Refine      到停靠点后判够不够准 (横向 σ ≤ GATE_LOCK_SD_LAT_CM、深度 σ ≤ GATE_LOCK_SD_ALONG_CM、
              法向可信或判为近正对); 不够 → 侧移 ±GATE_REFINE_DY (法向"两个解都说得通"时改上下) 再判,
              最多 GATE_REFINE_ROUNDS 轮; 法向定了而停靠方向差 > 3° → 挪到法线上再判。够了 → 锁定 (= 最新后验, 不取中位数)
  PlanLocked  (冲刺) 用锁定值规划; 起跑点离现在 > GATE_START_TOL → 挪过去 + 对正
  CommitTrack (冲刺) 同 dash.Commit, 按 G 时最新估计和锁定值横向差 > GATE_RELOCK_CM → 按最新值重锁、重规划、再等一次 G
停靠的最后一步沿用 dock.Hold (读 lock.standoff / lock.d_in)。
"""
import math

import rospy
import gate_plan as gp
from vb09.common import geom
from vb09.common.target import Lock
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.steps.common import move_to, turn_to
from vb09.flight.steps.dash import Commit, make_plan
from vb09.flight.devices import wrap_pi


def _r(v, nd=3):
    return [round(float(x), nd) for x in v] if v is not None else None


def _status(ctx):
    """最新一条原始 /gate/status (dict) 和它多旧 (s)。"""
    st, age = ctx.gate_in.latest()
    return (st or {}), age


def _fresh(ctx, max_age):
    """最新的有效样本 (judge 通过) 且不旧于 max_age → 样本 / None。"""
    s = ctx.gate_in.latest_valid()
    if s is None or rospy.Time.now().to_sec() - s["recv"] > max_age:
        return None
    return s


def _lock_from(s, kind):
    """样本 → Lock (锁定 / 粗估用)。sig_hva 是桥后验拆到靶面的 σ (cm)。"""
    sg = s.get("sig")
    f = {"c": s["c"], "aim": s["aim"], "n": s.get("n"), "k": 1,
         "spread_cm": 0.0, "spread_hva": (0.0, 0.0, 0.0), "aim_spread_hva": (0.0, 0.0, 0.0)}
    if sg:
        f["sig_hva"] = tuple(float(x) for x in sg)
        f["sd_hva_cm"] = [round(float(x), 2) for x in sg]
        f["sd_cm_xyz"] = [round(max(sg[0], sg[1]), 2)] * 3
    if s.get("n_sd") is not None:
        f["n_spread_deg"] = s.get("n_sd")
    return Lock(f, kind=kind, trk_id=(s.get("sel") or {}).get("trk_id"))


def _turn_toward(d_cur, d_goal, max_rad):
    """水平单位向量 d_cur 朝 d_goal 最多转 max_rad。"""
    a = math.atan2(d_cur[1], d_cur[0])
    b = math.atan2(d_goal[1], d_goal[0])
    e = wrap_pi(b - a)
    a += max(-max_rad, min(max_rad, e))
    return (math.cos(a), math.sin(a), 0.0)


def _track_info(ctx):
    if ctx.track is None:
        ctx.track = {}
    return ctx.track


# =============================================================================
class Detect(Step):
    """发现 + 选点。结束时 ctx.acq = 粗估 (选中的那个), 返回 Lock。"""
    name = "wait"
    vel_lim = "pre"

    def run(self, ctx, lock):
        kind = C.TARGET_KIND
        ctx.set_seg("wait", "找目标 / 选点")
        info = _track_info(ctx)
        info.update(step="detect", sel=None, cands=None, why=None)
        t0 = rospy.Time.now().to_sec()
        hp = (ctx.p_hover[0], ctx.p_hover[1], ctx.z_hold)
        my_seq = None
        t_sent = None
        ask_t = None
        last_why = "还没看到目标"
        dt = 0.0
        while True:
            now = rospy.Time.now().to_sec()
            st, age = _status(ctx)
            fresh_st = age < C.GATE_STATUS_MAX_AGE
            cands = [c for c in (st.get("cands") or []) if c.get("aim")] if fresh_st else []
            sel = st.get("sel") or {}
            info.update(sel=sel, cands=[{"id": c.get("id"), "score": c.get("score"), "hits": c.get("hits")}
                                        for c in cands])
            # 机头: 选中的 > 最高分的
            tgt = None
            s_ok = _fresh(ctx, C.GATE_STATUS_MAX_AGE)
            if s_ok is not None:
                tgt = s_ok["aim"]
            elif cands:
                tgt = cands[0]["aim"]
            p = ctx.pos()
            if tgt is not None and math.hypot(tgt[0] - p[0], tgt[1] - p[1]) > 0.5:
                ctx.yaw_step(math.atan2(tgt[1] - p[1], tgt[0] - p[0]), dt)
            ctx.hold(hp)
            # 1) 已选上 (桥回显了我的 seq, 状态 bound) 且样本有效 → 结束
            if my_seq is not None and sel.get("seq") == my_seq and sel.get("state") == "bound" and s_ok is not None:
                ctx.acq = _lock_from(s_ok, kind)
                d = geom.vnorm(geom.vsub(s_ok["aim"], p))
                rospy.loginfo(f"[gate] 选定目标 轨迹 {sel.get('trk_id')}: ({s_ok['aim'][0]:+.2f},{s_ok['aim'][1]:+.2f},"
                              f"{s_ok['aim'][2]:+.2f}) 距离 {d:.1f}m  σ {ctx._sig_txt(ctx.acq)}")
                info.update(why=None)
                return ctx.acq
            # 2) 终端数字键: 随时可选 (覆盖自动)
            pick = next((int(k) for k in ctx.flags if k in "12345"), None)
            if pick is not None:
                if pick <= len(cands):
                    my_seq = ctx.gate_in.send_select({"mode": "track", "trk_id": int(cands[pick - 1]["id"])},
                                                     why="终端键 %d" % pick)
                    t_sent = now
                else:
                    rospy.logwarn(f"[gate] 键 {pick}: 现在只有 {len(cands)} 个候选")
            # 3) 网页点选 (经 /flight_ctrl → GateInput.web_select)
            req = ctx.gate_in.take_web_select()
            if req is not None:
                my_seq = ctx.gate_in.send_select(req, why="网页")
                t_sent = now
            # 4) 自动选点
            if my_seq is None and cands:
                top = max(cands, key=lambda c: float(c.get("score") or 0.0))
                rivals = [c for c in cands if c is not top
                          and float(c.get("score") or 0.0) >= C.DOT_SEL_RATIO * float(top.get("score") or 0.0)
                          and geom.vnorm(geom.vsub(c["aim"], top["aim"])) > 0.5]
                good = float(top.get("score") or 0.0) >= C.DOT_SEL_SCORE and int(top.get("hits") or 0) >= C.DOT_SEL_HITS
                if good and not rivals:
                    my_seq = ctx.gate_in.send_select({"mode": "track", "trk_id": int(top["id"])},
                                                     why="自动 (分数 %.2f, 命中 %d)" % (top["score"], top["hits"]))
                    t_sent = now
                elif good and rivals:
                    if ask_t is None:
                        ask_t = now
                        rospy.logwarn("[gate] ★ 有 %d 个候选分数接近, 分不清 —— 按数字键选 (或网页点选):" % len(cands))
                        for i, c in enumerate(cands[:5]):
                            rospy.logwarn(f"[gate]     {i + 1}: 轨迹 {c['id']}  分数 {c.get('score')}  命中 {c.get('hits')}  "
                                          f"({c['aim'][0]:+.2f},{c['aim'][1]:+.2f},{c['aim'][2]:+.2f})  "
                                          f"D {c.get('D_trk') or c.get('D')}")
                    info.update(why="分不清, 等人选")
                    last_why = "%d 个候选分数接近, 等人选 (数字键 / 网页)" % (1 + len(rivals))
                    if now - ask_t > C.GATE_SELECT_TIMEOUT:
                        ctx.fail(f"{C.GATE_SELECT_TIMEOUT:.0f}s 没人选点")
                else:
                    last_why = "候选分数 %.2f / 命中 %d 还不够自动选" % (float(top.get("score") or 0), int(top.get("hits") or 0))
            elif not cands:
                last_why = "没看到目标"
            # 5) 发了没回显 → 重发 (桥重启 / 丢包); 选了但挂不上 (像素选点等候选)
            if my_seq is not None and sel.get("seq") != my_seq and t_sent is not None and now - t_sent > 2.0:
                rospy.logwarn(f"[gate] 选点 seq {my_seq} 2s 没回显 (桥没收到? 版本不对?) —— 重新选")
                my_seq = None
            elif my_seq is not None and sel.get("seq") == my_seq and sel.get("state") in ("pending", "lost"):
                last_why = "选点已收到, 等目标出现 (%s)" % sel.get("state")
            if now - t0 > C.GATE_ACQ_TIMEOUT and my_seq is None and ask_t is None:
                ctx.fail(f"{C.GATE_ACQ_TIMEOUT:.0f}s 内没选上目标 ({last_why})")
            if now - ctx.t_log > 1.0:
                ctx.t_log = now
                rospy.loginfo(f"    [detect] {last_why}  候选 {len(cands)}  选点 {sel.get('state', '-')}")
            dt = ctx.tick()


# =============================================================================
class Approach(Step):
    """边飞边估: 每拍按最新后验重算停靠点。子类给停靠距离。"""
    name = "align"
    vel_lim = "pre"
    LABEL = "抵近"

    def d_stop(self, ctx):
        raise NotImplementedError

    def run(self, ctx, lock):
        D = float(self.d_stop(ctx))
        info = _track_info(ctx)
        info.update(step="approach", d_stop=round(D, 3), S=None, sd_along_cm=None, dir_src=None, why=None)
        ctx.set_seg("align", f"{self.LABEL}到目标前 {D:.2f}m (边飞边估)")
        p = ctx.pos()
        aim0 = (lock or ctx.acq or {}).get("aim")
        if aim0 is None:
            ctx.fail("抵近: 没有目标估计")
        d_cur = geom.hunit(geom.vsub(aim0, p))
        if d_cur is None:
            ctx.fail("抵近: 目标在正上 / 正下方")
        sp = p
        t0 = rospy.Time.now().to_sec()
        t_seen = t0
        hold_until = 0.0
        aim_prev = None
        reach = 0
        need = max(1, int(round(0.5 * C.GATE_LOOP_HZ)))
        o = ctx.origin
        dt = 0.0
        s = None
        while True:
            now = rospy.Time.now().to_sec()
            p = ctx.pos()
            s = _fresh(ctx, C.GATE_APPR_LOST_S)
            if s is None:
                ctx.hold(sp)
                lost = now - t_seen
                info.update(why="没有有效估计 %.1fs" % lost)
                if lost > C.GATE_APPR_FAIL_S:
                    ctx.fail(f"{self.LABEL}: {lost:.0f}s 没有有效估计")
                if now - ctx.t_log > 1.0:
                    ctx.t_log = now
                    rospy.logwarn(f"    [{self.LABEL}] 没有有效估计 {lost:.1f}s —— 悬停等")
                dt = ctx.tick()
                continue
            t_seen = now
            aim = s["aim"]
            if aim_prev is not None and geom.vnorm(geom.vsub(aim, aim_prev)) > C.GATE_APPR_JUMP:
                rospy.logwarn(f"[gate] {self.LABEL}: 目标估计跳了 {geom.vnorm(geom.vsub(aim, aim_prev)):.2f}m —— 悬停 1s")
                hold_until = now + 1.0
            aim_prev = aim
            # 进入方向: 法向可信 → −n; 否则视线。限速转
            n = s.get("n")
            d_goal, src = (geom.hunit(geom.vmul(n, -1.0)), "法向") if n is not None else (None, "视线")
            if d_goal is None:
                d_goal, src = geom.hunit(geom.vsub(aim, p)) or d_cur, "视线"
            d_cur = _turn_toward(d_cur, d_goal, math.radians(C.GATE_APPR_TURN_DPS) * max(dt, 1e-3))
            sig = s.get("sig") or (99.0, 99.0, 99.0)
            sd_al = float(sig[2]) / 100.0
            dist_cmd = D + C.GATE_APPR_K * sd_al
            S = (aim[0] - d_cur[0] * dist_cmd, aim[1] - d_cur[1] * dist_cmd, aim[2])
            S = (S[0], S[1], o[2] + max(C.GATE_Z_MIN_DZ, min(C.GATE_Z_MAX_DZ, S[2] - o[2])))
            r_xy = math.hypot(S[0] - o[0], S[1] - o[1])
            if r_xy > C.GATE_XY_MAX:
                ctx.fail(f"{self.LABEL}: 停靠点离起飞点水平 {r_xy:.2f}m > 水平范围 {C.GATE_XY_MAX:.2f}m")
            info.update(S=_r(S), aim=_r(aim), sd_along_cm=round(sd_al * 100, 1), dist_cmd=round(dist_cmd, 2),
                        dir_src=src, dir=_r(d_cur, 4), why=None)
            # setpoint 斜坡 (同 move_to: 速度 GATE_REPO_V, 牵引绳 GATE_REPO_LEASH)
            if now < hold_until:
                ctx.hold(sp)
            else:
                dv = geom.vsub(S, sp)
                dn = geom.vnorm(dv)
                step = C.GATE_REPO_V * dt
                sp = S if dn <= step else geom.vadd(sp, geom.vmul(dv, step / max(dn, 1e-9)))
                lead = geom.vsub(sp, p)
                ln = geom.vnorm(lead)
                if ln > C.GATE_REPO_LEASH:
                    sp = geom.vadd(p, geom.vmul(lead, C.GATE_REPO_LEASH / ln))
                vff = geom.vmul(dv, C.GATE_REPO_V / dn) if dn > step and ln <= C.GATE_REPO_LEASH else (0.0, 0.0, 0.0)
                if math.hypot(aim[0] - p[0], aim[1] - p[1]) > 0.5:
                    ctx.yaw_step(math.atan2(aim[1] - p[1], aim[0] - p[0]), dt)
                ctx.setpoint.set_zhold_mode(sp[0], sp[1], sp[2], ctx.yaw_sp, vff[0], vff[1])
            # 到位: 离 (最新) S 够近、深度 σ 够小、方向已转到位
            err = geom.vnorm(geom.vsub(S, p))
            aligned = gp.ang_deg(d_cur, d_goal) < 2.0
            ok_sd = sd_al * 100 <= C.GATE_LOCK_SD_ALONG_CM
            # S 每拍随估计抖动 (几 mm–cm), 不能要求 setpoint 正好等于 S
            reach = reach + 1 if (err < C.GATE_REPO_TOL and aligned and ok_sd
                                  and geom.vnorm(geom.vsub(S, sp)) < 0.05) else 0
            if reach >= need:
                Sf = (aim[0] - d_cur[0] * D, aim[1] - d_cur[1] * D, S[2])
                lock = _lock_from(s, C.TARGET_KIND)
                lock.standoff, lock.d_in = Sf, d_cur
                ctx.acq = lock
                rospy.loginfo(f"[gate] {self.LABEL}到位: 目标 ({aim[0]:+.2f},{aim[1]:+.2f},{aim[2]:+.2f}) σ {ctx._sig_txt(lock)}"
                              f"  方向按{src}  用时 {now - t0:.0f}s")
                return lock
            if now - t0 > C.GATE_APPR_TIMEOUT:
                ctx.fail(f"{self.LABEL}: {C.GATE_APPR_TIMEOUT:.0f}s 没到位 (离停靠点 {err:.2f}m, 深度 σ {sd_al * 100:.0f}cm)")
            if now - ctx.t_log > 1.0:
                ctx.t_log = now
                rospy.loginfo(f"    [{self.LABEL}] 离目标 {geom.vnorm(geom.vsub(aim, p)):.2f}m  离停靠点 {err:.2f}m  "
                              f"σ {ctx._sig_txt({'sig_hva': sig})}  停在 {dist_cmd:.2f}m (方向按{src})")
            dt = ctx.tick()


class ApproachDock(Approach):
    """停靠模式: 停靠距离 = dock_dist。"""
    LABEL = "抵近"

    def d_stop(self, ctx):
        return C.GATE_DOCK_D


class ApproachDash(Approach):
    """冲刺模式: 停靠距离 = 最小起跑距离 (gate_plan.d_run_for(DASH_V)), 不超过 D_RUN_MAX。"""
    LABEL = "抵近起跑点"

    def d_stop(self, ctx):
        return gp.d_run_for(C.DASH_V, ctx.params)


# =============================================================================
class Refine(Step):
    """够准就锁; 不够就换视角。"""
    name = "align"
    vel_lim = "pre"

    @staticmethod
    def _judge(s, still):
        why = []
        if s is None:
            return ["没有有效估计"]
        sg = s.get("sig") or (99.0, 99.0, 99.0)
        if max(sg[0], sg[1]) > C.GATE_LOCK_SD_LAT_CM:
            why.append(f"横向 σ {max(sg[0], sg[1]):.1f}cm > {C.GATE_LOCK_SD_LAT_CM:.0f}")
        if sg[2] > C.GATE_LOCK_SD_ALONG_CM:
            why.append(f"深度 σ {sg[2]:.1f}cm > {C.GATE_LOCK_SD_ALONG_CM:.0f}")
        if s.get("n") is None and s.get("n_state") != "frontal":
            why.append("法向未定 (%s)" % (s.get("n_why") or s.get("n_state") or "?"))
        if not still:
            why.append("飞机还在动")
        return why

    def _sample(self, ctx, S, T):
        """在 S 悬停 T 秒 (先稳 0.5 s), 返回最新样本。"""
        t_end = rospy.Time.now().to_sec() + 0.5 + T
        while rospy.Time.now().to_sec() < t_end:
            ctx.hold(S)
            ctx.tick()
        return _fresh(ctx, C.GATE_STATUS_MAX_AGE)

    def run(self, ctx, lock):
        info = _track_info(ctx)
        info.update(step="refine", rounds=0, why=None)
        S = getattr(lock, "standoff", None) or ctx.pos()
        d_in = getattr(lock, "d_in", None) or geom.hunit(geom.vsub(lock.aim, S))
        D = geom.vnorm(geom.vsub((lock.aim[0], lock.aim[1], S[2]), S))
        o = ctx.origin
        move_to(ctx, S, "ring", C.GATE_REPO_TIMEOUT, "停靠点", ref=(lock.aim, d_in))
        turn_to(ctx, gp.yaw_of(d_in), S, "对正目标")
        ctx.set_seg("align", "判够不够准 → 锁定")
        rounds = 0
        moved_to_normal = False
        while True:
            s = self._sample(ctx, S, C.GATE_REFINE_WAIT_S)
            still = geom.vnorm(ctx.vel()) <= C.GATE_START_V_MAX
            why = self._judge(s, still)
            # 法向出来了, 停靠方向和 −n 差 > 3° → 挪到法线上 (法向误差只影响进入角, 不影响沿视线的命中)
            if s is not None and s.get("n") is not None and not moved_to_normal:
                dn = geom.hunit(geom.vmul(s["n"], -1.0))
                if dn is not None and gp.ang_deg(dn, d_in) > 3.0:
                    aim = s["aim"]
                    S2 = (aim[0] - dn[0] * D, aim[1] - dn[1] * D, S[2])
                    rospy.loginfo(f"[gate] 法向定了, 停靠方向差 {gp.ang_deg(dn, d_in):.1f}° —— 挪到法线上 "
                                  f"({geom.vnorm(geom.vsub(S2, S)):.2f}m)")
                    move_to(ctx, S2, "ring", C.GATE_REPO_TIMEOUT, "挪到法线上", ref=(aim, dn))
                    turn_to(ctx, gp.yaw_of(dn), S2, "对正法向")
                    S, d_in = S2, dn
                    moved_to_normal = True
                    continue
            info.update(why="; ".join(why) or None, rounds=rounds,
                        sig=_r(s.get("sig"), 1) if s else None, n_state=(s or {}).get("n_state"))
            if not why:
                f = _lock_from(s, C.TARGET_KIND)
                f["t_lock"] = rospy.Time.now().to_sec()
                f["locked"] = True
                ctx.locked = f
                f.standoff, f.d_in = S, d_in
                a = s["aim"]
                shift = geom.vnorm(geom.vsub(a, ctx.acq["aim"])) if ctx.acq and ctx.acq.get("aim") else float("nan")
                rospy.logwarn(f"[gate] ★ 锁定: 目标 ({a[0]:+.3f},{a[1]:+.3f},{a[2]:+.3f})  σ {ctx._sig_txt(f)}  "
                              f"法向 {'有 ±%.1f°' % s['n_sd'] if s.get('n') is not None and s.get('n_sd') else ('近正对, 按视线' if s.get('n') is None else '有')}"
                              f"  比抵近时挪了 {shift * 100:.0f}cm  换视角 {rounds} 轮")
                info.update(step="locked", why=None)
                return f
            if rounds >= C.GATE_REFINE_ROUNDS:
                ctx.fail("换了 %d 轮视角还不够准: %s" % (rounds, "; ".join(why)))
            rounds += 1
            # 换视角: 默认侧移; 法向"两个解都说得通"且已经侧移过 → 上下
            vertical = (rounds >= 2 and s is not None and s.get("n_state") == "ambig")
            dy = C.GATE_REFINE_DY
            if vertical:
                z_lo = max(S[2] - dy, o[2] + C.TELEOP_DZ_MIN)
                z_hi = min(S[2] + dy, o[2] + C.TELEOP_DZ_MAX)
                pts = [(S[0], S[1], z_lo), (S[0], S[1], z_hi), S]
                lab = "上下"
            else:
                right = (d_in[1], -d_in[0], 0.0)
                pts = [geom.vadd(S, geom.vmul(right, dy)), geom.vadd(S, geom.vmul(right, -dy)), S]
                lab = "左右"
            span = math.degrees(2.0 * math.atan2(dy, max(D, 0.5)))
            rospy.logwarn(f"[gate] 还不够准 ({'; '.join(why)}) —— 第 {rounds} 轮: {lab}移 ±{dy:.2f}m (视角跨度 ≈ {span:.0f}°)")
            ctx.set_seg("align", f"换视角: {lab}移 ±{dy:.2f}m")
            for q, nm in zip(pts, ("去", "回", "回停靠点")):
                move_to(ctx, q, "ring", C.GATE_REPO_TIMEOUT, f"{lab}移 {nm}", ref=(lock.aim, d_in), v=C.GATE_VSCAN_V)
            ctx.set_seg("align", "判够不够准 → 锁定")


# =============================================================================
class PlanLocked(Step):
    """冲刺: 用锁定值规划 → 起跑点挪了就挪过去 → 对正进入方向。"""
    name = "align"
    vel_lim = "pre"

    def run(self, ctx, lock):
        P = make_plan(ctx, ctx.locked, ctx.pos(), "锁定后规划")
        gap = gp.vnorm(gp.vsub(P.S, ctx.pos()))
        if gap > C.GATE_START_TOL:
            move_to(ctx, P.S, gp.yaw_of(P.dir), C.GATE_REPO_TIMEOUT, "到起跑点")
        turn_to(ctx, gp.yaw_of(P.dir), P.S, "对正进入方向")
        return lock


class CommitTrack(Commit):
    """同 Commit (等 G / Y、停稳), 按 G 时最新估计和锁定值横向差 > GATE_RELOCK_CM → 重锁、重规划、再等一次 G。"""

    def on_go(self, ctx):
        s = _fresh(ctx, C.GATE_STATUS_MAX_AGE)
        L = ctx.locked
        P = ctx.plan
        if s is None or L is None or P is None or P.dir is None:
            return True
        r_, u_, _ = gp.entry_axes(P.dir)
        dv = gp.vsub(s["aim"], L["aim"])
        lat = math.hypot(gp.vdot(dv, r_), gp.vdot(dv, u_)) * 100
        if lat <= C.GATE_RELOCK_CM:
            return True
        rospy.logwarn(f"[gate] ★ 最新估计和锁定值横向差 {lat:.1f}cm > {C.GATE_RELOCK_CM:.0f} —— 按最新值重锁、重规划, 再按一次")
        f = _lock_from(s, C.TARGET_KIND)
        f["t_lock"] = rospy.Time.now().to_sec()
        f["locked"] = True
        f.standoff, f.d_in = getattr(L, "standoff", None), getattr(L, "d_in", None)
        ctx.locked = f
        P2 = make_plan(ctx, f, ctx.pos(), "重锁后规划")
        if gp.vnorm(gp.vsub(P2.S, ctx.pos())) > C.GATE_START_TOL:
            move_to(ctx, P2.S, gp.yaw_of(P2.dir), C.GATE_REPO_TIMEOUT, "微调起跑点")
        turn_to(ctx, gp.yaw_of(P2.dir), P2.S, "对正进入方向")
        _track_info(ctx).update(relock=round(lat, 1))
        return False
