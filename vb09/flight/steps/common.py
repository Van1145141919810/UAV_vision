# -*- coding: utf-8 -*-
"""
vb09/flight/steps/common.py —— 各模式共用的步骤和动作
===============================================================================
2026-11-01 由 acquire.py / reposition.py / align.py 合并 (代码原样)。
  Acquire        原地悬停, 机头转向目标, 收够有效帧 → 粗估 (Lock)
  move_to        以 GATE_REPO_V 斜坡直线飞到一点 (机头盯目标 / 固定朝向, 全程在目标前 ≥ |REPO_X_MAX|)
  turn_to        原地转头
  align_lock     悬停取样 → 收敛判定 → 锁定 (Lock)
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock / ctx.gate_in 读目标。
"""
import math
import rospy
import gate_plan as gp
from vb09.common.target import Lock
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.telemetry import wrap_pi


class Acquire(Step):
    """原 GateRun.acquire。结束时 ctx.acq = 粗估, 返回同一个 Lock。"""
    name = "wait"
    vel_lim = "pre"

    def run(self, ctx, lock):
        ctx.set_seg("wait", f"悬停识别 (窗口 {C.HOVER_ACQ:.0f}s, 至少 {C.GATE_ACQ_MIN_N} 帧)")
        t_start = rospy.Time.now().to_sec()
        yaw0 = ctx.yaw_sp
        scan = [yaw0 + math.radians(C.GATE_SCAN_DEG), yaw0 - math.radians(C.GATE_SCAN_DEG), yaw0]
        scan_i = None                      # None = 还没开始扫; len(scan) = 扫完了
        hp = (ctx.p_hover[0], ctx.p_hover[1], ctx.z_hold)
        ctx.p_hover = hp
        dt = 0.0
        tol = math.radians(C.GATE_YAW_TOL_DEG)
        last_why = "还没看到环"
        rej_snap, _ = ctx.gate_in.rej_since(None)
        while True:
            now = rospy.Time.now().to_sec()
            p = ctx.pos()
            # 只用进入 ACQUIRE 之后收到的帧 (起飞爬升中的帧不算)
            smp = ctx.gate_in.valid_since(max(t_start, now - C.HOVER_ACQ))
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
                target = ctx.yaw_sp
            err = ctx.yaw_step(target, dt)
            if scanning and abs(wrap_pi(target - ctx.yaw_sp)) < 1e-3 and abs(err) < tol:
                scan_i += 1
            ctx.hold(hp)
            if len(smp) >= C.GATE_ACQ_MIN_N and fresh and abs(err) < math.radians(C.GATE_YAW_TOL_DEG):
                d_los = gp.hunit(gp.vsub(fresh[-1]["c"], p)) or (math.cos(ctx.yaw_sp), math.sin(ctx.yaw_sp), 0.0)
                f = gp.fuse_samples(smp, p, d=d_los)
                ctx._raw_sigma(f, smp)
                sh, sv, sa = f["spread_hva"]
                if f.get("raw"):                     # 每帧原始环心: 散布本来就大, 看中值的标准误
                    sh, sv, sa = (x * 2.56 / 1.25 for x in f["sig_hva"])
                if max(sh, sv) <= C.GATE_ACQ_SPREAD_CM and sa <= 2.0 * C.GATE_ACQ_SPREAD_CM:
                    ctx._sd_fields(f)
                    ctx.acq = f = Lock(f, kind="ring")
                    rospy.loginfo(f"[gate] 粗估: 环心 ({f['c'][0]:+.2f},{f['c'][1]:+.2f},{f['c'][2]:+.2f}) "
                                  f"k={f['k']} 离散 横{max(sh, sv):.1f}/深{sa:.1f}cm  σ {ctx._sig_txt(f)}  "
                                  f"法向 {'有' if f.get('n') else '无'}  红点 {'有' if f.get('aim') else '无'}")
                    return f
                last_why = (f"样本离散 横{max(sh, sv):.1f}/深{sa:.1f}cm > "
                            f"{C.GATE_ACQ_SPREAD_CM:.0f}/{2 * C.GATE_ACQ_SPREAD_CM:.0f}cm")
            elif smp:
                last_why = f"有效帧 {len(smp)}/{C.GATE_ACQ_MIN_N}" + (
                    "" if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) else
                    f", 机头还差 {math.degrees(err):+.0f}°")
            if now - t_start > C.GATE_ACQ_TIMEOUT:
                nv, nr, rej = ctx.gate_in.counters()
                ctx.fail(f"{C.GATE_ACQ_TIMEOUT:.0f}s 内没拿到稳定识别 ({last_why}; "
                          f"拒绝原因 {', '.join(f'{k}={v}' for k, v in rej.items() if v)})")
            if now - ctx.t_log > 1.0:
                ctx.t_log = now
                rej_snap, rtxt = ctx.gate_in.rej_since(rej_snap)
                if not smp and rtxt:
                    last_why = rtxt
                rospy.loginfo(f"    [wait] {last_why}")
            dt = ctx.tick()


def move_to(ctx, S, yaw_mode, timeout, label, ref=None):
    """
    setpoint 以 GATE_REPO_V 斜坡走直线到 S (3D, 带牵引绳)。yaw_mode:
      "ring" = 机头盯环心 (保持环在视野里)   数值 = 固定朝向
    全程查"飞机在环前 ≥1.5m" (环系 x ≤ REPO_X_MAX)。
    ref = (aim, dir): 不用冲刺计划时 (停靠) 给目标点和进入方向; 不给 = 用 ctx.plan (冲刺, 和以前一样)。
    """
    ctx.set_seg("align", label)
    if ref is None:
        P = ctx.plan
        ref_aim, ref_dir, goal = P.aim, P.dir, "起跑点"
    else:
        (ref_aim, ref_dir), goal = ref, "停靠点"
    fwd, _ = gp._ring_frame(ref_dir, (ctx.used() or {}).get("n"))
    sp = ctx.pos()
    t_end = rospy.Time.now().to_sec() + timeout
    reach = 0
    need = max(1, int(round(0.5 * C.GATE_LOOP_HZ)))
    dt = 0.0
    while True:
        now = rospy.Time.now().to_sec()
        p = ctx.pos()
        d = gp.vsub(S, sp)
        dn = gp.vnorm(d)
        step = C.GATE_REPO_V * dt
        sp = S if dn <= step else gp.vadd(sp, gp.vmul(d, step / max(dn, 1e-9)))
        lead = gp.vsub(sp, p)
        ln = gp.vnorm(lead)
        if ln > C.GATE_REPO_LEASH:
            sp = gp.vadd(p, gp.vmul(lead, C.GATE_REPO_LEASH / ln))
        if yaw_mode == "ring":
            c = (ctx.used() or {}).get("c") or ref_aim
            if math.hypot(c[0] - p[0], c[1] - p[1]) > 0.5:
                ctx.yaw_step(math.atan2(c[1] - p[1], c[0] - p[0]), dt)
        else:
            ctx.yaw_step(yaw_mode, dt)
        vff = gp.vmul(d, C.GATE_REPO_V / dn) if dn > step and ln <= C.GATE_REPO_LEASH else (0.0, 0.0, 0.0)
        ctx.setpoint.set_zhold_mode(sp[0], sp[1], sp[2], ctx.yaw_sp, vff[0], vff[1])
        x_ring = gp.vdot(gp.vsub(p, ref_aim), fwd)
        if x_ring > C.REPO_X_MAX + 0.3:
            ctx.fail(f"摆位中离环平面只剩 {-x_ring:.2f}m (< {-C.REPO_X_MAX:.1f}m)")
        err = gp.vnorm(gp.vsub(S, p))
        reach = reach + 1 if (err < C.GATE_REPO_TOL and sp == S) else 0
        if reach >= need:
            return
        if now > t_end:
            ctx.fail(f"{label}: {timeout:.0f}s 没到位 (还差 {err:.2f}m)")
        if now - ctx.t_log > 1.0:
            ctx.t_log = now
            rospy.loginfo(f"    [{label}] 离{goal} {err:.2f}m  离环平面 {-x_ring:.2f}m")
        dt = ctx.tick()


def turn_to(ctx, yaw_target, hold_p, label, timeout=15.0):
    t_end = rospy.Time.now().to_sec() + timeout
    dt = 0.0
    while True:
        err = ctx.yaw_step(yaw_target, dt)
        ctx.hold(hold_p)
        if abs(err) < math.radians(C.GATE_YAW_TOL_DEG) and \
                abs(wrap_pi(yaw_target - ctx.yaw_sp)) < 1e-3:
            return
        if rospy.Time.now().to_sec() > t_end:
            ctx.fail(f"{label}: 转头 {timeout:.0f}s 没到位 (差 {math.degrees(err):+.0f}°)")
        dt = ctx.tick()


def align_lock(ctx, hold_p):
    ctx.set_seg("align", f"在 {'原地 (影子)' if C.GATE_SHADOW else '起跑点'} 取样 {C.HOVER_ALIGN:.0f}s → 锁定")
    t_al = rospy.Time.now().to_sec() + 0.5          # 转头/刹停后先稳 0.5s
    last_why = ""
    while True:
        now = rospy.Time.now().to_sec()
        ctx.hold(hold_p)
        if now - t_al >= C.HOVER_ALIGN:
            p, v = ctx.pos(), ctx.vel()
            smp = ctx.gate_in.valid_since(max(t_al, now - C.HOVER_ALIGN))
            d_in = ctx.plan.dir if ctx.plan is not None and ctx.plan.dir is not None else \
                (gp.hunit(gp.vsub(smp[-1]["c"], p)) if smp else None)
            f = gp.fuse_samples(smp, p, d=d_in) if d_in is not None else gp.fuse_samples(smp, p)
            ctx._raw_sigma(f, smp)
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
                if ctx.acq and ctx.acq.get("n") and f.get("n"):
                    a = gp.ang_deg(ctx.acq["n"], f["n"])
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
                ctx._sd_fields(f)
                f["t_lock"] = now
                f["locked"] = True
                ctx.locked = f = Lock(f, kind="ring")
                c = f["c"]
                shift = gp.vnorm(gp.vsub(c, ctx.acq["c"])) if ctx.acq else float("nan")
                rospy.logwarn(f"[gate] ★ 锁定: 环心 ({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f}) "
                              f"k={f['k']} 单帧散布 横{max(f['spread_hva'][:2]):.1f}/深{f['spread_hva'][2]:.1f}cm  "
                              f"σ {ctx._sig_txt(f)}  比粗估挪了 {shift*100:.0f}cm"
                              + (f"  红点离环心 {gp.vnorm(gp.vsub(f['aim'], c))*100:.0f}cm"
                                 if f.get("aim") else "  (没有红点, 瞄环心)"))
                return f
            last_why = "; ".join(why)
            if now - ctx.t_log > 1.0:
                ctx.t_log = now
                rospy.loginfo(f"    [align] 还不能锁定: {last_why}")
            if now - t_al > C.GATE_ALIGN_MAX_S:
                ctx.fail(f"取样 {C.GATE_ALIGN_MAX_S:.0f}s 仍不收敛: {last_why}")
        ctx.tick()
