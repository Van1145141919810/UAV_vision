# -*- coding: utf-8 -*-
"""
vb09/flight/steps/align.py —— ALIGN: 悬停取样 → 收敛判定 → 锁定 (Lock); 冲刺档另做锁定后规划 + 微调起跑点
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.align 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
"""

import rospy
import gate_plan as gp
from vb09.common.target import Lock
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.steps.plan_dash import make_plan
from vb09.flight.steps.reposition import move_to


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


class Align(Step):
    """
    冲刺档 (原 phase_gate 的 else 分支):
      align(P.S) → make_plan(locked, pos, "锁定后规划") → 起跑点挪了 > GATE_START_TOL 就微调过去
    """
    name = "align"
    vel_lim = "pre"

    def run(self, ctx, lock):
        lock = align_lock(ctx, ctx.plan.S)
        P2 = make_plan(ctx, ctx.locked, ctx.pos(), "锁定后规划")
        gap = gp.vnorm(gp.vsub(P2.S, ctx.pos()))
        if gap > C.GATE_START_TOL:
            if gap > 0.5:
                rospy.logwarn(f"[gate] 锁定后起跑点挪了 {gap:.2f}m —— 再挪过去 (不重新取样)")
            move_to(ctx, P2.S, gp.yaw_of(P2.dir), C.GATE_REPO_TIMEOUT, "微调起跑点")
        return lock


class AlignHere(Step):
    """影子档: 原地 (当前位置) 取样锁定, 不摆位。原 phase_gate: run.align(run.pos())。"""
    name = "align"
    vel_lim = "pre"

    def run(self, ctx, lock):
        return align_lock(ctx, ctx.pos())
