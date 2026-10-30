# -*- coding: utf-8 -*-
"""
vb09/flight/steps/acquire.py —— ACQUIRE: 原地悬停, 机头转向环, 收够有效帧 → 粗估 (Lock)
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.acquire 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
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
