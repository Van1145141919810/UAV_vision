# -*- coding: utf-8 -*-
"""
vb09/flight/steps/standoff.py —— 停靠: STANDOFF 飞到目标前方停住对正 / HOLD 悬停记录检测
===============================================================================
2026-10-31 (PLAN_refactor_v2 第 7 步)。MODES["dock"] = [Acquire, StandOff, Hold]。环当目标; 步骤里没有
目标类型判断 —— 只用 Lock 的 aim / n (以后 ArUco / 红点给同样的 Lock 就能直接用)。

StandOff  停靠点 S = aim − dir·GATE_DOCK_D (水平), 高度 = aim 的高度
            dir = −n 的水平分量 (朝目标飞为正); Lock 没有 n → 用当前视线方向
          复用 reposition.move_to (≤ GATE_REPO_V, 机头盯目标, 全程在目标前 ≥ |REPO_X_MAX|) + turn_to 对正
          S 出了水平范围 (GATE_XY_MAX) / 高度带 [GATE_Z_MIN_DZ, GATE_Z_MAX_DZ] → 放弃 (回手动段)
Hold      在 S 悬停 GATE_DOCK_HOLD_S 秒; gate.csv 照常每拍一行。结束时打印验收数:
            停靠误差 (飞机位置离 S, 中位数 / 最大)   要求 < 20 cm
            aim 横向抖动 (每帧观测投到靶面 h / v 的标准差)   要求 < 3 cm
          然后回手动段 (换位置 T 再来 / L 降落), 和影子模式一样
"""
import math

import rospy
import gate_plan as gp
from vb09.common import geom
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.steps.reposition import move_to, turn_to

DOCK_ERR_OK_CM = 20.0          # 验收线 (PLAN_refactor_v2 第 7 步), 只用于打印 ✓ / ✗
DOCK_AIM_SD_OK_CM = 3.0


def _r(v, nd=3):
    return [round(float(x), nd) for x in v] if v is not None else None


class StandOff(Step):
    name = "align"
    vel_lim = "pre"

    def run(self, ctx, lock):
        lock = lock if lock is not None else ctx.acq
        if lock is None or lock.get("c") is None:
            ctx.fail("停靠: 没有目标估计")
        p = ctx.pos()
        aim = tuple(float(v) for v in lock.aim[:3])
        d_in, src = None, "法向"
        if lock.n is not None:
            d_in = geom.hunit(geom.vmul(lock.n, -1.0))
        if d_in is None:
            d_in, src = geom.hunit(geom.vsub(aim, p)), "视线"
        if d_in is None:
            ctx.fail("停靠: 目标在正上 / 正下方, 定不了停靠方向")
        D = C.GATE_DOCK_D
        S = (aim[0] - d_in[0] * D, aim[1] - d_in[1] * D, aim[2])
        o = ctx.origin
        r_xy = math.hypot(S[0] - o[0], S[1] - o[1])
        dz = S[2] - o[2]
        ctx.dock = {"S": _r(S), "aim": _r(aim), "d": round(D, 3), "dir": _r(d_in, 4), "dir_src": src,
                    "err_cm": None, "aim_sd_cm": None, "done": False}
        if r_xy > C.GATE_XY_MAX:
            ctx.fail(f"停靠点离起飞点水平 {r_xy:.2f}m > 水平范围 {C.GATE_XY_MAX:.2f}m")
        if not (C.GATE_Z_MIN_DZ <= dz <= C.GATE_Z_MAX_DZ):
            ctx.fail(f"停靠点高度 dz={dz:.2f}m 不在 [{C.GATE_Z_MIN_DZ:.2f}, {C.GATE_Z_MAX_DZ:.2f}] (相对起飞点)")
        rospy.loginfo(f"[gate] 停靠点 ({S[0]:+.2f},{S[1]:+.2f},{S[2]:+.2f}): 目标前 {D:.2f}m (方向按{src}), "
                      f"现在离它 {geom.vnorm(geom.vsub(S, p)):.2f}m")
        move_to(ctx, S, "ring", C.GATE_REPO_TIMEOUT, "停靠", ref=(aim, d_in))
        turn_to(ctx, gp.yaw_of(d_in), S, "对正目标")
        lock.standoff = S                    # 属性, 不进 dict (遥测 / gate.csv 不变)
        lock.d_in = d_in
        return lock


def _sd(xs):
    if len(xs) < 2:
        return float("nan")
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


class Hold(Step):
    name = "hold"
    vel_lim = "pre"

    def run(self, ctx, lock):
        S = getattr(lock, "standoff", None) or ctx.pos()
        d_in = getattr(lock, "d_in", None)
        T = C.GATE_DOCK_HOLD_S
        ctx.set_seg("hold", f"停靠: 悬停 {T:.0f}s, 记录检测 (gate.csv)")
        t0 = rospy.Time.now().to_sec() + 0.5          # 转头后先稳 0.5s 再算误差
        t_end = t0 + T
        errs = []
        if ctx.dock is not None:
            ctx.dock["t_end"] = round(t_end, 2)
        while True:
            now = rospy.Time.now().to_sec()
            if now >= t_end:
                break
            ctx.hold(S)
            if now >= t0:
                errs.append(geom.vnorm(geom.vsub(ctx.pos(), S)))
            if now - ctx.t_log > 1.0 and errs:
                ctx.t_log = now
                e_med, sd = self._stats(ctx, t0, errs, d_in)
                if ctx.dock is not None:
                    ctx.dock.update(err_cm=round(e_med * 100, 1), aim_sd_cm=sd)
                rospy.loginfo(f"    [停靠] 还剩 {t_end - now:.0f}s  离停靠点 {e_med * 100:.0f}cm"
                              + (f"  aim 抖动 h {sd[0]:.1f} / v {sd[1]:.1f} cm (n={sd[2]})" if sd else ""))
            ctx.tick()
        e_med, sd = self._stats(ctx, t0, errs, d_in)
        e_max = max(errs) if errs else float("nan")
        ok_e = e_med * 100 < DOCK_ERR_OK_CM
        ok_a = bool(sd) and max(sd[0], sd[1]) < DOCK_AIM_SD_OK_CM
        rospy.logwarn(f"[gate] ★ 停靠结果: 离停靠点 中位 {e_med * 100:.1f} / 最大 {e_max * 100:.1f} cm "
                      f"({'✓' if ok_e else '✗'} 要 < {DOCK_ERR_OK_CM:.0f})  "
                      + (f"aim 横向抖动 h {sd[0]:.1f} / v {sd[1]:.1f} cm, {sd[2]} 帧 "
                         f"({'✓' if ok_a else '✗'} 要 < {DOCK_AIM_SD_OK_CM:.0f})" if sd else "aim: 悬停期间没有有效帧 ✗"))
        if ctx.dock is not None:
            ctx.dock.update(err_cm=round(e_med * 100, 1), err_max_cm=round(e_max * 100, 1),
                            aim_sd_cm=sd, done=True, ok=bool(ok_e and ok_a))
        ctx.outcome = "dock"
        ctx.again("停靠做完了 (结果见日志 / 网页) —— 换个位置 T 再来, 或 L 降落")
        return lock

    @staticmethod
    def _stats(ctx, t0, errs, d_in):
        """→ (位置误差中位数 m, [aim 抖动 h cm, v cm, 帧数] 或 None)。aim 没有就用环心。"""
        e_med = gp.median(errs) if errs else float("nan")
        smp = ctx.gate_in.valid_since(t0)
        pts = [s.get("aim") or s["c"] for s in smp]
        if len(pts) < 3:
            return e_med, None
        n = geom.vmul(d_in, -1.0) if d_in is not None else geom.hunit(geom.vsub(ctx.pos(), pts[-1]))
        if n is None:
            return e_med, None
        right, up = geom.gate_axes(n)
        h = [geom.vdot(q, right) for q in pts]
        v = [geom.vdot(q, up) for q in pts]
        return e_med, [round(_sd(h) * 100, 2), round(_sd(v) * 100, 2), len(pts)]
