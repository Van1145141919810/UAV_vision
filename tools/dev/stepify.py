# -*- coding: utf-8 -*-
"""
stepify.py —— 一次性 (第 4.4 步): vb09/flight/mission.py 的 GateRun 各段方法 → vb09/flight/steps/*.py
  方法体原样搬, self. → ctx. ; 只改动这些 (每处都在下面写明):
    · acquire / align 结束时把结果包成 Lock 再返回
    · acquire 开头的 set_phase(pre) 和 dash 开头的 set_phase/back_ok/dash_tilt_on 交给 ctx.enter(step)
      (步骤只声明 vel_lim / committed, 平台去设) —— set_phase 不打日志、不发遥测, 顺序无可见差别
    · dash 里的 tick(dash=True) → tick() (ctx.tick 按当前步骤的 committed 决定)
"""
import ast
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
P = os.path.join(ROOT, "vb09", "flight", "mission.py")
STEPS = os.path.join(ROOT, "vb09", "flight", "steps")


def method_src(src, tree, name):
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "GateRun")
    m = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    lines = src.splitlines(keepends=True)
    # 连同方法前面紧挨的注释行 (# ---------------- XXX ----------------)
    start = m.lineno - 1
    while start > 0 and lines[start - 1].strip().startswith("#"):
        start -= 1
    if m.decorator_list:
        start = min(start, m.decorator_list[0].lineno - 1)
    return "".join(lines[start:m.end_lineno]), (start, m.end_lineno)


def sub1(s, old, new, cnt=1):
    n = s.count(old)
    if n != cnt:
        sys.exit("expected %d of %r, got %d" % (cnt, old[:80], n))
    return s.replace(old, new)


def to_ctx(body):
    return re.sub(r"\bself\.", "ctx.", body)


HDR = '''# -*- coding: utf-8 -*-
"""
vb09/flight/steps/{name}.py —— {doc}
===============================================================================
2026-10-30 第 4.4 步: 原 GateRun.{orig} 的方法体原样搬来 (self. → ctx.), 见 tools/dev/stepify.py。
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock 读目标。
"""
'''


def main():
    src = open(P, encoding="utf-8").read()
    tree = ast.parse(src)
    got = {}
    spans = []
    for name in ("acquire", "make_plan", "move_to", "turn_to", "align", "commit", "dash"):
        s, span = method_src(src, tree, name)
        got[name] = s
        spans.append(span)

    # ---------------- 各步骤 ----------------
    acq = to_ctx(got["acquire"])
    acq = sub1(acq, "    def acquire(ctx):\n", "    def run(self, ctx, lock):\n") if "def acquire(ctx)" in acq else \
        sub1(acq, "    def acquire(self):\n", "    def run(self, ctx, lock):\n")
    acq = sub1(acq, "        set_phase(\"gate\", vel=C.GATE_PRE_VEL_LIM)\n", "")
    acq = sub1(acq, "                    ctx.acq = f\n", "                    ctx.acq = f = Lock(f, kind=\"ring\")\n")
    acq = sub1(acq, """                                  f"法向 {'有' if f.get('n') else '无'}  红点 {'有' if f.get('aim') else '无'}")
                    return
""", """                                  f"法向 {'有' if f.get('n') else '无'}  红点 {'有' if f.get('aim') else '无'}")
                    return f
""")

    mk = to_ctx(got["make_plan"])
    mk = sub1(mk, "    def make_plan(ctx, est, p_from, label):\n", "def make_plan(ctx, est, p_from, label):\n") \
        if "def make_plan(ctx," in mk else sub1(mk, "    def make_plan(self, est, p_from, label):\n",
                                                 "def make_plan(ctx, est, p_from, label):\n")
    mk = "\n".join(ln[4:] if ln.startswith("    ") else ln for ln in mk.split("\n"))

    mv = to_ctx(got["move_to"])
    mv = sub1(mv, "    def move_to(self, S, yaw_mode, timeout, label):\n", "def move_to(ctx, S, yaw_mode, timeout, label):\n")
    mv = "\n".join(ln[4:] if ln.startswith("    ") else ln for ln in mv.split("\n"))
    tt = to_ctx(got["turn_to"])
    tt = sub1(tt, "    def turn_to(self, yaw_target, hold_p, label, timeout=15.0):\n",
              "def turn_to(ctx, yaw_target, hold_p, label, timeout=15.0):\n")
    tt = "\n".join(ln[4:] if ln.startswith("    ") else ln for ln in tt.split("\n"))

    al = to_ctx(got["align"])
    al = sub1(al, "    def align(self, hold_p):\n", "def align_lock(ctx, hold_p):\n")
    al = sub1(al, "                    ctx.locked = f\n", "                    ctx.locked = f = Lock(f, kind=\"ring\")\n")
    al = sub1(al, """                                     if f.get("aim") else "  (没有红点, 瞄环心)"))
                    return
""", """                                     if f.get("aim") else "  (没有红点, 瞄环心)"))
                    return f
""")
    al = "\n".join(ln[4:] if ln.startswith("    ") else ln for ln in al.split("\n"))

    cm = to_ctx(got["commit"])
    cm = sub1(cm, "    def commit(self):\n", "    def run(self, ctx, lock):\n")
    cm = cm.rstrip("\n") + "\n"
    # commit 的最后: 停稳在起跑点后 return → 返回 lock
    cm = sub1(cm, """            if e < C.GATE_START_TOL and gp.vnorm(v) < C.GATE_START_V_MAX:
                return
""", """            if e < C.GATE_START_TOL and gp.vnorm(v) < C.GATE_START_V_MAX:
                return lock
""")

    ds = to_ctx(got["dash"])
    ds = sub1(ds, "    def dash(self):\n", "    def run(self, ctx, lock):\n")
    ds = sub1(ds, """        set_phase("gate")                                  # 速度包络放开到冲刺档
        ctx.back_ok = False
        F.dash_tilt_on[0] = True                             # 停桨线换成 GATE_DASH_TILT_DEG
""", """        # 速度包络放开到冲刺档 / 不能再回手动段 / 停桨线换成 GATE_DASH_TILT_DEG:
        #   本步骤 committed = True, 由 ctx.enter(step) 设 (平台白名单属性, 步骤只声明)
""")
    ds = sub1(ds, "            ctx.tick(dash=True)\n", "            ctx.tick()                          # committed: LAND 推迟到刹停后\n")
    ds = sub1(ds, """                    ctx.hold(p)
                    rospy.logerr(f"[gate] ★ 卡纸: 规划 s={s_p:+.2f} v={v_p:.2f} / 实测 s={s_e:+.2f} "
                                 f"v={v_e:.2f} —— 停止前推, 设定点 = 当前位置")
                    return
""", """                    ctx.hold(p)
                    rospy.logerr(f"[gate] ★ 卡纸: 规划 s={s_p:+.2f} v={v_p:.2f} / 实测 s={s_e:+.2f} "
                                 f"v={v_e:.2f} —— 停止前推, 设定点 = 当前位置")
                    return lock
""")
    ds = ds.rstrip("\n") + "\n        return lock\n"

    os.makedirs(STEPS, exist_ok=True)
    files = {
        "acquire": (HDR.format(name="acquire", doc="ACQUIRE: 原地悬停, 机头转向环, 收够有效帧 → 粗估 (Lock)",
                               orig="acquire") + '''
import math

import rospy
import gate_plan as gp
from vb09.common.target import Lock
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.telemetry import wrap_pi


class Acquire(Step):
    name = "wait"
    vel_lim = "pre"

''' + acq.split("\n", 1)[1] if acq.startswith("    # ") else None),
    }
    # acquire: 方法前的注释行 (# ---- ACQUIRE ----) 去掉, 只留 def
    acq_body = acq[acq.index("    def run(self, ctx, lock):"):]
    files["acquire"] = HDR.format(name="acquire", doc="ACQUIRE: 原地悬停, 机头转向环, 收够有效帧 → 粗估 (Lock)",
                                  orig="acquire") + '''
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

''' + acq_body

    mk_body = mk[mk.index("def make_plan(ctx"):]
    files["plan_dash"] = HDR.format(name="plan_dash", doc="PLAN: gate_plan.plan_dash (进入方向 / 起跑点 S / 制动终点 E)",
                                    orig="make_plan") + '''
import math

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step


''' + mk_body.rstrip("\n") + '''


class PlanDash(Step):
    """用当前 Lock 规划一次 (原 phase_gate 里的 run.make_plan(run.acq, run.pos(), "粗规划"))。"""
    name = "wait"            # 规划在识别段里做, 不换段名
    vel_lim = "pre"

    def __init__(self, label="粗规划"):
        self.label = label

    def run(self, ctx, lock):
        make_plan(ctx, lock, ctx.pos(), self.label)
        return lock
'''
    mv_body = mv[mv.index("def move_to(ctx"):]
    tt_body = tt[tt.index("def turn_to(ctx"):]
    files["reposition"] = HDR.format(name="reposition", doc="REPOSITION: 直线慢飞到起跑点 S, 再转头对正进入方向",
                                     orig="move_to / turn_to") + '''
import math

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.telemetry import wrap_pi


''' + mv_body.rstrip("\n") + "\n\n\n" + tt_body.rstrip("\n") + '''


class Reposition(Step):
    """原 phase_gate: move_to(P.S, "ring", GATE_REPO_TIMEOUT, "摆位") + turn_to(yaw_of(P.dir), P.S, "对正进入方向")。"""
    name = "align"           # move_to 设的段名就是 align (遥测不变)
    vel_lim = "pre"

    def run(self, ctx, lock):
        P = ctx.plan
        move_to(ctx, P.S, "ring", C.GATE_REPO_TIMEOUT, "摆位")
        turn_to(ctx, gp.yaw_of(P.dir), P.S, "对正进入方向")
        return lock
'''
    al_body = al[al.index("def align_lock(ctx"):]
    files["align"] = HDR.format(name="align", doc="ALIGN: 悬停取样 → 收敛判定 → 锁定 (Lock); 冲刺档另做锁定后规划 + 微调起跑点",
                                orig="align") + '''
import rospy
import gate_plan as gp
from vb09.common.target import Lock
from vb09.flight import config as C
from vb09.flight.steps.base import Step
from vb09.flight.steps.plan_dash import make_plan
from vb09.flight.steps.reposition import move_to


''' + al_body.rstrip("\n") + '''


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
'''
    cm_body = cm[cm.index("    def run(self, ctx, lock):"):]
    files["commit"] = HDR.format(name="commit", doc="COMMIT: 打印计划, 等操作员按 G (降速时只认 Y), 再停稳在起跑点",
                                 orig="commit") + '''
import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step


class Commit(Step):
    name = "commit"
    vel_lim = "pre"

''' + cm_body
    ds_body = ds[ds.index("    def run(self, ctx, lock):"):]
    files["dash"] = HDR.format(name="dash", doc="DASH / BRAKE: 按时间表执行冲刺轨迹 (位置+速度+加速度前馈), 卡纸检测",
                               orig="dash") + '''
import math

import rospy
import gate_plan as gp
from vb09.flight import config as C
from vb09.flight.steps.base import Step


class Dash(Step):
    """
    committed = True: ctx.enter 放开速度包络到冲刺档、关掉回手动段、停桨线换成 GATE_DASH_TILT_DEG;
    ctx.tick 推迟 LAND 到刹停后。
    """
    name = "dash"
    committed = True

''' + ds_body
    for k, v in files.items():
        open(os.path.join(STEPS, k + ".py"), "w", encoding="utf-8").write(v)
        print("steps/%s.py" % k)

    # ---------------- mission.py: 删掉搬走的方法 ----------------
    lines = src.splitlines(keepends=True)
    for a, b in sorted(spans, reverse=True):
        del lines[a:b]
        while a < len(lines) and lines[a].strip() == "" and a > 0 and lines[a - 1].strip() == "":
            del lines[a]
    open(P, "w", encoding="utf-8").write("".join(lines))


if __name__ == "__main__":
    main()
