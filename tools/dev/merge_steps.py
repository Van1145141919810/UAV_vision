# -*- coding: utf-8 -*-
"""一次性: vb09/flight/steps 9 个文件 → base / common / dash / shadow / dock (按顶层定义原样搬, 2026-11-01)"""
import ast
import os
import sys

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "vb09", "flight", "steps")


def load(name):
    s = open(os.path.join(D, name + ".py"), encoding="utf-8").read()
    t = ast.parse(s)
    lines = s.splitlines(keepends=True)
    defs, imps, other = {}, [], []
    for n in t.body:
        seg = "".join(lines[n.lineno - 1:n.end_lineno])
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            imps.append(seg.strip())
        elif isinstance(n, (ast.FunctionDef, ast.ClassDef)):
            # 带上紧挨着的上方注释行
            i = n.lineno - 2
            while i >= 0 and lines[i].lstrip().startswith("#"):
                i -= 1
            defs[n.name] = "".join(lines[i + 1:n.end_lineno])
        elif isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant):
            pass
        else:
            other.append(seg)
    return defs, imps, other


SRC = {k: load(k) for k in ("acquire", "align", "commit", "dash", "plan_dash", "reposition", "shadow", "standoff")}
INTERNAL = ("from vb09.flight.steps.",)

PLAN = {
    "common": ("""vb09/flight/steps/common.py —— 各模式共用的步骤和动作
===============================================================================
2026-11-01 由 acquire.py / reposition.py / align.py 合并 (代码原样)。
  Acquire        原地悬停, 机头转向目标, 收够有效帧 → 粗估 (Lock)
  move_to        以 GATE_REPO_V 斜坡直线飞到一点 (机头盯目标 / 固定朝向, 全程在目标前 ≥ |REPO_X_MAX|)
  turn_to        原地转头
  align_lock     悬停取样 → 收敛判定 → 锁定 (Lock)
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail …), 只能通过 lock / ctx.gate_in 读目标。""",
               [("acquire", "Acquire"), ("reposition", "move_to"), ("reposition", "turn_to"), ("align", "align_lock")], []),
    "dash": ("""vb09/flight/steps/dash.py —— 冲刺穿环的步骤 (MODES["dash"])
===============================================================================
2026-11-01 由 plan_dash.py / reposition.py / align.py / commit.py / dash.py 合并 (代码原样)。
  make_plan / PlanDash   gate_plan.plan_dash + 不可行时降速
  Reposition             摆位到起跑点 S + 对正进入方向
  Align                  在 S 取样锁定 → 锁定后规划 → 起跑点挪了就微调
  Commit                 等 G / Y, 停稳
  Dash                   按时间表冲刺 + 制动 + 卡纸处置 (committed = True)""",
             [("plan_dash", "make_plan"), ("plan_dash", "PlanDash"), ("reposition", "Reposition"), ("align", "Align"),
              ("commit", "Commit"), ("dash", "Dash")], ["from vb09.flight.steps.common import align_lock, move_to, turn_to"]),
    "shadow": ("""vb09/flight/steps/shadow.py —— 影子模式的步骤 (MODES["shadow"])
===============================================================================
2026-11-01 由 align.py (AlignHere) / shadow.py 合并 (代码原样)。
  AlignHere   原地 (当前位置) 取样锁定, 不摆位
  Shadow      锁定后规划 → 计划发网页 → 原地悬停 3s → 回手动段""",
               [("align", "AlignHere"), ("shadow", "Shadow")],
               ["from vb09.flight.steps.common import align_lock", "from vb09.flight.steps.dash import make_plan"]),
}


def build(name, doc, items, extra_imps):
    imps, bodies, seen = [], [], set()
    for mod, d in items:
        for i in SRC[mod][1]:
            if i.startswith(INTERNAL) and "steps.base" not in i:
                continue
            if i not in seen:
                seen.add(i)
                imps.append(i)
        bodies.append(SRC[mod][0][d].rstrip() + "\n")
    imps += extra_imps
    # 标准库 / 第三方在前, 本项目在后 (保持各自原顺序)
    std = [i for i in imps if not i.startswith(("from vb09", "import gate_plan"))]
    gp_ = [i for i in imps if i.startswith("import gate_plan")]
    loc = [i for i in imps if i.startswith("from vb09")]
    out = '# -*- coding: utf-8 -*-\n"""\n' + doc + '\n"""\n' + "\n".join(std + gp_ + loc) + "\n\n\n" + "\n\n".join(bodies)
    ast.parse(out)
    open(os.path.join(D, name + ".py"), "w", encoding="utf-8").write(out)
    print(name, len(out.splitlines()))


for k, (doc, items, ex) in PLAN.items():
    build(k, doc, items, ex)
# 停靠: standoff.py 改名 dock.py, 只改导入
s = open(os.path.join(D, "standoff.py"), encoding="utf-8").read()
s = s.replace("vb09/flight/steps/standoff.py", "vb09/flight/steps/dock.py")
s = s.replace("from vb09.flight.steps.reposition import move_to, turn_to", "from vb09.flight.steps.common import move_to, turn_to")
s = s.replace("复用 reposition.move_to", "复用 common.move_to")
open(os.path.join(D, "dock.py"), "w", encoding="utf-8").write(s)
for k in ("acquire", "align", "commit", "plan_dash", "reposition", "standoff"):
    os.remove(os.path.join(D, k + ".py"))
