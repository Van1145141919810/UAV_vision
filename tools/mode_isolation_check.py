# -*- coding: utf-8 -*-
"""
tools/mode_isolation_check.py —— 模式分区的自检 (2026-11-06)

    python3 tools/mode_isolation_check.py          # 一致性 + 静态 + dry-run 扰动, ~30 s
    python3 tools/mode_isolation_check.py --sim    # 再加: 每个模式跑一次 sim, 不用的区填极端值, 结果要和不填一样

保证"当前模式不用的参数, 填什么 (含越界) 都不影响这一趟":
  1 一致性   modes.py 的步骤列表 == mode_spec.MODE_SPECS 的 steps; steps/ 里每个 Step 子类都在 STEP_USES 里;
             旋钮表的 section 都在 SECTIONS 里
  2 静态     每个步骤 (含它在 steps/ 里调用的模块级函数) 读的 C.参数 不属于它没声明的区;
             不用 dash 区的步骤不许调 gate_plan 的规划函数。
             步骤以外的飞行代码 (mission / preflight / safety / config …) 读区参数的地方列在 OUTSIDE_OK 里,
             每条写明为什么不影响别的模式; 新出现的读取 → 失败, 要么加守卫要么登记
  3 dry-run  每个模式: 基准档案 vs 把别的区的旋钮全填成极端 / 越界值 —— 必须都通过, 而且这个模式用到的
             全部参数、gate 阶段包络、停靠派生量完全相同
  4 sim      (--sim) 同 3 的扰动, 跑 sim 场景 (dash→normal, shadow, dock), 摘要和不扰动的在容差内相同
"""
from __future__ import annotations

import ast
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
from vb09.flight import mode_spec as MS          # noqa: E402
from vb09.profile import core as PC               # noqa: E402
from vb09.profile import gate as PG               # noqa: E402

STEPS_DIR = os.path.join(ROOT, "vb09", "flight", "steps")
FLIGHT_DIR = os.path.join(ROOT, "vb09", "flight")

# 区 → 参数 (旋钮写的 + 只属于这个区、没有旋钮的)
EXTRA = {
    "dash": {"GATE_SLOW_POLICY", "DASH_V_MIN", "DASH_V_STEP", "D_ALIGN_MIN", "AIM_MARGIN"},
    "dock": {"GATE_DOCK_D"},                     # 派生量 (derive): = dock_dist 或 GATE_DOCK_D_AUTO
}

# 步骤以外读区参数的地方: (文件, 参数) → 为什么不影响别的模式
OUTSIDE_OK = {
    ("config.py", "DASH_V"): "derive(): gate 包络只对有 Dash 步骤的模式按 DASH_V 放宽 (mode_spec.fast_v_params)",
    ("config.py", "D_ALIGN_MIN"): "只在注释里 / 定义处",
    ("config.py", "*"): "参数定义处",
    ("preflight.py", "*"): "print_config_summary / preflight_plan 有 _ms.uses(MISSION_MODE, 'dash') 守卫",
    ("mission.py", "DASH_V"): "手动段预规划有 dash 守卫; _telem_and_log 的 plan 段只在冲刺计划存在时输出",
    ("mission.py", "D_RUN_MAX"): "手动段预规划 (dash 守卫) / 冲刺计划遥测",
    ("mission.py", "PASS_RUN"): "冲刺计划遥测 (只有 ctx.plan 存在时)",
    ("mission.py", "BRAKE_ACC"): "冲刺计划遥测 (只有 ctx.plan 存在时)",
    ("mission.py", "M_BACK"): "冲刺计划遥测 (只有 ctx.plan 存在时)",
    ("mission.py", "GATE_DOCK_D"): "手动段提示, 有 _ms.uses(MISSION_MODE, 'dock') 守卫",
}

GP_GEOM_OK = {"vadd", "vsub", "vmul", "vdot", "vnorm", "hunit", "median", "ang_deg", "_ring_frame", "to_plane",
              "unit", "vcross", "yaw_of", "entry_axes",
              "fuse_samples"}                     # fuse_samples: 多帧样本 → 锁定值 (和目标 / 模式无关)

FAIL = []


def fail(msg):
    FAIL.append(msg)
    print("FAIL  " + msg)


def ok(msg):
    print("ok    " + msg)


def section_params():
    out = {s: set() for s in MS.SECTIONS}
    for k in PC.KNOBS:
        sec = k.get("section")
        if sec and k["key"] in PG.GATE_KNOB_PARAM:
            out[sec].add(PG.GATE_KNOB_PARAM[k["key"]])
    for s, ps in EXTRA.items():
        out[s] |= ps
    return out


# ---------------------------------------------------------------- 1 一致性
def modes_py_steps():
    tree = ast.parse(open(os.path.join(FLIGHT_DIR, "modes.py"), encoding="utf-8").read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "MODES" for t in node.targets):
            return {k.value: [e.func.id for e in v.elts] for k, v in zip(node.value.keys, node.value.values)}
    return {}


def step_classes():
    """steps/*.py 里 Step 的子类 → (文件, ClassDef, 模块 AST)。"""
    out = {}
    for fn in sorted(os.listdir(STEPS_DIR)):
        if not fn.endswith(".py"):
            continue
        tree = ast.parse(open(os.path.join(STEPS_DIR, fn), encoding="utf-8").read())
        for node in tree.body:
            if isinstance(node, ast.ClassDef) and any(isinstance(b, ast.Name) and b.id == "Step" for b in node.bases):
                out[node.name] = (fn, node, tree)
    return out


def check_consistency():
    mp = modes_py_steps()
    for m, spec in MS.MODE_SPECS.items():
        if mp.get(m) != list(spec["steps"]):
            fail("模式 %s: modes.py %s ≠ mode_spec %s" % (m, mp.get(m), list(spec["steps"])))
    for m in mp:
        if m not in MS.MODE_SPECS:
            fail("modes.py 的模式 %s 没在 mode_spec.MODE_SPECS 里" % m)
    for c in step_classes():
        if c not in MS.STEP_USES:
            fail("步骤 %s 没在 mode_spec.STEP_USES 里登记" % c)
    for k in PC.KNOBS:
        if k.get("section") and k["section"] not in MS.SECTIONS:
            fail("旋钮 %s 的 section %r 不在 SECTIONS 里" % (k["key"], k["section"]))
    labels = [k["choices"] for k in PC.KNOBS if k["key"] == "mode"][0]
    if labels != [v["label"] for v in MS.MODE_SPECS.values()]:
        fail("mode 旋钮的选项 %s ≠ MODE_SPECS 的 label" % labels)
    if not any(f.startswith("模式 ") for f in FAIL):
        ok("一致性: %d 个模式 / %d 个步骤 / 旋钮 section" % (len(mp), len(step_classes())))


# ---------------------------------------------------------------- 2 静态
class _Reads(ast.NodeVisitor):
    def __init__(self):
        self.c, self.gp, self.calls = set(), set(), set()

    def visit_Attribute(self, n):
        if isinstance(n.value, ast.Name):
            if n.value.id == "C":
                self.c.add(n.attr)
            elif n.value.id == "gp":
                self.gp.add(n.attr)
        self.generic_visit(n)

    def visit_Call(self, n):
        if isinstance(n.func, ast.Name):
            self.calls.add(n.func.id)
        self.generic_visit(n)


def _module_funcs():
    """steps/*.py 的模块级函数名 → FunctionDef (跨文件按名字找: common.move_to 之类)。"""
    out = {}
    for fn in sorted(os.listdir(STEPS_DIR)):
        if fn.endswith(".py"):
            tree = ast.parse(open(os.path.join(STEPS_DIR, fn), encoding="utf-8").read())
            for node in tree.body:
                if isinstance(node, ast.FunctionDef):
                    out[node.name] = node
    return out


def step_reads(cls_node, funcs):
    r = _Reads()
    r.visit(cls_node)
    seen, todo = set(), list(r.calls)
    while todo:
        f = todo.pop()
        if f in seen or f not in funcs:
            continue
        seen.add(f)
        r2 = _Reads()
        r2.visit(funcs[f])
        r.c |= r2.c
        r.gp |= r2.gp
        todo += list(r2.calls)
    return r


def check_static():
    sp = section_params()
    funcs = _module_funcs()
    n_bad = len(FAIL)
    for name, (fn, node, _t) in sorted(step_classes().items()):
        uses = set(MS.STEP_USES.get(name, ()))
        r = step_reads(node, funcs)
        for sec, ps in sp.items():
            if sec in uses:
                continue
            bad = sorted(r.c & ps)
            if bad:
                fail("步骤 %s (%s) 读了 %s 区的参数 %s, 但 STEP_USES 没声明这个区" % (name, fn, sec, bad))
        if "dash" not in uses:
            plan = sorted(x for x in r.gp if x not in GP_GEOM_OK)
            if plan:
                fail("步骤 %s 不用 dash 区, 却调了 gate_plan 的 %s" % (name, plan))
    allp = set().union(*sp.values())
    for fn in sorted(os.listdir(FLIGHT_DIR)):
        if not fn.endswith(".py") or fn in ("mode_spec.py",):
            continue
        r = _Reads()
        r.visit(ast.parse(open(os.path.join(FLIGHT_DIR, fn), encoding="utf-8").read()))
        for p in sorted(r.c & allp):
            if (fn, p) not in OUTSIDE_OK and (fn, "*") not in OUTSIDE_OK:
                fail("%s 读了区参数 C.%s —— 加模式守卫, 或登记到 OUTSIDE_OK 并写明为什么不影响别的模式" % (fn, p))
    if len(FAIL) == n_bad:
        ok("静态: 每个步骤只读自己声明的区; 步骤以外的读取都已登记")


# ---------------------------------------------------------------- 3 dry-run 扰动
BASE = ("task=穿环\nscene=室内\ntakeoff_h=1.5\nz_low=0.8\nz_high=2.2\nxy_radius=10\n"
        "dash_v=2.5\nd_back=4.0\nd_rear=2.0\nd_side=2.0\nl_f=0.30\nl_r=0.30\nw_half=0.25\n"
        "dock_dist=3.5\ndock_hold=20\nvscan=0.3\n")


def extremes(k):
    """一个旋钮的几组扰动值: 下限、上限、越界 (低 / 高), choice 取每个选项。"""
    if k["type"] == "choice":
        return list(k["choices"])
    lo, hi = k.get("min", 0.0), k.get("max", 10.0)
    return [lo, hi, lo - abs(lo) - 1.0, hi * 3 + 1.0]


def dry(text, sites):
    with open(os.path.join(sites, "t.conf"), "w", encoding="utf-8") as f:
        f.write(text)
    p = subprocess.run([sys.executable, os.path.join(ROOT, "stage14_mission.py"), "--sites-dir", sites,
                        "--site", "t", "--dry-run", "--json"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       cwd=ROOT, env=dict(os.environ, PYTHONHASHSEED="0"), timeout=120)
    out = p.stdout.decode("utf-8", "replace")
    try:
        return p.returncode, json.loads(out[out.index("{"):])
    except ValueError:
        return p.returncode, {"raw": out[-400:]}


def fingerprint(js, mode, sp):
    act = MS.sections(mode)
    other = set().union(*(ps for s, ps in sp.items() if s not in act)) if len(act) < len(sp) else set()
    params = {k: v for k, v in (js.get("params") or {}).items() if k not in other}
    d = js.get("derived") or {}
    return {"params": params, "env": (d.get("phase_envelope") or {}).get("gate"), "dock": d.get("dock"),
            "mode": (d.get("mode") or {}).get("key")}


def variants(mode):
    """别的区的旋钮: 每个旋钮的每个极端值单独一个变体, 再加一个"全部取最怪的"。"""
    act = MS.sections(mode)
    ks = [k for k in PC.KNOBS if k.get("section") and k["section"] not in act]
    out = []
    for k in ks:
        for v in extremes(k):
            out.append({k["key"]: v})
    if ks:
        out.append({k["key"]: extremes(k)[-1] for k in ks})
    return out


def apply(base, ov):
    lines = [ln for ln in base.splitlines() if ln.split("=")[0] not in ov]
    return "\n".join(lines + ["%s=%s" % (k, v) for k, v in ov.items()]) + "\n"


def check_dryrun():
    sp = section_params()
    sites = tempfile.mkdtemp()
    for mode, spec in MS.MODE_SPECS.items():
        base = BASE + "mode=%s\n" % spec["label"]
        rc0, js0 = dry(base, sites)
        if rc0 != 0:
            fail("模式 %s: 基准档案 dry-run 不通过: %s" % (mode, (js0.get("errors") or js0)))
            continue
        f0 = fingerprint(js0, mode, sp)
        vs = variants(mode)
        bad = 0
        for ov in vs:
            rc, js = dry(apply(base, ov), sites)
            if rc != 0:
                fail("模式 %s: 不用的旋钮 %s → 拒绝起飞: %s" % (mode, ov, js.get("errors") or js))
                bad += 1
                continue
            f1 = fingerprint(js, mode, sp)
            if f1 != f0:
                diff = [k for k in f0 if f0[k] != f1[k]]
                if "params" in diff:
                    diff += sorted(k for k in set(f0["params"]) | set(f1["params"])
                                   if f0["params"].get(k) != f1["params"].get(k))
                fail("模式 %s: 不用的旋钮 %s 改变了 %s" % (mode, ov, diff))
                bad += 1
        if not bad:
            ok("dry-run %-6s %d 个扰动 (不用的区: %s) 都通过, 用到的参数 / 包络 / 停靠派生量不变"
               % (mode, len(vs), ", ".join(s for s in MS.SECTIONS if s not in MS.sections(mode)) or "无"))


# ---------------------------------------------------------------- 4 sim 扰动
SIM_SCEN = {"dash": "normal", "shadow": "shadow", "dock": "dock"}


def check_sim():
    sys.path.insert(0, os.path.join(ROOT, "tools", "baseline"))
    import sim_check as SC
    sp = section_params()
    for mode, sc in SIM_SCEN.items():
        act = MS.sections(mode)
        sets = []
        for k in PC.KNOBS:
            sec = k.get("section")
            if sec and sec not in act and k["type"] == "float" and k["key"] in PG.GATE_KNOB_PARAM:
                sets += ["--set", "%s=%s" % (PG.GATE_KNOB_PARAM[k["key"]], k.get("max", 1.0))]
        if not sets:
            ok("sim %-6s 没有不用的区, 跳过" % mode)
            continue
        outs = []
        for extra in ([], sets):
            p = subprocess.run([sys.executable, os.path.join(ROOT, "sim", "run_sim.py"), sc] + extra,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=ROOT, timeout=600)
            outs.append(SC.summarize(sc, p.returncode, p.stdout.decode("utf-8", "replace")))
        d = SC.compare(outs[0], outs[1])
        if d:
            fail("sim %s: 不用的区填上限后结果变了: %s" % (mode, d))
        else:
            ok("sim %-6s 不用的区填上限 (%d 个参数), 摘要在容差内相同" % (mode, len(sets) // 2))
    del sp


def main():
    check_consistency()
    check_static()
    check_dryrun()
    if "--sim" in sys.argv:
        check_sim()
    print("=" * 60)
    print("ALL PASS" if not FAIL else "FAILED (%d)" % len(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
