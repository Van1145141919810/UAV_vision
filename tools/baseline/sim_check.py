# -*- coding: utf-8 -*-
"""
sim_check.py —— sim/run_sim.py 各场景并行跑, 摘要和 golden 比
仿真按墙钟走 (线程调度有抖动), 所以不比字节, 比结构 + 容差:
  精确: rc、纸的结局、段序列、遥测 task/schema、缺字段、config、gate.csv 列名、空中停桨
  容差: 穿纸速度 ±0.15 m/s、穿纸点 h/v ±4 cm、机头最远过纸面 ±0.2 m、plan.pts ±0.2 m
  python3 sim_check.py [--write] [场景 ...]
"""
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402

SCENARIOS = ["normal", "stuck", "shadow", "virtual", "nonormal", "oblique", "noring", "lowbatt", "dock", "dock_nonormal"]
# dock / dock_nonormal (没有法向 → 按视线停靠): 第 7 步新功能, golden 是第 7 步当时写的 (sim_check.py --write dock), 之前的提交没有这个场景
EXTRA = {"virtual": ["--set", "DASH_V=1.4"]}     # 虚拟环 2.6 m 只够 1.4 m/s (否则预检要人按 c)
TOL = {"cross_v": 0.15, "cross_h": 0.04, "cross_vv": 0.04, "max_past": 0.2, "pts": 0.2, "max_tilt": 4.0,
       "dock_err_cm": 5.0, "dock_S": 0.2}


def run_one(sc):
    env = dict(os.environ, SIM_QUIET="0", PYTHONHASHSEED="0")
    p = subprocess.run([sys.executable, os.path.join(_compat.ROOT, "sim", "run_sim.py"), sc] + EXTRA.get(sc, []),
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                       stdin=subprocess.DEVNULL, timeout=400, cwd=_compat.ROOT)
    return sc, p.returncode, p.stdout.decode("utf-8", "replace")


def summarize(sc, prc, out):
    s = {"proc_rc": prc}
    m = re.search(r"场景 \S+\s+rc=(\S+)", out)
    s["rc"] = m.group(1) if m else None
    m = re.search(r"纸: (\S+)\s+穿纸时 (.*)", out)
    s["paper"] = m.group(1) if m else None
    if m:
        c = re.search(r"v=([-\d.]+)m/s 离瞄点 h=([-+\d.]+) v=([-+\d.]+)", m.group(2))
        if c:
            s["cross_v"], s["cross_h"], s["cross_vv"] = (float(x) for x in c.groups())
    m = re.search(r"机头最远过纸面 ([-\d.]+)m.*最大倾角 ([\d.]+)°\s+空中停桨 (\S+)\s+最后 armed=(\S+) mode=(\S+)", out)
    if m:
        s["max_past"], s["max_tilt"] = float(m.group(1)), float(m.group(2))
        s["disarm_air"], s["armed_end"], s["mode_end"] = m.group(3), m.group(4), m.group(5)
    m = re.search(r"遥测 task=(\S+) schema=(\S+)\s+段序列 (.*?)\s+帧数", out)
    if m:
        s["task"], s["schema"], s["segs"] = m.group(1), m.group(2), m.group(3).split(" → ")
    m = re.search(r"gate 段缺字段: (.*)", out)
    s["missing"] = m.group(1).strip() if m else None
    m = re.search(r"plan\.pts=(.*)", out)
    if m:
        s["pts"] = eval(m.group(1))          # noqa: S307  (自己打印的列表)
    m = re.search(r"停靠: S=(\[.*?\]) 距离 ([\d.]+)m 方向按(\S+)\s+误差 中位 (\S+) / 最大 (\S+) cm"
                  r".* done=(\S+) ok=(\S+)\s+遥测 mode=(\S+)", out)
    if m:
        s["dock_S"] = eval(m.group(1))       # noqa: S307
        s["dock_d"], s["dock_dir"] = float(m.group(2)), m.group(3)
        s["dock_err_cm"] = float(m.group(4))
        s["dock_done"], s["dock_ok"], s["tel_mode"] = m.group(6), m.group(7), m.group(8)
    m = re.search(r"config task=(\S+)\s+derived\.gate=(\S+)", out)
    s["config"] = list(m.groups()) if m else None
    m = re.search(r"gate\.csv (\d+) 行, 列 (\d+)", out)
    s["csv_cols"] = int(m.group(2)) if m else None
    m = re.search(r"日志目录 (\S+)", out)
    if m:
        try:
            with open(os.path.join(m.group(1), "gate.csv")) as f:
                s["csv_header"] = f.readline().strip().split(",")
            with open(os.path.join(m.group(1), "flight_log.csv")) as f:
                s["flight_header"] = f.readline().strip().split(",")
        except OSError:
            pass
    if "Traceback" in out:
        s["traceback"] = out[out.index("Traceback"):][-600:]
    return s


def compare(g, n):
    diffs = []
    for k in sorted(set(g) | set(n)):
        a, b = g.get(k), n.get(k)
        if k in TOL and isinstance(a, (int, float)) and isinstance(b, (int, float)):
            if abs(float(a) - float(b)) > TOL[k]:
                diffs.append("%s %s → %s (tol %s)" % (k, a, b, TOL[k]))
        elif k == "dock_S" and isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b) or any(abs(x - y) > TOL[k] for x, y in zip(a, b)):
                diffs.append("dock_S %s → %s" % (a, b))
        elif k == "pts" and isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b) or any(abs(x - y) > TOL["pts"] for pa, pb in zip(a, b) for x, y in zip(pa, pb)):
                diffs.append("pts %s → %s" % (a, b))
        elif a != b:
            diffs.append("%s %s → %s" % (k, _util._short(a), _util._short(b)))
    return diffs


def main():
    write = _util.main_args()
    scs = [a for a in sys.argv[1:] if not a.startswith("-")] or SCENARIOS
    with ThreadPoolExecutor(max_workers=len(scs)) as ex:
        res = list(ex.map(run_one, scs))
    summ = {sc: summarize(sc, rc, out) for sc, rc, out in res}
    path = _util.golden_path("sim.json")
    import json
    if write:
        old = {}
        if os.path.exists(path):
            with open(path) as f:
                old = json.load(f)
        old.update(summ)
        with open(path, "w") as f:
            json.dump(old, f, ensure_ascii=False, indent=1, sort_keys=True)
        for sc in scs:
            print("sim_%-10s written  rc=%s paper=%s segs=%s" % (sc, summ[sc].get("rc"), summ[sc].get("paper"),
                                                              "→".join(summ[sc].get("segs") or [])))
        return 0
    with open(path) as f:
        gold = json.load(f)
    ok = True
    logdir = os.path.join(_compat.HERE, "out")
    for sc, _rc, out in res:
        d = compare(gold.get(sc, {}), summ[sc])
        if d:                                            # 留完整输出看原因
            os.makedirs(logdir, exist_ok=True)
            with open(os.path.join(logdir, "sim_%s.log" % sc), "w") as f:
                f.write(out)
            d.append("full log: tools/baseline/out/sim_%s.log" % sc)
        ok &= _util.report("sim_" + sc, not d, d or "same (within tolerance)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
