# -*- coding: utf-8 -*-
"""
runtime_dump.py —— 运行时产物等价验证: sim normal / shadow 跑一次, 比
  site_config.txt (飞行日志目录里那份 "本次生效的全部参数")
  /flight_ctrl/config 最后一条 (params / derived / 开关; 时间戳和路径归一化)
  gate.csv / flight_log.csv 列名
golden 从 **删除步骤完成后、搬家之前** 的提交生成 (见 --write-from REV), 因为删戳点本身会改参数集合。
  python3 runtime_dump.py                     和 golden 比
  python3 runtime_dump.py --write-from e6b73cb   在临时 worktree 里跑那个提交, 存 golden
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402

SCEN = ["normal", "shadow"]


def norm(s):
    s = re.sub(r"/tmp/tmp[\w]+", "<TMP>", s)
    s = re.sub(r"\b\d{10}\.\d+\b", "<T>", s)
    return s


def run(root):
    out = {}
    run_sim = os.path.join(root, "sim", "run_sim.py")
    if not os.path.exists(os.path.join(root, "sim")):
        shutil.copytree(os.path.join(_compat.ROOT, "sim"), os.path.join(root, "sim"))
    for sc in SCEN:
        p = subprocess.run([sys.executable, run_sim, sc], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL, env=dict(os.environ, SIM_QUIET="1"), timeout=400, cwd=root)
        txt = p.stdout.decode("utf-8", "replace")
        m = re.search(r"日志目录 (\S+)", txt)
        if not m:
            out[sc] = {"error": txt[-800:]}
            continue
        d = m.group(1)
        r = {}
        with open(os.path.join(d, "site_config.txt"), encoding="utf-8") as f:
            r["site_config"] = norm(f.read()).splitlines()
        with open(os.path.join(d, "sim_config_msg.json"), encoding="utf-8") as f:
            cm = json.load(f) or {}
        for k in ("t", "log_dir", "origin"):
            cm.pop(k, None)
        for k in ("INITIAL_X", "INITIAL_Y", "INITIAL_Z", "INITIAL_YAW"):   # sim 位姿带噪声: 只要求键在
            if k in (cm.get("params") or {}):
                cm["params"][k] = "<num>"
        r["config_msg"] = json.loads(norm(json.dumps(cm, ensure_ascii=False)))
        for fn in ("gate.csv", "flight_log.csv"):
            with open(os.path.join(d, fn), encoding="utf-8") as f:
                r[fn] = f.readline().strip()
        out[sc] = r
    return out


def _intended(g, n):
    """有意的改动 (逐条写明), 比较前两边都抹掉:
      4.5  旧写法 GATE_SHADOW = True 多一条"等同于 mode = 影子"的警告
      4.6  包络核对表的 "任务 穿环 (stage12)" 改了措辞
      7    新参数 GATE_DOCK_* (停靠): site_config 多几行, config 的 params 多几个键
      11-05 新参数 GATE_VSCAN_* (停靠后上下扫描): 同上"""
    def fix(x):
        if isinstance(x, str):
            return x.replace("穿环 (stage12)", "穿环 (模式见下面 开关 / 档案 mode)")
        if isinstance(x, list):
            return [fix(v) for v in x if not (isinstance(v, str) and ("是旧写法, 现在等同于 mode = 影子" in v
                                                                      or "GATE_DOCK_" in v or "GATE_VSCAN_" in v))]
        if isinstance(x, dict):
            return {k: fix(v) for k, v in x.items() if not k.startswith(("GATE_DOCK_", "GATE_VSCAN_"))}
        return x
    return fix(g), fix(n)


def main():
    if "--write-from" in sys.argv:
        rev = sys.argv[sys.argv.index("--write-from") + 1]
        wt = tempfile.mkdtemp()
        subprocess.check_call(["git", "worktree", "add", "-f", wt, rev], cwd=_compat.ROOT,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            shutil.rmtree(os.path.join(wt, "sim"))
            if not os.path.exists(os.path.join(wt, "stage14_mission.py")):     # 改名之前的提交
                os.symlink("stage12_gate.py", os.path.join(wt, "stage14_mission.py"))
            res = run(wt)          # 用当前的 sim (带 config dump), 跑旧代码
        finally:
            subprocess.call(["git", "worktree", "remove", "--force", wt], cwd=_compat.ROOT)
        res["_rev"] = rev
        print(_util.save_or_compare("runtime.json", res, True)[1])
        return 0
    res = run(_compat.ROOT)
    with open(_util.golden_path("runtime.json")) as f:
        rev = json.load(f).get("_rev")
    res["_rev"] = rev
    ok = _util.report("runtime", *_util.save_or_compare("runtime.json", res, False, drop=_intended))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
