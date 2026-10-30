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
        r["config_msg"] = json.loads(norm(json.dumps(cm, ensure_ascii=False)))
        for fn in ("gate.csv", "flight_log.csv"):
            with open(os.path.join(d, fn), encoding="utf-8") as f:
                r[fn] = f.readline().strip()
        out[sc] = r
    return out


def main():
    if "--write-from" in sys.argv:
        rev = sys.argv[sys.argv.index("--write-from") + 1]
        wt = tempfile.mkdtemp()
        subprocess.check_call(["git", "worktree", "add", "-f", wt, rev], cwd=_compat.ROOT,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            shutil.rmtree(os.path.join(wt, "sim"))
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
    ok = _util.report("runtime", *_util.save_or_compare("runtime.json", res, False))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
