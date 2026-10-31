# -*- coding: utf-8 -*-
"""
log_compare.py —— 飞行脚本日志行等价 (第 4 步 "段名、日志行不变"): sim 场景的 rospy 日志
  数字全部换成 #, 同一模板只看第一次出现 → 模板序列; 和 REV 提交 (删除完成、搬家之前) 跑出来的比。
  python3 log_compare.py [REV] [场景 ...]       (默认 REV = e6b73cb, 场景 normal shadow stuck noring)
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402

LINE = re.compile(r"^\d+\.\d+ ([IWEF]) (.*)$")
SPLIT = re.compile(r"(?=\d+\.\d\d [IWEF] )")
# 已知的不确定性 (同一份代码多跑几次也会变, 已核对): 起跑点正好在 D_RUN_MAX 边界上时 "就地起跑"/"前进到"
#   来回跳; z 掉高事件看仿真时序; 多线程同时 print 会把两行粘在一起 (按时间戳前缀拆开)
NOISE = [(re.compile(r"\[(就地起跑|前进到|后退到)[^\]]*\]"), "[<起跑>]")]
SKIP = re.compile(r"^\[zdrop\]|GATE_SHADOW = True 是旧写法")   # 后者: 4.5 有意新增的提示


def templates(root, sc):
    p = subprocess.run([sys.executable, os.path.join(root, "sim", "run_sim.py"), sc], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=400, cwd=root,
                       env=dict(os.environ, SIM_QUIET="0"))
    seen, out = set(), []
    raw = []
    for ln in p.stdout.decode("utf-8", "replace").splitlines():
        raw += [x for x in SPLIT.split(ln) if x]
    for ln in raw:
        m = LINE.match(ln)
        if not m or SKIP.search(m.group(2)):
            continue
        msg = m.group(2)
        for rx, rep in NOISE:
            msg = rx.sub(rep, msg)
        t = m.group(1) + " " + re.sub(r"/tmp/tmp\w+", "<TMP>", msg)
        t = re.sub(r"[-+]?\d+(\.\d+)?", "#", t)
        if t not in seen:
            seen.add(t)
            out.append(t)
    return out


def main():
    args = [a for a in sys.argv[1:]]
    rev = args.pop(0) if args and not args[0] in ("normal", "shadow", "stuck", "noring", "virtual") else "e6b73cb"
    scs = args or ["normal", "shadow", "stuck", "noring"]
    wt = tempfile.mkdtemp()
    subprocess.check_call(["git", "worktree", "add", "-f", wt, rev], cwd=_compat.ROOT,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ok = True
    try:
        shutil.rmtree(os.path.join(wt, "sim"))
        shutil.copytree(os.path.join(_compat.ROOT, "sim"), os.path.join(wt, "sim"))
        if not os.path.exists(os.path.join(wt, "stage14_mission.py")):     # 改名之前的提交
            os.symlink("stage12_gate.py", os.path.join(wt, "stage14_mission.py"))
        for sc in scs:
            a, b = templates(wt, sc), templates(_compat.ROOT, sc)
            if a == b:
                print("log_%-9s PASS  %d templates identical (vs %s)" % (sc, len(a), rev))
                continue
            ok = False
            import difflib
            d = [x for x in difflib.unified_diff(a, b, lineterm="", n=1)][2:40]
            print("log_%-9s FAIL  vs %s\n      " % (sc, rev) + "\n      ".join(d))
    finally:
        subprocess.call(["git", "worktree", "remove", "--force", wt], cwd=_compat.ROOT)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
