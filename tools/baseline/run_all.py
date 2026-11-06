# -*- coding: utf-8 -*-
"""
run_all.py —— 跑全部基线。  python3 tools/baseline/run_all.py [--quick] [--no-sim]
  --quick: 跳过 det_replay / onboard_replay (合成渲染 ~20s) 和 sim (~40s)
  第一次在存档版本上: 各脚本单独加 --write 生成 golden (顺序: det → fuse → judge, 后者用前者的输入)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER = ["dryrun_dump.py", "plan_grid.py", "det_replay.py", "onboard_replay.py", "fuse_replay.py",
         "judge_replay.py", "../mode_isolation_check.py", "sim_check.py", "runtime_dump.py"]
# ../mode_isolation_check.py (2026-11-06): 模式分区自检 (一致性 / 静态 / dry-run 扰动)。完整跑加 --sim 另行执行


def main():
    quick = "--quick" in sys.argv
    skip = set()
    if quick:
        skip |= {"det_replay.py", "onboard_replay.py", "sim_check.py", "runtime_dump.py"}
    if "--no-sim" in sys.argv:
        skip |= {"sim_check.py", "runtime_dump.py"}
    bad = []
    for s in ORDER:
        if s in skip:
            print("%-14s skip" % os.path.basename(s)[:-3])
            continue
        r = subprocess.run([sys.executable, os.path.join(HERE, s)])
        if r.returncode:
            bad.append(s)
    print("=" * 60)
    print("ALL PASS" if not bad else "FAILED: " + " ".join(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
