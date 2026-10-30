# -*- coding: utf-8 -*-
"""飞机是 Python 3.8: 用 3.8 的语法解析全部 .py (match / 括号 with / X | Y 注解求值 等会报出来)。
    python3 tools/py38_check.py            (任何解释器都能跑, 只查语法)
真正的 3.8 冒烟: 用 3.8 解释器跑 dry-run / sim (见 tools/baseline/README.md)。"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
bad = 0
for d, dirs, files in os.walk(ROOT):
    dirs[:] = [x for x in dirs if not x.startswith(".") and x not in ("__pycache__", "golden", "out")]
    for f in files:
        if f.endswith(".py"):
            p = os.path.join(d, f)
            try:
                ast.parse(open(p, encoding="utf-8").read(), p, feature_version=(3, 8))
            except SyntaxError as e:
                bad += 1
                print("%s:%s: %s" % (os.path.relpath(p, ROOT), e.lineno, e.msg))
print("py3.8 syntax: %s" % ("OK" if not bad else "%d file(s) bad" % bad))
sys.exit(1 if bad else 0)
