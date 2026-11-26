# -*- coding: utf-8 -*-
"""在飞机上验证：桥会读到哪份红点参数、生效值是多少"""
import os
import sys

sys.path.insert(0, os.getcwd())
from vb09.perception import registry

path = os.path.join(os.getcwd(), "gate_params.json")   # 桥的默认路径
obj, src = registry.load_config("reddot", path)
print("桥会读的参数文件:", src)
print("（默认路径是 gate_params.json，kind=reddot 时自动改读同目录 reddot_params.json）")
print()
keys = ("dot_d", "min_d_px", "open_k", "d_max", "min_score",
        "white_min", "red_min", "mono_score_min", "lost_timeout", "max_cands")
for k in keys:
    print("  %-16s = %s" % (k, getattr(obj, k, "?")))

# 用这几个参数算一下理论可检距离
fx = 1566.149
print()
print("  20 cm 红点可检距离 ≈ %.0f m" % (fx * 0.20 / obj.min_d_px))
print("  26 cm 红点可检距离 ≈ %.0f m" % (fx * 0.26 / obj.min_d_px))
