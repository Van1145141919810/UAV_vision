#!/usr/bin/env python3
"""
stage12_gate.py —— 已改名 (2026-10-30)
===============================================================================
飞行入口现在是同目录的 stage14_mission.py (参数 / 档案 / 行为都和 stage12 一样, 只是代码模块化了)。
这个文件只打印提示并退出, 免得旧的 ~/fly.sh 或手打命令以为还在用老脚本。
"""
import os
import sys

new = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stage14_mission.py")
print("\n" + "!" * 78)
print("stage12_gate.py 已改名为 stage14_mission.py (2026-10-30 模块化, 行为不变)。请改用:")
print("    python3 %s %s" % (new, " ".join(sys.argv[1:])))
print("网页「起飞配置」重新生成一次启动脚本 (~/fly.sh) 就会用新名字。")
print("!" * 78 + "\n")
sys.exit(1)
