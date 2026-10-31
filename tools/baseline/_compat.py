# -*- coding: utf-8 -*-
"""
_compat.py —— 基线脚本找被测代码的唯一地方。重构搬家时只改这里, 各脚本不动。
每个函数返回"当前布局"下的对象; 存档布局 (平铺文件) 和新布局 (vb09/ 包) 都认。
"""
import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
STUBS = os.path.join(ROOT, "sim", "stubs")
GOLDEN = os.path.join(HERE, "golden")
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def use_stubs():
    """桩 rospy / 消息 (sim/stubs)。要在 import 被测 ROS 模块之前调用。"""
    if STUBS not in sys.path:
        sys.path.insert(0, STUBS)


def _first(*names):
    err = None
    for n in names:
        try:
            return importlib.import_module(n)
        except ImportError as e:
            if e.name and not n.startswith(e.name) and e.name not in n:
                raise
            err = e
    raise err


def StereoRig():
    return _first("vb09.perception.stereo_rig", "ring_detector").StereoRig


def det():
    """GateDetector / GateConfig"""
    return _first("vb09.perception.ring.detect", "gate_detector")


def wire():
    """frame_to_msg / frame_from_msg / rig_to_msg"""
    return _first("vb09.common.wire", "gate_detector")


def gate_plan():
    return _first("vb09.flight.plan.gate_plan", "gate_plan")


def bridge():
    use_stubs()
    return _first("vision_bridge")


def entry():
    """飞行入口脚本路径 (dry-run 用)"""
    for n in ("stage14_mission.py", "stage12_gate.py"):
        p = os.path.join(ROOT, n)
        if os.path.exists(p):
            return p
    raise FileNotFoundError("no flight entry script")


SITE_CONF = ("task=穿环\nscene=室内\ntakeoff_h=1.5\nz_low=0.8\nz_high=2.2\nxy_radius=10\n"
             "dash_v=2.5\nd_back=4.0\nd_rear=2.0\nd_side=2.0\nl_f=0.30\nl_r=0.30\nw_half=0.25\n")


def flight_module():
    """
    import 飞行脚本 (桩 rospy), 档案 = SITE_CONF。→ 模块。
    存档布局: stage12_gate (import 时 setup/finalize)。
    """
    import tempfile
    use_stubs()
    d = tempfile.mkdtemp()
    with open(os.path.join(d, "bl.conf"), "w") as f:
        f.write(SITE_CONF)
    os.environ["HOME"] = tempfile.mkdtemp()
    argv = sys.argv
    name = os.path.basename(entry())[:-3]
    sys.argv = [name + ".py", "--sites-dir", d, "--site", "bl"]
    try:
        return _first(name)
    finally:
        sys.argv = argv


def judge():
    """→ (judge(st, now, p_now) → (sample, why), fuse_samples(samples, p_ref, d))"""
    S = flight_module()
    GateInput = getattr(S, "GateInput", None) or _first("vb09.flight.target_in").GateInput
    gi = GateInput()
    return gi.judge, _first("gate_plan").fuse_samples, S
