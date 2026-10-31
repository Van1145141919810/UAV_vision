# -*- coding: utf-8 -*-
"""
vb09/common/wire.py —— 地面 ↔ 飞机的网络格式: msgpack / zmq + 标定摘要 + 版本检查 + 按目标类型分发序列化
===============================================================================
2026-10-30 从 gate_detector.py / gcs_gate.py / vision_bridge.py 合并; 2026-11-01 接线: 环的
frame_to_msg / frame_from_msg 搬到 vb09/perception/ring/wire.py, 这里按 kind 分发 (默认 "ring")。

  pack / unpack     msgpack (use_bin_type / raw=False)
  require_msgpack(msg) / require_zmq(msg)   缺包时 sys.exit(msg) —— 各入口自己给提示文字
  codec(kind)       这种目标的序列化模块 (registry.CODECS): MSG_VERSION / frame_to_msg / frame_from_msg / LIB_FILES
  frame_to_msg(fr, kind) / frame_from_msg(m, kind)   = codec(kind) 的同名函数
  rig_to_msg(rig, cfg, kind)   标定摘要 {f, cx, cy, B, R1, size} + 版本身份 {lib, cfg, kind}

★2026-10-31 版本检查 (PLAN_refactor_v2 第 6 步 3):
  lib_version(kind)  检测库指纹 = COMMON_LIB_FILES + codec(kind).LIB_FILES 内容 (换行统一成 \n) 的 sha256 前 12 位
  cfg_digest(cfg)    检测参数 (dataclass) 全部字段的 sha256 前 12 位
  peer_mismatch(gate_msg, rig_msg, cfg, kind)  飞机侧: 目标类型 / 消息版本 v / 检测库 / 参数和本机不一致
    → 原因列表 (空 = 一致)。不一致时桥把这一帧强制当 SIM 发布 (不驱动), /gate/status 带 ver_bad = 原因列表。
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
from dataclasses import asdict

import numpy as np

from vb09.perception import registry

try:
    import zmq
except ImportError:
    zmq = None
try:
    import msgpack
except ImportError:
    msgpack = None


def pack(o):
    return msgpack.packb(o, use_bin_type=True)


def unpack(b):
    return msgpack.unpackb(b, raw=False)


def require_msgpack(msg="需要 msgpack:  pip3 install msgpack"):
    if msgpack is None:
        sys.exit(msg)


def require_zmq(msg="需要 pyzmq:  pip3 install pyzmq"):
    if zmq is None:
        sys.exit(msg)


# =============================================================================
# 一帧检测结果: 按目标类型分发 (registry.CODECS)
# =============================================================================
def codec(kind="ring"):
    return registry.load(registry.CODECS, kind)


def frame_to_msg(fr, kind="ring") -> dict:
    return codec(kind).frame_to_msg(fr)


def frame_from_msg(m, kind="ring"):
    return codec(kind).frame_from_msg(m)


def _vec(a):
    return None if a is None else [float(x) for x in np.asarray(a, float).ravel()]


def rig_to_msg(rig, cfg=None, kind="ring") -> dict:
    """飞机侧融合只需要这几个数 (和地面检测用的是**同一份**标定)。另带 lib / cfg / kind 给飞机核对版本。"""
    m = {"f": float(rig.f), "cx": float(rig.cx), "cy": float(rig.cy), "B": float(rig.B),
         "R1": _vec(rig.R1), "size": [int(rig.size[0]), int(rig.size[1])], "lib": lib_version(kind),
         "kind": kind}
    if cfg is not None:
        m["cfg"] = cfg_digest(cfg)
    return m


# =============================================================================
# 版本检查 (见文件头)
# =============================================================================
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# 决定"地面发的观测是什么意思"的公共文件 (序列化分发 + 校正); 各目标自己的在 codec(kind).LIB_FILES
COMMON_LIB_FILES = ("vb09/common/wire.py", "vb09/perception/stereo_rig.py")
_LIB_VER = {}


def lib_files(kind="ring"):
    return tuple(COMMON_LIB_FILES) + tuple(codec(kind).LIB_FILES)


def lib_version(kind="ring") -> str:
    """lib_files(kind) 的内容指纹。CRLF → LF (Windows 地面站 git 检出可能是 CRLF)。缺文件记 "missing"。"""
    if kind not in _LIB_VER:
        h = hashlib.sha256()
        for rel in lib_files(kind):
            h.update(rel.encode("utf-8") + b"\0")
            try:
                with open(os.path.join(_ROOT, *rel.split("/")), "rb") as f:
                    h.update(f.read().replace(b"\r\n", b"\n"))
            except OSError:
                h.update(b"missing")
            h.update(b"\0")
        _LIB_VER[kind] = h.hexdigest()[:12]
    return _LIB_VER[kind]


def cfg_digest(cfg) -> str:
    """检测参数 (dataclass, 环 = GateConfig) 全部字段 → 指纹。float 用 repr, 和 gate_params.json 的写法无关。"""
    d = asdict(cfg)
    return hashlib.sha256(json.dumps(d, sort_keys=True, default=repr).encode("utf-8")).hexdigest()[:12]


def peer_mismatch(gate_msg, rig_msg, cfg, kind="ring") -> list:
    """飞机侧: 地面这一包和本机不一致的地方 → 原因列表 (中文短句); 一致 → []。"""
    bad = []
    rig_msg = rig_msg if isinstance(rig_msg, dict) else {}
    k = rig_msg.get("kind", "ring")              # 接线之前的地面程序不带 kind = 环
    if k != kind:
        bad.append("目标类型 地面 %s ≠ 本机 %s" % (k, kind))
        return bad                               # 类型都不同, 下面几项没有可比性
    v = gate_msg.get("v") if isinstance(gate_msg, dict) else None
    mv = codec(kind).MSG_VERSION
    if v != mv:
        bad.append("消息版本 v=%s (本机 %d)" % (v, mv))
    lib = rig_msg.get("lib")
    if lib is None:
        bad.append("地面没报检测库版本 (旧版 gcs_gate?)")
    elif lib != lib_version(kind):
        bad.append("检测库 地面 %s ≠ 本机 %s" % (lib, lib_version(kind)))
    if cfg is not None:
        c = rig_msg.get("cfg")
        mine = cfg_digest(cfg)
        if c is None:
            bad.append("地面没报 GateConfig 摘要")
        elif c != mine:
            bad.append("GateConfig 地面 %s ≠ 本机 %s (gate_params.json 两边不一样?)" % (c, mine))
    return bad
