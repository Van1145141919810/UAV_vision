# -*- coding: utf-8 -*-
"""
vb09/perception/ring/wire.py —— 环的一帧检测结果 ↔ 网络字典 (地面 → 飞机)
===============================================================================
2026-11-01 从 vb09/common/wire.py 原样搬来 (接线: 每种目标自己的序列化, common/wire 按 kind 分发)。
  MSG_VERSION                     这份格式的版本 (下行包 gate.v); 改了字段含义就加 1
  frame_to_msg(fr) / frame_from_msg(m)   相机 (rect) 系观测 ↔ 纯 Python 字典
  LIB_FILES                       这种目标的检测库文件 (版本指纹用, 见 common/wire.lib_version)
网络输入当不可信数据: frame_from_msg 对缺失/非法字段直接丢掉, 不抛异常。
"""
from __future__ import annotations

import numpy as np

from vb09.perception.ring.frame import DotObs, GateFrame

# 决定"地面发的环观测是什么意思"的文件 (画图 / judge / locate 不算)
LIB_FILES = ("vb09/perception/ring/config.py", "vb09/perception/ring/frame.py",
             "vb09/perception/ring/det2d.py", "vb09/perception/ring/geom3d.py",
             "vb09/perception/ring/detect.py", "vb09/perception/ring/fusion.py",
             "vb09/perception/ring/wire.py", "vb09/perception/ring/plugin.py")


# =============================================================================
# 网络序列化（地面检测 → 飞机融合）
# =============================================================================
MSG_VERSION = 1
_DOT_SRC = ("stereo", "ray_plane", "mono")
_RING_SRC = ("stereo", "mono")
_NSRC = ("plane", "mono_plane", "mono_disp", "mono_est", "mono_mid", "mono_amb")


def _vec(a):
    return None if a is None else [float(x) for x in np.asarray(a, float).ravel()]


def _num(x, nd=6):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return round(x, nd) if np.isfinite(x) else None


def _arr3(v):
    """网络里来的向量: 必须是 3 个有限数, 否则 None。"""
    if v is None:
        return None
    try:
        a = np.asarray(v, float).ravel()
    except (TypeError, ValueError):
        return None
    return a if a.size == 3 and np.all(np.isfinite(a)) else None


def _pos(x, default=0.0):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return default
    return x if np.isfinite(x) and x > 0 else default


def frame_to_msg(fr: GateFrame) -> dict:
    """检测结果 → 纯 Python 字典 (msgpack/JSON 都能装)。只带相机(rect)系观测, 不带图像/位姿。"""
    r = fr.ring
    return {
        "v": MSG_VERSION, "t": float(fr.t), "stage": fr.stage,
        "ring_seen": bool(fr.ring_seen), "skipped_ring": bool(fr.skipped_ring),
        "dot_roi": [float(v) for v in fr.dot_roi] if fr.dot_roi is not None else None,
        "ring": {"ok": bool(r.ok), "src": r.src, "c": _vec(r.c), "n": _vec(r.n),
                 "sl": _num(r.sigma_lat), "sz": _num(r.sigma_z), "partial": bool(r.partial),
                 "side": r.side, "side_conf": _num(r.side_conf), "nc": int(r.n_corners),
                 "rfr": _num(r.r_free_ratio), "amb": bool(r.ambiguous), "prms": _num(r.plane_rms),
                 "nsrc": r.n_src, "n2": _vec(r.n_alt), "c2": _vec(r.c_alt)},
        "dots": [{"src": d.src, "p": _vec(d.p), "pix": [float(d.pix[0]), float(d.pix[1])],
                  "d": _num(d.diam_px), "md": _num(d.metric_d), "sl": _num(d.sigma_lat),
                  "sz": _num(d.sigma_z), "rp": _num(d.rp_delta)}
                 for d in fr.dots if d.ok and d.p is not None],
        "diag": {k: v for k, v in fr.diag.items() if isinstance(v, (bool, int, float, str))},
        "t_ms": {k: round(float(v), 2) for k, v in fr.t_ms.items()},
    }


def frame_from_msg(m: dict) -> GateFrame:
    """frame_to_msg 的逆。网络输入当不可信数据: 字段缺失/非法的观测直接丢掉, 不抛异常。"""
    fr = GateFrame(t=float(m.get("t") or 0.0))
    fr.ring_seen = bool(m.get("ring_seen"))
    fr.skipped_ring = bool(m.get("skipped_ring"))
    roi = m.get("dot_roi")
    if isinstance(roi, (list, tuple)) and len(roi) == 4:
        fr.dot_roi = tuple(float(v) for v in roi)
    r = m.get("ring") or {}
    c, n = _arr3(r.get("c")), _arr3(r.get("n"))
    ring = fr.ring
    ring.partial = bool(r.get("partial"))
    ring.side = r.get("side") if r.get("side") in ("front", "back") else None
    ring.side_conf = _pos(r.get("side_conf"))
    ring.n_corners = int(r.get("nc") or 0)
    if r.get("ok") and r.get("src") in _RING_SRC and c is not None and n is not None and c[2] > 0:
        nn = float(np.linalg.norm(n))
        if nn > 1e-6:
            ring.ok, ring.src, ring.c, ring.n = True, r["src"], c, n / nn
            ring.sigma_lat = _pos(r.get("sl"), 0.05)
            ring.sigma_z = _pos(r.get("sz"), 0.1)
            ring.ambiguous = bool(r.get("amb"))
            ring.r_free_ratio = float(r["rfr"]) if r.get("rfr") is not None else float("nan")
            ring.plane_rms = float(r["prms"]) if r.get("prms") is not None else float("nan")
            ring.n_src = r.get("nsrc") if r.get("nsrc") in _NSRC else ""
            n2 = _arr3(r.get("n2"))
            if ring.ambiguous and n2 is not None and float(np.linalg.norm(n2)) > 1e-6:
                ring.n_alt = n2 / float(np.linalg.norm(n2))
                c2 = _arr3(r.get("c2"))            # 老版地面端没有 c2 → 换法线时环心不换 (同以前)
                if c2 is not None and c2[2] > 0:
                    ring.c_alt = c2
    for d in m.get("dots") or []:
        if not isinstance(d, dict) or d.get("src") not in _DOT_SRC:
            continue
        p = _arr3(d.get("p"))
        pix = d.get("pix")
        if p is None or p[2] <= 0 or not (isinstance(pix, (list, tuple)) and len(pix) == 2):
            continue
        if d["src"] == "mono":
            p = p / np.linalg.norm(p)
        o = DotObs(True, d["src"], p, (float(pix[0]), float(pix[1])), _pos(d.get("d")),
                   float(d["md"]) if d.get("md") is not None else float("nan"),
                   _pos(d.get("sl"), 0.01), _pos(d.get("sz"), 0.05))
        if d.get("rp") is not None:
            o.rp_delta = float(d["rp"])
        fr.dots.append(o)
    diag = m.get("diag")
    if isinstance(diag, dict):
        fr.diag = {k: v for k, v in diag.items() if isinstance(v, (bool, int, float, str))}
    tms = m.get("t_ms")
    if isinstance(tms, dict):
        fr.t_ms = {k: float(v) for k, v in tms.items() if isinstance(v, (int, float))}
    return fr
