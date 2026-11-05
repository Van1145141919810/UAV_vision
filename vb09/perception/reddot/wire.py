# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/wire.py —— 红点一帧检测结果 ↔ 网络字典 (地面 → 飞机)
===============================================================================
接口同 ring/wire.py: MSG_VERSION / frame_to_msg / frame_from_msg / LIB_FILES。
只传候选的像素量 (Cand), 不带世界系 / 位姿 / 图像。改了字段含义就把 MSG_VERSION 加 1。
网络输入当不可信数据: frame_from_msg 对缺失 / 非法字段直接丢掉, 不抛异常; 候选最多收 32 个。
"""
from __future__ import annotations

import math

from vb09.perception.reddot.frame import Cand, DotFrame

# 决定"地面发的红点观测是什么意思"的文件 (locate / judge 是飞机侧逻辑, 不算)
LIB_FILES = ("vb09/perception/reddot/config.py", "vb09/perception/reddot/frame.py",
             "vb09/perception/reddot/detect.py", "vb09/perception/reddot/wire.py",
             "vb09/perception/reddot/plugin.py")

MSG_VERSION = 1
_MAX_CANDS = 32

# (Cand 属性, 网络键, 小数位)
_FIELDS = (("u", "u", 2), ("v", "v", 2), ("xn", "xn", 5), ("yn", "yn", 5), ("fb", "fb", 3),
           ("dmaj", "dj", 2), ("dmin", "dn", 2), ("mx", "mx", 4), ("my", "my", 4),
           ("area", "a", 0), ("fill", "fl", 3), ("circ", "ci", 3), ("axr", "ax", 3),
           ("red", "rd", 1), ("sat", "st", 1), ("disp", "ds", 3), ("pk", "pk", 3),
           ("uniq", "uq", 3), ("dy", "dy", 3), ("ps", "ps", 3))


def _f(x, default=None):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def frame_to_msg(fr: DotFrame) -> dict:
    cands = []
    for c in fr.cands[:_MAX_CANDS]:
        d = {}
        for attr, key, nd in _FIELDS:
            v = getattr(c, attr)
            if v is None or (isinstance(v, float) and not math.isfinite(v)):
                continue                      # 没配上的视差不发 (收端当 NaN)
            d[key] = round(float(v), nd) if nd else int(round(float(v)))
        d["nw"], d["nn"], d["pt"] = int(c.nw), int(c.nn), bool(c.partial)
        cands.append(d)
    return {"v": MSG_VERSION, "t": float(fr.t), "stage": fr.stage, "cands": cands,
            "diag": {k: v for k, v in fr.diag.items() if isinstance(v, (bool, int, float, str))},
            "t_ms": {k: round(float(v), 2) for k, v in fr.t_ms.items()}}


def frame_from_msg(m: dict) -> DotFrame:
    fr = DotFrame(t=_f(m.get("t"), 0.0))
    for d in (m.get("cands") or [])[:_MAX_CANDS]:
        if not isinstance(d, dict):
            continue
        u, v, xn, yn, fb = (_f(d.get(k)) for k in ("u", "v", "xn", "yn", "fb"))
        dj, dn = _f(d.get("dj")), _f(d.get("dn"))
        if None in (u, v, xn, yn, fb, dj, dn) or fb <= 0 or dj <= 0 or dn < 0 or dn > dj * 1.05:
            continue                          # 缺关键量 → 丢掉这个候选
        mx, my = _f(d.get("mx"), 0.0), _f(d.get("my"), 1.0)
        nm = math.hypot(mx, my)
        if nm < 1e-6:
            mx, my = 0.0, 1.0
        else:
            mx, my = mx / nm, my / nm
        c = Cand(u=u, v=v, xn=xn, yn=yn, fb=fb, dmaj=dj, dmin=dn, mx=mx, my=my,
                 area=_f(d.get("a"), 0.0), fill=_f(d.get("fl"), 0.0), circ=_f(d.get("ci"), 0.0),
                 axr=_f(d.get("ax"), 1.0), red=_f(d.get("rd"), 0.0), sat=_f(d.get("st"), 0.0),
                 disp=_f(d.get("ds"), float("nan")), pk=_f(d.get("pk"), 0.0), uniq=_f(d.get("uq"), 1.0),
                 dy=_f(d.get("dy"), 0.0), ps=_f(d.get("ps"), 0.0),
                 nw=max(0, min(8, int(_f(d.get("nw"), 0)))), nn=max(0, min(8, int(_f(d.get("nn"), 0)))),
                 partial=bool(d.get("pt")))
        fr.cands.append(c)
    diag = m.get("diag")
    if isinstance(diag, dict):
        fr.diag = {k: v for k, v in diag.items() if isinstance(v, (bool, int, float, str))}
    tms = m.get("t_ms")
    if isinstance(tms, dict):
        fr.t_ms = {k: float(v) for k, v in tms.items() if isinstance(v, (int, float))}
    return fr
