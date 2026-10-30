# -*- coding: utf-8 -*-
"""
vb09/perception/ring/locate.py —— 机上: 融合结果 → /gate/status 的环专用附加字段
===============================================================================
2026-10-30 从 vision_bridge._ring_extra 原样搬来 (只把 rospy 告警换成 warn 回调)。
以后的 Locator 插件接口 (update / reset / roi_request) 也放这里; 本轮 /gate/status 字段结构不变。
"""
from __future__ import annotations

import numpy as np

from .fusion import _cov_along_ray


def ring_extra(fuser, fr, T_WC, warn=None):
    """
    ★2026-10-19 按环心穿 (stage12): 估计器的 target 是**红点**; 红点没定位上时它是"环心 + 0.3m 先验"
    (ring_prior), σ 停在 6~7cm 压不下去。所以另发这一帧的环心观测, 让 stage12 自己多帧取中值:
      ring_w    这一帧的环心 (world)             ring_cov  它的协方差 (m², 行主序 3×3)
      ring_src  stereo (角点平面) / mono (椭圆)   ring_c    估计器累积的环心 (world, 显示用)
      n         估计器的环法向 (world, 已翻成指向相机一侧; 至少 3 次可信法向才有)
      aim       红点 (world) —— 只有估计器的来源是 dot_* 时才发
    """
    out = {}
    try:
        R_WR, t_WR = fuser.T_WR(T_WC)
        if R_WR is None:
            return out
        est, ring = fuser.est, fr.ring
        if ring.ok and ring.c is not None:
            cw = R_WR @ ring.c + t_WR
            C = R_WR @ _cov_along_ray(ring.c, max(ring.sigma_lat, 1e-3), max(ring.sigma_z, 1e-3)) @ R_WR.T
            out.update(ring_w=[round(float(v), 4) for v in cw], ring_src=ring.src,
                       ring_cov=[float("%.4g" % v) for v in C.reshape(-1)],
                       ring_partial=bool(ring.partial))
        if est.ring_c is not None:
            out["ring_c"] = [round(float(v), 4) for v in est.ring_c]
        n = est.ring_n
        if n is not None and est.ring_c is not None:
            n = np.asarray(n, float)
            if (t_WR - est.ring_c) @ n < 0:
                n = -n
            out["n"] = [round(float(v), 5) for v in n]
        tg = fr.target
        if tg is not None and tg.valid and str(tg.src).startswith("dot"):
            out["aim"] = [round(float(v), 4) for v in tg.pos_w]
    except Exception as e:                       # 附加字段出错不能影响主链路
        if warn is not None:
            warn("[gate] ring 附加字段出错: %r" % (e,))
    return out
