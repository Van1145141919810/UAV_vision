# -*- coding: utf-8 -*-
"""
tools/reddot_depth_sim.py —— 红点深度的合成回归 (numpy only, 不要 cv2 / 录像)

    python3 tools/reddot_depth_sim.py

2026-11-05 真机现象各一个场景 (地面半分辨率 f≈813, 红点 4.01 m):
  reacquire   估计器重建后重新看到 (以前: 标称直径 → 4.4x m 几秒后跳回 4.05) → 全程离真值 < 10 cm
  inherit     挡住 > 1 s (桥重建估计器, inherit 带直径) → 第一帧就融合尺寸, 离真值 < 10 cm
  rotate      转头 ±15° (位姿在转, 峰值 47°/s) + 模糊随角速度把长轴拉长 (峰值 +20%) → 转动期间不融合尺寸, 离真值 < 15 cm
  sizes       10 / 15 / 21 cm 都接受且直径估对 (±5%); 28 cm (红桶) 拒绝
  swap        同一位置 15 cm 换成 10 cm → 距离不跳 (< 15 cm), 直径几秒内改学成 10 cm
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from vb09.perception.reddot.config import DotConfig        # noqa: E402
from vb09.perception.reddot.frame import Cand, DotFrame    # noqa: E402
from vb09.perception.reddot.locate import DotLocator       # noqa: E402

CFG = DotConfig()
F = 813.3
FB = F * 0.0602
Z0 = 4.01
R_BC = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
P_DOT = np.array([Z0, 0.0, 0.0])
RNG = np.random.default_rng(7)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T_WC(yaw):
    T = np.eye(4)
    T[:3, :3] = Rz(yaw) @ R_BC
    return T


def frame(loc, t, yaw=0.0, D=0.15, blur=1.0):
    pc = (Rz(yaw) @ R_BC).T @ P_DOT
    Z = pc[2]
    u = 480 + F * pc[0] / Z
    dmaj = F * D / Z * 0.985 * blur * (1 + RNG.normal(0, 0.01))       # 面积法剩 ~1.5% 偏小
    c = Cand(u=u, v=270, xn=(u - 480) / F, fb=FB, dmaj=dmaj, dmin=dmaj * 0.98, fill=0.78, circ=0.9, axr=0.98,
             red=110, nw=8, disp=FB / Z + RNG.normal(0, 0.3), pk=0.97, uniq=0.3)
    fr = DotFrame(t=t, cands=[c])
    loc.update(fr, T_WC(yaw), t)
    return fr


def err(fr):
    return abs(fr.target.dist - Z0) if fr.target.valid else None


def run():
    res = []

    def check(name, ok, msg):
        res.append(ok)
        print("%-4s %-10s %s" % ("ok" if ok else "FAIL", name, msg))

    loc = DotLocator(CFG, F, np.eye(3))
    e = [err(frame(loc, k / 10)) for k in range(60)]
    e = [x for x in e if x is not None]
    check("reacquire", max(e[2:]) < 0.10, "最大误差 %.3f m (前两帧除外, 单帧双目噪声)" % max(e[2:]))

    old, loc = loc, DotLocator(CFG, F, np.eye(3))
    loc.inherit(old)
    frs = [frame(loc, 20 + k / 10) for k in range(30)]
    e = [err(f) for f in frs if f.target.valid]
    check("inherit", frs[1].diag.get("D_trk_cm", -1) > 0 and max(e[1:]) < 0.10,
          "第 2 帧直径 %s cm, 最大误差 %.3f m" % (frs[1].diag.get("D_trk_cm"), max(e[1:])))

    worst, fused_moving = 0.0, 0
    for k in range(40):
        moving = k < 20
        yaw = math.radians(15) * math.sin(2 * math.pi * k / 20) if moving else 0.0
        rate = abs(47.1 * math.cos(2 * math.pi * k / 20)) if moving else 0.0     # °/s, 峰值 47
        fr = frame(loc, 30 + k / 10, yaw=yaw, blur=1.0 + 0.2 * rate / 47.1)     # 模糊随角速度, 峰值长轴 +20%
        if fr.target.valid:
            worst = max(worst, err(fr))
        if fr.diag.get("rate_dps", 0) > CFG.rate_max_deg and fr.target.src == "dot_fused":
            fused_moving += 1
    check("rotate", worst < 0.15 and fused_moving == 0, "最大误差 %.3f m, 转动中融合尺寸 %d 帧" % (worst, fused_moving))

    for D, want in ((0.10, True), (0.15, True), (0.21, True), (0.28, False)):
        loc = DotLocator(CFG, F, np.eye(3))
        for k in range(40):
            fr = frame(loc, k / 10, D=D)
        Dt = fr.diag.get("D_trk_cm", -1)
        if want:
            check("size%2.0f" % (D * 100), fr.target.valid and abs(Dt / (D * 100) - 1) < 0.05,
                  "直径 %.1f cm, 误差 %.3f m" % (Dt, err(fr)))
        else:
            check("size%2.0f" % (D * 100), not fr.target.valid, "拒绝: %s" % ",".join(fr.rejected))

    loc = DotLocator(CFG, F, np.eye(3))
    for k in range(40):
        frame(loc, k / 10, D=0.15)
    worst = 0.0
    for k in range(80):
        fr = frame(loc, 4 + k / 10, D=0.10)
        worst = max(worst, err(fr) or 0.0)
    Dt = fr.diag.get("D_trk_cm", -1)
    check("swap", worst < 0.15 and abs(Dt - 10) < 0.6, "最大误差 %.3f m, 8 s 后直径 %.1f cm" % (worst, Dt))

    print("=" * 60)
    print("ALL PASS" if all(res) else "FAILED")
    return 0 if all(res) else 1


if __name__ == "__main__":
    sys.exit(run())
