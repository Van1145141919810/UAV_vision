# -*- coding: utf-8 -*-
"""
vb09/perception/ring/draw.py —— 检测结果画图 (draw_frame)
===============================================================================
2026-10-30 从 gate_detector.py 原样搬来。
"""
from __future__ import annotations

import cv2
import numpy as np

from .det2d import apply_wb
from .detect import GateDetector
from .frame import GateFrame
from .geom3d import ring_tilt_deg


STAGE_COLOR = {"search": (128, 128, 128), "far": (255, 200, 0), "mid": (0, 255, 0), "near": (0, 220, 255),
               "terminal": (0, 128, 255), "coast": (200, 200, 200), "blind": (0, 0, 255), "lost": (80, 80, 80)}


def draw_frame(img, fr: GateFrame, det: GateDetector, T_WC=None, extra_lines=None, text=True):
    """
    extra_lines: 额外显示的文字行。
    text=False: 只画椭圆 / 角点 / 红点 / 法线 (不写字) —— 地面 gcs_gate 用:
                网页的数字走 X-Det, 本地窗口的字由 HUD 在 1920×1080 画布上画 (同 gcs_aruco_det2)。
    """
    vis = apply_wb(img, det.cfg).copy()
    rig = det.rig
    gL = fr.dbg.get("gL")
    if gL is not None and gL["ok"]:
        cv2.ellipse(vis, gL["center"], (0, 255, 0), 1, cv2.LINE_AA)
        cv2.ellipse(vis, gL["outer"], (0, 255, 255), 1, cv2.LINE_AA)
        cv2.ellipse(vis, gL["inner"], (255, 255, 0), 1, cv2.LINE_AA)
    for x, y in fr.dbg.get("matched", np.zeros((0, 2))):
        cv2.circle(vis, (int(x), int(y)), 2, (0, 140, 255), -1, cv2.LINE_AA)
    if fr.dot_roi is not None:
        x0, y0, x1, y1 = (int(v) for v in fr.dot_roi)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 255), 1)
    d = fr.dot()
    if d is not None:
        cv2.circle(vis, (int(d.pix[0]), int(d.pix[1])), int(d.diam_px / 2) + 5, (255, 0, 0), 2, cv2.LINE_AA)
    if fr.ring.ok and fr.ring.n is not None:
        # 法线箭头 (长 = 环半径): 红 = 角点平面, 品红 = 单目解已定向, 橙 = 近正对取平分线, 灰 = 两解没分清 (两个都画)
        c = fr.ring.c
        L_ = det.cfg.outer_d / 2

        def px(p):
            return (int(rig.f * p[0] / p[2] + rig.cx), int(rig.f * p[1] / p[2] + rig.cy))
        q0 = px(c)
        cv2.drawMarker(vis, q0, (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
        ns = [fr.ring.n] + ([fr.ring.n_alt] if fr.ring.ambiguous and fr.ring.n_alt is not None else [])
        col = (0, 0, 255) if fr.ring.src == "stereo" else (
            (160, 160, 160) if fr.ring.ambiguous else ((0, 165, 255) if fr.ring.n_src == "mono_mid" else (255, 0, 255)))
        for n_ in ns:
            n_ = n_ if n_ @ c < 0 else -n_            # 统一画成指向相机这一侧
            p = c + L_ * n_
            if p[2] > 0.05:
                cv2.arrowedLine(vis, q0, px(p), col, 2, cv2.LINE_AA, tipLength=0.2)
    tg = fr.target
    if tg.valid and T_WC is not None:
        R_WR, t_WR = det._T_WR(T_WC)
        pix, _ = det._project(tg.pos_w, R_WR, t_WR)
        if pix is not None:
            cv2.drawMarker(vis, (int(pix[0]), int(pix[1])), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
    if not text:
        return vis
    col = STAGE_COLOR.get(fr.stage, (255, 255, 255))
    lines = [f"[{fr.stage.upper()}]  {fr.t_ms.get('total', 0):.0f}ms"
             + (" ring-skipped" if fr.skipped_ring else "")]
    r = fr.ring
    if r.ok:
        s = f"ring {r.src} c=({r.c[0]:+.2f},{r.c[1]:+.2f},{r.c[2]:.2f}) corners={r.n_corners} side={r.side}"
        if r.src == "stereo":
            s += f" rms={r.plane_rms * 1000:.0f}mm r/R={r.r_free_ratio:.3f}"
        if r.n is not None:
            s += f"  n:{r.n_src or '?'} tilt={ring_tilt_deg(r.c, r.n):.0f}deg"
        lines.append(s)
    for dd in fr.dots:
        if dd.src == "mono":
            lines.append(f"dot mono  pix=({dd.pix[0]:.0f},{dd.pix[1]:.0f}) d={dd.diam_px:.0f}px")
        else:
            lines.append(f"dot {dd.src:9s} Z={dd.p[2]:.3f}m  diam={dd.metric_d * 100:.1f}cm  "
                         f"sz={dd.sigma_z * 100:.1f}cm" + (f"  rp_delta={dd.rp_delta * 100:+.1f}cm"
                                                           if dd.rp_delta == dd.rp_delta else ""))
    if tg.valid:
        sd = np.sqrt(np.diag(tg.cov_w))
        lines.append(f"TARGET {tg.src} dist={tg.dist:.2f}m sd=({sd[0] * 100:.1f},{sd[1] * 100:.1f},{sd[2] * 100:.1f})cm "
                     f"age={tg.age * 1000:.0f}ms side={tg.side}")
    if fr.rejected:
        lines.append("rejected: " + ",".join(fr.rejected))
    dg = fr.diag
    if dg:
        lines.append("diag " + "  ".join("%s=%s" % (k, v) for k, v in dg.items()))
    lines += list(extra_lines or [])
    for i, s in enumerate(lines):
        y = 20 + 18 * i
        cv2.putText(vis, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.47, col if i == 0 else (255, 255, 255), 1,
                    cv2.LINE_AA)
    return vis
