# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/plugin.py —— 红点插件: Detector / Locator 两个适配器 (registry.DETECTORS / LOCATORS)
===============================================================================
接口同 ring/plugin.py。
  RedDotDetector(rig, cfg)   process(L, R, t, T_WC=None) → DotFrame   (给 T_WC 就在同进程里做机上定位)
  RedDotLocator              = locate.DotLocator (机上融合; gate_gcs 模式)
★ 顶层不 import cv2: LOCATORS 只要 numpy。
"""
from __future__ import annotations

from .locate import DotLocator


class RedDotDetector(object):
    kind = "reddot"

    def __init__(self, rig, cfg):
        from .detect import DotDetectorCore       # cv2 —— 只有检测端才 import
        self._cls = DotDetectorCore
        self.rig, self.cfg = rig, cfg
        self.det = DotDetectorCore(rig, cfg)
        self.loc = DotLocator(cfg, rig.f, rig.R1)

    def process(self, L, R, t, T_WC=None):
        fr = self.det.process(L, R, t)
        if T_WC is not None:
            self.loc.update(fr, T_WC, t)
        return fr

    def reset(self):
        self.det = self._cls(self.rig, self.cfg)
        self.loc = DotLocator(self.cfg, self.rig.f, self.rig.R1)

    def draw(self, img, fr, T_WC=None, text=True):
        import cv2
        out = img.copy()
        info = fr.dbg.get("dot") or {}
        sel = None
        best = info.get("best")
        if best is not None:
            sel = (best.last["u"], best.last["v"])
        for c in fr.cands:
            r = max(2, int(round(c.dmaj / 2)))
            hit = sel is not None and abs(c.u - sel[0]) < 1.0 and abs(c.v - sel[1]) < 1.0
            col = (0, 255, 0) if hit else (0, 165, 255)
            cv2.circle(out, (int(round(c.u)), int(round(c.v))), r, col, 2)
            if text:
                z = self.cfg.dot_d * self.loc.f / c.dmaj
                cv2.putText(out, "ps%.2f nw%d z%.1f" % (c.ps, c.nw, z), (int(c.u) + r + 4, int(c.v)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        if text:
            cv2.putText(out, "%s %s" % (fr.stage, " ".join(fr.rejected)), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return out

    def extra(self, fr, T_WC, warn=None):
        return self.loc.extra(fr, T_WC, warn=warn)


class RedDotLocator(DotLocator):
    pass
