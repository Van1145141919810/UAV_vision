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

    # ---- 地面显示钩子 (gcs_gate.py, gate_gcs 模式; 2026-11-05) ----------------------------------
    # gcs_gate 看检测器有没有 hud_rows: 有 → 用下面这几个画 HUD / 叠加 / 控制台 / 掩膜; 没有 (环) → 它自己的环代码。
    # 地面只有像素量 (候选), 选哪个、世界系位置、法向都是飞机融合后回传的 fst (= 上行包头里的 /gate/status,
    # 滞后一个往返)。这些方法只用于显示, 不影响发给飞机的观测。

    def _sel_uv(self, fst):
        """飞机选中的轨迹最后一次观测的像素位置 (地面工作分辨率), 没有 → None。"""
        if not fst or fst.get("trk_id") is None:
            return None
        for c in fst.get("cands") or []:
            if c.get("id") == fst.get("trk_id") and c.get("uv"):
                return float(c["uv"][0]), float(c["uv"][1])
        return None

    def _z(self, c):
        """(双目 Z 或 None, 尺寸 Z) —— 不加视差偏置, 只给人看个大概。"""
        z_st = c.fb / c.disp if (c.disp == c.disp and c.disp > 0.5) else None
        return z_st, self.cfg.dot_d * self.rig.f / max(c.dmaj, 1e-6)

    def _best(self, fr, sel):
        if not fr.cands:
            return None
        if sel is not None:
            return min(fr.cands, key=lambda c: (c.u - sel[0]) ** 2 + (c.v - sel[1]) ** 2)
        return max(fr.cands, key=lambda c: c.ps)

    def found(self, fr):
        """这一帧有没有候选 (过了像素预筛; 最终用不用由飞机的门限 / 轨迹决定)。"""
        return bool(fr.cands)

    def ground_draw(self, img, fr, fst):
        import cv2
        out = img.copy()
        sel = self._sel_uv(fst)
        for c in fr.cands:
            r = max(2, int(round(c.dmaj / 2)))
            hit = sel is not None and (c.u - sel[0]) ** 2 + (c.v - sel[1]) ** 2 < max(6.0, 0.7 * r) ** 2
            col = (0, 255, 0) if hit else (0, 165, 255)
            cv2.circle(out, (int(round(c.u)), int(round(c.v))), r, col, 2 if hit else 1)
            z_st, z_sz = self._z(c)
            cv2.putText(out, "ps%.2f nw%d %s" % (c.ps, c.nw, ("Z%.1f" % z_st) if z_st else "mono"),
                        (int(c.u) + r + 3, int(c.v) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        return out

    def hud_rows(self, fr, fst):
        """→ [(本帧检测, 颜色), (飞机融合, 颜色)] 两行英文。"""
        GRAY, GREEN, YEL, CYAN = (160, 160, 160), (80, 220, 80), (0, 200, 255), (220, 220, 120)
        sel = self._sel_uv(fst)
        b = self._best(fr, sel) if fr is not None else None
        if b is None:
            d = fr.diag if fr is not None else {}
            r1 = ("dot cand 0   (components %s, kept %s)" % (d.get("n_comp", "-"), d.get("n_keep", "-")), GRAY)
        else:
            z_st, z_sz = self._z(b)
            r1 = ("dot cand %d   %s ps=%.2f nw=%d d=%.0fpx axr=%.2f disp=%s uniq=%.2f   Z st=%s size=%.2fm"
                  % (len(fr.cands), "SEL" if sel is not None else "best", b.ps, b.nw, b.dmaj, b.axr,
                     ("%.1fpx" % b.disp) if b.disp == b.disp else "--", b.uniq,
                     ("%.2f" % z_st) if z_st else "--", z_sz), GREEN if sel is not None else CYAN)
        if not fst or fst.get("trk_id") is None:
            r2 = ("fused: no track yet%s" % (("  rejected " + " ".join(fst.get("rejected") or []))
                                              if fst and fst.get("rejected") else ""), GRAY)
        else:
            if fst.get("n"):
                nn = "normal +-%.1fdeg span %.0fdeg" % (fst.get("n_sd_deg") or 0, fst.get("n_span_deg") or 0)
            else:
                nn = "normal --"
            r2 = ("fused: trk %s hits %s score %.2f  D %.1fcm  cands %s  %s"
                  % (fst.get("trk_id"), fst.get("n_hit"), float(fst.get("score") or 0),
                     float(fst.get("D_est") or 0) * 100, fst.get("n_cand"), nn),
                  GREEN if fst.get("published") else YEL)
        return [r1, r2]

    def probe_dist(self, fr):
        """探针模式大字: 最好候选的双目距离 (没有 → 尺寸距离)。"""
        b = self._best(fr, None) if fr is not None else None
        if b is None:
            return None
        z_st, z_sz = self._z(b)
        return z_st if z_st else z_sz

    def console(self, fr, fst):
        b = self._best(fr, self._sel_uv(fst))
        s = "候选 %d" % len(fr.cands)
        if b is not None:
            z_st, z_sz = self._z(b)
            s += " 最好: ps=%.2f nw=%d 长轴=%.0fpx 视差=%s Z(双目)=%s Z(尺寸)=%.2fm" % (
                b.ps, b.nw, b.dmaj, ("%.1f" % b.disp) if b.disp == b.disp else "--",
                ("%.2f" % z_st) if z_st else "--", z_sz)
        if fst and fst.get("trk_id") is not None:
            s += " | 飞机: 轨迹 %s 命中 %s 分 %.2f %s" % (
                fst.get("trk_id"), fst.get("n_hit"), float(fst.get("score") or 0),
                ("法向 ±%.1f°" % fst["n_sd_deg"]) if fst.get("n") else ("无法向: %s" % (fst.get("n_why") or "-")))
        return s

    def mask(self, L):
        """--show-mask: 红度 > red_thr 的像素 (白)。"""
        import cv2
        from .detect import red_map
        m = ((red_map(L) > self.cfg.red_thr).astype("uint8") * 255)
        return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)


class RedDotLocator(DotLocator):
    pass
