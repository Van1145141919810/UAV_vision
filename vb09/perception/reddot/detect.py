# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/detect.py —— 红点检测 (地面 / 机上检测端, 用 cv2): 校正后左右目 → 候选的像素量
===============================================================================
流程 (claude/RESULT_reddot_p1_2026-11-05.md 验证过的那套):
  1. r = R − max(G,B) > red_thr → 开运算 → 连通域; 面积按 [min_d_px, max_d_px] 筛
  2. 每个连通域算像素特征: 红度、填充率(最小外接矩形)、圆度、长短轴比、周围白环扇区数
  3. 松的预筛 + 像素分 ps, 取前 max_cands 个
  4. 左右配对: 左目候选的红度图模板, 在右目**同一行带**里沿视差范围做归一化互相关 (不是独立检测再配)
  5. 左右目各做一次"半高软权重矩"精化: 圆心 (u, v), 长轴 dmaj (= 圆盘直径, 与倾斜无关), 短轴 dmin 和短轴方向;
     视差 = 左右圆心之差。长轴 / 短轴比 → 倾角, 短轴方向 → 倾斜方位 (locate.py 用它们出法向)
这里**不做**: 测距 / 物理直径 / 世界系 / 严格门限 / 时序 (都在机上 locate.py)。
像素门限按 f / ref_f 缩放, 所以地面用半分辨率也能跑 (但 10 m 外的红点只剩 ~12 px, 建议全分辨率)。
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from .config import DotConfig
from .frame import Cand, DotFrame

NAN = float("nan")


def red_map(img):
    """r = R − max(G, B), uint8 (负值饱和成 0)。"""
    b, g, r = cv2.split(img)
    return cv2.subtract(r, cv2.max(b, g))


def _sectors(img, rm, cx, cy, R):
    """1.3–1.8R 环带 8 扇区: (亮且低饱和 = 白 的扇区数, 不红的扇区数)。"""
    H, W = rm.shape
    x0, x1 = max(0, int(cx - 2 * R)), min(W, int(cx + 2 * R) + 1)
    y0, y1 = max(0, int(cy - 2 * R)), min(H, int(cy + 2 * R) + 1)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return 0, 0
    hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    ys, xs = np.mgrid[y0:y1, x0:x1]
    dx, dy = xs - cx, ys - cy
    rr = np.hypot(dx, dy)
    ring = (rr > 1.3 * R) & (rr < 1.8 * R)
    sec = np.minimum(((np.arctan2(dy, dx) + np.pi) / (2 * np.pi) * 8).astype(int), 7)
    white = (hsv[..., 1] < 60) & (hsv[..., 2] > 150)
    redp = rm[y0:y1, x0:x1] > 40
    nw = nn = 0
    for k in range(8):
        mk = ring & (sec == k)
        if mk.sum() < 10:
            continue
        if white[mk].mean() > 0.6:
            nw += 1
        if redp[mk].mean() < 0.15:
            nn += 1
    return nw, nn


def half_max_moments(rm, u0, v0, Req):
    """
    在 (u0, v0) 附近、半径 ~1.8·Req 的窗口里, 把 r 图按"平台值 / 背景"归一化成 [0,1] 的软权重, 取 > 0.5 的
    连通域 (含窗口中心的那块), 做加权二阶矩 → dict(u, v, dmaj, dmin, mx, my, partial); 不行返回 None。
    dmaj = 4·σ_major (均匀圆盘的 σ = R/2 → 等于直径); 面积等效直径在倾斜时偏小, 所以不用面积。
    """
    H, W = rm.shape
    rad = int(Req * 1.8 + 6)
    x0, y0 = int(round(u0)) - rad, int(round(v0)) - rad
    x1, y1 = min(W, x0 + 2 * rad), min(H, y0 + 2 * rad)
    x0, y0 = max(0, x0), max(0, y0)
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None
    sub = rm[y0:y1, x0:x1].astype(np.float32)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    dd = np.hypot(xx - u0, yy - v0)
    inner = dd < 0.5 * Req
    outer = dd > 1.5 * Req
    if inner.sum() < 3:
        return None
    plateau = float(np.median(sub[inner]))
    bg = float(np.percentile(sub[outer], 30)) if outer.sum() >= 10 else 0.0
    if plateau - bg < 10.0:
        return None
    w = np.clip((sub - bg) / (plateau - bg), 0.0, 1.0) * (dd < 1.6 * Req)
    bw = (w > 0.5).astype(np.uint8)
    n, lab = cv2.connectedComponents(bw, connectivity=8)
    cy_i, cx_i = int(round(v0)) - y0, int(round(u0)) - x0
    k = 0
    if 0 <= cy_i < lab.shape[0] and 0 <= cx_i < lab.shape[1]:
        k = int(lab[cy_i, cx_i])
    if k == 0:                              # 中心没落在掩膜里 (右目误配 / 圆环中空): 取离中心最近的最大块
        best = None
        for j in range(1, n):
            ys, xs = np.nonzero(lab == j)
            dj = float(np.hypot(xs.mean() + x0 - u0, ys.mean() + y0 - v0))
            if dj < 0.8 * Req and (best is None or len(xs) > best[1]):
                best = (j, len(xs))
        if best is None:
            return None
        k = best[0]
    m = lab == k
    if m.sum() < 5:
        return None
    px, py, ww = xx[m].astype(np.float64), yy[m].astype(np.float64), w[m].astype(np.float64)
    sw = ww.sum()
    mu = np.array([(px * ww).sum() / sw, (py * ww).sum() / sw])
    d = np.stack([px - mu[0], py - mu[1]])
    cov = (d * ww) @ d.T / sw
    ev, evec = np.linalg.eigh(cov)
    ev = np.maximum(ev, 1e-9)
    mdir = evec[:, 0]                       # 短轴方向
    partial = bool(px.min() <= 1 or py.min() <= 1 or px.max() >= W - 2 or py.max() >= H - 2)
    return dict(u=float(mu[0]), v=float(mu[1]), dmaj=float(4 * np.sqrt(ev[1])), dmin=float(4 * np.sqrt(ev[0])),
                mx=float(mdir[0]), my=float(mdir[1]), partial=partial)


def _match_row(rL_f, rR_f, x, y, w, h, dmax):
    """红度图模板匹配 (同一行带) → (视差 seed, 峰值, 第二峰/第一峰, 右目行偏移 int) 或 None。"""
    H, W = rL_f.shape
    m = 6
    x0, x1 = max(0, x - m), min(W, x + w + m)
    y0, y1 = max(0, y - m), min(H, y + h + m)
    ya, yb = max(0, y0 - 3), min(H, y1 + 3)
    sx0, sx1 = max(0, x0 - dmax), min(W, x1 + 20)
    tpl = rL_f[y0:y1, x0:x1]
    srch = rR_f[ya:yb, sx0:sx1]
    if srch.shape[0] < tpl.shape[0] or srch.shape[1] < tpl.shape[1] or tpl.std() < 1e-3:
        return None
    res = cv2.matchTemplate(srch, tpl, cv2.TM_CCOEFF_NORMED)
    j = np.unravel_index(int(np.argmax(res)), res.shape)
    pk = float(res[j])
    dyi, dxi = int(j[0]), int(j[1])
    sub = 0.0
    if 0 < dxi < res.shape[1] - 1:
        a, b, c = res[dyi, dxi - 1], res[dyi, dxi], res[dyi, dxi + 1]
        den = a - 2 * b + c
        if abs(den) > 1e-9:
            sub = float(np.clip(0.5 * (a - c) / den, -1, 1))
    r2 = res.copy()
    r2[:, max(0, dxi - 8):dxi + 9] = -1.0
    pk2 = float(r2.max())
    uniq = (max(pk2, 0.0) / pk) if pk > 1e-6 else 1.0
    disp = x0 - (sx0 + dxi + sub)
    return disp, pk, uniq, (ya + dyi) - y0


class DotDetectorCore(object):
    """rect 左右目 → DotFrame (只有 cands)。"""

    def __init__(self, rig, cfg: DotConfig):
        self.rig, self.cfg = rig, cfg
        self.f, self.cx, self.cy = float(rig.f), float(rig.cx), float(rig.cy)
        self.fb = float(rig.f) * float(rig.B)
        self.s = self.f / cfg.ref_f
        self._k = np.ones((cfg.open_k, cfg.open_k), np.uint8)

    # ---- 像素预筛分 (地面只用来排序 / 截断; 严格门限在机上) ----
    def _score(self, fill, circ, axr, red, nw, strict):
        c = self.cfg
        t_fill = 1.0 - min(1.0, abs(fill - 0.785) / (0.12 if strict else 0.2))
        t_circ = float(np.clip((circ - 0.5) / 0.4, 0, 1))
        t_axr = float(np.clip((axr - 0.4) / 0.4, 0, 1))
        t_red = float(np.clip((red - 50.0) / 80.0, 0, 1))
        return 0.30 * t_fill + 0.20 * t_circ + 0.10 * t_axr + 0.15 * t_red + 0.25 * (nw / 8.0)

    def process(self, L, R, t):
        t0 = time.perf_counter()
        cfg, s = self.cfg, self.s
        fr = DotFrame(t=float(t))
        rL, rR = red_map(L), red_map(R)
        m = (rL > cfg.red_thr).astype(np.uint8)
        if cfg.open_k > 1:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._k)
        n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        amin = 0.5 * np.pi * (0.5 * cfg.min_d_px * s) ** 2
        order = sorted((i for i in range(1, n) if st[i, 4] >= amin), key=lambda i: -st[i, 4])[:80]
        pre = []
        for i in order:
            x, y, w, h, a = (int(v) for v in st[i])
            deq = 2.0 * np.sqrt(a / np.pi)
            if deq < cfg.min_d_px * s or deq > cfg.max_d_px * s:
                continue
            mk = (lab[y:y + h, x:x + w] == i).astype(np.uint8)
            wts = rL[y:y + h, x:x + w][mk > 0].astype(np.float64)
            ys, xs = np.nonzero(mk)
            sw = wts.sum()
            cxp, cyp = float((xs * wts).sum() / sw) + x, float((ys * wts).sum() / sw) + y
            pts = np.stack([xs - (xs * wts).sum() / sw, ys - (ys * wts).sum() / sw])
            ev = np.maximum(np.linalg.eigvalsh((pts * wts) @ pts.T / sw), 1e-9)
            axr = float(np.sqrt(ev[0] / ev[1]))
            cn, _ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnt = max(cn, key=cv2.contourArea)
            per = max(float(cv2.arcLength(cnt, True)), 1.0)
            circ = float(4 * np.pi * a / per ** 2)
            (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
            fill = float(a / max((rw + 1.0) * (rh + 1.0), 1.0))     # 最小外接矩形填充率: 圆盘 / 椭圆 ≈ π/4, 与朝向无关, 方块 ≈ 1
            red = float(wts.mean())
            strict = deq >= cfg.shape_px * s
            lo, hi, cm = (cfg.fill_lo_s, cfg.fill_hi_s, cfg.circ_min_s)
            if not (lo <= fill <= hi and circ >= cm and axr >= 0.8 * cfg.axr_min and red >= 0.8 * cfg.red_min):
                continue
            nw, nn = _sectors(L, rL, cxp, cyp, deq / 2.0)
            pre.append(dict(i=i, box=(x, y, w, h), a=a, u=cxp, v=cyp, deq=deq, axr=axr, fill=fill, circ=circ,
                            red=red, nw=nw, nn=nn,
                            ps=self._score(min(max(fill, 0.0), 1.0), circ, axr, red, nw, strict),
                            sat=0.0))
        fr.diag["n_comp"] = len(order)
        pre.sort(key=lambda d: -d["ps"])
        pre = pre[:cfg.max_cands]
        fr.diag["n_keep"] = len(pre)
        t1 = time.perf_counter()
        rL_f = np.clip(rL, 0, 150).astype(np.float32)
        rR_f = np.clip(rR, 0, 150).astype(np.float32)
        dmax = int(cfg.disp_max * s)
        for d in pre:
            Req = d["deq"] / 2.0
            hm = half_max_moments(rL, d["u"], d["v"], Req)
            if hm is None:                       # 精化不了 → 用掩膜矩 (不出法向: 标 partial)
                hm = dict(u=d["u"], v=d["v"], dmaj=d["deq"], dmin=d["deq"] * d["axr"], mx=0.0, my=1.0, partial=True)
            c = Cand(u=hm["u"], v=hm["v"], xn=(hm["u"] - self.cx) / self.f, yn=(hm["v"] - self.cy) / self.f,
                     fb=self.fb, dmaj=hm["dmaj"], dmin=hm["dmin"], mx=hm["mx"], my=hm["my"], area=float(d["a"]),
                     fill=d["fill"], circ=d["circ"], axr=d["axr"], red=d["red"], nw=d["nw"], nn=d["nn"],
                     partial=bool(hm["partial"]), ps=float(d["ps"]))
            x, y, w, h = d["box"]
            mt = _match_row(rL_f, rR_f, x, y, w, h, dmax)
            if mt is not None:
                disp_tm, pk, uniq, dyi = mt
                c.pk, c.uniq = pk, uniq
                hr = half_max_moments(rR, hm["u"] - disp_tm, hm["v"] + dyi, Req)
                if hr is not None:
                    c.disp = hm["u"] - hr["u"]
                    c.dy = hm["v"] - hr["v"]
                else:
                    c.disp = disp_tm
            fr.cands.append(c)
        t2 = time.perf_counter()
        fr.t_ms = {"seg": (t1 - t0) * 1e3, "match": (t2 - t1) * 1e3, "total": (t2 - t0) * 1e3}
        return fr
