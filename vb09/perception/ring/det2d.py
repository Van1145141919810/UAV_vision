# -*- coding: utf-8 -*-
"""
vb09/perception/ring/det2d.py —— 单目 2D: 分割 / 找环 (椭圆 RANSAC) / 正反面 / 角点 / 红点
===============================================================================
2026-10-30 从 gate_detector.py 原样搬来 (与 gate_tune.py v2 相同的算法)。
"""
from __future__ import annotations

import cv2
import numpy as np

from .config import GateConfig


def apply_wb(img, cfg: GateConfig):
    g = np.asarray(cfg.wb, np.float32)
    if np.allclose(g, 1.0):
        return img
    return cv2.convertScaleAbs(img.astype(np.float32) * g[None, None, :])


def segment(img, cfg: GateConfig):
    """
    "彩色像素"掩码。只看 S (饱和度) 会把**暗处的杂色**也算进去 —— 夜里窗外的树 S≈90 但 V≈80,
    2026-10-15 实拍: 它和环粘成一片, 椭圆拟错/被 hole 判据拒掉。彩度 C = S·V/255 = max-min
    是"绝对鲜艳程度", 环 C 中位数 ~110, 窗外树 ~30, 用它分得开。
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    m = (S > cfg.s_min) & (V > cfg.v_min)
    if cfg.c_min > 0:
        b, g, r = cv2.split(img)
        C = cv2.subtract(cv2.max(cv2.max(b, g), r), cv2.min(cv2.min(b, g), r))
        m &= C > cfg.c_min
    sat = m.astype(np.uint8) * 255
    return {"H": H, "S": S, "V": V, "sat": sat, "gray": cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)}


def _scale_el(el, s, about_center=False):
    (cx, cy), (w, h), a = el
    if about_center:
        return ((cx, cy), (w * s, h * s), a)
    return ((cx * s, cy * s), (w * s, h * s), a)


def _el_small_to_full(el, fs):
    """缩小图 (find_scale) 的椭圆 → 工作图。像素中心对齐: x_full = (x_small + 0.5)/fs − 0.5。
    2026-10-21: 以前直接 x/fs, 环心固定往左上偏 (0.5/fs − 0.5) ≈ 0.93px (fs=0.35)。"""
    (cx, cy), (w, h), a = el
    return (((cx + 0.5) / fs - 0.5, (cy + 0.5) / fs - 0.5), (w / fs, h / fs), a)


def _el_full_to_small(el, fs):
    (cx, cy), (w, h), a = el
    return (((cx + 0.5) * fs - 0.5, (cy + 0.5) * fs - 0.5), (w * fs, h * fs), a)


def _shift_el(el, dx, dy=0.0):
    (cx, cy), (w, h), a = el
    return ((cx + dx, cy + dy), (w, h), a)


_TRIG = {}


def _el_points(el, n=90):
    (cx, cy), (w, h), ang = el
    if n not in _TRIG:
        t = np.linspace(0, 2 * np.pi, n, endpoint=False)
        _TRIG[n] = (np.cos(t), np.sin(t))
    ct, st = _TRIG[n]
    c, s = np.cos(np.radians(ang)), np.sin(np.radians(ang))
    return cx + w / 2 * ct * c - h / 2 * st * s, cy + w / 2 * ct * s + h / 2 * st * c


def _estimate_square(small):
    n, _, st, _ = cv2.connectedComponentsWithStats(small, connectivity=4)
    if n < 13:
        return None
    w, h, a = st[1:, 2].astype(float), st[1:, 3].astype(float), st[1:, 4].astype(float)
    sel = (a >= 4) & (w >= 2) & (h >= 2) & (np.maximum(w, h) <= 1.8 * np.minimum(w, h)) & (a >= 0.4 * w * h)
    if sel.sum() < 10:
        return None
    s = np.sort(np.sqrt(a[sel]))
    lo = np.searchsorted(s, s * 0.8, "left")
    hi = np.searchsorted(s, s * 1.25, "right")
    cnt = hi - lo
    i = int(np.argmax(cnt * s))
    return float(np.median(s[lo[i]:hi[i]])) if cnt[i] >= 10 else None


def _inliers(el, pts, w, cfg, shape):
    (cx, cy), (W, H), ang = el
    if not np.all(np.isfinite([cx, cy, W, H, ang])):
        return None
    a, b = W / 2.0, H / 2.0
    sh, sw = shape
    if min(a, b) < 3 or min(a, b) / max(a, b) < 0.25 or max(a, b) > 4 * max(sh, sw):
        return None
    t = np.radians(ang)
    c, s = np.cos(t), np.sin(t)
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    xp, yp = dx * c + dy * s, -dx * s + dy * c
    rn = np.sqrt((xp / a) ** 2 + (yp / b) ** 2)
    rad = np.sqrt(a * b)
    hw = cfg.hw_ratio * rad
    inl = (np.abs(rn - 1) * rad < max(1.5, 0.3 * hw)) & (w > 0.5 * hw) & (w < 1.6 * hw)
    return inl, xp, yp, a, b, hw


def _eval_hyp(el, pts, w, closed, cfg, quick=None):
    q = quick or _inliers(el, pts, w, cfg, closed.shape)
    if q is None:
        return None
    inl, xp, yp, a, b, hw = q
    n = int(inl.sum())
    if n < 10:
        return None
    sh, sw = closed.shape
    (cx, cy), (W, H), ang = el
    th = np.arctan2(yp[inl] / b, xp[inl] / a)
    cov = float((np.bincount(np.clip(((th + np.pi) / (2 * np.pi) * 36).astype(int), 0, 35), minlength=36) > 0).mean())
    ex, ey = _el_points(el, 120)
    inimg = (ex >= 0) & (ex < sw) & (ey >= 0) & (ey < sh)
    if inimg.sum() < 12:
        return None
    onb = float((closed[ey[inimg].astype(int), ex[inimg].astype(int)] > 0).mean())
    w_ratio = float(np.median(w[inl]) / hw)
    hv = []
    for k in (0.55, 0.7):
        hx, hy = _el_points(((cx, cy), (W * k, H * k), ang), 90)
        ok = (hx >= 0) & (hx < sw) & (hy >= 0) & (hy < sh)
        if ok.sum() >= 8:
            hv.append(closed[hy[ok].astype(int), hx[ok].astype(int)] > 0)
    hole = float(np.concatenate(hv).mean()) if hv else 0.0
    score = n * onb * np.exp(-((w_ratio - 1) / 0.3) ** 2) * np.sqrt(cov) * max(0.0, 1 - 2 * hole)
    return {"el": el, "n": n, "cov": cov, "onb": onb, "inl": inl, "w_ratio": w_ratio, "hole": hole,
            "score": float(score)}


def _refine(h, pts, w, closed, cfg, iters=3):
    for _ in range(iters):
        if h is None or h["n"] < 6:
            return h
        try:
            el = cv2.fitEllipse(pts[h["inl"]].astype(np.float32))
        except cv2.error:
            return h
        h2 = _eval_hyp(el, pts, w, closed, cfg)
        if h2 is None or h2["score"] < h["score"] * 0.98:
            return h
        h = h2
    return h


def _accept(h, cfg):
    return h is not None and h["onb"] >= cfg.min_onband and h["cov"] >= cfg.min_coverage and h["hole"] <= cfg.max_hole


def find_gate(seg, cfg: GateConfig, prior=None, rng=None):
    """色带中心线(距离变换脊线) + RANSAC 椭圆。返回 dict(ok, center/outer/inner 椭圆, band_roi, ...)。"""
    rng = rng or np.random.default_rng(0)
    fs = cfg.find_scale
    small = cv2.resize(seg["sat"], None, fx=fs, fy=fs, interpolation=cv2.INTER_AREA)
    small = (small > 64).astype(np.uint8) * 255
    sq = _estimate_square(small)
    if prior is not None:
        sq = max(sq or 0.0, max(prior[1]) * fs * ((cfg.outer_d / 2) / cfg.r_center) * cfg.square / cfg.outer_d)
    base = int(cfg.close_k) | 1
    ks = [min(61, max(base, int(round(1.6 * sq))) | 1)] if sq else sorted({base, 9, 15, 23})
    best = None
    rej = None                                   # 诊断: 所有核里得分最高的"被拒"假设
    for k in ks:
        g = _find_gate_k(seg, small, k, cfg, prior, rng)
        if g.get("rej") and (rej is None or g["rej"]["score"] > rej["score"]):
            rej = dict(g["rej"], k=k)
        if best is None or (g["ok"] and (not best["ok"] or g["score"] > best["score"])):
            best = g
        if g["ok"] and g["coverage"] >= 0.6 and 0.8 <= g["width_ratio"] <= 1.25 and g["hole"] <= 0.1:
            break
    best["tried_k"] = list(ks)
    best["sq_est"] = float(sq) if sq else None
    if not best["ok"] and rej is not None:
        best["rej"] = rej
    return best


def _find_gate_k(seg, small, k, cfg, prior, rng):
    Hh, Ww = seg["sat"].shape
    fs = cfg.find_scale
    closed = cv2.morphologyEx(small, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    out = {"ok": False, "why": "", "prior_used": False, "close_k": k, "score": 0.0}
    dt = cv2.distanceTransform(closed, cv2.DIST_L2, 3)
    ridge = (dt >= cv2.dilate(dt, np.ones((3, 3), np.uint8))) & (dt >= 1.0)
    ys, xs = np.nonzero(ridge)
    if len(xs) < 15:
        out["why"] = "no_ridge"
        return out
    if len(xs) > 1500:
        sel = rng.choice(len(xs), 1500, replace=False)
        xs, ys = xs[sel], ys[sel]
    pts = np.stack([xs, ys], 1).astype(np.float64)
    w = dt[ys, xs].astype(np.float64)
    # 先验够好就跳过 RANSAC（跟踪时大部分帧走这里，省一半以上时间）
    if prior is not None:
        ph = _refine(_eval_hyp(_el_full_to_small(prior, fs), pts, w, closed, cfg), pts, w, closed, cfg)
        if _accept(ph, cfg) and ph["cov"] >= 0.5 and 0.8 <= ph["w_ratio"] <= 1.25 and ph["hole"] <= 0.1:
            return _finish_gate(out, ph, pts, cfg, fs, Hh, Ww, prior_used=True)
    _, lab = cv2.connectedComponents(closed, connectivity=8)
    lab_pts = lab[ys, xs]
    comps, counts = np.unique(lab_pts, return_counts=True)
    probs = counts / counts.sum()
    by_comp = {c_: np.flatnonzero(lab_pts == c_) for c_ in comps}
    quick = []
    for it in range(cfg.ransac_iters):
        if it % 2 == 0:
            pool = by_comp[rng.choice(comps, p=probs)]
            if len(pool) < 5:
                continue
            idx = rng.choice(pool, 5, replace=False)
        else:
            idx = rng.choice(len(pts), 5, replace=False)
        try:
            el = cv2.fitEllipse(pts[idx].astype(np.float32))
        except cv2.error:
            continue
        q = _inliers(el, pts, w, cfg, closed.shape)
        if q is not None:
            nq = int(q[0].sum())
            if nq >= 10:
                quick.append((nq, el, q))
    quick.sort(key=lambda t_: -t_[0])
    hyps = [h for h in (_eval_hyp(el, pts, w, closed, cfg, quick=q) for _, el, q in quick[:30]) if h is not None]
    hyps.sort(key=lambda h: -h["score"])
    best = None
    for h in hyps[:8]:
        h = _refine(h, pts, w, closed, cfg)
        if _accept(h, cfg) and (best is None or h["score"] > best["score"]):
            best = h
    if prior is not None:
        ph = _refine(_eval_hyp(_el_full_to_small(prior, fs), pts, w, closed, cfg), pts, w, closed, cfg)
        if _accept(ph, cfg) and (best is None or ph["score"] >= 0.6 * best["score"]):
            best, out["prior_used"] = ph, True
    if best is None:
        out["why"] = "no_consensus" if hyps else "no_hypothesis"
        if hyps:
            h0 = _refine(hyps[0], pts, w, closed, cfg) or hyps[0]
            out["rej"] = {"score": float(h0["score"]), "onb": float(h0["onb"]), "cov": float(h0["cov"]),
                          "hole": float(h0["hole"]), "w_ratio": float(h0["w_ratio"]), "n": int(h0["n"]),
                          "major_px": float(max(h0["el"][1]) / fs)}
        return out
    return _finish_gate(out, best, pts, cfg, fs, Hh, Ww, out["prior_used"])


def _finish_gate(out, best, pts, cfg, fs, Hh, Ww, prior_used):
    out["prior_used"] = prior_used
    center = _el_small_to_full(best["el"], fs)
    if max(center[1]) < cfg.min_center_px:
        out["why"] = "too_small"
        return out
    ko, ki = (cfg.outer_d / 2) / cfg.r_center, (cfg.inner_d / 2) / cfg.r_center
    outer, inner = _scale_el(center, ko, True), _scale_el(center, ki, True)
    roi = np.zeros((Hh, Ww), np.uint8)
    cv2.ellipse(roi, _scale_el(outer, 1.05, True), 255, -1)
    cv2.ellipse(roi, _scale_el(inner, 0.95, True), 0, -1)
    ex, ey = _el_points(outer, 180)
    out.update(ok=True, why="ok", center=center, outer=outer, inner=inner, band_roi=roi,
               outer_major=float(max(outer[1])), center_major=float(max(center[1])),
               coverage=best["cov"], onband=best["onb"], width_ratio=best["w_ratio"], hole=best["hole"],
               score=best["score"], partial=bool(((ex < 0) | (ex >= Ww) | (ey < 0) | (ey >= Hh)).any()),
               centerline_px=(pts[best["inl"]] + 0.5) / fs - 0.5)  # 中心线内点（工作图像素）
    return out


def hue_side(seg, gate):
    H, sat = seg["H"], seg["sat"]
    (cx, cy), (w, h), _ = gate["center"]
    ys, xs = np.nonzero((gate["band_roi"] > 0) & (sat > 0))
    th = np.degrees(np.arctan2((ys - cy) / (h / 2), (xs - cx) / (w / 2)))
    L, R = np.abs(np.abs(th) - 180) <= 45, np.abs(th) <= 45
    hv = H[ys, xs]
    red, blue = (hv < 10) | (hv > 165), (hv > 95) & (hv < 130)
    f = lambda sel, c: float(c[sel].mean()) if sel.sum() >= 150 else float("nan")
    lr, lb, rr_, rb = f(L, red), f(L, blue), f(R, red), f(R, blue)

    def lab(r_, b_):
        if r_ == r_ and r_ > 0.6 and r_ > 2 * b_:
            return "red"
        if b_ == b_ and b_ > 0.6 and b_ > 2 * r_:
            return "blue"
        return None

    a, b = lab(lr, lb), lab(rr_, rb)
    if a == "red" and b == "blue":
        return "front", 1.0
    if a == "blue" and b == "red":
        return "back", 1.0
    if (a == "red" and b is None) or (b == "blue" and a is None):
        return "front", 0.5
    if (a == "blue" and b is None) or (b == "red" and a is None):
        return "back", 0.5
    return None, 0.0


def find_corners(seg, gate, cfg):
    sq = gate["outer_major"] * cfg.square / cfg.outer_d
    r = {"sq_px": float(sq), "pts": np.zeros((0, 2)), "xok": np.zeros(0, bool), "pol": np.zeros(0, np.int8)}
    if sq < cfg.min_square_px:
        return r
    # ★2026-10-15: 先裁到色带外接框。goodFeaturesToTrack 的特征值图是在**整幅图**上算的,
    #   mask 只在最后筛点 —— 不裁的话环再小也是全图开销。
    bx, by, bw, bh = cv2.boundingRect(gate["band_roi"])
    if bw < 8 or bh < 8:
        return r
    gray = seg["gray"][by:by + bh, bx:bx + bw]
    g = cv2.GaussianBlur(gray, (0, 0), cfg.gray_blur) if cfg.gray_blur > 0 else gray
    pts = cv2.goodFeaturesToTrack(g, maxCorners=int(cfg.max_corners), qualityLevel=max(cfg.corner_quality, 1e-3),
                                  minDistance=max(3.0, cfg.mind_factor * sq), mask=gate["band_roi"][by:by + bh, bx:bx + bw],
                                  blockSize=max(3, int(cfg.corner_block) | 1))
    if pts is None:
        return r
    win = max(2, int(round(0.35 * sq)))
    pts = cv2.cornerSubPix(g, pts.astype(np.float32), (win, win), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)).reshape(-1, 2)
    pts = pts + np.array([bx, by], np.float32)          # 回到工作图坐标
    sat = seg["sat"]
    Hh, Ww = sat.shape
    rad = max(3.0, cfg.x_r_factor * sq)
    t = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    xs = np.round(pts[:, 0:1] + rad * np.cos(t)[None]).astype(int)
    ys = np.round(pts[:, 1:2] + rad * np.sin(t)[None]).astype(int)
    inb = (xs >= 0).all(1) & (xs < Ww).all(1) & (ys >= 0).all(1) & (ys < Hh).all(1)
    s = sat[np.clip(ys, 0, Hh - 1), np.clip(xs, 0, Ww - 1)] > 0
    trans = (s != np.roll(s, 1, axis=1)).sum(1)
    q = s[:, [6, 18, 30, 42]]
    xok = inb & (trans == 4) & (q[:, 0] == q[:, 2]) & (q[:, 1] == q[:, 3]) & (q[:, 0] != q[:, 1])
    r.update(pts=pts, xok=xok, pol=q[:, 0].astype(np.int8))
    return r


def find_dot(img, cfg: GateConfig, pred_diam=None, roi=None):
    """白底包围的红色圆斑。roi=(x0,y0,x1,y1) 时只在 ROI 内找（末段省算力）。"""
    Hh, Ww = img.shape[:2]
    x0, y0, x1, y1 = (0, 0, Ww, Hh) if roi is None else (max(0, int(roi[0])), max(0, int(roi[1])),
                                                        min(Ww, int(roi[2])), min(Hh, int(roi[3])))
    out = {"cands": [], "blobs": [], "best": None, "pred_diam": pred_diam, "roi": (x0, y0, x1, y1)}
    if x1 - x0 < 8 or y1 - y0 < 8:
        return out
    crop = img[y0:y1, x0:x1]
    b, g, r = cv2.split(crop)
    m = (cv2.subtract(r, cv2.max(g, b)) > cfg.red_thr).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    S, V = hsv[..., 1], hsv[..., 2]
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    ch, cw = m.shape
    min_area = np.pi * (cfg.dot_min_px / 2) ** 2
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < min_area or w < 3 or h < 3:
            continue
        fill = a / max(np.pi * w * h / 4, 1)
        if fill < 0.5:
            continue
        cs = cv2.findContours((lab[y:y + h, x:x + w] == i).astype(np.uint8), cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_NONE)[-2]
        per = cv2.arcLength(max(cs, key=len), True)
        circ = 4 * np.pi * a / max(per * per, 1)
        diam = 2 * np.sqrt(a / np.pi)
        cx, cy = cen[i]
        bl = {"c": (float(cx + x0), float(cy + y0)), "diam": float(diam), "circ": float(circ), "fill": float(fill),
              "ratio": float(diam / pred_diam) if pred_diam else float("nan"),
              "surr_S": float("nan"), "surr_V": float("nan"), "shape_ok": False, "ok": False,
              "touch": bool(x == 0 or y == 0 or x + w >= cw or y + h >= ch)}
        bl["shape_ok"] = circ >= cfg.dot_circ_min and cfg.dot_fill_min <= fill <= 1.15 and not bl["touch"]
        if bl["shape_ok"]:
            rad = diam / 2
            ya, yb = int(max(0, cy - 2 * rad)), int(min(ch, cy + 2 * rad + 1))
            xa, xb = int(max(0, cx - 2 * rad)), int(min(cw, cx + 2 * rad + 1))
            yy, xx = np.ogrid[ya:yb, xa:xb]
            dd = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
            ann = (dd > 1.3 * rad) & (dd < 1.8 * rad)
            if ann.sum() > 8:
                bl["surr_S"] = float(S[ya:yb, xa:xb][ann].mean())
                bl["surr_V"] = float(V[ya:yb, xa:xb][ann].mean())
            bl["ok"] = bl["surr_S"] < cfg.s_min and bl["surr_V"] > cfg.surround_v_min
        out["blobs"].append(bl)
    out["cands"] = [bl for bl in out["blobs"] if bl["ok"]]
    if pred_diam:
        sized = [bl for bl in out["cands"] if abs(bl["ratio"] - 1) <= cfg.dot_size_tol]
        out["best"] = min(sized, key=lambda bl: abs(bl["ratio"] - 1)) if sized else None
    elif out["cands"]:
        out["best"] = max(out["cands"], key=lambda bl: bl["circ"] * bl["diam"])
    return out
