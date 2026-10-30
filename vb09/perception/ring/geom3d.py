# -*- coding: utf-8 -*-
"""
vb09/perception/ring/geom3d.py —— 双目 / 3D 几何: 角点配对、平面拟合、环心定位、单目椭圆 → 圆位姿、法线定向
===============================================================================
2026-10-30 从 gate_detector.py (+ ring_detector.py 的 ellipse_to_conic / circle_pose_from_ellipse) 原样搬来。
只依赖 numpy (不 import cv2) —— 飞机侧融合 (fusion.py) 也用这里的 _angle。
_basis 两个文件里各有一份 (数值完全相同), 现在只留这一份。
坐标: rect = 校正后左目系 (x 右, y 下, z 前)。
"""
from __future__ import annotations

import numpy as np

from .config import GateConfig


def _ray(rig, u, v):
    return np.array([(u - rig.cx) / rig.f, (v - rig.cy) / rig.f, 1.0])


def match_corners(cL, cR, d_guess, rig, cfg):
    """同极性 X 角点按行配对 → 3D 点（rect 系）。"""
    PL, PR = cL["pts"][cL["xok"]], cR["pts"][cR["xok"]]
    if len(PL) == 0 or len(PR) == 0:
        return np.zeros((0, 3)), np.zeros((0, 2))
    polL, polR = cL["pol"][cL["xok"]], cR["pol"][cR["xok"]]
    tol = 0.45 * 2 * cL["sq_px"]
    used = {}
    for i, (x, y) in enumerate(PL):
        sel = np.flatnonzero((np.abs(PR[:, 1] - y) <= cfg.y_tol) & (polR == polL[i]))
        if not len(sel):
            continue
        dx = np.abs(PR[sel, 0] - (x - d_guess))
        j, e = sel[np.argmin(dx)], dx.min()
        if e <= tol and (j not in used or e < used[j][1]):
            used[j] = (i, e)
    if not used:
        return np.zeros((0, 3)), np.zeros((0, 2))
    ii = np.array([i for i, _ in used.values()])
    jj = np.array(list(used.keys()))
    xl, y, xr = PL[ii, 0], PL[ii, 1], PR[jj, 0]
    d = xl - xr
    g = d > 0
    Z = rig.f * rig.B / d[g]
    P = np.stack([(xl[g] - rig.cx) * Z / rig.f, (y[g] - rig.cy) * Z / rig.f, Z], 1)
    return P, PL[ii][g]


def fit_plane(P, iters=100, thr=0.02, rng=None):
    rng = rng or np.random.default_rng(0)
    n = len(P)
    if n < 6:
        return None
    best = None
    idx = rng.integers(0, n, size=(iters, 3))
    for a, b, c in idx:
        nv = np.cross(P[b] - P[a], P[c] - P[a])
        L = np.linalg.norm(nv)
        if L < 1e-9:
            continue
        nv /= L
        inl = np.abs((P - P[a]) @ nv) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 6:
        return None
    Q = P[best]
    mu = Q.mean(0)
    _, _, Vt = np.linalg.svd(Q - mu)
    nv = Vt[2]
    if nv @ mu > 0:
        nv = -nv  # 法线指向相机
    rms = float(np.sqrt(np.mean(((Q - mu) @ nv) ** 2)))
    return {"mu": mu, "n": nv, "rms": rms, "n_pts": int(best.sum())}


def _basis(n):
    a = np.array([0.0, 1.0, 0.0]) if abs(n[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, n)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(n, e1)


def ring_center_in_plane(px, plane, rig, radius):
    """中心线像素点 → 射线与角点平面求交 → 平面内拟合固定半径圆 → 环心。"""
    mu, n = plane["mu"], plane["n"]
    rays = np.stack([(px[:, 0] - rig.cx) / rig.f, (px[:, 1] - rig.cy) / rig.f, np.ones(len(px))], 1)
    den = rays @ n
    ok = np.abs(den) > 1e-6
    s = (mu @ n) / den[ok]
    P = rays[ok] * s[:, None]
    P = P[s > 0]
    if len(P) < 10:
        return None
    e1, e2 = _basis(n)
    uv = np.stack([(P - mu) @ e1, (P - mu) @ e2], 1)

    def fit(uv):
        A = np.stack([uv[:, 0], uv[:, 1], np.ones(len(uv))], 1)
        sol, *_ = np.linalg.lstsq(A, (uv ** 2).sum(1), rcond=None)
        c = sol[:2] / 2
        r_free = float(np.sqrt(max(sol[2] + c @ c, 1e-9)))
        for _ in range(10):  # 固定半径 GN
            d = uv - c
            rho = np.linalg.norm(d, axis=1) + 1e-12
            J = -d / rho[:, None]
            step, *_ = np.linalg.lstsq(J, -(rho - radius), rcond=None)
            c = c + step
            if np.linalg.norm(step) < 1e-6:
                break
        res = np.linalg.norm(uv - c, axis=1) - radius
        return c, r_free, res

    c, r_free, res = fit(uv)
    mad = np.median(np.abs(res)) * 1.4826 + 1e-4
    keep = np.abs(res) < max(3 * mad, 0.01)
    if keep.sum() >= 10 and keep.sum() < len(uv):
        c, r_free, res = fit(uv[keep])
    return {"c": mu + c[0] * e1 + c[1] * e2, "n": n, "r_free": r_free,
            "rms": float(np.sqrt(np.mean(res ** 2))), "n_pts": int(len(res))}


def _angle(a, b):
    """两条法线的夹角 (度), 不分正反"""
    return float(np.degrees(np.arccos(np.clip(abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b)), 0, 1))))


def ring_tilt_deg(c, n):
    """环面相对视线的倾角 (度): 0 = 正对"""
    return _angle(np.asarray(c, float), np.asarray(n, float))


def _row_roots(C, y):
    A = C[0, 0]
    Bq = 2 * (C[0, 1] * y + C[0, 2])
    C0 = C[1, 1] * y * y + 2 * C[1, 2] * y + C[2, 2]
    D = Bq * Bq - 4 * A * C0
    s = np.sqrt(np.where(D > 0, D, 0))
    r1, r2 = (-Bq - s) / (2 * A), (-Bq + s) / (2 * A)
    return np.minimum(r1, r2), np.maximum(r1, r2), D > 0


def ellipse_disparity_normal(elL, elR, rig, frac=0.8, n=60):
    """
    左右目中心线椭圆 → 环平面法线, 不用角点。
    平面在校正图上的视差是 (u, v) 的仿射函数 d = a + b·u + c·v (近的一侧视差大),
    逐行取左右两段弧的视差, 最小二乘拟合 (a, b, c) → 法线 ∝ (b, c, (a + b·cx + c·cy)/f)。
    """
    CL, CR = ellipse_to_conic(elL), ellipse_to_conic(elR)
    yc, hh = 0.5 * (elL[0][1] + elR[0][1]), 0.5 * max(elL[1])
    ys = np.linspace(yc - frac * hh, yc + frac * hh, n)      # 避开上下顶点 (交点对行太敏感)
    l1, l2, ok1 = _row_roots(CL, ys)
    r1, r2, ok2 = _row_roots(CR, ys)
    ok = ok1 & ok2
    if ok.sum() < 8:
        return None
    u = np.r_[l1[ok], l2[ok]]
    v = np.r_[ys[ok], ys[ok]]
    d = np.r_[l1[ok] - r1[ok], l2[ok] - r2[ok]]
    (a, b, c), *_ = np.linalg.lstsq(np.c_[np.ones_like(u), u, v], d, rcond=None)
    nn = np.array([b, c, (a + b * rig.cx + c * rig.cy) / rig.f])
    nrm = float(np.linalg.norm(nn))
    return nn / nrm if nrm > 0 and np.isfinite(nrm) else None


def pick_ring_normal(sols, plane_n, ref_n, cfg: GateConfig, disp_n=None):
    """
    单目椭圆的两个 (c, n) 解里挑。返回 (下标 j, 法线 n, n_src, 区分度°)。

    单目椭圆: 倾斜 ≥ 15° 时对的那个解远处也只差 1~3°, 但两个解几何上等价, 光靠椭圆分不出;
             接近正对时扁率被噪声主导, 两解各偏 5~12°, 平分线 (≈视线方向) 反而准。
      1) 有角点平面 (远处残差大, 法线仍有几度精度) → 选离它近的, 要拉开 normal_sep_deg     mono_plane
         且离选中的解 ≤ normal_ref_max_deg (角点太少的坏平面会和两个解都差很远)
      2) 左右目椭圆视差算的法线 (≤ eldisp_max_dist)  → 同上                               mono_disp
      3) 有估计器里的环法线 (机上一体时)            → 同上                               mono_est
      4) 两解夹角 ≤ normal_mid_deg  → 近正对/扁率是噪声: 取平分线, 误差 ≲ 夹角/2, 只显示     mono_mid
      5) 都不行 → 老办法 "最正对" 那个, 标二义 (另一个解放 n_alt, 飞机侧有环法线时再挑)   mono_amb
    """
    if len(sols) == 1:
        return 0, sols[0][1], "mono_mid", 0.0
    n0, n1 = sols[0][1], sols[1][1]
    for ref, tag in ((plane_n, "mono_plane"), (disp_n, "mono_disp"), (ref_n, "mono_est")):
        if ref is None:
            continue
        a = [_angle(n0, ref), _angle(n1, ref)]
        k = int(np.argmin(a))
        if a[1 - k] - a[k] >= cfg.normal_sep_deg and a[k] <= cfg.normal_ref_max_deg:
            return k, sols[k][1], tag, a[1 - k] - a[k]
    sep = _angle(n0, n1)
    j = int(np.argmax([-(n_ @ (c_ / np.linalg.norm(c_))) for c_, n_ in sols]))
    if sep <= cfg.normal_mid_deg:
        m = n0 + (n1 if n0 @ n1 > 0 else -n1)
        return j, m / np.linalg.norm(m), "mono_mid", sep
    return j, sols[j][1], "mono_amb", sep


# ----------------------------------------------------------------------------
# 单目: 椭圆 → 圆位姿 (原 ring_detector.py)
# ----------------------------------------------------------------------------
def ellipse_to_conic(el):
    """cv2.RotatedRect 椭圆 → 3x3 对称二次曲线矩阵 C，[x y 1] C [x y 1]^T = 0。"""
    (cx, cy), (w, h), ang = el
    a, b = w / 2.0, h / 2.0
    t = np.deg2rad(ang)
    ct, st = np.cos(t), np.sin(t)
    A = ct * ct / (a * a) + st * st / (b * b)
    Bc = 2 * ct * st * (1 / (a * a) - 1 / (b * b))
    Cc = st * st / (a * a) + ct * ct / (b * b)
    D = -2 * A * cx - Bc * cy
    E = -Bc * cx - 2 * Cc * cy
    F = A * cx * cx + Bc * cx * cy + Cc * cy * cy - 1
    return np.array([[A, Bc / 2, D / 2], [Bc / 2, Cc, E / 2], [D / 2, E / 2, F]])


def circle_pose_from_ellipse(ellipse, K, radius):
    """已知半径的圆，由其投影椭圆求位姿。返回最多两个 (c, n)，n 指向相机。"""
    C = ellipse_to_conic(ellipse)
    Q = K.T @ C @ K
    Q = 0.5 * (Q + Q.T)
    w, V = np.linalg.eigh(Q)
    if (w > 0).sum() == 1:
        w = -w
    pos, neg = np.flatnonzero(w > 0), np.flatnonzero(w < 0)
    if len(pos) != 2 or len(neg) != 1:
        return []
    i1, i2 = pos[np.argsort(-w[pos])]
    i3 = neg[0]
    l1, l2, l3 = w[i1], w[i2], w[i3]
    V = V[:, [i1, i2, i3]]
    a = np.sqrt(max((l1 - l2) / (l1 - l3), 0.0))
    b = np.sqrt(max((l2 - l3) / (l1 - l3), 0.0))
    sols = []
    for sgn in ((1.0, -1.0) if a > 1e-9 else (1.0,)):
        n = V @ np.array([sgn * a, 0.0, b])
        n /= np.linalg.norm(n)
        # 平面 n·X = 1 截圆锥 → 平面内圆 → 按半径缩放
        e1, e2 = _basis(n)
        E = np.stack([e1, e2], 1)
        p0 = n.copy()
        M = E.T @ Q @ E
        g = E.T @ Q @ p0
        c0 = p0 @ Q @ p0
        try:
            w0 = -np.linalg.solve(M, g)
        except np.linalg.LinAlgError:
            continue
        Fp = c0 + g @ w0
        m = 0.5 * np.trace(M)
        r2 = -Fp / m
        if r2 <= 0:
            continue
        s = radius / np.sqrt(r2)
        c = s * (p0 + E @ w0)
        if c[2] < 0:
            c = -c
        if n @ c > 0:
            n = -n
        sols.append((c, n))
    return sols
