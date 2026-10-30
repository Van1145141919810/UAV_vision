"""
ring_detector.py — 红色圆环检测与位姿估计（双目弧线三角化 + 单目椭圆交叉校验）

纯函数 / 无 ROS 依赖，Windows 离线调试与机载共用。Python 3.8+，OpenCV 4.x，numpy。

流水线（每帧）:
    并排帧 3840x1080 → 拆左右目 → 缩放(默认 0.5) → 双目校正 remap
    → 红色得分 R-max(G,B) → 阈值掩码 → 连通域 + 椭圆拟合 + 形状门限（左右目各一次）
    → 只取椭圆竖直跨度中间段的行，逐行取 左弧/右弧 的内外边缘(亚像素) → 中点 = 管中心线
    → 同一行左右目按顺序配对 → 视差 → 3D 点（校正后左目系）
    → RANSAC 平面 + 圆拟合(先自由半径→尺度检查, 再固定已知半径联合精化)
    → 单目椭圆两解中取与双目法线一致者 → 交叉校验
    → 输出 校正左目系 与 原始左目系 下的 环心 c / 法线 n（n 指向相机）

坐标约定:
    相机系 OpenCV: x 右, y 下, z 前。
    环系: 原点=环心, x 右, y 上, z = 法线(指向观察者/相机)。与 OpenCV ArUco 标记坐标系一致。
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ----------------------------------------------------------------------------
# 配置
# ----------------------------------------------------------------------------
@dataclass
class RingConfig:
    # 环几何（卷尺实测）
    radius: float = 0.50        # 管中心线半径 (m) = (外径 + 内径) / 4
    tube: float = 0.03          # 红色管/色带宽度 (m)

    # 红色分割（score = R - max(G, B)，uint8 饱和减法）
    score_thr: int = 40
    r_min: int = 50             # R 通道下限，滤掉暗处噪声

    # 候选门限
    min_ellipse_area: float = 300.0     # 椭圆面积下限 (px^2)
    min_axis_ratio: float = 0.25        # 短轴/长轴 下限（≈ 斜视 75° 以内）
    max_fill: float = 0.6               # 红色像素 / 椭圆面积 上限（环是空心的）
    max_fit_resid: float = 0.06         # 轮廓到椭圆的相对残差中位数上限（完整环）
    max_fit_resid_partial: float = 0.15 # 触边（部分出画）时放宽

    # 逐行扫描
    row_band: float = 0.7       # 只用椭圆竖直半跨度的 ±70% 行（上下段与极线平行，无视差）
    max_slope: float = 1.2      # |dx/dy| 上限，越小越只用陡峭段
    run_gap: int = 3            # 合并间隔 <= run_gap 的红色段
    min_run: int = 2

    # 深度范围 / 视差匹配
    z_min: float = 0.4
    z_max: float = 10.0
    disp_tol_rel: float = 0.5   # |d - d_guess| < disp_tol_rel * d_guess + disp_tol_abs
    disp_tol_abs: float = 4.0

    # 拟合
    min_points: int = 30
    max_fit_points: int = 600   # 拟合前抽稀到这个点数以内
    ransac_iters: int = 200
    ransac_thr: float = 0.02    # 平面 RANSAC 内点阈值下限 (m)，会按 3σ_Z 自动放大
    fix_radius: bool = True     # 最终结果固定半径=radius 精化
    depth_scale: float = 1.0    # 双目尺度修正（离线由 r_fit/radius 的中位数标定后填回）
    max_radius_dev: float = 0.15  # |r_fit/radius - 1| 上限，超出说明配错了东西

    # 双目 / 单目分工
    stereo_max_z: float = 4.0   # 超过这个距离只信单目（双目深度噪声 ∝ Z^2）
    require_mono_agree: bool = True
    max_mono_angle_deg: float = 15.0
    max_mono_dist_rel: float = 0.15

    seed: int = 0


# ----------------------------------------------------------------------------
# 标定文件读取 + 校正
# ----------------------------------------------------------------------------
_KEYS = {
    "K1": ["K1", "M1", "cameraMatrix1", "K_left", "KL", "mtx_l", "mtxL", "left_K", "camera_matrix_left"],
    "D1": ["D1", "dist1", "distCoeffs1", "D_left", "DL", "dist_l", "distL", "left_D", "dist_coeffs_left"],
    "K2": ["K2", "M2", "cameraMatrix2", "K_right", "KR", "mtx_r", "mtxR", "right_K", "camera_matrix_right"],
    "D2": ["D2", "dist2", "distCoeffs2", "D_right", "DR", "dist_r", "distR", "right_D", "dist_coeffs_right"],
    "R": ["R", "R_lr", "rotation"],
    "T": ["T", "T_lr", "translation"],
    "size": ["image_size", "imageSize", "size", "img_size", "imgSize"],
}


def _to_array(v):
    if v is None:
        return None
    if isinstance(v, dict):  # opencv-matrix 风格 {rows, cols, data}
        if "data" in v:
            a = np.asarray(v["data"], float)
            if "rows" in v and "cols" in v:
                a = a.reshape(int(v["rows"]), int(v["cols"]))
            return a
        return None
    return np.asarray(v, float)


def load_stereo_calib(path: str):
    """读取双目标定。支持 cv2.FileStorage 写的 yaml/xml/json，以及 PyYAML / json 写的普通字典。
    返回 dict: K1 D1 K2 D2 R T size(w,h, 单目尺寸)。"""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"找不到标定文件: {os.path.abspath(path)}（--calib 要写完整路径或相对当前目录的路径）")
    found = {}
    fs_keys = []
    # 先用 Python 读文本再交给 FileStorage(内存模式)：Windows 下 cv2 不能直接打开含中文的路径
    with open(path, "rb") as f:
        raw = f.read()
    try:
        txt_mem = raw.decode("utf-8")
    except UnicodeDecodeError:
        txt_mem = raw.decode("gbk", errors="replace")
    try:
        fs = cv2.FileStorage(txt_mem, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
        if fs.isOpened():
            root = fs.root()
            fs_keys = list(root.keys()) if hasattr(root, "keys") else []
            for k, names in _KEYS.items():
                for n in names:
                    node = fs.getNode(n)
                    if node.empty():
                        continue
                    m = node.mat()
                    if m is None:
                        if node.isSeq():
                            m = np.array([node.at(i).real() for i in range(node.size())])
                        elif node.isReal() or node.isInt():
                            m = np.array([node.real()])
                    if m is not None:
                        found[k] = np.asarray(m, float)
                        break
            fs.release()
    except Exception:  # 解析失败(cv2.error / SystemError) → 走 PyYAML/json 兜底
        pass

    if any(k not in found for k in ("K1", "D1", "K2", "D2", "R", "T")):
        data = None
        try:
            if path.lower().endswith(".json"):
                import json
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            else:
                import yaml  # PyYAML
                with open(path, "r", encoding="utf-8") as f:
                    txt = f.read()
                if txt.startswith("%YAML"):
                    txt = txt.split("\n", 1)[1]
                txt = txt.replace("!!opencv-matrix", "")
                data = yaml.safe_load(txt)
        except Exception:
            data = None
        if isinstance(data, dict):
            low = {str(k).lower(): v for k, v in data.items()}
            for k, names in _KEYS.items():
                if k in found:
                    continue
                for n in names:
                    if n.lower() in low:
                        a = _to_array(low[n.lower()])
                        if a is not None:
                            found[k] = a
                            break
            fs_keys = fs_keys or list(data.keys())

    miss = [k for k in ("K1", "D1", "K2", "D2", "R", "T") if k not in found]
    if miss:
        raise ValueError(f"标定文件 {path} 缺少 {miss}；文件里的键: {fs_keys}。"
                         f"在 ring_detector._KEYS 里补上对应别名即可。")
    out = {
        "K1": found["K1"].reshape(3, 3), "D1": found["D1"].ravel(),
        "K2": found["K2"].reshape(3, 3), "D2": found["D2"].ravel(),
        "R": found["R"], "T": found["T"].reshape(3, 1),
    }
    if out["R"].size == 3:  # Rodrigues 向量
        out["R"], _ = cv2.Rodrigues(out["R"].reshape(3, 1))
    out["R"] = out["R"].reshape(3, 3)
    # ★2026-10-15: 平移单位统一成米。棋盘格按 mm 给尺寸 (square_mm) 时 T 是 mm
    #   (本相机 T≈-60.0 → 基线 60mm)。旧版直接当米用 → B=60m → 所有双目深度大 1000 倍,
    #   角点平面 / 红点双目永远不成立, 只剩单目路径在工作。基线不可能 >1m, 据此判单位。
    out["T_unit"] = "m"
    if float(np.linalg.norm(out["T"])) > 1.0:
        out["T"] = out["T"] / 1000.0
        out["T_unit"] = "mm→m"
    if "size" in found and found["size"].size >= 2:
        w, h = found["size"].ravel()[:2]
        out["size"] = (int(w), int(h))
    else:
        out["size"] = None
    return out


def _scale_K(K, s):
    S = np.array([[s, 0, 0.5 * s - 0.5], [0, s, 0.5 * s - 0.5], [0, 0, 1.0]])
    return S @ K


class StereoRig:
    """按 scale 缩放后的双目校正。所有检测都在"校正后左目系"里做。"""

    def __init__(self, K1, D1, K2, D2, R, T, size_full=(1920, 1080), scale=0.5, alpha=0.0):
        w, h = size_full
        self.scale = float(scale)
        self.size_full = (int(w), int(h))
        self.size = (int(round(w * scale)), int(round(h * scale)))
        K1s, K2s = _scale_K(np.asarray(K1, float), scale), _scale_K(np.asarray(K2, float), scale)
        D1 = np.asarray(D1, float).ravel()
        D2 = np.asarray(D2, float).ravel()
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1s, D1, K2s, D2, self.size, np.asarray(R, float), np.asarray(T, float).reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha)
        self.mapL = cv2.initUndistortRectifyMap(K1s, D1, R1, P1, self.size, cv2.CV_16SC2)
        self.mapR = cv2.initUndistortRectifyMap(K2s, D2, R2, P2, self.size, cv2.CV_16SC2)
        self.R1, self.R2, self.P1, self.P2, self.Q = R1, R2, P1, P2, Q
        self.f = float(P1[0, 0])
        self.cx = float(P1[0, 2])
        self.cy = float(P1[1, 2])
        self.B = float(-P2[0, 3] / P2[0, 0])  # 基线 (m)，正数
        self.K = P1[:3, :3].copy()            # 校正后左目针孔内参（无畸变）
        self.T_unit = "m"
        if self.B <= 0:
            raise ValueError(f"基线为负({self.B:.4f})：左右目可能反了，检查并排帧左右顺序/标定的 T 符号")
        if self.B > 1.0:
            raise ValueError(f"基线 {self.B:.3f} m 不合理：标定的 T 多半是 mm，用 from_file() 读会自动换算")

    @classmethod
    def from_file(cls, path, size_full=None, scale=0.5, alpha=0.0):
        c = load_stereo_calib(path)
        size = size_full or c["size"] or (1920, 1080)
        rig = cls(c["K1"], c["D1"], c["K2"], c["D2"], c["R"], c["T"], size, scale, alpha)
        rig.T_unit = c.get("T_unit", "m")
        return rig

    def split(self, sbs):
        """并排帧 → (左, 右)，并缩放到 self.size（若已是该尺寸则不缩放）。"""
        w2 = sbs.shape[1] // 2
        left, right = sbs[:, :w2], sbs[:, w2:]
        if (left.shape[1], left.shape[0]) != self.size:
            left = cv2.resize(left, self.size, interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, self.size, interpolation=cv2.INTER_AREA)
        return left, right

    def rectify(self, left, right):
        l = cv2.remap(left, self.mapL[0], self.mapL[1], cv2.INTER_LINEAR)
        r = cv2.remap(right, self.mapR[0], self.mapR[1], cv2.INTER_LINEAR)
        return l, r

    def to_cam(self, v):
        """校正左目系 → 原始左目相机系（只旋转，原点相同）。"""
        return self.R1.T @ np.asarray(v, float)


def decode_sbs_mjpg(buf, scale=0.5):
    """机载用：直接对 MJPG 原始缓冲做 DCT 域降采样解码，比 全解码+resize 快得多。
    V4L2 下 cap.set(cv2.CAP_PROP_CONVERT_RGB, 0) 后 cap.read() 得到的就是 MJPG 字节。"""
    flag = {1.0: cv2.IMREAD_COLOR, 0.5: cv2.IMREAD_REDUCED_COLOR_2,
            0.25: cv2.IMREAD_REDUCED_COLOR_4, 0.125: cv2.IMREAD_REDUCED_COLOR_8}[float(scale)]
    return cv2.imdecode(np.frombuffer(buf, np.uint8), flag)


# ----------------------------------------------------------------------------
# 红色分割 + 椭圆候选
# ----------------------------------------------------------------------------
def red_score(bgr):
    b, g, r = cv2.split(bgr)
    s = cv2.subtract(r, cv2.max(g, b))
    return s, r


def red_mask(score, r, cfg: RingConfig):
    m = (score > cfg.score_thr) & (r > cfg.r_min)
    return m.astype(np.uint8) * 255


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


def conic_row_x(C, y):
    """二次曲线与第 y 行的两个交点 x（升序），无交点返回 None。"""
    a = C[0, 0]
    b = 2 * (C[0, 1] * y + C[0, 2])
    c = C[1, 1] * y * y + 2 * C[1, 2] * y + C[2, 2]
    disc = b * b - 4 * a * c
    if disc < 0 or abs(a) < 1e-15:
        return None
    sq = np.sqrt(disc)
    x1, x2 = (-b - sq) / (2 * a), (-b + sq) / (2 * a)
    return (min(x1, x2), max(x1, x2))


def ellipse_half_height(el):
    (_, _), (w, h), ang = el
    a, b = w / 2.0, h / 2.0
    t = np.deg2rad(ang)
    return float(np.sqrt((a * np.sin(t)) ** 2 + (b * np.cos(t)) ** 2))


@dataclass
class RingCandidate:
    ellipse: tuple
    conic: np.ndarray
    partial: bool          # 触图像边缘（环部分出画），此时不做单目
    fill: float
    resid: float
    area: float
    bbox: tuple            # (x, y, w, h) 全图坐标
    comp: np.ndarray       # bbox 内属于该连通域的像素 (uint8 0/255，闭运算后)

    @property
    def center(self):
        return self.ellipse[0]

    @property
    def bbox_cx(self):
        return self.bbox[0] + 0.5 * self.bbox[2]


def _eval_candidate(parts, W, H, cfg: RingConfig):
    """parts: [(x, y, w, h, area_px, comp_roi, outer_pts_fullimg, hole_area), ...]
    多个部分（部分出画时环会断成左右两段）合并后评估。返回 (RingCandidate|None, why)。"""
    pts = np.vstack([p[6] for p in parts])
    on_border = (pts[:, 0] <= 1) | (pts[:, 0] >= W - 2) | (pts[:, 1] <= 1) | (pts[:, 1] >= H - 2)
    partial = bool(on_border.any())
    fit_pts = pts[~on_border]
    if len(fit_pts) < 20:
        return None, "few_contour_pts"
    el = cv2.fitEllipse(fit_pts.astype(np.float32))
    (ex, ey), (ew, eh), ang = el
    a, b = max(ew, eh) / 2.0, min(ew, eh) / 2.0
    if b < 4 or b / a < cfg.min_axis_ratio:
        return None, "axis_ratio"
    e_area = np.pi * a * b
    if e_area < cfg.min_ellipse_area:
        return None, "too_small"
    area_px = float(sum(p[4] for p in parts))
    holes = float(sum(p[7] for p in parts))
    fill = area_px / e_area
    if fill > cfg.max_fill:
        return None, "not_hollow"
    if not partial and holes / e_area < 0.2:
        return None, "no_hole"
    t = np.deg2rad(ang)
    d = fit_pts - np.array([ex, ey])
    xp = d[:, 0] * np.cos(t) + d[:, 1] * np.sin(t)
    yp = -d[:, 0] * np.sin(t) + d[:, 1] * np.cos(t)
    rr = np.sqrt((xp / (ew / 2)) ** 2 + (yp / (eh / 2)) ** 2)
    resid = float(np.median(np.abs(rr - 1)))
    if resid > (cfg.max_fit_resid_partial if partial else cfg.max_fit_resid):
        return None, "bad_fit"
    x0 = min(p[0] for p in parts)
    y0 = min(p[1] for p in parts)
    x1 = max(p[0] + p[2] for p in parts)
    y1 = max(p[1] + p[3] for p in parts)
    comp = np.zeros((y1 - y0, x1 - x0), np.uint8)
    for p in parts:
        sub = comp[p[1] - y0:p[1] - y0 + p[3], p[0] - x0:p[0] - x0 + p[2]]
        np.maximum(sub, p[5], out=sub)
    return RingCandidate(el, ellipse_to_conic(el), partial, fill, resid, float(e_area),
                         (int(x0), int(y0), int(x1 - x0), int(y1 - y0)), comp), "ok"


def find_ring(mask, cfg: RingConfig) -> Tuple[Optional[RingCandidate], str]:
    """在红色掩码里找最大的"空心椭圆"。环部分出画时会断成几段，额外尝试两两合并。"""
    H, W = mask.shape
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n <= 1:
        return None, "no_red"
    parts = []
    for lab in np.argsort(-stats[1:, cv2.CC_STAT_AREA])[:6] + 1:
        x, y, w, h, area_px = (int(v) for v in stats[lab])
        if area_px < 30 or w < 12 or h < 12:
            break
        comp = (labels[y:y + h, x:x + w] == lab).astype(np.uint8) * 255
        res = cv2.findContours(comp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        cnts, hier = res[-2], res[-1]
        if hier is None:
            continue
        hier = hier[0]
        outer = [i for i in range(len(cnts)) if hier[i][3] == -1]
        i = max(outer, key=lambda j: len(cnts[j]))
        pts = cnts[i].reshape(-1, 2) + np.array([x, y])
        holes = sum(cv2.contourArea(cnts[j]) for j in range(len(cnts)) if hier[j][3] != -1)
        parts.append((x, y, w, h, area_px, comp, pts, holes))
    best, why = None, "no_candidate"
    combos = [[p] for p in parts]
    touch = [p for p in parts
             if (p[0] <= 1 or p[1] <= 1 or p[0] + p[2] >= W - 1 or p[1] + p[3] >= H - 1)]
    def thick(p):  # 平均厚度 ≈ 2*面积/周长
        return 2.0 * p[4] / max(len(p[6]), 1)

    for i in range(len(touch)):
        for j in range(i + 1, len(touch)):
            p, q = touch[i], touch[j]
            if min(p[4], q[4]) < 0.1 * max(p[4], q[4]):
                continue  # 面积差太多：不是同一个环的两段弧
            tp, tq = thick(p), thick(q)
            if max(tp, tq) > 5.0 * min(tp, tq):
                continue  # 粗细差太多
            combos.append([p, q])
    for cb in combos:
        cand, w = _eval_candidate(cb, W, H, cfg)
        if cand is None:
            why = w
            continue
        if best is None or cand.area > best.area:
            best = cand
    return best, ("ok" if best is not None else why)


# ----------------------------------------------------------------------------
# 逐行取弧中心线（向量化）
# ----------------------------------------------------------------------------
def _row_arcs(score, raw_mask, cand: RingCandidate, cfg: RingConfig):
    """返回 dict: y -> (cL, wL, cR, wR) 的数组形式。
    每行只保留"恰好能分出左弧、右弧"的行：取该连通域在这一行最外侧的两段。"""
    x, y, w, h = cand.bbox
    Hh, Ww = raw_mask.shape
    comp = (cand.comp > 0) & (raw_mask[y:y + h, x:x + w] > 0)
    if cfg.run_gap > 0:  # 水平闭运算：填段内小缝，不移动段的外边缘
        kk = np.ones((1, cfg.run_gap + 1), np.uint8)
        comp = cv2.morphologyEx(comp.astype(np.uint8), cv2.MORPH_CLOSE, kk) > 0
        comp &= cand.comp > 0
    pad = np.zeros((h, 1), bool)
    M = np.hstack([pad, comp, pad]).astype(np.int8)
    d = np.diff(M, axis=1)
    rs, cs = np.nonzero(d == 1)    # 段起点 (行, 列)  列已是 ROI 内坐标
    re_, ce = np.nonzero(d == -1)  # 段终点+1
    ce = ce - 1
    keep = (ce - cs + 1) >= cfg.min_run
    rs, cs, ce = rs[keep], cs[keep], ce[keep]
    if len(rs) == 0:
        return None
    cnt = np.bincount(rs, minlength=h)
    first = np.searchsorted(rs, np.arange(h), "left")
    last = np.searchsorted(rs, np.arange(h), "right") - 1
    rows = np.flatnonzero(cnt >= 2)
    if len(rows) == 0:
        return None
    i0, i1 = first[rows], last[rows]
    ls, le, rs2, re2 = cs[i0] + x, ce[i0] + x, cs[i1] + x, ce[i1] + x
    yy = rows + y
    # 触图像左右边缘的段：弧被截断，丢弃
    ok = (ls > 0) & (re2 < Ww - 1)
    span = re2 - ls
    ok &= ((le - ls) <= 0.45 * span) & ((re2 - rs2) <= 0.45 * span) & (span >= 6)
    yy, ls, le, rs2, re2 = yy[ok], ls[ok], le[ok], rs2[ok], re2[ok]
    if len(yy) == 0:
        return None
    S = score.astype(np.float32)
    thr = float(cfg.score_thr)

    def edge_left(s):
        a, b = S[yy, np.maximum(s - 1, 0)], S[yy, s]
        den = np.where(b > a, b - a, 1.0)
        xl = np.where(b > a, (s - 1) + (thr - a) / den, s - 0.5)
        return np.clip(xl, s - 1.0, s.astype(float))

    def edge_right(e):
        a, b = S[yy, e], S[yy, np.minimum(e + 1, Ww - 1)]
        den = np.where(a > b, a - b, 1.0)
        xr = np.where(a > b, e + (a - thr) / den, e + 0.5)
        return np.clip(xr, e.astype(float), e + 1.0)

    a0, b0 = edge_left(ls), edge_right(le)
    a1, b1 = edge_left(rs2), edge_right(re2)
    return {"y": yy, "c0": 0.5 * (a0 + b0), "w0": b0 - a0, "c1": 0.5 * (a1 + b1), "w1": b1 - a1}


def extract_arc_pairs(sL, mL, sR, mR, candL: RingCandidate, candR: RingCandidate,
                      d_guess: float, cfg: RingConfig):
    """返回 ndarray [N, 5]: y, xL, xR, arc(0左/1右), width_px"""
    A = _row_arcs(sL, mL, candL, cfg)
    Bm = _row_arcs(sR, mR, candR, cfg)
    if A is None or Bm is None:
        return np.zeros((0, 5))
    yL, yR = A["y"], Bm["y"]
    common, iL, iR = np.intersect1d(yL, yR, return_indices=True)
    if not candL.partial:  # 完整环：只用竖直跨度中间段
        (_, ey), _, _ = candL.ellipse
        hy = ellipse_half_height(candL.ellipse)
        band = np.abs(common - ey) <= cfg.row_band * hy
        common, iL, iR = common[band], iL[band], iR[band]
    tol = cfg.disp_tol_rel * d_guess + cfg.disp_tol_abs
    out = []
    for arc in (0, 1):
        cL, wL = A[f"c{arc}"][iL], A[f"w{arc}"][iL]
        cR, wR = Bm[f"c{arc}"][iR], Bm[f"w{arc}"][iR]
        d = cL - cR
        ok = (d > 0) & (np.abs(d - d_guess) <= tol) & (np.abs(wL - wR) <= 0.5 * np.maximum(wL, wR) + 3)
        y, xl, xr, ww = common[ok].astype(float), cL[ok], cR[ok], wL[ok]
        if len(y) >= 3:  # 斜率过滤：只留陡峭段（上下段与极线近平行，视差不可靠）
            slope = np.gradient(xl, y)
            s_ok = np.abs(slope) <= cfg.max_slope
            y, xl, xr, ww = y[s_ok], xl[s_ok], xr[s_ok], ww[s_ok]
        else:
            continue
        out.append(np.stack([y, xl, xr, np.full_like(y, arc), ww], 1))
    return np.vstack(out) if out else np.zeros((0, 5))


def triangulate(pairs, rig: StereoRig, depth_scale=1.0):
    y, xL, xR = pairs[:, 0], pairs[:, 1], pairs[:, 2]
    d = xL - xR
    Z = rig.f * rig.B / d
    X = (xL - rig.cx) * Z / rig.f
    Y = (y - rig.cy) * Z / rig.f
    return np.stack([X, Y, Z], 1) * depth_scale


# ----------------------------------------------------------------------------
# 3D 圆拟合
# ----------------------------------------------------------------------------
def _basis(n):
    a = np.array([0.0, 1.0, 0.0]) if abs(n[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, n)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    return e1, e2


def plane_ransac(P, iters, thr, rng):
    n = len(P)
    idx = rng.integers(0, n, size=(iters, 3))
    a, b, c = P[idx[:, 0]], P[idx[:, 1]], P[idx[:, 2]]
    N = np.cross(b - a, c - a)
    L = np.linalg.norm(N, axis=1)
    good = L > 1e-9
    N, a = N[good] / L[good, None], a[good]
    if len(N) == 0:
        return np.ones(n, bool)
    D = np.einsum("ij,ij->i", N, a)
    dist = np.abs(P @ N.T - D[None, :])        # n x iters
    cnt = (dist < thr).sum(0)
    j = int(np.argmax(cnt))
    return dist[:, j] < thr


def _circle_resid(P, c, n, r):
    q = P - c
    h = q @ n
    rho = np.linalg.norm(q - h[:, None] * n[None, :], axis=1)
    return np.concatenate([h, rho - r])


def fit_circle3d(P, radius=None, init=None, iters=15):
    """3D 圆拟合。radius=None 时半径自由。返回 (c, n, r, rms)。"""
    if init is None:
        mu = P.mean(0)
        _, _, Vt = np.linalg.svd(P - mu, full_matrices=False)
        n = Vt[2]
        e1, e2 = _basis(n)
        u, v = (P - mu) @ e1, (P - mu) @ e2
        A = np.stack([u, v, np.ones_like(u)], 1)
        sol, *_ = np.linalg.lstsq(A, u * u + v * v, rcond=None)
        uc, vc = sol[0] / 2, sol[1] / 2
        r = float(np.sqrt(max(sol[2] + uc * uc + vc * vc, 1e-9)))
        c = mu + uc * e1 + vc * e2
    else:
        c, n, r = init
        c, n = np.array(c, float), np.array(n, float)
    if radius is not None:
        r = float(radius)
    free_r = radius is None
    lam = 1e-3
    for _ in range(iters):
        e1, e2 = _basis(n)

        def unpack(p):
            nn = n + p[3] * e1 + p[4] * e2
            nn = nn / np.linalg.norm(nn)
            return c + p[:3], nn, (r + p[5] if free_r else r)

        npar = 6 if free_r else 5
        p0 = np.zeros(npar)
        f0 = _circle_resid(P, *unpack(p0))
        J = np.empty((len(f0), npar))
        eps = 1e-6
        for k in range(npar):
            dp = p0.copy()
            dp[k] = eps
            J[:, k] = (_circle_resid(P, *unpack(dp)) - f0) / eps
        JTJ = J.T @ J
        step = np.linalg.solve(JTJ + lam * np.diag(np.diag(JTJ) + 1e-12), -J.T @ f0)
        c_new, n_new, r_new = unpack(step)
        if np.sum(_circle_resid(P, c_new, n_new, r_new) ** 2) < np.sum(f0 ** 2):
            c, n, r = c_new, n_new, r_new
            lam *= 0.3
        else:
            lam *= 10
            continue
        if np.linalg.norm(step[:3]) < 1e-6 and np.linalg.norm(step[3:5]) < 1e-7:
            break
    rms = float(np.sqrt(np.mean(_circle_resid(P, c, n, r) ** 2)))
    return c, n, float(r), rms


# ----------------------------------------------------------------------------
# 单目：椭圆 → 圆位姿（两解）
# ----------------------------------------------------------------------------
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


# ----------------------------------------------------------------------------
# 主入口
# ----------------------------------------------------------------------------
@dataclass
class RingResult:
    ok: bool = False
    source: str = "none"            # "stereo" / "mono" / "none"
    reason: str = ""
    # 校正左目系
    c: Optional[np.ndarray] = None
    n: Optional[np.ndarray] = None
    # 原始左目系（接 T_BC 用这个）
    c_cam: Optional[np.ndarray] = None
    n_cam: Optional[np.ndarray] = None
    # 诊断
    n_points: int = 0
    r_fit: float = float("nan")     # 自由半径拟合值 → r_fit/radius 是双目尺度因子
    rms: float = float("nan")
    stereo_c: Optional[np.ndarray] = None
    stereo_n: Optional[np.ndarray] = None
    mono: list = field(default_factory=list)  # [(c, n), ...]
    mono_c: Optional[np.ndarray] = None       # 选中的单目解
    mono_n: Optional[np.ndarray] = None
    mono_angle_deg: float = float("nan")      # 单目 vs 双目 法线夹角
    mono_dist_rel: float = float("nan")       # 单目 vs 双目 距离相对差
    normal_ambiguous: bool = False
    candL: Optional[RingCandidate] = None
    candR: Optional[RingCandidate] = None
    pairs: Optional[np.ndarray] = None        # [N,5] y,xL,xR,arc,w
    t_ms: dict = field(default_factory=dict)


def _ang(a, b):
    return float(np.degrees(np.arccos(np.clip(abs(float(a @ b)), -1, 1))))


def detect_ring(left, right, rig: StereoRig, cfg: RingConfig, rng=None) -> RingResult:
    """left/right: 已校正的 BGR 图（rig.rectify 的输出）。"""
    rng = rng or np.random.default_rng(cfg.seed)
    res = RingResult()
    t0 = time.perf_counter()
    sL, rL = red_score(left)
    sR, rR = red_score(right)
    mL, mR = red_mask(sL, rL, cfg), red_mask(sR, rR, cfg)
    t1 = time.perf_counter()
    candL, whyL = find_ring(mL, cfg)
    candR, whyR = find_ring(mR, cfg)
    res.candL, res.candR = candL, candR
    t2 = time.perf_counter()
    res.t_ms.update(mask=(t1 - t0) * 1e3, contour=(t2 - t1) * 1e3)

    # ---- 单目（左目）----
    if candL is not None and not candL.partial:
        res.mono = circle_pose_from_ellipse(candL.ellipse, rig.K, cfg.radius + cfg.tube / 2)

    stereo_ok, why = False, ""
    if candL is None:
        why = f"left:{whyL}"
    elif candR is None:
        why = f"right:{whyR}"
    else:
        (_, eyL), _, _ = candL.ellipse
        (_, eyR), _, _ = candR.ellipse
        hy = ellipse_half_height(candL.ellipse)
        d_guess = candL.bbox_cx - candR.bbox_cx
        if not (candL.partial or candR.partial) and abs(eyL - eyR) > 3 + 0.05 * hy:
            why = f"row_mismatch({eyL - eyR:+.1f}px)"
        elif d_guess <= 0:
            why = "neg_disparity"
        else:
            t3 = time.perf_counter()
            pairs = extract_arc_pairs(sL, mL, sR, mR, candL, candR, d_guess, cfg)
            t4 = time.perf_counter()
            res.t_ms["rows"] = (t4 - t3) * 1e3
            res.pairs = pairs
            if len(pairs) < cfg.min_points:
                why = f"few_points({len(pairs)})"
            else:
                P = triangulate(pairs, rig, cfg.depth_scale)
                inr = (P[:, 2] > cfg.z_min) & (P[:, 2] < cfg.z_max)
                P, pairs = P[inr], pairs[inr]
                if len(P) < cfg.min_points:
                    why = f"few_points_z({len(P)})"
                else:
                    zmed = float(np.median(P[:, 2]))
                    sigz = zmed ** 2 / (rig.f * rig.B) * 0.5
                    thr = max(cfg.ransac_thr, 3 * sigz)
                    inl = plane_ransac(P, cfg.ransac_iters, thr, rng)
                    Pi = P[inl]
                    if len(Pi) < cfg.min_points:
                        why = f"few_inliers({len(Pi)})"
                    else:
                        if len(Pi) > cfg.max_fit_points:  # 均匀抽稀，LM 耗时 ∝ 点数
                            Pi = Pi[np.linspace(0, len(Pi) - 1, cfg.max_fit_points).astype(int)]
                        # 1) 固定半径拟合（条件好，不会发散）
                        c, n, _, rms = fit_circle3d(Pi, cfg.radius)
                        # 2) 去离群后再拟合一次
                        rr = _circle_resid(Pi, c, n, cfg.radius)
                        rr = np.hypot(rr[:len(Pi)], rr[len(Pi):])
                        mad = np.median(rr) * 1.4826 + 1e-6
                        k = rr < max(3 * mad, 3 * sigz, 0.005)
                        if k.sum() >= cfg.min_points:
                            Pi = Pi[k]
                            c, n, _, rms = fit_circle3d(Pi, cfg.radius, init=(c, n, cfg.radius))
                        # 3) 尺度诊断：点到圆心的平面内距离中位数
                        q = Pi - c
                        h_ = q @ n
                        res.r_fit = float(np.median(np.linalg.norm(q - h_[:, None] * n[None, :], axis=1)))
                        if not cfg.fix_radius:
                            c, n, _, rms = fit_circle3d(Pi, None, init=(c, n, res.r_fit))
                        if n @ c > 0:
                            n = -n
                        res.stereo_c, res.stereo_n, res.rms = c, n, float(rms)
                        res.n_points = len(Pi)
                        if abs(res.r_fit / cfg.radius - 1) > cfg.max_radius_dev:
                            why = f"radius_dev(r_fit={res.r_fit:.3f})"
                        elif c[2] > cfg.stereo_max_z:
                            why = f"too_far_for_stereo({c[2]:.2f}m)"
                        else:
                            stereo_ok = True
            res.t_ms["fit"] = (time.perf_counter() - t4) * 1e3

    # ---- 单目解选择 + 交叉校验 ----
    if res.mono:
        if res.stereo_n is not None:
            j = int(np.argmax([abs(m[1] @ res.stereo_n) for m in res.mono]))
        else:
            # 无双目：取"最正对相机"的解（飞机大致从正面来的先验），标记二义
            j = int(np.argmax([-(m[1] @ (m[0] / np.linalg.norm(m[0]))) for m in res.mono]))
            res.normal_ambiguous = len(res.mono) > 1
        res.mono_c, res.mono_n = res.mono[j]
        if res.stereo_c is not None:
            res.mono_angle_deg = _ang(res.mono_n, res.stereo_n)
            dm, ds = np.linalg.norm(res.mono_c), np.linalg.norm(res.stereo_c)
            res.mono_dist_rel = float((dm - ds) / ds)

    if stereo_ok:
        agree = (np.isnan(res.mono_angle_deg)
                 or (res.mono_angle_deg <= cfg.max_mono_angle_deg
                     and abs(res.mono_dist_rel) <= cfg.max_mono_dist_rel))
        if agree or not cfg.require_mono_agree:
            res.ok, res.source, res.c, res.n = True, "stereo", res.stereo_c, res.stereo_n
            res.reason = "ok" if agree else "ok(mono_disagree)"
        else:
            res.reason = (f"mono_disagree(ang={res.mono_angle_deg:.1f}deg,"
                          f"dist={res.mono_dist_rel * 100:+.1f}%)")
    elif res.mono_c is not None:
        res.ok, res.source, res.c, res.n = True, "mono", res.mono_c, res.mono_n
        res.reason = f"mono_only[{why}]"
    else:
        res.reason = why or "no_ring"

    if res.ok:
        res.c_cam, res.n_cam = rig.to_cam(res.c), rig.to_cam(res.n)
    res.t_ms["total"] = (time.perf_counter() - t0) * 1e3
    return res


# ----------------------------------------------------------------------------
# 可视化
# ----------------------------------------------------------------------------
def project(rig: StereoRig, X):
    X = np.atleast_2d(X)
    return np.stack([rig.f * X[:, 0] / X[:, 2] + rig.cx, rig.f * X[:, 1] / X[:, 2] + rig.cy], 1)


def circle_points(c, n, r, k=120):
    e1, e2 = _basis(np.asarray(n, float))
    t = np.linspace(0, 2 * np.pi, k, endpoint=False)
    return np.asarray(c)[None, :] + r * (np.cos(t)[:, None] * e1 + np.sin(t)[:, None] * e2)


def draw_result(img, res: RingResult, rig: StereoRig, cfg: RingConfig, color=(255, 0, 255), text=True):
    out = img.copy()
    if res.candL is not None:
        cv2.ellipse(out, res.candL.ellipse, (0, 255, 255), 1, cv2.LINE_AA)
    if res.pairs is not None and len(res.pairs):
        for y, xL, _, arc, _ in res.pairs:
            cv2.circle(out, (int(round(xL)), int(round(y))), 1,
                       (0, 255, 0) if arc == 0 else (255, 255, 0), -1)
    if res.ok:
        pts = project(rig, circle_points(res.c, res.n, cfg.radius))
        cv2.polylines(out, [np.round(pts).astype(np.int32)], True, color, 2, cv2.LINE_AA)
        p0 = project(rig, res.c)[0]
        p1 = project(rig, res.c + 0.3 * res.n)[0]
        cv2.arrowedLine(out, tuple(np.round(p0).astype(int)), tuple(np.round(p1).astype(int)),
                        color, 2, cv2.LINE_AA, tipLength=0.2)
    if text:
        lines = [f"{res.source} {res.reason}"]
        if res.ok:
            yaw = np.degrees(np.arctan2(-res.n[0], -res.n[2]))
            pit = np.degrees(np.arctan2(res.n[1], -res.n[2]))
            lines.append(f"c=({res.c[0]:+.2f},{res.c[1]:+.2f},{res.c[2]:.2f})m "
                         f"yaw={yaw:+.1f} pitch={pit:+.1f}")
        if not np.isnan(res.r_fit):
            lines.append(f"pts={res.n_points} r_fit/R={res.r_fit / cfg.radius:.3f} rms={res.rms * 1000:.1f}mm")
        if not np.isnan(res.mono_angle_deg):
            lines.append(f"mono vs stereo: {res.mono_angle_deg:.1f}deg {res.mono_dist_rel * 100:+.1f}%")
        lines.append(" ".join(f"{k}={v:.1f}" for k, v in res.t_ms.items()) + " ms")
        for i, s in enumerate(lines):
            cv2.putText(out, s, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, s, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out
