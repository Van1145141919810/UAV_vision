# -*- coding: utf-8 -*-
"""
vb09/perception/stereo_rig.py —— 双目校正 (StereoRig) + 标定读取 + MJPG 降采样解码
===============================================================================
2026-10-30 从 ring_detector.py 原样搬来 (老的"红色圆环检测"管线已删除)。
所有检测都在"校正后左目系"里做。纯函数 / 无 ROS 依赖, Windows 离线调试与机载共用。

★ 标定单位: stereo_calib.yaml 的 T 是 mm。load_stereo_calib 会自动换成米。
坐标约定: 相机系 OpenCV: x 右, y 下, z 前。
"""
from __future__ import annotations

import os

import cv2
import numpy as np


_KEYS = {
    "K1": ["K1", "M1", "cameraMatrix1", "K_left", "KL", "mtx_l", "mtxL", "left_K", "camera_matrix_left"],
    "D1": ["D1", "dist1", "distCoeffs1", "D_left", "DL", "dist_l", "distL", "left_D", "dist_coeffs_left"],
    "K2": ["K2", "M2", "cameraMatrix2", "K_right", "KR", "mtx_r", "mtxR", "right_K", "camera_matrix_right"],
    "D2": ["D2", "dist2", "distCoeffs2", "D_right", "DR", "dist_r", "distR", "right_D", "dist_coeffs_right"],
    "R": ["R", "R_lr", "rotation"],
    "T": ["T", "T_lr", "translation"],
    "size": ["image_size", "imageSize", "size", "img_size", "imgSize"],
    # ★2026-10-31 (PLAN_refactor_v2 §4 第 6 步 1): 尺寸也可能分两个标量写
    "w": ["image_width", "imageWidth", "img_width", "width"],
    "h": ["image_height", "imageHeight", "img_height", "height"],
    # ★2026-10-31 (第 6 步 2): 标定工具记下的"采集时左右对调 / 旋转 180°"。本代码不做这两种变换,
    #   不为 0 → from_file 拒绝 (以前被悄悄忽略 → 左右目或上下颠倒, 基线符号 / 校正全错)
    "swap_halves": ["swap_halves", "swapHalves", "swap_lr"],
    "flip_180": ["flip_180", "flip180", "rotate_180"],
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
    返回 dict: K1 D1 K2 D2 R T size(w,h, 单目尺寸) T_unit swap_halves flip_180 (没写 = 0)。"""
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
                    if node.isInt() or node.isReal():   # 标量 (image_width: 1920): 新版 cv2 的 mat() 会抛
                        m = np.array([node.real()])
                    else:
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
    if "w" in found and "h" in found and found["w"].size >= 1 and found["h"].size >= 1:
        wh = (int(found["w"].ravel()[0]), int(found["h"].ravel()[0]))
        if out["size"] is not None and out["size"] != wh:
            raise ValueError(f"标定文件 {path} 自相矛盾: image_size = {out['size'][0]}x{out['size'][1]}, "
                             f"image_width/height = {wh[0]}x{wh[1]}")
        out["size"] = wh
    for k in ("swap_halves", "flip_180"):
        v = found.get(k)
        out[k] = int(round(float(v.ravel()[0]))) if v is not None and v.size >= 1 else 0
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
        """
        size_full: 调用方写死的单目全分辨率 (地面 1920×1080, 机上 cam_w/2 × cam_h)。
        ★2026-10-31: 标定文件里写了尺寸且和 size_full 不一致 → ValueError (以前悄悄用 size_full,
          内参按别的分辨率标的 → 校正和 f 全错, 测距偏一个倍数)。文件里没写尺寸 → 照旧用 size_full。
        """
        c = load_stereo_calib(path)
        if size_full is not None and c["size"] is not None \
                and (int(size_full[0]), int(size_full[1])) != c["size"]:
            raise ValueError(f"标定尺寸不符: {path} 是按 {c['size'][0]}x{c['size'][1]} 标的, "
                             f"这里的单目分辨率是 {int(size_full[0])}x{int(size_full[1])}。"
                             f"换对应分辨率的标定, 或改采集分辨率 (机上 ~cam_w/~cam_h)")
        flags = ["%s = %d" % (k, c[k]) for k in ("swap_halves", "flip_180") if c.get(k)]
        if flags:
            raise ValueError(f"标定文件 {path} 里 {', '.join(flags)}: 这份标定是在左右对调 / 旋转 180° 的画面上做的, "
                             f"本代码不做这种变换 (以前被悄悄忽略)。按现在的接线 / 安装方向重标, "
                             f"或确认后把这两个键改成 0")
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
