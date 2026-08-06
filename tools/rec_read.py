#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/rec_read.py —— 读 tools/rec_stereo.py 存的截图目录 (飞机上 / 地面站都能跑, 不要 ROS)
===============================================================================
    python3 tools/rec_read.py <目录>                         # 列表: 每张的时刻、标签、位姿、大小
    python3 tools/rec_read.py <目录> --export out/           # 导出校正后左目 PNG (全分辨率)
    python3 tools/rec_read.py <目录> --export out/ --both    # 左右都导 (左右同行, 看视差用)
    python3 tools/rec_read.py <目录> --export out/ --calib new.yaml   # 重标以后用新标定重新校正

在代码里用 (以后红点检测的离线回放就从这里取帧):
    from tools.rec_read import Recording
    rec = Recording(path)
    for fr in rec.frames():
        fr.n, fr.t, fr.label            # 序号、t_cap (ROS 秒)、标签
        L, R = fr.stereo()              # 全分辨率原始左右目 (BGR), 未校正
        Lr, Rr = rec.rig.rectify(L, R)  # 校正 (rig = 目录里那份标定, scale=1)
        T_WB = fr.pose                  # 4×4 world←body (截图时按 t_cap 插值好的), 没有就是 None
Python 3.8。
"""
import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _quat_R(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Frame(object):
    def __init__(self, rec, row):
        self.rec = rec
        self.n = int(row["n"])
        self.file = row["file"]
        self.t = float(row["t_cap"])
        self.label = row.get("label", "")
        v = [row.get(k, "") for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        self._p = [float(x) for x in v] if all(v) else None

    def jpeg(self):
        with open(os.path.join(self.rec.path, self.file), "rb") as fh:
            return fh.read()

    def sbs(self, flag=cv2.IMREAD_COLOR):
        return cv2.imdecode(np.frombuffer(self.jpeg(), np.uint8), flag)

    def stereo(self):
        img = self.sbs()
        w2 = img.shape[1] // 2
        return img[:, :w2], img[:, w2:]

    @property
    def pose(self):
        if self._p is None:
            return None
        T = np.eye(4)
        T[:3, :3] = _quat_R(*self._p[3:])
        T[:3, 3] = self._p[:3]
        return T


class Recording(object):
    def __init__(self, path, calib=None):
        self.path = path
        self.calib = calib or os.path.join(path, "stereo_calib.yaml")
        with open(os.path.join(path, "meta.json")) as fh:
            self.meta = json.load(fh)
        with open(os.path.join(path, "frames.csv")) as fh:
            self.rows = list(csv.DictReader(fh))
        self._rig = None

    @property
    def rig(self):
        """截图时那份标定 (或 --calib 指定的), 全分辨率 (scale=1)。"""
        if self._rig is None:
            sys.path.insert(0, ROOT)
            from vb09.perception.stereo_rig import StereoRig
            w, h = self.meta.get("actual_wh") or self.meta["req_wh"]
            self._rig = StereoRig.from_file(self.calib, size_full=(w // 2, h), scale=1.0)
        return self._rig

    def frames(self, label=None):
        for r in self.rows:
            if label is None or r.get("label") == label:
                yield Frame(self, r)


def summary(rec):
    m = rec.meta
    print("目录   %s" % rec.path)
    print("相机   %s  %s  时钟 %s  标定 %s  控件 %s"
          % (m.get("device"), "x".join(map(str, m.get("actual_wh") or [])), m.get("clock"),
             m.get("calib_sha12"), " ".join(m.get("ctrl_set") or []) or "默认 (自动曝光)"))
    frs = list(rec.frames())
    if not frs:
        print("没有截图")
        return
    t0 = frs[0].t
    tot = 0
    for f in frs:
        size = os.path.getsize(os.path.join(rec.path, f.file))
        tot += size
        T = f.pose
        ps = "(%.2f, %.2f, %.2f)" % tuple(T[:3, 3]) if T is not None else "无位姿"
        print("  %-28s +%7.2f s  %-22s %5.0f KB  %s" % (f.file, f.t - t0, ps, size / 1e3, f.label))
    print("共 %d 张, %.1f MB, 有位姿 %d 张" % (len(frs), tot / 1e6, sum(f.pose is not None for f in frs)))


def main():
    ap = argparse.ArgumentParser(description="读 rec_stereo 截图目录")
    ap.add_argument("path")
    ap.add_argument("--export", default="", help="导出校正图到这个目录")
    ap.add_argument("--both", action="store_true", help="左右目都导 (默认只导左目)")
    ap.add_argument("--calib", default="", help="不用目录里那份标定")
    args = ap.parse_args()

    rec = Recording(args.path, args.calib or None)
    summary(rec)
    if not args.export:
        return
    os.makedirs(args.export, exist_ok=True)
    n = 0
    for fr in rec.frames():
        base = os.path.join(args.export, os.path.splitext(fr.file)[0])
        L, R = rec.rig.rectify(*fr.stereo())
        cv2.imwrite(base + "_L.png", L)
        if args.both:
            cv2.imwrite(base + "_R.png", R)
        n += 1
    print("导出 %d 张 → %s" % (n, args.export))


if __name__ == "__main__":
    main()
