# -*- coding: utf-8 -*-
"""
tools/calib_check.py —— 标定读取的自检 (PLAN_refactor_v2 第 6 步 1/2)
    python3 tools/calib_check.py          全过打印 ALL PASS, 否则退出码 1
用临时文件造几份标定 (cv2.FileStorage yaml + 普通 json), 检查:
  · 尺寸: image_size 或 image_width/height; 和调用方的单目分辨率不一致 → ValueError
  · swap_halves / flip_180: 不为 0 → ValueError
"""
import json
import os
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from vb09.perception.stereo_rig import StereoRig, load_stereo_calib  # noqa: E402

K = np.array([[1579, 0, 960], [0, 1579, 540], [0, 0, 1.0]])
D = np.zeros(5)
R = np.eye(3)
T = np.array([-60.08, 0, 0.0])
TMP = tempfile.mkdtemp(prefix="calib_check_")


def mk_fs(name, extra):
    p = os.path.join(TMP, name + ".yaml")
    fs = cv2.FileStorage(p, cv2.FILE_STORAGE_WRITE)
    for k, v in dict(K1=K, D1=D, K2=K, D2=D, R=R, T=T).items():
        fs.write(k, v)
    for k, v in extra.items():
        fs.write(k, v)
    fs.release()
    return p


def mk_json(name, extra):
    p = os.path.join(TMP, name + ".json")
    d = {k: np.asarray(v).tolist() for k, v in dict(K1=K, D1=D, K2=K, D2=D, R=R, T=T).items()}
    d.update(extra)
    with open(p, "w") as f:
        json.dump(d, f)
    return p


# (名字, 造文件, 额外键, 期望: "ok" / 报错信息里要有的字)
CASES = [
    ("none", mk_fs, {}, "ok"),
    ("size_ok", mk_fs, {"image_size": np.array([1920, 1080.0])}, "ok"),
    ("size_bad", mk_fs, {"image_size": np.array([1280, 720.0])}, "标定尺寸不符"),
    ("wh_ok", mk_fs, {"image_width": 1920, "image_height": 1080}, "ok"),
    ("wh_bad", mk_fs, {"image_width": 960, "image_height": 540}, "标定尺寸不符"),
    ("conflict", mk_fs, {"image_size": np.array([1920, 1080.0]), "image_width": 1280, "image_height": 720},
     "自相矛盾"),
    ("json_wh_bad", mk_json, {"image_width": 1280, "image_height": 720}, "标定尺寸不符"),
    ("json_size_ok", mk_json, {"image_size": [1920, 1080]}, "ok"),
    ("flags_zero", mk_fs, {"swap_halves": 0, "flip_180": 0}, "ok"),
    ("swap", mk_fs, {"swap_halves": 1}, "swap_halves = 1"),
    ("flip", mk_fs, {"flip_180": 1, "image_size": np.array([1920, 1080.0])}, "flip_180 = 1"),
    ("json_swap", mk_json, {"swap_halves": 1}, "swap_halves = 1"),
    ("json_flags_false", mk_json, {"swap_halves": False, "flip_180": False}, "ok"),
]


def main():
    bad = 0
    for name, mk, extra, want in CASES:
        p = mk(name, extra)
        try:
            StereoRig.from_file(p, (1920, 1080), 0.5)
            got = "ok"
            msg = "size=%s" % (load_stereo_calib(p)["size"],)
        except ValueError as e:
            got = str(e)
            msg = got[:80]
        ok = (got == "ok") if want == "ok" else (want in got)
        bad += not ok
        print("%-4s %-14s %s" % ("ok" if ok else "FAIL", name, msg))
    print("=" * 60)
    print("ALL PASS" if not bad else "%d FAIL" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
