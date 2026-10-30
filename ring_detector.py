# -*- coding: utf-8 -*-
"""
ring_detector.py —— 兼容壳 (2026-10-30)
===============================================================================
StereoRig / load_stereo_calib / decode_sbs_mjpg → vb09/perception/stereo_rig.py
ellipse_to_conic / circle_pose_from_ellipse     → vb09/perception/ring/geom3d.py
代码原样。老的"红色圆环检测"管线 (RingConfig / detect_ring …) 已删除。
本仓库里的程序都直接 import 新位置; 这个文件只给还没改的老工具用, 确认没人用后可以删。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vb09.perception.stereo_rig import StereoRig, load_stereo_calib, decode_sbs_mjpg  # noqa: E402,F401
from vb09.perception.ring.geom3d import ellipse_to_conic, circle_pose_from_ellipse  # noqa: E402,F401
