# -*- coding: utf-8 -*-
"""
gate_detector.py —— 兼容壳 (2026-10-30)
===============================================================================
代码搬到 vb09/perception/ring/ (算法说明在 vb09/perception/ring/__init__.py) 和 vb09/common/wire.py,
原样未改。本仓库里的程序都直接 import 新位置; 这个文件只给还没改的老工具 (gate_tune.py 之类)
继续 `import gate_detector as gd; gd.GateDetector(...)` 用。确认飞机 / 地面都没有老工具在用后可以删。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vb09.perception.ring.config import GateConfig  # noqa: E402,F401
from vb09.perception.ring.frame import RingObs, DotObs, TargetState, GateFrame  # noqa: E402,F401
from vb09.perception.ring.det2d import (  # noqa: E402,F401
    apply_wb, segment, find_gate, hue_side, find_corners, find_dot)
from vb09.perception.ring.geom3d import (  # noqa: E402,F401
    match_corners, fit_plane, ring_center_in_plane, ring_tilt_deg, ellipse_disparity_normal,
    pick_ring_normal, ellipse_to_conic, circle_pose_from_ellipse, _basis, _angle)
from vb09.perception.ring.fusion import TargetEstimator, GateFuser, _cov_along_ray, _N_FUSE  # noqa: E402,F401
from vb09.perception.ring.detect import GateDetector  # noqa: E402,F401
from vb09.perception.ring.draw import STAGE_COLOR, draw_frame  # noqa: E402,F401
from vb09.perception.ring.wire import MSG_VERSION, frame_to_msg, frame_from_msg  # noqa: E402,F401
from vb09.common.wire import rig_to_msg  # noqa: E402,F401
