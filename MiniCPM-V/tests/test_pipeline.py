# -*- coding: utf-8 -*-
"""不依赖模型的单元测试（只测 CV 和数据结构）。"""

import numpy as np
import cv2

from mcpv import config as C
from mcpv.cv import find_candidates, crop_for_vlm
from mcpv.schema import DetectionResult, Candidate
from mcpv.vlm import parse_verdict


def _canvas(w=1920, h=1080, gray=245):
    return np.full((h, w, 3), gray, np.uint8)


def test_white_dot_passes():
    """白底上画一个红圆 → 应该产出 1 个通过白底过滤的候选。"""
    img = _canvas()
    cv2.circle(img, (900, 500), 30, (30, 30, 220), -1)
    cands = find_candidates(img)
    passed = [c for c in cands if c.passed_cv]
    assert len(passed) >= 1
    c = passed[0]
    assert abs(c.cx - 900) < 5 and abs(c.cy - 500) < 5
    assert c.score >= C.RING_SCORE_MIN


def test_empty_image_no_candidate():
    """纯白画面 → 一个候选都没有。"""
    assert find_candidates(_canvas()) == []


def test_red_pipe_rejected():
    """又长又红的条（模拟管道）→ 圆度不够，应被淘汰。"""
    img = _canvas()
    cv2.rectangle(img, (200, 300), (1600, 330), (30, 30, 200), -1)
    cands = find_candidates(img)
    assert all(not c.passed_cv for c in cands)


def test_crop_size():
    """裁剪块永远是 CROP_OUT_PX 的正方形。"""
    img = _canvas()
    cv2.circle(img, (500, 500), 25, (20, 20, 210), -1)
    c = [x for x in find_candidates(img) if x.passed_cv][0]
    patch = crop_for_vlm(img, c)
    assert patch.shape[:2] == (C.CROP_OUT_PX, C.CROP_OUT_PX)


def test_parse_verdict():
    assert parse_verdict("A") == "A"
    assert parse_verdict("B：其他红色物体") == "B"
    assert parse_verdict("不是") == "B"
    assert parse_verdict("") == "?"


def test_schema_to_dict():
    c = Candidate(cx=1, cy=2, diameter_px=10.0, bbox_px=[0, 0, 20, 20],
                  circularity=0.9, ring_s=20.0, ring_v=200.0, score=170.0,
                  passed_cv=True, vlm_verdict="A")
    r = DetectionResult(image="x.jpg", image_path="p", image_size=[100, 80],
                        has_target=True, stage="vlm", n_candidates=1,
                        n_passed_cv=1, elapsed_s=1.0, vlm_calls=1, candidates=[c])
    d = r.to_dict()
    assert d["has_target"] is True
    assert d["best"]["mode"] == "A"
    assert d["candidates"][0]["bbox_px"] == [0, 0, 20, 20]


# ---------------- 点击 → 飞行（纯几何）----------------

def test_aim_center_goes_forward():
    """点画面正中 → 基本"往前飞"（带一点下，因为相机下俯 5.23°）。"""
    from mcpv.aim import Aimer
    a = Aimer.from_config(1920, 1080)
    cmd = a.command(a.cx, a.cy, step_m=3.0)
    assert cmd.forward > 2.8
    assert abs(cmd.left) < 0.2
    assert -0.6 < cmd.up < 0.0


def test_aim_right_click_moves_right():
    """点画面右侧 → left 为负（机体系 y 轴向左为正）。"""
    from mcpv.aim import Aimer
    a = Aimer.from_config(1920, 1080)
    cmd = a.command(a.cx + 500, a.cy, step_m=3.0)
    assert cmd.left < -0.5


def test_aim_scales_with_resolution():
    """半分辨率时焦距应正好是标定值的一半。"""
    from mcpv.aim import Aimer
    from mcpv import config as C
    a = Aimer.from_config(960, 540)
    assert abs(a.fx - C.FX / 2) < 1.0
    assert abs(a.cx - C.CX / 2) < 1.0


def test_range_from_size():
    """f=1579、20 cm 标记占 40 px → 距离约 7.9 m。"""
    from mcpv.aim import Aimer
    z = Aimer.range_from_size(40, 0.20, 1579.0)
    assert abs(z - 7.9) < 0.1
