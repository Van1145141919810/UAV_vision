# -*- coding: utf-8 -*-
"""
tools/reddot_replay.py —— 红点识别离线回放 (reddot_exp 目录: *.jpg 左|右并排 + frames.csv + poses.csv + stereo_calib.yaml)

    python3 tools/reddot_replay.py <数据目录> [--cam 俯仰 横滚 偏航] [--lever x y z] [--truth 0-10] [--seq] [--save 目录]

  默认逐帧: 每帧新建一个 DotLocator, 只看单帧门限 + 深度融合 (不依赖时序)。
  --seq   按时间顺序喂同一个 locator (轨迹 / 自校准 D_eff / 法向一致性)。
  --dt 0.2 --rep 10   (配 --seq) 稀疏快照的时间压缩: 每帧当成悬停 rep 帧、间隔 dt 秒喂进去 (位姿仍按真实 t_cap)。
          reddot_exp 的快照隔 6–13 s, 超过 track_timeout, 不压缩的话每帧都是新目标, 测不出轨迹和多视角法向。
  --truth a-b  这个范围的帧(含)里应该看到红点, 其余不该有; 汇总 检出 / 误检。
  --cam   外参 (俯仰向下为正, 单位度)。默认读 meta.json 里的 cam_extrinsics —— 那是拍摄时的旧值,
          更新过外参后必须显式传新值。
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from vb09.perception.stereo_rig import StereoRig                      # noqa: E402
from vb09.perception.reddot.config import DotConfig                   # noqa: E402
from vb09.perception.reddot.detect import DotDetectorCore             # noqa: E402
from vb09.perception.reddot.locate import DotLocator                  # noqa: E402


def make_T_BC(pitch_down_deg, roll_deg, yaw_deg, lever):              # 同 vision_bridge.make_T_BC (它 import ROS)
    R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    p, r, yw = np.radians([pitch_down_deg, roll_deg, yaw_deg])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Rz = np.array([[np.cos(yw), -np.sin(yw), 0], [np.sin(yw), np.cos(yw), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Rx @ Ry @ R0
    T[:3, 3] = lever
    return T


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_at(P, t):
    i = min(max(int(np.searchsorted(P[:, 0], t)), 1), len(P) - 1)
    a, b = P[i - 1], P[i]
    s = (t - a[0]) / max(b[0] - a[0], 1e-6)
    q = a[4:8] * (1 - s) + b[4:8] * s
    T = np.eye(4)
    T[:3, :3] = quat_R(q / np.linalg.norm(q))
    T[:3, 3] = a[1:4] * (1 - s) + b[1:4] * s
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--cam", type=float, nargs=3, metavar=("PITCH", "ROLL", "YAW"))
    ap.add_argument("--lever", type=float, nargs=3, default=[0.08, 0.03, -0.115])
    ap.add_argument("--truth", default="")
    ap.add_argument("--seq", action="store_true")
    ap.add_argument("--dt", type=float, default=0.0)
    ap.add_argument("--rep", type=int, default=1)
    ap.add_argument("--save", default="")
    ap.add_argument("--cfg", default="")
    a = ap.parse_args()
    D = a.dir
    cam = a.cam
    if cam is None:
        ex = json.load(open(os.path.join(D, "meta.json"))).get("cam_extrinsics", {})
        cam = [ex.get("cam_pitch_deg", 0.0), ex.get("cam_roll_deg", 0.0), ex.get("cam_yaw_deg", 0.0)]
        print("⚠ 没给 --cam, 用 meta.json 里拍摄时的旧外参 %s" % (cam,))
    T_BC = make_T_BC(cam[0], cam[1], cam[2], a.lever)
    cfg = DotConfig.load(a.cfg) if a.cfg else DotConfig()
    rig = StereoRig.from_file(os.path.join(D, "stereo_calib.yaml"), (1920, 1080), 1.0)
    P = np.loadtxt(os.path.join(D, "poses.csv"), delimiter=",", skiprows=1)
    det = DotDetectorCore(rig, cfg)
    loc = DotLocator(cfg, rig.f, rig.R1)
    truth = None
    if a.truth:
        lo, hi = (int(v) for v in a.truth.split("-"))
        truth = (lo, hi)
    n_tp = n_fn = n_fp = 0
    for k, row in enumerate(csv.DictReader(open(os.path.join(D, "frames.csv")))):
        im = cv2.imread(os.path.join(D, row["file"]))
        L, R = rig.rectify(*rig.split(im))
        t = float(row["t_cap"])
        fr = det.process(L, R, t)
        fr0 = fr
        T_WC = pose_at(P, t) @ T_BC
        if not a.seq:
            loc = DotLocator(cfg, rig.f, rig.R1)
            # 单帧模式没有时序: 同一帧喂两次让 min_hits 过
            loc.update(fr, T_WC, t)
            loc.update(fr, T_WC, t + 1e-3)
        elif a.dt > 0:
            import copy
            for _ in range(max(1, a.rep)):
                t_seq = getattr(main, "_t", 0.0) + a.dt
                main._t = t_seq
                fr = copy.deepcopy(fr0)
                loc.update(fr, T_WC, t_seq)
        else:
            loc.update(fr, T_WC, t)
        found = fr.target.valid and fr.stage not in ("lost", "search")    # 和桥 GateOut.emit 的 published 一致
        line = "%s %-6s ncand=%d pass=%s rej=[%s]" % (row["file"], fr.stage, len(fr.cands),
                                                       fr.diag.get("n_pass"), " ".join(fr.rejected))
        if found:
            ex = loc.extra(fr, T_WC)
            line += " | Z=%.2f(st %.2f sz %.2f) D=%.1fcm aim=%s ±%s score=%.2f" % (
                fr.diag.get("z", -1), fr.diag.get("z_st", -1), fr.diag.get("z_sz", -1), fr.diag.get("d_est_cm", -1),
                ex.get("aim"), ex.get("aim_sd_cm"), ex.get("score", 0))
            if "n" in ex:
                line += " n=%s ±%.1f° 跨度%.0f°" % (ex["n"], ex["n_sd_deg"], ex.get("n_span_deg", 0))
            elif ex.get("n_why"):
                line += " (没有 n: %s)" % ex["n_why"]
            line += " 轨迹%d" % len(loc.tracks)
        print(line)
        if truth is not None:
            inside = truth[0] <= k <= truth[1]
            if inside and found:
                n_tp += 1
            elif inside:
                n_fn += 1
            elif found:
                n_fp += 1
        if a.save:
            os.makedirs(a.save, exist_ok=True)
            from vb09.perception.reddot.plugin import RedDotDetector
            cv2.imwrite(os.path.join(a.save, row["file"]), RedDotDetector.draw(
                type("S", (), {"cfg": cfg, "loc": loc})(), L, fr))
    if truth is not None:
        print("检出 %d / 漏检 %d / 误检 %d" % (n_tp, n_fn, n_fp))


if __name__ == "__main__":
    main()
