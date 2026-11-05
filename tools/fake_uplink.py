# -*- coding: utf-8 -*-
"""
tools/fake_uplink.py —— 没有飞机时, 假装是 gate 模式 (机上检测) 的桥, 把检测画面发给地面 gcs_view.py
================================================================================================
2026-11-05。用的是**桥里真实的** vision_bridge.GateWorker (桩 rospy, 同 tools/baseline/onboard_replay):
录像帧 + 位姿 → 校正 → Detector.process → extra → GateOut.emit → Detector.draw 叠加 → JPEG,
然后按 uplink_loop 的包头格式 PUB 到 tcp://*:5555 —— gcs_view 收到的和真飞机发的一模一样。
同时 SUB 地面的 5556 回包, 打印往返时延 (验证 gcs_view 的回包)。

  python3 tools/fake_uplink.py <录像目录> [--target reddot] [--hz 10] [--rep 10] [--cam P R Y] [--loops 0]

  录像目录 = tools/rec_stereo.py 存的格式 (reddot_exp): *.jpg 左|右并排 + frames.csv + poses.csv + stereo_calib.yaml
  每张快照当成"悬停 --rep 帧" 按 --hz 发 (时间压缩, 位姿用快照的真实位姿), 这样轨迹 / 法向逻辑和飞行时一样累积。
  --loops 0 = 一直循环; Ctrl-C 退出。
  然后另一个终端:  python3 gcs_view.py --drone 127.0.0.1 [--record rec_view/]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "baseline"))
sys.path.insert(0, ROOT)


class _Cam(object):
    def register(self, name):
        return threading.Event()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--target", default="reddot")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--rep", type=int, default=10, help="每张快照重复发几帧 (模拟悬停)")
    ap.add_argument("--cam", type=float, nargs=3, default=[-0.85, 1.18, 1.71], metavar=("PITCH", "ROLL", "YAW"),
                    help="外参 (默认 = reddot_exp 拟合值; 旧外参 5.23 3.4 -0.73)")
    ap.add_argument("--loops", type=int, default=0, help="循环几遍, 0 = 一直循环")
    ap.add_argument("--jpeg-q", type=int, default=85)
    a = ap.parse_args()

    import _compat                                    # noqa: E402  (桩 rospy)
    vb = _compat.bridge()
    import cv2
    import numpy as np
    import rospy
    from geometry_msgs.msg import PoseStamped
    from vb09.common.wire import pack, unpack, zmq

    if zmq is None:
        sys.exit("pip install pyzmq")
    D = a.dir
    rows = list(csv.DictReader(open(os.path.join(D, "frames.csv"))))
    P = np.loadtxt(os.path.join(D, "poses.csv"), delimiter=",", skiprows=1)
    cfg = {"target": a.target, "calib": os.path.join(D, "stereo_calib.yaml"), "cam_w": 3840, "cam_h": 1080,
           "det_scale": 0.5, "gate_params": "/nonexistent.json", "gate_sim": True, "max_pose_gap": 0.1,
           "gate_reset_age": 0.0, "det_hz": 30, "tx_hz": a.hz, "jpeg_q": a.jpeg_q,
           "cam_pitch_deg": a.cam[0], "cam_roll_deg": a.cam[1], "cam_yaw_deg": a.cam[2],
           "cam_x": 0.08, "cam_y": 0.03, "cam_z": -0.115}
    rospy.PRINT[0] = False
    w = vb.GateWorker(cfg, _Cam())

    def pose_at(t):
        i = min(max(int(np.searchsorted(P[:, 0], t)), 1), len(P) - 1)
        x, y = P[i - 1], P[i]
        s = (t - x[0]) / max(y[0] - x[0], 1e-6)
        q = x[4:8] * (1 - s) + y[4:8] * s
        return x[1:4] * (1 - s) + y[1:4] * s, q / np.linalg.norm(q)

    ctx = zmq.Context()
    up = ctx.socket(zmq.PUB)
    up.setsockopt(zmq.SNDHWM, 2)
    up.setsockopt(zmq.LINGER, 0)
    up.bind("tcp://*:5555")
    down = ctx.socket(zmq.SUB)
    down.setsockopt(zmq.CONFLATE, 1)
    down.setsockopt(zmq.LINGER, 0)
    down.setsockopt_string(zmq.SUBSCRIBE, "")
    down.connect("tcp://127.0.0.1:5556")
    rtt = []

    def down_loop():
        while True:
            if down.poll(200):
                m = unpack(down.recv())
                if m.get("t_cap") is not None:
                    rtt.append((time.time() - float(m["t_cap"])) * 1000.0)
    threading.Thread(target=down_loop, daemon=True).start()
    print("[fake_uplink] 发 tcp://*:5555 (gate 模式包头), 收 127.0.0.1:5556 回包。目标 %s, %d 张快照 × %d 帧 @%.0fHz"
          % (a.target, len(rows), a.rep, a.hz))

    seq, loop = 0, 0
    imgs = [cv2.imread(os.path.join(D, r["file"])) for r in rows]
    try:
        while a.loops == 0 or loop < a.loops:
            loop += 1
            for r, img in zip(rows, imgs):
                p, q = pose_at(float(r["t_cap"]))
                for _ in range(max(1, a.rep)):
                    t = time.time()                    # 用本机时钟当"飞机时钟": 回包的往返就是真的
                    for tp in (t - 0.04, t):           # 真飞机位姿 30 Hz; 每帧只给一个会碰到 max_pose_gap (0.1 s)
                        ps = PoseStamped()
                        ps.header.stamp = rospy.Time(tp)
                        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
                        (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                         ps.pose.orientation.w) = (float(v) for v in q)
                        w.on_pose(ps)
                    w.last_prev = 0.0                  # 每帧都出预览
                    w.step(None, img, rospy.Time(t))
                    item = w.take_preview()
                    if item is not None:
                        jpg, iw, ih, stamp, st = item
                        seq += 1
                        head = {"seq": seq, "t_cap": stamp.to_sec(), "w": iw, "h": ih, "fmt": "jpeg",
                                "src": "gate_left_rect", "sbs": False, "tx_mode": "gate",
                                "K": [w.rig.f, w.rig.f, w.rig.cx, w.rig.cy], "ground_z": 0.0,
                                "pose": {"p": [float(v) for v in p], "q": [float(v) for v in q]},
                                "fc": {"mode": "FAKE(fake_uplink)", "armed": False, "connected": True},
                                "ph": json.dumps({"phase": "fake_uplink", "sub_mode": "", "air": False}),
                                "gate": st, "jpg": jpg}
                        if rtt:
                            s = sorted(rtt[-50:])
                            head.update(rtt_ms=rtt[-1], rtt_p50=s[len(s) // 2], rtt_p95=s[int(len(s) * 0.95)],
                                        rtt_max=s[-1], rtt_n=len(rtt), rtt_stall=0)
                        up.send(pack(head))
                    time.sleep(max(0.0, 1.0 / a.hz - (time.time() - t)))
                st = w.out.last or {}
                print("[fake_uplink] %s  阶段 %-6s 距离 %s  %s  往返 %s"
                      % (r["file"], st.get("stage"), st.get("dist", "--"),
                         ("n ±%.1f°" % st["n_sd_deg"]) if st.get("n") else ("无法向: %s" % st.get("n_why", "-")),
                         ("%.0f ms" % rtt[-1]) if rtt else "没收到回包"))
    except KeyboardInterrupt:
        pass
    print("[fake_uplink] 发了 %d 帧, 收到回包 %d" % (seq, len(rtt)))


if __name__ == "__main__":
    main()
