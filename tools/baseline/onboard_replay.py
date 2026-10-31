# -*- coding: utf-8 -*-
"""
onboard_replay.py —— 机上检测 (vision_bridge _mode:=gate) 等价验证
  synth 合成双目 (放大成 3840×1080 并排帧) + 位姿 → vision_bridge.GateWorker.step
  (校正 → Detector.process(带 T_WC) → 估计器重置判定 → Detector.extra → GateOut.emit → 预览画图)
  判定: /gate/status JSON 逐字段相同 (ms / det_ms 耗时字段除外), 每帧都出预览
golden 在 2026-11-01 接线之前的提交 (3d21c21 之后、接线之前) 上写的; 接线后逐字段相同。
"""
import json
import os
import sys
import tempfile
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _compat  # noqa: E402
import _util  # noqa: E402
import synth  # noqa: E402

N = 40
TIMING = {"ms", "det_ms"}


def _calib(path):
    import cv2
    import numpy as np
    K = np.array([[1579.0, 0, 959.5], [0, 1579.0, 539.5], [0, 0, 1.0]])
    D = np.zeros(5)
    R, _ = cv2.Rodrigues(np.array([0.002, -0.004, 0.001]))      # = synth.make_rig
    T = np.array([-60.08, 0.4, -2.5])                            # mm (加载时换成米)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    for k, v in dict(K1=K, D1=D, K2=K, D2=D, R=R, T=T).items():
        fs.write(k, v)
    fs.release()


class _Cam(object):
    def register(self, name):
        return threading.Event()


def run():
    import cv2
    import numpy as np
    vb = _compat.bridge()
    import rospy
    from geometry_msgs.msg import PoseStamped
    calib = os.path.join(tempfile.mkdtemp(), "calib.yaml")
    _calib(calib)
    cfg = {"calib": calib, "cam_w": 3840, "cam_h": 1080, "det_scale": 0.5, "gate_params": "/nonexistent.json",
           "gate_sim": True, "max_pose_gap": 0.1, "gate_reset_age": 0.0, "det_hz": 15, "tx_hz": 10,
           "jpeg_q": 85}
    cfg.update(synth.CAM)
    out = []
    rospy._bus.pop("/gate/status", None)
    rospy.Subscriber("/gate/status", None, lambda m: out.append(json.loads(m.data)))
    w = vb.GateWorker(cfg, _Cam())
    rig = synth.make_rig()
    nprev = 0
    for i, (t, T_WC, p, q, fl) in enumerate(synth.sequence(rig)[:N]):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time(t)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
        (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w) = q
        w.on_pose(ps)
        L, R = synth.render_pair(rig, T_WC, fl["dot"], fl["visible"], seed=i)
        w.step(None, cv2.resize(np.hstack([L, R]), (3840, 1080)), rospy.Time(t))
        nprev += w.take_preview() is not None
    return {"status": _util.strip_keys(out, TIMING), "n_preview": nprev, "n_reset": w.n_reset}


def main():
    write = _util.main_args()
    ok = _util.report("onboard", *_util.save_or_compare("onboard.json", run(), write))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
