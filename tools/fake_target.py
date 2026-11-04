#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/fake_target.py —— 虚拟目标: 没有环 / 没有识别时, 测飞行逻辑 (Acquire → 停靠 / 影子)
===============================================================================
在世界系里放一个固定的假目标, 按 /gate/status 的格式发出去 (sim=false), 外加 /vision_bridge/cam
(mode=gate_gcs), 飞行脚本 stage14_mission.py 就当它是真识别。相机、桥、地面站都不用开。

测的是: 预检 → 起飞 → 手动段 → T → 转头找目标 → 粗估 → 摆位 (move_to) → 对正 (turn_to) → 悬停 → 回手动段 → 降落,
         以及 gate.csv / events.log / 网页遥测。用真实 PX4 + VINS。
不测的: 相机、检测、标定、桥 (虚拟目标的位置是精确的, 只加了一点高斯噪声和延迟)。

    # 飞机上 (桥 run_vision.py 先停掉 —— 两个节点同时发 /gate/status 会打架, 本工具检测到会拒绝启动)
    python3 tools/fake_target.py --ahead 4.5           # 目标 = 启动时飞机正前方 4.5 m, 离地 1.5 m
    python3 stage14_mission.py --site <档案>           # 档案里 mode = 停靠, dock_dist = 3

    停靠点 = 目标前 dock_dist (沿法向) → 飞机会从起飞点往前飞 ahead − dock_dist (上例 1.5 m)。
    ★ 目标是虚拟的, 可以"在墙里" —— 真正要飞到的只有停靠点。房间纵深按 ahead − dock_dist + 余量 算。

几何约定 (和 sim/run_sim.py 一样): 法向 n 指向进近侧 (= 指回飞机); 只有飞机在目标前方、
距离 0.8–9 m、机头偏离视线 < --fov 时才"看得到", 看不到时发 stage=search 的空帧 (和真桥一样)。
启动时锚定一次 (之后飞机怎么动目标都不动); 重启本工具会按新位置重新锚定 —— 飞行中别重启, 或用 --at 固定。
Python 3.8。
"""
import argparse
import json
import math
import random
import sys
import threading
import time

STATUS_TOPIC = "/gate/status"
CAM_TOPIC = "/vision_bridge/cam"
POSE_TOPIC = "/mavros/local_position/pose"


def unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return [x / n for x in a]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def yaw_of_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FakeTarget(object):
    """纯计算 (不碰 ROS): 给定目标几何 + 当前飞机位姿 → 一条 /gate/status dict。"""

    def __init__(self, c, n, noise_cm=1.0, fov_deg=40.0, normal=True, seed=None):
        self.c = list(c)
        self.n = unit(n)                      # 指向进近侧
        self.noise = noise_cm / 100.0
        self.fov = math.radians(fov_deg)
        self.normal = normal
        self.rng = random.Random(seed)
        self.n_vis = 0
        self.n_all = 0

    @classmethod
    def anchored(cls, p0, yaw0, ahead, left, up, z_ground, yaw_off_deg=0.0, **kw):
        """目标 = p0 前方 ahead / 左 left, 高度 = z_ground + up; 法向指回飞机 (可偏 yaw_off)。"""
        f = (math.cos(yaw0), math.sin(yaw0))
        l = (-f[1], f[0])
        c = [p0[0] + ahead * f[0] + left * l[0], p0[1] + ahead * f[1] + left * l[1], z_ground + up]
        a = yaw0 + math.pi + math.radians(yaw_off_deg)
        return cls(c, [math.cos(a), math.sin(a), 0.0], **kw)

    def visible(self, p, yaw):
        rel = [self.c[i] - p[i] for i in range(3)]
        dist = math.sqrt(dot(rel, rel))
        brg = math.atan2(rel[1], rel[0])
        off = math.atan2(math.sin(brg - yaw), math.cos(brg - yaw))
        front = dot([-r for r in rel], self.n) > 0.3
        return front and 0.8 < dist < 9.0 and abs(off) < self.fov, dist

    def status(self, t_cap, p, yaw):
        self.n_all += 1
        vis, dist = self.visible(p, yaw)
        st = {"t": t_cap, "stage": "mid" if vis else "search", "pose": "ok", "sim": False,
              "fake": True, "published": vis}
        if not vis:
            return st
        self.n_vis += 1
        g = self.rng.gauss
        sd_cm = max(0.5, 2.0 * self.noise * 100.0)        # 报的 σ = 实际噪声的 2 倍 (保守, 和真桥量级一致)
        tgt = [self.c[i] + g(0, self.noise) for i in range(3)]
        st.update(target=[round(v, 4) for v in tgt], aim=[round(v, 4) for v in tgt],
                  dist=round(dist, 3), sd_cm=[sd_cm, sd_cm, sd_cm], src="fake", age_ms=0)
        if self.normal:
            st["n"] = [round(v, 4) for v in unit([self.n[i] + g(0, 0.005) for i in range(3)])]
            st["n_sd_deg"] = 0.5
        return st


def other_publishers(topic, me):
    """→ topic 上除我以外的发布者列表; 查不了返回 None。"""
    try:
        import rosgraph
        pubs, _, _ = rosgraph.Master(me).getSystemState()
        for t, nodes in pubs:
            if t == topic:
                return [x for x in nodes if x != me]
        return []
    except Exception:
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="虚拟目标: 按 /gate/status 格式发一个固定的假目标")
    ap.add_argument("--ahead", type=float, default=4.5, help="目标在启动时机头前方多远 m (默认 4.5)")
    ap.add_argument("--left", type=float, default=0.0, help="目标偏左多少 m (右为负)")
    ap.add_argument("--up", type=float, default=1.5, help="目标离地 m (= 启动时飞机高度 + 这个; 默认 1.5 = takeoff_h)")
    ap.add_argument("--yaw-off", type=float, default=0.0, help="法向相对'指回飞机'再转多少 ° (测斜进入)")
    ap.add_argument("--at", default=None, help="直接给世界坐标 x,y,z (不锚定; 法向仍按 --yaw-off 相对启动机头)")
    ap.add_argument("--no-normal", action="store_true", help="不发法向 (停靠方向按视线)")
    ap.add_argument("--rate", type=float, default=15.0, help="发布频率 Hz (默认 15, 接近地面检测)")
    ap.add_argument("--lat", type=float, default=0.12, help="假装的拍摄延迟 s (t = 现在 − lat)")
    ap.add_argument("--noise", type=float, default=1.0, help="目标位置高斯噪声 cm (默认 1)")
    ap.add_argument("--fov", type=float, default=40.0, help="机头偏离视线多少 ° 以内算看得到")
    ap.add_argument("--dock", type=float, default=3.0, help="只用来打印停靠点 (= 档案 dock_dist)")
    ap.add_argument("--force", action="store_true", help="/gate/status 上已有别的发布者也照样启动")
    a = ap.parse_args(argv)

    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped

    rospy.init_node("fake_target", anonymous=False)
    me = rospy.get_name()
    others = other_publishers(STATUS_TOPIC, me)
    if others:
        print("[fake] ✗ %s 已经有发布者 %s —— 桥 (run_vision.py) 还开着? 先停掉它。"
              "确实要一起发就加 --force" % (STATUS_TOPIC, others))
        if not a.force:
            return 2
    elif others is None:
        print("[fake] (查不了 %s 的发布者, 自己确认桥已经停了)" % STATUS_TOPIC)

    lk = threading.Lock()
    pose = [None]

    def on_pose(m):
        with lk:
            pose[0] = m

    rospy.Subscriber(POSE_TOPIC, PoseStamped, on_pose, queue_size=1)
    pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=5)
    pub_cam = rospy.Publisher(CAM_TOPIC, String, queue_size=1)

    t_end = time.time() + 10.0
    while pose[0] is None and time.time() < t_end and not rospy.is_shutdown():
        time.sleep(0.05)
    if pose[0] is None:
        print("[fake] ✗ 10 s 收不到 %s (mavros / VINS 没起来?)" % POSE_TOPIC)
        return 1
    with lk:
        m = pose[0]
    p0 = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
    yaw0 = yaw_of_quat(m.pose.orientation)
    kw = dict(noise_cm=a.noise, fov_deg=a.fov, normal=not a.no_normal)
    if a.at:
        c = [float(v) for v in a.at.split(",")]
        ang = yaw0 + math.pi + math.radians(a.yaw_off)
        ft = FakeTarget(c, [math.cos(ang), math.sin(ang), 0.0], **kw)
    else:
        ft = FakeTarget.anchored(p0, yaw0, a.ahead, a.left, a.up, p0[2], yaw_off_deg=a.yaw_off, **kw)
    c, n = ft.c, ft.n
    S = [c[0] + n[0] * a.dock, c[1] + n[1] * a.dock, c[2]]
    print("[fake] 飞机 (%+.2f,%+.2f,%+.2f) 机头 %+.1f°" % (p0[0], p0[1], p0[2], math.degrees(yaw0)))
    print("[fake] 虚拟目标 c = (%+.3f,%+.3f,%+.3f)  法向 %s  噪声 %.1f cm  延迟 %.2f s  %.0f Hz"
          % (c[0], c[1], c[2], "(%+.2f,%+.2f)" % (n[0], n[1]) if not a.no_normal else "不发", a.noise, a.lat, a.rate))
    if a.no_normal:
        print("[fake] 不发法向: 停靠点 = 目标前 dock_dist 沿按 T 那一刻的视线方向 (取决于当时飞机在哪)")
    else:
        print("[fake] dock_dist=%.1f 时停靠点 ≈ (%+.2f,%+.2f,%+.2f): 离起飞点水平 %.2f m"
              % (a.dock, S[0], S[1], S[2], math.hypot(S[0] - p0[0], S[1] - p0[1])))
    print("[fake] 复现同一目标: --at %.3f,%.3f,%.3f  (法向按 --yaw-off 相对机头)" % tuple(c))

    period = 1.0 / a.rate
    nxt = time.time()
    t_cam = 0.0
    t_print = time.time()
    while not rospy.is_shutdown():
        now = time.time()
        with lk:
            m = pose[0]
        p = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        yaw = yaw_of_quat(m.pose.orientation)
        st = ft.status(rospy.Time.now().to_sec() - a.lat, p, yaw)
        pub.publish(String(json.dumps(st)))
        if now - t_cam > 1.0:
            t_cam = now
            pub_cam.publish(String(json.dumps({"cfg": {"mode": "gate_gcs"}, "fake": True})))
        if now - t_print > 2.0:
            t_print = now
            vis, d = ft.visible(p, yaw)
            print("[fake] 飞机 (%+.2f,%+.2f,%+.2f) 离目标 %.2f m  %s  已发 %d (看得到 %d)"
                  % (p[0], p[1], p[2], d, "看得到" if vis else "看不到", ft.n_all, ft.n_vis))
        nxt += period
        d = nxt - time.time()
        if d > 0:
            time.sleep(d)
        else:
            nxt = time.time()
    return 0


if __name__ == "__main__":
    sys.exit(main())
