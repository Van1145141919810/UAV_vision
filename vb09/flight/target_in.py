# -*- coding: utf-8 -*-
"""
vb09/flight/target_in.py —— /gate/status、/vision_bridge/cam 的接收 + 单帧判定窗口 (GateInput)
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import json
import math
import threading
from collections import deque

import rospy
from std_msgs.msg import String
from vb09.flight import config as C

from vb09.flight.ctx import F
from vb09.perception import registry


# =============================================================================
# ==========================   穿环: 识别输入   ===============================
# =============================================================================
class GateInput:
    """
    /gate/status (vision_bridge 每帧一条 JSON) 和 /vision_bridge/cam 的接收。
    回调里当场按当时的飞机位姿判一次能不能用 (judge), 能用的进滑窗 self.win。
    读的一方只拿拷贝, 回调和状态机互不阻塞。

    字段 (INTERFACE_gate.md §1):
      target = 环心 c (world)          —— 现在的 bridge 就有
      n / aim / n_sd_deg / cov         —— 约定里的新增字段, bridge 补上之前是 None
                                          (cov: 2026-10-19 的 vision_bridge 已经发)
      t = 拍摄时刻 t_cap, sim, pose ("ok" = 位姿插值成功), dist, sd_cm, published
    """
    REASONS = ("unpub", "pose", "sim", "old", "dist", "sd", "sd_along", "n_sd", "view", "no_n", "bad")

    def __init__(self):
        self.lock = threading.Lock()
        self.last = None
        self.last_recv = None
        self.n_msgs = 0
        self.cam = None
        self.cam_recv = None
        self.win = deque(maxlen=900)          # 有效样本 (dict)
        self.n_valid = 0
        # 2026-11-01 接线: 判定按目标类型选 (C.TARGET_KIND → vb09/perception/registry.py JUDGES)
        self._judge = registry.load(registry.JUDGES, C.TARGET_KIND)(C)
        # 2026-11-05: 判定插件自己的拒绝原因也计数 (以前红点的 stale / score / hits … 被丢掉, 日志只写"没看到")
        extra = tuple(k for k in getattr(self._judge, "REASONS", ()) if k not in self.REASONS)
        self.rej = {k: 0 for k in self.REASONS + extra}
        self.rej_txt = dict(self.REJ_TXT, **getattr(self._judge, "REJ_TXT", {}))
        # 选点 (2026-11-08, PLAN_dot_v2 §4): 飞行脚本决定, 经 /gate/select 发给桥; 网页点选先到这里排队
        self.tracked = bool(getattr(self._judge, "TRACKED", False))
        self._sel_pub = None
        self._sel_seq = 0
        self._web_sel = None

    # ---- 回调 ----
    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except Exception:
            with self.lock:
                self.rej["bad"] += 1
            return
        now = rospy.Time.now().to_sec()
        p = F.tele.pose
        p_now = (p.pose.position.x, p.pose.position.y, p.pose.position.z) if p else None
        smp, why = self.judge(st, now, p_now)
        with self.lock:
            self.last, self.last_recv = st, now
            self.n_msgs += 1
            if smp is not None:
                self.win.append(smp)
                self.n_valid += 1
            elif why in self.rej:
                self.rej[why] += 1

    def on_cam(self, msg):
        try:
            c = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.cam, self.cam_recv = c, rospy.Time.now().to_sec()

    # ---- 选点 ----
    def send_select(self, cmd, why=""):
        """发一条选点指令给桥 (/gate/select)。cmd: {"mode": "track", "trk_id": …} / {"mode": "ray", "xn", "yn", "t_cap"}
        / {"mode": "none"}。→ seq (桥在 /gate/status 的 sel.seq 回显)。"""
        if self._sel_pub is None:
            self._sel_pub = rospy.Publisher("/gate/select", String, queue_size=10)
        with self.lock:
            self._sel_seq += 1
            seq = self._sel_seq
        msg = dict(cmd, seq=seq)
        self._sel_pub.publish(String(data=json.dumps(msg)))
        rospy.loginfo("[gate] 选点 → 桥 seq %d: %s%s" % (seq, json.dumps(cmd, ensure_ascii=False),
                                                      ("  (%s)" % why) if why else ""))
        return seq

    def web_select(self, cmd):
        """网页点选 (flight_monitor → /flight_ctrl/cmd) 先存这里, Detect 步骤取走再发。"""
        with self.lock:
            self._web_sel = dict(cmd)

    def take_web_select(self):
        with self.lock:
            c, self._web_sel = self._web_sel, None
        return c

    # ---- 判定 (2026-10-30: 环专用的判定搬到 vb09/perception/ring/judge.py, 经 JUDGES 调用) ----
    def judge(self, st, now, p_now):
        """→ (样本 dict | None, 拒绝原因)"""
        return self._judge.judge(st, now, p_now)

    @property
    def n_aim_far(self):
        return self._judge.n_aim_far

    # ---- 读 ----
    def valid_since(self, t_recv):
        with self.lock:
            return [s for s in self.win if s["recv"] >= t_recv]

    def latest(self):
        """→ (最新一条原始 JSON, 收到至今秒数)"""
        with self.lock:
            st, t = self.last, self.last_recv
        if st is None:
            return None, float("inf")
        return st, rospy.Time.now().to_sec() - t

    def latest_valid(self):
        with self.lock:
            return self.win[-1] if self.win else None

    def cam_mode(self):
        with self.lock:
            c, t = self.cam, self.cam_recv
        if c is None or rospy.Time.now().to_sec() - t > 5.0:
            return None
        return (c.get("cfg") or {}).get("mode") or "aruco"

    def counters(self):
        with self.lock:
            return self.n_valid, sum(self.rej.values()), dict(self.rej)

    REJ_TXT = {"unpub": "桥没发布", "pose": "位姿不对", "sim": "假数据", "old": "帧太旧", "dist": "距离超范围",
               "sd": "横向σ太大", "sd_along": "深度σ太大", "n_sd": "法向不稳", "view": "斜视角太大",
               "no_n": "没法向", "bad": "消息坏"}

    def rej_since(self, snap):
        """snap = 上次的 (n_valid, rej dict)。→ (新快照, 这段时间的拒绝说明 或 None)。
        2026-10-21: 以前识别到了但每帧都被拒时, 日志只写"没看到环", 看不出是被哪道门卡住。"""
        nv, _, rej = self.counters()
        if snap is None:
            return (nv, rej), None
        d = {k: v - snap[1].get(k, 0) for k, v in rej.items() if v - snap[1].get(k, 0) > 0}
        if not d:
            return (nv, rej), None
        with self.lock:
            st = self.last
        extra = ""
        if d.get("sd_along") or d.get("sd") or d.get("dist"):
            try:
                rw = (st or {}).get("ring_w") or (st or {}).get("target")
                if rw is not None and F.tele.pose is not None:
                    q = F.tele.pose.pose.position
                    extra = f" (最新一帧环心距 {math.dist(rw[:3], (q.x, q.y, q.z)):.1f}m)"
            except Exception:
                pass
        txt = "识别到了但单帧被拒: " + ", ".join(f"{self.rej_txt.get(k, k)}×{v}" for k, v in
                                          sorted(d.items(), key=lambda kv: -kv[1])) + extra
        return (nv, rej), txt

