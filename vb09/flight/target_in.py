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
import gate_plan as gp      # 同目录, 纯函数
from vb09.flight import config as C

from vb09.flight.ctx import F


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
        self.rej = {k: 0 for k in self.REASONS}
        self.n_valid = 0
        self.n_aim_far = 0

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

    # ---- 判定 ----
    def judge(self, st, now, p_now):
        """→ (样本 dict | None, 拒绝原因)"""
        try:
            # ★按环心穿 (2026-10-19): 新版桥每帧发环心观测 ring_w (+ ring_cov)。target 是红点的
            #   融合估计, 红点没定位上时只是"环心 + 0.3m 先验" (σ 停在 6~7cm) —— 有 ring_w 就用它当 c,
            #   红点 (aim) 有就瞄红点, 没有就瞄环心。老版桥没有 ring_w → 照旧用 target。
            rw = st.get("ring_w")
            raw = rw is not None
            tsrc = str(st.get("src") or "")
            t_age = st.get("age_ms")
            if (C.GATE_DOT_PREFER and st.get("target") is not None and st.get("published", True)
                    and tsrc.startswith(("dot_stereo", "dot_ray_plane"))
                    and isinstance(t_age, (int, float)) and t_age <= C.GATE_DOT_MAX_AGE_MS):
                raw = False                        # 双目红点: 比 3.5m 外的单目环心准一个量级 → 用桥的融合估计
            if raw:
                c = tuple(float(v) for v in rw[:3])
            else:
                c = st.get("target")
                if c is None or not st.get("published", True):
                    return None, "unpub"
                c = tuple(float(v) for v in c[:3])
            if st.get("pose", "ok") != "ok":
                return None, "pose"
            if st.get("sim") and not (C.GATE_SHADOW or C.GATE_ALLOW_SIM_DRIVE):
                return None, "sim"
            t_cap = float(st.get("t", now))
            if now - t_cap > C.GATE_CAP_MAX_AGE:
                return None, "old"
            dist = None if raw else st.get("dist")
            if not isinstance(dist, (int, float)):
                dist = gp.vnorm(gp.vsub(c, p_now)) if p_now else float("nan")
            if not (C.GATE_D_MIN <= dist <= C.GATE_D_MAX):
                return None, "dist"
            n = st.get("n")
            if n is not None:
                n = gp.vunit(tuple(float(v) for v in n[:3]))
                nsd = st.get("n_sd_deg")
                if isinstance(nsd, (int, float)) and nsd > C.GATE_N_SD_MAX_DEG:
                    return None, "n_sd"
                if p_now is not None:
                    los = gp.vsub(p_now, c)
                    if gp.vdot(los, n) < 0:
                        n = gp.vmul(n, -1.0)      # 约定: 指向进近侧
                    if gp.ang_deg(los, n) > C.GATE_VIEW_ANG_MAX:
                        return None, "view"
            elif C.GATE_REQUIRE_NORMAL:
                return None, "no_n"
            # σ 拆到靶面 (§1): 进入方向 ≈ −n, 没有法向就用视线方向 (和规划一致)
            d_in = gp.hunit(gp.vmul(n, -1.0)) if n is not None else None
            if d_in is None and p_now is not None:
                d_in = gp.hunit(gp.vsub(c, p_now))
            sd = st.get("sd_cm") or []
            cov_c = st.get("ring_cov") if raw else st.get("cov")
            sig = gp.sigma_hva(d_in, cov_c, None if raw else sd) if d_in is not None else None
            if sig is None:                        # 拆不了 (没有 σ / 正上方) → 按最大分量当横向
                sd_max = max(float(v) for v in sd) if sd else float("nan")
                sig = (sd_max, sd_max, sd_max)
            sd_lat = max(sig[0], sig[1])
            if not (sd_lat <= C.GATE_SD_MAX_CM):
                return None, "sd"
            lim_al = C.GATE_SD_ALONG_MAX_CM
            if raw and dist == dist:
                lim_al = max(lim_al, C.GATE_SD_ALONG_PER_M_CM * dist)
            if not (sig[2] <= lim_al):
                return None, "sd_along"
            aim = st.get("aim")
            if aim is not None:
                aim = tuple(float(v) for v in aim[:3])
                if gp.vnorm(gp.vsub(aim, c)) > C.R_IN - 0.05:
                    aim = None                     # 红点跑到环外 = 红点识别有问题, 按环心瞄
                    self.n_aim_far += 1
            return {"t": t_cap, "recv": now, "c": c, "aim": aim, "n": n, "dist": float(dist),
                    "sd": sd_lat, "sig": sig, "n_sd": st.get("n_sd_deg"),
                    "raw": raw, "src": "ring_" + str(st.get("ring_src")) if raw else st.get("src"),
                    "p": p_now}, "ok"
        except Exception:
            return None, "bad"

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
        txt = "识别到了但单帧被拒: " + ", ".join(f"{self.REJ_TXT.get(k, k)}×{v}" for k, v in
                                          sorted(d.items(), key=lambda kv: -kv[1])) + extra
        return (nv, rej), txt

