# -*- coding: utf-8 -*-
"""
vb09/perception/ring/judge.py —— 飞行脚本侧: 一条 /gate/status 能不能当样本用 (环)
===============================================================================
2026-10-30 从 stage12_gate.GateInput.judge 原样搬来 (第 4.3 步)。飞行脚本经 JUDGES["ring"] 调用:
    judge = JUDGES["ring"](C)          # C = vb09.flight.config (门限都在里面: GATE_SD_MAX_CM …)
    sample, why = judge.judge(st, now, p_now)
gp.xxx → geom.xxx (gate_plan 本来就是从 vb09/common/geom 重新导出的, 同一份函数)。
和目标类型有关的判断都在这里, 飞行步骤里不出现 `if kind == ...`。
"""
from vb09.common import geom


class RingJudge(object):
    """
    字段 (INTERFACE_gate.md §1):
      target = 环心 c (world) / ring_w = 这一帧的环心观测 (优先) / n / aim / n_sd_deg / cov / ring_cov
      t = 拍摄时刻 t_cap, sim, pose ("ok" = 位姿插值成功), dist, sd_cm, published
    """
    kind = "ring"

    def __init__(self, C):
        self.C = C
        self.n_aim_far = 0           # 红点跑到环外、改瞄环心的帧数

    def judge(self, st, now, p_now):
        """→ (样本 dict | None, 拒绝原因)"""
        C = self.C
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
                dist = geom.vnorm(geom.vsub(c, p_now)) if p_now else float("nan")
            if not (C.GATE_D_MIN <= dist <= C.GATE_D_MAX):
                return None, "dist"
            n = st.get("n")
            if n is not None:
                n = geom.vunit(tuple(float(v) for v in n[:3]))
                nsd = st.get("n_sd_deg")
                if isinstance(nsd, (int, float)) and nsd > C.GATE_N_SD_MAX_DEG:
                    return None, "n_sd"
                if p_now is not None:
                    los = geom.vsub(p_now, c)
                    if geom.vdot(los, n) < 0:
                        n = geom.vmul(n, -1.0)      # 约定: 指向进近侧
                    if geom.ang_deg(los, n) > C.GATE_VIEW_ANG_MAX:
                        return None, "view"
            elif C.GATE_REQUIRE_NORMAL:
                return None, "no_n"
            # σ 拆到靶面 (§1): 进入方向 ≈ −n, 没有法向就用视线方向 (和规划一致)
            d_in = geom.hunit(geom.vmul(n, -1.0)) if n is not None else None
            if d_in is None and p_now is not None:
                d_in = geom.hunit(geom.vsub(c, p_now))
            sd = st.get("sd_cm") or []
            cov_c = st.get("ring_cov") if raw else st.get("cov")
            sig = geom.sigma_hva(d_in, cov_c, None if raw else sd) if d_in is not None else None
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
                if geom.vnorm(geom.vsub(aim, c)) > C.R_IN - 0.05:
                    aim = None                     # 红点跑到环外 = 红点识别有问题, 按环心瞄
                    self.n_aim_far += 1
            return {"t": t_cap, "recv": now, "c": c, "aim": aim, "n": n, "dist": float(dist),
                    "sd": sd_lat, "sig": sig, "n_sd": st.get("n_sd_deg"),
                    "raw": raw, "src": "ring_" + str(st.get("ring_src")) if raw else st.get("src"),
                    "p": p_now}, "ok"
        except Exception:
            return None, "bad"
