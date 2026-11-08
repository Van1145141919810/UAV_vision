# -*- coding: utf-8 -*-
"""
vb09/perception/reddot/judge.py —— 飞行脚本侧: 一条 /gate/status 能不能当红点样本用
===============================================================================
接口同 ring/judge.py: Judge(C); judge(st, now, p_now) → (样本 | None, 原因); n_aim_far。
没有环: c = aim = 红点中心 (target)。门限先用 flight/config 里已有的 GATE_*; 红点专用的 DOT_* 用 getattr
带默认值 (还没挪进 flight/config)。numpy / geom only, 不拉 cv2。
"""
from vb09.common import geom


class RedDotJudge(object):
    kind = "reddot"
    # 2026-11-08 (PLAN_dot_v2): 桥里有诚实的世界系后验 + 选点 → 飞行用 modes.MODES_TRACKED (边飞边估的步骤链)。
    #   单帧判定只卡"能不能当样本" (发现级: 远到 DOT_D_MAX、σ 宽); 够不够准由步骤按 σ 判 (Approach / Refine)
    TRACKED = True
    # 环的 judge 没有的拒绝原因 (flight/target_in.py 合进计数和日志说明)
    # 目标 × 模式支持表 (2026-11-06; 档案校验读)。冲刺 / 影子的规划现在还按环的几何 (瞄点按内环半径收,
    #   纸面 = 环平面), 红点的"可穿区域 / 法向来源"接口没做之前标实验性
    MODES = {
        "dock": "ok",
        "shadow": ("experimental", "原地规划 (不抵近), 只看计划不飞"),
        "dash": ("experimental", "抵近 + 持续估计 + 冲刺还没真机飞过 —— 先用停靠验证精度"),
    }
    REASONS = ("stale", "src", "score", "hits", "ambig")
    REJ_TXT = {"stale": "估计太旧", "src": "不是红点来源", "score": "分数低", "hits": "命中太少",
               "ambig": "多个红点分不清"}

    def __init__(self, C):
        self.C = C
        self.n_aim_far = 0                    # 接口占位 (没有环, 永远 0)

    def _g(self, name, default):
        return getattr(self.C, name, default)

    def judge(self, st, now, p_now):
        C = self.C
        try:
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
            age = st.get("age_ms")
            if isinstance(age, (int, float)) and age > self._g("DOT_MAX_AGE_MS", 600):
                return None, "stale"
            if not str(st.get("src") or "").startswith("dot_"):
                return None, "src"
            dist = st.get("dist")
            if not isinstance(dist, (int, float)):
                dist = geom.vnorm(geom.vsub(c, p_now)) if p_now else float("nan")
            if not (C.GATE_D_MIN <= dist <= self._g("DOT_D_MAX", 14.0)):     # 发现级: 远处也要 (抵近靠它)
                return None, "dist"
            # 选了点 (桥只发布选中的那条): 是人 / 飞行脚本选的, 不再按分数 / 命中数 / 多候选拒绝
            sel = st.get("sel") or {}
            chosen = sel.get("state") == "bound"
            if not chosen and float(st.get("score", 1.0)) < self._g("DOT_SCORE_MIN", 0.5):
                return None, "score"
            if not chosen and int(st.get("n_hit", 99)) < self._g("DOT_HITS_MIN", 3):
                return None, "hits"
            # 候选多且接近: 不确定是哪个 → 不当样本
            others = [] if chosen else \
                [x for x in (st.get("cands") or []) if int(x.get("id", -1)) != int(st.get("trk_id", -2))]
            for x in others:
                if float(x.get("score", 0.0)) >= 0.7 * float(st.get("score", 1.0)) and x.get("aim"):
                    if geom.vnorm(geom.vsub(tuple(x["aim"][:3]), c)) > 0.5:
                        return None, "ambig"
            # 法向是多视角才有的附加量 (locate.normal_consensus): 不可信就只丢法向, 位置样本照用
            #   (环的 judge 是整帧拒绝; 红点的位置和法向来源不同, 不该连坐)
            n = st.get("n")
            if n is not None:
                n = geom.vunit(tuple(float(v) for v in n[:3]))
                nsd = st.get("n_sd_deg")
                if isinstance(nsd, (int, float)) and nsd > C.GATE_N_SD_MAX_DEG:
                    n = None
            if n is not None and p_now is not None:
                los = geom.vsub(p_now, c)
                if geom.vdot(los, n) < 0:
                    n = geom.vmul(n, -1.0)
                if geom.ang_deg(los, n) > C.GATE_VIEW_ANG_MAX:
                    return None, "view"
            if n is None and C.GATE_REQUIRE_NORMAL:
                return None, "no_n"
            d_in = geom.hunit(geom.vmul(n, -1.0)) if n is not None else None
            if d_in is None and p_now is not None:
                d_in = geom.hunit(geom.vsub(c, p_now))
            sd = st.get("sd_cm") or []
            sig = geom.sigma_hva(d_in, st.get("cov"), sd) if d_in is not None else None
            if sig is None:
                sd_max = max(float(v) for v in sd) if sd else float("nan")
                sig = (sd_max, sd_max, sd_max)
            # σ 门限是发现级的 (远处深度 σ 几十 cm 也要用来抵近); 锁定线在 Refine (GATE_LOCK_SD_*)
            if not (max(sig[0], sig[1]) <= self._g("DOT_SD_MAX_CM", 40.0)):
                return None, "sd"
            if not (sig[2] <= self._g("DOT_SD_ALONG_MAX_CM", 300.0)):
                return None, "sd_along"
            return {"t": t_cap, "recv": now, "c": c, "aim": c, "n": n, "dist": float(dist),
                    "sd": max(sig[0], sig[1]), "sig": sig, "n_sd": st.get("n_sd_deg"),
                    "n_state": st.get("n_state"), "n_why": st.get("n_why"), "sel": sel or None,
                    "raw": False, "src": st.get("src"), "p": p_now}, "ok"
        except Exception:
            return None, "bad"
