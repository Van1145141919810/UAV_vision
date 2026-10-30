# -*- coding: utf-8 -*-
"""
vb09/common/target.py —— 两份固定契约: Target (一个候选) / Lock (前段交给后段)
===============================================================================
PLAN_refactor_v2 §2.2。感知插件产出 Target, 飞行步骤之间传 Lock; 步骤里不出现 `if kind == ...`。

Target (一个候选)
  id, kind, pos_w[3], cov_w[9], n[3]|None, n_sd_deg|None, size|None, score, src, t, quality{}
  环 = 1 个候选 (pos_w = 环心, 另带 aim = 红点或环心, n, 内外径)
  以后: ArUco = 1 个 (带 n、边长); 红点 = 多个 (每条世界系轨迹一个)
  ★ 本轮 /gate/status 的字段结构不变 (网页和 judge 都依赖它), Target 先只定义, 红点那一轮再接。

Lock (前段 → 后段)
  字段语义和原来的 GateRun.acq / GateRun.locked (= gate_plan.fuse_samples 的结果) 完全一致:
    c, aim, n, k, spread_cm, spread_hva, aim_spread_hva, sig_hva, n_spread_deg, raw, sd_hva_cm, ...
    锁定后另有 t_lock, locked
  所以 Lock 是 dict (老代码 / 遥测 / gate.csv 全按 dict 读); kind / trk_id 是属性, 不进 dict
  (不改变遥测里任何字段)。plan_dash(aim, c, n, …) 的签名不变。
"""


class Target(dict):
    """一个候选。见文件头。字段按 dict 存, 常用的给属性。"""
    FIELDS = ("id", "kind", "pos_w", "cov_w", "n", "n_sd_deg", "size", "score", "src", "t", "quality")

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        for k in self.FIELDS:
            self.setdefault(k, {} if k == "quality" else None)

    def __getattr__(self, k):
        if k in self.FIELDS:
            return self.get(k)
        raise AttributeError(k)


class Lock(dict):
    """
    前段交给后段的目标估计。dict 内容 = fuse_samples 的结果 (+ t_lock / locked);
    kind (目标类型) / trk_id (红点轨迹号, 环没有) / size (环内径等, 可选) 是属性。
    """

    def __init__(self, d=None, kind="ring", trk_id=None, size=None):
        super().__init__(d or {})
        self.kind = kind
        self.trk_id = trk_id
        self.size = size

    @property
    def c(self):
        return self.get("c")

    @property
    def aim(self):
        return self.get("aim") or self.get("c")

    @property
    def n(self):
        return self.get("n")

    @property
    def sig_hva(self):
        return self.get("sig_hva")

    @property
    def k(self):
        return self.get("k")

    @property
    def t_lock(self):
        return self.get("t_lock")
