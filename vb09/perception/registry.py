# -*- coding: utf-8 -*-
"""
vb09/perception/registry.py —— 感知插件注册表 (显式字典, 不自动扫描目录)
===============================================================================
2026-11-01 接线: 每种目标 (kind) 在下面每张表里各占一行, 值是 "模块:名字" 字符串,
**用到时才 import** (load)。这样飞行脚本 / 机上融合只加载自己那一份, 不会被迫拉起 cv2 和整个检测库。

  表          谁用                                        接口 (环的实现)
  CODECS      地面发 / 飞机收 (vb09/common/wire.py)        模块: MSG_VERSION, frame_to_msg(fr), frame_from_msg(m), LIB_FILES
                                                           (vb09/perception/ring/wire.py)
  CONFIGS     gcs_gate / vision_bridge                     dataclass, 有 load(path) 和 lost_timeout   (ring/config.py: GateConfig)
  DETECTORS   gcs_gate (地面) / vision_bridge gate 模式     process(L, R, t, T_WC=None) → frame; reset(); draw(img, fr, T_WC, text);
                                                           extra(fr, T_WC, warn) (机上检测时的 /gate/status 附加字段)
                                                           (ring/plugin.py: RingDetector)
  LOCATORS    vision_bridge gate_gcs 模式 (机上融合)       Locator(cfg, f, R1); same_rig(f, R1); update(fr, T_WC, t);
                                                           extra(fr, T_WC, warn); f    (ring/plugin.py: RingLocator)
  JUDGES      飞行脚本 vb09/flight/target_in.py            Judge(C); judge(st, now, p_now) → (样本 | None, 原因); n_aim_far
                                                           (ring/judge.py: RingJudge)

加一种目标 = 写 vb09/perception/<kind>/ 下的这几样 + 每张表加一行。选哪种:
  地面 gcs_gate.py --target <kind>   机上 vision_bridge _target:=<kind> (run_vision.py --target)   飞行脚本 C.TARGET_KIND
"""
import importlib

CODECS = {
    "ring": "vb09.perception.ring.wire",
}
CONFIGS = {
    "ring": "vb09.perception.ring.config:GateConfig",
}
DETECTORS = {
    "ring": "vb09.perception.ring.plugin:RingDetector",
}
LOCATORS = {
    "ring": "vb09.perception.ring.plugin:RingLocator",
}
JUDGES = {
    "ring": "vb09.perception.ring.judge:RingJudge",
}

_TABLE_NAME = {id(CODECS): "CODECS", id(CONFIGS): "CONFIGS", id(DETECTORS): "DETECTORS",
               id(LOCATORS): "LOCATORS", id(JUDGES): "JUDGES"}


def kinds(table=JUDGES):
    return sorted(table)


def load(table, kind):
    """→ 表里 kind 那一行指向的模块 (值没有 ":") 或对象 (值是 "模块:名字")。没有这一行 → ValueError。"""
    ref = table.get(kind)
    if ref is None:
        raise ValueError("未知目标类型 %r (%s 里只有: %s)" % (kind, _TABLE_NAME.get(id(table), "?"),
                                                       ", ".join(sorted(table)) or "无"))
    mod, _, name = ref.partition(":")
    m = importlib.import_module(mod)
    return getattr(m, name) if name else m


def load_config(kind, path):
    """→ (配置对象, 来源说明)。path 存在就 load(path), 否则代码默认值。"""
    import os
    cls = load(CONFIGS, kind)
    if path and os.path.exists(path):
        return cls.load(path), path
    return cls(), None
