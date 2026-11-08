# -*- coding: utf-8 -*-
"""
vb09/flight/mode_spec.py —— 任务模式的元数据 (纯数据, 不 import ROS / numpy)
===============================================================================
2026-11-06。dry-run、档案校验、网页 (/api/schema) 在 import ROS 之前就要知道
"这个模式由哪些步骤组成、用哪些档案旋钮、要不要按冲刺查", 所以放在这里, 不放在 modes.py
(modes.py 构造真正的步骤对象, 步骤文件 import rospy)。

三层, 都是显式表 (不扫描目录):
  SECTIONS      档案旋钮的"区"。旋钮表 (profile/gate.py GATE_KNOBS) 每项的 section 字段指向这里;
                没有 section = 公共旋钮 (现场 / 高度 / 水平 / 目标 / 模式), 所有模式都用
  STEP_USES     步骤类名 → 它读哪些区的参数。步骤之外的代码 (derive、预检、手动段) 也按区判断
  MODE_SPECS    模式键 → 中文名、步骤类名、说明、是否会冲刺、网页预览用哪种
  ⇒ 模式用到的区 = 它各步骤 STEP_USES 的并集 (sections(mode)), 不手写

加一个模式: 写步骤 (vb09/flight/steps/) → STEP_USES 登记 → MODE_SPECS 加一行 → modes.py 加一行。
网页表单、档案校验、预检会自动按区处理; tools/mode_isolation_check.py 查三者一致、
并查"不用的区的参数不影响这个模式"。

目标 × 模式: 不在这里写。目标插件的判定类 (perception/<kind>/judge.py) 用 MODES 声明支持哪些模式
(ok / experimental), 档案校验读它 (profile/gate.py _check_support)。
"""
from collections import OrderedDict

# 区: 键 → (中文名, 说明)。顺序 = 网页上的显示顺序
SECTIONS = OrderedDict([
    ("dash", ("冲刺", "冲刺速度、过纸、起跑、降速策略和冲刺细调")),
    ("dash_site", ("冲刺场地", "环后空间、起飞点后方、两侧 —— 冲刺规划查放不放得下")),
    ("body", ("机体", "机头 / 机尾 / 半宽 —— 冲刺规划查机身穿不穿得过、离墙多远")),
    ("dock", ("停靠", "停靠距离、悬停时间")),
    ("scan", ("上下扫描", "到位后上下换视角定法向 (环的停靠用; 红点在抵近 / 持续估计里自己换视角)")),
    ("track", ("抵近 / 持续估计", "红点: 边飞边估的抵近、不够准时左右 / 上下换视角、等 G 时估计变了就重锁")),
])

# 步骤类名 → 用到的区。新步骤必须登记 (mode_isolation_check 会查 modes.py 里的每个步骤都在这里)
STEP_USES = {
    "Acquire": (),
    "PlanDash": ("dash", "dash_site", "body"),
    "Reposition": ("dash",),
    "Align": ("dash",),
    "Commit": ("dash",),
    "Dash": ("dash", "body"),          # 刹停判断用机尾长 L_R
    "AlignHere": ("dash",),
    "Shadow": ("dash", "dash_site", "body"),   # 影子在原地再规划一次冲刺 (plan_dash), 计划发网页
    "StandOff": ("dock",),
    "VScan": ("scan",),
    "Hold": ("dock",),
    # 跟踪类目标 (红点) 的步骤 (steps/track.py, 2026-11-08)
    "Detect": ("track",),
    "Approach": ("track",),            # 基类 (ApproachDock / ApproachDash 只给停靠距离)
    "ApproachDock": ("dock", "track"),
    "ApproachDash": ("dash", "track"),
    "Refine": ("track",),
    "PlanLocked": ("dash", "dash_site", "body"),
    "CommitTrack": ("dash", "track"),
}

# 步骤类名 → 它飞行时的最快指令速度来自哪个参数 (没登记 = 只按摆位速度 GATE_REPO_V 飞)。
# derive() 用它定这个模式的 gate 阶段速度硬限: 只有真的要冲的模式才按冲刺速度放宽。
STEP_FAST_V = {
    "Dash": "DASH_V",
}

# 模式。label 是档案 mode 旋钮的中文选项; drives = 这一趟会不会真的冲 (网页开关标签用);
# preview = 网页右侧核对表 / 俯视图用哪种 (web/views/config.js PREVIEWS)
# steps_tracked = 判定插件 TRACKED = True 的目标 (红点) 用的步骤链 (modes.MODES_TRACKED)
MODE_SPECS = OrderedDict([
    ("dash", dict(label="冲刺", preview="dash", drives=True,
                  steps=("Acquire", "PlanDash", "Reposition", "Align", "Commit", "Dash"),
                  steps_tracked=("Detect", "ApproachDash", "Refine", "PlanLocked", "CommitTrack", "Dash"),
                  desc="识别 → 锁定 → 摆位到起跑点 → 冲刺穿过目标 → 刹停")),
    ("shadow", dict(label="影子", preview="dash", drives=False,
                    steps=("Acquire", "PlanDash", "AlignHere", "Shadow"),
                    steps_tracked=("Detect", "PlanDash", "AlignHere", "Shadow"),
                    desc="识别、锁定、冲刺规划照做, 不摆位、不冲刺, 计划发网页; 做完回手动段")),
    ("dock", dict(label="停靠", preview="dock", drives=False,
                  steps=("Acquire", "StandOff", "VScan", "Hold"),
                  steps_tracked=("Detect", "ApproachDock", "Refine", "Hold"),
                  desc="识别 → 飞到目标前停靠距离处对正 → (上下扫描) → 悬停记录检测 → 回手动段; 不冲刺")),
])

DEFAULT_MODE = "dash"
LABEL_TO_MODE = {v["label"]: k for k, v in MODE_SPECS.items()}


def sections(mode):
    """模式用到的区 (集合)。未知模式 → 所有区 (保守: 什么都查)。"""
    spec = MODE_SPECS.get(mode)
    if spec is None:
        return set(SECTIONS)
    out = set()
    for s in tuple(spec["steps"]) + tuple(spec.get("steps_tracked", ())):     # 两套步骤的并集 (保守)
        out.update(STEP_USES.get(s, SECTIONS.keys()))
    return out


def fast_v_params(mode):
    """模式里会用到的"冲刺级"速度参数名 (derive 定速度硬限用)。"""
    spec = MODE_SPECS.get(mode)
    steps = spec["steps"] if spec else STEP_FAST_V.keys()
    return [STEP_FAST_V[s] for s in steps if s in STEP_FAST_V]


def uses(mode, section):
    return section in sections(mode)


def mode_of(knob_mode=None, shadow=False, default=DEFAULT_MODE):
    """档案 mode 旋钮 (中文) + 旧写法 GATE_SHADOW → 模式键。和 config.derive() 的规则一致: GATE_SHADOW 优先。"""
    if shadow:
        return "shadow"
    if knob_mode:
        return LABEL_TO_MODE.get(knob_mode, knob_mode)
    return default


def schema():
    """给网页 /api/schema: 模式表 + 区表 (全是可 JSON 的基本类型)。"""
    return {
        "modes": [dict(key=k, label=v["label"], desc=v["desc"], drives=v["drives"], preview=v["preview"],
                       steps=list(v["steps"]), sections=sorted(sections(k), key=list(SECTIONS).index))
                  for k, v in MODE_SPECS.items()],
        "sections": [dict(key=k, label=v[0], desc=v[1]) for k, v in SECTIONS.items()],
        "default_mode": DEFAULT_MODE,
    }
