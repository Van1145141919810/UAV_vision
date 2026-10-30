# -*- coding: utf-8 -*-
"""
vb09/flight/steps/base.py —— 飞行步骤接口 (PLAN_refactor_v2 §2.3)
===============================================================================
  name        这一步设的遥测段名 (gate.seg); 规划之类不换段名的步骤沿用上一段的名字
  KNOBS       这一步用到的档案旋钮 (以后档案旋钮从这里自动收集; 本轮先手写在 flight_profile)
  vel_lim     "pre" = 冲刺前的速度包络 (GATE_PRE_VEL_LIM); None = 不改
  committed   True = 过了这一步就不能中止 (冲刺): 平台推迟 LAND 到刹停后, 换用 GATE_DASH_TILT_DEG 停桨线,
              不能再回手动段。这些是平台白名单属性 —— 步骤只能声明, 不能自己改
  check(params) → (errors, warnings, derived)   起飞前静态检查 (本轮还没用)
  run(ctx, lock) → lock                           放弃 raise StepFail, 回手动段 raise StepBack
步骤只能通过 ctx 下指令 (hold / tick / set_seg / yaw_step / fail / setpoint …), 只能通过 lock / ctx.gate_in 读目标。
步骤里不许出现 `if kind == ...`: 和目标类型有关的判断放进 Judge / Locator。
"""


class StepFail(Exception):
    """
    冲刺前的正常退出 (放弃 / LAND / 接管)。原因在 args[0]。(原 GateStop)
    retry=True  = 这一次不行 (锁不住 / 不可行 / G 超时 …), GATE_ABORT_TO_MANUAL 时回手动段重来
    retry=False = LAND 请求 / 接管 / 手动段超时: 不重来, 降落
    """

    def __init__(self, why, retry=True):
        super().__init__(why)
        self.retry = retry


class StepBack(StepFail):
    """操作员要回手动段 (冲刺前按 M 或任一方向键)。(原 GateBack)"""


class Step(object):
    name = ""
    KNOBS = ()
    vel_lim = None
    committed = False

    def check(self, params):
        return [], [], {}

    def run(self, ctx, lock):
        return lock

    def __repr__(self):
        return type(self).__name__
