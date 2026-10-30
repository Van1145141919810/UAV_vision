# -*- coding: utf-8 -*-
"""
vb09/flight/steps —— 飞行步骤 (可插拔)。组合成任务见 vb09/flight/modes.py。
  acquire    Acquire        悬停识别 → 粗估
  plan_dash  PlanDash       冲刺规划 (make_plan)
  reposition Reposition     摆位到起跑点 + 对正 (move_to / turn_to)
  align      Align / AlignHere   取样锁定 (冲刺档另做锁定后规划 + 微调)
  commit     Commit         等 G / Y, 停稳
  dash       Dash           冲刺 + 制动 (committed)
  shadow     Shadow         影子档: 锁定后规划, 发网页, 悬停 3s, 回手动段
"""
