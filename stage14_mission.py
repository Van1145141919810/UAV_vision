#!/usr/bin/env python3
"""
stage14_mission.py —— 实飞入口 (原 stage12_gate.py, 文件头曾写 stage13; 2026-10-30 改名, 见 PLAN_refactor_v2)
===============================================================================
2026-10-30 模块化之后这个文件只做: 档案 setup → C.derive() → finalize (--dry-run 在这里退出) →
终端检查 → import vb09.flight (ROS) → main()。
  参数        vb09/flight/config.py (C.名字)        平台      vb09/flight/{ctx,telemetry,setpoint,keys,
  任务模式    档案 mode = 冲刺 / 影子 → vb09/flight/modes.py          log,safety,preflight,phases,runner}.py
  步骤        vb09/flight/steps/*.py (Acquire / PlanDash / Reposition / Align / Commit / Dash / Shadow)
  目标判定    vb09/perception/ring/judge.py (JUDGES["ring"])
下面是原来的说明 (流程 / 键盘 / 安全分级都没变)。

穿环 (撞环) 实飞脚本 v1: 悬停定位 → 冻结 → 按预定轨迹冲刺 → 原地降落
-------------------------------------------------------------------------------
方案: 项目文档 claude/PLAN_gate_v1.md (2026-10-19 讨论后简化, 见下面"和方案的差别")
接口: 项目文档 claude/INTERFACE_gate.md (遥测 gate 段 / 配置 derived.gate / gate.csv)
规划: 同目录 gate_plan.py (纯函数, 可离线跑)

启动 (档案与参数见 flight_profile.py; 档案里写 task = 穿环)
  python3 stage14_mission.py --site <档案> --dry-run   只打印生效包络, 不需要 ROS
  python3 stage14_mission.py --site <档案>             真飞: 必须在**普通** SSH 终端里

文件名约定 (2026-10-19): stage<版本号>_<任务>.py —— 编号是版本, 后缀是任务
  (_gate 穿环 / _track 停靠跟踪; 老名字 stage11_real.py 也算跟踪)。flight_monitor 按任务各取编号最大的。

前提
  视觉链路: python3 run_vision.py gate_gcs (地面检测) 或 run_vision.py gate --drive (机上检测) —— 预检会查
  PX4: MPC_TILTMAX_AIR ≤ MAX_TILT_DEG − 5 (=25°), MPC_XY_VEL_MAX ≥ 1.1×DASH_V —— 预检会查

ROS 接口
  节点   /flight_ctrl   (和 stage11 同名, 不能同时跑)
  订阅   /gate/status   String JSON (vision_bridge 每帧一条: target=环心, 可选 n / aim / n_sd_deg)
         /vision_bridge/cam  String JSON (图传模式, 预检用)
         /mavros/...  /vins_estimator/odometry  (同 stage11)
  发布   /mavros/setpoint_raw/local   PositionTarget  SP_PUB_HZ (冲刺段带加速度前馈)
         /flight_ctrl/status      JSON 2Hz   (同 stage11 + task)
         /flight_ctrl/telemetry   JSON 5Hz, 提交/冲刺/制动段 20Hz  (+ task, schema, gate 段)
         /flight_ctrl/config      JSON latched (+ task, derived.gate, derived.speed_chain)

飞行流程 (phase 全程 = "gate", 段名在遥测 gate.seg)
  preflight → prime → OFFBOARD → arm → takeoff
    → MANUAL   (manual) 手动段 (GATE_MANUAL_START): 键和 stage11 一样, 把飞机开到想起跑的地方,
                        按 T 开始下面这一次。GATE_MANUAL_TIMEOUT 一个键都没按 → 降落
    → ACQUIRE  (wait)   原地悬停, 机头慢慢转向环, 收 HOVER_ACQ 秒有效识别; 8s 看不到 → ±30° 扫一次
    → PLAN     (wait)   gate_plan.plan_dash: 进入方向 / 起跑点 S / 制动终点 E / 撞纸速度; 不可行先降速
                        起跑点动态: 离环在 [最小助跑, D_RUN_MAX] 之间就地起跑, 近了后退、远了前进
    → REPOSITION(align) 直线慢飞到 S (≤ GATE_REPO_V), 全程在环前 1.5m 以外, 机头盯环
    → ALIGN    (align)  在 S 悬停 HOVER_ALIGN 秒取样 → 收敛判定 → **锁定** → 用锁定值重新规划
    → COMMIT   (commit) 打印计划, 等操作员按 G (GATE_CONFIRM_DASH)。之后不能中止
    → DASH / BRAKE      按时间表执行轨迹 (位置+速度+加速度前馈, 全程 P 模式), 不看识别
    → HOLD     (hold)   以**当前位置**重锚悬停 GATE_HOLD_S → 原地缓降 → AUTO.LAND
  冲刺前任何一步放弃 (锁不住 / 不可行 / G 超时 / 找不到环) 或按 M / 方向键 → 回 MANUAL, 可以再 T
    (GATE_ABORT_TO_MANUAL; LAND 请求 / 接管 / 手动段超时照常降落)
  影子 GATE_SHADOW=True: ACQUIRE → PLAN → 在原地做 ALIGN 锁定 → 发计划给网页 → 不摆位不冲刺 → 回 MANUAL
  虚拟环 GATE_VIRTUAL_OFFSET=d: 把环平面往进近侧挪 d 米, 真纸面当墙 → 真冲刺但停在纸前 (T3)

键盘
  手动段   W前 S后 A左 D右 | R升 F降 | Q左转 E右转 | 空格悬停 | T = 开始找环/规划
  冲刺前   M 或任一方向键 = 回手动段   L = 就地降落   X = 停桨   G = 确认冲刺 (只在 COMMIT 段有效)
  冲刺中   X = 停桨 (立刻)   L / SSH 断连 = 记下, **刹停之后**再降落 (冲刺中缓降比刹完更危险)

安全分级 (沿用 stage11)
  FATAL (空中停桨)   倾角 > MAX_TILT_DEG(30°) 持续 FATAL_CONFIRM_S / MAVROS 断链 / 按 X
                    按 G 后到刹停悬停结束: 倾角 > GATE_DASH_TILT_DEG(45°) 持续 0.3s (急刹 + 撞纸)
  LAND  (缓降)       围栏、VIO 老化/跳变、视觉断供、速度超限、低电压、SSH 断连 …
                    —— 冲刺/制动段收到的 LAND **推迟到刹停后**执行 (见上)
  卡纸              冲刺中 (过纸面前) 实测速度比规划低 > GATE_STUCK_DV 且落后规划 > GATE_STUCK_DS
                    持续 GATE_STUCK_HOLD → 设定点改成当前位置 (停止前推) → 原地降落
  接管 / 脚本异常    同 stage11 (phase_handover / cleanup_decision)

和方案 PLAN_gate_v1.md 的差别 (2026-10-19 定的"先简单方案")
  · 全程 P 模式 (盲区不切速度模式), 不做 vision_gate (VINS 照常喂 EKF)
  · 不做 kill_watchdog / 姿态模式 (L2) / D-att: 倾角停桨沿用 stage11 的 safety_monitor,
    只是按 G 之后到刹停悬停结束换成 GATE_DASH_TILT_DEG (默认 45°/0.3s)
  · 刹车起点 (2026-10-21 改): 机头碰纸后再冲 PASS_RUN 才刹, s_b = −L_F + PASS_RUN + v·T_LAG
    (滞后补偿); 停止点要离环后墙 ≥ M_BACK, 放不下就降速
  · 助跑距离 ≥ D_ALIGN_MIN (起跑点要看得到整个环)
  · 穿环后不返航: 刹停 → 原地降落

2026-10-19 首飞后的修改 (项目文档 claude/ISSUE_gate_run_2026-10-19.md)
  · §1 识别 σ / 环心离散都按靶面拆成 横 h / 竖 v / 深度 分开判 (双目深度 σ ∝ 距离²)
  · §2 预检: 桥里的环估计 age_ms > 1s (旧状态) → 拒绝起飞 (新版 vision_bridge 会自己重置)
  · §3 预检: PX4 参数读不到 → 先 param/pull 等缓存, 还读不到就拒绝起飞; 读到的值写进 site_config.txt
  · §5 cleanup: 落地后停在 AUTO.LAND 就不动了 (不再去切 MANUAL / LOITER)
  · 手动段 + T 开始、冲刺前放弃回手动段、动态起跑点、冲刺段停桨线 45°

日志  ~/uav_logs/flight_NNNN/   (编号递增, 不用日期: 飞机每次重启时钟都会重置; 目录里有 run_info.txt)
  flight_log.csv (phase=gate)   gate.csv (穿环段每拍一行, 列见 init_log_dir)
  vins_raw.csv  ekf2_health.csv  timing.csv  z_drop_events.log  site_config.txt  events.log
===============================================================================
"""

import os
import sys

# numpy / rospy / mavros 消息在"现场档案校验"之后才 import (搜 "ROS 相关 import"),
# 所以 --dry-run 不需要 ROS。CONFIG 和派生量区不许用 ROS 的东西。


# =============================================================================
# ############################   CONFIG   ######################################
# =============================================================================
# 2026-10-30: 全部参数搬到 vb09/flight/config.py, 统一写 C.名字。现场请用 `--site <档案>`。
from vb09.flight import config as C  # noqa: E402

# =============================================================================
# ####################   现场档案（必须在派生量之前套用!）   ##################
# =============================================================================
# 下面"派生量"区是用上面的参数算出来的。档案必须在这里 (module 级、派生量之前)
# 写进全局, 否则派生量拿的是旧值。两段式:
#   setup()    在这里: 解析档案 + --set, 写进全局, 只拦"读不懂"的错误
#   finalize() 在派生量之后: 对最终生效值做全部一致性 + 安全检查; --dry-run 在那里退出
_SITE_REPORT = None
_ORIG_ARGV = list(getattr(sys, "argv", []))   # setup() 会改 sys.argv, 先留一份给提示用
try:
    import flight_profile as _fp
except ImportError:
    _fp = None
    # 没有 flight_profile 时 --dry-run 会被无视 → 本来只想看一眼却真飞了。
    # 所以只要带了任何 "-" 开头的参数就拒绝运行。
    if any(a.startswith("-") for a in _ORIG_ARGV[1:]):
        print("[site] [!] 命令行带了 %s, 但没找到 flight_profile.py "
              "—— 拒绝运行。把 flight_profile.py 和本文件放同一目录。"
              % " ".join(a for a in _ORIG_ARGV[1:] if a.startswith("-")))
        sys.exit(1)
    print("[site] [!] 没找到 flight_profile.py —— 本次用**代码默认值**飞, "
          "且**没有**任何参数校验。把 flight_profile.py 和本文件放同一目录即可。")
if _fp is not None:
    _SITE_REPORT = _fp.setup(sys.argv, vars(C))

# =============================================================================
# ##########################   派生量 (不要手改)   ###########################
# =============================================================================
C.derive()

# =============================================================================
# ##############   现场档案第二段: 对最终生效值做全部校验   #################
# =============================================================================
# 必须在派生量**之后**: 这里检查的是最终值(代码默认 + 档案 + --set)和派生量
# (PHASE_ENVELOPE / 停靠壳 / 可用带)。不过 → 拒绝起飞; --dry-run → 打印后退出。
if _fp is not None:
    _SITE_REPORT = _fp.finalize(_SITE_REPORT, vars(C))


# =============================================================================
# ##################   终端检查: 只能在普通 SSH 会话里飞   ###################
# =============================================================================
# 放在 finalize 之后: --dry-run 已经在上面退出了, 所以网页预览(子进程)和
# 在 tmux 里做 dry-run 都不受影响 —— 只拦"真飞"。
#
# 为什么: SSH 断连保护靠的是"断连后 stdin 读到 EOF → 立即缓降"(见 KeyReader)。
#   tmux / screen / VSCode 内置终端在 SSH 断开后**继续替你保持伪终端**,
#   stdin 永远不会 EOF:
#     · 穿环段的识别/摆位/等 G 都会一直等下去, 直到各自超时
#     · RC 的 SWC/SWD 是坏的 —— 键盘是唯一人工急停, 断了就只剩 QGC
#   stdin 不是终端(nohup / 后台 / 网页启动)更糟: X 急停不存在, 也按不了 G。
#   所以这几种情况一律拒绝起飞。VINS / mavros / flight_monitor 照常放 tmux 里。
def _terminal_problems():
    probs = []
    if os.environ.get("TMUX"):
        probs.append("在 tmux 里运行")
    if os.environ.get("STY"):
        probs.append("在 screen 里运行")
    if os.environ.get("TERM_PROGRAM") == "vscode" or os.environ.get("VSCODE_IPC_HOOK_CLI"):
        probs.append("在 VSCode 内置终端里运行")
    try:
        if not sys.stdin.isatty():
            probs.append("stdin 不是终端 (后台/nohup/重定向启动)")
    except Exception:
        probs.append("stdin 不可用")
    return probs


if __name__ == "__main__":
    _probs = _terminal_problems()
    if _probs:
        print("\n" + "!" * 78)
        print("拒绝起飞: 检测到 %s" % "、".join(_probs))
        print("  这样运行时 SSH 断开不会让脚本收到 EOF, 断连自动降落保护失效;")
        print("  RC 急停开关是坏的, 键盘是唯一人工急停通道。")
        print("")
        print("  请在笔记本上新开一个**普通**终端窗口(PowerShell / Windows Terminal):")
        print("      ssh visbot@<飞机IP>")
        print("      cd %s" % os.path.dirname(os.path.abspath(__file__)))
        print("      python3 %s %s" % (os.path.basename(__file__), " ".join(_ORIG_ARGV[1:])))
        print("  不要套 tmux, 不要用 VSCode 内置终端。")
        print("  (--dry-run 不受此限制, 在哪里跑都行)")
        print("!" * 78 + "\n")
        sys.exit(1)


# =============================================================================
# ##########################   ROS 相关 import   #############################
# =============================================================================
# 放在校验之后: --dry-run 到不了这里, 所以 dry-run 不需要 ROS。
# 2026-10-30: 平台层搬到 vb09/flight/ (ctx / telemetry / setpoint / keys / log / safety / preflight /
#   phases / target_in / mission / runner), 共享状态在 vb09/flight/ctx.py 的单例 F 里。
import rospy  # noqa: E402

from vb09.flight.ctx import F, bind_defaults  # noqa: E402
from vb09.flight.runner import main  # noqa: E402,F401

F.site_report = _SITE_REPORT
F.fp = _fp
bind_defaults()


if __name__ == "__main__":
    _rc = 1
    try:
        _rc = main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        # preflight 阶段 (还没发任何设定点、没切模式) 按的 Ctrl+C 会到这里
        rospy.logwarn("Interrupted before flight sequence")
        _rc = 1
    finally:
        # disable_signals=True 后 rospy 不会自己关, 退出前手动关
        try:
            rospy.signal_shutdown("flight script exit")
        except Exception:
            pass
    sys.exit(_rc)

