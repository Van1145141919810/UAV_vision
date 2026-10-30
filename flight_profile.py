# -*- coding: utf-8 -*-
"""
flight_profile.py —— 现场档案（site profile）：把「现场环境」翻译成飞行限制
===============================================================================
它解决的问题
-------------------------------------------------------------------------------
换一个现场（目标贴在一楼 / 二楼 / 三楼、场地大 / 小、室内 / 室外），要动的
参数有 20 多个，散在 stage11_real.py 的 8 个组里。**现场手改代码 = 必错**
（2026-09-16 就出过一次：只改了 TRACK_Z_MAX，而真正那道门是
TARGET_Z_NOMINAL ± TARGET_Z_TOLERANCE，结果飞机整场不听使唤）。

所以这里只暴露 **11 个「人能感知」的旋钮**，其余全部由规则推导。
旋钮的完整定义（类型/单位/范围/说明）在下面的 KNOBS 表里 —— 它是**唯一来源**:
档案解析、范围校验、网页表单、写回 .conf 全部从这张表生成。

    site_name   名字（只用于打印）
    z_low       目标最低高度 m          ← z 门限带下边界
    z_high      目标最高高度 m          ← 上边界（同时决定高度钳位/安全硬顶）
    takeoff_h   起飞后先升到多高 m
    xy_radius   允许离起飞点多远 m      ← 水平软限（硬围栏 = 软限 + 余量）
    entry_dist  离目标多近才交给视觉 m  ← 进场门（比这远就停在 MANUAL）
    standoff    停在离目标多远 m        ← ★2026-10-09 新增
    detect_far  空中最远能检测到多远 m  ← ★2026-10-09 新增（换相机/换标记必改）
    mission_s   一次作业时长 s
    speed       慢 / 中 / 快            ← 速度档
    scene       地面 / 低空 / 高空 / 室内 ← 决定缓降类安全项 + 起降漂移限制

★ 2026-10-19 穿环 (stage12_gate.py; 文件名后缀 = 任务: _gate 穿环, _track / _real 停靠跟踪)
-------------------------------------------------------------------------------
任务由飞行脚本决定 (脚本全局 TASK; 没有 = 停靠跟踪)。档案加 `task = 穿环` 声明用途,
和脚本对不上 → 拒绝起飞; 给了本任务不用的旋钮 (旋钮表的 tasks 字段) → 也报错。
穿环用到的旋钮:
    共用        site_name scene takeoff_h xy_radius(= 起跑点/制动终点的水平范围)
                z_low z_high (= 瞄点允许的高度带)
    穿环        dash_v run_max  d_back d_rear d_side (场地, 卷尺量)  l_f l_r w_half (机体, 实测)
    高级 (adv)  dash_acc brake_acc t_lag e_along v_tear_min entry_ang_max d_straight
检查 (_check_gate) 和派生量 (derived.gate / speed_chain) 用同目录 gate_plan.py 算,
和飞行时的规划是同一份代码。停靠跟踪 (stage11) 的检查和输出没有改 (_check_track)。

设计原则（每条都有理由）
-------------------------------------------------------------------------------
1. **未知键必须硬报错**，不能静默忽略 —— "z_hig = 9" 拼错却照样起飞，是最危险的。
2. **安全项按"后果"区分方向**（2026-10-09 修正）:
     · 触发 LAND(缓降) 的判据 → 只准收紧（更早缓降 = 更保守）
     · 触发 FATAL(空中停桨) 的判据 → **不准随场景收紧**。阈值越紧越容易
       空中停桨, 在高空那就是从 10m 摔下来。旧版"高空 MAX_TILT=20°"正好反了。
3. **校验不过就拒绝起飞**（fail-closed）：宁可现场尴尬 3 分钟，不要空中出事。
4. **两段式**（2026-10-09 修正）:
     setup()    在派生量**之前**: 解析档案 + --set, 写进全局
     finalize() 在派生量**之后**: 对**最终生效值**做全部校验, dry-run 在这里输出
   旧版只校验"覆盖字典", 于是 --set 能绕过一致性检查、不给 --site 时代码默认值
   从来没被校验过、dry-run 看不到派生量。现在不管值来自哪里, 都过同一道检查。
5. **纯标准库、不 import ROS** → 能在笔记本上离线单测。
6. **生效值写进飞行日志目录**（site_config.txt）→ 复盘时知道当时是什么限制。

档案文件格式（`~/flight_sites/<名字>.conf`）
-------------------------------------------------------------------------------
    # 注释随便写
    site_name  = 二楼玻璃破窗
    z_low      = 0.7
    z_high     = 9.0
    takeoff_h  = 1.5
    xy_radius  = 8.0
    entry_dist = 3.0
    standoff   = 1.2
    detect_far = 4.0
    mission_s  = 150
    speed      = 中
    scene      = 高空

    # ★ 2026-10-09(2): 原始参数覆盖也可以写进档案, 和命令行 --set 完全等价,
    #   照样过全部校验。这样"档案 = 完整配置", 启动命令永远只有 --site 名字,
    #   不会因为少复制一段 --set 而悄悄变成另一套配置(比如影子模式丢了)。
    set.TRACK_SHADOW    = True
    set.TRACK_LEASH_MAX = 0.8

命令行（跟在 stage11_real.py 后面）
-------------------------------------------------------------------------------
    --site 二楼            用 ~/flight_sites/二楼.conf
    --set KEY=VALUE        单次临时覆盖参数（不落盘; 只允许大写的数值/开关参数;
                           安全项照样受限; 优先级高于档案里的 set.KEY）
    --dry-run              只打印生效包络和校验结果就退出（不需要 ROS/roscore）
    --json                 配合 --dry-run: 输出机器可读 JSON（给网页用）
    --sites-dir DIR        档案目录（默认 ~/flight_sites）

    ★ 2026-10-09(2): 任何以 "-" 开头、却不是上面五个的参数 → 直接报错拒绝运行。
      旧版会把它原样交给 ROS(ROS 不认也不报错), 于是 `--dryrun` 拼错 = 真起飞。
      ROS 自己的参数都是 `name:=value` 形式, 不以 "-" 开头, 不受影响。
      --site / --set / --sites-dir 后面缺值同样报错(旧版会静默用默认值飞)。

单独运行
-------------------------------------------------------------------------------
    python3 flight_profile.py               列出现有档案
    python3 flight_profile.py 二楼          只做旋钮级检查（完整检查请用
                                            stage11_real.py --site 二楼 --dry-run）
    python3 flight_profile.py --schema      输出旋钮表 JSON（网页表单用）
===============================================================================
"""

import json
import math
import os
import re
import sys

DEFAULT_SITES_DIR = os.path.expanduser("~/flight_sites")

# =============================================================================
# 旋钮表 —— 唯一来源
# =============================================================================
# 字段:
#   key       旋钮名（档案里写的名字）
#   type      "str" / "float" / "choice"
#   unit      单位（显示用）
#   label     中文名（表单标题）
#   help      一句话说明（表单提示 + 写回 .conf 时的行尾注释）
#   min/max   硬范围，越界 = 拒绝起飞
#   warn_lo / warn_hi   (阈值, 理由) —— 越过只警告
#   choices   choice 类型的可选值
#   group     表单分组
#   tasks     ★2026-10-19 适用的任务 ("track" 停靠跟踪 / "gate" 穿环)。没有 = 两种都用。
#             档案里给了本任务不用的旋钮 → 报错 (不静默忽略)。网页可按它隐藏无关旋钮。
#   adv       ★2026-10-19 细调项, 网页放进「高级」折叠区
#
# 任务由**飞行脚本**决定 (脚本全局 TASK: stage11 没有 = "track", stage12 = "gate")。
# 档案里的 task 旋钮只是声明"这份档案是给哪种任务写的", 和脚本对不上 → 拒绝起飞
# (防止拿停靠跟踪的档案去飞穿环, 或反过来)。
TASK_NAMES = {"跟踪": "track", "穿环": "gate"}
KNOBS = [
    dict(key="site_name", type="str", unit="", label="现场名称", group="现场",
         help="只用于打印和日志"),
    dict(key="task", type="choice", unit="", label="任务", group="现场",
         choices=["跟踪", "穿环"],
         help="这份档案给哪种任务用; 必须和飞行脚本一致 (stage11=跟踪, stage12=穿环)"),
    dict(key="scene", type="choice", unit="", label="场景", group="现场",
         choices=["室内", "地面", "低空", "高空"],
         help="决定缓降类安全项和起降阶段的水平漂移限制"),

    dict(key="z_low", type="float", unit="m", label="目标最低高度", group="高度",
         min=0.5, min_why="低于 0.5m 会进地效 + 桨风吹目标板，PX4 落地检测也可能误判",
         warn_lo=(0.7, "偏低(建议 ≥0.7)：桨风会吹到手/板，测量会变毛"),
         help="z 门限带下边界(相对起飞点)，低于此的目标整帧丢弃; 穿环: 瞄点允许的最低高度"),
    dict(key="z_high", type="float", unit="m", label="目标最高高度", group="高度",
         min=0.8, max=20.0,
         warn_hi=(10.0, "超过 10m：要重新评估 VIO 高度表现 / 电池 / 法规"),
         help="z 门限带上边界，同时决定高度钳位顶和安全硬顶(+1m); 穿环: 瞄点允许的最高高度"),
    dict(key="takeoff_h", type="float", unit="m", label="起飞高度", group="高度",
         min=0.5, min_why="0.5 以下起飞判定/离地检测不可靠", max=5.0,
         warn_hi=(2.0, "起飞段是位置阶跃(没改成斜坡)，PX4 会以最大爬升率去追。"
                       "建议 ≤2.0，再高用手动段 R 键爬升"),
         help="起飞后先升到的高度"),

    dict(key="xy_radius", type="float", unit="m", label="水平范围", group="水平",
         min=0.5, max=50.0,
         help="允许离起飞点多远(软限)；硬围栏 = 软限 + 自动余量"),

    dict(key="standoff", tasks=["track"], type="float", unit="m", label="停靠距离", group="目标距离",
         min=0.3, max=10.0,
         help="飞机停在离目标多远(水平)；实际停靠 ≈ 停靠距离 + 死区"),
    dict(key="detect_far", tasks=["track"], type="float", unit="m", label="检测远端", group="目标距离",
         min=0.5, max=30.0,
         help="空中实测最远能检测到多远；换相机/换标记后必须重测"),
    dict(key="entry_dist", tasks=["track"], type="float", unit="m", label="进场门", group="目标距离",
         min=0.5,
         warn_hi=(15.0, "很大：远距离 PnP 更毛，跳变门会频繁拒帧(看 n_rej_jump)"),
         help="目标比这远就不交给视觉，停在 MANUAL；应 ≤ 检测远端"),

    dict(key="speed", tasks=["track"], type="choice", unit="", label="速度档", group="任务",
         choices=["慢", "中", "快"],
         help="水平/垂直跟踪速度和手动速度"),
    dict(key="mission_s", tasks=["track"], type="float", unit="s", label="作业时长", group="任务",
         min=30.0, min_why="接近 + 作业根本不够", max=1800.0,
         warn_hi=(600.0, "超过 10 分钟：一块电池撑不住，请核对续航"),
         help="跟踪段时长；手动段时长按它推导"),

    # ---------------- 穿环 (stage12) ----------------
    dict(key="dash_v", tasks=["gate"], type="float", unit="m/s", label="冲刺速度", group="穿环",
         min=0.8, max=4.0, warn_hi=(3.0, "超过 3 m/s: 先在 T3 虚拟环逐级验证过再用"),
         help="冲刺巡航速度上限; 场地放不下时起飞后自动降速 (每次 0.3, 最低 1.2)"),
    dict(key="d_back", tasks=["gate"], type="float", unit="m", label="环后空间", group="穿环场地",
         min=1.0, max=50.0,
         help="纸面 → 环后最近障碍 (墙) 的距离, 卷尺量。冲刺要过纸后再冲一段才刹车, 需要 ≈ "
              "1.3 + 0.2v + v²/5 m (2.5 m/s 约 3.05, 2.0 约 2.5, 1.3 约 1.9); 不够会自动降速"),
    dict(key="back_margin", tasks=["gate"], type="float", unit="m", label="环后墙余量", group="穿环场地",
         min=0.1, max=2.0,
         warn_lo=(0.2, "低于 20cm: 纸面深度估计误差 (锁定时最大约 10cm) 和刹车超调都要从这里出"),
         help="冲刺刹停后机头离环后墙至少多远 (默认 0.3)。只管环后这一面; 两侧 / 起飞点后方的余量不变"),
    dict(key="d_rear", tasks=["gate"], type="float", unit="m", label="起飞点后方空地", group="穿环场地",
         min=0.3, max=50.0,
         help="起飞点往后 (背离环) 还有多少空地。起跑点可能在起飞点后面"),
    dict(key="d_side", tasks=["gate"], type="float", unit="m", label="两侧空间", group="穿环场地",
         min=0.6, max=50.0,
         help="环心 → 左右两侧最近障碍的距离 (取小的那边)"),
    dict(key="l_f", tasks=["gate"], type="float", unit="m", label="机头长", group="机体",
         min=0.05, max=1.0, help="机体中心 → 机头最前端 (含护桨, 不含针), 实测"),
    dict(key="l_r", tasks=["gate"], type="float", unit="m", label="机尾长", group="机体",
         min=0.05, max=1.0, help="机体中心 → 机尾最后端 (含护桨), 实测"),
    dict(key="w_half", tasks=["gate"], type="float", unit="m", label="半宽", group="机体",
         min=0.05, max=0.55, help="机体中心 → 最外侧 (含护桨), 实测。决定瞄点能偏离环心多远"),
    dict(key="allow_slow", tasks=["gate"], type="choice", unit="", label="场地不够时降速", group="穿环",
         choices=["否", "确认", "是"],
         help="场地放不下设定的冲刺速度时: 否 = 规划失败回手动段 (说明卡在哪、最高能冲多快); "
              "确认 (默认) = 降到能飞的速度, 冲刺前必须按 Y 确认; 是 = 自动降速 (会标出来)"),
    dict(key="brake_over", tasks=["gate"], adv=True, type="float", unit="s", label="刹车超调系数",
         group="穿环", min=0.0, max=1.0,
         help="刹停实际比规划多冲 ≈ 系数 × 冲刺速度 (默认 0.3: 10-21 实测 2.0 m/s 多冲 0.59 m)。"
              "离墙和水平范围按含超调的停止点查; 轨迹本身不变"),
    dict(key="pass_run", tasks=["gate"], type="float", unit="m", label="过纸冲刺距离", group="穿环",
         min=0.2, max=3.0,
         help="冲刺: 机头碰到纸后保持冲刺速度再往前冲多远才开始刹车 (默认 1.0), 让机身整个穿过纸、不被纸缠住。"
              "控制滞后已自动补偿; 越大需要的环后空间越大"),
    dict(key="run_max", tasks=["gate"], type="float", unit="m", label="最远起跑距离", group="穿环",
         min=0.0, max=8.0, warn_hi=(5.0, "超过 5 m: 远处锁定的横向 σ 和盲冲中的 VIO 漂移都会变大"),
         help="开始规划时飞机离环在 [最小助跑, 它] 之间就地起跑; 更远就前进到它, 更近就后退到最小助跑。"
              "0 = 总是先飞到最小助跑处 (约 3 m) 再起跑 —— 要从按 T 的位置直接起跑, 设成 ≥ 那里离环的距离"),
    dict(key="dash_acc", tasks=["gate"], adv=True, type="float", unit="m/s²", label="冲刺加速度",
         group="穿环", min=0.5, max=4.0, help="加速段加速度 (2.0 ≈ 前倾 11.5°)"),
    dict(key="brake_acc", tasks=["gate"], adv=True, type="float", unit="m/s²", label="制动减速度",
         group="穿环", min=0.5, max=4.0, help="刹车段减速度 (2.5 ≈ 后仰 14.3°)"),
    dict(key="t_lag", tasks=["gate"], adv=True, type="float", unit="s", label="控制滞后",
         group="穿环", min=0.0, max=1.0, help="只用于估计撞纸速度; T2 实测后替换"),
    dict(key="e_along", tasks=["gate"], adv=True, type="float", unit="m", label="沿法向误差预算",
         group="穿环", min=0.05, max=1.0, help="纸面位置估计最多差多少; 刹停窗口半宽必须 ≥ 它"),
    dict(key="v_tear_min", tasks=["gate"], adv=True, type="float", unit="m/s", label="撕纸最低速度",
         group="穿环", min=0.3, max=4.0, help="撕纸测试测到的最低速度 × 2; 最坏撞纸速度低于它 → 降不了速就放弃"),
    dict(key="entry_ang_max", tasks=["gate"], adv=True, type="float", unit="°", label="进入偏角上限",
         group="穿环", min=0.0, max=40.0, help="进入方向和环法向的最大夹角"),
    dict(key="d_straight", tasks=["gate"], adv=True, type="float", unit="m", label="稳定直线段",
         group="穿环", min=0.0, max=5.0, help="加速完到纸面之间至少留多少匀速直线"),
]
KNOB_INDEX = {k["key"]: k for k in KNOBS}

# ★2026-10-30 删掉的旋钮 (戳点模式)。老档案里还有 → **警告并忽略** (列出来), 不拒绝起飞。
#   例外: gate_mode = 戳点 必须报错 —— 不能把一份戳点档案悄悄当冲刺飞。
REMOVED_KNOBS = {
    "gate_mode": "穿环模式",
    "poke_v": "戳点速度", "poke_gap": "针尖离纸距离", "hit_v": "接触速度", "poke_depth": "压入深度",
    "creep": "爬行距离", "poke_dwell": "戳点停留", "retreat_v": "退回速度",
}
# 同理: 档案里的 set.<参数> 指向已删除的参数 → 警告并忽略; set.GATE_POKE = True → 报错
REMOVED_SET_PREFIX = ("POKE_",)
REMOVED_SET_KEYS = ("GATE_POKE",)


def _removed_knob(path, ln, k, v, errs, notes):
    """已删除的旋钮。→ True = 处理过了 (调用方 continue)。"""
    if k not in REMOVED_KNOBS:
        return False
    if k == "gate_mode" and v.strip() != "冲刺":
        errs.append("%s:%d gate_mode = %s: 戳点模式已删除 (2026-10-30) —— 这份档案不能当冲刺飞。"
                    "要冲刺就把 gate_mode 这一行删掉" % (path, ln, v.strip()))
    elif notes is not None:
        notes.append("%s:%d 旋钮 %s (%s) 已删除, 忽略这一行" % (path, ln, k, REMOVED_KNOBS[k]))
    return True


def _removed_set(path, ln, pk, v, errs, notes):
    if pk in REMOVED_SET_KEYS:
        if v.strip() not in ("False", "false", "0", "0.0"):
            errs.append("%s:%d set.%s = %s: 戳点模式已删除 (2026-10-30) —— 这份档案不能当冲刺飞。"
                        "要冲刺就把这一行删掉" % (path, ln, pk, v.strip()))
        elif notes is not None:
            notes.append("%s:%d set.%s 已删除, 忽略这一行" % (path, ln, pk))
        return True
    if pk.startswith(REMOVED_SET_PREFIX):
        if notes is not None:
            notes.append("%s:%d set.%s 是已删除的戳点参数, 忽略这一行" % (path, ln, pk))
        return True
    return False

# 穿环旋钮 → 参数 (一对一)
GATE_KNOB_PARAM = {
    "dash_v": "DASH_V", "d_back": "D_BACK", "d_rear": "D_REAR", "d_side": "D_SIDE",
    "l_f": "L_F", "l_r": "L_R", "w_half": "W_HALF", "dash_acc": "DASH_ACC",
    "brake_acc": "BRAKE_ACC", "t_lag": "T_LAG", "e_along": "E_ALONG",
    "v_tear_min": "V_TEAR_MIN", "entry_ang_max": "ENTRY_ANG_MAX", "d_straight": "D_STRAIGHT",
    "run_max": "D_RUN_MAX",
    "pass_run": "PASS_RUN", "back_margin": "M_BACK", "brake_over": "BRAKE_OVER_T",
}


def script_task(g):
    """飞行脚本的任务。stage11 没有 TASK 全局 = 停靠跟踪。"""
    t = (g or {}).get("TASK") if isinstance(g, dict) else None
    return t if t in ("track", "gate") else "track"

# 速度档 → (TRACK_V_MAX, TRACK_VZ_MAX, TELEOP_XY_SPEED, TELEOP_Z_SPEED)
SPEED_TABLE = {
    "慢": (0.30, 0.15, 0.30, 0.30),
    "中": (0.45, 0.20, 0.50, 0.50),
    "快": (0.60, 0.35, 0.80, 0.70),
}

# 场景 → 只动两类东西:
#   VISION_LOST_LAND_S  缓降类(LAND)判据, 收紧 = 更早缓降 = 更保守
#   BASE_DRIFT_LIM      起飞/悬停/降落阶段的水平硬限。室外有风, 0.6m 一起飞就越界
# ★ 不再动 MAX_TILT_DEG: 它触发的是空中停桨, 收紧 = 更容易摔 (见原则 2)
SCENE_TABLE = {
    "室内": {"VISION_LOST_LAND_S": 0.50, "BASE_DRIFT_LIM": 0.60},
    "地面": {"VISION_LOST_LAND_S": 0.80, "BASE_DRIFT_LIM": 1.00},
    "低空": {"VISION_LOST_LAND_S": 0.60, "BASE_DRIFT_LIM": 1.50},
    "高空": {"VISION_LOST_LAND_S": 0.60, "BASE_DRIFT_LIM": 2.00},
}

# =============================================================================
# 安全规则 —— 对**最终生效值**检查（不管来自默认值、档案还是 --set）
# =============================================================================
# (参数, 规则, 界限, 后果类型, 理由)
#   "<=" / ">=" : 缓降类，只准往保守方向改
#   "range"     : 停桨类，两头都有界 —— 太松失控才停，太紧正常飞也停
#   "=="        : 开关，必须保持
SAFETY_RULES = [
    ("MAX_TILT_DEG", "range", (25.0, 30.0), "FATAL",
     "触发空中停桨。低于 25° 正常机动/阵风就会误停桨(高空=坠机)，高于 30° 失控才停"),
    ("FATAL_CONFIRM_S", "range", (0.15, 0.50), "FATAL",
     "停桨去抖时间。太短毛刺就停桨，太长真失控时反应太慢"),
    ("VISION_LOST_LAND_S", "<=", 1.00, "LAND",
     "视觉断供多久就缓降。09-14 备用机实测断供后 5.6s 发散"),
    ("POSE_STALE_SEC", "<=", 0.15, "LAND", "位姿老化阈值"),
    ("POSE_JUMP_MAX", "<=", 0.20, "LAND", "位姿跳变阈值"),
    ("CELL_MIN_FLY", ">=", 3.50, "LAND", "单节最低电压"),
    ("FENCE_VIOLATION_LANDS", "==", True, "LAND",
     "围栏违规必须走缓降，不许改成空中停桨"),
    ("VISION_LOST_ENABLE", "==", True, "LAND",
     "视觉断供保护(09-14 坠机后新增)，不许关"),
]

# 穿环 (stage12) 才有的安全项, 只在 _check_gate 里查
GATE_SAFETY_RULES = [
    ("GATE_DASH_TILT_DEG", "range", (30.0, 50.0), "FATAL",
     "冲刺段 (按 G 后到刹停悬停) 的停桨倾角。急刹 + 撞纸有短时大倾角; 低于 30° 比平时还紧, "
     "高于 50° 已接近翻机才停"),
    ("GATE_DASH_TILT_CONFIRM_S", "range", (0.15, 0.50), "FATAL", "冲刺段停桨去抖时间"),
]

# 单独运行 flight_profile.py 时没有 stage11 的全局可参考, 用这组保底值推导
_STANDALONE_DEFAULTS = {
    "DESCEND_SPEED": 0.50, "RETURN_SPEED": 0.60, "TAKEOFF_HEIGHT": 1.5,
    "TRACK_Z_MAX": 10.0, "TELEOP_DZ_MAX": 10.0, "MAX_HEIGHT": 11.0,
    # 穿环 (和 stage12_gate.py 的代码默认值一致)
    "DASH_V_MIN": 1.2, "D_BACK": 2.0, "D_REAR": 2.0, "D_SIDE": 2.0,
    "L_F": 0.28, "L_R": 0.28, "W_HALF": 0.28, "PASS_RUN": 1.0, "M_BACK": 0.3,
    "BRAKE_OVER_T": 0.3, "GATE_SLOW_POLICY": 1,
}

# ★ 2026-10-09(2): 必须为正数的参数。--set 能把它们改成 0 或负数, 而 stage11 的
#   派生量区会拿它们做除数(TRACK_AGE_FULL = .../F_DETECT, TRACK_LOG_DECIM = CTRL_HZ/
#   TRACK_LOG_HZ …) —— 不先拦就会在派生量区抛 ZeroDivisionError, 网页只能看到
#   "dry-run 没有输出 JSON"。所以这一组在 setup() 里、派生量之前就检查。
#   只检查 stage11 里真实存在的那些(g.get)。
POSITIVE_PARAMS = (
    "F_DETECT", "SP_PUB_HZ", "CTRL_HZ", "MONITOR_HZ", "LOG_HZ", "TIMING_LOG_HZ",
    "EKF_LOG_HZ", "TRACK_LOG_HZ", "PHASE_LOOP_HZ", "STATUS_HZ", "TELEMETRY_HZ",
    "TRACK_V_MAX", "TRACK_VZ_MAX", "TELEOP_XY_SPEED", "TELEOP_Z_SPEED",
    "RETURN_SPEED", "DESCEND_SPEED", "TRACK_KP", "TRACK_Z_KP",
    "TRACK_LEASH_MAX", "TRACK_Z_LEASH", "POS_LOOP_TAU",
    "TRACK_STANDOFF", "DETECT_RANGE_MAX", "TRACK_XY_MAX", "TELEOP_XY_MAX",
    "MAX_HEIGHT", "TRACK_DURATION", "TELEOP_DURATION", "TAKEOFF_TIMEOUT",
    "RETURN_TIMEOUT", "DESCEND_TIMEOUT", "AUTO_LAND_TIMEOUT", "AUTO_LAND_SPEED_EST",
    # ★ 2026-10-19 穿环 (stage12)
    "DASH_V", "DASH_V_MIN", "DASH_V_STEP", "DASH_ACC", "BRAKE_ACC", "D_ALIGN_MIN",
    "L_F", "L_R", "W_HALF", "R_IN", "D_BACK", "D_REAR", "D_SIDE", "GATE_XY_MAX",
    "GATE_REPO_V", "GATE_YAW_RATE", "GATE_HOLD_S", "HOVER_ACQ", "HOVER_ALIGN",
    "GATE_STATUS_MAX_AGE", "GATE_LOG_HZ",
    "PASS_RUN", "M_BACK",
)

# 档案里 set.KEY 的 KEY 必须长这样(和网页、命令行 --set 同一规则)
_SET_KEY_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")


# =============================================================================
# 小工具
# =============================================================================
def allowed_params(g):
    """可被覆盖的参数 = 大写、不以 _ 开头、值是数字或开关。
    旧版直接用 set(g.keys())，于是 --set np=1 能把 numpy 覆盖掉。"""
    return {k for k, v in g.items()
            if k.isupper() and not k.startswith("_")
            and isinstance(v, (bool, int, float))}


def _coerce(name, value, current):
    """把 value 转成 current 的类型。返回 (值, 错误信息或 None)。
    int 参数仍然写成 int，bool 仍然写成 bool —— 类型不能被现场改掉。"""
    if isinstance(current, bool):
        if isinstance(value, bool):
            return value, None
        s = str(value).strip()
        if s in ("True", "true", "1", "1.0"):
            return True, None
        if s in ("False", "false", "0", "0.0"):
            return False, None
        return None, "%s 是开关，需要 True/False，得到 %r" % (name, value)
    if isinstance(current, int):
        if isinstance(value, bool):
            return None, "%s 需要整数，得到开关值" % name
        try:
            f = float(value)
        except (TypeError, ValueError):
            return None, "%s 需要整数，得到 %r" % (name, value)
        if not math.isfinite(f) or f != int(f):
            return None, "%s 需要整数，得到 %r" % (name, value)
        return int(f), None
    if isinstance(current, float):
        if isinstance(value, bool):
            return None, "%s 需要数字，得到开关值" % name
        try:
            f = float(value)
        except (TypeError, ValueError):
            return None, "%s 需要数字，得到 %r" % (name, value)
        if not math.isfinite(f):
            return None, "%s 需要有限数字，得到 %r" % (name, value)
        return f, None
    return None, "%s 的类型不支持覆盖" % name


def _fmt(v):
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, float):
        return "%g" % v
    return str(v)


# =============================================================================
# 档案解析 / 写回
# =============================================================================
def parse_conf_text(text, path="<text>", notes=None):
    """
    返回 (knobs: dict, sets: list["KEY=VALUE"], errors: list)。
    未知键 / 重复键 / 非有限数字 → 报错（不静默）。
    已删除的旋钮 (REMOVED_KNOBS) → 说明追加到 notes (给了的话), 不算错误; gate_mode = 戳点 例外, 报错。

    ★ 2026-10-09(2):
      · 数字必须是有限值。旧版 float("nan") 能通过 —— nan 和任何数比较都是 False,
        于是 `xy_radius = nan` 会绕过全部范围检查, 飞行时水平围栏永远不触发。
      · 支持 `set.KEY = VALUE` 行(原始参数覆盖, 等价于命令行 --set)。
        这里只检查语法; 参数是否存在、类型、安全规则在 setup()/finalize() 里查。
    """
    knobs, sets, errs = {}, [], []
    set_keys = set()
    for ln, raw in enumerate(text.splitlines(), 1):
        s = raw.split("#", 1)[0].strip()
        if not s:
            continue
        if "=" not in s:
            errs.append("%s:%d 不是 key = value 形式: %r" % (path, ln, raw.strip()))
            continue
        k, v = s.split("=", 1)
        k, v = k.strip(), v.strip()

        if k.startswith("set."):
            pk = k[4:].strip()
            if not _SET_KEY_RE.match(pk):
                errs.append("%s:%d set.%s: 参数名必须是大写字母/数字/下划线" % (path, ln, pk))
                continue
            if not v or any(c.isspace() for c in v):
                errs.append("%s:%d set.%s 的值不能为空或含空格: %r" % (path, ln, pk, v))
                continue
            if pk in set_keys:
                errs.append("%s:%d set.%s 重复出现 —— 不知道该信哪一个" % (path, ln, pk))
                continue
            set_keys.add(pk)
            if _removed_set(path, ln, pk, v, errs, notes):
                continue
            sets.append("%s=%s" % (pk, v))
            continue

        spec = KNOB_INDEX.get(k)
        if spec is None and _removed_knob(path, ln, k, v, errs, notes):
            continue
        if spec is None:
            errs.append("%s:%d **未知键 %r** —— 拼错了不会静默生效，这里直接报错。"
                        "允许的键: %s (以及 set.参数名)"
                        % (path, ln, k, ", ".join(KNOB_INDEX)))
            continue
        if k in knobs:
            errs.append("%s:%d 键 %r 重复出现 —— 不知道该信哪一个" % (path, ln, k))
            continue
        if spec["type"] == "float":
            try:
                f = float(v)
            except ValueError:
                errs.append("%s:%d %s 需要数字，得到 %r" % (path, ln, k, v))
                continue
            if not math.isfinite(f):
                errs.append("%s:%d %s 需要有限数字，得到 %r（nan/inf 会绕过所有范围检查）"
                            % (path, ln, k, v))
                continue
            knobs[k] = f
        else:
            knobs[k] = v
    return knobs, sets, errs


def format_conf(knobs, header=None, sets=None):
    """
    旋钮 dict (+ set 列表) → .conf 文本（按旋钮表顺序，行尾带说明）。
    与 parse_conf_text 互逆。sets = ["KEY=VALUE", ...]
    """
    L = []
    if header:
        for h in str(header).splitlines():
            L.append("# " + h)
        L.append("")
    width = max(len(k["key"]) for k in KNOBS)
    for spec in KNOBS:
        k = spec["key"]
        if k not in knobs or knobs[k] is None or knobs[k] == "":
            continue
        v = knobs[k]
        vs = _fmt(v) if spec["type"] == "float" else str(v)
        unit = (" " + spec["unit"]) if spec["unit"] else ""
        L.append("%-*s = %-10s # %s%s：%s"
                 % (width, k, vs, spec["label"], unit, spec["help"]))
    if sets:
        L.append("")
        L.append("# ---- 原始参数覆盖（等价于命令行 --set，照样过全部校验）")
        for s in sets:
            pk, v = s.split("=", 1)
            L.append("set.%s = %s" % (pk.strip(), v.strip()))
    return "\n".join(L) + "\n"


def load_site(name, sites_dir, notes=None):
    """返回 (knobs, sets, path, errors)。★ 2026-10-09(2) 多了 sets。notes: 见 parse_conf_text"""
    path = os.path.join(os.path.expanduser(sites_dir), name + ".conf")
    if not os.path.exists(path):
        return None, [], path, ["档案不存在: %s" % path]
    with open(path, encoding="utf-8") as f:
        knobs, sets, errs = parse_conf_text(f.read(), path, notes)
    return knobs, sets, path, errs


def list_sites(sites_dir=DEFAULT_SITES_DIR):
    d = os.path.expanduser(sites_dir)
    if not os.path.isdir(d):
        return []
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".conf"))


def schema_json():
    """旋钮表 + 档位表，给网页表单用。"""
    return {"knobs": KNOBS, "speed_table": SPEED_TABLE, "scene_table": SCENE_TABLE,
            "safety_rules": [dict(param=p, rule=r, limit=l, kind=k, why=w)
                             for p, r, l, k, w in SAFETY_RULES]}


# =============================================================================
# 旋钮级检查（只看旋钮本身，不需要 stage11 的全局）
# =============================================================================
def validate_knobs(knobs):
    """返回 (errors, warnings)。范围来自 KNOBS 表。"""
    errs, warns = [], []
    for k, v in knobs.items():
        spec = KNOB_INDEX[k]
        if spec["type"] == "choice":
            if v not in spec["choices"]:
                errs.append("%s 只能是 %s，得到 %r" % (k, "/".join(spec["choices"]), v))
            continue
        if spec["type"] != "float":
            continue
        if "min" in spec and v < spec["min"]:
            errs.append("%s=%g 低于下限 %g%s" % (k, v, spec["min"],
                        ("：" + spec["min_why"]) if spec.get("min_why") else ""))
        if "max" in spec and v > spec["max"]:
            errs.append("%s=%g 超过上限 %g" % (k, v, spec["max"]))
        if "warn_lo" in spec and v < spec["warn_lo"][0] and not (
                "min" in spec and v < spec["min"]):
            warns.append("%s=%g %s" % (k, v, spec["warn_lo"][1]))
        if "warn_hi" in spec and v > spec["warn_hi"][0]:
            warns.append("%s=%g %s" % (k, v, spec["warn_hi"][1]))

    zlo, zhi = knobs.get("z_low"), knobs.get("z_high")
    if (zlo is None) != (zhi is None):
        errs.append("z_low 和 z_high 必须一起给（只给一个无法确定高度带）")
    elif zlo is not None:
        if not zlo < zhi:
            errs.append("z_low(%.2f) 必须小于 z_high(%.2f)" % (zlo, zhi))
        elif zhi - zlo < 0.30:
            errs.append("高度带只有 %.2fm，太窄：tag 稍一移动就出带被整帧丢弃" % (zhi - zlo))

    if knobs.get("scene") == "室内" and knobs.get("speed") == "快":
        errs.append("scene=室内 不允许 speed=快")
    return errs, warns


def check_knob_task(knobs, task, script="飞行脚本"):
    """档案的 task 旋钮要和脚本一致; 给了本任务不用的旋钮也报错。返回 errors。"""
    E = []
    t = knobs.get("task")
    if t is not None and TASK_NAMES.get(t) != task:
        E.append("档案是给「%s」写的 (task = %s), 但 %s 是「%s」脚本 —— 换对档案或脚本"
                 % (t, t, script, {v: k for k, v in TASK_NAMES.items()}.get(task, task)))
    for k in knobs:
        ts = KNOB_INDEX[k].get("tasks")
        if ts and task not in ts and k != "task":
            E.append("旋钮 %s (%s) 只用于「%s」, %s 不用它 —— 从档案里删掉, 免得以为它生效了"
                     % (k, KNOB_INDEX[k]["label"],
                        "/".join({v: kk for kk, v in TASK_NAMES.items()}.get(x, x) for x in ts),
                        script))
    return E


# =============================================================================
# 旋钮 → 参数覆盖
# =============================================================================
def _build_gate_overrides(knobs, view):
    """穿环 (stage12) 的旋钮 → 参数。和停靠跟踪共用的只有 起飞高度 / 高度带 / 水平范围 / 场景。"""
    ov, rep, warns = {}, [], []

    def cur(k):
        if k in ov:
            return ov[k]
        if k in view:
            return view[k]
        return _STANDALONE_DEFAULTS.get(k)

    tk = knobs.get("takeoff_h")
    if tk is not None:
        ov["TAKEOFF_HEIGHT"] = tk
        ov["TAKEOFF_TIMEOUT"] = max(10.0, math.ceil(tk / 0.5) + 5.0)
        rep.append("起飞高度 %.2f m (超时 %.0f s)" % (tk, ov["TAKEOFF_TIMEOUT"]))

    zlo, zhi = knobs.get("z_low"), knobs.get("z_high")
    if zlo is not None and zhi is not None:
        ov["GATE_Z_MIN_DZ"], ov["GATE_Z_MAX_DZ"] = zlo, zhi
        top = max(zhi, cur("TAKEOFF_HEIGHT"))
        ov["MAX_HEIGHT"] = top + 1.0
        rep.append("瞄点高度带 [%.2f, %.2f] m / 安全硬顶 %.2f m" % (zlo, zhi, top + 1.0))
    elif tk is not None and tk + 1.0 > cur("MAX_HEIGHT"):
        ov["MAX_HEIGHT"] = tk + 1.0
        rep.append("起飞高度高于原硬顶 → 抬高 MAX_HEIGHT")
    if "MAX_HEIGHT" in ov:
        dsp = cur("DESCEND_SPEED")
        ov["DESCEND_TIMEOUT"] = max(45.0, math.ceil(ov["MAX_HEIGHT"] / dsp * 2) + 5.0)

    r = knobs.get("xy_radius")
    if r is not None:
        ov["GATE_XY_MAX"] = r
        margin = min(2.0, max(0.5, 0.15 * r))
        ov["ENVELOPE_MARGIN"] = margin
        rep.append("水平范围 %.2f m (起跑点/制动终点都要在内) / 硬围栏 %.2f m" % (r, r + margin))

    given = [(k, GATE_KNOB_PARAM[k]) for k in GATE_KNOB_PARAM if knobs.get(k) is not None]
    for k, pname in given:
        ov[pname] = knobs[k]
    if given:
        rep.append("穿环: " + "  ".join("%s=%s" % (pn, _fmt(knobs[k])) for k, pn in given))
    if knobs.get("allow_slow") is not None:
        ov["GATE_SLOW_POLICY"] = {"否": 0, "确认": 1, "是": 2}[knobs["allow_slow"]]
        rep.append("场地不够时降速: %s" % knobs["allow_slow"])
    if "DASH_V" in ov and ov["DASH_V"] < cur("DASH_V_MIN"):
        ov["DASH_V_MIN"] = ov["DASH_V"]          # 冲刺速度本身比降速下限还低: 不降速
    miss = [k for k in ("d_back", "d_rear", "d_side", "l_f", "l_r", "w_half")
            if knobs.get(k) is None]
    if miss:
        warns.append("档案没给 %s —— 用的是代码默认值 (%s)。这些是**现场/实测量**, 请在档案里填写"
                     % ("、".join(KNOB_INDEX[k]["label"] for k in miss),
                        ", ".join("%s=%s" % (GATE_KNOB_PARAM[k], _fmt(cur(GATE_KNOB_PARAM[k])))
                                  for k in miss)))

    sc = knobs.get("scene")
    if sc in SCENE_TABLE:
        for k, v in SCENE_TABLE[sc].items():
            ov[k] = v
        rep.append("场景 %s (%s)" % (sc, ", ".join(
            "%s=%g" % kv for kv in sorted(SCENE_TABLE[sc].items()))))
    return ov, rep, warns


def build_overrides(knobs, view, task="track"):
    """
    把旋钮翻译成**具体参数覆盖**。返回 (overrides, report_lines, warnings)。
    没给的旋钮 = 用代码里的默认值（不改）。

    view = 当前参数视图(代码默认值 + --set)。推导里用到的速度/现有上限都从
    这里取，不再写死 0.50 / 0.60 / 0.98 —— 否则 --set DESCEND_SPEED 之后
    超时不会跟着变。
    task = 飞行脚本的任务 (见 script_task)。穿环走 _build_gate_overrides。
    """
    if task == "gate":
        return _build_gate_overrides(knobs, view)
    ov, rep, warns = {}, [], []

    def cur(k):
        if k in ov:
            return ov[k]
        if k in view:
            return view[k]
        return _STANDALONE_DEFAULTS.get(k)

    zlo, zhi = knobs.get("z_low"), knobs.get("z_high")
    tk = knobs.get("takeoff_h")
    z_given = zlo is not None and zhi is not None

    # ---- 起飞高度
    if tk is not None:
        ov["TAKEOFF_HEIGHT"] = tk
        # 台阶式爬升: 按 0.5m/s 保守估
        ov["TAKEOFF_TIMEOUT"] = max(10.0, math.ceil(tk / 0.5) + 5.0)
        rep.append("起飞高度 %.2f m (超时 %.0f s)" % (tk, ov["TAKEOFF_TIMEOUT"]))

    # ---- 高度带 + 高度三件套（钳位顶 / 手动顶 / 安全硬顶）
    if z_given:
        ov["TARGET_Z_NOMINAL"] = (zlo + zhi) / 2.0
        ov["TARGET_Z_TOLERANCE"] = (zhi - zlo) / 2.0
        rep.append("目标高度带 [%.2f, %.2f] m  →  门限 = %.2f ± %.2f"
                   % (zlo, zhi, ov["TARGET_Z_NOMINAL"], ov["TARGET_Z_TOLERANCE"]))
        # top 取"目标最高"和"起飞高度"里大的 —— 起飞高度不能比手动顶还高
        top = max(zhi, cur("TAKEOFF_HEIGHT"))
        ov["TRACK_Z_MAX"] = top
        ov["TELEOP_DZ_MAX"] = top
        ov["MAX_HEIGHT"] = top + 1.0
        rep.append("高度钳位顶 %.2f m / 安全硬顶 %.2f m" % (top, top + 1.0))
    elif tk is not None:
        # ★ 2026-10-09 修: 旧版只给 takeoff_h 时 top=takeoff_h，会把
        #   MAX_HEIGHT 从 11 压到 2.5、TRACK_Z_MAX 压到 1.5，而门限带还是默认的
        #   [0.5,10] → 5m 的目标被接受却够不着 = 09-16 那种"看得见不动"。
        #   现在没给高度带时**只抬不压**。
        raised = []
        for k in ("TRACK_Z_MAX", "TELEOP_DZ_MAX"):
            if tk > cur(k):
                ov[k] = tk
                raised.append(k)
        if cur("TRACK_Z_MAX") + 1.0 > cur("MAX_HEIGHT"):
            ov["MAX_HEIGHT"] = cur("TRACK_Z_MAX") + 1.0
            raised.append("MAX_HEIGHT")
        if raised:
            rep.append("起飞高度高于原上限 → 抬高 %s" % ", ".join(raised))

    # ---- 降落超时: 按实际降落速度反算
    if "MAX_HEIGHT" in ov:
        top = ov["MAX_HEIGHT"]
        dsp = cur("DESCEND_SPEED")
        ov["DESCEND_TIMEOUT"] = max(45.0, math.ceil(top / dsp * 2) + 5.0)
        rep.append("降落超时 %.0f s (按 %.2fm/s 降 %.1fm 的两倍留量)"
                   % (ov["DESCEND_TIMEOUT"], dsp, top))

    # ---- 水平范围
    r = knobs.get("xy_radius")
    if r is not None:
        ov["TRACK_XY_MAX"] = r
        ov["TELEOP_XY_MAX"] = r
        margin = min(2.0, max(0.5, 0.15 * r))     # 小场地别用 1m 余量
        ov["ENVELOPE_MARGIN"] = margin
        rsp = cur("RETURN_SPEED")
        # 圆形围栏: 最远 = 半径 + 余量
        ov["RETURN_TIMEOUT"] = max(40.0, math.ceil((r + margin) / rsp * 1.5) + 5.0)
        rep.append("水平范围 软限 %.2f m / 硬围栏 %.2f m (返航超时 %.0f s)"
                   % (r, r + margin, ov["RETURN_TIMEOUT"]))

    # ---- 目标距离
    sd = knobs.get("standoff")
    if sd is not None:
        ov["TRACK_STANDOFF"] = sd
        rep.append("停靠距离 %.2f m" % sd)
    far = knobs.get("detect_far")
    if far is not None:
        ov["DETECT_RANGE_MAX"] = far
        rep.append("检测远端 %.2f m" % far)
    ed = knobs.get("entry_dist")
    if ed is not None:
        ov["TRACK_ENTRY_MAX_D"] = ed
        rep.append("进场门 目标比 %.2f m 远就不进闭环" % ed)

    # ---- 作业时长
    # ★ 2026-10-09: 不再推 TRACK_NOINPUT_MAX。那是 SSH 假死检测(链路活性),
    #   和任务长短无关 —— 旧版任务 600s 时要 10 分钟无输入才判死。
    ms = knobs.get("mission_s")
    if ms is not None:
        ov["TRACK_DURATION"] = ms
        ov["TELEOP_DURATION"] = min(60.0, max(10.0, 0.2 * ms))
        rep.append("作业 %g s (手动段 %g s)" % (ms, ov["TELEOP_DURATION"]))

    # ---- 速度档
    sp = knobs.get("speed")
    if sp in SPEED_TABLE:
        v, vz, txy, tz = SPEED_TABLE[sp]
        ov["TRACK_V_MAX"], ov["TRACK_VZ_MAX"] = v, vz
        ov["TELEOP_XY_SPEED"], ov["TELEOP_Z_SPEED"] = txy, tz
        rep.append("速度档 %s (水平 %.2f / 垂直 %.2f m/s, 手动 %.2f/%.2f)"
                   % (sp, v, vz, txy, tz))

    # ---- 场景
    sc = knobs.get("scene")
    if sc in SCENE_TABLE:
        for k, v in SCENE_TABLE[sc].items():
            ov[k] = v
        rep.append("场景 %s (%s)" % (sc, ", ".join(
            "%s=%g" % kv for kv in sorted(SCENE_TABLE[sc].items()))))

    return ov, rep, warns


# =============================================================================
# 命令行
# =============================================================================
def pop_our_args(argv):
    """
    从 argv 里挑出本模块的参数, **其余原样留给 ROS**
    （ROS 会往 argv 里塞 __name:= __log:= _param:=value 这类东西，
     用 argparse 直接 parse 会炸 —— 所以只能自己挑）。
    返回 (opts, rest, errors)

    ★ 2026-10-09(2): 任何以 "-" 开头却不认识的参数 → 报错。旧版会把它原样
      交给 ROS, ROS 不认识也不报错 —— `--dryrun` 拼错就真的起飞了。
      ROS 的参数都是 `name:=value` / `__name:=x` / `_param:=v`, 不以 "-" 开头。
    """
    opts = {"site": None, "sets": [], "dry_run": False, "json": False,
            "sites_dir": None}
    errs = []
    rest = list(argv[:1])            # argv[0] 是脚本自己, 原样保留
    i = 1
    while i < len(argv):
        a = argv[i]
        if a in ("--site", "--sites-dir", "--set"):
            if i + 1 >= len(argv) or argv[i + 1].startswith("-"):
                errs.append("%s 后面缺少值（旧版会静默按代码默认值起飞）" % a)
                i += 1
                continue
            val = argv[i + 1]
            if a == "--site":
                if opts["site"] is not None:
                    errs.append("--site 给了两次 (%s / %s)" % (opts["site"], val))
                opts["site"] = val
            elif a == "--sites-dir":
                opts["sites_dir"] = val
            else:
                opts["sets"].append(val)
            i += 2
            continue
        if a == "--dry-run":
            opts["dry_run"] = True; i += 1; continue
        if a == "--json":
            opts["json"] = True; i += 1; continue
        if a.startswith("-"):
            errs.append("不认识的参数 %r —— 拼错了？允许的是: %s"
                        % (a, " ".join(OUR_FLAGS)))
            i += 1
            continue
        rest.append(a); i += 1
    return opts, rest, errs


OUR_FLAGS = ("--site", "--sites-dir", "--set", "--dry-run", "--json")


def parse_sets(sets, g, src="--set"):
    """--set KEY=VALUE（或档案里的 set.KEY）。KEY 必须是可覆盖的参数，值按原类型转换。"""
    allowed = allowed_params(g)
    ov, errs = {}, []
    for s in sets:
        if "=" not in s:
            errs.append("%s 需要 KEY=VALUE 形式: %r" % (src, s))
            continue
        k, v = s.split("=", 1)
        k, v = k.strip(), v.strip()
        if k not in allowed:
            if k in g:
                errs.append("%s %s: 这个名字不是可覆盖的参数"
                            "（只允许大写的数值/开关参数）" % (src, k))
            else:
                errs.append("%s %s: 没有这个参数（拼错不会静默生效）" % (src, k))
            continue
        val, e = _coerce(k, v, g[k])
        if e:
            errs.append("%s %s" % (src, e))
            continue
        ov[k] = val
    return ov, errs


def check_positive(g):
    """POSITIVE_PARAMS 里存在于 g 的必须 > 0。返回 errors。"""
    E = []
    for k in POSITIVE_PARAMS:
        v = g.get(k)
        if v is None or isinstance(v, bool):
            continue
        if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
            E.append("%s=%s 必须是正数（它会被当作除数/速度/频率用）" % (k, _fmt(v)))
    return E


# =============================================================================
# 第一段: setup —— 派生量之前
# =============================================================================
def setup(argv, g, default_sites_dir=None):
    """
    解析档案 + --set → 写进模块全局 g。**不做一致性校验**(那是 finalize 的事，
    因为派生量还没算)。只拦"读不懂"的错误: 档案格式、未知键、旋钮越界、--set 拼错。
    返回 report dict，交给 finalize 继续用。
    """
    opts, rest, arg_errs = pop_our_args(argv)
    sys.argv[:] = rest                       # 把非本模块参数还给 ROS
    sites_dir = opts["sites_dir"] or default_sites_dir or DEFAULT_SITES_DIR

    report = {"src": "默认值(代码里的保底)", "site": opts["site"], "knobs": {},
              "sets": [], "applied": [], "lines": [], "warnings": [], "errors": [],
              "dry_run": opts["dry_run"], "json": opts["json"]}
    if arg_errs:
        report["errors"] += arg_errs
        _die(report, g)

    knobs, conf_sets = {}, []
    if opts["site"]:
        knobs, conf_sets, path, errs = load_site(opts["site"], sites_dir, report["warnings"])
        report["src"] = path
        if errs:
            report["errors"] += errs
            _die(report, g)
        knobs = knobs or {}
    report["knobs"] = knobs
    report["sets"] = list(conf_sets) + list(opts["sets"])

    e1, w1 = validate_knobs(knobs)
    report["errors"] += e1
    report["warnings"] += w1
    task = script_task(g)
    report["task"] = task
    report["errors"] += check_knob_task(knobs, task, os.path.basename(sys.argv[0] or "飞行脚本"))

    # 档案里的 set.KEY 和命令行 --set 走同一条路; 命令行优先级更高
    conf_ov, e2 = parse_sets(conf_sets, g, src="档案 set.")
    cli_ov, e3 = parse_sets(opts["sets"], g, src="--set")
    report["errors"] += e2 + e3
    if report["errors"]:
        _die(report, g)
    sets = dict(conf_ov)
    sets.update(cli_ov)

    view = {k: g[k] for k in allowed_params(g)}
    view.update(sets)
    ov, rep, w3 = build_overrides(knobs, view, task)
    report["lines"] += rep
    report["warnings"] += w3
    if sets:
        ov.update(sets)                       # set 优先级最高
        if conf_ov:
            report["lines"].append("档案 set. 覆盖: " + ", ".join(
                "%s=%s" % (k, _fmt(v)) for k, v in sorted(conf_ov.items())))
        if cli_ov:
            report["lines"].append("--set 临时覆盖: " + ", ".join(
                "%s=%s" % (k, _fmt(v)) for k, v in sorted(cli_ov.items())))

    # ---- 写进模块全局（必须在派生量之前！否则派生量会拿旧值算）
    for k, v in ov.items():
        if k not in g:
            report["errors"].append(
                "参数 %s 不存在于 %s —— 两个文件版本不配套？"
                % (k, os.path.basename(sys.argv[0] or "飞行脚本")))
            continue
        val, e = _coerce(k, v, g[k])
        if e:
            report["errors"].append(e)
            continue
        g[k] = val
        report["applied"].append((k, val))
    # 派生量区要拿这些做除数 → 必须在派生量之前拦住 0 / 负数
    report["errors"] += check_positive(g)
    if report["errors"]:
        _die(report, g)
    return report


# =============================================================================
# 第二段: finalize —— 派生量之后，对最终生效值做全部校验
# =============================================================================
def _check_safety(g, E, rules=None):
    """SAFETY_RULES —— 两种任务共用 (rules 给了就查那一组)。"""
    for p, rule, lim, kind, why in (SAFETY_RULES if rules is None else rules):
        if p not in g:
            E.append("安全参数 %s 不存在 —— 两个文件版本不配套？" % p)
            continue
        v = g[p]
        bad = ((rule == "<=" and v > lim) or (rule == ">=" and v < lim)
               or (rule == "==" and v != lim)
               or (rule == "range" and not (lim[0] <= v <= lim[1])))
        if bad:
            lim_s = ("[%g, %g]" % lim) if rule == "range" else ("%s %s" % (rule, lim))
            E.append("安全项 %s=%s 不满足 %s [%s]：%s" % (p, _fmt(v), lim_s, kind, why))


def check_effective(g):
    """返回 (errors, warnings)。g = 飞行脚本的全局（含派生量）。按脚本任务分派。"""
    if script_task(g) == "gate":
        return _check_gate(g)
    return _check_track(g)


def _check_track(g):
    """停靠跟踪 (stage11)。2026-10-19 之前的 check_effective, 内容未改。"""
    E, W = [], []
    get = g.get

    # ---------------- 安全规则 ----------------
    _check_safety(g, E)
    if get("TRACK_ALLOW_SIM_DRIVE"):
        W.append("TRACK_ALLOW_SIM_DRIVE=True —— 假数据也能驱动飞机！只在台架测试时用")

    # ---------------- 高度 ----------------
    nom, tol = g["TARGET_Z_NOMINAL"], g["TARGET_Z_TOLERANCE"]
    b_lo, b_hi = nom - tol, nom + tol
    if tol <= 0 or b_hi - b_lo < 0.30:
        E.append("z 门限带 [%.2f, %.2f] 太窄或为空" % (b_lo, b_hi))
    if b_lo < 0.50 - 1e-9:
        E.append("z 门限带下边界 %.2f < 0.50：地效 + 桨风吹板 + 落地误判" % b_lo)
    if g["TRACK_Z_MIN"] >= b_lo:
        W.append("高度钳位下限 TRACK_Z_MIN=%.2f 不低于门限带下边界 %.2f："
                 "目标在带底时飞机够不着(err_z 恒有余差)" % (g["TRACK_Z_MIN"], b_lo))
    if b_hi > g["TRACK_Z_MAX"] + 1e-6:
        msg = ("门限带上边界 %.2f > 高度钳位顶 TRACK_Z_MAX=%.2f：高处目标会被接受却够不着"
               "（09-16「看得见但不动」同类）" % (b_hi, g["TRACK_Z_MAX"]))
        (E if get("TRACK_Z_ENABLE") else W).append(msg)
    if g["TRACK_Z_MAX"] > g["MAX_HEIGHT"] - 0.5 + 1e-9:
        E.append("高度钳位顶 %.2f 离安全硬顶 %.2f 太近(<0.5m)：飞到上限就触发缓降"
                 % (g["TRACK_Z_MAX"], g["MAX_HEIGHT"]))
    if g["TELEOP_DZ_MAX"] > g["MAX_HEIGHT"] - 0.5 + 1e-9:
        E.append("手动高度顶 %.2f 离安全硬顶 %.2f 太近(<0.5m)"
                 % (g["TELEOP_DZ_MAX"], g["MAX_HEIGHT"]))
    tk = g["TAKEOFF_HEIGHT"]
    if not (g["TELEOP_DZ_MIN"] <= tk <= g["TELEOP_DZ_MAX"]):
        E.append("起飞高度 %.2f 不在手动高度带 [%.2f, %.2f] 内"
                 % (tk, g["TELEOP_DZ_MIN"], g["TELEOP_DZ_MAX"]))
    if tk < 0.5:
        E.append("起飞高度 %.2f < 0.5：离地判定不可靠" % tk)
    if tk > 2.0:
        W.append("起飞高度 %.1f m > 2.0：起飞段是位置阶跃，PX4 会以最大爬升率去追" % tk)
    if g["TAKEOFF_TIMEOUT"] < tk / 0.5 + 3.0:
        W.append("起飞超时 %.0fs 偏短(升 %.1fm 按 0.5m/s 估要 %.0fs)"
                 % (g["TAKEOFF_TIMEOUT"], tk, tk / 0.5))
    if g["TRACK_Z_DEADBAND_EXIT"] >= g["TRACK_Z_DEADBAND"]:
        E.append("TRACK_Z_DEADBAND_EXIT 必须 < TRACK_Z_DEADBAND，否则滞回不起作用")

    # ---------------- 水平 ----------------
    xy_min = min(g["TRACK_XY_MAX"], g["TELEOP_XY_MAX"])
    hard = xy_min + g["ENVELOPE_MARGIN"]
    base = g["BASE_DRIFT_LIM"]
    if base < 0.30:
        E.append("BASE_DRIFT_LIM=%.2f < 0.30：正常悬停漂移就会触发缓降" % base)
    if base + 0.40 > hard + 1e-9:
        E.append("起降阶段水平限制(最大 %.2f m)比水平硬围栏 %.2f m 还大 —— "
                 "xy_radius 太小或 scene 选错" % (base + 0.40, hard))
    stop = g.get("TRACK_STOP_DIST", g["TRACK_STANDOFF"] + g["TRACK_DEADBAND"])
    if g["TRACK_XY_MAX"] < stop:
        W.append("水平软限 %.2f m 小于停靠距离 %.2f m：目标靠近时飞机没有后退空间"
                 % (g["TRACK_XY_MAX"], stop))
    if g["TRACK_DEADBAND_EXIT"] >= g["TRACK_DEADBAND"]:
        E.append("TRACK_DEADBAND_EXIT 必须 < TRACK_DEADBAND，否则滞回不起作用")

    # ---------------- 目标距离 ----------------
    far = g["DETECT_RANGE_MAX"]
    if stop >= far:
        E.append("停靠距离 %.2f m ≥ 检测远端 %.2f m：飞机停下的位置已经看不见目标"
                 % (stop, far))
    elif far - stop < 0.30:
        W.append("靠近方向可用带只有 %.2f m：目标稍一远离就丢" % (far - stop))
    ed = g["TRACK_ENTRY_MAX_D"]
    if ed <= stop:
        E.append("进场门 %.2f m ≤ 停靠距离 %.2f m：永远进不了闭环" % (ed, stop))
    if ed > far + 1e-6:
        W.append("进场门 %.2f m > 检测远端 %.2f m：进场门实际不起作用"
                 "（想关掉它请明说；否则把 entry_dist 设到 ≤ detect_far）" % (ed, far))
    inner = g["TRACK_STANDOFF"] - g["TRACK_DEADBAND"]
    if inner <= g["DETECT_RANGE_MIN"]:
        W.append("死区内边界 %.2f m ≤ 近端检测距离 %.2f m：后退前读数可能已不可信"
                 % (inner, g["DETECT_RANGE_MIN"]))

    # ---------------- 速度 ----------------
    ratio = get("VEL_ENVELOPE_RATIO", 2.0)
    pe = get("PHASE_ENVELOPE")
    if isinstance(pe, dict):
        need_tr = 1.7 * max(g["TRACK_V_MAX"], g["TELEOP_XY_SPEED"])
        if pe["track"][1] < need_tr:
            E.append("track 速度包络 %.2f m/s < 1.7×指令速度 %.2f m/s：一跟踪就会触发缓降"
                     "（09-15 实测实际速度≈1.73×指令）" % (pe["track"][1], need_tr))
        need_te = 1.7 * g["TELEOP_XY_SPEED"]
        if pe["teleop"][1] < need_te:
            E.append("teleop 速度包络 %.2f m/s < 1.7×手动速度 %.2f m/s"
                     % (pe["teleop"][1], need_te))
    if ratio < 1.7:
        E.append("VEL_ENVELOPE_RATIO=%.2f < 1.7：低于实测超调比例" % ratio)
    tau = get("POS_LOOP_TAU", 1.0)
    if g["TRACK_V_MAX"] * tau > g["TRACK_LEASH_MAX"] + 1e-9:
        W.append("牵引绳 %.2f m < 速度 %.2f × 位置环时间常数 %.1fs：正常跟踪时牵引绳会"
                 "一直饱和，飞机达不到 V_MAX。牵引绳没有自动放大(09-15 记录它影响振荡)，"
                 "需要更快就 --set TRACK_LEASH_MAX=%.2f 并观察是否摆动"
                 % (g["TRACK_LEASH_MAX"], g["TRACK_V_MAX"], tau,
                    g["TRACK_V_MAX"] * tau * 1.2))
    # ★ 2026-10-09(2): z 同理。前馈关闭时 z 通道是纯位置控制,
    #   能达到的垂直速度 ≈ TRACK_Z_LEASH / 位置环时间常数。
    ff = bool(get("TRACK_VZ_FEEDFORWARD", False))
    if g.get("TRACK_Z_ENABLE") and not ff and \
            g["TRACK_VZ_MAX"] * tau > g["TRACK_Z_LEASH"] + 1e-9:
        W.append("垂直牵引绳 %.2f m < 垂直速度 %.2f × %.1fs：z 通道是纯位置控制，"
                 "实际垂直速度到不了 TRACK_VZ_MAX（约 %.2f m/s 封顶）。"
                 "要真正达到 VZ_MAX 需要 set.TRACK_VZ_FEEDFORWARD = True（单独飞一次验证）"
                 % (g["TRACK_Z_LEASH"], g["TRACK_VZ_MAX"], tau,
                    g["TRACK_Z_LEASH"] / tau))
    if ff:
        W.append("TRACK_VZ_FEEDFORWARD=True：垂直速度前馈生效，z 动态和以往所有飞行都不同"
                 "（以往数据全是纯位置 z 环）。这一次别同时改别的参数")

    # ---------------- 时间 ----------------
    need_d = (g["MAX_HEIGHT"] - g["DESCEND_Z_TARGET_DZ"]) / g["DESCEND_SPEED"] * 1.2 + 3
    if g["DESCEND_TIMEOUT"] < need_d:
        W.append("降落超时 %.0fs < 从硬顶降下来约需 %.0fs：超时会直接切 AUTO.LAND"
                 % (g["DESCEND_TIMEOUT"], need_d))
    xy_max = max(g["TRACK_XY_MAX"], g["TELEOP_XY_MAX"]) + g["ENVELOPE_MARGIN"]
    need_r = xy_max / g["RETURN_SPEED"] * 1.2 + 3
    if g["RETURN_TIMEOUT"] < need_r:
        W.append("返航超时 %.0fs < 从围栏边返航约需 %.0fs：超时会就地降落"
                 % (g["RETURN_TIMEOUT"], need_r))
    if g["TRACK_NOINPUT_MAX"] > 180:
        W.append("TRACK_NOINPUT_MAX=%.0fs 太长：SSH 假死要这么久才发现"
                 % g["TRACK_NOINPUT_MAX"])

    # ---------------- 频率 ----------------
    if g["SP_PUB_HZ"] <= 2:
        E.append("SP_PUB_HZ=%s ≤ 2：PX4 会退出 OFFBOARD" % g["SP_PUB_HZ"])
    if g["CTRL_HZ"] % g["TRACK_LOG_HZ"] != 0:
        W.append("TRACK_LOG_HZ=%s 不能整除 CTRL_HZ=%s，实际日志频率会打折"
                 % (g["TRACK_LOG_HZ"], g["CTRL_HZ"]))
    return E, W


def _gate_preview(g):
    """穿环的静态派生量 (gate_plan.static_preview)。gate_plan.py 不在 → 抛 ImportError。"""
    import gate_plan
    return gate_plan.static_preview(gate_plan.params_from_globals(g)), gate_plan


def _check_gate(g):
    """穿环 (stage12)。场地/机体在起飞后按环的实际位置再算一次 (stage12 的 PLAN 段),
    这里只查"正对进入"时就放不下的情况, 和与飞控/安全线的配合。"""
    E, W = [], []
    get = g.get
    _check_safety(g, E)
    _check_safety(g, E, GATE_SAFETY_RULES)
    if get("GATE_DASH_TILT_DEG", 0) < get("MAX_TILT_DEG", 0):
        E.append("冲刺段停桨倾角 %.0f° 比平时 %.0f° 还小" % (g["GATE_DASH_TILT_DEG"], g["MAX_TILT_DEG"]))

    # ---------------- 高度 ----------------
    tk, top = g["TAKEOFF_HEIGHT"], g["MAX_HEIGHT"]
    zlo, zhi = g["GATE_Z_MIN_DZ"], g["GATE_Z_MAX_DZ"]
    if tk < 0.5:
        E.append("起飞高度 %.2f < 0.5：离地判定不可靠" % tk)
    if tk > top - 0.5 + 1e-9:
        E.append("起飞高度 %.2f 离安全硬顶 %.2f 太近(<0.5m)" % (tk, top))
    if g["TAKEOFF_TIMEOUT"] < tk / 0.5 + 3.0:
        W.append("起飞超时 %.0fs 偏短(升 %.1fm 按 0.5m/s 估要 %.0fs)"
                 % (g["TAKEOFF_TIMEOUT"], tk, tk / 0.5))
    if zlo < 0.5 - 1e-9:
        E.append("瞄点高度下限 %.2f < 0.50：地效 + 桨风, 冲刺高度太低" % zlo)
    if not zlo < zhi:
        E.append("瞄点高度带 [%.2f, %.2f] 为空" % (zlo, zhi))
    if zhi > top - 0.5 + 1e-9:
        E.append("瞄点高度上限 %.2f 离安全硬顶 %.2f 太近(<0.5m)" % (zhi, top))

    # ---------------- 水平 ----------------
    hard = g["GATE_XY_MAX"] + g["ENVELOPE_MARGIN"]
    base = g["BASE_DRIFT_LIM"]
    if base < 0.30:
        E.append("BASE_DRIFT_LIM=%.2f < 0.30：正常悬停漂移就会触发缓降" % base)
    if base + 0.40 > hard + 1e-9:
        E.append("起降阶段水平限制(最大 %.2f m)比水平硬围栏 %.2f m 还大" % (base + 0.40, hard))

    # ---------------- 冲刺 ----------------
    v, vmin = g["DASH_V"], g["DASH_V_MIN"]
    if vmin > v + 1e-9:
        E.append("DASH_V_MIN=%.2f > DASH_V=%.2f" % (vmin, v))
    if g["V_TEAR_MIN"] > vmin + 1e-9:
        W.append("撕纸最低速度 %.2f > 降速下限 %.2f: 场地逼得降速时会先因为撞纸速度不够而放弃"
                 % (g["V_TEAR_MIN"], vmin))
    if g.get("M_BACK", 0.3) < 0.1:
        E.append("环后墙余量 M_BACK=%.2f m < 0.1 m" % g["M_BACK"])
    tilt_plan = math.degrees(math.atan(max(g["DASH_ACC"], g["BRAKE_ACC"]) / 9.81))
    if tilt_plan > g["MAX_TILT_DEG"] - 10.0:
        E.append("规划倾角 %.1f° (加/减速度 %.1f m/s²) 离停桨线 %.0f° 不到 10°"
                 % (tilt_plan, max(g["DASH_ACC"], g["BRAKE_ACC"]), g["MAX_TILT_DEG"]))
    if get("D_RUN_MAX", 0) > get("GATE_D_MAX", 99) - 1.0:
        W.append("最远起跑距离 %.1f m 离识别上限 %.1f m 不到 1 m: 起跑点上锁定会很难收敛"
                 % (g["D_RUN_MAX"], g["GATE_D_MAX"]))
    if g["D_ALIGN_MIN"] < 2.4:
        W.append("D_ALIGN_MIN=%.2f < 2.4: 起跑点离环太近时识别相机看不到整个环, 锁定会用残缺的环"
                 % g["D_ALIGN_MIN"])
    try:
        pv, gp = _gate_preview(g)
    except ImportError:
        E.append("找不到 gate_plan.py —— 放到飞行脚本同目录")
        pv, gp = None, None
    if pv is not None:
        if gp.r_safe(gp.params_from_globals(g)) <= 0.0:
            E.append("半宽 %.2f m: 机体放不进内环 (内半径 %.2f, 留 %.2f 余量)"
                     % (g["W_HALF"], g["R_IN"], g["AIM_MARGIN"]))
        # ★2026-10-21 刹车点 = 机头过纸 PASS_RUN 之后 (+滞后补偿)。环后空间决定能冲多快
        gpp = gp.params_from_globals(g)
        vmin_need = gp.dash_stop(g["DASH_V_MIN"], gpp)["d_back_need"]
        pol = int(get("GATE_SLOW_POLICY", 1))
        slow_ok = pol >= 1
        if pv.get("v_room") is None:
            E.append("环后空间 D_BACK=%.2f m 不够: 过纸冲刺 %.2f m + 刹车超调, 降到 %.2f m/s 也要 ≥ %.2f m "
                     "—— 加大环后空间, 或减小 pass_run (过纸冲刺) / back_margin (环后墙余量)"
                     % (g["D_BACK"], g["PASS_RUN"], g["DASH_V_MIN"], vmin_need))
        elif pv["v_room"] < g["DASH_V"] - 1e-9:
            msg = ("环后空间 D_BACK=%.2f m 只够冲 %.2f m/s, 设定 %.2f。按设定速度冲需要 D_BACK ≥ %.2f m"
                   % (g["D_BACK"], pv["v_room"], g["DASH_V"], pv["d_back_need"]))
            if pol == 2:
                W.append(msg + " (allow_slow = 是: 起飞后会自动降到 %.2f)" % pv["v_room"])
            elif pol == 1:
                W.append(msg + " (allow_slow = 确认: 起飞后降到 %.2f, 冲刺前要按 Y 确认)" % pv["v_room"])
            else:
                E.append(msg + " —— 不允许降速 (allow_slow = 否): 把 dash_v 改成 %.2f, "
                               "或加大环后空间, 或 allow_slow = 确认" % pv["v_room"])
        ps = gp.pass_short(gpp)
        if ps:
            W.append(ps)
        vo = get("GATE_VIRTUAL_OFFSET") or 0
        if vo > 0:
            vv = gp.dash_v_room(gpp, d_back=vo)
            if vv is None:
                E.append("虚拟环偏移 %.2f m (真纸面当墙) 放不下冲刺: 降到 %.2f m/s 也要偏移 ≥ %.2f m"
                         % (vo, g["DASH_V_MIN"], vmin_need))
            else:
                if vv >= g["DASH_V"] - 1e-9:
                    W.append("虚拟环偏移 %.2f m 够按 %.2f m/s 冲" % (vo, g["DASH_V"]))
                elif slow_ok:
                    W.append("虚拟环偏移 %.2f m 只够冲 %.2f m/s (会降速%s)"
                             % (vo, vv, ", 冲刺前按 Y 确认" if pol == 1 else ""))
                else:
                    E.append("虚拟环偏移 %.2f m 只够冲 %.2f m/s, 设定 %.2f 且不允许自动降速 —— "
                             "加大偏移, 或 dash_v 改成 %.2f" % (vo, vv, g["DASH_V"], vv))
        if pv.get("error"):
            E.append("%s轨迹放不下: %s" % ("冲刺", pv["error"]))
        elif pv["v_hit_worst"] < g["V_TEAR_MIN"] - 1e-3:     # 滞后模型渐近到 v, 留数值余量
            W.append("正对进入时最坏撞纸速度 %.2f < 撕纸下限 %.2f m/s —— 起飞后的规划会拒绝"
                     % (pv["v_hit_worst"], g["V_TEAR_MIN"]))
    pe = get("PHASE_ENVELOPE")
    if isinstance(pe, dict) and "gate" in pe:
        if pe["gate"][1] < 1.7 * v:
            E.append("gate 速度包络 %.2f m/s < 1.7×%s速度 %.2f m/s：一%s就会触发缓降"
                     % (pe["gate"][1], "冲刺", 1.7 * v, "冲刺"))
    elif isinstance(pe, dict):
        E.append("PHASE_ENVELOPE 里没有 gate —— 冲刺会落到默认包络 (1 m/s) 一冲就缓降")

    # ---------------- 开关 ----------------
    if get("GATE_ALLOW_SIM_DRIVE"):
        W.append("GATE_ALLOW_SIM_DRIVE=True —— 标了 sim 的识别结果也能驱动冲刺！只在台架测试时用")
    if get("GATE_SHADOW"):
        W.append("GATE_SHADOW=True: 影子模式 —— 识别、锁定、规划照做, 不摆位、不冲刺; 做完回手动段 (可换位置再 T), L 降落")
    if (get("GATE_VIRTUAL_OFFSET") or 0) > 0:
        W.append("虚拟环 GATE_VIRTUAL_OFFSET=%.2f m: 真冲刺, 停在纸前 (T3 测试)"
                 % g["GATE_VIRTUAL_OFFSET"])
    if not get("GATE_CONFIRM_DASH", True):
        W.append("GATE_CONFIRM_DASH=False: 锁定后不等按 G, 直接冲刺")

    # ---------------- 时间 / 频率 ----------------
    need_d = (top - g["DESCEND_Z_TARGET_DZ"]) / g["DESCEND_SPEED"] * 1.2 + 3
    if g["DESCEND_TIMEOUT"] < need_d:
        W.append("降落超时 %.0fs < 从硬顶降下来约需 %.0fs：超时会直接切 AUTO.LAND"
                 % (g["DESCEND_TIMEOUT"], need_d))
    if g["SP_PUB_HZ"] <= 2:
        E.append("SP_PUB_HZ=%s ≤ 2：PX4 会退出 OFFBOARD" % g["SP_PUB_HZ"])
    return E, W


def _derived_gate(g):
    """穿环的 derived (INTERFACE_gate.md §4)。停靠跟踪专用的字段一律不给 (null)。"""
    pe = g.get("PHASE_ENVELOPE") or {}
    try:
        pv, gp = _gate_preview(g)
    except ImportError:
        pv = {}
    gate = {"r_in": g["R_IN"], "r_out": g.get("R_OUT", 0.8), "dot_d": g.get("DOT_D", 0.11)}
    gate.update(pv)
    env_v = (pe.get("gate") or [None, None])[1]
    chain = [["DASH_V", g["DASH_V"]]]
    if env_v:
        chain.append(["gate 速度硬限 ÷1.7", env_v / 1.7])
    lo, hi = g["GATE_Z_MIN_DZ"], g["GATE_Z_MAX_DZ"]
    return {
        "task": "gate",
        "z_band": [lo, hi],
        "z_clamp": [lo, hi],
        "teleop_dz": None,
        "max_height": g["MAX_HEIGHT"],
        "takeoff_height": g["TAKEOFF_HEIGHT"],
        "xy_soft": g["GATE_XY_MAX"],
        "xy_hard": g["GATE_XY_MAX"] + g["ENVELOPE_MARGIN"],
        "base_drift": g["BASE_DRIFT_LIM"],
        "standoff_shell": None, "detect_range": None, "entry_dist": None,
        "approach_band": None, "leash": None,
        "phase_envelope": {k: list(v) for k, v in pe.items()},
        "timeouts": {k: g[k] for k in ("TAKEOFF_TIMEOUT", "DESCEND_TIMEOUT")
                     if k in g},
        "speeds": {k: g[k] for k in ("DASH_V", "DASH_ACC", "BRAKE_ACC", "GATE_REPO_V",
                                     "DESCEND_SPEED") if k in g},
        "safety": {p: g.get(p) for p, _, _, _, _ in SAFETY_RULES + GATE_SAFETY_RULES},
        "switches": {k: g.get(k) for k in ("GATE_SHADOW", "GATE_ALLOW_SIM_DRIVE",
                                           "GATE_CONFIRM_DASH", "GATE_VIRTUAL_OFFSET",
                                           "GATE_REQUIRE_NORMAL", "GATE_MANUAL_START",
                                           "GATE_ABORT_TO_MANUAL",
                                           "GATE_SLOW_POLICY")},
        "site": {k: g.get(k) for k in ("D_BACK", "D_REAR", "D_SIDE", "L_F", "L_R", "W_HALF",
                                       "D_RUN_MAX", "PASS_RUN", "M_BACK")},
        "gate": gate,
        "speed_chain": chain,
    }


def derived_snapshot(g):
    """生效包络的关键量（给 dry-run 文本/JSON 和网页预览用）。"""
    if script_task(g) == "gate":
        return _round_floats(_derived_gate(g))
    nom, tol = g["TARGET_Z_NOMINAL"], g["TARGET_Z_TOLERANCE"]
    pe = g.get("PHASE_ENVELOPE") or {}
    return _round_floats({
        "z_band": [nom - tol, nom + tol],
        "z_clamp": [g["TRACK_Z_MIN"], g["TRACK_Z_MAX"]],
        "teleop_dz": [g["TELEOP_DZ_MIN"], g["TELEOP_DZ_MAX"]],
        "max_height": g["MAX_HEIGHT"],
        "takeoff_height": g["TAKEOFF_HEIGHT"],
        "xy_soft": g["TRACK_XY_MAX"],
        "xy_hard": g["TRACK_XY_MAX"] + g["ENVELOPE_MARGIN"],
        "base_drift": g["BASE_DRIFT_LIM"],
        "standoff_shell": [g["TRACK_STANDOFF"] - g["TRACK_DEADBAND"],
                           g["TRACK_STANDOFF"] + g["TRACK_DEADBAND"]],
        "detect_range": [g["DETECT_RANGE_MIN"], g["DETECT_RANGE_MAX"]],
        "entry_dist": g["TRACK_ENTRY_MAX_D"],
        "approach_band": g["DETECT_RANGE_MAX"]
                         - (g["TRACK_STANDOFF"] + g["TRACK_DEADBAND"]),
        "leash": [g["TRACK_LEASH_MAX"], g["TRACK_Z_LEASH"]],
        "phase_envelope": {k: list(v) for k, v in pe.items()},
        "timeouts": {k: g[k] for k in ("TAKEOFF_TIMEOUT", "TELEOP_DURATION",
                                       "TRACK_DURATION", "TRACK_NOINPUT_MAX",
                                       "RETURN_TIMEOUT", "DESCEND_TIMEOUT")},
        "speeds": {k: g[k] for k in ("TRACK_V_MAX", "TRACK_VZ_MAX",
                                     "TELEOP_XY_SPEED", "TELEOP_Z_SPEED",
                                     "RETURN_SPEED", "DESCEND_SPEED")},
        "safety": {p: g.get(p) for p, _, _, _, _ in SAFETY_RULES},
        "switches": {k: g.get(k) for k in ("TRACK_SHADOW", "TRACK_ALLOW_SIM_DRIVE",
                                           "TRACK_Z_ENABLE", "TRACK_VZ_FEEDFORWARD",
                                           "PREFLIGHT_REQUIRE_VISION")},
    })


def _round_floats(x, nd=4):
    """去掉 0.6999999 这类浮点噪声（只影响显示/JSON，不影响生效值）。"""
    if isinstance(x, bool):
        return x
    if isinstance(x, float):
        return round(x, nd)
    if isinstance(x, dict):
        return {k: _round_floats(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_round_floats(v, nd) for v in x]
    return x


def finalize(report, g):
    """
    在 stage11_real.py **派生量之后**调用。对最终生效值做全部校验:
      错误 → 拒绝起飞 (exit 1)
      dry-run → 打印(文本或 JSON)后 exit 0
    否则把警告并进 report，返回给启动摘要用。
    """
    try:
        e, w = check_effective(g)
    except Exception as ex:          # 校验自己出错 = 无法证明安全 = 拒绝起飞
        e, w = ["校验过程出错 (%s: %s) —— 两个文件版本不配套, 或参数值异常"
                % (type(ex).__name__, ex)], []
    report["errors"] += e
    for x in w:
        if x not in report["warnings"]:
            report["warnings"].append(x)
    try:
        report["derived"] = derived_snapshot(g)
    except Exception as ex:
        report["derived"] = None
        report["errors"].append("生成包络摘要出错 (%s: %s)" % (type(ex).__name__, ex))
    if report["errors"]:
        _die(report, g)
    if report["dry_run"]:
        if report["json"]:
            _print_json(report, g, ok=True)
        else:
            print_envelope(report, g)
        sys.exit(0)
    return report


# =============================================================================
# 输出
# =============================================================================
def _params_dict(g):
    return {k: g[k] for k in sorted(allowed_params(g))}


def _print_json(report, g, ok):
    out = {
        "ok": ok,
        "task": script_task(g) if g is not None else report.get("task"),
        "src": report.get("src"),
        "site": report.get("site"),
        "knobs": report.get("knobs", {}),
        "sets": report.get("sets", []),
        "applied": [[k, v] for k, v in report.get("applied", [])],
        "lines": report.get("lines", []),
        "warnings": report.get("warnings", []),
        "errors": report.get("errors", []),
        "derived": report.get("derived"),
        "params": _params_dict(g) if g is not None else {},
    }
    if out["derived"] is None and g is not None:
        try:
            out["derived"] = derived_snapshot(g)
        except Exception:
            out["derived"] = None
    print(json.dumps(out, ensure_ascii=False, indent=1))


def _die(report, g=None):
    if report.get("json"):
        _print_json(report, g, ok=False)
        sys.exit(1)
    print("\n" + "!" * 78)
    print("现场档案校验不通过 —— **拒绝起飞**（宁可现在尴尬 3 分钟）")
    print("档案来源: %s" % report.get("src"))
    for e in report["errors"]:
        print("  ✗ " + e)
    for w in report.get("warnings", []):
        print("  ⚠️ " + w)
    print("!" * 78 + "\n")
    sys.exit(1)


def print_envelope(report, g):
    """打印"这次生效的包络"。现场起飞前核对就看这一块。"""
    d = report.get("derived") or derived_snapshot(g)
    if script_task(g) == "gate":
        return _print_envelope_gate(report, g, d)
    print("=" * 78)
    print("现场档案: %s" % report["src"])
    for l in report["lines"]:
        print("  · " + l)
    if report["applied"]:
        print("  · 实际覆盖参数 %d 个:" % len(report["applied"]))
        for k, v in report["applied"]:
            print("      %-22s = %s" % (k, _fmt(v)))
    print("-" * 78)
    print("生效包络核对表:")
    print("    高度带        [%.2f, %.2f] m   (门限 %.2f ± %.2f)"
          % (d["z_band"][0], d["z_band"][1],
             g["TARGET_Z_NOMINAL"], g["TARGET_Z_TOLERANCE"]))
    print("    高度钳位      [%.2f, %.2f] m   手动 [%.2f, %.2f] m   安全硬顶 %.2f m"
          % (d["z_clamp"][0], d["z_clamp"][1], d["teleop_dz"][0], d["teleop_dz"][1],
             d["max_height"]))
    print("    起飞高度      %.2f m" % d["takeoff_height"])
    print("    水平范围      软限 %.2f m / 硬围栏 %.2f m / 起降漂移 %.2f m"
          % (d["xy_soft"], d["xy_hard"], d["base_drift"]))
    print("    目标距离      停靠壳 [%.2f, %.2f] m  检测 [%.2f, %.2f] m  "
          "进场门 %.2f m  可用带 %.2f m"
          % (d["standoff_shell"][0], d["standoff_shell"][1],
             d["detect_range"][0], d["detect_range"][1],
             d["entry_dist"], d["approach_band"]))
    sp = d["speeds"]
    print("    速度          跟踪 %.2f / 垂直 %.2f   手动 %.2f / %.2f   "
          "返航 %.2f  降落 %.2f m/s"
          % (sp["TRACK_V_MAX"], sp["TRACK_VZ_MAX"], sp["TELEOP_XY_SPEED"],
             sp["TELEOP_Z_SPEED"], sp["RETURN_SPEED"], sp["DESCEND_SPEED"]))
    print("    牵引绳        水平 %.2f m / 垂直 %.2f m" % tuple(d["leash"]))
    t = d["timeouts"]
    print("    时间          起飞 %.0fs  手动 %.0fs  跟踪 %.0fs  无按键 %.0fs  "
          "返航 %.0fs  降落 %.0fs"
          % (t["TAKEOFF_TIMEOUT"], t["TELEOP_DURATION"], t["TRACK_DURATION"],
             t["TRACK_NOINPUT_MAX"], t["RETURN_TIMEOUT"], t["DESCEND_TIMEOUT"]))
    if d["phase_envelope"]:
        print("    阶段包络      (水平硬限 m / 速度硬限 m/s)")
        for name in ("takeoff", "hover", "teleop", "track", "return_home",
                     "descend", "auto_land"):
            if name in d["phase_envelope"]:
                h, v = d["phase_envelope"][name]
                print("        %-12s %6.2f / %.2f" % (name, h, v))
    s = d["safety"]
    print("    安全          倾角停桨 %.0f°  去抖 %.2fs  视觉断供 %.2fs 后缓降  "
          "围栏违规缓降 %s"
          % (s["MAX_TILT_DEG"], s["FATAL_CONFIRM_S"], s["VISION_LOST_LAND_S"],
             s["FENCE_VIOLATION_LANDS"]))
    sw = d["switches"]
    print("    开关          影子 %s  假数据驱动 %s  z环 %s  z前馈 %s  起飞前要视觉 %s"
          % (sw["TRACK_SHADOW"], sw["TRACK_ALLOW_SIM_DRIVE"], sw["TRACK_Z_ENABLE"],
             sw.get("TRACK_VZ_FEEDFORWARD"), sw["PREFLIGHT_REQUIRE_VISION"]))
    if report["warnings"]:
        print("-" * 78)
        for w in report["warnings"]:
            print("  ⚠️ " + w)
    print("=" * 78)
    print("校验通过。")


def _start_line(gt):
    """起跑点说明。run_max ≤ 最小助跑 (比如 0) 时不是"动态", 而是总会飞到离环最小助跑处。"""
    lo, hi = gt["d_align"], gt.get("d_run_max", gt["d_align"])
    if hi <= lo + 1e-6:
        return ("起跑点        固定: 按 T 后总会飞到离环 %.2f m 处起跑 (run_max ≤ 最小助跑; "
                "想就地起跑把 run_max 设成 ≥ 起飞点离环的距离)" % lo)
    return "起跑点        动态: 离环 [%.2f, %.2f] m 就地起跑, 近了后退 / 远了前进" % (lo, hi)


def _gate_lines(g, d):
    """穿环包络核对表 (dry-run 文本 / site_config.txt 共用)。"""
    gt = d.get("gate") or {}
    pe = d.get("phase_envelope") or {}
    L = []
    L.append("任务          穿环 (stage12)")
    L.append("高度          瞄点带 [%.2f, %.2f] m   起飞 %.2f m   安全硬顶 %.2f m"
             % (d["z_band"][0], d["z_band"][1], d["takeoff_height"], d["max_height"]))
    L.append("水平          范围 %.2f m / 硬围栏 %.2f m / 起降漂移 %.2f m"
             % (d["xy_soft"], d["xy_hard"], d["base_drift"]))
    st = d.get("site") or {}
    L.append("场地          环后 %.2f m  起飞点后方 %.2f m  两侧 %.2f m"
             % (st.get("D_BACK"), st.get("D_REAR"), st.get("D_SIDE")))
    L.append("机体          机头 %.2f m  机尾 %.2f m  半宽 %.2f m  → 瞄点离环心 ≤ %.2f m"
             % (st.get("L_F"), st.get("L_R"), st.get("W_HALF"), gt.get("r_safe", float("nan"))))
    if "s_brake" in gt:
        vp = gt.get("v_plan", gt["dash_v"])
        L.append("冲刺(正对)    %.2f m/s%s  加 %.1f / 减 %.1f m/s²  助跑 %.2f m  用时 %.2f s"
                 % (vp, ("" if abs(vp - gt["dash_v"]) < 1e-9 else
                         " (环后空间限速, 设定 %.2f)" % gt["dash_v"]),
                    gt["dash_acc"], gt["brake_acc"], gt["d_align"], gt["t_dash"]))
        L.append("过纸冲刺      机头碰纸后再冲 %.2f m 才刹车 (滞后补偿 %.2f m)  刹车起点 s=%+.2f  停止 s=%+.2f"
                 % (gt["pass_run"], gt["pass_lag"], gt["s_brake"], gt["overshoot"]))
        L.append("环后墙        含刹车超调 %.2f m 后机头离墙 %.2f m (要求 ≥ %.2f, 再富余 %.2f)   按 %.2f m/s 冲要 D_BACK ≥ %.2f m"
                 % (gt.get("brake_over", 0.0), gt["back_margin"] + gt["tol"], gt["back_margin"], gt["tol"],
                    gt["dash_v"], gt["d_back_need"]))
        L.append("降速          %s" % {0: "否: 场地放不下设定速度时规划失败、回手动段",
                                       1: "确认: 场地放不下时降到能飞的速度, 冲刺前要按 Y",
                                       2: "是: 场地放不下时自动降速 (会告警)"}.get(gt.get("slow_policy", 1), "?")
                 + "   (起飞前 preflight 会用地面看到的环先算一遍)")
        L.append(_start_line(gt))
    if "gate" in pe:
        L.append("阶段包络      gate %.2f m / %.2f m/s" % tuple(pe["gate"]))
    s = d["safety"]
    L.append("安全          倾角停桨 %.0f°  去抖 %.2fs  (冲刺段 %s° / %ss)  视觉断供 %.2fs 后缓降"
             % (s["MAX_TILT_DEG"], s["FATAL_CONFIRM_S"], _fmt(s.get("GATE_DASH_TILT_DEG")),
                _fmt(s.get("GATE_DASH_TILT_CONFIRM_S")), s["VISION_LOST_LAND_S"]))
    sw = d["switches"]
    L.append("开关          影子 %s  假数据驱动 %s  冲刺前按 G %s  虚拟环 %s m  必须有法向 %s"
             % (sw.get("GATE_SHADOW"), sw.get("GATE_ALLOW_SIM_DRIVE"), sw.get("GATE_CONFIRM_DASH"),
                sw.get("GATE_VIRTUAL_OFFSET"), sw.get("GATE_REQUIRE_NORMAL")))
    L.append("              手动段起步 %s  放弃后回手动段 %s"
             % (sw.get("GATE_MANUAL_START"), sw.get("GATE_ABORT_TO_MANUAL")))
    return L


def _print_envelope_gate(report, g, d):
    print("=" * 78)
    print("现场档案: %s" % report["src"])
    for l in report["lines"]:
        print("  · " + l)
    if report["applied"]:
        print("  · 实际覆盖参数 %d 个:" % len(report["applied"]))
        for k, v in report["applied"]:
            print("      %-22s = %s" % (k, _fmt(v)))
    print("-" * 78)
    print("生效包络核对表:")
    for l in _gate_lines(g, d):
        print("    " + l)
    if report["warnings"]:
        print("-" * 78)
        for w in report["warnings"]:
            print("  ⚠️ " + w)
    print("=" * 78)
    print("校验通过。")


def site_config_text(report, g, extra=None):
    """写进飞行日志目录的 site_config.txt 内容。"""
    d = report.get("derived") or derived_snapshot(g)
    if script_task(g) == "gate":
        return _site_config_text_gate(report, g, d, extra)
    L = ["# 本次飞行生效的现场配置（自动生成，复盘用）",
         "# 档案来源: %s" % report["src"], ""]
    if report["knobs"]:
        L.append("[旋钮]")
        for k in sorted(report["knobs"]):
            L.append("  %-12s = %s" % (k, report["knobs"][k]))
        L.append("")
    if report.get("sets"):
        L.append("[原始参数覆盖 (档案 set. + 命令行 --set, 后者优先)]")
        for s in report["sets"]:
            L.append("  " + s)
        L.append("")
    L.append("[推导出的参数覆盖]")
    for k, v in report["applied"]:
        L.append("  %-22s = %s" % (k, _fmt(v)))
    L.append("")
    L.append("[生效包络]")
    L.append("  高度带        [%.2f, %.2f] m" % tuple(d["z_band"]))
    L.append("  高度钳位      [%.2f, %.2f] m   硬顶 %.2f m"
             % (d["z_clamp"][0], d["z_clamp"][1], d["max_height"]))
    L.append("  起飞高度      %.2f m" % d["takeoff_height"])
    L.append("  水平范围      软限 %.2f / 硬围栏 %.2f / 起降漂移 %.2f m"
             % (d["xy_soft"], d["xy_hard"], d["base_drift"]))
    L.append("  目标距离      停靠壳 [%.2f, %.2f]  检测远端 %.2f  进场门 %.2f m"
             % (d["standoff_shell"][0], d["standoff_shell"][1],
                d["detect_range"][1], d["entry_dist"]))
    t = d["timeouts"]
    L.append("  时间          起飞 %.0f / 手动 %.0f / 跟踪 %.0f / 无按键 %.0f / "
             "返航 %.0f / 降落 %.0f s"
             % (t["TAKEOFF_TIMEOUT"], t["TELEOP_DURATION"], t["TRACK_DURATION"],
                t["TRACK_NOINPUT_MAX"], t["RETURN_TIMEOUT"], t["DESCEND_TIMEOUT"]))
    if d["phase_envelope"]:
        L.append("  阶段包络      " + "  ".join(
            "%s=%.2f/%.2f" % (k, v[0], v[1]) for k, v in d["phase_envelope"].items()))
    L.append("")
    L.append("[全部生效参数]")
    for k, v in _params_dict(g).items():
        L.append("  %-26s = %s" % (k, _fmt(v)))
    if report["warnings"]:
        L.append("")
        L.append("[警告]")
        for w in report["warnings"]:
            L.append("  ⚠️ " + w)
    if extra:
        L.append("")
        L += extra
    return "\n".join(L) + "\n"


def _site_config_text_gate(report, g, d, extra=None):
    L = ["# 本次飞行生效的现场配置（自动生成，复盘用）",
         "# 档案来源: %s" % report["src"], ""]
    if report["knobs"]:
        L.append("[旋钮]")
        for k in sorted(report["knobs"]):
            L.append("  %-14s = %s" % (k, report["knobs"][k]))
        L.append("")
    if report.get("sets"):
        L.append("[原始参数覆盖 (档案 set. + 命令行 --set, 后者优先)]")
        for s in report["sets"]:
            L.append("  " + s)
        L.append("")
    L.append("[推导出的参数覆盖]")
    for k, v in report["applied"]:
        L.append("  %-22s = %s" % (k, _fmt(v)))
    L.append("")
    L.append("[生效包络]")
    L += ["  " + x for x in _gate_lines(g, d)]
    L.append("")
    L.append("[全部生效参数]")
    for k, v in _params_dict(g).items():
        L.append("  %-26s = %s" % (k, _fmt(v)))
    if report["warnings"]:
        L.append("")
        L.append("[警告]")
        for w in report["warnings"]:
            L.append("  ⚠️ " + w)
    if extra:
        L.append("")
        L += extra
    return "\n".join(L) + "\n"


# =============================================================================
# 直接运行
# =============================================================================
if __name__ == "__main__":
    args = sys.argv[1:]
    d = DEFAULT_SITES_DIR
    if "--sites-dir" in args:
        i = args.index("--sites-dir")
        d = args[i + 1]
        del args[i:i + 2]
    if "--schema" in args:
        print(json.dumps(schema_json(), ensure_ascii=False, indent=1))
        sys.exit(0)
    if not args:
        print("用法: python3 flight_profile.py <站点名> [--sites-dir DIR]")
        print("      python3 flight_profile.py --schema")
        print("现有档案:")
        for n in list_sites(d):
            print("   " + n)
        sys.exit(0)
    name = args[0]
    knobs, sets, path, errs = load_site(name, d)
    if errs:
        _die({"src": path, "errors": errs, "warnings": []})
    e, w = validate_knobs(knobs)
    _task = TASK_NAMES.get(knobs.get("task"), "track")
    e += check_knob_task(knobs, _task, "任务「%s」" % (knobs.get("task") or "跟踪"))
    ov, rep, w2 = build_overrides(knobs, dict(_STANDALONE_DEFAULTS), _task)
    if sets:
        rep.append("档案 set. 覆盖(单独运行时不检查参数是否存在): " + ", ".join(sets))
    for x in e:
        print("  ✗ " + x)
    for x in w + w2:
        print("  ⚠️ " + x)
    if e:
        sys.exit(1)
    print("档案 %s 旋钮级检查通过" % path)
    for l in rep:
        print("  · " + l)
    print("  (具体参数覆盖 %d 个)" % len(ov))
    print("  ★ 这只是旋钮级检查。完整检查(含代码默认值和派生量)请用:")
    print("      python3 %s --site %s --dry-run"
          % ("stageNN_gate.py" if _task == "gate" else "stageNN_track.py", name))
