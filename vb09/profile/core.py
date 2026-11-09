# -*- coding: utf-8 -*-
"""
2026-10-30 第 5 步: 原 flight_profile.py 拆成 core (解析/格式化/--set/setup/finalize/JSON) 和
gate (穿环旋钮/校验/派生/打印); 代码原样, 互相引用加 _g. / _c. 前缀。根目录 flight_profile.py 是兼容壳。
vb09/profile/core.py —— 现场档案（site profile）：把「现场环境」翻译成飞行限制 (通用部分)
===============================================================================
它解决的问题
-------------------------------------------------------------------------------
换一个现场（室内 / 室外、场地大小、环后空间），要动的参数有二十多个，散在飞行脚本
CONFIG 区的各组里。**现场手改代码 = 必错**（2026-09-16 就出过一次：只改了一个上限，
真正起作用的那道门却没动，飞机整场不听使唤）。

所以这里只暴露「人能感知 / 能用卷尺量」的旋钮，其余全部由规则推导。
旋钮的完整定义（类型/单位/范围/说明）在下面的 KNOBS 表里 —— 它是**唯一来源**:
档案解析、范围校验、网页表单、写回 .conf 全部从这张表生成。

    共用        site_name task scene takeoff_h xy_radius(= 起跑点/制动终点的水平范围)
                z_low z_high (= 瞄点允许的高度带)
    穿环        dash_v run_max pass_run allow_slow  d_back back_margin d_rear d_side (场地, 卷尺量)
                l_f l_r w_half (机体, 实测)
    高级 (adv)  dash_acc brake_acc t_lag e_along v_tear_min entry_ang_max d_straight brake_over

检查 (_check_gate) 和派生量 (derived.gate / speed_chain) 用同目录 gate_plan.py 算,
和飞行时的规划是同一份代码。

★ 2026-10-30 清理: 戳点模式和停靠跟踪 (stage11) 已删除。
  老档案里的戳点旋钮 (REMOVED_KNOBS) → 警告并忽略; gate_mode = 戳点 / set.GATE_POKE = True → 报错。
  task 旋钮只剩 穿环 (task = 跟踪 → 报错)。

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
    site_name  = 一楼大厅
    task       = 穿环
    scene      = 室内
    takeoff_h  = 1.5
    z_low      = 0.8
    z_high     = 2.2
    xy_radius  = 10
    dash_v     = 2.5
    d_back     = 4.0
    d_rear     = 2.0
    d_side     = 2.0
    l_f        = 0.30
    l_r        = 0.30
    w_half     = 0.25

    # ★ 2026-10-09(2): 原始参数覆盖也可以写进档案, 和命令行 --set 完全等价,
    #   照样过全部校验。这样"档案 = 完整配置", 启动命令永远只有 --site 名字,
    #   不会因为少复制一段 --set 而悄悄变成另一套配置(比如影子模式丢了)。
    set.GATE_SHADOW         = True
    set.GATE_VIRTUAL_OFFSET = 2.0

命令行（跟在飞行脚本后面）
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
                                            <飞行脚本> --site 二楼 --dry-run）
    python3 flight_profile.py --schema      输出旋钮表 JSON（网页表单用）
===============================================================================
"""

import json
import math
import os
import re
import sys

from vb09.profile import gate as _g


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
#   tasks     ★2026-10-19 适用的任务。2026-10-30 起只有 "gate" (穿环); 网页按它过滤旋钮。
#             档案里给了本任务不用的旋钮 → 报错 (不静默忽略)。网页可按它隐藏无关旋钮。
#   adv       ★2026-10-19 细调项, 网页放进「高级」折叠区
#
# 档案里的 task 旋钮只是声明"这份档案是给哪种任务写的", 和脚本对不上 → 拒绝起飞。
TASK_NAMES = {"穿环": "gate"}
CORE_KNOBS = [
    dict(key="site_name", type="str", unit="", label="现场名称", group="现场",
         help="只用于打印和日志"),
    dict(key="task", type="choice", unit="", label="任务", group="现场",
         choices=["穿环"],
         help="这份档案给哪种任务用 (2026-10-30 起只剩 穿环; 停靠跟踪 stage11 已删除)"),
    dict(key="scene", type="choice", unit="", label="场景", group="现场",
         choices=["室内", "地面", "低空", "高空"],
         help="决定缓降类安全项和起降阶段的水平漂移限制"),

    dict(key="z_low", type="float", unit="m", label="目标最低高度", group="高度",
         min=0.5, min_why="低于 0.5m 会进地效 + 桨风吹目标板，PX4 落地检测也可能误判",
         warn_lo=(0.7, "偏低(建议 ≥0.7)：桨风会吹到手/板，测量会变毛"),
         help="瞄点允许的最低高度 (相对起飞点)"),
    dict(key="z_high", type="float", unit="m", label="目标最高高度", group="高度",
         min=0.8, max=20.0,
         warn_hi=(10.0, "超过 10m：要重新评估 VIO 高度表现 / 电池 / 法规"),
         help="瞄点允许的最高高度 (相对起飞点), 同时决定安全硬顶 (+1m)"),
    dict(key="takeoff_h", type="float", unit="m", label="起飞高度", group="高度",
         min=0.5, min_why="0.5 以下起飞判定/离地检测不可靠", max=5.0,
         warn_hi=(2.0, "起飞段是位置阶跃(没改成斜坡)，PX4 会以最大爬升率去追。"
                       "建议 ≤2.0，再高用手动段 R 键爬升"),
         help="起飞后先升到的高度"),

    dict(key="xy_radius", type="float", unit="m", label="水平范围", group="水平",
         min=0.5, max=50.0,
         help="允许离起飞点多远(软限)；硬围栏 = 软限 + 自动余量"),

]
# 穿环旋钮在 vb09/profile/gate.py (GATE_KNOBS), 表顺序 = 共用在前、穿环在后 (和拆分前同一个顺序)
KNOBS = CORE_KNOBS + _g.GATE_KNOBS
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


def script_task(g):
    """飞行脚本的任务。2026-10-30 起只有穿环 (停靠跟踪 stage11 已删除)。"""
    return "gate"

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

# 单独运行 flight_profile.py 时没有飞行脚本的全局可参考, 用这组保底值推导
_STANDALONE_DEFAULTS = {
    "DESCEND_SPEED": 0.50, "TAKEOFF_HEIGHT": 1.5, "MAX_HEIGHT": 11.0,
    # 穿环 (和 vb09/flight/config.py 的代码默认值一致)
    "DASH_V_MIN": 1.2, "D_BACK": 2.0, "D_REAR": 2.0, "D_SIDE": 2.0,
    "L_F": 0.28, "L_R": 0.28, "W_HALF": 0.28, "PASS_RUN": 1.0, "M_BACK": 0.3,
    "BRAKE_OVER_T": 0.3, "GATE_SLOW_POLICY": 1,
}

# ★ 2026-10-09(2): 必须为正数的参数。--set 能把它们改成 0 或负数, 而飞行脚本的
#   派生量区会拿它们做除数 —— 不先拦就会在派生量区抛 ZeroDivisionError, 网页只能看到
#   "dry-run 没有输出 JSON"。所以这一组在 setup() 里、派生量之前就检查。
#   只检查飞行脚本里真实存在的那些(g.get)。
POSITIVE_PARAMS = (
    "SP_PUB_HZ", "MONITOR_HZ", "LOG_HZ", "TIMING_LOG_HZ",
    "EKF_LOG_HZ", "PHASE_LOOP_HZ", "STATUS_HZ", "TELEMETRY_HZ",
    "TELEOP_XY_SPEED", "TELEOP_Z_SPEED", "DESCEND_SPEED", "TELEOP_XY_MAX",
    "MAX_HEIGHT", "TAKEOFF_TIMEOUT",
    "DESCEND_TIMEOUT", "AUTO_LAND_TIMEOUT", "AUTO_LAND_SPEED_EST",
    # ★ 2026-11-08 跟踪类目标的抵近 / 选点
    "DOT_SEL_SCORE", "DOT_SEL_HITS", "DOT_SEL_RATIO", "DOT_RIVAL_HITS_REL", "GATE_SELECT_TIMEOUT", "GATE_APPR_TURN_DPS", "GATE_APPR_LOST_S",
    "GATE_APPR_FAIL_S", "GATE_APPR_JUMP", "GATE_APPR_TIMEOUT", "GATE_REFINE_DY", "GATE_REFINE_WAIT_S", "GATE_RELOCK_CM",
    # ★ 2026-10-19 穿环 (stage12)
    "DASH_V", "DASH_V_MIN", "DASH_V_STEP", "DASH_ACC", "BRAKE_ACC", "D_ALIGN_MIN",
    "L_F", "L_R", "W_HALF", "R_IN", "D_BACK", "D_REAR", "D_SIDE", "GATE_XY_MAX",
    "GATE_REPO_V", "GATE_YAW_RATE", "GATE_HOLD_S", "HOVER_ACQ", "HOVER_ALIGN",
    "GATE_STATUS_MAX_AGE", "GATE_LOG_HZ",
    "PASS_RUN", "M_BACK",
    "GATE_DOCK_HOLD_S",          # 2026-10-31 停靠
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
    if isinstance(current, str) and not isinstance(value, bool):
        return str(value), None          # 只有旋钮推导能写字符串参数 (MISSION_MODE); --set 本来就只认数字/开关
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
    from vb09.flight import mode_spec as _ms
    sup = {}
    for name, kind in _g.TARGET_NAMES.items():
        t = _g.target_support(kind)
        sup[kind] = {m: list(v) for m, v in t.items()} if t else None
    return {"knobs": KNOBS, "scene_table": SCENE_TABLE,
            # 2026-11-06: 模式表 / 区表 (vb09/flight/mode_spec.py) + 目标 → 键 + 目标 × 模式支持表 (判定插件 MODES)
            "modes": _ms.schema(), "targets": [dict(label=k, key=v) for k, v in _g.TARGET_NAMES.items()],
            "target_support": sup,
            "safety_rules": [dict(param=p, rule=r, limit=l, kind=k, why=w)
                             for p, r, l, k, w in SAFETY_RULES]}


# =============================================================================
# 旋钮级检查（只看旋钮本身，不需要飞行脚本的全局）
# =============================================================================
def validate_knobs(knobs, inactive=()):
    """返回 (errors, warnings)。范围来自 KNOBS 表。
    inactive: 当前模式不用的旋钮 (别的区)。它们越界不算错误 —— 由 setup 降成提示并且不写进参数。"""
    errs, warns = [], []
    for k, v in knobs.items():
        if k in inactive:
            continue
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


def build_overrides(knobs, view, task="gate"):
    """
    把旋钮翻译成**具体参数覆盖**。返回 (overrides, report_lines, warnings)。
    没给的旋钮 = 用代码里的默认值（不改）。
    view = 当前参数视图(代码默认值 + --set)。推导里用到的速度/现有上限都从这里取。
    task 只剩 "gate" (参数保留给调用方兼容)。
    """
    return _g._build_gate_overrides(knobs, view)


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

    task = script_task(g)
    report["task"] = task
    report["errors"] += check_knob_task(knobs, task, os.path.basename(sys.argv[0] or "飞行脚本"))

    # 档案里的 set.KEY 和命令行 --set 走同一条路; 命令行优先级更高
    conf_ov, e2 = parse_sets(conf_sets, g, src="档案 set.")
    cli_ov, e3 = parse_sets(opts["sets"], g, src="--set")
    report["errors"] += e2 + e3
    sets = dict(conf_ov)
    sets.update(cli_ov)

    # ★2026-11-06 按模式分区: 当前模式不用的旋钮 (别的区) 保留在档案里, 但不校验范围、越界的不写进参数
    mode, inactive = _g.knob_activity(knobs, sets, g)
    report["mode"] = mode
    report["inactive"] = sorted(inactive)
    e1, w1 = validate_knobs(knobs, inactive)
    report["errors"] += e1
    report["warnings"] += w1
    if report["errors"]:
        _die(report, g)
    knobs, w4 = _g.drop_inactive_bad(knobs, inactive)
    report["warnings"] += w4

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
    """返回 (errors, warnings)。g = 飞行脚本的全局（含派生量）。"""
    return _g._check_gate(g)


def derived_snapshot(g):
    """生效包络的关键量（给 dry-run 文本/JSON 和网页预览用）。"""
    return _round_floats(_g._derived_gate(g))


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
    在飞行脚本**派生量之后**调用。对最终生效值做全部校验:
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
    return _g._print_envelope_gate(report, g, d)


def site_config_text(report, g, extra=None):
    """写进飞行日志目录的 site_config.txt 内容。"""
    d = report.get("derived") or derived_snapshot(g)
    return _g._site_config_text_gate(report, g, d, extra)
