# -*- coding: utf-8 -*-
"""全局配置：路径、模型、阈值。

所有"魔法数字"都放这里，改参数不用翻代码。
阈值来自 2026-11-02 的实测（见 docs/02_实测日志）。
"""

from pathlib import Path

# ---------- 路径 ----------
ROOT = Path(__file__).resolve().parents[2]      # 项目根目录
SAMPLES_DIR = ROOT / "data" / "samples"
RESULTS_DIR = ROOT / "data" / "results"

# ---------- VLM ----------
VLM_HOST = "http://127.0.0.1:11434"
VLM_MODEL = "openbmb/minicpm-v4.7:1b"
VLM_TIMEOUT = 600          # 秒；冷启 6~7 s，留足余量
VLM_KEEP_ALIVE = "30m"     # 让模型常驻显存，避免每次重载

VLM_PROMPT = (
    "这是从画面里裁出来的局部。只看画面正中心的红色物体，它属于哪一种？\n"
    "A：贴在白纸或玻璃上的圆形标记\n"
    "B：其他红色物体（管道、砖、招牌、叶子、贴纸、灯）\n"
    "只回答 A 或 B。"
)

# ---------- CV 候选 ----------
CHROMA_MIN = 0.18          # 归一化色度阈值 (R-max(G,B))/(R+G+B)，抗曝光
AREA_MIN = 20              # 候选最小面积（像素）★ 不能设大：7 px 的圆只有 ~38 px²
AREA_MAX_RATIO = 0.03      # 候选最大面积（占全图比例）
CIRC_MIN = 0.55            # 圆度 4πA/P²，1.0 = 正圆（候选门槛，别太高）

# ---------- 候选排序 ----------
# 实测：
#   · 合成集 6 背景 × 6 直径，按 cdiff 排 ⇒ 7 px 也 6/6 全中
#   · 真实 8 张，按 cdiff 排 ⇒ 真点进前三 2/2
#   · 按白底打分排只有 2/6（合成）—— 白底先验不是通用判据
RANK_BY = "cdiff"          # cdiff | circularity | score
USE_WHITE_GATE = False     # True = 只有"浅色底"的候选才送 VLM
CDIFF_MIN = 0.25           # 局部色度对比下限（低于这个不算"孤立红点"）

# ---------- 白底环形带（只作为特征输出，不再当门槛）----------
# score = 环带平均亮度 V̄ - 1.5 × 环带平均饱和度 S̄
# 红点贴白纸：V 高 S 低 ⇒ score 大；红砖/叶子/管道 ⇒ score 小或负
RING_INNER = 1.5           # 环带内径 = 1.5 × 半径
RING_OUTER = 2.6           # 环带外径 = 2.6 × 半径
RING_SCORE_MIN = 100.0     # 判为"浅色底"的经验阈值（仅打标，用于上报）
RING_V_MIN = 150.0         # 环带亮度下限（仅打标，不影响筛除）

# ---------- 性能 ----------# 环形带统计只对圆度最高的前 N 个候选做（否则大图会算几十秒）
MAX_STATS = 80

# ---------- 裁剪 ----------
CROP_SCALE = 2.2           # 裁剪半宽 = 2.2 × 等效直径
CROP_MIN_HALF = 48         # 最小半宽（像素）
CROP_MAX_HALF = 240        # 最大半宽（像素）
CROP_OUT_PX = 384          # 送模型前统一放大到 384×384

# ---------- 相机内参（2026-11-04 15:54 重新标定）----------
# 两套数，取决于我们吃的是哪种图：
#
#   USE_RECTIFIED = False → 原始左目（K1）。从相机直接取 1920×1080 的图用它。
#   USE_RECTIFIED = True  → 极线校正后的左目（P1）。如果哪天改用文豪
#                           vision_bridge 已经 remap 过的帧，必须切到这个。
#
# 两套差多少：fx 差 3.9%，cx 差 36 像素 ≈ 1.3°，30 m 处横向差 0.7 m。
# **搞错不会报错，只会系统性打偏**，所以做成开关 + 在读数里带出来。
USE_RECTIFIED = False

CALIB_W, CALIB_H = 1920, 1080

# K1 / P1（都是 1920×1080 下）
_K1 = (1566.1488, 1573.3822, 932.2771, 517.2610)    # 原始左目
_P1 = (1626.6183, 1626.6183, 968.4112, 514.6710)    # 校正后左目

FX, FY, CX, CY = _P1 if USE_RECTIFIED else _K1

# 畸变（K1 下；P1 已校正，不需要）
D1 = (-0.0211200, 0.0902393, -0.0073969, -0.0015599, 0.0)

# ---------- 相机安装角（s14/run_vision.py 的 CAM 常量）----------
# 2026-11-04 复核：**没有变**，仍是 pitch 5.23 / roll 3.40 / yaw −0.73
# 出处注释：2026-10-20 stage13_cam_calib 手持标定（33 工位, 2.9~5.3m），
#          pitch 5.23±0.25°，旧值 1.3° 少了约 4°
CAM_PITCH_DOWN_DEG = 5.23
CAM_ROLL_DEG = 3.40
CAM_YAW_DEG = -0.73

# ---------- 点击 → 飞行 ----------
CLICK_STEP_M = 3.0         # 一次点击默认前进多少米
CLICK_STANDOFF_M = 1.0     # 留出的安全距离（别冲到墙上）
WINDOW_REAL_W_M = 2.0
REQUIRE_ON_WINDOW = False  # True=只认贴在窗户上的红圆（真机时打开）
MARKER_REAL_M = 0.15   # ★用户 2026-11-04 尺量：红圆直径 15 cm（不是 A4 内接 20 cm）
MIN_STEP_M = 0.4
MAX_RANGE_M = 80.0      # 用来估距的窗户宽度（实测一次后改）

# ---------- 输出 ----------
MAX_CANDIDATES = 3         # JSON 里最多保留几个
DISPLAY_CANDIDATES = 5     # 界面上最多画几个红点候选         # 最多送 3 个候选给 VLM
VLM_MAX_CHECK = 2          # 最多实际调用 2 次（省时间）
