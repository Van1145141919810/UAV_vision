# -*- coding: utf-8 -*-
"""
vb09/flight/config.py —— 飞行脚本的全部参数 (原 stage12_gate.py 的 CONFIG 区, 2026-10-30 原样搬来)
===============================================================================
所有模块统一写 `C.DASH_V` (from vb09.flight import config as C), 不再用裸名。
入口脚本的顺序 (和以前在 stage12 模块级做的一样):
    report = flight_profile.setup(sys.argv, vars(C))   # 档案 + --set 写进这里
    C.derive()                                          # 派生量 (GATE_PRE_VEL_LIM / PHASE_ENVELOPE …)
    flight_profile.finalize(report, vars(C))            # 对最终生效值做全部校验; --dry-run 在这里退出
★ 这里的值是"不给档案时"的代码默认值 (默认没有 gate_params.json / 档案时就飞这些),
  重构时整块原样搬, 不许改。派生量在 derive() 之前**不存在** (--set 不能直接改它们, 和以前一样)。
"""
from vb09.flight import mode_spec as _ms     # 纯数据 (derive 按模式定速度包络)

# =============================================================================
# ############################   CONFIG (常改区)   ############################
# =============================================================================
# 现场请用 `--site <档案>` (flight_profile.py, 档案里 task = 穿环), 不要手改这里。
# 这里的值是"不给档案时"的代码默认值, 同样要过 flight_profile 的检查。
TASK                 = "gate"    # flight_profile / 网页靠它区分任务。不要改

# =============================================================================
# ========================  A. 高度与可飞范围  ========================
# =============================================================================
# 所有高度都相对起飞点 (INITIAL_Z)。
MAX_HEIGHT           = 3.5     # m，安全硬顶, 超过 (去抖后) → 缓降
TAKEOFF_HEIGHT       = 1.5     # m，起飞高度 (ACQUIRE 在这个高度识别; 起跑点高度 = 瞄点高度)
GATE_Z_MIN_DZ        = 0.5     # m，瞄点允许的最低高度 (档案 z_low)。低于它冲刺高度太低 → 不飞
GATE_Z_MAX_DZ        = 2.5     # m，瞄点允许的最高高度 (档案 z_high)
GATE_XY_MAX          = 8.0     # m，起跑点 / 制动终点离起飞点最远多少 (档案 xy_radius)
ENVELOPE_MARGIN      = 1.00    # m，水平硬围栏 = GATE_XY_MAX + 此余量
BASE_DRIFT_LIM       = 0.60    # m，起飞阶段水平硬限 (descend +0.2, auto_land +0.4)

# =============================================================================
# ========================  B. 环、机体、场地 (实测量)  ========================
# =============================================================================
R_IN                 = 0.60    # m，环内半径 (内径 1.2m)
R_OUT                = 0.80    # m，环外半径 (显示用)
DOT_D                = 0.11    # m，红点直径 (显示用)
# 机体 (含护桨 / 破纸杆), 档案 l_f / l_r / w_half。2026-10-19 实测: 带护罩 56×56cm, 高 22cm
#   (VINS 相机离地 11cm ≈ 机身半高, 识别相机离地 4cm)。装破纸杆后 L_F 要加上杆长
L_F                  = 0.28    # m，中心 → 机头
L_R                  = 0.28    # m，中心 → 机尾
W_HALF               = 0.28    # m，中心 → 最外侧
# 场地, 档案 d_back / d_rear / d_side。★ 每个现场卷尺量
D_BACK               = 2.0     # m，纸面 → 环后墙
D_REAR               = 2.0     # m，起飞点后方空地
D_SIDE               = 2.0     # m，环心 → 两侧障碍 (小的那边)

# =============================================================================
# ========================  C. 识别输入与锁定  ========================
# =============================================================================
GATE_STATUS_TOPIC    = "/gate/status"
CAM_STATUS_TOPIC     = "/vision_bridge/cam"
GATE_STATUS_MAX_AGE  = 0.50    # s，收到后多久算旧 (按本机收到时刻)
GATE_STATUS_STALE_S  = 0.30    # s，预检: 最新一条 /gate/status 收到至今超过它 → 警告 (地面检测停了?)
GATE_PREFLIGHT_TARGET_AGE_MS = 1000.0  # ms，预检: 桥里的环估计 age_ms 超过它 = 旧状态, 拒绝起飞
GATE_CAP_MAX_AGE     = 1.20    # s，拍摄时刻 t_cap 到现在最多多久 (链路飞行中可到 0.8s)
GATE_D_MIN           = 1.5     # m，识别距离可信范围
GATE_D_MAX           = 8.0     # m
GATE_VIEW_ANG_MAX    = 60.0    # °，视线和法向的最大夹角 (有法向时才查)
# σ / 离散度都按**靶面坐标**拆开判 (2026-10-19 首飞问题 §1): 横向 h / 竖直 v 决定能不能穿环,
#   沿进入方向 (深度) 只影响停止点。双目深度 σ ∝ 距离², 横向 ∝ 距离 —— 合在一起判会被深度卡死。
#   σ 来自 /gate/status 的 cov (没有就用 sd_cm 的对角), 投到 right / up / 进入方向上。
GATE_SD_MAX_CM       = 8.0     # cm，单帧能不能用: 横向 (h、v 的大者) σ 上限
GATE_SD_ALONG_MAX_CM = 20.0    # cm，单帧能不能用: 深度 σ 上限
GATE_DOT_PREFER      = True    # 桥的红点估计是双目红点 (dot_stereo / dot_ray_plane) 且刚更新过 → 样本用红点, 不用环心观测。
GATE_DOT_MAX_AGE_MS  = 300.0   # ms，红点估计最多这么旧还算"刚更新"
                               #   2026-10-21 flight_0021: 起跑点 4.0m 在双目角点平面范围 (≤3.5m) 外, 环心只有
                               #   单目 (还部分出画, σ×2) → 每帧 sd_along 被拒; 同时红点双目 σ 横 0.2/深 1.5cm 却没人用
                               #   (有 ring_w 就一律用 ring_w)。3 次都"有效帧 0 < 10"。
GATE_SD_ALONG_PER_M_CM = 7.0   # cm/m (2026-10-23: 4.5 → 7.0, 见下)，桥每帧的环心观测 (ring_w) 用的深度上限放宽到 max(上面, 这个 × 距离)。
                               #   2026-10-21 flight_0020: 远处只有单目环, 深度 σ = 3%·Z, Z>6.7m 就 >20cm →
                               #   7.2m 悬停时每一帧都被 sd_along 拒掉, 日志却写"还没看到环", 手动抵近到 6.5m 才行。
                               #   单帧深度 σ 大没关系: 锁定看的是多帧中值 (σ/√k) 和离散度, 那两道门没放松。
                               #   2026-10-23: 环部分出画时检测端单目 σ×2 (=6%·Z), 0021/0023 在 3.8~4.6m 每帧
                               #   都被拒 (又还没看到红点) → 只能手动推到 3.5m。7cm/m 让 partial 的单目环也能进。
GATE_LOCK_SD_LAT_CM  = 3.0     # cm，锁定: 窗口内横向 σ 中位数上限 (PLAN §3)
GATE_LOCK_SD_ALONG_CM = 10.0   # cm，锁定: 窗口内深度 σ 中位数上限 (≈ E_ALONG / 2.5; 虚拟环 2m 时在离真环 ~5m 处锁定, 深度 σ ≈ 8cm)
GATE_N_SD_MAX_DEG    = 3.0     # °，n_sd_deg (有才查)
GATE_ACQ_MIN_N       = 10      # ACQUIRE: 窗口内至少这么多有效帧
GATE_ACQ_SPREAD_CM   = 10.0    # cm，ACQUIRE 粗估的样本横向离散度上限 (p90−p10); 深度 ×2
GATE_ALIGN_SPREAD_CM = 5.0     # cm，ALIGN 锁定时样本横向离散度上限 (p90−p10, 环心和红点)
GATE_ALIGN_SPREAD_ALONG_CM = 12.0  # cm，ALIGN 锁定时样本深度离散度上限
GATE_N_AGREE_DEG     = 5.0     # °，ACQUIRE 和 ALIGN 两个视角估的法向夹角上限
GATE_REQUIRE_NORMAL  = False   # True = 没有法向 n 的识别帧一律不用 (bridge 补上 n 之后建议打开)
HOVER_ACQ            = 3.0     # s，ACQUIRE 取样窗口
HOVER_ALIGN          = 2.0     # s，ALIGN 取样时长
GATE_ALIGN_MAX_S     = 5.0     # s，ALIGN 最多延长到这么久, 还不收敛 → 放弃
GATE_ACQ_SCAN_AFTER  = 8.0     # s，这么久没看到环 → ±30° 慢扫一次
GATE_ACQ_TIMEOUT     = 25.0    # s，ACQUIRE 总超时 → 放弃 (降落)
GATE_SCAN_DEG        = 30.0    # °
# 两道 sim 保险 (桥的 sim 标志 + 影子模式)
GATE_SHADOW          = False   # True = 识别/锁定/规划照做, 不摆位不冲刺 (网页上看计划)。= 档案 mode = 影子
# 任务模式 (2026-10-30, 档案 mode): "dash" 冲刺 / "shadow" 影子 / "dock" 停靠 —— 选 vb09/flight/modes.py 的步骤列表。
#   字符串, 只能由档案的 mode 旋钮设 (--set 只认数字/开关)。旧写法 GATE_SHADOW = True 仍然有效 (= 影子)
MISSION_MODE         = "dash"
# 目标类型 (2026-11-01 接线): 选 vb09/perception/registry.py 里的判定 JUDGES[TARGET_KIND]。
#   字符串, 现在只有 "ring"; 桥 (_target:=) 和地面 (gcs_gate --target) 要一致
TARGET_KIND          = "ring"
GATE_ALLOW_SIM_DRIVE = False   # False = /gate/status 里 sim=true 的帧不许用来冲刺 (影子模式可以用)
# 停靠 (2026-10-31, 档案 mode = 停靠): 识别 → 飞到目标前方停住、对正 → 悬停记录检测 → 回手动段。不冲刺
GATE_DOCK_D_STOP     = 0.0     # m，停靠点离目标多远 (水平; 沿法向, 没有法向沿视线)。0 = 自动 GATE_DOCK_D_AUTO
GATE_DOCK_D_AUTO     = 3.0     # m，停靠距离不填时用这个。2026-11-06 以前是 max(3.0, D_ALIGN_MIN) —— 借了冲刺的参数
GATE_DOCK_HOLD_S     = 20.0    # s，在停靠点悬停多久 (期间每拍照常写 gate.csv)
# 上下扫描 (2026-11-05, 档案 vscan): 停靠点到位后先降 dz、再升到 +dz、回停靠点 —— 只改高度, 不横移。
#   给感知换视角: 红点的椭圆法向单个视角有两个解分不开, 视线变 ≥ 10° 才能定 (3 m 处 ±0.3 m ≈ 11°)。
#   高度夹在手动段高度带 [TELEOP_DZ_MIN, TELEOP_DZ_MAX] (相对起飞点) 里
GATE_VSCAN_DZ        = 0.0     # m，扫描半幅。0 = 不扫 (老行为)
GATE_VSCAN_V         = 0.2     # m/s，扫描速度
GATE_VSCAN_WAIT_S    = 2.0     # s，扫完回到停靠点后最多再等多久法向
GATE_VSCAN_SPAN_DEG  = 10.0    # °，预计视角跨度低于它就提示 (感知 normal_view_min_deg 同值)
# 跟踪类目标 (红点, 判定插件 TRACKED = True) 的步骤链 (2026-11-08, claude/PLAN_dot_v2.md):
#   Detect (发现 + 选点) → Approach (边飞边估, 每拍按最新估计重算停靠点) → Refine (够准就锁, 不够就侧移 / 上下换视角)
#   → 停靠: Hold  /  冲刺: PlanLocked → Commit (按 G 时估计变了就重锁) → Dash
DOT_D_MAX            = 14.0    # m，红点单帧判定的最远距离 (发现级; 环用 GATE_D_MAX)
DOT_SD_MAX_CM        = 40.0    # cm，红点单帧判定的横向 σ 上限 (发现级; 锁定线 GATE_LOCK_SD_LAT_CM 在 Refine 里判)
DOT_SD_ALONG_MAX_CM  = 300.0   # cm，… 深度 σ 上限 (10 m 处只有立体时 σ 50–80 cm 也要能抵近)
DOT_SEL_SCORE        = 0.5     # 自动选点: 最高分的候选至少这么高
DOT_SEL_HITS         = 5       # … 且至少命中这么多帧
DOT_SEL_RATIO        = 0.7     # 第二名分数 ≥ 此值 × 第一名 (且离开 > 0.5 m) → 分不清, 等人选 (终端数字键 1–5 / 网页点选)
DOT_RIVAL_HITS_REL   = 0.25    # … 而且第二名命中 ≥ 此值 × 第一名、有过双目、在瞄点高度带内才算 (flight_0051–0053: 远处无双目的假候选)
GATE_SELECT_TIMEOUT  = 30.0    # s，等人选点超时 → 放弃 (回手动段)
GATE_APPR_K          = 2.0     # 抵近: 停在 停靠距离 + k·σ_深度 处 (远处深度不准时停在估计的近端; 越近 σ 越小, 停靠点跟着往前挪)
GATE_APPR_TURN_DPS   = 8.0     # °/s，进入方向从视线转到法线的最大速率 (法向一出来停靠点不跳)
GATE_APPR_LOST_S     = 1.0     # s，这么久没有有效估计 → 悬停等
GATE_APPR_FAIL_S     = 5.0     # s，… 等这么久还没有 → 放弃 (回手动段)
GATE_APPR_JUMP       = 0.5     # m，估计一拍跳这么多 → 悬停 1 s 等它收敛
GATE_APPR_TIMEOUT    = 60.0    # s，抵近总时长上限
GATE_REFINE_DY       = 0.3     # m，Refine 不够准时左右侧移的半幅 (3 m 处 ±0.3 m ≈ 11° 视角; ±0.25 只有 9.5°, 法向要 ≥ 10°)
GATE_REFINE_ROUNDS   = 2       # Refine 最多换几轮视角 (侧移 / 上下), 还不够准 → 放弃 (回手动段)
GATE_REFINE_WAIT_S   = 2.0     # s，每次到位后取样多久再判
GATE_RELOCK_CM       = 5.0     # cm，等 G 期间最新估计和锁定值横向差超过它 → 按最新值重锁、重规划, 再等一次 G

# =============================================================================
# ========================  D. 冲刺规划 (gate_plan.py)  ========================
# =============================================================================
DASH_V               = 2.5     # m/s，巡航速度上限 (档案 dash_v)
DASH_V_MIN           = 1.2     # m/s，场地放不下时最多降到这么低, 再不行就放弃
DASH_V_STEP          = 0.3     # m/s，每次降多少
DASH_ACC             = 2.0     # m/s²，加速 (≈ 前倾 11.5°)
BRAKE_ACC            = 2.5     # m/s²，减速 (≈ 后仰 14.3°)
T_LAG                = 0.2     # s，控制滞后。用于过纸冲刺的补偿 + 估撞纸速度 (T2 实测后替换)
D_STRAIGHT           = 1.5     # m，加速完到纸面至少留的匀速直线
D_ALIGN_MIN          = 2.8     # m，起跑点离纸面最近多少 (识别相机 ≥2.4m 才看得到整个环)
D_RUN_MAX            = 4.0     # m，动态起跑点 (档案 run_max): 开始规划时离环在 [最小助跑, 它] 之间
                               #   就地起跑, 更远前进到它, 更近后退到最小助跑。0 = 总是最小助跑
ENTRY_ANG_MAX        = 20.0    # °，进入方向与法向最大夹角
E_ALONG              = 0.25    # m，纸面位置误差预算 (只用来估最坏撞纸速度; 2026-10-21 起不再决定刹车点)
V_TEAR_MIN           = 1.2     # m/s，撕纸最低速度 (撕纸测试 × 2)
M_PASS               = 0.2     # m，(旧方案的停止窗口下限用, 冲刺现在不用)
M_WALL               = 0.3     # m，离两侧 / 起飞点后方障碍的余量
# ---- 过纸冲刺 (2026-10-21): 机头碰纸后保持冲刺速度再冲 PASS_RUN 才刹车, 免得纸缠在机身上 ----
#   轨迹刹车起点 s_b = −L_F + PASS_RUN + DASH_V·T_LAG (后一项是滞后补偿, 飞机实际匀速过纸 ≈ PASS_RUN)
#   停止点 s_stop = s_b + v²/(2·BRAKE_ACC), 要满足 s_stop + L_F + M_BACK ≤ D_BACK, 放不下就降速。
#   冲刺档案里 L_F / L_R 只填机身 (含护桨, 不含针)。
PASS_RUN             = 1.0     # m，过纸冲刺距离 (档案 pass_run)
BRAKE_OVER_T         = 0.3     # s，刹车超调 ≈ 它 × 冲刺速度 (10-21 实测 2.0m/s 多冲 0.59m)。查离墙 / 水平范围时加上 (档案 brake_over)
GATE_SLOW_POLICY     = 1       # 场地放不下 DASH_V 时 (档案 allow_slow): 0 否 = 规划失败回手动段;
                               #   1 确认 = 降到能飞的速度, 冲刺前必须按 Y (G 不算); 2 是 = 自动降速
                               #   起飞前 preflight 还会用地面看到的环先算一遍 (见 [4/6])
M_BACK               = 0.3     # m，刹停后机头离环后墙至少多远 (档案 back_margin)
AIM_MARGIN           = 0.10    # m，机身截面 (2·W_HALF × 2·H_HALF) 四角离内圈至少留这么多
H_HALF               = 0.11    # m，机身半高 (含护桨; 2026-10-19 实测高 22cm)。机身扁, 上下比左右能多偏
# ---- 撞击点 (2026-10-19): 让针尖打红点/环心, 而不是机体中心 ----
#   机体中心 = 飞控位姿的原点 (mavros local_position)。站在机尾往机头看, 卷尺量针尖相对它的偏移。
#   例: 针在摄像头右上方, 针尖在机体中心右 6cm、上 8cm → NEEDLE_RIGHT=0.06  NEEDLE_UP=0.08
NEEDLE_RIGHT         = 0.0     # m，针尖在机体中心右边多少 (右为正)
NEEDLE_UP            = 0.0     # m，针尖比机体中心高多少 (上为正)
# 看纸上破洞的经验修正 (也能临时吃掉相机外参的偏差): 洞要往右挪 → 加正的 RIGHT, 往上挪 → 加正的 UP
#   例: 洞在红点左下 (左 4cm、下 10cm) → AIM_TRIM_RIGHT=+0.04  AIM_TRIM_UP=+0.10
AIM_TRIM_RIGHT       = 0.0     # m
AIM_TRIM_UP          = 0.0     # m
REPO_X_MAX           = -1.5    # m，摆位时飞机必须在环前方至少这么远 (环系 x ≤ 此值)
GATE_VIRTUAL_OFFSET  = 0.0     # m，> 0 = 虚拟环测试: 环平面往进近侧挪这么多, 真纸面当墙
GATE_ACC_FF          = True    # 冲刺下发加速度前馈 (False = 只给位置+速度)

# =============================================================================
# ========================  E. 摆位 / 提交 / 冲刺 / 收尾  ========================
# =============================================================================
GATE_REPO_V          = 0.6     # m/s，摆位速度 (setpoint 斜坡)
GATE_REPO_LEASH      = 0.6     # m，摆位 setpoint 领先实际位置的上限
GATE_REPO_TOL        = 0.10    # m，到达起跑点的容差
GATE_REPO_TIMEOUT    = 25.0    # s
GATE_YAW_RATE        = 0.3     # rad/s，所有转头的速率上限 (VINS 怕快转)
GATE_YAW_TOL_DEG     = 5.0     # °，转头到位容差
GATE_START_TOL       = 0.15    # m，冲刺起点: 飞机离 S 不超过这么多
GATE_START_V_MAX     = 0.15    # m/s，冲刺起点: 飞机速度不超过这么多
GATE_START_WAIT      = 3.0     # s，等不到"停稳在 S" → 放弃
GATE_CONFIRM_DASH    = True    # 锁定后等操作员按 G 才冲刺
# 手动段 (2026-10-19): 起飞后先手动, 按 T 才开始找环/规划 —— 起飞点不合适就先开过去
GATE_MANUAL_START    = True    # False = 起飞后直接找环 (老行为)
GATE_ABORT_TO_MANUAL = True    # 冲刺前放弃 (锁不住 / 不可行 / G 超时 / 按 M) → 回手动段; False = 降落
GATE_MANUAL_TIMEOUT  = 120.0   # s，手动段这么久没按 T → 降落 (按任何键都重新计时)
TELEOP_XY_SPEED      = 0.4     # m/s，手动平移
TELEOP_Z_SPEED       = 0.3     # m/s，手动升降
TELEOP_YAW_RATE      = 0.3     # rad/s，手动转向 (VINS 怕快转)
TELEOP_DEADMAN       = 0.3     # s，松键后速度归零
TELEOP_DZ_MIN        = 0.5     # m，手动段高度带 (相对起飞点)
GATE_CONFIRM_TIMEOUT = 30.0    # s，等 G 超时 → 放弃 (降落)
GATE_STOP_V          = 0.30    # m/s，轨迹结束后速度低于它算停稳
GATE_STOP_EXTRA      = 1.5     # s，轨迹结束后最多再等这么久 (等不到也进 HOLD, 记一笔)
GATE_HOLD_S          = 1.0     # s，刹停后在当前位置悬停多久再降
GATE_DEV_WARN        = 0.30    # m，冲刺中估计 s 和规划 s 差这么多 → 只记日志/告警
GATE_STUCK_ENABLE    = True
GATE_STUCK_DV        = 1.0     # m/s，实测速度比规划低这么多 (或实测 < GATE_STOP_V) …
GATE_STUCK_DS        = 0.40    # m，… 并且落后规划这么多 (两条同时满足) …
GATE_STUCK_HOLD      = 0.15    # s，… 持续这么久 → 判卡纸

# =============================================================================
# ========================  F. 各阶段时序  ========================
# =============================================================================
TAKEOFF_TIMEOUT      = 10.0    # s
TAKEOFF_REACH_TOL    = 0.20    # m
TAKEOFF_REACH_S      = 0.50    # s，连续到位时间
TAKEOFF_LIFTOFF_DZ   = 0.10    # m，判定离地
DESCEND_SPEED        = 0.50    # m/s
DESCEND_Z_TARGET_DZ  = 0.30    # m，下降到此高度后切 AUTO.LAND
DESCEND_TIMEOUT      = 45.0    # s
DESCEND_REACH_TOL    = 0.20    # m
PRIME_SECONDS        = 2.0     # s
POSE_WAIT_TIMEOUT    = 10.0    # s
OFFBOARD_TIMEOUT     = 1.5     # s
ARM_TIMEOUT          = 1.5     # s
ARM_MAX_TRIES        = 3
PARK_MODE            = "AUTO.LOITER"
AUTO_LAND_TIMEOUT    = 15.0    # s，下限; 实际按进入时高度放大
AUTO_LAND_SPEED_EST  = 0.50    # m/s
Z_FILTER_ALPHA       = 0.35    # z 低通 (监控 / 日志用, 不进控制)
ZDROP_SLOW_WINDOW    = 3.0     # s
ZDROP_SLOW_THRESH    = 0.10    # m
ZDROP_FAST_WINDOW    = 0.5     # s
ZDROP_FAST_THRESH    = 0.05    # m
ZDROP_REFRACTORY     = 2.0     # s
ZDROP_MIN_SAMPLES    = 5

# =============================================================================
# ========================  G. 安全防线 (flight_profile.SAFETY_RULES 守着)
# =============================================================================
MAX_TILT_DEG         = 30.0    # °，触发空中停桨。PX4 MPC_TILTMAX_AIR 必须 ≤ 此值 − 5
# 冲刺段单独的停桨线 (2026-10-19): 按 G 之后到刹停悬停结束。急刹 + 撞纸会有短时大倾角,
#   能自己回正; 真翻了 (>45° 还持续) 才停桨。其余阶段仍是 MAX_TILT_DEG
GATE_DASH_TILT_DEG   = 45.0    # °
GATE_DASH_TILT_CONFIRM_S = 0.30  # s
VEL_ENVELOPE_RATIO   = 2.0     # 阶段水平速度硬限 = max(基础值, 本系数 × 该阶段指令速度)
FATAL_CONFIRM_S      = 0.25    # s
FENCE_VIOLATION_LANDS = True
POSE_STALE_SEC       = 0.15    # s
POSE_JUMP_MAX        = 0.20    # m
VISION_LOST_LAND_S   = 0.80    # s
VISION_LOST_ENABLE   = True
STATIC_CHECK_S       = 3.0     # s
STATIC_XY_STD_MAX    = 0.015   # m
STATIC_Z_STD_MAX     = 0.030   # m
STATIC_TILT_MAX_DEG  = 3.0     # °
CELL_MIN_FLY         = 3.50    # V/芯
N_CELLS              = 4
CELL_V_SPLIT         = 18.0    # V
cell_count           = [N_CELLS]
PX4_TILT_MARGIN_DEG  = 5.0     # °，预检: MPC_TILTMAX_AIR ≤ MAX_TILT_DEG − 此值, 否则拒绝起飞

# =============================================================================
# ========================  H. 循环频率、日志与话题  ========================
# =============================================================================
SP_PUB_HZ            = 50      # setpoint 发布 (冲刺轨迹在发布时按时间采样)
GATE_LOOP_HZ         = 30      # 穿环状态机 / gate.csv
MONITOR_HZ           = 20
LOG_HZ               = 10
TIMING_LOG_HZ        = 10
EKF_LOG_HZ           = 5
VINS_LOG_HZ          = 30      # Hz，vins_raw.csv 轮询 (按 stamp 去重, 高于 VINS 频率即可)
PHASE_LOOP_HZ        = 10      # takeoff 等简单阶段
LOG_ROOT             = "~/uav_logs"
THROTTLE_S           = 2.0
FSYNC_INTERVAL_S     = 1.0
NODE_NAME            = "flight_ctrl"
STATUS_TOPIC         = "/%s/status" % NODE_NAME
STATUS_HZ            = 2.0
TELEMETRY_TOPIC      = "/%s/telemetry" % NODE_NAME
TELEMETRY_HZ         = 5.0
TELEMETRY_HZ_FAST    = 20.0    # 提交 / 冲刺 / 制动段
CONFIG_TOPIC         = "/%s/config" % NODE_NAME


def derive():
    """派生量 (不要手改)。必须在 flight_profile.setup() 之后调用 (否则拿的是旧值)。"""
    global GATE_PRE_VEL_LIM, TELEOP_XY_MAX, TELEOP_DZ_MAX, PHASE_ENVELOPE, MISSION_MODE, GATE_DOCK_D
    # 旧写法 GATE_SHADOW = True (档案 set. / 命令行 --set) → 影子 (flight_profile 会打印提示)
    if GATE_SHADOW:
        MISSION_MODE = "shadow"
    # =============================================================================
    # ##########################   派生量 (不要手改)   ###########################
    # =============================================================================
    # 各阶段 (水平硬限 m, 水平速度硬限 m/s)。水平硬限都相对起飞点。
    # gate: 冲刺段的速度硬限要能容下冲刺速度 (flight_profile 查 ≥ 1.7 × DASH_V);
    #       冲刺前 (识别/摆位) 用 GATE_PRE_VEL_LIM 收紧 (set_phase 动态覆盖)。
    GATE_PRE_VEL_LIM     = max(1.2, VEL_ENVELOPE_RATIO * GATE_REPO_V, VEL_ENVELOPE_RATIO * TELEOP_XY_SPEED)
    TELEOP_XY_MAX        = GATE_XY_MAX            # 手动段水平软限 (相对起飞点); 硬围栏再加 ENVELOPE_MARGIN
    GATE_DOCK_D          = GATE_DOCK_D_STOP if GATE_DOCK_D_STOP > 0 else GATE_DOCK_D_AUTO   # 停靠距离 (实际用的)
    TELEOP_DZ_MAX        = MAX_HEIGHT - 0.5       # 手动段高度上限
    PHASE_ENVELOPE = {
        "takeoff":       (BASE_DRIFT_LIM, 1.0),
        # 2026-11-06: 速度硬限按**这个模式里最快的步骤**定 (mode_spec.STEP_FAST_V): 冲刺模式按 DASH_V,
        #   影子 / 停靠只按摆位速度 —— 以前一律按 DASH_V, 停靠模式的安全包络会跟着 dash_v 变
        "gate":          (GATE_XY_MAX + ENVELOPE_MARGIN,
                          max([1.2, VEL_ENVELOPE_RATIO * GATE_REPO_V]
                              + [VEL_ENVELOPE_RATIO * globals()[k] for k in _ms.fast_v_params(MISSION_MODE)])),
        "descend":       (BASE_DRIFT_LIM + 0.20, 0.8),   # 进入时会按实际位置动态放宽
        "auto_land":     (BASE_DRIFT_LIM + 0.40, 1.0),   # 同上
        "_default":      (BASE_DRIFT_LIM + 0.20, 1.0),
    }
