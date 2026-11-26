# UAV_vision (VB09) 数据与通信接口约定 v3 — 2026-11-26

> 覆盖 ROS 话题、机地 ZMQ 协议、`/gate/status`（`ring` + `reddot`）、选点通道 `/gate/select`、遥测、日志与 `flight_monitor` HTTP API。
> 架构与文件地图见 [`docs/DEV_GUIDE.md`](./DEV_GUIDE.md)。
>
> **演进原则**：**只增字段，不改旧字段语义**。缺字段时飞行判定与网页按降级逻辑处理，不崩溃。

---

## 0. 坐标、外参与时间约定

| 项 | 约定 |
| --- | --- |
| `world` | 飞控局部坐标系（ENU，米），与 `/mavros/local_position/pose` 一致 |
| `body` | 机体 FLU 系（`x` 前、`y` 左、`z` 上），原点 = 飞控中心 |
| `cam`（raw / rect） | OpenCV 相机光学系（`x` 右、`y` 下、`z` 前）；**校正左目转世界系必须乘 `R1ᵀ`**：`R_WR = R_WC @ R1.T` |
| 相机外参 `T_BC` | `cam_x/y/z` = 左目在机体 FLU 中的位置（m）；`cam_pitch_deg` 正 = 镜头低头、`cam_roll_deg` 正 = 右滚、`cam_yaw_deg` 正 = 左偏 |
| 时间 `t_cap` | ROS 秒（飞机时钟），由飞机采图时打戳、地面原样回传，**不要求机地时钟同步** |
| 法向 `n` | 目标平面单位法向，**指向进近侧（相机一侧）**，即 `(p_cam − c)·n > 0` |
| 靶面坐标 | 站在进近侧面向目标：`d = −n`，`right = normalize(d × [0,0,1])`，`up = right × d`；`h = (p − c)·right`（右正），`v = (p − c)·up`（上正），`along = (p − c)·n`（穿面为 0，过面为负） |

靶面几何唯一实现：Python `vb09/common/geom.py` 与 JS `vb09/web/kinds/ring.js`，由 `tools/geom_check.py` 数值对拍（容差 `< 1e-12`）。

---

## 1. `/gate/status`（`vision_bridge.py` 每帧发布 JSON）

`gate`（机上检测）与 `gate_gcs`（地面检测 + 机上融合）都发布这一个话题。`gate` / `gate_gcs` 模式下不再发布 `/track/target` 与 `/gate/target`。

### 1.1 公共头（每帧都有）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `t` | float | 拍摄时刻 `t_cap`（ROS 秒） |
| `kind` | str | 目标插件类型：`"ring"` 或 `"reddot"`（由 `Locator.extra` / `Detector.extra` 注入） |
| `stage` | str | 状态阶段：`search / far / mid / near / terminal / blind / coast / lost` |
| `pose` | str | 位姿插值结果：`"ok"` 或失败原因（如 `pose_gap(120ms)`） |
| `ms` / `det_ms{}` | float / dict | 本帧总耗时与分段耗时（ms） |
| `used` / `rejected` | list[str] | 本帧采用的观测来源 / 被拒原因计数（如 `mono_weak×2`） |
| `diag{}` | dict | 检测与定位诊断标量（视差、候选数、角速度、立体比例等） |
| `sim` | bool | **`true` = 飞行脚本禁止用其驱动实飞**（影子模式或 `GATE_ALLOW_SIM_DRIVE` 除外） |

### 1.2 估计器有效时（`target` 非空）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `target` | `[3]` | 估计器当前目标位置（world，m） |
| `dist` / `src` / `age_ms` | float / str / float | 相机到目标距离（m）、来源（`ring_* / dot_stereo / dot_fused / dot_size / dot_size_prior`）、估计年龄（ms） |
| `sd_cm` | `[3]` | 世界系 xyz 标准差（cm） |
| `cov` | `[9]` | 世界系 3×3 协方差（m²，行主序） |
| `published` | bool | 是否在发布有效期内（`age <= lost_timeout` 且非 `lost/search`） |

### 1.3 `kind == "ring"` 专用附加字段（`vb09/perception/ring/locate.py`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `ring` | dict | `{ok, src, partial, corners, side}` 本帧 2D/3D 环检测摘要 |
| `ring_w` / `ring_cov` | `[3]` / `[9]` | **本帧**环心观测（world）及其协方差 |
| `ring_src` / `ring_partial` | str / bool | `stereo`（角点平面）或 `mono`（椭圆）；是否部分出画 |
| `ring_c` | `[3]` | 估计器累积环心（world，显示用） |
| `n` | `[3]` | 估计器环法向（world，指向进近侧；≥3 次可信法向才发） |
| `aim` | `[3]` | 红点瞄点（world，仅来源 `dot_*` 时存在） |

### 1.4 `kind == "reddot"` 专用附加字段（`vb09/perception/reddot/locate.py` `DotLocator.extra`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `sel` | dict | **始终存在**。桥当前选点回显：`{seq: int\|null, mode: "none"\|"track"\|"ray", state: "none"\|"pending"\|"bound"\|"lost", trk_id: int\|null}` |
| `cands` | list[dict] | **始终存在**（最多前 5 条轨迹，按分降序）。每项：`{id, aim[3], score, D, D_trk, Z_st, Z_sz, d_px, uv[2], sd_cm, hits, n_st, age}` |
| `aim` / `aim_sd_cm` | `[3]` / `[3]` | 当前发布轨迹的世界系中心（m）与 xyz 标准差（cm） |
| `D_est` / `D_trk` / `D_n` | float / float\|null / int | 本帧反推直径 / 轨迹自估真直径（m，`D_eff` 为兼容别名）/ 直径样本数 |
| `score` / `trk_id` / `n_hit` / `n_cand` | float / int | 轨迹综合分、编号、命中数、达标候选总数 |
| `st_scale` / `st_scale_sd` | float | 联合估计的立体比例 `k = 1/m` 及其 1σ 不确定度 |
| `n_bins` / `n_dir` / `n_vbins` | int | 位置桶数 / 独立视线方向数 / 法向视角桶数 |
| `n` / `n_sd_deg` / `n_k` / `n_span_deg` | `[3]` / float / int / float | 多视角共识法向（指向相机侧）、不确定度（°）、一致桶数、视线跨度（°） |
| `n_state` / `n_why` | str / str | 法向状态：`ok` / `frontal`（近正对，按视线进入）/ `ambig`（镜像双解难分）/ `span`（跨度不够）/ `few`（桶数不够）；诊断说明 |

### 1.5 `gate_gcs` 专用字段与版本校验

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `ground_sim` / `ground_stage` | bool / str | 地面 `gcs_gate --mode != 2`（或 `--probe`）；地面检测阶段 |
| `ver_bad` | list[str] | 机地不一致原因（`kind` / `v` / `lib` / `cfg`）；非空时 `sim` 被**强制置 `true`** |

**ZMQ 下行包（`:5556`，msgpack 单帧，兼容 `ZMQ_CONFLATE`）**：
`{t_cap, found, gate: codec(kind).frame_to_msg(fr), rig: rig_to_msg(rig, cfg, kind), sim}`
- `rig` = `{f, cx, cy, B, R1[9], size[2], lib, cfg, kind}`。
- `lib` = `COMMON_LIB_FILES`（`common/wire.py`、`stereo_rig.py`）+ `codec(kind).LIB_FILES` 统一 LF 后的 sha256 前 12 位。
- `cfg` = `asdict(cfg)` JSON 的 sha256 前 12 位。

---

## 2. 选点协议：`/flight_ctrl/select` 与 `/gate/select`（`reddot` 跟踪类目标）

两跳通道（决策权归飞行脚本，网页不直连桥）：
1. **网页 → 飞行脚本**：`POST /api/select` → `flight_monitor` 发布 `/flight_ctrl/select`（`std_msgs/String` JSON）→ `GateInput.on_web_select` 排队。
2. **飞行脚本 → 桥**：手动段（`MissionCtx.manual_select`）或 `Detect` 步骤调用 `GateInput.send_select(cmd)`，分配基于时间戳递增的 `seq`，发布到 `/gate/select`（`std_msgs/String` JSON）→ `vision_bridge.SelectInbox` → `DotLocator.select(cmd, T_WC)`。

### 消息格式（`/gate/select`）

| `mode` | 必填字段 | 含义 |
| --- | --- | --- |
| `"track"` | `seq`, `trk_id` | 选定已有轨迹 ID（自动选点 / 键盘 `1–5` / 网页「选」按钮） |
| `"ray"` | `seq`, `xn`, `yn`, `t_cap` | 按校正左目归一化视线选点（网页画面点击；桥按 `t_cap` 插值 `T_WC` 转世界视线，容差 `sel_ang_deg / sel_lat`，弱候选放行） |
| `"none"` | `seq` | 取消选点（飞行脚本启动时 `init_select()` 会先发一条清空桥上一次运行残留的选点） |

**可靠性机制**：
- 桥在每帧 `/gate/status.sel` 回显 `seq` 与 `state`；
- `Detect` 只认本次运行发过的 `sent_seqs`；超过 1 s 未回显则按**原 `seq`** 重发（最多 4 次）；
- 选中后桥**只发布该轨迹**，若丢失不自动切换成干扰红点，而按方位角持续尝试重挂。

---

## 3. 预览元数据 `X-Det`（`preview_server` HTTP 响应头）

地面 `gcs_gate.py`（`:8091`）与 `gcs_view.py` 在 `/snapshot.jpg` 响应头 `X-Det` 中附带 JSON 元数据：
- **`reddot` 画面点选字段**（`gcs_gate.py` 注入，`vb09/web/kinds/reddot.js` 消费）：
  - `pick`: `{f, cx, cy, w, h, img_w, t_cap}` —— 将浏览器点击像素换算为校正左目归一化坐标 `(xn, yn)` 及拍摄时刻 `t_cap`；
  - `dot_sel`: 当前选点状态；
  - `dot_cands`: 候选列表（含 `uv` 像素坐标，用于画面叠画编号 `1..5`）。
- **`proj` 字段**（3D 环投影叠加）：仅 `flight_monitor --fake` 发送，真机链路未启用。

---

## 4. `/flight_ctrl/*`（飞行脚本发布）

### 4.1 `/flight_ctrl/telemetry`（5 Hz；提交 / 冲刺 / 制动段 20 Hz）
顶层：`task = "gate"`、`schema = 2`、`mode`（`dash / shadow / dock`）、`phase`、`pose`、`vel`、`safety`、`gate`。

`gate` 段核心字段：

| 字段 | 含义 |
| --- | --- |
| `seg` | 当前段名：`manual / wait / align / commit / dash / brake / hold / abort` |
| `mode` | 同顶层 `mode` |
| `gate_used` | 当前生效目标：`c [3]`、`n [3]`、`aim [3]`、`sd_hva_cm [3]`、`n_sd_deg`、`locked`、`t_lock` |
| `along` / `off [h,v]` / `pred_cross [h,v]` / `ttc` | 靶面实时投影与预计穿越点 |
| `plan` / `brake` / `commit` | 冲刺计划路径点、降速说明、刹停点与提交状态 |
| `dock` | 停靠模式状态：`S [3]`、`aim [3]`、`d`、`dir [3]`、`dir_src`、`err_cm`、`err_max_cm`、`aim_sd_cm [h,v,n]`、`done`、`ok` |
| `track` | **跟踪类目标（红点）步骤状态**（`steps/track.py`）：`step`（`detect / approach / refine / locked`）、`sel`、`cands`、`d_stop`、`dist_cmd`、`S [3]`、`sd_along_cm`、`dir_src`、`rounds`、`sig [3]`、`n_state`、`relock`、`normal_skip`、`why` |

### 4.2 `/flight_ctrl/config`（latched）与 `/flight_ctrl/status`（2 Hz）
- `/flight_ctrl/config`：包含 `task`、`params` 与 `derived`（`mode`、`target`、`support`、`gate`、`dock`、`phase_envelope`、`speed_chain`），结构与 `--dry-run --json` 一致。
- `/flight_ctrl/status`：供 `vision_bridge` 转发给地面 HUD（包含 `task`、飞控模式、阶段等）。

---

## 5. 飞行日志目录（`~/uav_logs/flight_NNNN/`）

因飞机重启时钟会重置，日志目录按递增序号编号（`.flight_seq`），每秒后台 `fsync` 防断电丢数据：

| 文件 | 频率 / 内容 |
| --- | --- |
| `run_info.txt` | 飞行编号、脚本名、本地时钟、ROS 时间、`uptime_s` |
| `site_config.txt` | 生效现场档案、全部参数、预检读到的 PX4 参数 |
| `events.log` | 完整 rosout 文本日志与异常堆栈 |
| `flight_log.csv` | 10 Hz：`t, phase, px/py/pz/pz_filt, vx/vy/vz, roll/pitch/yaw, sp_p/v/yaw, act_*, thrust, batt_v, mode, armed, pose_age_ms` |
| `vins_raw.csv` | 30 Hz（按 stamp 去重）：`vins_x/y/z, vins_vx/vy/vz, vins_q*, mavros_z, diff_z` |
| `ekf2_health.csv` | 5 Hz：EKF2 各项状态标志位 + `landed_state` |
| `timing.csv` | 每帧 pose：`delay_ms, interval_ms, stamp_backward_count_total, last_jump_cm` |
| `z_drop_events.log` | `z_filt - z_setpoint` 突降快照（慢降 ≥10cm/3s、快降 ≥5cm/0.5s） |
| `gate.csv` | `GATE_LOOP_HZ`（30 Hz）：穿环/停靠/红点每拍状态、目标、σ 与检测 `diag`（列见 `vb09/flight/log.py` `GATE_CSV_COLS`） |

---

## 6. `flight_monitor.py` HTTP API（默认 `:8080`）

| 方法与路由 | 用途 |
| --- | --- |
| `GET /` 及 `WEB_FILES` 白名单 | 静态网页资源（其余路径 404） |
| `GET /api/state` | 当前快照：`pose`（含度制 `roll/pitch_up`）、`gate`（含 `age`）、`px4`、`stream_seq` 等 |
| `GET /api/stream?since=N` | 增量遥测与状态流（不丢冲刺高速点） |
| `GET /api/config` / `/api/events` / `/api/checks` / `/api/paths` | 配置、事件、机况检查（含 `ver_bad` / `sim` 判定）、文件路径 |
| `GET /api/schema` | 档案旋钮表 + `mode_spec.schema()`（模式与分区定义，驱动配置页动态显隐） |
| `GET /api/sites` / `GET /api/site?name=` | 列出与读取 `~/flight_sites/*.conf` |
| `POST /api/preview` | 子进程运行 `stage14_mission.py --dry-run --json` 返回校验与核对表 |
| `POST /api/save` / `POST /api/launcher` / `POST /api/launcher_temp` | 保存现场档案、生成带 SHA 与 60 s 有效期的 `~/fly.sh` |
| `POST /api/select` | **网页红点选点**：校验 `{mode: track\|ray\|none, ...}` 后发布到 `/flight_ctrl/select` |
| `GET /api/logs` / `GET /api/log?dir=&file=` | 回放页只读日志（单文件 ≤ 40 MB） |
| `GET /fake/snapshot.jpg` | 仅 `--fake` 模式下的模拟识别画面（支持 `--fake-kind reddot`） |
