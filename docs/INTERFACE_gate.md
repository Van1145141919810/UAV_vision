# 穿环（撞环）数据接口约定 v2 — 2026-10-31

v1（2026-10-19）写的时候生产方还没实现。**v2 第 1、2 节已经改成和代码实际行为一致**（PLAN_refactor_v2 第 6 步 4）。
代码位置按重构后的目录写（`vb09/…`）。

原则不变：**只加字段，不改老字段的含义**。缺哪个字段，网页就少画哪一块，不会报错（见第 9 节）。
stage11 / 停靠跟踪已删除（2026-10-30），所以 `task` 只剩 `"gate"`。

---

## 0. 坐标与时间

| 项 | 约定 |
|---|---|
| world | VIO odom（ENU，米），和 `/mavros/local_position/pose` 同一个系 |
| 时间 | ROS 秒（飞机时钟）。`t_cap` = 拍摄时刻，和现在的链路一样由飞机打、地面原样回填 |
| `c` | 环中心（world） |
| `n` | 环平面单位法向，**指向进近侧**（飞机过来的那一边）。桥发布前翻成 `(p_cam − c)·n > 0`；judge 收到后按当前位置再翻一次 |
| 靶面坐标 | 站在进近侧看向环。`d = −n`，`right = normalize(d × [0,0,1])`，`up = right × d` |
| | `h = (p − c)·right`（右为正），`v = (p − c)·up`（上为正） |
| `along` | `(p − c)·n`。离环越近越小，穿过环平面时 = 0，过环后为负 |

实现：Python `vb09/common/geom.py`（`gate_axes` / `to_plane`，飞机、地面、假数据共用一份），
网页 `vb09/web/kinds/ring.js`（`gateAxes` / `toPlane`）。两边由 `tools/geom_check.py` 对拍。

---

## 1. `/gate/status`（vision_bridge `GateOut.emit`，每帧一条 JSON）

`gate`（机上检测）和 `gate_gcs`（地面检测，飞机融合）两种模式都发这一个话题。
`/track/target`、`/gate/target` 在这两种模式下**已不再发布**（2026-10-30；`aruco` 模式的 `/track/target` 不变）。

### 1.1 每帧都有

| 字段 | 类型 | 含义 |
|---|---|---|
| `t` | 数 | `t_cap`（拍摄时刻，ROS 秒） |
| `stage` | 串 | 检测阶段 `search / far / mid / near / terminal / blind / lost …` |
| `pose` | 串 | 位姿插值结果，`"ok"` 或原因（如 `pose_gap(120ms)`） |
| `ms` `det_ms{}` | 数 / 对象 | 本帧耗时（ms），`det_ms` 按阶段分 |
| `ring` | 对象 | `{ok, src, partial, corners, side}` 这一帧的 2D/3D 环检测 |
| `dots` `used` `rejected` | 列表 | 定位上的红点来源；估计器用了 / 拒绝了哪些观测 |
| `skip_ring` `diag{}` | 布尔 / 对象 | 跳过环检测（近距离只找红点）；检测器诊断（标量） |
| `sim` | 布尔 | **true = 飞行脚本不拿它驱动**（影子模式除外）。见 1.4 |

### 1.2 估计器有效时（`target` 存在）

| 字段 | 类型 | 含义 |
|---|---|---|
| `target` | `[3]` | 估计器的目标（world）。**是红点**；红点没定位上时是"环心 + 0.3 m 先验"（σ 停在 6–7 cm） |
| `dist` `src` `age_ms` `side` | | 距离（m）、来源（`ring_* / dot_stereo / dot_ray_plane …`）、估计年龄、正反面 |
| `sd_cm` | `[3]` | 世界系 xyz 标准差（cm） |
| `cov` | `[9]` | 世界系协方差（m²，行主序 3×3） |
| `published` | 布尔 | 在发布期内（年龄 ≤ `lost_timeout` 且阶段不是 lost/search） |

### 1.3 环专用附加字段（`vb09/perception/ring/locate.py` `ring_extra`，两种模式都有）

| 字段 | 类型 | 含义 |
|---|---|---|
| `ring_w` | `[3]` | **这一帧**的环心观测（world） |
| `ring_cov` | `[9]` | 它的协方差（m²，行主序 3×3） |
| `ring_src` | 串 | `stereo`（角点平面）/ `mono`（椭圆） |
| `ring_partial` | 布尔 | 环部分出画 |
| `ring_c` | `[3]` | 估计器累积的环心（world，显示用） |
| `n` | `[3]` | 估计器的环法向（world，指向相机一侧）。至少 3 次可信法向才有 |
| `aim` | `[3]` | 红点（world）。只有估计器来源是 `dot_*` 时才有 |

飞行脚本（`vb09/perception/ring/judge.py`）的用法：**有 `ring_w` 就用它当环心 c**（多帧取中值），σ 用 `ring_cov`；
`GATE_DOT_PREFER` 且来源是双目红点（年龄 ≤ `GATE_DOT_MAX_AGE_MS`）时改用 `target` + `cov`。
有 `aim` 就瞄红点（离环心 > `R_IN − 5 cm` 视为红点识别错，改瞄环心），没有就瞄环心。老版桥没有 `ring_w` → 用 `target`。

### 1.4 `gate_gcs` 才有

| 字段 | 类型 | 含义 |
|---|---|---|
| `ground_sim` | 布尔 | 地面 `gcs_gate --mode` ≠ 2（或 `--probe`） |
| `ground_stage` | 串 | 地面检测阶段 |
| `ver_bad` | 列表 | **2026-10-31 新增**。地面和飞机不一致的原因（中文短句）：消息版本 `v`、检测库指纹 `lib`、`GateConfig` 摘要 `cfg`、或地面没报（旧版 gcs_gate）。有它时这一帧 `sim` 被**强制为 true**。一致时不带这个字段 |

`sim` = 飞机 `_gate_sim` 或 `ground_sim` 或 `ver_bad` 非空。飞行脚本只在 `GATE_SHADOW`（影子）或 `GATE_ALLOW_SIM_DRIVE` 时用 sim 数据。

### 1.5 v1 里列了、但**没有实现**的字段

`n_sd_deg`（法向不确定度）、`aim_sd_cm`、`r_in` / `r_out`。judge 和网页都容忍缺失
（judge 有 `n_sd_deg` 才查 `GATE_N_SD_MAX_DEG`；网页环尺寸用 `derived.gate`，再没有用 0.6 / 0.8 m）。

### 1.6 下行包（地面 → 飞机，`gate_gcs`）

`{t_cap, found, gate: frame_to_msg(…), rig: rig_to_msg(rig, cfg), sim}`，msgpack。`vb09/common/wire.py`：

- `gate.v` = `MSG_VERSION`（现在 1）
- `rig` = `{f, cx, cy, B, R1[9], size[2], lib, cfg}`。`f` / `R1` 变了 → 飞机重置估计器；`lib` / `cfg` 和飞机算出来的不一致 → 1.4 的 `ver_bad`
- `lib` = 检测库文件（`wire / stereo_rig / ring/{config,frame,det2d,geom3d,detect,fusion}`）内容的 sha256 前 12 位，换行统一成 LF。两边启动时都会打印 `检测库 xxxx  GateConfig yyyy`，对一下就知道是不是同一版

---

## 2. 画面叠加：预览元数据 `X-Det` 里的 `proj`

> **状态：未实现。** 现在只有 `flight_monitor --fake --fake-task gate` 的假画面带 `proj`（网页叠加功能用它自测）。
> 真机链路（`vision_bridge` 上行包头、`gcs_gate.preview_meta`）都**不发** `proj`，所以真机画面上没有紫色叠加环。

格式（网页已能接收）：

```json
"proj": {
  "K":    [fx, fy, cx, cy],        // 这张预览图（未缩放前）的针孔内参
  "size": [w, h],                  // K 对应的图像尺寸；浏览器按实际收到的尺寸换算
  "T_WC": [16 个数, 行主序 4×4],   // 相机光学系（x右 y下 z前）→ world，这一帧的
  "approx": false,                 // 位姿不是按 t_cap 插值的就给 true，页面会标「位姿近似」
  "t": t_cap
}
```

以后实现时的来源建议（未做）：

| 模式 | 预览图 | `proj` 怎么来 |
|---|---|---|
| `gate` | 校正后左目（`GateWorker` 画好叠加） | 桥在上行包头放 `proj`：`K = [rig.f, rig.f, rig.cx, rig.cy]`，`size = rig.size`，`T_WC` 用**校正后**左目：`R = R_WC_raw · R1ᵀ`，平移不变 |
| `gate_gcs` | 地面画的校正左目 | 桥在包头加 `T_WC_raw`（`poses.at(t_cap) · T_BC`）；地面用自己的 `R1`、`K_rect` 合成 |
| `aruco` | 不变 | 不需要 |

---

## 3. `/flight_ctrl/telemetry`（`stage14_mission.py`）

顶层：`"task": "gate"`、`"schema": 2`。阶段名 `"gate"`，段在 `gate.seg`。

`gate` 段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `seg` | 串 | 步骤名（`vb09/flight/steps/*` 的 `Step.name`）：`manual` 手动 / `wait` 等待识别·规划 / `align` 摆位·对准 / `commit` 提交 / `dash` 冲刺 / `brake` 制动 / `hold` 过环悬停 / `abort` 中止 |
| `gate_used` | 对象 | 脚本**实际在用**的环：`c [3]`、`n [3]`、`aim [3]`、`sd_cm [3]` 或 `cov [9]`、`sd_hva_cm [3]`、`n_sd_deg`、`locked`、`t_lock` |
| `along` | 数 | 见第 0 节（按 `gate_used` 算） |
| `off` | `[h, v]` | 飞机在靶面上的投影（m） |
| `pred_cross` | `[h, v]` | 脚本自己算的预计穿越点（含延迟补偿） |
| `pred_sd_cm` | `[h, v]` | 预计穿越点的不确定度 |
| `ttc` | 数 / null | 预计到环时间（s） |
| `v_along_meas` `v_along_cmd` | 数 | 沿 −n 的实测 / 指令速度（朝环飞为正） |
| `gate_age` | 数 | 脚本用的环估计年龄（s） |
| `commit` | 对象 | `d` 提交距离、`passed`、`abort_ok` |
| `plan` | 对象 | `seq`、`pts [[x,y,z],…]`、`labels […]`、降速时 `slow` `v` `v_set` `slow_why` |
| `brake` | 对象 | `end [3]`、`dist`、`acc` |
| `t` | 数 | |

频率：提交 / 冲刺 / 制动段 20 Hz，其它 5 Hz。

---

## 4. `/flight_ctrl/config`

| 字段 | 含义 |
|---|---|
| 顶层 `task` | `"gate"` |
| `derived.gate` | `r_in r_out dot_d`、`dash_v`、`dash_acc`、`brake_acc`、`brake_dist`、`d_align`、`d_commit`、`overshoot` |
| `derived.phase_envelope.gate` | 穿环段的 `[水平硬限 m, 速度硬限 m/s]` |
| `derived.speed_chain` | `[["名字", 值], …]` 每一道限速 |

`--dry-run --json` 输出同样的 `derived`。

---

## 5. `/flight_ctrl/status`

只加 `task`，其余字段**不改**（vision_bridge → 地面 HUD 在用）。

---

## 6. 日志 `gate.csv`（回放页）

列由 `vb09/flight/log.py` 定义（`t` 列是 ROS 秒，和 `flight_log.csv` 对齐）。网页默认勾 `along off_h off_v`。

---

## 7. `flight_profile.py`（配置页表单）

- `KNOBS` 的 `adv=True` 项放进「高级」折叠区。
- `task` 旋钮只剩 穿环。**`mode` 旋钮**（2026-10-30）：冲刺 / 影子，选 `vb09/flight/modes.py` 的步骤列表；旧写法 `GATE_SHADOW = True` 仍等于影子（打印"旧写法"提示）。
- 已删除的旋钮（戳点、跟踪）在档案里出现 → 警告并忽略；`gate_mode = 戳点` → 报错拒绝。

---

## 8. flight_monitor 接口

| 接口 | 内容 |
|---|---|
| `/` 和 `vb09/web/` 下的固定文件 | 网页（白名单静态路由，别的路径 404） |
| `/api/state` | `gate`（`/gate/status` 最新一条，`age` = 收到到现在的秒数）、`px4`、`stream_seq` … |
| `/api/stream?since=N` | 序号 > N 的遥测和环状态 |
| `/api/logs` `/api/log?dir=&file=` | 日志列表 / 读一个文件（≤ 40 MB，不跟出根目录） |
| `/fake/snapshot.jpg` | 仅 `--fake --fake-task gate`：假识别画面 + 带 `proj` 的 X-Det |

机况检查「环识别 /gate/status」：无数据 → 不通过；有 `ver_bad` → 不通过并列出原因；`sim` → 提示不会驱动。

---

## 9. 缺字段时网页怎么显示

| 缺什么 | 网页 |
|---|---|
| 整个 `/gate/status` | 机况检查「环识别」不通过 |
| `n` | 环只画中心点；靶面视图显示「没有法向 n」 |
| `aim` | 红点按环中心画，标「红点按环中心」 |
| `cov` | 误差圆用 `sd_cm` 最大值 |
| 遥测 `gate` 段 | 用感知 + 当前位姿自己算，标签显示「感知」 |
| X-Det `proj` | 画面不叠加（真机现在就是这样，见第 2 节） |
| `derived.gate` | 环尺寸用 0.6 / 0.8 m、红点 11 cm |
