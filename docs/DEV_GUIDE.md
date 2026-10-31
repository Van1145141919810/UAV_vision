# VB09 开发指南：加东西时改哪里 — 2026-11-01

给以后写代码的人（和新对话）用。接口字段见 `docs/INTERFACE_gate.md`。
本文回答：**要做 X，该动哪几个文件、按什么顺序、怎么验证**。
A 节是文件地图（每个文件做什么、关键符号），B 节是调用链，C 节是"想改 X → 先看哪里"；第 0–9 节是具体步骤。

**新对话怎么用**：把最新的代码 zip 放进项目文件，开头说"先读 `claude/DEV_GUIDE.md`"。
不用把所有文件都发过来 —— 按 C 节找到要改的那 2–5 个文件就够了。

---

## A. 文件地图

部署文件 65 个（不算空的 `__init__.py`），约 18,000 行。其中根目录有 5 个兼容壳，
确认飞机和地面站上没有老脚本 import 它们之后可以删。行数是 2026-11-01 的。

### 根目录：入口（飞机上就在这个目录里运行）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `stage14_mission.py` | 225 | 飞行入口：档案 → 派生量 → dry-run 退出 → 终端检查 → import ROS → `runner.main()` | `_SITE_REPORT`、`_terminal_problems` |
| `gate_plan.py` | 693 | 冲刺规划，纯函数，可离线跑 | `Params`、`params_from_globals`、`plan_dash`、`preflight_check`、`static_preview`、`fuse_samples`、`Traj`、`_ring_frame` |
| `flight_monitor.py` | 405 | 网页后端：HTTP 路由、静态白名单 `WEB_FILES`、命令行参数 | `Handler.do_GET/do_POST`、`main` |
| `run_vision.py` | 217 | 视觉链路启动器：模式、`--target`、相机外参 `CAM`、需要的文件 `NEED`，用子进程起桥 | `main` |
| `vision_bridge.py` | 1119 | 图传桥（ROS 节点）：采图、上行、下行、位姿插值、融合、发布 | `NewCam`、`PoseBuffer`、`GateOut`（发布 /gate/status）、`GateWorker`（机上检测，用 Detector）、`GateFuseWorker`（地面检测时的机上融合，用 Locator + codec）、`Bridge`、`read_cfg` |
| `gcs_gate.py` | 939 | 地面检测程序：收帧 → 校正 → Detector → codec → 回传；本地 HUD、网页预览 | `reader_loop`、`draw_hud`、`preview_meta`、`main` |
| 兼容壳：`stage12_gate.py`、`flight_profile.py`、`gate_detector.py`、`ring_detector.py`、`preview_server.py` | 16–67 | 改名提示 / 转发到 vb09（`flight_profile.py` 还带命令行查档案） | |

### `vb09/common/`：公共（飞机 + 地面）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `geom.py` | 186 | 纯 Python 几何：向量、四元数、靶面坐标 | `gate_axes`、`to_plane`、`entry_axes`、`sigma_hva`、`spread_hva`、`rpy_from_quat`、`quat_to_R` |
| `wire.py` | 148 | 网络公共部分：msgpack/zmq、按 kind 分发序列化、标定摘要、版本检查 | `pack/unpack`、`codec(kind)`、`frame_to_msg/frame_from_msg(…, kind)`、`rig_to_msg(rig, cfg, kind)`、`COMMON_LIB_FILES`、`lib_version(kind)`、`cfg_digest`、`peer_mismatch(…, kind)` |
| `target.py` | 71 | 契约 | `Target`、`Lock`（dict + 属性 kind / trk_id / size） |
| `preview_server.py` | 333 | 地面网页预览（MJPEG + X-Det 元数据） | `publish`、`start` |

### `vb09/perception/`：感知（插件）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `registry.py` | 66 | **插件注册表**：每种目标在每张表里一行，用到时才 import | `CODECS`、`CONFIGS`、`DETECTORS`、`LOCATORS`、`JUDGES`、`load(table, kind)`、`load_config(kind, path)`、`kinds()` |
| `stereo_rig.py` | 241 | 读标定、双目校正、MJPG 降采样解码（所有目标共用） | `_KEYS`、`load_stereo_calib`、`StereoRig.from_file / split / rectify`、`decode_sbs_mjpg` |
| `ring/plugin.py` | 68 | **环插件的两个适配器**（接口见 registry 文件头） | `RingDetector`（process / reset / draw / extra）、`RingLocator`（same_rig / update / extra） |
| `ring/wire.py` | 137 | 环的一帧结果 ↔ 网络字典；这种目标的检测库文件表 | `MSG_VERSION`、`frame_to_msg`、`frame_from_msg`、`LIB_FILES` |
| `ring/config.py` | 125 | 环检测和融合参数（默认值 = 现场参数） | `GateConfig`（`load` / `save`、`lost_timeout`） |
| `ring/frame.py` | 86 | 一帧结果的数据结构 | `GateFrame`、`RingObs`、`DotObs`、`TargetState` |
| `ring/det2d.py` | 394 | 2D：白平衡、红色分割、找环、角点、红点 | `segment`、`find_gate`、`find_corners`、`find_dot` |
| `ring/geom3d.py` | 259 | 3D：角点配对、平面、环心、法向、椭圆 → 圆位姿 | `fit_plane`、`ring_center_in_plane`、`pick_ring_normal`、`circle_pose_from_ellipse` |
| `ring/detect.py` | 386 | 检测主流程 | `GateDetector.process` → `_find_ring_2d` / `_ring_diag` / `_locate_ring` / `_find_dots` / `_remember_ref` |
| `ring/fusion.py` | 329 | 世界系融合（只依赖 numpy） | `TargetEstimator`、`GateFuser`（`update`、`T_WR`、`same_rig`） |
| `ring/locate.py` | 50 | 融合结果 → /gate/status 的环字段 | `ring_extra` |
| `ring/judge.py` | 103 | 飞行脚本侧的单帧判定 | `RingJudge.judge` |
| `ring/draw.py` | 101 | 画叠加 | `draw_frame` |

### `vb09/flight/`：飞行脚本

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `config.py` | 277 | **全部参数**，分 A–G 区；`derive()` 算派生量 | `MISSION_MODE`、`TARGET_KIND`、`PHASE_ENVELOPE`、`derive` |
| `ctx.py` | 226 | 共享状态单例 `F`、阶段和包络、中止/降落请求、遥测数据仓库 | `FlightContext`、`F`、`bind_defaults`、`all_params`、`set_phase`、`request_land/abort`、`check_abort`、`telem_put` |
| `devices.py` | 393 | 飞机的输入输出：遥测汇聚、设定点、终端键盘 | `Telemetry`、`SetpointState`（`set_zhold_mode / set_traj_mode / set_velocity_mode`）、`KeyReader`、`yaw_from_quat`、`tilt_deg_from_quat`、`wrap_pi` |
| `safety.py` | 214 | 安全监控线程 | `safety_monitor`、`Debounce` |
| `preflight.py` | 440 | 起飞前检查 + 配置摘要 | `preflight`、`preflight_plan`、`print_config_summary`、`PX4_PULL_TIMEOUT` |
| `phases.py` | 507 | 通用飞行段 | `phase_takeoff`、`phase_descend`、`phase_auto_land`、`phase_handover`、`arm_with_confirm`、`cleanup_decision`、`emergency_land`、`pose_cb` |
| `log.py` | 767 | 日志目录、各 csv（gate.csv 列表在约 57 行）、发布线程、遥测、config | `init_log_dir`、`gate_logger_loop`、`build_telemetry`、`publish_config`、`status_loop` |
| `target_in.py` | 144 | 收 /gate/status → `JUDGES[C.TARGET_KIND]` → 样本滑窗 | `GateInput.on_status / valid_since / latest_valid / counters` |
| `mission.py` | 584 | 步骤的 ctx + 任务循环 | `MissionCtx`（`enter`、`tick`、`hold`、`fail`、`again`、`manual`、`_telem_and_log`、`new_attempt`）、`run_mission`、`phase_gate` |
| `modes.py` | 20 | 模式 → 步骤列表 | `MODES` |
| `runner.py` | 306 | `main()`：飞行主序列 | `main` |
| `steps/base.py` | 46 | 步骤接口 | `Step`、`StepFail`、`StepBack` |
| `steps/common.py` | 221 | 各模式共用 | `Acquire`、`move_to`、`turn_to`、`align_lock` |
| `steps/dash.py` | 218 | 冲刺穿环 | `make_plan`、`PlanDash`、`Reposition`、`Align`、`Commit`、`Dash`（`committed = True`） |
| `steps/shadow.py` | 38 | 影子 | `AlignHere`、`Shadow` |
| `steps/dock.py` | 136 | 停靠 | `StandOff`、`Hold` |

### `vb09/profile/`：现场档案

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `core.py` | 804 | 通用：档案解析和格式化、`--set`、setup / finalize、JSON、核对表 | `parse_conf_text`、`format_conf`、`schema_json`（网页表单）、`build_overrides`、`setup`、`finalize`、`derived_snapshot`、`SAFETY_RULES`、`POSITIVE_PARAMS`、`SCENE_TABLE` |
| `gate.py` | 506 | 穿环：旋钮表、旋钮 → 参数、校验（含 TARGET_KIND）、派生、打印 | `GATE_KNOBS`、`MODE_NAMES`、`GATE_KNOB_PARAM`、`GATE_SAFETY_RULES`、`_build_gate_overrides`、`_check_gate`、`_derived_gate`、`_gate_lines` |

### `vb09/monitor/`：网页后端

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `common.py` | 123 | 常量、找飞行脚本、任务表 | `SCRIPT_RE`、`discover_scripts`、`script_for` |
| `store.py` | 336 | ROS 数据缓存、频率统计、遥测环形缓冲；飞行日志读取（回放） | `Store`（`set_*`、`snapshot`）、`STORE`、`list_logs`、`read_log` |
| `ros.py` | 190 | 订阅各话题写进 STORE | `start_ros` |
| `checks.py` | 419 | 机况检查、文件与路径 | `run_checks`、`gate_checks`、`camera_check`、`paths_info` |
| `launcher.py` + `fly.sh.tmpl` | 343 + 74 | dry-run 预览、保存档案、生成 fly.sh | `dry_run`、`save_site`、`write_launcher`、`regen_launcher` |
| `fake.py` | 417 | `--fake` 假数据（假飞机 + 假识别画面） | `start_fake_gate` |

### `vb09/web/`：网页（11 个普通 `<script>` 按顺序加载，共享全局作用域）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `monitor.html` / `monitor.css` | 524 / 359 | 页面骨架、样式；`<script>` 列表在 html 末尾 | |
| `core.js` | 133 | 全局状态 `S`、工具函数、`SEG` / `PHASE` 中文表 | `fx`、`nz`、`P()` 参数、`tel()` |
| `kinds/ring.js` | 116 | 环的几何和数据来源 | `gateAxes`、`toPlane`、`gatePerceived`、`gateUsed`、`gateNow` |
| `live.js` | 295 | 轮询 `/api/state` 和 `/api/stream`，顶栏状态标签、提示行 | `pollStream`、`pollState`、`setChip`、`updateHint`、`onState` |
| `views/monitor.js` | 679 | 监控页：俯视图、姿态、高度、安全余量、穿环卡片、靶面视图、曲线 | `drawTop`、`renderGate`、`drawTarget`、`renderSafety` |
| `views/detail.js` | 86 | 详情页、事件 | `renderDetail`、`addEvent` |
| `views/checks.js` | 43 | 机况检查页 + 文件与路径 | `pollChecks`、`pollPaths`、`loadPaths` |
| `views/config.js` | 441 | 起飞配置：表单、预览、保存、fly.sh | `buildForm`、`readKnobs`、`initConfig` |
| `views/help.js` | 54 | 帮助页 | |
| `views/cam.js` | 313 | 识别画面 + 叠加 | `camLoop`、`renderGateDet`、`drawCamOverlay` |
| `views/replay.js` | 269 | 回放 | `loadFlight`、`renderReplay` |
| `main.js` | 112 | 卡片收起、尺寸变化、标签页、主题、启动 | `initFolds`、`requestRender` |

### `tools/` 和 `sim/`

| 文件 | 做什么 |
|---|---|
| `tools/baseline/run_all.py` | 跑全部等价基线（`--quick` 不跑合成渲染和 sim） |
| `tools/baseline/dryrun_dump.py` | 所有档案的 `--dry-run`（JSON + 文本）。`EXPECT_CHANGED`、`_NEW_KEY` 在这里 |
| `tools/baseline/plan_grid.py` | `gate_plan` 参数扫描 |
| `tools/baseline/det_replay.py` | 合成双目 → 检测器（地面 / 机上两条路径）；`synth.py` 生成合成序列 |
| `tools/baseline/onboard_replay.py` | 合成并排帧 → `vision_bridge.GateWorker`（机上检测模式，含预览画图） |
| `tools/baseline/fuse_replay.py` / `judge_replay.py` | 地面观测 → 桥的机上融合 → /gate/status（含 5 种版本不符的变体）/ 判定 |
| `tools/baseline/sim_check.py` | `sim/run_sim.py` 10 个场景并行跑，按容差和 golden 比 |
| `tools/baseline/runtime_dump.py` | site_config.txt、config 消息、csv 列名。`_intended` 在这里 |
| `tools/baseline/log_compare.py` | 日志模板和 e6b73cb 比（机器很忙时偶尔因时序差一行，单独重跑一次） |
| `tools/baseline/sites/*.conf`、`golden/` | 用例档案、基线结果 |
| `tools/wiring_check.py` | 插件接线自检：每张表能加载、接口齐、飞行 / 机上融合不拉 cv2、版本检查认 kind |
| `tools/calib_check.py` / `geom_check.py` / `py38_check.py` | 标定读取 / Python 和 JS 坐标对拍 / 3.8 语法 |
| `tools/dev/js_check.py`、`page_check.js` | 网页语法 / 无头浏览器冒烟 |
| `tools/dev/qualify.py`、`rep.py`、`merge_modules.py`、`api_snapshot.py` | 重构辅助（AST 改名、精确替换、整文件合并、网页 API 快照）。一次性的拆分脚本已删，在 git 历史里 |
| `sim/run_sim.py` + `sim/stubs/` | 离线仿真：桩 rospy + 粗糙的位置环 + 假识别 + 纸 |

---

## B. 调用链

**飞行**
```
fly.sh → stage14_mission.py
  profile.setup(argv, vars(C)) → C.derive() → profile.finalize()   (--dry-run 到此为止)
  → _terminal_problems() → import vb09.flight → bind_defaults() → runner.main()
      订阅 → 起线程: setpoint 发布 / safety_monitor / 各 logger / telemetry / status
      → preflight() → prime → OFFBOARD → arm_with_confirm() → phase_takeoff()
      → mission.phase_gate() → run_mission(MODES[C.MISSION_MODE])
            手动段 ─T→ for step: ctx.enter(step); lock = step.run(ctx, lock)   每拍 ctx.tick()
      → phase_descend() → phase_auto_land() → finally: cleanup_decision()
目标数据: /gate/status → GateInput.on_status → registry.load(JUDGES, C.TARGET_KIND).judge → 滑窗
          → 步骤 ctx.gate_in.valid_since() → gp.fuse_samples → Lock
```

**视觉（gate_gcs：地面检测，飞机融合）**
```
飞机 run_vision.py [--target K] → vision_bridge.py [_target:=K]: NewCam → 降采样并排帧 → zmq 上行
地面 gcs_gate.py --target K: StereoRig.rectify → DETECTORS[K].process → CODECS[K].frame_to_msg
                             + rig_to_msg(rig, cfg, K) (带 lib / cfg / kind) → zmq 下行
飞机 GateFuseWorker.on_obs: CODECS[K].frame_from_msg → PoseBuffer.at(t_cap)·T_BC → LOCATORS[K].update
     → peer_mismatch(…, K) (不一致 → 强制 SIM + ver_bad) → Locator.extra → GateOut.emit → /gate/status
```
gate 模式（机上检测）：`GateWorker` 用 `DETECTORS[K]`（process 带 T_WC，检测器内部自带融合）→ `extra` → `GateOut.emit`，预览用 `draw`。

**网页**
```
flight_monitor.py → ros.start_ros → STORE ← 网页轮询 /api/state、/api/stream
配置页 → /api/schema (KNOBS) 和 /api/preview → launcher.dry_run (子进程跑入口 --dry-run --json)
生成 fly.sh → launcher.write_launcher (fly.sh.tmpl, 写入脚本名 + SHA)
回放页 → /api/logs、/api/log → store.list_logs / read_log
```

---

## C. 想改 X → 先看哪里

| 想改 | 文件 |
|---|---|
| 某个参数的默认值 | `vb09/flight/config.py`（检测参数：`perception/ring/config.py`，会触发版本检查） |
| 档案里能写什么、网页表单 | `profile/gate.py` `GATE_KNOBS` |
| 档案校验 / 拒绝 / 警告 | `profile/gate.py` `_check_gate`；通用部分在 `profile/core.py` |
| dry-run / site_config 核对表文字 | `profile/gate.py` `_gate_lines` |
| 冲刺几何（起跑点、刹车、降速） | `gate_plan.py` |
| 某一段的飞行行为 | `vb09/flight/steps/<模式家族>.py`（共用的在 `common.py`） |
| 段的顺序 / 新模式 | `vb09/flight/modes.py` |
| 手动段按键 / 回手动段 / 收尾悬停 | `vb09/flight/mission.py` `manual`、`run_mission`；键盘读取在 `devices.py` `KeyReader` |
| 停桨、围栏、包络 | `flight/safety.py`、`config.derive()` 的 `PHASE_ENVELOPE` |
| 起飞前检查 | `flight/preflight.py` |
| 起飞 / 降落 / 接管 / cleanup | `flight/phases.py`、`flight/runner.py` |
| 设定点怎么发 / 遥测怎么收 | `flight/devices.py` |
| 遥测 gate 段 / gate.csv | `flight/mission.py` `_telem_and_log`；gate.csv 列名在 `flight/log.py` |
| 目标类型 / 插件选择 | `perception/registry.py`；飞行侧 `C.TARGET_KIND`，桥 `_target:=`，地面 `--target` |
| 一帧识别能不能用（拒绝原因） | `perception/ring/judge.py` |
| 检测算法 | `perception/ring/det2d.py`、`geom3d.py`、`detect.py` |
| 世界系估计 / σ / 重置 | `perception/ring/fusion.py`；重置判定在 `vision_bridge.py` `_stale` / `_reset_age` |
| 地面 → 飞机的消息格式 | `perception/ring/wire.py`（改字段含义要加 `MSG_VERSION`） |
| 版本检查 | `common/wire.py` `peer_mismatch`、`lib_files` |
| /gate/status 字段 | `vision_bridge.GateOut.emit`、`perception/ring/locate.py` |
| 相机外参 | `run_vision.py` `CAM`（传给桥的 `_cam_*` 参数） |
| 标定读取 | `perception/stereo_rig.py` |
| 地面 HUD / 本地窗口 | `gcs_gate.py` `draw_hud`、`to_canvas` |
| 机况检查 | `monitor/checks.py` |
| 网页监控页 / 穿环卡片 | `web/views/monitor.js`（`renderGate`），顶栏在 `web/live.js` |
| 网页配置页 | `web/views/config.js` |
| 识别画面叠加 | `web/views/cam.js`、`web/kinds/ring.js` |
| fly.sh 内容 | `monitor/fly.sh.tmpl`、`monitor/launcher.py` |
| 飞行脚本怎么被找到 | `monitor/common.py` `SCRIPT_RE` |

---

## 0. 规则（每次都适用）

| 规则 | 说明 |
|---|---|
| 飞机是 Python 3.8 | 不用 `match`、`X \| Y` 类型、`dict \| dict`、`str.removeprefix`。提交前跑 `python3.8 tools/py38_check.py` |
| 参数统一写 `C.名字` | 都在 `vb09/flight/config.py`。派生量写在 `derive()` 里，要加到 `global` 那一行 |
| 共享可变状态放 `F` | `vb09/flight/ctx.py` 的 `FlightContext`。不要新增模块级可变全局（线程之间会不共享） |
| 任何地方都不写 `if kind == ...` | 和目标类型有关的东西放进插件（`vb09/perception/<kind>/`），用 `registry.load(表, kind)` 取 |
| 不许悄悄改设定 | 偏离档案的行为（降速、换模式、强制 SIM……）要打日志，还要在网页上显示 |
| 文件名用英文 | 飞行入口 `stageNN_<任务>.py`（NN 是版本）；flight_monitor 按 `SCRIPT_RE` 取编号最大的 |
| `GateConfig` 默认值就是现场参数 | 改了会触发版本检查：地面、飞机两边都要同时更新 |
| 纯搬迁和改行为分开提交 | 改行为的提交要在说明里写清楚哪些基线输出变了、为什么 |

### 每次提交前

```
python3 tools/baseline/run_all.py --quick     # 1 分钟内：dry-run / 规划 / 检测 / 融合 / judge
python3 tools/baseline/run_all.py             # 动了飞行逻辑时：再加 10 个 sim 场景 + 运行时产物
python3 tools/baseline/log_compare.py         # 动了日志措辞时
python3 tools/wiring_check.py                 # 动了 registry / 插件 / wire
python3 tools/dev/js_check.py                 # 动了网页
node tools/dev/page_check.js /tmp/pc          # 动了网页：6 个标签页，console 错误必须是 0
/path/to/python3.8 tools/py38_check.py
```

**基线有意变化时**，三选一：
- dry-run：在 `tools/baseline/dryrun_dump.py` 的 `EXPECT_CHANGED` 里写 `(原因, 期望 rc, 输出里必须有的文字)`；如果是所有用例都多出来的新参数，加到 `_NEW_KEY`。
- 运行时产物：在 `runtime_dump.py` 的 `_intended()` 里加一条。
- sim 新场景：先写进 `sim/run_sim.py`，再用 `sim_check.py --write <场景>` 只写这一个场景的 golden。
- 新的等价基线脚本：golden 要在**改动之前**的代码上写（例：`onboard_replay.py` 是先 `git stash` 掉接线改动再 `--write`）。

golden 和 Python 版本绑定（float 末位不同）。**不要整体重写 golden**，否则就失去了"和存档对比"的意义。

---

## 1. 加一个飞行模式（用已有的步骤拼）

例：`dock` 就是这样加的（提交 eda6dde 可以当模板）。

1. `vb09/flight/modes.py`：`MODES["新键"] = [Acquire(), ..., ...]`
2. `vb09/profile/gate.py`
   - `GATE_KNOBS` 里 `mode` 的 `choices` 加中文名，`help` 里说明这个模式做什么。
   - `MODE_NAMES` 加 `"中文名": "新键"`。
   - `_check_gate` 里加一条 `W.append(...)`，说清楚这一趟会做什么（这条警告就是网页和 dry-run 上的显示）。
   - 如果和 `GATE_SHADOW` 冲突，照 `停靠` 的写法加一条矛盾警告。
3. 网页 `vb09/web/live.js`：`#c-sub` / `#c-loop` 两个状态标签加上新模式的显示（遥测顶层和 `gate` 段都有 `mode` 字段）。
4. 验证：
   - `tools/baseline/sites/` 加一份 `mode_新键.conf`，再在 `EXPECT_CHANGED` 里登记。
   - `sim/run_sim.py` 加场景（字符串旋钮只能写进档案，`--set` 只认数字和开关），然后 `sim_check.py --write <场景>`。

`run_mission` 不用改：手动段、`StepBack` / `StepFail` 回手动段、收尾悬停、推迟的 LAND 都是平台统一做的。
步骤做完想回手动段，调 `ctx.again("提示")`。

---

## 2. 加一个步骤（Step）

文件：一个模式家族一个文件。属于已有家族的放进 `steps/dash.py` / `shadow.py` / `dock.py`；
几个模式都用的放 `steps/common.py`；新家族（比如红点的 Detect / Select / Approach / Confirm）新建一个文件。
继承 `steps/base.py` 的 `Step`。

```python
class MyStep(Step):
    name = "align"        # 遥测 gate.seg 段名。网页 SEG 表 (vb09/web/core.js) 里没有的新段名要在那里加中文
    vel_lim = "pre"       # 冲刺前的速度包络 GATE_PRE_VEL_LIM；None = 不改
    committed = False     # True = 过了就不能中止 (平台推迟 LAND、换停桨线、不能回手动段)。只有冲刺用

    def run(self, ctx, lock):
        ctx.set_seg(self.name, "说明文字")    # 网页提示行显示
        while ...:
            ctx.hold(p)                       # 下指令: hold / setpoint / yaw_step
            dt = ctx.tick()                   # 每拍必须调: 键盘、LAND/接管检查、遥测、gate.csv、节拍
        return lock
```

- **能用的 ctx 接口**：`pos()`、`vel()`、`hold(p)`、`tick()`、`set_seg()`、`yaw_step(yaw, dt)`、`fail(why)`、`again(why)`、`setpoint`、`gate_in`（样本：`valid_since(t)`、`latest_valid()`）、`origin`、`acq`、`locked`、`plan`。
- **放弃**：`ctx.fail("原因")` 会抛 `StepFail`。`GATE_ABORT_TO_MANUAL` 时回手动段，否则降落。
- **移动**：复用 `steps/common.py` 的 `move_to(ctx, S, yaw_mode, timeout, label, ref=(aim, dir))` 和 `turn_to`，不要自己写速度斜坡。
- **步骤之间传东西**：用 `Lock` 的**属性**（例：`lock.standoff = S`），不要往 dict 里加键，否则遥测和 gate.csv 会跟着变。需要给网页看的，放 `ctx` 上一个字段，再在 `mission.py` `_telem_and_log` 里输出（参考 `ctx.dock`）。
- **新参数**：见第 4 节。新步骤的旋钮暂时手写在 `profile/gate.py`（`Step.KNOBS` 还没接上，见第 8 节）。

---

## 3. 加一种目标（红点 / ArUco …）

2026-11-01 接线之后，感知这一层按目标类型（kind）从 `vb09/perception/registry.py` 选插件。
环（`ring`）是现在唯一的一种，可以当模板。加一种目标 = 新建 `vb09/perception/<kind>/` + 每张表加一行：

| 表 | 要写的 | 接口（环的实现） | 谁调用 |
|---|---|---|---|
| `CODECS` | `<kind>/wire.py` 模块 | `MSG_VERSION`、`frame_to_msg(fr)`、`frame_from_msg(m)`（网络输入当不可信数据，坏字段丢掉不抛异常）、`LIB_FILES`（这种目标的检测库文件，版本指纹用） | `common/wire.py` 分发 |
| `CONFIGS` | 参数 dataclass | `load(path)`、`lost_timeout` 属性；默认值就是现场参数 | `gcs_gate`、`vision_bridge`（`registry.load_config`） |
| `DETECTORS` | 检测器适配器 | `process(L, R, t, T_WC=None) → frame`、`reset()`、`draw(img, fr, T_WC, text)`、`extra(fr, T_WC, warn)` | `gcs_gate`（地面，不给 T_WC）、桥 `GateWorker`（机上检测，给 T_WC） |
| `LOCATORS` | 机上融合适配器 | `Locator(cfg, f, R1)`、`f`、`same_rig(f, R1)`、`update(fr, T_WC, t)`、`extra(fr, T_WC, warn)` | 桥 `GateFuseWorker`（gate_gcs 模式） |
| `JUDGES` | 飞行侧判定 | `Judge(C)`、`judge(st, now, p_now) → (样本 \| None, 原因)`、`n_aim_far` | `flight/target_in.py` |

要点：
- **frame 对象**：Detector 产出、codec 序列化、Locator 更新的同一个对象。桥的通用部分用它的 `target`（`valid` / `age` / `pos_w` / `cov_w` / `dist` / `src` / `side`，见 `ring/frame.py` 的 `TargetState`）、`t_ms`、`stage`；但 `GateOut.emit` 现在**还读环的结构**（`ring`、`dots`、`used`、`rejected`、`skipped_ring`、`diag`）—— 新目标要么提供同名字段，要么先把 emit 改成"公共头 + 插件附加字段"（下表第 2 行）。
- **样本**：judge 产出的样本要带和环一样的字段（`c`、`aim`、`n`、`sig`、`t`、`recv` …），步骤（Acquire、StandOff、PlanDash…）就不用改。
- **不拉 cv2**：`LOCATORS` / `CODECS` / `CONFIGS` / `JUDGES` 指向的模块顶层不许 import cv2（机上融合进程和飞行脚本不装也不该加载检测库）。`tools/wiring_check.py` 会查。
- **选哪种**：地面 `gcs_gate.py --target <kind>`、机上 `run_vision.py --target <kind>`（= 桥 `_target:=`）、飞行脚本 `C.TARGET_KIND`。三处不一致 → 桥的版本检查强制 SIM（`ver_bad` 里写"目标类型 …"）。档案校验会拒绝没有判定插件的 `TARGET_KIND`。
- **版本**：改了 `<kind>/wire.py` 的字段含义就把 `MSG_VERSION` 加 1。
- **R1 坑**：用校正系观测时必须乘 `R1ᵀ`（`R_WR = R_WC·R1ᵀ`），漏了在 5 m 处横向会偏大约 25 cm。

**还是环专用、加新目标时要一起改的**（第 8 节也列着）：

| 位置 | 现状 |
|---|---|
| `gcs_gate.py` 的 HUD、掩膜视图、控制台诊断 | 按环的字段写；`--target` 不是 ring 时直接拒绝启动。要把 `hud_lines` 之类挪进 Detector |
| `vision_bridge.GateOut.emit` | `/gate/status` 的 `ring` 子对象按环拼。本轮约定不改结构；红点那一轮改名 `/target/status`，用"公共头 + 插件附加字段" |
| 上行分辨率 | 写死半分辨率；红点要全分辨率（计划里的 `Locator.roi_request`） |
| `flight/target_in.py` 的拒绝原因表 `REASONS` / `REJ_TXT` | 是环的 judge 用的那一套；新 judge 有新原因要加进去 |
| `mission.py` `_telem_and_log`、gate.csv 列 | 按"有 c 和 n 的目标"算（靶面坐标、`R_IN`）；红点没有 n 时会走视线方向，r_in 要换 |
| 网页 | `kinds/ring.js`；新目标加 `kinds/<kind>.js`，同时加进 `WEB_FILES` 白名单和 `monitor.html` 的 `<script>` 列表 |
| `run_vision.py` 的 `NEED` | 列的是环的文件 |

插件目录名用 `reddot`，不要用 `dot`。红点的方案见 `claude/PLAN_dot_v1.md`。

---

## 4. 加一个参数 / 档案旋钮

**只是参数**（用 `--set` 或 `set.X = ...` 调）：
1. `vb09/flight/config.py` 加上 `NAME = 默认值  # 单位，含义`。
2. 必须是正数（会被当除数、速度、频率用）的，加进 `profile/core.py` 的 `POSITIVE_PARAMS`。
3. 基线：dry-run 和 site_config 的 params 会多出这个键。用 `_NEW_KEY`（dryrun）和 `_intended`（runtime）把它滤掉，并在提交说明里写出来。

**档案旋钮**（网页配置页表单里能看到）：
1. `profile/gate.py` 的 `GATE_KNOBS` 加 `dict(key=..., type="float|choice|...", unit, label, group, min, max, help, adv=True?)`。
2. 简单的一对一映射：加进 `GATE_KNOB_PARAM`。要换算或有多个联动的：写在 `_build_gate_overrides` 里，并 `rep.append(...)` 说明。
3. 范围或联动检查放 `_check_gate`：`E` 是拒绝，`W` 是警告。
4. 网页表单来自 `/api/schema`，不用改网页。
5. **字符串参数**（像 `MISSION_MODE`）只能由旋钮设，`--set` 只认数字和开关。

---

## 5. 加遥测字段 / 日志列 / 网页显示

| 要加 | 改哪里 | 注意 |
|---|---|---|
| 遥测 `gate` 段 | `flight/mission.py` `_telem_and_log` 的 `g[...]` | 会被另一个线程序列化：放可变对象时要拷贝（`dict(...)`） |
| 遥测顶层 | `flight/log.py` `build_telemetry` | |
| gate.csv 列 | `flight/log.py` 的列表（约 57 行）+ `_telem_and_log` 里 `row` 的**同一位置** | sim_check 和 runtime 会比较列名，要登记成有意变化；回放页自动认新列 |
| `/flight_ctrl/config` / dry-run 派生量 | `profile/gate.py` `_derived_gate` | 文档 INTERFACE §4 |
| `/gate/status` 字段 | `vision_bridge.GateOut.emit` 的 `extra`，或 `ring/locate.py` | 只加字段，不改老字段的含义。写进 INTERFACE §1 |
| 机况检查 | `monitor/checks.py`（`add(名字, ok/warn/fail/info, 摘要, 提示)`） | |
| 网页卡片 / 标签 | `web/views/monitor.js`（穿环卡片在 `renderGate`）、`web/live.js`（顶部状态标签） | 尽量加进已有文件（按标签页分：`views/<页>.js`）。真要新文件，要同时加进 `flight_monitor.py` 的 `WEB_FILES` 白名单**和** `monitor.html` 的 `<script>`。普通 `<script>` 共享一个全局作用域，函数提升只在同一个文件内有效 → 按依赖顺序放 |
| 假数据 | `monitor/fake.py` | 让 `--fake` 能看到新字段 |

---

## 6. 改检测 / 融合 / 标定

- 文件：`perception/ring/{det2d,geom3d,detect,fusion}.py`、`perception/stereo_rig.py`。
- 只要动了 `wire.lib_files(kind)` 里的任何文件（`common/wire.py`、`stereo_rig.py` + `ring/wire.py` 的 `LIB_FILES`），**检测库指纹就会变** → 只更新一边的话，飞机会强制 SIM。部署时两边一起换。
- 基线：`det_replay`（合成双目 66 帧）、`onboard_replay`（桥的机上检测）、`fuse_replay`。数值有意改变时，检测的 golden 要重写，在提交说明里写清楚原因。现在**没有真实录像**基线；有了 gcs `--record` 录像就加进 `det_replay`。
- 改 `GateConfig` 默认值 = 改现场参数（默认没有 gate_params.json），也会触发版本检查。
- 标定：`load_stereo_calib` 的 `_KEYS` 加别名；尺寸、swap、flip 的检查见 `tools/calib_check.py`。

---

## 7. 改安全 / 平台层（谨慎）

| 内容 | 文件 |
|---|---|
| 倾角停桨、围栏、视觉老化、速度包络 | `flight/safety.py`；阈值规则在 `profile/core.py` `SAFETY_RULES` 和 `profile/gate.py` `GATE_SAFETY_RULES` |
| 各阶段包络 | `config.derive()` 里的 `PHASE_ENVELOPE` + `ctx.set_phase(名字, vel=...)` |
| 起飞 / 降落 / 接管 / cleanup | `flight/phases.py`；主序列在 `flight/runner.py` |
| 预检 | `flight/preflight.py` |
| 冲刺的特殊处理（推迟 LAND、停桨线换成 `GATE_DASH_TILT_DEG`） | 只能通过 `Step.committed = True` 声明，`MissionCtx.enter` 统一生效 |

改了这些必须跑完整 `run_all.py`（`stuck` / `lowbatt` / `noring` 场景就是测它们的）和 `log_compare.py`。

---

## 8. 还没插件化的地方（以后加东西会碰到）

| 位置 | 现状 | 要做的 |
|---|---|---|
| `gcs_gate.py` HUD / 掩膜视图 / 控制台诊断 | 环专用；`--target` 不是 ring 就拒绝 | 挪进 Detector（`hud_lines(fr)` 之类） |
| `vision_bridge.GateOut.emit` | `/gate/status` 结构按环 | 红点那一轮：`/target/status` = 公共头 + 插件附加字段 |
| 上行分辨率 | 写死半分辨率 | 按 kind 决定（`roi_request`） |
| `mission.py` `_telem_and_log`、gate.csv | 按有 c / n 的目标算 | 目标相关的字段挪进插件 |
| `target_in.py` 的 `REASONS` | 环 judge 的原因表 | 由 Judge 提供 |
| 档案旋钮 | 手写在 `profile/gate.py` | 从各 `Step.KNOBS` 自动收集（单独提交，dry-run 基线必须过） |
| `Step.check()` | 没被调用 | 接进 `_check_gate`，按 `MODES[mode]` 查各步骤 |
| 预检 / 手动段预规划 | 停靠模式也按冲刺查和提示 | 按模式区分（偏保守，不急） |
| 入口文件名 | `stageNN_mission.py`，flight_monitor 按编号挑 | 如果要让版本号退出接口：入口用固定名字，版本写进代码和日志 |
| 根目录兼容壳（5 个） | 只为老脚本 import 留着 | 飞机和地面站 `grep -rn "gate_detector\|ring_detector\|flight_profile\|preview_server"` 确认没人用就删 |

---

## 9. 部署清单

1. 飞机：整个目录（根目录入口 + `vb09/`）+ `stereo_calib.yaml`（`gate_params.json` 可选）。
2. 地面：`gcs_gate.py` + `vb09/common/` + `vb09/perception/` + 同一份标定。
3. 两边启动日志里的 `目标 / 检测库 / GateConfig` 要一样（地面 `gcs_gate` 打印一行，飞机桥打印"本机检测库 … GateConfig …"）。
4. flight_monitor 重新生成 fly.sh（fly.sh 记了脚本名和 SHA，代码更新后会警告）。
5. 真机回归顺序：影子 → T3 虚拟环 → 正式冲刺；停靠模式用来单独测检测稳定性。
