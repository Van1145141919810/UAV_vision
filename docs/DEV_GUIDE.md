# VB09 开发指南：加东西时改哪里 — 2026-10-31

给以后写代码的人（和新对话）用。目录和调用链见 README；接口字段见 `docs/INTERFACE_gate.md`。
本文回答：**要做 X，该动哪几个文件、按什么顺序、怎么验证**。
A 节是文件地图（每个文件做什么、关键符号），B 节是调用链，C 节是"想改 X → 先看哪里"；第 0–9 节是具体步骤。

---

## A. 文件地图

行数是 2026-10-31 的。只列可以找的入口点（类和函数），不是全部。

### 根目录：入口（飞机上就在这个目录里运行）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `stage14_mission.py` | 225 | 飞行入口：档案 → 派生量 → dry-run 退出 → 终端检查 → import ROS → `runner.main()` | `_SITE_REPORT`、`_terminal_problems` |
| `stage12_gate.py` | 17 | 空壳，只打印"已改名" | |
| `gate_plan.py` | 693 | 冲刺规划，纯函数，可离线跑 | `Params`、`params_from_globals`、`plan_dash`、`preflight_check`、`static_preview`、`fuse_samples`、`Traj`、`_ring_frame` |
| `flight_profile.py` | 67 | 转发 `vb09/profile`，再加 `__main__`（命令行查档案） | |
| `flight_monitor.py` | 406 | 网页后端：HTTP 路由、静态白名单 `WEB_FILES`、命令行参数 | `Handler.do_GET/do_POST`、`main` |
| `run_vision.py` | 210 | 视觉链路启动器：模式、相机外参 `CAM`、需要的文件 `NEED`，用子进程起桥 | `main` |
| `vision_bridge.py` | 1115 | 图传桥（ROS 节点）：采图、上行、下行、位姿插值、融合、发布 | `NewCam`、`PoseBuffer`、`GateOut`（发布 /gate/status）、`GateWorker`（机上检测）、`GateFuseWorker`（地面检测时的机上融合）、`Bridge`、`read_cfg` |
| `gcs_gate.py` | 935 | 地面检测程序：收帧 → 校正 → 检测 → 回传，本地 HUD，网页预览 | `reader_loop`、`draw_hud`、`preview_meta`、`main` |
| `gate_detector.py` / `ring_detector.py` / `preview_server.py` | 16–24 | 兼容壳，转发到 vb09 | |

### `vb09/common/`：公共（飞机 + 地面）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `geom.py` | 186 | 纯 Python 几何：向量、四元数、靶面坐标 | `gate_axes`、`to_plane`、`entry_axes`、`sigma_hva`、`spread_hva`、`rpy_from_quat`、`quat_to_R` |
| `wire.py` | 239 | 地面 ↔ 飞机的网络格式、版本检查 | `pack/unpack`、`frame_to_msg`、`frame_from_msg`、`rig_to_msg`、`MSG_VERSION`、`LIB_FILES`、`lib_version`、`cfg_digest`、`peer_mismatch` |
| `target.py` | 71 | 契约 | `Target`、`Lock`（dict + 属性 kind / trk_id / size） |
| `preview_server.py` | 333 | 地面网页预览（MJPEG + X-Det 元数据） | `publish`、`start` |

### `vb09/perception/`：感知

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `stereo_rig.py` | 241 | 读标定、双目校正、MJPG 降采样解码 | `_KEYS`、`load_stereo_calib`、`StereoRig.from_file / split / rectify`、`decode_sbs_mjpg` |
| `registry.py` | 12 | 插件注册表 | `JUDGES` |
| `ring/config.py` | 125 | 环检测和融合参数（默认值 = 现场参数） | `GateConfig`（`load` / `save`） |
| `ring/frame.py` | 86 | 一帧结果的数据结构 | `GateFrame`、`RingObs`、`DotObs`、`TargetState` |
| `ring/det2d.py` | 394 | 2D：白平衡、红色分割、找环、角点、红点 | `segment`、`find_gate`、`find_corners`、`find_dot` |
| `ring/geom3d.py` | 259 | 3D：角点配对、平面、环心、法向、椭圆 → 圆位姿 | `fit_plane`、`ring_center_in_plane`、`pick_ring_normal`、`circle_pose_from_ellipse` |
| `ring/detect.py` | 386 | 检测主流程 | `GateDetector.process` → `_find_ring_2d` / `_ring_diag` / `_locate_ring` / `_find_dots` / `_remember_ref` |
| `ring/fusion.py` | 329 | 世界系融合（只依赖 numpy） | `TargetEstimator`、`GateFuser`（`update`、`T_WR`、`same_rig`） |
| `ring/locate.py` | 50 | 机上：融合结果 → /gate/status 的环字段 | `ring_extra` |
| `ring/judge.py` | 103 | 飞行脚本侧的单帧判定 | `RingJudge.judge` |
| `ring/draw.py` | 101 | 画叠加 | `draw_frame` |

### `vb09/flight/`：飞行脚本

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `config.py` | 274 | **全部参数**，分 A–G 区；`derive()` 算派生量（包络、`GATE_DOCK_D`、shadow → MISSION_MODE） | `MISSION_MODE`、`PHASE_ENVELOPE`、`derive` |
| `ctx.py` | 227 | 共享状态单例 `F`、阶段和包络、中止/降落请求、遥测数据仓库 | `FlightContext`、`F`、`bind_defaults`、`all_params`、`set_phase`、`request_land/abort`、`check_abort`、`telem_put` |
| `telemetry.py` | 174 | 订阅回调的数据（位姿、速度、状态、电池、VINS） | `Telemetry` |
| `setpoint.py` | 140 | 设定点状态（VEL / ZHOLD / TRAJ） | `SetpointState.set_zhold_mode / set_traj_mode / set_velocity_mode` |
| `safety.py` | 214 | 安全监控线程 | `safety_monitor`、`Debounce` |
| `preflight.py` | 441 | 起飞前检查 + 配置摘要 | `preflight`、`preflight_plan`、`print_config_summary`、`PX4_PULL_TIMEOUT` |
| `phases.py` | 507 | 通用飞行段 | `phase_takeoff`、`phase_descend`、`phase_auto_land`、`phase_handover`、`arm_with_confirm`、`cleanup_decision`、`emergency_land`、`pose_cb` |
| `log.py` | 768 | 日志目录、各 csv（gate.csv 列表在约 57 行）、发布线程、遥测、config | `init_log_dir`、`gate_logger_loop`、`build_telemetry`、`publish_config`、`status_loop` |
| `keys.py` | 98 | 终端键盘 | `KeyReader` |
| `target_in.py` | 143 | 收 /gate/status → judge → 样本滑窗 | `GateInput.on_status / valid_since / latest_valid / counters` |
| `mission.py` | 585 | 步骤的 ctx + 任务循环 | `MissionCtx`（`enter`、`tick`、`hold`、`fail`、`again`、`manual`、`_telem_and_log`、`new_attempt`）、`run_mission`、`phase_gate` |
| `modes.py` | 24 | 模式 → 步骤列表 | `MODES` |
| `runner.py` | 306 | `main()`：飞行主序列 | `main` |
| `steps/base.py` | 46 | 步骤接口 | `Step`、`StepFail`、`StepBack` |
| `steps/acquire.py` | 88 | 悬停识别 → 粗估 | `Acquire` |
| `steps/plan_dash.py` | 55 | 规划 + 降速 | `make_plan`、`PlanDash` |
| `steps/reposition.py` | 94 | 移动 / 转头 | `move_to`、`turn_to`、`Reposition` |
| `steps/align.py` | 107 | 取样锁定 | `align_lock`、`Align`、`AlignHere` |
| `steps/commit.py` | 69 | 等 G | `Commit` |
| `steps/dash.py` | 86 | 冲刺、制动、卡纸 | `Dash`（`committed = True`） |
| `steps/shadow.py` | 27 | 影子收尾 | `Shadow` |
| `steps/standoff.py` | 136 | 停靠 | `StandOff`、`Hold` |

### `vb09/profile/`：现场档案

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `core.py` | 804 | 通用：档案解析和格式化、`--set`、setup / finalize、JSON、核对表 | `parse_conf_text`、`format_conf`、`schema_json`（网页表单）、`build_overrides`、`setup`、`finalize`、`derived_snapshot`、`SAFETY_RULES`、`POSITIVE_PARAMS`、`SCENE_TABLE` |
| `gate.py` | 496 | 穿环：旋钮表、旋钮 → 参数、校验、派生、打印 | `GATE_KNOBS`、`MODE_NAMES`、`GATE_KNOB_PARAM`、`GATE_SAFETY_RULES`、`_build_gate_overrides`、`_check_gate`、`_derived_gate`、`_gate_lines` |

### `vb09/monitor/`：网页后端

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `common.py` | 123 | 找飞行脚本、任务表 | `SCRIPT_RE`、`discover_scripts`、`script_for` |
| `store.py` | 287 | ROS 数据缓存、频率统计、遥测环形缓冲 | `Store`（`set_*`、`snapshot`）、`STORE` |
| `ros.py` | 190 | 订阅各话题写进 STORE | `start_ros` |
| `checks.py` | 419 | 机况检查、文件与路径 | `run_checks`、`gate_checks`、`camera_check`、`paths_info` |
| `launcher.py` + `fly.sh.tmpl` | 343 | dry-run 预览、保存档案、生成 fly.sh | `dry_run`、`save_site`、`write_launcher`、`regen_launcher` |
| `logs.py` | 58 | 回放接口 | `list_logs`、`read_log` |
| `fake.py` | 417 | `--fake` 假数据（假飞机 + 假识别画面） | `start_fake_gate` |

### `vb09/web/`：网页（普通 `<script>` 按顺序加载，共享全局作用域）

| 文件 | 行 | 做什么 | 关键符号 |
|---|---|---|---|
| `monitor.html` / `monitor.css` | 525 / — | 页面骨架、样式 | |
| `core.js` | 133 | 全局状态 `S`、工具函数、`SEG` / `PHASE` 中文表 | `fx`、`nz`、`P()` 参数、`tel()` |
| `kinds/ring.js` | 116 | 环的几何和数据来源 | `gateAxes`、`toPlane`、`gatePerceived`、`gateUsed`、`gateNow` |
| `live.js` | 295 | 轮询 `/api/state` 和 `/api/stream`，顶栏状态标签、提示行 | `pollStream`、`pollState`、`setChip`、`updateHint`、`onState` |
| `views/monitor.js` | 679 | 监控页：俯视图、姿态、高度、安全余量、穿环卡片、靶面视图、曲线 | `drawTop`、`renderGate`、`drawTarget`、`renderSafety` |
| `views/detail.js` | 86 | 详情页、事件 | `renderDetail`、`addEvent` |
| `views/checks.js` | 22 | 机况检查页 | `pollChecks` |
| `views/config.js` | 441 | 起飞配置：表单、预览、保存、fly.sh | `buildForm`、`readKnobs`、`initConfig` |
| `views/help.js` | 54 | 帮助页 | |
| `views/cam.js` | 313 | 识别画面 + 叠加 | `camLoop`、`renderGateDet`、`drawCamOverlay` |
| `ui.js` | 53 | 卡片收起、文件路径 | |
| `views/replay.js` | 269 | 回放 | `loadFlight`、`renderReplay` |
| `main.js` | 82 | 标签页、主题、启动 | |

### `tools/` 和 `sim/`

| 文件 | 做什么 |
|---|---|
| `tools/baseline/run_all.py` | 跑全部等价基线（`--quick` 不跑 sim） |
| `tools/baseline/dryrun_dump.py` | 所有档案的 `--dry-run`（JSON + 文本）。`EXPECT_CHANGED`、`_NEW_KEY` 在这里 |
| `tools/baseline/plan_grid.py` | `gate_plan` 参数扫描 |
| `tools/baseline/det_replay.py` / `fuse_replay.py` / `judge_replay.py` | 检测 / 融合 / 判定回放；`synth.py` 生成合成双目 |
| `tools/baseline/sim_check.py` | `sim/run_sim.py` 10 个场景并行跑，按容差和 golden 比 |
| `tools/baseline/runtime_dump.py` | site_config.txt、config 消息、csv 列名。`_intended` 在这里 |
| `tools/baseline/log_compare.py` | 日志模板和 e6b73cb 比 |
| `tools/baseline/sites/*.conf`、`golden/` | 用例档案、基线结果 |
| `tools/calib_check.py` / `geom_check.py` / `py38_check.py` | 标定读取 / Python 和 JS 坐标对拍 / 3.8 语法 |
| `tools/dev/js_check.py`、`page_check.js` | 网页语法 / 无头浏览器冒烟 |
| `tools/dev/qualify.py`、`split_*.py`、`stepify.py`、`api_snapshot.py`、`rep.py` | 重构时用的一次性脚本（AST 改名、切文件），留着备查 |
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
目标数据: /gate/status → GateInput.on_status → JUDGES["ring"].judge → 滑窗 → 步骤 ctx.gate_in.valid_since() → gp.fuse_samples → Lock
```

**视觉（gate_gcs）**
```
飞机 run_vision.py → vision_bridge.py: NewCam → 降采样并排帧 → zmq 上行
地面 gcs_gate.py: StereoRig.rectify → GateDetector.process → frame_to_msg + rig_to_msg(rig, cfg) → zmq 下行
飞机 GateFuseWorker.on_obs: peer_mismatch → frame_from_msg → PoseBuffer.at(t_cap)·T_BC → GateFuser.update
     → ring_extra → GateOut.emit → /gate/status
```

**网页**
```
flight_monitor.py → ros.start_ros → STORE ← 网页轮询 /api/state、/api/stream
配置页 → /api/schema (KNOBS) 和 /api/preview → launcher.dry_run (子进程跑入口 --dry-run --json)
生成 fly.sh → launcher.write_launcher (fly.sh.tmpl, 写入脚本名 + SHA)
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
| 某一段的飞行行为 | `vb09/flight/steps/<段>.py` |
| 段的顺序 / 新模式 | `vb09/flight/modes.py` |
| 手动段按键 / 回手动段 / 收尾悬停 | `vb09/flight/mission.py` `manual`、`run_mission` |
| 停桨、围栏、包络 | `flight/safety.py`、`config.derive()` 的 `PHASE_ENVELOPE` |
| 起飞前检查 | `flight/preflight.py` |
| 起飞 / 降落 / 接管 / cleanup | `flight/phases.py`、`flight/runner.py` |
| 遥测 gate 段 / gate.csv | `flight/mission.py` `_telem_and_log`；gate.csv 列名在 `flight/log.py` |
| 一帧识别能不能用（拒绝原因） | `perception/ring/judge.py` |
| 检测算法 | `perception/ring/det2d.py`、`geom3d.py`、`detect.py` |
| 世界系估计 / σ / 重置 | `perception/ring/fusion.py`、`vision_bridge.py` `_stale` / `_reset_age` |
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
| 步骤里不许出现 `if kind == ...` | 和目标类型有关的判断放进 Judge / Locator |
| 不许悄悄改设定 | 偏离档案的行为（降速、换模式、强制 SIM……）要打日志，还要在网页上显示 |
| 文件名用英文 | 飞行入口 `stageNN_<任务>.py`（NN 是版本）；flight_monitor 按 `SCRIPT_RE` 取编号最大的 |
| `GateConfig` 默认值就是现场参数 | 改了会触发版本检查：地面、飞机两边都要同时更新 |
| 纯搬迁和改行为分开提交 | 改行为的提交要在说明里写清楚哪些基线输出变了、为什么 |

### 每次提交前

```
python3 tools/baseline/run_all.py --quick     # 1 分钟内：dry-run / 规划 / 检测 / 融合 / judge
python3 tools/baseline/run_all.py             # 动了飞行逻辑时：再加 10 个 sim 场景 + 运行时产物
python3 tools/baseline/log_compare.py         # 动了日志措辞时
python3 tools/dev/js_check.py                 # 动了网页
node tools/dev/page_check.js /tmp/pc          # 动了网页：6 个标签页，console 错误必须是 0
/path/to/python3.8 tools/py38_check.py
```

**基线有意变化时**，三选一：
- dry-run：在 `tools/baseline/dryrun_dump.py` 的 `EXPECT_CHANGED` 里写 `(原因, 期望 rc, 输出里必须有的文字)`；如果是所有用例都多出来的新参数，加到 `_NEW_KEY`。
- 运行时产物：在 `runtime_dump.py` 的 `_intended()` 里加一条。
- sim 新场景：先写进 `sim/run_sim.py`，再用 `sim_check.py --write <场景>` 只写这一个场景的 golden。

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

文件：`vb09/flight/steps/<名字>.py`，继承 `steps/base.py` 的 `Step`。

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
- **移动**：复用 `steps/reposition.py` 的 `move_to(ctx, S, yaw_mode, timeout, label, ref=(aim, dir))` 和 `turn_to`，不要自己写速度斜坡。
- **步骤之间传东西**：用 `Lock` 的**属性**（例：`lock.standoff = S`），不要往 dict 里加键，否则遥测和 gate.csv 会跟着变。需要给网页看的，放 `ctx` 上一个字段，再在 `mission.py` `_telem_and_log` 里输出（参考 `ctx.dock`）。
- **新参数**：见第 4 节。新步骤的旋钮暂时手写在 `profile/gate.py`（`Step.KNOBS` 还没接上，见第 8 节）。

---

## 3. 加一种目标（红点 / ArUco …）

现在只有环，**一半是插件化的**。按层改：

| 层 | 现在 | 加新目标要做的 |
|---|---|---|
| 地面检测 | `gcs_gate.py` 直接用 `ring.detect.GateDetector` | 新建 `vb09/perception/<kind>/detect.py`。**DETECTORS 注册表还没有**：要在 gcs 入口按 kind 选 |
| 序列化 | `wire.frame_to_msg / frame_from_msg` 是环的结构 | 新 kind 自己的 to/from_msg；**`MSG_VERSION` 加 1**；`LIB_FILES` 加上新文件 |
| 机上融合 | `vision_bridge.GateFuseWorker` 直接用 `ring.fusion.GateFuser` + `ring.locate.ring_extra` | 新建 `<kind>/fusion.py`、`<kind>/locate.py`。**LOCATORS 注册表还没有**：要在桥里按 kind 选 |
| 话题 | `/gate/status`，字段是环的 | 本轮约定不改结构。红点那一轮再改名 `/target/status`，用"公共头 + 附加字段" |
| 判定 | `registry.JUDGES = {"ring": RingJudge}` | 新建 `<kind>/judge.py`（`judge(st, now, p_now) -> (样本 \| None, 原因)`），在 `JUDGES` 里加一行 |
| 选判定 | `flight/target_in.py:48` 写死 `JUDGES["ring"]` | 改成按目标类型选（要新加参数，比如 `C.TARGET_KIND`，现在还没有） |
| 步骤 | 只读 `Lock` 的 `c / aim / n / sig_hva` | 新目标的 judge 产出同样字段的样本，步骤就不用改 |
| 遥测 / gate.csv | `MissionCtx._telem_and_log` 按环算（靶面 h/v、r_in…） | 和目标类型有关的部分要挪进插件（见第 8 节） |
| 网页 | `kinds/ring.js` | 新建 `kinds/<kind>.js`，加进 `WEB_FILES` 和 `monitor.html` 的 `<script>` 列表 |

插件目录名用 `reddot`，不要用 `dot`。红点的方案见 `claude/PLAN_dot_v1.md`（上行要全分辨率，环用半分辨率就够）。
**R1 坑**：用校正系观测时必须乘 `R1ᵀ`（`R_WR = R_WC·R1ᵀ`），漏了在 5 m 处横向会偏大约 25 cm。

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
| 网页卡片 / 标签 | `web/views/monitor.js`（穿环卡片在 `renderGate`）、`web/live.js`（顶部状态标签） | 新 JS 文件要同时加进 `flight_monitor.py` 的 `WEB_FILES` 白名单**和** `monitor.html` 的 `<script>`。普通 `<script>` 共享一个全局作用域，函数提升只在同一个文件内有效 → 按依赖顺序放 |
| 假数据 | `monitor/fake.py` | 让 `--fake` 能看到新字段 |

---

## 6. 改检测 / 融合 / 标定

- 文件：`perception/ring/{det2d,geom3d,detect,fusion}.py`、`perception/stereo_rig.py`。
- 只要动了 `wire.LIB_FILES` 里的任何文件，**检测库指纹就会变** → 只更新一边的话，飞机会强制 SIM。部署时两边一起换。
- 基线：`det_replay`（合成双目 66 帧）、`fuse_replay`。数值有意改变时，检测的 golden 要重写，在提交说明里写清楚原因。现在**没有真实录像**基线；有了 gcs `--record` 录像就加进 `det_replay`。
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
| `flight/target_in.py:48` | 写死 `JUDGES["ring"]` | 按 `C.TARGET_KIND` 选 |
| `vision_bridge.py` | `GateWorker` / `GateFuseWorker` 写死环 | 加 `LOCATORS[kind]` |
| `gcs_gate.py` | 写死 `GateDetector` | 加 `DETECTORS[kind]` |
| `mission.py` `_telem_and_log` | 遥测和 gate.csv 按环算 | 把目标相关的字段挪进插件 |
| 档案旋钮 | 手写在 `profile/gate.py` | 从各 `Step.KNOBS` 自动收集（单独提交，dry-run 基线必须过） |
| `Step.check()` | 没被调用 | 接进 `_check_gate`，按 `MODES[mode]` 查各步骤 |
| 预检 / 手动段预规划 | 停靠模式也按冲刺查和提示 | 按模式区分（偏保守，不急） |
| 入口文件名 | `stageNN_mission.py`，flight_monitor 按编号挑 | 如果要让版本号退出接口：入口用固定名字，版本写进代码和日志 |

---

## 9. 部署清单

1. 飞机：整个目录（根目录入口 + `vb09/`）+ `stereo_calib.yaml`（`gate_params.json` 可选）。
2. 地面：`gcs_gate.py` + `vb09/common/` + `vb09/perception/` + 同一份标定。
3. 两边启动日志里的 `检测库 / GateConfig` 两个数要一样。
4. flight_monitor 重新生成 fly.sh（fly.sh 记了脚本名和 SHA，代码更新后会警告）。
5. 真机回归顺序：影子 → T3 虚拟环 → 正式冲刺；停靠模式用来单独测检测稳定性。
