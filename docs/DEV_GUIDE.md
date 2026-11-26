# UAV_vision (VB09) 开发指南 — 2026-11-26

> 面向开发者与 AI Coding Agent 的架构地图与接入指南。
> 数据接口字段与 ROS / ZMQ 协议见 [`docs/INTERFACE.md`](./INTERFACE.md)；现场快速启动与摄像头标定见 [`README.md`](../README.md)。
>
> **新 Agent 快速上手方式**：先读本文 **第 0 节（红线与验证）** + **A/B 节（文件地图与调用链）**，再按 **C 节速查表** 只读要改的那 2–5 个文件即可，无需通读全仓。

---

## 0. 项目背景、运行红线与提交前验证

### 硬件与运行环境
- **机型**：OWL2 / VB09 四旋翼（带护桨 `56×56×22 cm`），PX4 v1.12.3 飞控。
- **机载端**：Khadas 机载电脑（`visbot`，`192.168.2.20`），Ubuntu 20.04 + ROS Noetic + MAVROS + **Python 3.8**（无外网，每次重启系统时钟会重置，故飞行日志按递增编号 `~/uav_logs/flight_NNNN/` 组织）。
- **定位源**：内置 `640×480` 双目（`B=70mm`，`stereo120.launch`）跑 VINS_Fusion（适合近场/室内；室外空地易失锁，规划切换 GPS + 下视激光）。
- **识别相机**：前视 `3840×1080` MJPG USB 双目（左右并排 1920×1080/眼，基线 `B≈60.8mm`，3mm 镜头，`stereo_calib.yaml`）。
- **地面站**：Ubuntu 4090 服务器或笔记本，运行 `gcs_gate.py`（2D 检测服务）或 `gcs_view.py`（机上检测观看/录屏），通过 ZMQ `:5555`（上行）/ `:5556`（下行）通信。
- **架构分工**：地面站只做无状态 2D 像素检测（输入 JPEG，输出像素量），**所有 3D 测距、世界系融合、法向、门限、选点绑定与飞控都在飞机上跑**；`t_cap` 由飞机打戳并由地面原样回传，无需机地时钟同步。

### 代码红线（修改前必读）
| 红线 | 说明 |
| --- | --- |
| **机载 Python 3.8 语法** | 禁止 `match`、`X \| Y` 类型联合、`dict \| dict`、`str.removeprefix`。改完必跑 `python3 tools/py38_check.py`。 |
| **不拉 `cv2` 边界** | `JUDGES`、`LOCATORS`、`CODECS`、`CONFIGS`、`vb09/flight/*`、`vb09/profile/*` 顶层**严禁** `import cv2`（飞行脚本和 `--dry-run` 不依赖 `cv2` / ROS）。由 `tools/wiring_check.py` 检查。 |
| **参数统一写 `C.名字`** | 全部飞行参数在 `vb09/flight/config.py`；派生量写在 `C.derive()` 并声明 `global`。 |
| **共享可变状态放 `F`** | 放在 `vb09/flight/ctx.py` 的单例 `F`，不要新增模块级可变全局（多线程不共享）。 |
| **模式隔离** | 档案里**当前模式不用的区（section）的参数**即使越界，也不能影响当前模式或阻止起飞。由 `vb09/flight/mode_spec.py` + `tools/mode_isolation_check.py` 守卫。 |
| **目标类型走注册表** | 不在飞行核心代码里写 `if kind == "ring"`，统一通过 `vb09/perception/registry.py`（`CODECS / CONFIGS / DETECTORS / LOCATORS / JUDGES`）按需加载。 |
| **双端同源与指纹检查** | 改动 `vb09/common/wire.py`、`stereo_rig.py` 或 `LIB_FILES` / `GateConfig` / `DotConfig` 默认值会改变检测库/配置指纹，**飞机与地面站必须同时更新**，否则飞机强制把地面结果标 `sim=true`（`ver_bad`）。 |
| **禁止静默改设定** | 任何偏离档案的行为（降速、强制 SIM、换模式、跳过法线挪位）都必须打日志并在网页显示。 |
| **实飞终端红线** | `stage14_mission.py` 拒绝在 `tmux` / `screen` / VSCode 内置终端中实飞（确保 SSH 断连产生 stdin EOF 触发降落）。 |

### 提交前验证命令

```bash
python3 tools/py38_check.py                   # Python 3.8 语法检查（零依赖）
python3 tools/mode_isolation_check.py         # 模式元数据一致性 + 静态 AST + dry-run 扰动（零 numpy 依赖）
python3 tools/geom_check.py                   # Python 与 JS 靶面坐标对拍（需 node）
python3 tools/wiring_check.py                 # 感知插件注册表 / 接口 / 不拉 cv2 / 版本检查（需 numpy）
python3 tools/calib_check.py                  # 标定尺寸 / swap_halves / flip_180 检查（需 cv2+numpy）
python3 tools/reddot_depth_sim.py             # 红点直径自标定 / 继承 / 旋转防漂仿真
python3 tools/reddot_normal_sim.py            # 红点多视角法向共识仿真
python3 tools/reddot_track_sim.py             # 红点分桶后验 / 立体比例 / 选点仿真
python3 tools/baseline/run_all.py --quick     # 等价基线：dry-run / 规划 / 检测 / 融合 / judge
python3 tools/baseline/run_all.py             # 完整基线（含离线 sim 全部场景 + 运行时产物）
python3 tools/baseline/log_compare.py         # 日志模板对比
python3 tools/dev/js_check.py                 # 网页 JS 语法检查
node tools/dev/page_check.js /tmp/pc          # 网页无头冒烟（需 playwright；红点加 FAKE_KIND=reddot）
```

> **基线维护规则**：不要整体重写 `tools/baseline/golden/`。有意变更行为时：dry-run 登记在 `dryrun_dump.py`（`EXPECT_CHANGED` / `_NEW_KEY`），运行时产物登记在 `runtime_dump.py`（`_intended`），sim 新场景用 `sim_check.py --write <场景名>` 单独写入。

---

## A. 文件地图

### 1. 根目录入口

| 文件 | 行 | 部署 | 职责与关键符号 |
| --- | --- | --- | --- |
| `stage14_mission.py` | 225 | 飞机 | **实飞主入口**：`profile.setup → C.derive → profile.finalize`（`--dry-run` 在此退出）→ `_terminal_problems` → import ROS → `runner.main()` |
| `gate_plan.py` | 693 | 飞机 | 冲刺轨迹规划（纯 Python，无 ROS/numpy）：`Params`、`plan_dash`、`d_run_for`、`dash_stop`、`preflight_check`、`static_preview`、`fuse_samples`、`Traj1D` |
| `run_vision.py` | 225 | 飞机 | 图传桥 Python 启动器：`CAM` 外参默认值、`--target` / `--gcs` / `--drive` / `--lock`、检查 `NEED` 并拉起 `vision_bridge.py` |
| `vision_bridge.py` | 1178 | 飞机 | 图传与机上融合 ROS 节点：`NewCam`、`PoseBuffer`、`SelectInbox`（`/gate/select`）、`GateOut`（`/gate/status`）、`GateWorker`（机上检测）、`GateFuseWorker`（地面检测+机上融合）、`Bridge` |
| `gcs_gate.py` | 984 | 地面 | 地面检测服务（`gate_gcs`）：收帧 → `StereoRig.rectify` → `DETECTORS[kind].process` → `CODECS[kind].frame_to_msg` → 回传；带本地 HUD、录像/回放与网页预览（`X-Det.pick`） |
| `gcs_view.py` | 606 | 地面 | 机上检测模式（`gate`）配套地面观看/录屏端：无检测、无需标定，仅展示 HUD 与 `--record` |
| `flight_monitor.py` | 431 | 飞机 | 机载网页监控后端（`:8080`）：白名单静态路由 `WEB_FILES`、`/api/*`（含 `POST /api/select`）、`--fake --fake-kind reddot` |
| `stereo_calib.yaml` | 98 | 双端 | 当前生效双目标定（1920×1080/眼，T 单位 mm，`swap_halves=0`，`flip_180=0`） |
| 兼容壳（5 个） | 16–67 | — | `stage12_gate.py`、`flight_profile.py`、`gate_detector.py`、`ring_detector.py`、`preview_server.py`（转发到 `vb09/`） |

### 2. `vb09/common/`（飞机 + 地面公共库）

| 文件 | 行 | 职责与关键符号 |
| --- | --- | --- |
| `geom.py` | 186 | 纯 Python 向量/四元数/靶面坐标：`gate_axes`、`to_plane`、`entry_axes`、`sigma_hva`、`spread_hva`、`rpy_from_quat`、`quat_to_R` |
| `wire.py` | 148 | ZMQ/msgpack 编解码与版本检查：`codec(kind)`、`frame_to_msg/from_msg`、`rig_to_msg`、`COMMON_LIB_FILES`、`lib_version(kind)`、`cfg_digest`、`peer_mismatch` |
| `target.py` | 71 | 飞行步骤间传递契约：`Target`、`Lock`（dict + `kind / trk_id / standoff / d_in` 属性） |
| `preview_server.py` | 333 | 地面/机上 HTTP MJPEG 预览与 `X-Det` 头发布：`publish`、`start` |

### 3. `vb09/perception/`（感知插件层）

| 文件 | 行 | 职责与关键符号 |
| --- | --- | --- |
| `registry.py` | 80 | **插件注册表**（延迟 import）：`CODECS`、`CONFIGS`、`DETECTORS`、`LOCATORS`、`JUDGES`、`load`、`load_config`（按插件 `PARAMS_FILE` 隔离参数文件） |
| `stereo_rig.py` | 241 | 标定读取与双目校正：`load_stereo_calib`、`StereoRig.from_file / split / rectify`、`decode_sbs_mjpg` |
| `ring/*`（11 文件） | ~1,750 | **环插件**（`kind = "ring"`）：`config (GateConfig)`、`frame`、`det2d`、`geom3d`、`detect`、`fusion`、`locate`、`judge (RingJudge)`、`draw`、`wire`、`plugin (RingDetector / RingLocator)` |
| `reddot/config.py` | 149 | **红点参数** `DotConfig`（`PARAMS_FILE = "reddot_params.json"`；物理直径范围 `[0.08, 0.24]`、分桶后验、立体比例、选点与多视角法向门限） |
| `reddot/frame.py` | 72 | 红点帧结构：`Cand`（2D 像素特征 + 视差）、`DotFrame`、`DotObs`、`TargetState` |
| `reddot/detect.py` | 246 | 红点 2D 检测 + 同行带模板匹配 + 面积法半高矩精化：`red_map`、`half_max_moments`、`DotDetectorCore.process`（用 `cv2`） |
| `reddot/locate.py` | 865 | **红点机上定位核心**（纯 `numpy`）：`_Track`（视角×距离分桶 + 位置/立体比例 `[p; m=1/k]` Huber 联合最小二乘、每轨迹自估直径 + `dmem` 记忆继承）、`normal_consensus`（多视角似然比消镜像歧义）、`DotLocator`（`update` / `select` 方位绑定重挂 / `inherit` / `extra`） |
| `reddot/judge.py` | 108 | 红点飞行侧单帧判定：`RedDotJudge`（声明 `TRACKED = True`、`MODES` 支持表、发现级门限、选点后豁免分数/竞争者拒绝） |
| `reddot/wire.py` | 84 | 红点下行编解码：`MSG_VERSION`、`LIB_FILES`、`frame_to_msg` / `frame_from_msg` |
| `reddot/plugin.py` | 186 | 红点适配器：`RedDotDetector`（含地面 HUD/叠加/掩膜钩子 `ground_draw / hud_rows / console / mask / probe_dist`）与 `RedDotLocator` |

### 4. `vb09/flight/`（机载飞行平台与步骤）

| 文件 | 行 | 职责与关键符号 |
| --- | --- | --- |
| `config.py` | 311 | 全部飞行参数（A–H 区）与 `derive()`（`PHASE_ENVELOPE`、`GATE_PRE_VEL_LIM`、`GATE_DOCK_D`、`TELEOP_DZ_MAX`） |
| `mode_spec.py` | 125 | **模式分区元数据（纯数据，不拉 ROS/numpy）**：`SECTIONS`、`STEP_USES`、`STEP_FAST_V`、`MODE_SPECS`（含 `steps` 与 `steps_tracked`）、`sections()`、`fast_v_params()`、`schema()` |
| `modes.py` | 39 | 模式→步骤映射：`MODES`（环）、`MODES_TRACKED`（红点等 `TRACKED=True` 目标）、`steps_for(mode, tracked)` |
| `ctx.py` | 226 | 共享状态单例 `F`：`FlightContext`、`set_phase`、`request_land / request_abort`、`check_abort`、`telem_put` |
| `devices.py` | 393 | 飞控/键盘 IO：`Telemetry`、`SetpointState`（`VEL / ZHOLD / TRAJ`）、`KeyReader`（cbreak 读键盘，EOF=断连） |
| `safety.py` | 214 | 安全监控线程：`safety_monitor`、`Debounce`（FATAL 停桨 vs LAND 缓降、冲刺段独立倾角线） |
| `preflight.py` | 454 | 6 项起飞前检查 + 配置摘要 + 地面预规划：`preflight`、`preflight_plan`、`read_px4_params`、`print_config_summary` |
| `phases.py` | 507 | 通用阶段：`phase_takeoff`、`phase_descend`、`phase_auto_land`、`phase_handover`、`arm_with_confirm`、`cleanup_decision`、`emergency_land`、`pose_cb` |
| `log.py` | 768 | 编号日志目录与后台落盘：`flight_log.csv`、`vins_raw.csv`、`ekf2_health.csv`、`timing.csv`、`gate.csv`、`z_drop_events.log`、`events.log`、遥测发布 |
| `target_in.py` | 244 | 识别输入与选点通道：`GateInput`（收 `/gate/status` → `JUDGES[TARGET_KIND]`、收 `/flight_ctrl/select`、发 `/gate/select` + `init_select` + `resend_select` + `own_selection`） |
| `mission.py` | 622 | 任务状态机：`MissionCtx`（`enter / tick / manual / manual_select / _telem_and_log`）、`run_mission`、`phase_gate` |
| `runner.py` | 308 | `main()` 主序列：单实例保护、ROS 订阅/发布、服务等待、preflight → 起飞 → `phase_gate` → 降落 → cleanup |
| `steps/base.py` | 46 | 步骤基类：`Step`（`name / vel_lim / committed`）、`StepFail`、`StepBack` |
| `steps/common.py` | 223 | 共用动作：`Acquire`、`move_to`、`turn_to`、`align_lock` |
| `steps/dash.py` | 234 | 冲刺步骤（环与冲刺尾部共用）：`make_plan`、`PlanDash`、`Reposition`、`Align`、`Commit`（含 `on_go` 钩子）、`Dash`（`committed=True`） |
| `steps/shadow.py` | 38 | 影子步骤：`AlignHere`、`Shadow` |
| `steps/dock.py` | 209 | 停靠步骤：`StandOff`、`VScan`（上下扫描定法向）、`Hold` |
| `steps/track.py` | 505 | **跟踪类目标步骤链（红点）**：`Detect`（自动/数字键/网页选点 + 回显重发 + 竞争者过滤）、`Approach / ApproachDock / ApproachDash`（边飞边估，停在 `D + k·σ_along`，水平边界解析限幅）、`Refine`（按诚实 σ 与法向锁，不够准时左右/上下换视角）、`PlanLocked`、`CommitTrack`（按 G 时偏移超 `GATE_RELOCK_CM` 重锁） |

### 5. `vb09/profile/`、`vb09/monitor/`、`vb09/web/`

| 目录/文件 | 行 | 职责与关键符号 |
| --- | --- | --- |
| `profile/core.py` | 825 | 通用档案解析/格式化、`--set`、`setup / finalize`、`SAFETY_RULES`、`SCENE_TABLE`、`schema_json` |
| `profile/gate.py` | 723 | 任务旋钮表 `GATE_KNOBS`（带 `section`）、`TARGET_NAMES`、`MODE_NAMES`、分区校验（仅校验当前模式使用的区）、目标×模式支持度检查 `_check_target` |
| `monitor/{common,store,ros,checks,launcher,fake}.py` + `fly.sh.tmpl` | ~1,600 | 网页后端缓存、ROS 订阅（含转发 `POST /api/select → /flight_ctrl/select`）、机况检查、dry-run 预览与 `fly.sh` 生成、`--fake` 假数据 |
| `web/monitor.{html,css}` + `{core,live,main}.js` | ~1,460 | 页面骨架、全局状态 `S`、轮询 `/api/state` & `/api/stream`、顶栏状态标签 |
| `web/kinds/{ring,reddot}.js` | 116 / 127 | 目标专用前端：`ring.js`（靶面几何与对拍）；`reddot.js`（候选表、选/取消按钮、画面点击选点 `dotPickAt`、候选序号叠加） |
| `web/views/{monitor,detail,checks,config,help,cam,replay}.js` | ~2,000 | 7 个页面视图；`config.js` 按 `/api/schema` 自动按目标+模式显隐分区表单 |

### 6. `tools/` 与 `sim/`

| 工具 | 用途 |
| --- | --- |
| `tools/stereo_calib.py`、`tools/stereo_range.py` | 标定板生成、双目内参标定（输出 `stereo_calib.yaml`）、ArUco 实时测距与卷尺误差验收 |
| `tools/rec_stereo.py`、`tools/rec_read.py` | 机上 3840×1080 原始 MJPG + 位姿截图（含 `:8092` 网页预览与 `--aruco` 外参采图提示）/ 读回导出校正 PNG |
| `tools/calib_check.py`、`tools/wiring_check.py`、`tools/geom_check.py`、`tools/py38_check.py` | 标定校验 / 插件接线校验 / Python-JS 几何对拍 / Python 3.8 语法检查 |
| `tools/mode_isolation_check.py` | 模式分区一致性 + AST 静态读参审计 + 102 组 dry-run 扰动（`--sim` 加仿真扰动） |
| `tools/reddot_{depth,normal,track}_sim.py`、`tools/reddot_replay.py` | 红点深度/法向/分桶选点单元测试仿真与离线回放 |
| `tools/fake_target.py`、`tools/fake_uplink.py` | 虚拟目标 `/gate/status` 发布器（室内无实体目标测飞行逻辑）/ 离线录像回放测试 `gcs_gate` & `gcs_view` |
| `tools/baseline/*`、`sim/run_sim.py`、`sim/stubs/*` | 回归基线全套与无 ROS 离线飞行仿真器（含 13+ 场景：环/红点/停靠/冲刺/卡纸/低电等） |

---

## B. 核心调用链

### 1. 飞行主链（含双轨模式分发）
```text
fly.sh → stage14_mission.py
  profile.setup(argv, vars(C)) → C.derive() → profile.finalize()   (--dry-run 到此退出)
  → _terminal_problems() → import vb09.flight → bind_defaults() → runner.main()
      订阅 ROS 话题 → F.gate_in.init_select()（跟踪类目标清旧选点）
      → 启动线程: setpoint(50Hz) / safety_monitor(20Hz) / loggers / telemetry / status
      → preflight() → prime → OFFBOARD → arm_with_confirm() → phase_takeoff()
      → mission.phase_gate() → run_mission(steps_for(C.MISSION_MODE, F.gate_in.tracked))
            手动段（含 manual_select 立即选点）─按 T→
              [环: MODES]          Acquire → PlanDash → Reposition → Align → Commit → Dash (或 StandOff→VScan→Hold)
              [红点: MODES_TRACKED] Detect  → ApproachDash → Refine → PlanLocked → CommitTrack → Dash (或 ApproachDock→Refine→Hold)
            每拍 ctx.tick() 统一处理键盘 / LAND / takeover / 遥测 / gate.csv
      → phase_descend() → phase_auto_land() → finally: cleanup_decision()
```

### 2. 视觉感知链（`gate_gcs` 地面检测 + 飞机融合）
```text
飞机 run_vision.py --target K → vision_bridge.py _target:=K
  NewCam(3840×1080 MJPG) → 降采样并排帧 + t_cap → ZMQ :5555 上行
地面 gcs_gate.py --target K
  StereoRig.rectify → DETECTORS[K].process(L, R, t_cap) → CODECS[K].frame_to_msg
  + rig_to_msg(rig, cfg, K)（含 kind / lib / cfg 指纹）→ ZMQ :5556 下行
飞机 GateFuseWorker.on_obs
  CODECS[K].frame_from_msg → PoseBuffer.at(t_cap)·T_BC → SelectInbox.drain(locator)
  → LOCATORS[K].update(fr, T_WC, t_cap) → peer_mismatch(…, K) → LOCATORS[K].extra → GateOut.emit → /gate/status
```
- `gate` 模式（机上检测）：`GateWorker` 直接调 `DETECTORS[K].process(L, R, t, T_WC)` 并在本地融合，上行发带叠加的左目图给 `gcs_view.py`。

### 3. 红点选点链（Web / 键盘 → 飞行脚本 → 桥）
```text
方式 1（网页按钮/画面点击）：web/kinds/reddot.js → POST /api/select → flight_monitor → /flight_ctrl/select
                                                      ↓
方式 2（终端数字键 1–5）──→ GateInput（手动段 manual_select 或 Detect 步骤）→ /gate/select (带递增 seq)
                                                      ↓
                            vision_bridge.SelectInbox → DotLocator.select(cmd, T_WC) → /gate/status.sel 回显 seq
```
> 选哪个目标的决策权始终在**飞行脚本**（冲刺中/锁定后不换目标），桥只负责按 `seq` 绑定和方位重挂。

---

## C. “想改 X → 先看哪里”速查表

| 需求 | 目标文件 |
| --- | --- |
| 改飞行参数默认值 / 派生包络 | `vb09/flight/config.py`（`derive()`） |
| 改红点 / 环检测与定位默认参数 | `vb09/perception/reddot/config.py` / `ring/config.py`（改默认值会变指纹，双端同步） |
| 增删档案旋钮 / 表单分组 / 校验 | `vb09/profile/gate.py`（`GATE_KNOBS`、`_check_gate`）、`vb09/flight/mode_spec.py` |
| 改红点 2D 分割 / 模板匹配 / 直径矩 | `vb09/perception/reddot/detect.py` |
| 改红点分桶后验 / 立体比例 / 多视角法向 / 选点重挂 | `vb09/perception/reddot/locate.py` |
| 改红点单帧判定 / 抵近 / 换视角 / 重锁 | `vb09/perception/reddot/judge.py`、`vb09/flight/steps/track.py` |
| 改冲刺轨迹 / 刹车超调 / 降速策略 | `gate_plan.py`、`vb09/flight/steps/dash.py` |
| 改停靠 / 上下扫描 / 悬停统计 | `vb09/flight/steps/dock.py` |
| 改手动段 / 回手动段 / 遥测 `gate` 段 | `vb09/flight/mission.py`、`vb09/flight/devices.py`（`KeyReader`） |
| 改安全防线 / 起飞 / 降落 / 接管 / 预检 | `vb09/flight/safety.py`、`phases.py`、`preflight.py`、`runner.py` |
| 改相机外参默认值 | **两处同步改**：`run_vision.py`（`CAM`）与 `vision_bridge.py`（`cam_*` 默认值） |
| 改标定文件读取或新增键别名 | `vb09/perception/stereo_rig.py`（`_KEYS`） |
| 改地面检测 HUD / 画面元数据 | `gcs_gate.py`、`vb09/perception/reddot/plugin.py` |
| 改网页监控 / 红点选点面板 / 配置表单 | `vb09/web/views/monitor.js`、`kinds/reddot.js`、`views/config.js`、`flight_monitor.py`（`WEB_FILES`） |

---

## 1. 接入一个新飞行模式

1. **定义元数据**（`vb09/flight/mode_spec.py`）：
   - 若有新参数分区，在 `SECTIONS` 加一行；
   - 在 `MODE_SPECS` 加模式键（`label` 中文名、`preview`、`drives`、`steps`、`steps_tracked`、`desc`）；
   - 若某步骤需要冲刺级速度包络，登记进 `STEP_FAST_V`。
2. **注册步骤列表**（`vb09/flight/modes.py`）：在 `MODES` 和 `MODES_TRACKED` 加上同键名的步骤列表（必须与 `MODE_SPECS` 完全一致）。
3. **旋钮与支持度**（`vb09/profile/gate.py` & 各 `judge.py`）：
   - `GATE_KNOBS` 里 `mode` 的 `choices` 加上中文 `label`；
   - 在各目标判定插件的 `MODES` 字典声明对该模式的支持等级（`"ok"` 或 `("experimental", "原因")`）。
4. **验证**：跑 `python3 tools/mode_isolation_check.py`（自动检查 `modes.py` vs `mode_spec.py` vs `steps` 一致性及未使用分区扰动隔离），并在 `sim/run_sim.py` 补充仿真场景。

---

## 2. 接入一个新步骤（Step）

1. 在 `vb09/flight/steps/` 下对应家族文件（或新文件）继承 `Step`：
   ```python
   class MyStep(Step):
       name = "align"      # 遥测 gate.seg；新段名需在 vb09/web/core.js SEG 表加中文
       vel_lim = "pre"     # 冲刺前速度包络；None = 不收紧
       committed = False   # True = 不可中止（推迟 LAND、切冲刺倾角线）

       def run(self, ctx, lock):
           ctx.set_seg(self.name, "说明")
           while ...:
               ctx.hold(p)
               dt = ctx.tick()
           return lock
   ```
2. **必须登记**到 `vb09/flight/mode_spec.py` 的 `STEP_USES["MyStep"] = (...)`，声明它读取哪些区的参数（否则 `mode_isolation_check.py` 静态 AST 检查会直接报错）。
3. 移动尽量复用 `steps/common.py` 的 `move_to` / `turn_to`；步骤间传值挂在 `Lock` 属性上（不要改 dict 键以免污染遥测/CSV）。

---

## 3. 接入一种新目标类型（Perception Plugin）

参考现有两个完整实现：**`vb09/perception/ring/`（冻结式目标）** 与 **`vb09/perception/reddot/`（跟踪式目标，`TRACKED = True`）**。

1. 新建 `vb09/perception/<kind>/`，实现 5 个契约并在 `vb09/perception/registry.py` 对应表各加一行：
   - `CODECS[kind]`：`wire.py`（`MSG_VERSION`、`LIB_FILES`、`frame_to_msg`、`frame_from_msg`）—— **严禁 import `cv2`**。
   - `CONFIGS[kind]`：`config.py`（dataclass，提供 `load()`、`lost_timeout`；建议声明独立 `PARAMS_FILE = "<kind>_params.json"` 避免误读 `gate_params.json`）。
   - `DETECTORS[kind]`：`plugin.py` 检测器适配器（`process(L, R, t, T_WC=None)`、`reset()`、`draw()`、`extra()`；若要在 `gcs_gate.py` 支持地面检测，补充显示钩子 `found / ground_draw / hud_rows / console / mask / probe_dist`）。
   - `LOCATORS[kind]`：`plugin.py` 机上融合适配器（`Locator(cfg, f, R1)`、`f`、`same_rig`、`update`、`extra`；可选 `inherit(old)` 重建继承、`select(cmd, T_WC)` 选点绑定）—— **严禁 import `cv2`**。
   - `JUDGES[kind]`：`judge.py` 飞行侧判定（`judge(st, now, p_now) → (sample, reason)`、`MODES` 支持表、`REASONS / REJ_TXT` 扩展拒绝原因；若边飞边估则置 `TRACKED = True`）—— **严禁 import `cv2`**。
2. **坐标坑**：校正左目系转世界系必须乘 `R1ᵀ`，即 `R_WR = R_WC @ R1.T`（漏乘在 5 m 处横向偏 ~25 cm）。
3. **档案与网页**：
   - `vb09/profile/gate.py`：`GATE_KNOBS` 的 `target` 选项与 `TARGET_NAMES` 加新目标；
   - `vb09/web/kinds/<kind>.js`（如需专用面板），同时加入 `flight_monitor.py` 的 `WEB_FILES` 与 `vb09/web/monitor.html` 的 `<script>` 列表。
4. **验证**：必跑 `python3 tools/wiring_check.py`、`tools/mode_isolation_check.py`、`tools/py38_check.py`。

---

## 4. 接入新参数 / 档案旋钮 / 网页字段

- **仅参数**：加进 `vb09/flight/config.py`（正数加 `profile/core.py` `POSITIVE_PARAMS`），在基线 `dryrun_dump.py`（`_NEW_KEY`）与 `runtime_dump.py`（`_intended`）登记。
- **档案旋钮**：在 `profile/gate.py` `GATE_KNOBS` 添加并**必须指定 `section`**（除非所有模式都用），映射写进 `GATE_KNOB_PARAM` 或 `_build_gate_overrides`，网页配置页由 `/api/schema` 自动渲染。
- **遥测 / 日志**：`gate` 段在 `flight/mission.py` `_telem_and_log` 写入（可变对象先拷贝）；`gate.csv` 列定义在 `flight/log.py` `GATE_CSV_COLS` 并与 `_telem_and_log` 同步。

---

## 5. 当前架构待完善项（给后续迭代的备忘）

1. **`reddot + dash` 仍标为 `experimental`**：`gate_plan.py` 仍按环内径 `R_IN` 裁剪瞄点、`ApproachDash` 复用含 `D_ALIGN_MIN=2.8m` 的 `d_run_for`，无环可穿区域/安全边界尚未独立抽象。
2. **`preflight.py` 两处模式隔离小缺口**（因 `mode_isolation_check.py` 白名单 `("preflight.py", "*")` 漏检）：`MPC_XY_VEL_MAX < 1.1 * C.DASH_V` 与 `d0 < C.D_ALIGN_MIN - 0.3` 未加 `_ms.uses(C.MISSION_MODE, "dash")` 守卫；另 `[4/6]` 建议补 `st.get("kind") == C.TARGET_KIND` 一致性校验。
3. **起降安全增强（来自室外 `flight_0054–0056` 复盘）**：`phase_takeoff` 建议纳入 `KeyReader`（支持起飞中 `X`/`L` 急停）并增加 `mavros_z - vins_z` 冲突熔断；`phase_descend` 建议增加触地/停滞判据（防 VIO Z 正漂后在地上空转至超时）。
4. **外参 `yaw` 与 `EV_POS_X/Y/Z`**：识别相机 `yaw` 仍有 `±1.5°` 不确定度（需从长距离抵近横向漂移斜率精调）；飞控 `EV_POS_X/Y/Z` 仍为 `0/0/0`。
5. **待清理兼容壳**：确认无旧脚本引用后可移除根目录 5 个兼容转发文件。
