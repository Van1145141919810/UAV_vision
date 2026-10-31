# VB09 穿环打击 · 代码 2026-11-01（模块化重构 + 合并 + 感知插件接线后）

基线：存档 `VB09_gate_archive_2026-10-30.zip`（git 第一个提交 f2f4b38）。重构计划见项目文档 `claude/PLAN_refactor_v2.md`，
每一步一个提交（`git log`）。接口约定：`docs/INTERFACE_gate.md`（v2，按代码实际行为写）。
**开发指南（文件地图、调用链、加模式 / 步骤 / 目标 / 参数 / 网页时改哪里）：`docs/DEV_GUIDE.md`**，改东西之前先看它。
系统说明：docs/VB09_gate_system_guide.pdf（在线版 https://claude.ai/artifact/TCw68ZtrMRDnAPFB3ethyi，重构前写的）。

## 入口（根目录，都很薄）

| 文件 | 部署 | 作用 |
| --- | --- | --- |
| stage14_mission.py | 飞机 | **飞行入口**（原 stage12_gate.py）。档案 `mode = 冲刺 / 影子 / 停靠` |
| stage12_gate.py | 飞机 | 空壳：打印"已改名"后退出 1 |
| gate_plan.py | 飞机 | 冲刺规划（纯函数，dry-run / 网页预览共用） |
| flight_profile.py | 飞机 | 现场档案（转发 `vb09/profile/`；`python3 flight_profile.py` 照旧可用） |
| flight_monitor.py | 飞机 | 监控网页后端（页面在 `vb09/web/`） |
| run_vision.py + vision_bridge.py | 飞机 | 图传桥：采图、位姿插值、世界系融合、/gate/status。`--target`（= `_target:=`）选目标类型 |
| gcs_gate.py | 地面站 | 地面检测。`--target` 选目标类型（HUD 现在只支持 ring） |
| gate_detector.py / ring_detector.py / preview_server.py | — | 兼容壳：转发到 `vb09/`，给老脚本 import 用 |

## vb09/

```
common/      geom.py (向量/四元数/靶面坐标)  wire.py (msgpack/zmq + 按 kind 分发序列化 + 版本检查)  target.py (Target/Lock)  preview_server.py
perception/  registry.py (插件注册表: CODECS / CONFIGS / DETECTORS / LOCATORS / JUDGES)  stereo_rig.py (校正 + 标定读取)
             ring/  plugin (RingDetector / RingLocator)  wire (环的消息格式)  config (GateConfig)  frame  det2d  geom3d
                    fusion  detect (GateDetector)  draw  locate  judge (RingJudge)
flight/      config.py (全部参数, C.名字)  ctx.py (共享状态 F)  devices.py (遥测 / 设定点 / 键盘)  log safety preflight phases runner
             target_in.py (收 /gate/status → JUDGES[TARGET_KIND])  mission.py (MissionCtx + run_mission)  modes.py (MODES)
             steps/  base  common (Acquire, move_to, turn_to, align_lock)  dash  shadow  dock
profile/     core.py (解析/格式化/--set/setup/finalize)  gate.py (穿环旋钮/校验/派生/打印)
monitor/     common store (+ 日志回放) checks launcher (+ fly.sh.tmpl) ros fake
web/         monitor.html monitor.css core.js live.js main.js kinds/ring.js views/{monitor,detail,checks,config,help,cam,replay}.js
```
- **飞机**：整个目录（根目录入口 + `vb09/`）放一个文件夹，外加 `stereo_calib.yaml`（`gate_params.json` 可选）。
- **地面站**：`gcs_gate.py` + `vb09/common/` + `vb09/perception/` + `stereo_calib.yaml`。
  ★ 地面和飞机的 `vb09/` 要一起更新：两边启动时都打印 `目标 / 检测库 xxxxxxxxxxxx / GateConfig yyyyyyyyyyyy`，
  不一致时飞机把地面结果强制当 SIM（不驱动），网页「环识别」报"版本不符"。
- 飞机 Python 3.8（`tools/py38_check.py` 查语法），地面 ≥ 3.8。

## 模式（档案 `mode`）

| mode | 步骤 (`vb09/flight/modes.py`) | 说明 |
| --- | --- | --- |
| 冲刺（默认） | Acquire → PlanDash → Reposition → Align → Commit → Dash | 和重构前逐段一致 |
| 影子 | Acquire → PlanDash → AlignHere → Shadow | 旧写法 `set.GATE_SHADOW = True` 同义（会提示） |
| 停靠 | Acquire → StandOff → Hold | **新**：飞到目标前 `dock_dist`（默认 max(3, 最小助跑)）对正，悬停 `dock_hold` 秒（默认 20）记录检测，打印停靠误差 / aim 抖动，回手动段。不冲刺 |

## 验证（`tools/`）

```
python3 tools/baseline/run_all.py            # 等价基线: dry-run / 规划 / 检测 / 融合 / judge / sim 10 场景 / 运行时产物
python3 tools/baseline/run_all.py --quick    # 不跑 sim
python3 tools/baseline/log_compare.py        # 日志模板和删除步骤后 (e6b73cb) 比
python3 tools/calib_check.py                 # 标定读取 (尺寸 / swap_halves / flip_180)
python3 tools/wiring_check.py                # 感知插件接线 (注册表 / 接口 / 不拉 cv2 / 版本检查认 kind)
python3 tools/geom_check.py                  # Python 和网页的靶面坐标对拍 (需要 node)
/path/to/python3.8 tools/py38_check.py
node tools/dev/page_check.js <输出目录>       # 网页冒烟 (需要 playwright)
```

golden 和 Python 版本绑定（float 末位）：现在的 golden 是 Python 3.13 生成的。

## 行为变化（相对存档，都有单独提交）

| 提交 | 变化 |
| --- | --- |
| 1a/1b/1c | 删戳点、stage11 / 停靠跟踪、死代码。档案里的已删除键警告并忽略；`gate_mode = 戳点` 报错 |
| 1b | 桥在 gate / gate_gcs 模式下不再发 `/track/target`、`/gate/target`（aruco 模式不变） |
| 4.5 | 新 `mode` 旋钮；旧 `GATE_SHADOW = True` 多一条提示 |
| 4.6 | 入口改名 stage14_mission.py |
| 6.1 | 标定文件写的尺寸和程序的单目分辨率不一致 → 拒绝启动 |
| 6.2 | 标定 `swap_halves` / `flip_180` 不为 0 → 拒绝启动 |
| 6.3 | 地面 / 飞机检测库、GateConfig、消息版本不一致（或旧版 gcs_gate）→ 强制 SIM + 网页报出 |
| 7 | 停靠模式（新功能） |
| 合并 / 接线（2026-11-01） | 行为不变（全部基线 + 新的 onboard_replay 逐字段相同）。检测库指纹的文件集合变了 → 指纹值变，地面和飞机一起更新 |

## SHA-256（部署文件）
    e035bbd5f203c73048b063267a5e4f8f7ea185c4b1119c0d5f5fef01e4b93dc9  flight_monitor.py
    ab7c360593c70fa5d0667eb7b698ba7c1fae9378af90a91076a145d95fbe883c  flight_profile.py
    7846a2126be861f74716fa115b29c3bb0036e35ab1f3e52e6ec281bea7a0de6a  gate_detector.py
    c515292cb61bb531bf20b1ac44197e8e66db607a7c95d601162b054166db5f80  gate_plan.py
    62641b633c395ed4aed7116565a60dcda8b9dae54766cd3fc55fe7326b710d86  gcs_gate.py
    bc1d8e2f581116b91b47a7eb0290bb28e98eb7d8012e240d9a6ad5e800935cfd  preview_server.py
    8a51d5bb95d9727c336f3f5bfc4f6e5c756df7dc4df575f9e6a95f9b55161197  ring_detector.py
    02b968aa76263a439aa4277662cc662f51804a2d1ba9f3c434a70da0776304d3  run_vision.py
    92d6215346aee15ffbb76960085ee016cd9163baf88c48c90e0e383c321ab3ad  stage12_gate.py
    74275e904751080373257554f5dc1ca52a8662facc4d774bbfda7e07e103528b  stage14_mission.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/__init__.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/common/__init__.py
    0ab12d144d8c744d54dd85e3c832ae19a3493b1c8ed7768145a1c687b39d7ac7  vb09/common/geom.py
    41771fe71580c87a3f26839a27c64faa0ab619858db9cce1238910a4784918a4  vb09/common/preview_server.py
    422fc74a5fa3081673570e0090223d197032cd580f98970d1729ff14f36c7a37  vb09/common/target.py
    d3365aceb91922804a8411a499951a6f0b44e0ed8f0dfe175026dc0716765da5  vb09/common/wire.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/flight/__init__.py
    b891fc6b2210f905f6a03f63b480605739180aef045118afc78f2ecf5adc38a7  vb09/flight/config.py
    bbaa2651a0904c36182fabab2bfcb7c91d4d551adf84778248d04bf1537db724  vb09/flight/ctx.py
    7283250fa0089c5e0a2ebc8ad4a6672cc8c6c0d90c4bbfe305f05d68caf3390d  vb09/flight/devices.py
    bbc102c0a8accf4db87a7c600f4b4e4893fa2b804a29d251846cb43685cd3443  vb09/flight/log.py
    32eaa05f6be886de652fc02578ac0b99b3c16ef3aae45f5c6bb9529395a167d6  vb09/flight/mission.py
    575ed197a334119ee90f3a728eaccf28e3accf48fa43daf16e01bdb493e90bb8  vb09/flight/modes.py
    0ae086865f2833f716f0ea8cd826a399d401665f34e135ae5d03b61ed8f65805  vb09/flight/phases.py
    4d66b2cff21c9940116448a76f8a24e8a5a8f86fc21119f47ff093c07f7dde4a  vb09/flight/preflight.py
    36c2b9000ae4f1da862358566c1204a94bade773117c8e14edad9706c896fcdd  vb09/flight/runner.py
    864ee4b2d43019b31bb2e6b133357a38dd61c9fb0a8cd9fa61d7d83ab65be6fb  vb09/flight/safety.py
    e89b261e91817c22c56969e75d9c229ef90bb4b7061d8036ed12738caf222869  vb09/flight/steps/__init__.py
    571d69934881fa109c4bae74c305ad810aa6af8f1a25101f54c489656b7ed881  vb09/flight/steps/base.py
    6290179c7ce813f8b47d5cb1202f6bc4f66b7ae43d39334aced5325578ad4a98  vb09/flight/steps/common.py
    d82b0328292c0b33aad8fb5bf900bdad291b7c93bc1a88d7835a84bc1945110d  vb09/flight/steps/dash.py
    0168958beedc75069ba408c8e8644e679154e497f9e8861bcee762237865b0ef  vb09/flight/steps/dock.py
    06a6567e30a884233721abc9ebf5729eda547a62ac433219e423a2e0aa299296  vb09/flight/steps/shadow.py
    c934c724041800d139a30d0605174d114fa0bd9d18de8be06558390410a6a0a6  vb09/flight/target_in.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/monitor/__init__.py
    79b7bf89749db6747aad5b63c968fc49acf7b955bd3664578e83bb2b1594c0b3  vb09/monitor/checks.py
    63d1ccf21ac07a7f3c865f45c7d239de2cac2d1abd5cf78f40480f8746b48f1f  vb09/monitor/common.py
    4a10630b072fac688c6c9d25a8399f473b6bd0da32fc21fb84705ac7d4735a0d  vb09/monitor/fake.py
    345b9fbe630a97ba6bdabd25c92a8c0e7040f6834ac63335cfa79c0a0ace8c6f  vb09/monitor/fly.sh.tmpl
    cdf845ea1e5e8350c35af2cab32c7a32fa0860e7f18732c7ea3fda4c40e23cbc  vb09/monitor/launcher.py
    81eb44958c1e744b58f170a486fe9a3b6141408ccb810d81a9e9ee776c941590  vb09/monitor/ros.py
    e34587d15b42f2d9a3adf6290a4bde6bb1536cd9fb7b50e60828cc63926aca1a  vb09/monitor/store.py
    7a38c8a1230cc1caf4d067d694dfccfbb8f54222255b15ceab5437af9f28f084  vb09/perception/__init__.py
    8dd4a774511b4a4219679d538ccc7eb252e4626158c0596dec806b818d4d47a5  vb09/perception/registry.py
    a240e1a0ee700dfa69b81d8b1520392d932b2ff03ba43da5b085dea1fdc8d74b  vb09/perception/ring/__init__.py
    95b5fdce7a67f3b1dc55472a3c158110db4aebd4a350372d7d08fbf9f836a4a8  vb09/perception/ring/config.py
    523a61986fbf89e1673511b41f589fe87239a2aa5c6d2e76d448d28564bc6e3a  vb09/perception/ring/det2d.py
    a1e02d6dfd960ffc961bdbde0faf34e68d2f6008b2a96efb18d26a8b4e47b579  vb09/perception/ring/detect.py
    7775dff842f62f400fb5c53a92c3a30cff22518430c207e4cb8337f431298a71  vb09/perception/ring/draw.py
    95f42eeec636a1bb9020bcd8f079397f20457e52fe40e4f6a7845568bf036bfa  vb09/perception/ring/frame.py
    9028849c2b693b768691cd589ed37e6303aedb82b4bc99ca780dcd0857eab93e  vb09/perception/ring/fusion.py
    b2a097735c43dbbe0a5eb85b71406f57f5183e201ba2684e013f8e6f6d51cce7  vb09/perception/ring/geom3d.py
    6e09c74881dd646a3db079513eb643ce3007662b2b57159d0372937d8000fcc9  vb09/perception/ring/judge.py
    ce0cea952de1827d375880d9895eb450ea68b85232094575928e8dd67dec6929  vb09/perception/ring/locate.py
    4be24e6c9fc3c92313293eb31c87392c738d71162b2b98468bd3368b5e89aa7b  vb09/perception/ring/plugin.py
    819d48b9ce8dbb9b18960d49e763e683a0e80747638cb8058374528f8b4f1a2a  vb09/perception/ring/wire.py
    825205760bce8a721f545f627bed9b05a61fd1fe758b314d543ad3f81cd4c130  vb09/perception/stereo_rig.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/profile/__init__.py
    db065a48d58733eb51381d100d4c5068da846277bd0e0b49d59fae7b8b7a1d71  vb09/profile/core.py
    73e0dafca3e1de79027de4a98d0faf8789b0521a5131a61e60896ca88d9141c5  vb09/profile/gate.py
    ad2c4d7281ae66bfe5c602ec31cc0a19320c0661c6a479fe894cf002dc0fd96a  vb09/web/core.js
    c23fb098b48cbd87f9f31db56a26101bc7c3a9894e9db6429672b5800dc37e73  vb09/web/kinds/ring.js
    98747f2a0604a14850f2291cd92832cc7b97d4aaa078e854601250c943428850  vb09/web/live.js
    7cb5d946b1d857721f475e071462fe2d8682b6c6df47b4feec2fd28a9656663e  vb09/web/main.js
    dc1df2cc0376378392ea35c060e96eb3ba835cfcb41dcbb366dafc1038413c22  vb09/web/monitor.css
    f097dd69eca80d9722fbd426aa2e6c16941ff6318e2c7c8ade9184142296a054  vb09/web/monitor.html
    1a30bd7de2df92e4765b82d33dabb1b7df4b89439d050a3b9e8029cb2cf6b8e9  vb09/web/views/cam.js
    10608e726359c2652ce9dae2c53cad7faf1fea5a22e62a84ee19f3a03bde224b  vb09/web/views/checks.js
    139f715152508bdaf80c4f50aba3dc230c6758c00a73388962eb51f9c8803879  vb09/web/views/config.js
    56703662155c78ece82cc613e1ffd8fa97065237e6a1518e89e60748714a05d3  vb09/web/views/detail.js
    c7c098875b6a2ca10ee8bfd6cef12b9fb26ba67af4e927564a68c47e9c761b34  vb09/web/views/help.js
    46e92d2ae61c7bebf30e3a0405b6aadbc9d0f0805d54e6266a11dd238755fe42  vb09/web/views/monitor.js
    d4e2317125548669c7885d0cedf56db8085a382fa46fbc9da9a4b2e99f502591  vb09/web/views/replay.js
    ddffe6fd5e25c4c2ca902b6fb26ca9823f868bceb51372ab9496726ac92d0c24  vision_bridge.py
