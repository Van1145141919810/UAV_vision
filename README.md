# VB09 穿环打击 · 代码 2026-10-31（模块化重构后）

基线：存档 `VB09_gate_archive_2026-10-30.zip`（git 第一个提交 f2f4b38）。重构计划见项目文档 `claude/PLAN_refactor_v2.md`，
每一步一个提交（`git log`）。接口约定：`docs/INTERFACE_gate.md`（v2，按代码实际行为写）。
**开发指南（加模式 / 步骤 / 目标 / 参数 / 网页时改哪里）：`docs/DEV_GUIDE.md`**。
系统说明：docs/VB09_gate_system_guide.pdf（在线版 https://claude.ai/artifact/TCw68ZtrMRDnAPFB3ethyi，重构前写的）。

## 入口（根目录，都很薄）

| 文件 | 部署 | 作用 |
| --- | --- | --- |
| stage14_mission.py | 飞机 | **飞行入口**（原 stage12_gate.py）。档案 `mode = 冲刺 / 影子 / 停靠` |
| stage12_gate.py | 飞机 | 空壳：打印"已改名"后退出 1 |
| gate_plan.py | 飞机 | 冲刺规划（纯函数，dry-run / 网页预览共用） |
| flight_profile.py | 飞机 | 现场档案（转发 `vb09/profile/`；`python3 flight_profile.py` 照旧可用） |
| flight_monitor.py | 飞机 | 监控网页后端（页面在 `vb09/web/`） |
| run_vision.py + vision_bridge.py | 飞机 | 图传桥：采图、位姿插值、世界系融合、/gate/status |
| gcs_gate.py | 地面站 | 地面检测 |
| gate_detector.py / ring_detector.py / preview_server.py | — | 兼容壳：转发到 `vb09/`，给老脚本 import 用 |

## vb09/

```
common/      geom.py (向量/四元数/靶面坐标)  wire.py (msgpack/zmq + 帧序列化 + 版本检查)  target.py (Target/Lock)  preview_server.py
perception/  stereo_rig.py (校正 + 标定读取)  registry.py (JUDGES)
             ring/  config (GateConfig)  frame  det2d  geom3d  fusion  detect (GateDetector)  draw  locate  judge
flight/      config.py (全部参数, C.名字)  ctx.py (共享状态 F)  telemetry setpoint keys log safety preflight phases runner
             mission.py (MissionCtx + run_mission)  modes.py (MODES)
             steps/  acquire plan_dash reposition align commit dash shadow standoff(StandOff/Hold)
profile/     core.py (解析/格式化/--set/setup/finalize)  gate.py (穿环旋钮/校验/派生/打印)
monitor/     common store checks launcher (+ fly.sh.tmpl) logs ros fake
web/         monitor.html monitor.css core.js live.js ui.js main.js kinds/ring.js views/{monitor,detail,checks,config,help,cam,replay}.js
```

- **飞机**：整个目录（根目录入口 + `vb09/`）放一个文件夹，外加 `stereo_calib.yaml`（`gate_params.json` 可选）。
- **地面站**：`gcs_gate.py` + `vb09/common/` + `vb09/perception/` + `stereo_calib.yaml`。
  ★ 地面和飞机的 `vb09/` 要一起更新：两边启动时都打印 `检测库 xxxxxxxxxxxx  GateConfig yyyyyyyyyyyy`，
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

## SHA-256（部署文件）
    b990d74995ecba6d67f18eb897ebacf8547917cb36865918490c8bd2404b0231  flight_monitor.py
    ab7c360593c70fa5d0667eb7b698ba7c1fae9378af90a91076a145d95fbe883c  flight_profile.py
    5cb3a8737a104bd074af15b0caeef90c3f88f106de1ff0145a368fb987c9900d  gate_detector.py
    c515292cb61bb531bf20b1ac44197e8e66db607a7c95d601162b054166db5f80  gate_plan.py
    70b4409c2155a702e5985b659df0735a7858e87688d16676e542f6f94b74b0d3  gcs_gate.py
    bc1d8e2f581116b91b47a7eb0290bb28e98eb7d8012e240d9a6ad5e800935cfd  preview_server.py
    8a51d5bb95d9727c336f3f5bfc4f6e5c756df7dc4df575f9e6a95f9b55161197  ring_detector.py
    8cea078a65e6fa9dc079001c59f3b17256dda7d0b12efeea1b0cbac796f1b5bd  run_vision.py
    92d6215346aee15ffbb76960085ee016cd9163baf88c48c90e0e383c321ab3ad  stage12_gate.py
    74275e904751080373257554f5dc1ca52a8662facc4d774bbfda7e07e103528b  stage14_mission.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/__init__.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/common/__init__.py
    0ab12d144d8c744d54dd85e3c832ae19a3493b1c8ed7768145a1c687b39d7ac7  vb09/common/geom.py
    41771fe71580c87a3f26839a27c64faa0ab619858db9cce1238910a4784918a4  vb09/common/preview_server.py
    422fc74a5fa3081673570e0090223d197032cd580f98970d1729ff14f36c7a37  vb09/common/target.py
    98f884ab06dcd2694cf196c2d8a938884fa22853e9a0c4ced2feb397ec94c3ff  vb09/common/wire.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/flight/__init__.py
    d20eaf195cc23c65f20d98c91bb0634d9fe6005c60a5de8bcc0fd82d78d8f88d  vb09/flight/config.py
    5384ddf7a831b3690dcbff184526627eec9be344eaa5a4127d36a15faaaad282  vb09/flight/ctx.py
    89c275fba5eff116c4001e980d1ac7483267a9208a519e3e38479916bdab3527  vb09/flight/keys.py
    5f8f7d523b572ead03f799daf42fbeff13d9160b1d5bbd890793ddd94f2ea5db  vb09/flight/log.py
    0ebad0d2611f7e2877d7c62b8adf5a2d90db254b20c37b5534dc7fde35d4c8ab  vb09/flight/mission.py
    8aef9c274424cd30c06126813d15d84d61209b6e75bb67bd044da9976702b937  vb09/flight/modes.py
    18868b532907c5560341bbfef5c723371cffafead7b215758d19ee2df8fd42f1  vb09/flight/phases.py
    1b6c481aa7fb778d4728b3e345c58e87ec4bc41bf974cb45bc1a19c2f726a817  vb09/flight/preflight.py
    36c2b9000ae4f1da862358566c1204a94bade773117c8e14edad9706c896fcdd  vb09/flight/runner.py
    8923a147da74fdb1d047a85cfdb1c4e2bbef7dfe2606f6b2d824235d6d64f290  vb09/flight/safety.py
    365699ddc1a03f789e7ff46bd47dce3f4775fbcb70f2168d78f51b3faa3f2a31  vb09/flight/setpoint.py
    2904d4fa91511fc566f1d31c4951b28df0e30166e224bc9321b481a8d3ebeeca  vb09/flight/steps/__init__.py
    4a0c82bfa9eeb51fb9ef058465dc93ec5ac6da842a046836297824a636273609  vb09/flight/steps/acquire.py
    898cd8bab20491bcf40232142d962c8acbf70c786406f4dd8068de7a76269618  vb09/flight/steps/align.py
    571d69934881fa109c4bae74c305ad810aa6af8f1a25101f54c489656b7ed881  vb09/flight/steps/base.py
    a18c5dab5fb37f65e20a27dccf86c0a3ddf20025f93688a6a294615a506a133e  vb09/flight/steps/commit.py
    ff8d3e693826e1a69fd8ee313852a6ccf104f0cfdeacbe72cc81029a15975255  vb09/flight/steps/dash.py
    4d04d5fd1e8206ae1a8f3f81eee7c302b86cda0cb74e3de233afdc7189aa3e83  vb09/flight/steps/plan_dash.py
    855b66113cc9b14dbbe3b5bc586972bc022aef35ba0d854ecdd2796649f84122  vb09/flight/steps/reposition.py
    cb86c562501b070b898cc9d35565e70cfabf86bbb7a799efdb2cdf2314de7ec8  vb09/flight/steps/shadow.py
    d3cda615a3c03a54af7e4e0f996791b80b6b090654b2b59cbdf8fe1d615f9bf1  vb09/flight/steps/standoff.py
    e10eef430b38c29e10c2b0642a1808f20bba72f4983396d784c56eb44e525f9d  vb09/flight/target_in.py
    d4fb91f8b79b38dd096c6f4cad3095530cc6abbfd3dfb5504cd441ddd1524dff  vb09/flight/telemetry.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/monitor/__init__.py
    79b7bf89749db6747aad5b63c968fc49acf7b955bd3664578e83bb2b1594c0b3  vb09/monitor/checks.py
    63d1ccf21ac07a7f3c865f45c7d239de2cac2d1abd5cf78f40480f8746b48f1f  vb09/monitor/common.py
    4a10630b072fac688c6c9d25a8399f473b6bd0da32fc21fb84705ac7d4735a0d  vb09/monitor/fake.py
    345b9fbe630a97ba6bdabd25c92a8c0e7040f6834ac63335cfa79c0a0ace8c6f  vb09/monitor/fly.sh.tmpl
    cdf845ea1e5e8350c35af2cab32c7a32fa0860e7f18732c7ea3fda4c40e23cbc  vb09/monitor/launcher.py
    762dc932c04290323687c74f3024abf32362d7e09f92e7c5b653a048ebf83d2a  vb09/monitor/logs.py
    81eb44958c1e744b58f170a486fe9a3b6141408ccb810d81a9e9ee776c941590  vb09/monitor/ros.py
    a8fd6d8dd34e69fc36098b2879cd30fb0a34f2acc43a1c4023f894b12ce1e709  vb09/monitor/store.py
    7a38c8a1230cc1caf4d067d694dfccfbb8f54222255b15ceab5437af9f28f084  vb09/perception/__init__.py
    03f33c21a7a6427291539ff3b322f508dc150d2622e380b1ed6421854552ca68  vb09/perception/registry.py
    c8db02c904415786122e6cba38c07015aa193ecfc1789d6ff67112e48a58ad2d  vb09/perception/ring/__init__.py
    95b5fdce7a67f3b1dc55472a3c158110db4aebd4a350372d7d08fbf9f836a4a8  vb09/perception/ring/config.py
    523a61986fbf89e1673511b41f589fe87239a2aa5c6d2e76d448d28564bc6e3a  vb09/perception/ring/det2d.py
    a1e02d6dfd960ffc961bdbde0faf34e68d2f6008b2a96efb18d26a8b4e47b579  vb09/perception/ring/detect.py
    7775dff842f62f400fb5c53a92c3a30cff22518430c207e4cb8337f431298a71  vb09/perception/ring/draw.py
    fc033f4ff3ffd0aaa221581d05fce7efa6e324a44bd39b124e0d9b5691ece494  vb09/perception/ring/frame.py
    9028849c2b693b768691cd589ed37e6303aedb82b4bc99ca780dcd0857eab93e  vb09/perception/ring/fusion.py
    b2a097735c43dbbe0a5eb85b71406f57f5183e201ba2684e013f8e6f6d51cce7  vb09/perception/ring/geom3d.py
    6e09c74881dd646a3db079513eb643ce3007662b2b57159d0372937d8000fcc9  vb09/perception/ring/judge.py
    ce0cea952de1827d375880d9895eb450ea68b85232094575928e8dd67dec6929  vb09/perception/ring/locate.py
    825205760bce8a721f545f627bed9b05a61fd1fe758b314d543ad3f81cd4c130  vb09/perception/stereo_rig.py
    e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855  vb09/profile/__init__.py
    db065a48d58733eb51381d100d4c5068da846277bd0e0b49d59fae7b8b7a1d71  vb09/profile/core.py
    402fe86528cfa46a2df7841f81847b1c3471686037607c31af57339bddff4bc4  vb09/profile/gate.py
    ad2c4d7281ae66bfe5c602ec31cc0a19320c0661c6a479fe894cf002dc0fd96a  vb09/web/core.js
    c23fb098b48cbd87f9f31db56a26101bc7c3a9894e9db6429672b5800dc37e73  vb09/web/kinds/ring.js
    98747f2a0604a14850f2291cd92832cc7b97d4aaa078e854601250c943428850  vb09/web/live.js
    0ed75ba6fbe954001a983f776c104b49a00fd6abbac3cbdf84e9a941c042be94  vb09/web/main.js
    dc1df2cc0376378392ea35c060e96eb3ba835cfcb41dcbb366dafc1038413c22  vb09/web/monitor.css
    1a38bab98361b3aec8f6e3cc45a7c6096db202cb51adbf7dfb2ade3ad42de785  vb09/web/monitor.html
    d7030f002c6d873da0b3f7c725894a56c3a0e21336ac02c955541624bc384a5f  vb09/web/ui.js
    1a30bd7de2df92e4765b82d33dabb1b7df4b89439d050a3b9e8029cb2cf6b8e9  vb09/web/views/cam.js
    8093a514841094e0607f0efb4f9145174e1f863593fc7b274f62876b4f0e954a  vb09/web/views/checks.js
    139f715152508bdaf80c4f50aba3dc230c6758c00a73388962eb51f9c8803879  vb09/web/views/config.js
    56703662155c78ece82cc613e1ffd8fa97065237e6a1518e89e60748714a05d3  vb09/web/views/detail.js
    c7c098875b6a2ca10ee8bfd6cef12b9fb26ba67af4e927564a68c47e9c761b34  vb09/web/views/help.js
    46e92d2ae61c7bebf30e3a0405b6aadbc9d0f0805d54e6266a11dd238755fe42  vb09/web/views/monitor.js
    d4e2317125548669c7885d0cedf56db8085a382fa46fbc9da9a4b2e99f502591  vb09/web/views/replay.js
    0acf5eac71d7a4d61a9e798eddf13044a176cc6e2746bb11e25eff7bbfb1ca07  vision_bridge.py
