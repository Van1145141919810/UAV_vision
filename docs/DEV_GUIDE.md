# VB09 开发指南：加东西时改哪里 — 2026-10-31

给以后写代码的人（和新对话）用。目录和调用链见 README；接口字段见 `docs/INTERFACE_gate.md`。
本文回答：**要做 X，该动哪几个文件、按什么顺序、怎么验证**。

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
