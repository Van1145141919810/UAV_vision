# mcpv — 无人机画面里的红色标记检测

给一张无人机拍的照片，输出 **这段 JSON** 和 **一张画了框的图**。

链路由两段组成：**传统 CV 找候选** → **MiniCPM-V 判定"这是不是标记"**。
不是让大模型去"找"（它找不准），而是让它做**最后一道语义保险**。

```
图 → CV(归一化色度/圆度/局部色度对比) → 候选 → 裁剪放大 → MiniCPM-V(A/B) → JSON + 标注图
                                              ↘ 没候选就直接判"无"，不调模型
```

---

## 快速开始

### 方式 1：VS Code 一键（推荐）

用 VS Code 打开本文件夹，然后：

| 操作 | 结果 |
|---|---|
| **`Ctrl+Shift+B`** | 跑全部样本，打印表格，输出标注图 |
| **`F5`** | 同上（调试模式） |
| 右键某张图 → 任务 `检测当前打开的图片` | 只跑这一张 |

### 方式 2：双击

双击根目录的 **`run_demo.cmd`**。

### 方式 3：命令行

```powershell
$env:PYTHONPATH="src"
python scripts/demo.py                                   # 全部样本
python -m mcpv.cli data\samples\test2.jpg --save          # 单张
python -m mcpv.cli data\samples\test2.jpg --json          # 只要 JSON
python -m mcpv.cli data\samples\test2.jpg --no-vlm        # 只跑 CV（不调模型）
```

> 结果都在 `data/results/`：`<名字>.json` 和 `<名字>_annotated.jpg`。

---

## 点击 → 飞行（交互演示）

```powershell
$env:PYTHONPATH="src"
python scripts\demo_click.py                         # 从 data/samples 里挑图
python scripts\demo_click.py data\samples\win06.jpg   # 指定一张
```

窗口里：

| 元素 | 含义 |
|---|---|
| 淡绿细框 | 窗户提示（**纯装饰**：漏了也能点，误报不点就行） |
| 黄框 | 红点候选（CV 出的） |
| **左键点任意位置** | 选中这个点，立刻算出"前/左/上 各飞多少米" |
| 红色准星 | 点击位置；箭头是飞行方向 |
| 顶部文字 | `前 2.86 m  右 0.90 m  下 0.05 m` |

按键（**刻意避开飞行键**，免得手抖按错）：

| 键 | 作用 |
|---|---|
| `n` / `p` | 下一张 / 上一张 |
| `v` | 开关窗户提示框（v = view，**不是 W**） |
| `Esc` | 取消当前选中 |
| `回车` | 发送（存档 JSON） |
| `q` / 关窗口 | 退出 |

> 为什么不用 W/R/Q/E：这些是文豪手动段的飞行键（WASD 平移、RF 升降、QE 转向）。
> 界面和飞行**共用一套键盘习惯**，占一个就容易在演示中途按错。

**这一段不调用任何模型、不连飞机**，纯几何：

```
像素 (u,v) → 相机内参反投影 → 机体系方向 → 乘 step_m
若已知距离 Z，则自动收在 (Z − standoff) 以内，避免冲过头
```

想知道目标多远，用"已知实物尺寸"反算：`Z = f × 实物尺寸 ÷ 图上像素`。
红点用 **0.20 m**、窗户用 **2.0 m**（都在 `config.py` 里改）。

---

## VLM 语义层 + JSON 交接单（这是给 Jev 的东西）

```powershell
$env:PYTHONPATH="src"
python scripts\demo_scene.py                                  # 自动挑一扇窗
python scripts\demo_scene.py data\samples\win06.jpg --click 2100,1400
python scripts\demo_scene.py data\samples\test3.jpg --click 880,690 --note
```

跑完会打印并保存一份 JSON：

```json
{
  "schema": "mcpv/record-v1",
  "time_str": "2026-11-04 19:16:02",
  "image": "data/samples/test3.jpg",
  "selection": { "kind": "point", "point": [880.0, 690.0] },
  "vlm": {
    "is_window": false,
    "window_state": "n/a",
    "glare": "有",
    "note": "",
    "raw": "窗户=B / 开合=skip / 反光=C",
    "elapsed_s": 1.5
  },
  "geometry": { "forward_m": 2.975, "left_m": -0.065, "up_m": -0.383 }
}
```

**字段分工（重要）：**

| 字段 | 谁产出 | 谁消费 | 说明 |
|---|---|---|---|
| `selection.bbox_px` / `point` | CV 或人 | 文豪（瞄准） | 几何 |
| `geometry.*` | 公式 | 飞控 | 位移指令 |
| `vlm.is_window` / `window_state` / `glare` | **MiniCPM-V** | **Jev** | **CV 给不出来** |
| `vlm.raw` | MiniCPM-V 原话 | 人 | 证据 |

**JSON 存在哪：**

```
data/results/record_<时间戳>.json    ← 每按一次"发送"/跑一次脚本，一个文件
data/results/session.jsonl           ← 所有记录追加成一行一条，方便回放
```

点击演示里的"发送"也会走同一条路：**双击锁定 → 回车 → 自动跑语义层 → 存 JSON**，
窗口底部会显示 `已发送 窗=开着 反光=有 → record_xxx.json`。

---

## 目录结构

```
MiniCPM-V/
├── src/mcpv/            ← 源代码（真正的程序）
│   ├── config.py          所有阈值/内参/路径（改参数只改这里）
│   ├── schema.py          输出 JSON 的结构（对下游的契约）
│   ├── cv.py              色度分割 + 圆度 + 局部色度对比
│   ├── vlm.py             Ollama 客户端（显式传 base64，不会丢图）
│   ├── aim.py             像素 → 飞行方向（纯几何，不调模型）
│   ├── scene.py           VLM 语义层（是不是窗户/开着吗/反光吗）
│   ├── report.py          组装 JSON 交接单 + 落盘
│   ├── windows.py         窗户提示框（只用于界面）
│   ├── detect.py          主流程 + 画框
│   └── cli.py             命令行入口
├── scripts/
│   ├── demo.py            一键演示（跑全部样本）
│   ├── demo_click.py      点击 → 飞行 交互演示
│   └── demo_scene.py      单张图：选点 + 语义 + 位移 + 存 JSON
├── tests/               ← 单元测试（10 个，不需要模型）
├── experiments/         ← 前期实验脚本（问法对比、参数搜索等，留档）
├── data/samples/        ← 样本图片（test*/neg*/win*）
├── data/results/        ← 输出（json / 标注图 / 实验 csv）
├── docs/                ← 方案与实测日志（00/01/02）
├── vendor/vb09/         ← 同学给的系统说明（只读，不在这里改）
├── .vscode/             ← 一键运行配置
├── requirements.txt
└── run_demo.cmd
```

---

## 输出示例

```json
{
  "image": "test2.jpg",
  "image_size": [1706, 1279],
  "has_target": true,
  "stage": "vlm",
  "n_candidates": 6,
  "n_passed_cv": 1,
  "elapsed_s": 0.4,
  "vlm_calls": 1,
  "candidates": [
    {
      "cx": 874, "cy": 729,
      "diameter_px": 26.6,
      "bbox_px": [816, 671, 931, 786],
      "circularity": 0.88,
      "ring_s": 23.9, "ring_v": 230.6, "score": 194.8,
      "passed_cv": true,
      "vlm_verdict": "A",
      "vlm_text": "A"
    }
  ],
  "best": { "bbox_px": [816, 671, 931, 786], "mode": "A", "score": 194.8 }
}
```

**字段给谁：**

| 字段 | 谁用 |
|---|---|
| `has_target` / `best.bbox_px` | 文豪的飞行脚本（要不要接近、往哪接近） |
| `candidates[].vlm_verdict` / `ring_*` / `score` | Jev（张光明）做决策的输入 |
| `vlm_text` | 人看（证据），**不进控制** |
| `image_size` / `bbox_px` | 换算到相机坐标 |

> ⚠️ `bbox_px` 是**CV 给的**，不是模型给的。模型输出的坐标实测偏 200 px，不可用。

---

## 环境（**统一用 mcpv 虚拟环境**）

| 项 | 值 |
|---|---|
| Python | `C:\miniforge3\envs\mcpv\python.exe`（3.12.16） |
| 已装 | `opencv 5.0.0` / `numpy 2.5.4` / `pillow 13.0.0` |
| 不需要 | `torch` / `transformers`（推理走 Ollama，不在 Python 里） |
| 模型 | `openbmb/minicpm-v4.7:1b`（Ollama，1.8 GB） |

**所有启动入口已经写死这个解释器**，不用手动 activate：

- `run_demo.cmd` / `run_click.cmd` / `run_scene.cmd`（内部 `set "PY=...mcpv\python.exe"`）
- VS Code 任务与 F5 配置
- 脚本启动时第一行会打印 `Python: ...`，**跑错环境一眼能看出来**

如果要在终端里手动跑：

```powershell
conda activate mcpv
$env:PYTHONPATH="src"
python scripts\demo_click.py
```

> ⚠️ 别用 base：base 里也有 opencv，但版本不同，混着跑会出现"我这能跑你那不能跑"的怪问题。

- 模型：`openbmb/minicpm-v4.7:1b`（Q4_K_M，约 1.8 GB，4 GB 显卡够用）
- 服务：Ollama 跑在 `http://127.0.0.1:11434`
- 依赖：只有 `opencv-python / numpy / pillow`，**不需要 torch**

---

## 已知限制（务必先读）

### 两种工作点（一个是"看得远"，一个是"不误报"）

同一套代码，只改 `config.py` 里的 `AREA_MIN`：

| 配置 | 真实 8 张 | 合成 10–20 px | 合成 7 px | 无目标对照误报 |
|---|---|---|---|---|
| `AREA_MIN=20`（**默认，召回优先**） | 7/8 | **6/6** | 4/6 | 2/6 |
| `AREA_MIN=80`（保守，只看大目标） | 8/8 | 2/6 | 0/6 | 0/6 |

默认选 20：**任务本身是"远处找到它"，漏检比误报更致命**。
代价是桌面上一个红色圆形指示灯会被当成目标（neg2 就是这种情况）——这种歧义要靠多帧/上下文/人工解决，1B 模型分不开。

| 限制 | 数据 |
|---|---|
| **100 m 处看不到目标** | 20 cm 标记在 1080p、f=1579 下只有 **3.2 px**；实测检出下限 **4–7 px**（45 m 能认，79 m 认不出） |
| 样本只有 8 张 | 阈值是在这 8 张上调出来的，**有过拟合风险** |
| 单帧看一次 6 s | 视觉编码是主要成本，**不能做连续闭环**，只能"停-看-走" |
| 模型给不了置信度 | 让 1B 打 0–100 分，它只会吐 0/5/10 ⇒ 概率必须交给 Jev/Laya |
| 小目标歧义 | 7 px 以下，红色指示灯和红色标记在 1B 眼里几乎一样 |

---

## 测试

```powershell
pip install pytest
python -m pytest tests -q
```

6 个用例，**不需要模型**（只测 CV 逻辑和数据结构），几秒钟跑完。

---

## 改参数

全在 `src/mcpv/config.py`。最常改的三个：

| 参数 | 默认 | 作用 |
|---|---|---|
| `RING_SCORE_MIN` | 100 | 白底打分阈值（调高=更严，漏报↑误报↓） |
| `RING_V_MIN` | 150 | 环带亮度下限 |
| `CIRC_MIN` | 0.60 | 圆度下限（0.6 以下不算"圆"） |

---

## 文档

| 文件 | 内容 |
|---|---|
| `docs/00_判断与初步方案-20261031.md` | 项目判断、像素预算、三层架构 |
| `docs/01_系统分层与接口方案-20261102.md` | SEEK 段设计、与文豪状态机的接口、交接协议 |
| `docs/02_实测日志-20261102.md` | 全部实测数据（问法对比、涂掉对照、距离阈值、grounding 失败） |

---

## 收进 VB09 主仓库后的说明（2026-11-26 黄圣鑫）

这一份是从本地工程原样搬进 `UAV_vision/MiniCPM-V/` 的，**只收代码和小文本**。
下面几件事换台机器必须自己做一遍，否则跑不起来：

### 1. 模型权重不入库（687 MB）——自己下

```powershell
# 方式一：huggingface-cli
pip install huggingface_hub
huggingface-cli download openbmb/MiniCPM-V-2_6 --local-dir MiniCPM-V\.hf\MiniCPM-V-2_6

# 方式二：直接 git lfs clone 到 .hf/ 目录
git lfs install
git clone https://huggingface.co/openbmb/MiniCPM-V-2_6 MiniCPM-V\.hf\MiniCPM-V-2_6
```

模型目录要落在 **`.hf\`** 下（`.gitignore` 已忽略）。
把 `HF_HUB_OFFLINE=1` 设上可以省掉每次启动 20 秒的 hub 超时探测。

### 2. 虚拟环境不入库（277 MB）——自己建

```powershell
python -m venv venv_det
venv_det\Scripts\pip install -r MiniCPM-V\requirements.txt
```

### 3. 本 README 上面的「快速开始」有一处过期

- 里面提到的 `run_demo.cmd` **已经删掉了**，用 `起飞.cmd` 的菜单或 `scripts/` 下的脚本
- `data/samples/`（20 张实测 jpg，77 MB）**没有入库**，所以上面那些示例命令默认会找不到图。
  自己往 `MiniCPM-V\data\samples\` 丢图就行，目录结构不变。

### 4. 和主仓库的耦合点只有一个

`gcs_gate.py` 里有一小段可选钩子，把三路融合建议画到网页「识别画面」上：

```python
_ADVICE_ON = True    # 改成 False 就完全恢复原样
```

它按这个顺序找 `scripts/advice_overlay.py`：
`环境变量 CPMV_ADV_DIR` → `<仓库根>/MiniCPM-V/scripts` → `<上两级>/MiniCPM-V/scripts`。
**三个都找不到就自动关闭**，网页画面和以前完全一样，不影响检测和推流。

### 5. 已知事项

- `scripts/` 和 `docs/` 里有一批**写死了本地绝对路径**（`D:\Desktop\无人系统\...`）的
  演示/评测脚本 —— 都是离线跑图用的，不影响飞行链路，但换机器要改路径。
- 本目录**不含**：`.hf/`（权重）、`venv_det/`（虚拟环境）、`data/`（原始素材）、
  `old/`（旧版菜单）、`vendor/`（外部参考资料 PDF）、`*.bak_*`（历史备份）、
  `eval/` 下的出图目录（`figures/`、`demo_out/`、`*_figs/`、`vlm_out/`）。

