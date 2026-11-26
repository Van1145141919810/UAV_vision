# UAV_vision (VB09 / OWL2)

OWL2 / VB09 自主视觉识别与飞行控制代码（PX4 v1.12.3 + ROS Noetic + 双目视觉，支持 `ring` 穿环与 `reddot` 无环红点，任务模式 `冲刺 / 影子 / 停靠`）。

> 机载环境为 Python 3.8（Khadas `visbot`，`192.168.2.20`），地面站 Python ≥ 3.8（标定工具需 OpenCV ≥ 4.7）。**飞机与地面站的 `vb09/` 和 `stereo_calib.yaml` 必须同步更新**，否则版本检查会强制进入 SIM 模式。

---

## 一、快速启动

### 1. 飞控与 VIO（飞机 tmux，每次开机必做）

```bash
# 窗口 1：MAVROS（把 <QGC_IP> 换成地面站 IP）
roslaunch mavros px4_visquad.launch fcu_url:=/dev/ttyS7:921600 gcs_url:=udp://@<QGC_IP>:14550

# 窗口 2：VINS（开机自启会崩，需手动拉起）
roslaunch vins stereo120.launch

# 必做：停掉开机自启的 competing 控制器
rosnode kill /geometric_controller
```

### 2. 视觉识别链路

**推荐：地面检测模式（`gate_gcs`）**

```bash
# 地面站先启（--target 选 reddot 或 ring；--mode 1 仅观察/SIM，--mode 2 驱动飞机）
python gcs_gate.py --drone 192.168.2.20 --target reddot --mode 1
# 真飞时改 --mode 2

# 飞机 tmux 再启图传桥（--gcs 填地面站 IP，--target 必须与地面一致）
python3 run_vision.py gate_gcs --gcs 192.168.2.67 --target reddot
```

**备选：机上检测模式（`gate`）**

```bash
# 飞机 tmux（默认 SIM，真飞加 --drive）
python3 run_vision.py gate --target reddot --drive

# 地面站只看画面 / 录屏（可选）
python gcs_view.py --drone 192.168.2.20 --record rec_view/
```

### 3. 网页监控（可选，飞机 tmux + 笔记本端口转发）

```bash
# 飞机 tmux
python3 flight_monitor.py

# 笔记本新开终端做 SSH 端口转发，然后浏览器打开 http://localhost:8080
ssh -N -o ExitOnForwardFailure=yes -L 8080:127.0.0.1:8080 visbot@192.168.2.20
```

### 4. 启动飞行脚本（⚠️ 必须在普通 SSH 终端里，不能套 tmux / VSCode 内置终端）

```bash
# 先做离线预检（任意终端可跑，不依赖 ROS）
python3 stage14_mission.py --site <档案名> --dry-run

# 真飞（普通 SSH 窗口，保证断连时 stdin EOF 触发自动降落）
python3 stage14_mission.py --site <档案名>
# 或直接跑网页监控生成的 ~/fly.sh（1 分钟内有效）
```

**常用键盘**：
- 手动段：`WASD` 平移 / `RF` 升降 / `QE` 转向 / `空格` 悬停 / `1–5` 选红点候选 / **`T` 开始任务**
- 冲刺前：`M` 或方向键回手动段 / `G` 确认冲刺（降速时按 `Y`） / **`L` 降落** / **`X` 立即停桨**

---

## 二、摄像头标定（调焦 → 内参 → 测距验收 → 外参 → 同步）

> 严格按顺序操作：**调焦锁死后绝不再拧镜头**，否则内参全部作废。相机同时只能被一个进程打开，操作前先 `Ctrl+C` 停掉 `run_vision.py`。

### 1. 手动调焦距

```bash
# 飞机上启动全分辨率预览，用手机/电脑打开 http://<飞机IP>:8092/
python3 tools/rec_stereo.py --no-ros --name focus_test
```

对准目标工作距离（5–10 m 标定板/清晰文字，或 10 m+ 远处高对比边缘），分别微调左右镜头至最锐利并点胶/锁紧固定。

### 2. 双目内参标定（`tools/stereo_calib.py`，需 OpenCV ≥ 4.7）

1. **打印标定板**（9×6 ChArUco，`DICT_4X4_50`，28 mm 方格；100% 原尺寸打印后用钢尺核对 100 mm 刻度）：
   ```bash
   python tools/stereo_calib.py board --square 28.0 --out charuco_A4.pdf
   ```
2. **采集 + 标定**（推荐相机插笔记本跑实时窗口，或机上用 `rec_stereo.py` 采图后离线标定）：
   ```bash
   python tools/stereo_calib.py --cam 1 --square 28.0 --calib stereo_calib.yaml
   ```
   - 窗口按键：`s` 保存（绿字 `READY` 时存，覆盖九宫格各位置及 20°–40° 倾斜，拍 30–40 对） / `d` 删上一对 / `w` 换左右眼 / **`c` 开始标定** / `v` 切换校正绿线核对 / `q` 退出。
   - 对已保存图片重新标定：`python tools/stereo_calib.py calib --dir calib_imgs --square 28.0 --calib stereo_calib.yaml`。
   - 合格标准：`rms_stereo < 1.0 px`、`T[0] ≈ -60 mm`、校正行差均值 `< 0.5 px`、`image_width=1920`、`image_height=1080`、`swap_halves=0`、`flip_180=0`。

### 3. 双目测距验收（`tools/stereo_range.py`）

```bash
python tools/stereo_range.py --cam 1 --calib stereo_calib.yaml --marker-mm 160
```

在 2 m / 4 m 处放 ArUco 码（`DICT_4X4_50`），按 `r` 输入卷尺实测距离，确认误差在 1–5 cm 内（退出时自动输出 `ranging_log.csv` 及视差偏置建议）。

### 4. 相机-飞控外参标定（`pitch / roll / yaw` + 杠杆臂）

1. **杠杆臂（卷尺实测）**：左目相对飞控中心，FLU 系（前 `x` / 左 `y` / 上 `z`），当前值 `0.080 / 0.030 / -0.115` m。
2. **机上采外参数据**（先启 MAVROS + VINS，停掉 `run_vision.py`；A4 ArUco ID `30` 固定在墙上）：
   ```bash
   python3 tools/rec_stereo.py --name extcal --burst 3 --aruco 30 --marker-mm 160
   ```
   打开 `http://<飞机IP>:8092/`，手持飞机在 1.5–4 m 覆盖九宫格与多姿态，看到 `左✓ 右✓ 静止` 按回车拍 20–35 组（输出在 `~/rec/<时间>_extcal/`）。
3. **离线解算并写入**：用外参手眼脚本算出 `pitch / roll / yaw` 后，**同步更新以下两处**：
   - `run_vision.py` 顶部 `CAM = {...}`
   - `vision_bridge.py` 中 `cam_pitch_deg / cam_roll_deg / cam_yaw_deg / cam_x / cam_y / cam_z` 默认值

### 5. 双端同步与自检

1. 把新 `stereo_calib.yaml` **同时覆盖到飞机和地面站**。
2. 导出校正图核对行对齐：
   ```bash
   python3 tools/rec_read.py <rec目录> --export /tmp/rect_check --both --calib stereo_calib.yaml
   ```
3. 运行自检：
   ```bash
   python3 tools/calib_check.py           # 标定尺寸与标志位检查
   python3 tools/mode_isolation_check.py  # 模式分区隔离自检
   python3 tools/wiring_check.py          # 感知插件接线检查
   python3 tools/py38_check.py            # Python 3.8 语法检查
   ```

---

## 三、目录速查

| 路径 | 作用 |
| --- | --- |
| `stage14_mission.py` | 实飞主入口（档案 `mode = 冲刺 / 影子 / 停靠`，`target = 环 / 红点`） |
| `run_vision.py` / `vision_bridge.py` | 机上视觉桥：采图、位姿插值、世界系融合、`/gate/status` 与 `/gate/select` |
| `gcs_gate.py` / `gcs_view.py` | 地面站检测（`gate_gcs`）/ 机上检测观看录屏（`gate`） |
| `flight_monitor.py` / `vb09/web/` | 机载监控网页后端与前端（含红点候选列表与画面点选） |
| `gate_plan.py` / `flight_profile.py` | 冲刺规划（纯函数）/ 现场档案解析与安全校验 |
| `vb09/` | 核心包：`common/`、`perception/{ring,reddot}/`、`flight/`、`profile/`、`monitor/`、`web/` |
| `tools/` | 标定（`stereo_calib.py`、`stereo_range.py`、`rec_stereo.py`、`rec_read.py`、`calib_check.py`）与回归测试 |
| `stereo_calib.yaml` | 当前生效的双目标定文件（飞机与地面站需保持一致） |
