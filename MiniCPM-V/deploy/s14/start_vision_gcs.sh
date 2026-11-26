#!/bin/bash
# start_vision_gcs.sh —— 机上桥用【地面检测】模式：全分辨率 + 原样转发
#   _tx_max_w:=3840   并排宽度 3840 = 每目 1920x1080（默认 1920 → 每目只有 960）
#   _raw_max_kb:=900  允许原样转发相机自己的 MJPEG（默认 260 → 必重编码）
#   --tx-hz 10        限到 10Hz：原样转发在 15Hz 会把 WiFi 压到 31Mbit/s 丢 9%，
#                     限到 10Hz 后实测 7.8Hz / 往返 95ms / 丢包 0%
#   实测对比（2026-11-12，真机，桨已拆）：
#     重编码q85  9.4Hz 往返263ms 0%丢
#     原样 15Hz  14.1Hz 往返130ms 9%丢
#     原样 10Hz  7.8Hz  往返 95ms 0%丢   ← 采用（延迟最低、无丢包）
S14=/home/visbot/program/UAV/src/uav_test/scripts/detect_track/s14
GCS="${1:-192.168.2.179}"
TXW="${2:-3840}"
TXHZ="${3:-10}"
set +u
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/visbot/ros_ws/devel/setup.bash 2>/dev/null
pkill -f vision_bridge.py 2>/dev/null
pkill -f run_vision.py 2>/dev/null
command -v tmux >/dev/null 2>&1 && tmux kill-session -t vision 2>/dev/null
sleep 1
# 2026-11-25：新飞机上**没装 tmux**（command -v tmux 为空）→ 原来的 tmux 启动会静默失败，
#   桥根本没起来但脚本照样打印"已启动"。这里加 nohup 兜底。
if command -v tmux >/dev/null 2>&1; then
  tmux new -d -s vision -c "$S14" "python3 run_vision.py gate_gcs --target reddot --gcs $GCS --tx-hz $TXHZ _tx_max_w:=$TXW _raw_max_kb:=900 > /tmp/vision_gcs.log 2>&1"
else
  # 没有 tmux 时用 setsid 派生：**必须 setsid + 三重重定向**，否则 ssh 会一直等到子进程退出
  cd "$S14" || exit 1
  setsid nohup python3 run_vision.py gate_gcs --target reddot --gcs "$GCS" --tx-hz "$TXHZ" _tx_max_w:="$TXW" _raw_max_kb:=900 < /dev/null > /tmp/vision_gcs.log 2>&1 &
  disown 2>/dev/null || true
  cd "$HOME" || true
fi
sleep 8
echo "--- 地面站 $GCS  上行宽度 $TXW  上行帧率 $TXHZ Hz  原样转发 ---"
grep -E '生效参数|上行 bind|下行 connect' /tmp/vision_gcs.log | head -3 || true
if pgrep -f 'vision_bridge.py|run_vision.py' >/dev/null 2>&1; then
  echo "    (桥进程在跑)"
else
  echo "    ★ 桥没起来！看 /tmp/vision_gcs.log"
fi