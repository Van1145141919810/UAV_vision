#!/bin/bash
# start_vision.sh —— 用 tmux 托管机上视觉（SSH 断开也不会死）
S14=/home/visbot/program/UAV/src/uav_test/scripts/detect_track/s14
set +u
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/visbot/ros_ws/devel/setup.bash 2>/dev/null
DRIVE="${1:-drive}"
EXTRA=""
[ "$DRIVE" = "drive" ] && EXTRA="--drive"
pkill -f vision_bridge.py 2>/dev/null
tmux kill-session -t vision 2>/dev/null
sleep 1
tmux new -d -s vision -c "$S14" \
  "python3 run_vision.py gate --target reddot $EXTRA > /tmp/vision.log 2>&1"
sleep 7
echo "  --- tmux 会话 ---"
tmux ls 2>/dev/null
echo "  --- 关键日志 ---"
grep -E '参数 |标定 |相机' /tmp/vision.log | head -5
echo "  --- 进程 ---"
pgrep -af vision_bridge.py | head -1