#!/bin/bash
# start_monitor.sh —— 用 tmux 托管监控网页（只绑 127.0.0.1:8080）
S14=/home/visbot/program/UAV/src/uav_test/scripts/detect_track/s14
set +u
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/visbot/ros_ws/devel/setup.bash 2>/dev/null
pkill -f flight_monitor.py 2>/dev/null
tmux kill-session -t monitor 2>/dev/null
sleep 1
tmux new -d -s monitor -c "$S14" "python3 flight_monitor.py > /tmp/monitor.log 2>&1"
sleep 6
if pgrep -f flight_monitor.py >/dev/null; then
  echo "  OK 监控已启动 PID $(pgrep -f flight_monitor.py | head -1)  tmux: monitor"
  echo "     本机 127.0.0.1:8080  ->  笔记本 http://127.0.0.1:8080 （经 SSH 隧道）"
else
  echo "  FAIL:"; head -8 /tmp/monitor.log
fi