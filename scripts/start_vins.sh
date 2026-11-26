#!/usr/bin/env bash
# scripts/start_vins.sh —— 在 tmux session "uav" 的 "vins" 窗口启动 VINS_Fusion (stereo120.launch)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "$SCRIPT_DIR/_common.sh"

usage() {
    echo "用法: $0 [-y]"
}

if ! uav_parse_args "$@"; then
    usage
    exit 0
fi

if uav_window_exists "vins"; then
    echo "[uav] 窗口 'vins' 已存在（session: ${UAV_SESSION}），不做任何操作。"
    exit 0
fi

ROS_ENV="$(uav_ros_env_snippet)"
CMD="${ROS_ENV}; roslaunch vins stereo120.launch"

uav_launch_window "vins" "$HOME" "$CMD" || true
