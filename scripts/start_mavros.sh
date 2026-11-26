#!/usr/bin/env bash
# scripts/start_mavros.sh —— 在 tmux session "uav" 的 "mavros" 窗口启动 MAVROS
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "$SCRIPT_DIR/_common.sh"

usage() {
    echo "用法: $0 [--gcs <上位机IP>] [--fcu <串口>] [-y]"
    echo "  默认上位机 IP: ${DEFAULT_GCS_IP}，串口: ${DEFAULT_FCU_URL}"
}

if ! uav_parse_args "$@"; then
    usage
    exit 0
fi

if uav_window_exists "mavros"; then
    echo "[uav] 窗口 'mavros' 已存在（session: ${UAV_SESSION}），不做任何操作。"
    exit 0
fi

uav_prompt_var GCS_IP "上位机 IP (QGC / 地面站)" "$DEFAULT_GCS_IP"
FCU_URL="${FCU_URL:-$DEFAULT_FCU_URL}"

ROS_ENV="$(uav_ros_env_snippet)"
CMD="${ROS_ENV}; roslaunch mavros px4_visquad.launch fcu_url:=${FCU_URL} gcs_url:=udp://@${GCS_IP}:14550"

uav_launch_window "mavros" "$HOME" "$CMD" || true
