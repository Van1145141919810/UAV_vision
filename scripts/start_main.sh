#!/usr/bin/env bash
# scripts/start_main.sh —— 在 tmux session "uav" 的 "main" 主窗口清理 /geometric_controller 并保留交互终端
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "$SCRIPT_DIR/_common.sh"

usage() {
    echo "用法: $0 [--dir <项目路径>] [-y]"
    echo "  默认项目路径: ${DEFAULT_PROJ_DIR}"
}

if ! uav_parse_args "$@"; then
    usage
    exit 0
fi

if uav_window_exists "main"; then
    echo "[uav] 窗口 'main' 已存在（session: ${UAV_SESSION}），不做任何操作。"
    exit 0
fi

uav_prompt_var PROJ_DIR "项目路径" "$DEFAULT_PROJ_DIR"
PROJ_DIR="$(uav_expand_path "$PROJ_DIR")"

ROS_ENV="$(uav_ros_env_snippet)"
KILL_CMD='if rosnode list 2>/dev/null | grep -Fxq "/geometric_controller"; then rosnode kill /geometric_controller && echo "[main] 已清理 /geometric_controller"; else echo "[main] 未发现 /geometric_controller（无需清理）"; fi'
CMD="${ROS_ENV}; cd $(printf '%q' "$PROJ_DIR") 2>/dev/null || true; ${KILL_CMD}"

uav_launch_window "main" "$PROJ_DIR" "$CMD" || true
