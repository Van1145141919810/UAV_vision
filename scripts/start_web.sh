#!/usr/bin/env bash
# scripts/start_web.sh —— 在 tmux session "uav" 的 "web" 窗口启动网页监控后端 (flight_monitor.py)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "$SCRIPT_DIR/_common.sh"

usage() {
    echo "用法: $0 [--dir <项目路径>] [-y] [透传参数...]"
    echo "  默认项目路径: ${DEFAULT_PROJ_DIR}"
}

if ! uav_parse_args "$@"; then
    usage
    exit 0
fi

if uav_window_exists "web"; then
    echo "[uav] 窗口 'web' 已存在（session: ${UAV_SESSION}），不做任何操作。"
    exit 0
fi

uav_prompt_var PROJ_DIR "项目路径" "$DEFAULT_PROJ_DIR"
PROJ_DIR="$(uav_expand_path "$PROJ_DIR")"

if [ ! -f "${PROJ_DIR}/flight_monitor.py" ]; then
    echo "[uav] 警告: 未找到 ${PROJ_DIR}/flight_monitor.py，请确认项目路径是否正确！" >&2
fi

EXTRA=""
if [ ${#UAV_EXTRA_ARGS[@]} -gt 0 ]; then
    printf -v EXTRA ' %q' "${UAV_EXTRA_ARGS[@]}"
fi

ROS_ENV="$(uav_ros_env_snippet)"
CMD="${ROS_ENV}; cd $(printf '%q' "$PROJ_DIR") && python3 flight_monitor.py${EXTRA}"

uav_launch_window "web" "$PROJ_DIR" "$CMD" || true
