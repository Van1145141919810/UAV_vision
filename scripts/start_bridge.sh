#!/usr/bin/env bash
# scripts/start_bridge.sh —— 在 tmux session "uav" 的 "bridge" 窗口启动视觉图传桥 (run_vision.py)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "$SCRIPT_DIR/_common.sh"

usage() {
    echo "用法: $0 [--gcs <上位机IP>] [--dir <项目路径>] [--mode <gate_gcs|gate|aruco>] [--target <reddot|ring>] [-y] [透传参数...]"
    echo "  默认上位机 IP: ${DEFAULT_GCS_IP}"
    echo "  默认项目路径:  ${DEFAULT_PROJ_DIR}"
    echo "  默认模式/目标: ${DEFAULT_VISION_MODE} / ${DEFAULT_TARGET_KIND}"
}

if ! uav_parse_args "$@"; then
    usage
    exit 0
fi

if uav_window_exists "bridge"; then
    echo "[uav] 窗口 'bridge' 已存在（session: ${UAV_SESSION}），不做任何操作。"
    exit 0
fi

uav_prompt_var GCS_IP "上位机 IP" "$DEFAULT_GCS_IP"
uav_prompt_var PROJ_DIR "项目路径" "$DEFAULT_PROJ_DIR"
uav_prompt_var VISION_MODE "视觉模式 (gate_gcs / gate / aruco)" "$DEFAULT_VISION_MODE"
uav_prompt_var TARGET_KIND "目标类型 (reddot / ring)" "$DEFAULT_TARGET_KIND"
PROJ_DIR="$(uav_expand_path "$PROJ_DIR")"

if [ ! -f "${PROJ_DIR}/run_vision.py" ]; then
    echo "[uav] 警告: 未找到 ${PROJ_DIR}/run_vision.py，请确认项目路径是否正确！" >&2
fi

EXTRA=""
if [ ${#UAV_EXTRA_ARGS[@]} -gt 0 ]; then
    printf -v EXTRA ' %q' "${UAV_EXTRA_ARGS[@]}"
fi

ROS_ENV="$(uav_ros_env_snippet)"
RUN_CMD="python3 run_vision.py $(printf '%q' "$VISION_MODE") --gcs $(printf '%q' "$GCS_IP")"
if [ "$VISION_MODE" = "gate_gcs" ] || [ "$VISION_MODE" = "gate" ]; then
    RUN_CMD+=" --target $(printf '%q' "$TARGET_KIND")"
fi
RUN_CMD+="${EXTRA}"

CMD="${ROS_ENV}; cd $(printf '%q' "$PROJ_DIR") && ${RUN_CMD}"

uav_launch_window "bridge" "$PROJ_DIR" "$CMD" || true
