#!/usr/bin/env bash
# scripts/start_all.sh —— 一键拉起 uav tmux 会话的全部机载常驻服务窗口
# 窗口: main (清理 /geometric_controller + 主指令台) / mavros / vins / bridge / web
# 已存在的窗口一律跳过不做任何操作；各窗口常驻交互 bash，进程退出后不关闭。
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "$SCRIPT_DIR/_common.sh"

usage() {
    cat <<EOF
用法: $0 [选项]
  --gcs <IP>          上位机 IP（默认: ${DEFAULT_GCS_IP}）
  --dir <路径>        项目路径（默认: ${DEFAULT_PROJ_DIR}）
  --mode <模式>       视觉模式 gate_gcs / gate / aruco（默认: ${DEFAULT_VISION_MODE}）
  --target <目标>     目标类型 reddot / ring（默认: ${DEFAULT_TARGET_KIND}）
  --fcu <串口>        飞控串口（默认: ${DEFAULT_FCU_URL}）
  -y, --yes           跳过交互询问，未指定项全部使用默认值
  -h, --help          显示此帮助
EOF
}

if ! uav_parse_args "$@"; then
    usage
    exit 0
fi

uav_require_tmux

echo "================================================================"
echo "  VB09 机载服务一键启动 (tmux session: ${UAV_SESSION})"
echo "  直接回车采用括号内默认值"
echo "================================================================"
uav_prompt_var GCS_IP      "上位机 IP"                        "$DEFAULT_GCS_IP"
uav_prompt_var PROJ_DIR    "项目路径"                         "$DEFAULT_PROJ_DIR"
uav_prompt_var VISION_MODE "视觉模式 (gate_gcs / gate / aruco)" "$DEFAULT_VISION_MODE"
uav_prompt_var TARGET_KIND "目标类型 (reddot / ring)"         "$DEFAULT_TARGET_KIND"
PROJ_DIR="$(uav_expand_path "$PROJ_DIR")"
FCU_URL="${FCU_URL:-$DEFAULT_FCU_URL}"

export GCS_IP PROJ_DIR VISION_MODE TARGET_KIND FCU_URL
export UAV_NONINTERACTIVE=1

echo "----------------------------------------------------------------"
echo "  上位机 IP: ${GCS_IP}"
echo "  项目路径:  ${PROJ_DIR}"
echo "  视觉链路:  ${VISION_MODE}  (target=${TARGET_KIND})"
echo "----------------------------------------------------------------"

# 1. 先起 mavros（拉起 roscore）
"$SCRIPT_DIR/start_mavros.sh"

# 2. 等待 ROS master 在线后再起 vins / main / bridge / web
uav_wait_ros_master 15 || true

# 3. 启动 vins
"$SCRIPT_DIR/start_vins.sh"

# 4. 启动 main 主窗口（清理 /geometric_controller 并停留为操作终端）
"$SCRIPT_DIR/start_main.sh"

# 5. 启动图传桥与网页监控
"$SCRIPT_DIR/start_bridge.sh"
"$SCRIPT_DIR/start_web.sh"

# 默认聚焦到 main 窗口
if uav_window_exists "main"; then
    tmux select-window -t "${UAV_SESSION}:main" 2>/dev/null || true
fi

echo "================================================================"
echo "  全部窗口检查/启动完成。当前窗口列表:"
tmux list-windows -t "$UAV_SESSION" -F '    #I: #W' 2>/dev/null || true
echo ""
echo "  进入会话:   tmux attach -t ${UAV_SESSION}   (Ctrl+b w 切换窗口)"
echo "  重启服务:   进对应窗口按 Ctrl+C 后按 ↑ 回车即可重跑"
echo "  网页转发:   ssh -N -o ExitOnForwardFailure=yes -L 8080:127.0.0.1:8080 visbot@192.168.2.20"
if [ "$VISION_MODE" = "gate_gcs" ]; then
    echo "  地面检测:   python gcs_gate.py --drone 192.168.2.20 --target ${TARGET_KIND} --mode 1"
fi
echo "  ★ 实飞请在【新开的普通 SSH 终端】运行 stage14_mission.py，不要在 tmux 里实飞！"
echo "================================================================"
