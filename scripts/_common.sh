#!/usr/bin/env bash
# scripts/_common.sh —— VB09 机载后台服务 tmux 启动公共库
# 所有子脚本和 start_all.sh 共用此文件（session 名固定为 uav）。

UAV_SESSION="uav"

DEFAULT_GCS_IP="192.168.2.67"
DEFAULT_PROJ_DIR="$HOME/program/UAV/src/uav_test/scripts/detect_track/UAV_vision"
DEFAULT_VISION_MODE="gate_gcs"
DEFAULT_TARGET_KIND="reddot"
DEFAULT_FCU_URL="/dev/ttyS7:921600"

# 展开路径开头的 ~
uav_expand_path() {
    local p="$1"
    case "$p" in
        "~")   echo "$HOME" ;;
        "~/"*) echo "$HOME/${p#~/}" ;;
        *)     echo "$p" ;;
    esac
}

# 按正确 overlay 顺序加载 ROS 环境（不依赖 ~/.bashrc）
uav_source_ros() {
    local f
    for f in \
        "/opt/ros/noetic/setup.bash" \
        "/opt/appNormalfs/ros_ws/devel/setup.bash" \
        "$HOME/ros_ws/devel/setup.bash" \
        "$HOME/program/UAV/devel/setup.bash"
    do
        if [ -f "$f" ]; then
            # shellcheck disable=SC1090
            source "$f"
        fi
    done
}

# 生成窗口内执行用的 ROS 加载命令串
uav_ros_env_snippet() {
    cat <<'EOF'
for _f in /opt/ros/noetic/setup.bash /opt/appNormalfs/ros_ws/devel/setup.bash "$HOME/ros_ws/devel/setup.bash" "$HOME/program/UAV/devel/setup.bash"; do [ -f "$_f" ] && source "$_f"; done
EOF
}

# 交互询问参数（已通过命令行或环境变量提供，或 NONINTERACTIVE=1 时不提问）
# 用法: uav_prompt_var <变量名> <提示文案> <默认值>
uav_prompt_var() {
    local var_name="$1"
    local prompt="$2"
    local def_val="$3"
    local cur_val="${!var_name:-}"

    if [ -n "$cur_val" ]; then
        return 0
    fi
    if [ "${UAV_NONINTERACTIVE:-0}" = "1" ] || [ ! -t 0 ]; then
        printf -v "$var_name" '%s' "$def_val"
        return 0
    fi
    local input=""
    read -r -p "${prompt} [默认: ${def_val}]: " input
    if [ -z "$input" ]; then
        printf -v "$var_name" '%s' "$def_val"
    else
        printf -v "$var_name" '%s' "$input"
    fi
}

# 解析公共命令行参数（供各脚本复用）
# 支持: --gcs <IP>  --dir <项目路径>  --mode <gate_gcs|gate|aruco>  --target <reddot|ring>  -y/--yes
uav_parse_args() {
    UAV_EXTRA_ARGS=()
    while [ $# -gt 0 ]; do
        case "$1" in
            --gcs)
                GCS_IP="${2:?--gcs 需要 IP}"
                shift 2
                ;;
            --dir)
                PROJ_DIR="${2:?--dir 需要路径}"
                shift 2
                ;;
            --mode)
                VISION_MODE="${2:?--mode 需要模式}"
                shift 2
                ;;
            --target)
                TARGET_KIND="${2:?--target 需要目标类型}"
                shift 2
                ;;
            --fcu)
                FCU_URL="${2:?--fcu 需要飞控串口}"
                shift 2
                ;;
            -y|--yes|--default)
                UAV_NONINTERACTIVE=1
                shift
                ;;
            -h|--help)
                return 2
                ;;
            *)
                UAV_EXTRA_ARGS+=("$1")
                shift
                ;;
        esac
    done
    return 0
}

# 检查 tmux 是否安装
uav_require_tmux() {
    if ! command -v tmux >/dev/null 2>&1; then
        echo "[uav] 错误: 未安装 tmux，请先执行 sudo apt-get install -y tmux" >&2
        exit 1
    fi
}

# 判断 session 中是否已有同名窗口
uav_window_exists() {
    local win="$1"
    tmux has-session -t "$UAV_SESSION" 2>/dev/null || return 1
    tmux list-windows -t "$UAV_SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$win"
}

# 在 session=uav 中创建窗口并发送命令：
#   - 窗口已存在 → 打印提示，什么都不做，返回 1
#   - 窗口不存在 → 新建常驻交互式 bash 窗口并 send-keys 执行命令，返回 0
#   （进程退出或按 Ctrl+C 后仍停留在交互式 bash，按 ↑ + 回车即可手动重启）
uav_launch_window() {
    local win="$1"
    local workdir="$2"
    local cmd="$3"

    uav_require_tmux

    if uav_window_exists "$win"; then
        echo "[uav] 窗口 '${win}' 已存在（session: ${UAV_SESSION}），不做任何操作。"
        return 1
    fi

    if [ ! -d "$workdir" ]; then
        workdir="$HOME"
    fi

    if tmux has-session -t "$UAV_SESSION" 2>/dev/null; then
        tmux new-window -d -t "$UAV_SESSION" -n "$win" -c "$workdir" bash
    else
        tmux new-session -d -s "$UAV_SESSION" -n "$win" -c "$workdir" bash
    fi

    # 窗口退出后仍可手动操作；同时通过交互式 bash 启动命令，保留命令历史便于 ↑ 回车重启
    tmux send-keys -t "${UAV_SESSION}:${win}" "$cmd" C-m
    echo "[uav] 已启动窗口 '${win}'（session: ${UAV_SESSION}）"
    return 0
}

# 等待 ROS master 就绪（最多 timeout 秒）
uav_wait_ros_master() {
    local timeout="${1:-15}"
    uav_source_ros
    if ! command -v rostopic >/dev/null 2>&1; then
        echo "[uav] 提示: 当前环境未找到 rostopic，跳过 ROS master 检测"
        return 0
    fi
    local i=0
    echo -n "[uav] 等待 ROS master 就绪"
    while [ "$i" -lt "$timeout" ]; do
        if rostopic list >/dev/null 2>&1; then
            echo " OK"
            return 0
        fi
        echo -n "."
        sleep 1
        i=$((i + 1))
    done
    echo " 超时 (${timeout}s)，继续执行"
    return 1
}
