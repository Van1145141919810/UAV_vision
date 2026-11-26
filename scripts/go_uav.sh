#!/usr/bin/env bash
# scripts/go_uav.sh —— 切换当前终端到项目目录（必须用 source 执行）
# 用法:
#   source scripts/go_uav.sh                  # 进入默认目录 ~/program/UAV/src/uav_test/scripts/detect_track/UAV_vision
#   source scripts/go_uav.sh <自定义路径>     # 进入指定目录
# 也可单独拷到 ~/scripts/go_uav.sh 使用，或在 ~/.bashrc 加别名:
#   alias gouav='source ~/program/UAV/src/uav_test/scripts/detect_track/UAV_vision/scripts/go_uav.sh'

_uav_default_dir="$HOME/program/UAV/src/uav_test/scripts/detect_track/UAV_vision"

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "[go_uav] 注意: 直接运行脚本无法改变当前终端目录，请改用 source："
    echo "    source ${BASH_SOURCE[0]} [项目路径]"
    echo "  默认路径: ${_uav_default_dir}"
    exit 1
fi

_uav_target="${1:-${PROJ_DIR:-$_uav_default_dir}}"
case "$_uav_target" in
    "~")   _uav_target="$HOME" ;;
    "~/"*) _uav_target="$HOME/${_uav_target#~/}" ;;
esac

if [ ! -d "$_uav_target" ]; then
    echo "[go_uav] 目录不存在: $_uav_target" >&2
    unset _uav_default_dir _uav_target
    return 1
fi

cd "$_uav_target" || { unset _uav_default_dir _uav_target; return 1; }
echo "[go_uav] 已切换到: $(pwd)"
unset _uav_default_dir _uav_target
