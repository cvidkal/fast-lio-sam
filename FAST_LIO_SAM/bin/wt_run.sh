#!/usr/bin/env bash
# wt_run.sh — 在「当前这棵 worktree」的 install 环境里跑 ros2, 自动隔离 DDS 域。
#
# 为什么需要它: 多个 worktree 同时 live 跑会抢同名 topic (/Odometry /rslidar_points …),
# fastlio 触发 "lidar loop back" 雪崩。给每个分支一个唯一 ROS_DOMAIN_ID 就互不可见。
# 域 id 由分支名 hash 出 (1..101 稳定可复现, 永不为 0 → 不撞默认域 0 的 main)。
#
# 用法:
#   bin/wt_run.sh launch fast_lio_sam mapping_bag.launch.py bag:=... config_file:=...
#   bin/wt_run.sh run   fast_lio_sam fastlio_mapping
#   ROS_DOMAIN_ID=42 bin/wt_run.sh launch ...     # 手动指定域则不再 hash
#
# 在「别的终端」对同一 worktree 操作 (echo / hz / service call) 时, 先 export 脚本
# 打印的那行 ROS_DOMAIN_ID, 否则收不到消息 (跟生产陷阱 #3 同类坑)。
set -eo pipefail

PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FLS_UNDERLAY="${FLS_UNDERLAY:-$HOME/fls_ws/install/setup.bash}"

BRANCH="$(git -C "$PKG_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo default)"
# 已显式给 ROS_DOMAIN_ID 就尊重它; 否则按分支名 hash 到 1..101。
if [ -z "${ROS_DOMAIN_ID:-}" ]; then
  ROS_DOMAIN_ID=$(( ($(printf '%s' "$BRANCH" | cksum | cut -d' ' -f1) % 101) + 1 ))
fi
export ROS_DOMAIN_ID

INSTALL_SETUP="$PKG_ROOT/install/setup.bash"
if [ ! -f "$INSTALL_SETUP" ]; then
  echo "[wt_run] !! 没找到 $INSTALL_SETUP — 先跑 bin/wt_build.sh" >&2
  exit 1
fi

echo "[wt_run] 分支          : $BRANCH"
echo "[wt_run] ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "[wt_run] 其它终端先跑   : export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[wt_run] ----------------------------------------"

# shellcheck disable=SC1090
source "$ROS_SETUP"
[ -f "$FLS_UNDERLAY" ] && source "$FLS_UNDERLAY"  # shellcheck disable=SC1090
# shellcheck disable=SC1090
source "$INSTALL_SETUP"

exec ros2 "$@"
