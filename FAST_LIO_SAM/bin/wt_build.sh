#!/usr/bin/env bash
# wt_build.sh — 在「当前这棵 worktree」里自包含地编译 fast_lio_sam。
#
# 为什么需要它: git worktree 给每个分支独立源码树, 但 colcon 默认要
# `ws/src/<包>` 布局 + ws 根放 build/install/log。直接 `colcon build` 会编到
# 那个写死软链指向的 ws (通常是 main), 不是你当前改的分支。本脚本把产物固定
# 落在「脚本所属的那棵树」的 FAST_LIO_SAM/{build,install,log} (被 .gitignore 吃掉),
# 从而每个 worktree 各编各的, 互不污染。
#
# 用法:
#   bin/wt_build.sh                 # Release 编当前 worktree
#   BUILD_TYPE=Debug bin/wt_build.sh
#   bin/wt_build.sh --symlink-install   # 透传额外 colcon 参数
#
# 依赖来源: fast_lio_sam 唯一的非 apt 工作区依赖是 livox_ros_driver2 (msg stub)。
# 脚本 source 一个已编好的 underlay 拿它; 用 FLS_UNDERLAY 覆盖路径。
set -eo pipefail

# 包根 = 本脚本所在目录的上一级 (bin/.. = FAST_LIO_SAM)。
# 用 BASH_SOURCE 而非 pwd, 这样不管在哪调都锁定「脚本自己那棵 worktree」。
PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FLS_UNDERLAY="${FLS_UNDERLAY:-$HOME/fls_ws/install/setup.bash}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

echo "[wt_build] 包根       : $PKG_ROOT"
echo "[wt_build] 分支       : $(git -C "$PKG_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')"
echo "[wt_build] BUILD_TYPE : $BUILD_TYPE"

# 注意: ROS setup 脚本引用未定义变量, 不能开 set -u。
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ -f "$FLS_UNDERLAY" ]; then
  echo "[wt_build] underlay   : $FLS_UNDERLAY (拿 livox_ros_driver2 等依赖)"
  # shellcheck disable=SC1090
  source "$FLS_UNDERLAY"
else
  echo "[wt_build] !! 找不到 underlay $FLS_UNDERLAY"
  echo "[wt_build]    若 livox_ros_driver2 不在别处, 编译会失败。设 FLS_UNDERLAY 指向有它的 install/setup.bash。"
fi

cd "$PKG_ROOT"
# 从包根跑 colcon: 产物落 ./build ./install ./log, 被 FAST_LIO_SAM/.gitignore 吃掉。
# legacy/ 有 COLCON_IGNORE, 不会撞同名 catkin 包 fast_lio_sam。
set -x
colcon build --packages-select fast_lio_sam \
  --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE" "$@"
{ set +x; } 2>/dev/null

echo
echo "[wt_build] 完成。运行用:  bin/wt_run.sh launch fast_lio_sam <launch> ..."
echo "[wt_build] 或手动 source: source \"$PKG_ROOT/install/setup.bash\""
