#!/usr/bin/env bash
# map_bag.sh — 在机器人(airy-dog)上对一个 rosbag「确定性离线建图 + 生成 2D occupancy」。
#
# 只需给 bag 目录即可。封装了本仓库验证过的最优流程 (见 issue #49 / #50, memory):
#   1) 离线 data_mode=1 直读 bag (lock-step 喂帧, 不掉帧, 可复现) —— 不是 ros2 bag play
#   2) 回环 ICP fitness 0.1 (室外大场景甜点; 0.02 太严会拒真回环, 0.3 太松)
#   3) 2D 走"每格相对当地局部地面的高度带"切片 + raycast
#      (自适应斜坡/起伏地形, 不依赖机器人是否走近该处; 取代旧的"相对最近轨迹点"切法)
#
# 用法:
#   bin/map_bag.sh <bag_dir> [选项]
#     --fitness F     回环 ICP fitness 阈值 (默认 0.1)
#     --out DIR       输出目录 (默认 ~/dog_maps/<bag名>; 必须在 $HOME 下)
#     --scan-line N   Airy 线数 (默认沿用 config 的 96; 192 模式传 192)
#     --rel-min M     2D 障碍带下界 = 当地地面上方多少 m 起算 (默认 0.3, 避开地面层)
#     --rel-max M     2D 障碍带上界 = 当地地面上方多少 m 止 (默认 2.5)
#     --res R         2D 分辨率 (默认 0.05 m/px)
#     --no-2d         只建 3D, 跳过 2D
#     --deploy        生成后部署到 /opt/dog/map/<bag名> 并重指 current_map
#
# 例:
#   bin/map_bag.sh /home/nvidia/mou/dog/bags/airy_20260604_202204 --deploy
#   bin/map_bag.sh ~/bags/room1 --rel-min 0.2 --rel-max 2.0   # 收紧障碍带
#
# 前提: fast_lio_sam 已编译且离线模式可用。若 [1/3] 报 "data_mode=1 ... disabled",
#       说明 install 二进制旧了, 先重编:
#         cd ~/ws/fls && source /opt/ros/humble/setup.bash && \
#           colcon build --packages-select fast_lio_sam --cmake-args -DCMAKE_BUILD_TYPE=Release
#
# 可用环境变量覆盖路径: ROS_SETUP, WS_SETUP, PKG_SRC, ROS_DOMAIN_ID
set -uo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
# 写死指向「含离线模式的已知好 build」(cgj 的 ~/ws/fls, 2026-06-04 重编)。
# 注意: nvidia 的 fast_lio_sam_ws 是旧 build, 离线模式被 FATAL 禁用, 不能用。
# /home/cgj 权限 750 → 本脚本须由 cgj 运行。可用 WS_SETUP/PKG_SRC 环境变量覆盖。
WS_SETUP="${WS_SETUP:-/home/cgj/ws/fls/install/setup.bash}"
PKG_SRC="${PKG_SRC:-/home/cgj/ws/fls/src/FAST_LIO_SAM}"
DOMAIN="${ROS_DOMAIN_ID:-42}"

FITNESS=0.1; OUT=""; SCAN_LINE=""; REL_MIN=0.3; REL_MAX=2.5; RES=0.05; DO_2D=1; DO_DEPLOY=0

usage() { sed -n '2,30p' "$0"; exit 1; }
[ $# -ge 1 ] || usage
BAG="$1"; shift
while [ $# -gt 0 ]; do
  case "$1" in
    --fitness)   FITNESS="$2"; shift 2;;
    --out)       OUT="$2"; shift 2;;
    --scan-line) SCAN_LINE="$2"; shift 2;;
    --rel-min)   REL_MIN="$2"; shift 2;;
    --rel-max)   REL_MAX="$2"; shift 2;;
    --res)       RES="$2"; shift 2;;
    --no-2d)     DO_2D=0; shift;;
    --deploy)    DO_DEPLOY=1; shift;;
    -h|--help)   usage;;
    *) echo "未知选项: $1"; usage;;
  esac
done

# --- 校验 bag ---
BAG="$(cd "$BAG" 2>/dev/null && pwd)" || { echo "!! 找不到 bag 目录"; exit 1; }
[ -f "$BAG/metadata.yaml" ] || { echo "!! $BAG 里没有 metadata.yaml, 不像 rosbag2 目录"; exit 1; }
STAMP="$(basename "$BAG")"
[ -n "$OUT" ] || OUT="$HOME/dog_maps/$STAMP"
case "$OUT" in "$HOME"/*) ;; *) echo "!! --out 必须在 \$HOME 下 (saveMap 写 \$HOME+savePCDDirectory)"; exit 1;; esac
mkdir -p "$OUT"
SUBDIR="/${OUT#"$HOME"/}/"   # savePCDDirectory: 相对 $HOME, 头尾带 /

echo "=========================================="
echo " bag     : $BAG"
echo " out     : $OUT"
echo " fitness : $FITNESS   2D带:[$REL_MIN,$REL_MAX]  res:$RES  2d:$DO_2D  deploy:$DO_DEPLOY"
echo "=========================================="

# --- source ROS (含未定义变量, 临时关 -u) ---
set +u; source "$ROS_SETUP"; source "$WS_SETUP"; set -u
export ROS_DOMAIN_ID="$DOMAIN"

# --- 生成离线 config (基于狗预设) ---
BASECFG="$(ros2 pkg prefix fast_lio_sam)/share/fast_lio_sam/config/airy_test_no_extr_est.yaml"
CFG="$OUT/offline_config.yaml"
sed -e "s|^\( *data_mode:\).*|\1 1|" \
    -e "s|^\( *bag_path:\).*|\1 \"$BAG\"|" \
    -e "s|^\( *historyKeyframeFitnessScore:\).*|\1 $FITNESS|" \
    -e "s|^\( *savePCDDirectory:\).*|\1 \"$SUBDIR\"|" \
    "$BASECFG" > "$CFG"
[ -n "$SCAN_LINE" ] && sed -i "s|^\( *scan_line:\).*|\1 $SCAN_LINE|" "$CFG"

# --- [1/3] 离线建图 (阻塞到 bag 读完自动 saveMap; 退出码 -11 是已知无害 segfault) ---
echo "[1/3] 离线确定性建图 (data_mode=1) ..."
ros2 run fast_lio_sam fastlio_mapping --ros-args --params-file "$CFG" > "$OUT/map_run.log" 2>&1 || true
if grep -q "is disabled in the ROS2 port" "$OUT/map_run.log"; then
  echo "!! 离线模式被禁 —— install 二进制旧了。先重编:"
  echo "   cd ~/ws/fls && source /opt/ros/humble/setup.bash && colcon build --packages-select fast_lio_sam --cmake-args -DCMAKE_BUILD_TYPE=Release"
  exit 2
fi
[ -f "$OUT/GlobalMap.pcd" ] || { echo "!! 没生成 GlobalMap.pcd, 末尾日志:"; tail -20 "$OUT/map_run.log"; exit 3; }
LC=$(grep -c "LOOP cur=" "$OUT/map_run.log" 2>/dev/null)
FR=$(grep -oE "offline bag finished: lidar=[0-9]+ imu=[0-9]+" "$OUT/map_run.log" | tail -1)
echo "    OK: GlobalMap.pcd $(du -h "$OUT/GlobalMap.pcd" | cut -f1) | 接受回环 ${LC:-0} | ${FR:-?}"

# --- [2/3] 2D occupancy (局部地面自适应带 + raycast) ---
if [ "$DO_2D" = 1 ]; then
  echo "[2/3] 生成 2D occupancy (局部地面自适应带 + raycast) ..."
  GB="$OUT/groundband.pcd"
  python3 - "$OUT/GlobalMap.pcd" "$GB" "$REL_MIN" "$REL_MAX" "$PKG_SRC" <<'PY'
import sys
gm, out, hmin, hmax, pkg = sys.argv[1:6]
sys.path.insert(0, pkg + "/tools/pcd_to_occgrid")
import numpy as np
from pcd_to_occgrid import load_pcd_xyz
g = load_pcd_xyz(gm)
# 自适应局部地面切: 每个 0.2m xy 网格格用格内 z 低分位估当地地面,
# 保留"地面上方 [hmin,hmax] m"的点为障碍。比旧的"相对最近轨迹点"更适配斜坡/起伏地形,
# 且不依赖机器人是否走近该处 —— 解决斜坡场景 + 机器人没走近区域漏墙 (见 issue #61)。
cs = 0.20
ci = np.floor(g[:, 0] / cs).astype(np.int64)
cj = np.floor(g[:, 1] / cs).astype(np.int64)
key = ci * 4000037 + cj                         # xy 格哈希 (cj 跨度 << 4000037, 不碰撞)
order = np.argsort(key, kind="stable")
ks = key[order]; zo = g[order, 2]
bnd = np.r_[0, np.where(np.diff(ks))[0] + 1, len(ks)]
gh = np.empty(len(g))
for a, b in zip(bnd[:-1], bnd[1:]):
    seg = zo[a:b]
    gnd = np.percentile(seg, 8) if b - a >= 8 else seg.min()   # 低分位估地面, 抗个别低飞点
    gh[order[a:b]] = seg - gnd
keep = (gh >= float(hmin)) & (gh <= float(hmax))
gb = g[keep]
with open(out, "w") as f:
    f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
    f.write("WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" % (len(gb), len(gb)))
    np.savetxt(f, gb, fmt="%.4f")
print("    局部地面带保留 %d/%d 点 (%.0f%%)" % (len(gb), len(g), 100.0 * len(gb) / max(len(g), 1)))
PY
  python3 "$PKG_SRC/tools/pcd_to_occgrid/pcd_to_occgrid.py" "$GB" -o "$OUT/occgrid" \
      --resolution "$RES" --z-min -100 --z-max 100 --raycast "$OUT/trajectory.pcd" \
      2>&1 | grep -E "尺寸|raycast 完成|写出" || true
fi

# --- [3/3] 部署 ---
if [ "$DO_DEPLOY" = 1 ]; then
  [ "$DO_2D" = 1 ] || { echo "!! --deploy 需要 2D, 别同时给 --no-2d"; exit 4; }
  echo "[3/3] 部署到 /opt/dog/map/$STAMP ..."
  DST="/opt/dog/map/$STAMP"; mkdir -p "$DST"
  cp -f "$OUT/occgrid.pgm" "$DST/map.pgm"
  cp -f "$OUT/GlobalMap.pcd" "$DST/GlobalMap.pcd"
  python3 -c "from PIL import Image; Image.open('$OUT/occgrid.pgm').save('$DST/map.png')"
  sed 's#^image:.*#image: map.png#' "$OUT/occgrid.yaml" > "$DST/map.yaml"
  ( cd "$DST" && rm -f GlobalMap.pcd.zip && zip -q GlobalMap.pcd.zip GlobalMap.pcd )
  ln -sfn "$DST" /opt/dog/map/current_map
  echo "    current_map -> $(readlink -f /opt/dog/map/current_map)"
fi

echo "=========================================="
echo " 完成。产物目录: $OUT"
echo "   3D: $OUT/GlobalMap.pcd"
[ "$DO_2D" = 1 ]     && echo "   2D: $OUT/occgrid.pgm + occgrid.yaml"
[ "$DO_DEPLOY" = 1 ] && echo "   已部署: /opt/dog/map/current_map -> $STAMP"
echo "=========================================="
