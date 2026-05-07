#!/usr/bin/env bash
# fast-lio-sam 跑一遍 bag, 在 bag 退出后主动调 /save_map service
# 拿 PGO 修正后的 GlobalMap.pcd, 再 SIGINT fastlio.
#
# 为什么需要这一步:
#   - launch 退出时只发 SIGINT, fastlio 把当前 IEKF 状态累积的 scans.pcd
#     写到 PCD/ 下 — 这份 PCD 用的是前端 (可能漂飞) 的位姿, 不是 PGO 修正后的
#   - PGO 修正后的全局地图只有调 /save_map service 才会触发拼接 (见 src/laserMapping.cpp
#     的 saveMapService) — 输出 GlobalMap.pcd / trajectory.pcd / transformations.pcd
#
# 用法:
#   ./run_with_savemap.sh <bag_dir> <run_dir> [config_yaml]
# 例:
#   ./run_with_savemap.sh /data/airy_xxx /tmp/run1
#   ./run_with_savemap.sh /data/airy_xxx /tmp/run1 config/airy_handheld.yaml
set -e

BAG="${1:?bag dir required}"
RUN="${2:?run dir required}"
CFG="${3:-$(ros2 pkg prefix fast_lio_sam)/share/fast_lio_sam/config/airy.yaml}"

mkdir -p "$RUN"

# 1) fastlio 节点 (use_sim_time=true, 跟随 bag /clock)
ros2 run fast_lio_sam fastlio_mapping --ros-args \
    --params-file "$CFG" \
    -p use_sim_time:=true \
    > "$RUN/fastlio.log" 2>&1 &
FLIO_PID=$!
echo "fastlio_pid=$FLIO_PID"

# 等 fastlio 起来 + IMU init
sleep 5

# 2) bag play (foreground, 用 --clock 提供 sim time)
echo "=== bag play start at $(date +%H:%M:%S) ==="
ros2 bag play "$BAG" --clock 100 --rate 1.0 > "$RUN/bag_play.log" 2>&1
echo "=== bag play done at $(date +%H:%M:%S) ==="

# 3) 等 PGO 处理完最后几帧 (loopClosureFrequency=1.0 + isam 收敛)
sleep 8

# 4) call /save_map -> PGO 修正后的全局地图, 输出到 yaml 里 savePCDDirectory 指定位置
echo "=== save_map call at $(date +%H:%M:%S) ==="
ros2 service call /save_map fast_lio_sam/srv/SaveMap "{resolution: 0.0, destination: ''}" \
    > "$RUN/save_map.log" 2>&1 || echo "save_map failed"
echo "=== save_map returned at $(date +%H:%M:%S) ==="
cat "$RUN/save_map.log"

# 5) SIGINT fastlio 让它正常退出
sleep 3
kill -INT $FLIO_PID 2>/dev/null || true
wait $FLIO_PID 2>/dev/null || true
echo "=== all done at $(date +%H:%M:%S) ==="
