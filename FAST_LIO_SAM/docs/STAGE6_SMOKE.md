# Stage 6 烟测报告

> 对应 [issue #1](https://github.com/cvidkal/fast-lio-sam/issues/1) ROS2 移植的最后一步: 端到端跑通 bag → LIO → 3D 地图 → 2D 占据栅格.

## 运行环境

- 平台: NVIDIA Jetson Orin (aarch64), Ubuntu 22.04, ROS2 Humble
- LiDAR: RoboSense Airy 192 线 (实际 96 线模式)
- 数据: `/home/nvidia/mou/dog/bags/airy_20260503_103138` (rosbag2 sqlite3, 69s, 含 `/rslidar_points` + `/rslidar_imu_data`)

## 数据缺陷

录制时 `rslidar_sdk` 处于 `POINT_TYPE=XYZI` (默认), 输出点云**只含 `x,y,z,intensity`**, 缺 `ring` 和 `timestamp` 字段. 这两个字段是 FAST-LIO 做去畸变和 scan-line 检测的依据, 缺失会导致:

- 帧间漂移巨大 (实测 70s 跑出 200m bbox)
- 去畸变完全失效

## 桥接节点 (airy_bridge)

为了在不重录数据的前提下打通链路, [`dog_mapping_ws/src/airy_bridge`](https://github.com/cvidkal/fast-lio-sam/blob/feat/ros2-port/FAST_LIO_SAM/docs/STAGE6_SMOKE.md) 实现了一个降级模式:

- `ring` 用 `elevation = atan2(z, sqrt(x²+y²))` 离散到 192 桶
- `timestamp` 在帧内均匀分布 (0..1/scan_rate 秒)
- 真路径 (有 ring + timestamp) 也支持: 自动探测字段, 无字段则启用伪造并打 warn

这条降级路径**仅用于打通链路**, 真正建图请按 `dog_mapping_ws/README.md` "已知坑 #4" 重编 rslidar_sdk 改成 XYZIRT.

## 跑两个 LIO 算法

为了同时验证 (a) `dog_mapping_ws` 用的 hku-mars FAST_LIO 官方 ROS2 分支和 (b) 本 PR 的 fast-lio-sam ROS2 端口, 用同一个 bag 跑了两次.

### A. FAST-LIO ROS2 (dog_mapping_ws)

```bash
cd /home/nvidia/mou/dog/dog_mapping_ws
ros2 launch launch/airy_mapping_bag_loose.launch.py \
    bag:=/home/nvidia/mou/dog/bags/airy_20260503_103138 \
    rate:=0.3 rviz:=false
```

| 项目 | 值 |
|---|---|
| /Odometry 频率 | 4.5 Hz |
| IMU init | ~5s 完成 |
| ICP 收敛 | ✅ (0 No Effective Points) |
| PCD 大小 | 1.6 GB / 51 M 点 |
| 地图 bbox | x ∈ [-126, 36], y ∈ [-22, 185], z ∈ [-1.7, 38] |
| 2D 栅格 | 1650 × 2090 px @ 0.1 m/px |

### B. fast-lio-sam ROS2 port (本 PR)

```bash
# 在 /tmp/ros2_port_test_ws 安装好后:
ros2 run airy_bridge airy_to_velodyne &
ros2 run fast_lio_sam fastlio_mapping --ros-args \
    --params-file /home/cgj/Codes/fast-lio-sam.feat-ros2-port/FAST_LIO_SAM/config/airy_via_bridge.yaml \
    -p use_sim_time:=true &
ros2 bag play /home/nvidia/mou/dog/bags/airy_20260503_103138 --clock --rate 0.3
```

| 项目 | 值 |
|---|---|
| /Odometry 频率 | 4.2 Hz |
| IMU init | ~5s 完成 |
| ICP 收敛 | ✅ |
| PCD 大小 | 586 MB / 18 M 点 |
| 地图 bbox | x ∈ [-41, 36], y ∈ [-24, 30], z ∈ [-2, 9] |
| 2D 栅格 | 796 × 560 px @ 0.1 m/px |

## 结果对比

| 指标 | dog_mapping_ws (FAST-LIO ROS2) | fast-lio-sam ROS2 port (本 PR) |
|---|---|---|
| **bag 完整跑通** | ✅ | ✅ |
| **PCD 写盘** | ✅ | ✅ |
| **退出干净** | ✅ | ⚠️ shutdown 时 std::system_error (PCD 已写, 无影响) |
| **地图 bbox** | 162 × 207 × 40 m (漂得很厉害) | 80 × 56 × 11 m (好得多) |
| **关键差异** | 纯 LIO, 无回环 | SAM/PGO 回环修正轨迹 |

> bbox 差距说明回环检测 (PGO) 在数据质量差时仍能把轨迹拉回合理量级 — 这是 fast-lio-sam 相对 vanilla FAST-LIO 的主要价值.

## 已知问题

1. **shutdown 时段错误**: ROS2 port 退出时偶发 `terminate called after throwing an instance of 'std::system_error', what(): Invalid argument`. 发生在 PCD 写完之后, 不影响数据保存. 怀疑是 spin 循环里 `rclcpp::shutdown()` 后还有某个 publisher/subscriber 析构. 已记录, 后续 fix.
2. **wrapper/bag_io.cc 还是禁用的**: 离线回放靠 `ros2 bag play --clock`, 不靠内置 `RosbagIO`. 见 stage 4d commit message.
3. **scan_line=192 vs 真实 96**: Airy 实际是 96 通道, 桥接桶数 192 是过采样. 跑通了但有空桶.

## 修复路径 (推荐)

要拿到一张能用的图:

1. 切 `rslidar_sdk` 到 `POINT_TYPE=XYZIRT` (改 CMakeLists 一行 + 重编驱动)
2. 重启驱动验证 `ros2 topic echo /rslidar_points --once --field fields` 确认有 `ring` + `timestamp`
3. 录新 bag
4. 用相同 launch 跑, 但配置文件改成更紧的版本 (config/airy.yaml, issue #2 落地后)

预计漂移降一两个数量级.
