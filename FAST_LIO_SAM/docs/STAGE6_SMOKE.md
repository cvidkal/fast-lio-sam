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

## 烟测 v2: 真实 XYZIRT 数据 (2026-05-04)

按"修复路径"重做了一遍. 结果和预期一致, 但**踩出了 5 个新坑** (都已记录在 dog_mapping_ws/README.md 的"已知坑"节).

**配置**: rslidar_sdk POINT_TYPE=XYZIRT 重编, scan_line 改成实测的 96 (不是 192), 静止 30s LIVE 模式 (不回放 bag).

**结果**:

| 指标 | v1 (伪造字段) | **v2 (真实 XYZIRT)** |
|---|---|---|
| bbox | 162 × 207 × 40 m | **7.07 × 4.64 × 2.33 m** ✅ 真实房间尺寸 |
| 点数 | 51 M | 2.66 M |
| loop back | 大量 | **0** |
| No Effective | 0 (但漂移) | **0** |
| Odometry 频率 | 3.5 Hz | 3.8 Hz |

数量级差距确认: **POINT_TYPE=XYZIRT 是 Airy + FAST-LIO 的硬性前提**, 不是可选项.

## 真实数据踩出的 5 个坑

(都在 `dog_mapping_ws/README.md` "已知坑" 节有记录)

1. **rslidar_sdk POINT_TYPE 默认 XYZI, 必须切 XYZIRT** — 没 ring/timestamp LIO 直接漂. 而且如果是非 owner 编 cmake, configure_file 会因 utime EPERM 报错, 要先删 `*.in` 的目标文件.

2. **eno1 网口需要 UP + 192.168.1.102/24** — driver bind 才能成 (一开始网线插着但接口 DOWN, 翻车了 10 分钟).

3. **FastDDS shm 跨用户死锁** — driver 跑 nvidia 用户 (systemd), 应用跑 cgj 用户. `/dev/shm/fastrtps_*` 的权限链对另一边不通, 消息默默丢失. 表现非常诡异 (`topic list` 看得到, `echo` / `hz` 收不到). 解决: 强制走 UDP, 见 `dog_mapping_ws/config/fastdds_no_shm.xml`.

4. **airy_bridge 默认 BEST_EFFORT pub** vs **FAST-LIO velodyne handler 默认 RELIABLE sub** — DDS 不匹配, fastlio 永远收不到. 改 bridge 输出端为 RELIABLE 就通. (社区/教学版 ROS2 节点常踩这个坑.)

5. **回放 bag 时 driver 还在 LIVE 发同名 topic** — 双源混乱, 同一个 `/rslidar_points` 上的 stamp 来回跳, fastlio 触发 "lidar loop back" 雪崩. 解决: 回放前 stop driver service, 或干脆只用 LIVE 模式不回放.

这些坑是 ROS2 移植本身的功能完整性 **之外** 的部署陷阱, 但任何想真正用这个 fork 跑 Airy 数据的人都会撞上, 所以记录在案.
