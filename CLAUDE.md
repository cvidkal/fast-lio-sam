# CLAUDE.md — fast-lio-sam (cvidkal fork)

> 给未来的 Claude 会话用的项目地图。读完这一份就能在仓库里干活，不需要从头摸。

## 这是什么

`cvidkal/fast-lio-sam` 是 **HKU-MARS FAST-LIO2** 的一个 fork，叠了 **SAM/PGO 回环检测** (LIO-SAM 风格的 GTSAM 因子图后端)。本仓库已**完整移植到 ROS2 Humble** (PR #9)、原生支持 **RoboSense Airy / Helios / Bpearl** 等所有 `rslidar_sdk v1.5+ PointXYZIRT(double timestamp)` 雷达 (PR #11)。

主要消费者: **机器狗 (quadruped)** + **人手持采集**, 都是 LiDAR-Inertial 建图. 硬件:
- NVIDIA Jetson Orin (aarch64 / Ubuntu 22.04 / ROS2 Humble)
- RoboSense Airy 192/96 通道半球形 LiDAR (走 Ethernet 192.168.1.200 ↔ 主机 eno1 192.168.1.102)
- 内置 Airy IMU (~200 Hz)

> 手持采集模式注意: 起步时 IMU 通常没法保证水平 → FAST-LIO init 会把略歪的"重力方向"
> 固化进世界坐标系 (实测可以差几度). 后处理工具 `tools/align_floor` 解决这个.

## 目录结构

```
FAST_LIO_SAM/
├── package.xml          ← ament_cmake (legacy/ros1/ 是 catkin 备份)
├── CMakeLists.txt       ← 原生写的 (老 ROS1 没 CMakeLists)
├── PORTING.md           ← ROS2 移植 6 stage 的全程记录
├── src/
│   ├── laserMapping.cpp ← 主节点 2620 LOC, ROS2 化改了 ~600 行 (stage 4a-d)
│   ├── preprocess.h/cpp ← 各家 LiDAR 解码; lidar_type ∈ {1..5}
│   ├── IMU_Processing.hpp / GNSS_Processing.hpp / common_lib.h ← header-only
│   └── ...
├── include/
│   ├── ikd-Tree/        ← submodule, 增量 KD-tree
│   └── IKFoM_toolkit/   ← 流形上 IKF
├── config/              ← ROS2 yaml (用 `/**:\n  ros__parameters:` 包一层)
│   ├── velodyne16.yaml
│   ├── airy.yaml         ← Airy 原生 lidar_type=5 (狗用, 默认 ext_est=true)
│   ├── airy_handheld.yaml ← Airy 手持采集预设 (DIFOP 真外参 + ext_est=false)
│   └── airy_via_bridge.yaml ← Airy 走 airy_bridge 兼容路径 (lidar_type=2)
├── launch/
│   ├── mapping_velodyne16.launch.py
│   ├── mapping_airy.launch.py     ← LIVE 建图 (driver 实时)
│   └── mapping_bag.launch.py      ← rosbag2 回放 (PR #10)
├── tools/
│   ├── pcd_to_occgrid/             ← PCD → nav2 PGM/YAML (PR #7), 支持 --raycast / --auto-floor
│   ├── airy_extrinsic/             ← DIFOP 外参解析 CLI (PR #8) + examples/
│   └── align_floor/                ← gravity 后校正 (RANSAC + Rodrigues)
├── bin/
│   └── run_with_savemap.sh         ← bag 跑完后自动调 /save_map 拿 PGO 修正 GlobalMap
├── scripts/
│   └── bag_inspect.py              ← rosbag2 体检 (PR #10)
├── docs/
│   ├── STAGE6_SMOKE.md             ← 烟测报告 + 5 个部署 gotcha
│   └── rosbag2_workflow.md         ← bag 录/查/回放/转图工作流
├── rviz_cfg/                       ← RViz config 文件
├── msg/Pose6D.msg
├── srv/SaveMap.srv  SavePose.srv
├── wrapper/                        ← bag_io.cc 现在禁用 (依赖内部 lightning fw)
└── legacy/ros1/                    ← 原始 ROS1 launch + config + package.xml
```

## LiDAR 类型 (`lidar_type` 枚举)

定义在 `src/preprocess.h`:

| 值 | enum | 适用 | 字段格式 |
|---|---|---|---|
| 1 | `LIVOX` | Livox Avia / Mid-360 | `livox_ros_driver2::msg::CustomMsg` |
| 2 | `VELO16` | Velodyne / 通过 airy_bridge 中转的 Robosense | `(x,y,z,intensity,float time,uint16 ring)` |
| 3 | `OUST64` | Ouster | Ouster point struct |
| 4 | `RS128` | **老** suteng_msgs 的 Robosense (`float time` ms 相对) | rslidar_ros::Point |
| **5** | `RSLIDAR_NEW` | **现行 rslidar_sdk v1.5+** Airy/Helios/Bpearl/E1/M1/Ruby Plus | `(x,y,z,intensity,uint16 ring,double timestamp)` ← **Airy 用这个** |

PR #11 加了 `lidar_type=5`, 直接吃 rslidar_sdk POINT_TYPE=XYZIRT 输出，不再需要 airy_bridge。

## 常见命令

### 编译
```bash
# 假设 ws 在 /home/cgj/ws/fls
cd ~/ws/fls
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio_sam --cmake-args -DCMAKE_BUILD_TYPE=Release
```

构建依赖 (apt):
```
ros-humble-{rclcpp,std-msgs,nav-msgs,sensor-msgs,geometry-msgs,tf2,tf2-ros,
            tf2-geometry-msgs,pcl-ros,pcl-conversions,visualization-msgs,
            rosbag2-cpp,gtsam}
libpcl-dev libeigen3-dev libgoogle-glog-dev libgeographic-dev
```
还要 `livox_ros_driver2` 在 ws 里 (msg-only stub 也行, 见 `dog_mapping_ws/src/livox_ros_driver2/`).

### LIVE 建图 (Airy, driver 必须先起)
```bash
ros2 launch fast_lio_sam mapping_airy.launch.py
```
默认 yaml: `config/airy.yaml`, `lidar_type=5`, 订阅 `/rslidar_points` + `/rslidar_imu_data`。

### bag 回放建图
```bash
ros2 launch fast_lio_sam mapping_bag.launch.py \
    bag:=<rosbag2_dir> \
    config_file:=$(ros2 pkg prefix fast_lio_sam)/share/fast_lio_sam/config/airy.yaml \
    rate:=1.0 \
    stop_service:=airy-lidar.service    # ← 自动 stop driver, 完成后 start (要 NOPASSWD sudo)
```

### bag 体检
```bash
python3 $(ros2 pkg prefix fast_lio_sam)/share/fast_lio_sam/scripts/bag_inspect.py <bag_dir>
# 检查字段 / 频率 / 单调性 / IMU-LiDAR 时间重叠
# 退出码非零 = 致命问题, 不要喂 LIO
```

### PCD → 2D nav2 地图
```bash
python3 .../tools/pcd_to_occgrid/pcd_to_occgrid.py \
    /tmp/scans.pcd -o ~/maps/airy_room \
    --resolution 0.05 --z-min 0.10 --z-max 1.50
```

### Airy 出厂外参
```bash
python3 .../tools/airy_extrinsic/airy_extrinsic.py live --port 7788 -o airy_extrinsic.yaml
# 或 pcap / bag 模式. 输出直接粘到 mapping.extrinsic_T/R
```

## 部署陷阱 (5 个，必读)

详见 `docs/STAGE6_SMOKE.md`。简版:

1. **`rslidar_sdk` 必须 `POINT_TYPE=XYZIRT`** + `ENABLE_IMU_DATA_PARSE=ON` 重编。默认 `XYZI` 没 ring/timestamp，LIO 直接漂 (实测 70s 漂出 200m bbox)。
   - 如果以非 owner 编译它, `configure_file` 会 EPERM (utime 不够权限)。`rm -f *.cmake *.hpp` 让 cmake 重建即可。

2. **`eno1` 网口要 UP @ `192.168.1.102/24`** driver 才能 bind UDP 6699/6688/7788。

3. **FastDDS 跨用户 shm 死锁**: driver 在 nvidia 用户 (systemd `airy-lidar.service`), 你的工具一般在 cgj 用户。`/dev/shm/fastrtps_*` 文件权限链不通, **消息默默丢失**, 表现是 `ros2 topic list` 看得到, `echo`/`hz` 收不到。
   - 解决: `export FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only.xml>` 强走 UDP。模板见 `dog_mapping_ws/config/fastdds_no_shm.xml`。

4. **QoS 兼容方向**: `RELIABLE pub` ↔ `BEST_EFFORT sub` ✅; `BEST_EFFORT pub` ↔ `RELIABLE sub` ❌。fastlio_mapping 默认 BEST_EFFORT 订阅, driver 默认 RELIABLE 发布, 这条没问题。但如果你写 bridge 节点, **输出端要 RELIABLE** (兼容下游 RELIABLE 默认), **输入端 BEST_EFFORT** (兼容 RELIABLE 上游)。

5. **bag 回放期间 LIVE driver 必须停**: 同一 topic 两套 stamp, fastlio 触发 `lidar loop back` 雪崩, ICP 完全乱。`mapping_bag.launch.py` 的 `stop_service` 参数处理这个 (要 sudoers NOPASSWD)。或用 `ROS_DOMAIN_ID=42` 隔离。

## ROS1→ROS2 化的变化 (移植细节)

`PORTING.md` 有完整 6 stage 故事。要点:
- `roscpp` → `rclcpp` (header `<rclcpp/rclcpp.hpp>`, msg 类型多一层 `::msg::`)
- `ros::Publisher pubX` → `rclcpp::Publisher<T>::SharedPtr pubX` (要写明类型!)
- `pub.publish(msg)` → `pub->publish(msg)`
- `nh.param<T>("k", v, def)` → `node->declare_parameter<T>("k", def); node->get_parameter("k", v)`
- `ros::Time().fromSec(x)` → `rclcpp::Time(int64_t(x*1e9), RCL_ROS_TIME)`
- `header.stamp.toSec()` → `rclcpp::Time(header.stamp).seconds()`
- service 回调签名: `bool fn(Req&, Res&)` → `void fn(const SharedPtr<Req>, SharedPtr<Res>)` (返回 void, 改写 res->success)
- `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`, 需要 `node` 引用
- launch xml → launch.py
- yaml: 必须 `/**:\n  ros__parameters:\n` 包一层, 嵌套用 `.` (例如 `mapping.extrinsic_T`)

## 已知小毛病 / TODO

- **`wrapper/bag_io.cc` 禁用中**: 依赖内部 `lightning` framework (IMUPtr / Vec3d / global ToSec), 不在 repo 里。`#include` 注释了, CMakeLists 不编。离线回放走 `ros2 bag play --clock` 路径 (`mapping_bag.launch.py`)。
- **PGO 静止欠定**: SAM 因子图在纯静止数据上会抛 `IndeterminantLinearSystemException`。算法本性, 不是 bug, 但要在文档里 highlight。
- **`scan_line` 与硬件**: Airy 96/192 模式可切, 但 yaml 里要手动改 (rslidar_sdk DIFOP install_mode 决定实际值)。当前 `airy.yaml` 写 96。
- **`launch` 退出时不调 `saveMap`**: `src/laserMapping.cpp` 主循环退出后没自动 call `saveMapService`, 导致 PCD 用的是前端 IEKF 状态 (可能漂飞), 不是 PGO 修正后的. 临时方案: 用 `bin/run_with_savemap.sh` 在 SIGINT 前手动 `ros2 service call /save_map`. 长期方案: 在 main 末尾或 SigHandle 后加 `if (savePCD) saveMap();` 调用 (line 2545 那行注释).
- **gravity 没真正对齐**: FAST-LIO 在 IMU init 那 0.1s 锁死世界 z. 手持/狗站歪时整张图差几度. 用 `tools/align_floor/align_floor.py` 后处理校正.

## 兄弟工作区

`/home/nvidia/mou/dog/dog_mapping_ws/` 是 quadruped 集成工作区 (不在本仓库内), 含:
- `src/airy_bridge/` — Robosense → Velodyne 字段重排 Python 节点 (PR #11 后理论上不再需要, 留作兼容老 lidar_type=2 路径)
- `src/FAST_LIO/` — vanilla hku-mars FAST_LIO ROS2 (用作对照, 不带 SAM)
- `config/` — Airy 调好的 yaml, `fastdds_no_shm.xml`
- `scripts/00_setup_root.sh` — 一次性 sudo 安装系统依赖 + 改 rslidar_sdk
- `results/SESSION_SUMMARY.md` / `HANDOFF.md` — 跨 session 上下文交接

## 当前 issue / PR 状态 (2026-05-05)

| # | 状态 |
|---|---|
| #1 ROS2 移植 | ✅ closed (PR #9) |
| #2 Airy 原生 lidar_type | ✅ closed (PR #11) |
| #3 Airy DIFOP 外参 | ✅ closed (PR #8) |
| #4 quadruped 调参预设 `airy_dog.yaml` | **OPEN** — 等狗能动后调真实运动数据 |
| #5 rosbag2 replay launch | ✅ closed (PR #10) |
| #6 pcd_to_occgrid | ✅ closed (PR #7) |

## 相关上游

- [hku-mars/FAST_LIO `ROS2`](https://github.com/hku-mars/FAST_LIO/tree/ROS2) - 官方 FAST-LIO ROS2
- [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) - 社区 ROS2 端口
- [rohrschacht/FAST_LIO_SLAM_ros2](https://github.com/rohrschacht/FAST_LIO_SLAM_ros2) - SAM/PGO ROS2 移植参考
- [TixiaoShan/LIO-SAM `ros2`](https://github.com/TixiaoShan/LIO-SAM/tree/ros2) - LIO-SAM 官方 ROS2
- [RuanJY/robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio) - Robosense PointXYZIRT 适配参考 (PR #11 借鉴了它的 Point 结构)
