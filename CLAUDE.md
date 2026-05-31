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
│   ├── airy_test_no_extr_est.yaml ← Airy 原生 lidar_type=5 (真实外参, **首选 — 狗用**)
│   ├── airy_handheld.yaml ← Airy 手持采集预设 (DIFOP 真外参 + ext_est=false)
│   ├── airy_no_loop.yaml  ← 同上但关 loop closure
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

### Worktree 下编译/运行 (多分支并行, 必读)

git worktree 给每个分支独立源码树, 但 colcon 的"一个 `ws/src/<包>` + ws 根放
build/install/log"工作方式跟它冲突。**直接 `cd ~/fls_ws && colcon build` 编的是那个
写死软链指向的树 (通常是 main), 不是你当前 worktree 改的分支。** 用仓内 helper 让每个
worktree 自包含 (已验证 2026-05-31: in-place 编 2.5min / EXIT 0, 产物不弄脏 git):

```bash
# 编当前 worktree (产物落 FAST_LIO_SAM/{build,install,log}, 被 .gitignore 吃掉)
bin/wt_build.sh                              # Release; 透传额外 colcon 参数, 如 --symlink-install
# 跑当前 worktree (按分支名 hash 出唯一 ROS_DOMAIN_ID, 自动隔离 DDS, 多分支同时跑不抢 topic)
bin/wt_run.sh launch fast_lio_sam mapping_bag.launch.py bag:=... config_file:=...
# 别的终端对同一 worktree 操作 (echo/hz/service call) 前, 先 export wt_run 打印的那行 ROS_DOMAIN_ID
```

**三条铁律** (不照做 worktree 就会互相污染):
1. **必须从 `FAST_LIO_SAM/` 里跑 colcon** (helper 已用 `BASH_SOURCE/..` 锁定包根)。
   在 worktree 根跑产物落根目录 → 那里没 `.gitignore` (它在 `FAST_LIO_SAM/` 里) → 弄脏 git。
2. **source `~/fls_ws/install` 当 underlay** 白嫖已编好的 `livox_ros_driver2` (唯一非 apt
   工作区依赖) + apt 依赖。会有一句 `overriding fast_lio_sam` warning, 无害。覆盖路径用 `FLS_UNDERLAY`。
3. **每个 worktree 唯一 `ROS_DOMAIN_ID`** (helper 自动按分支 hash 到 1..101, 永不为 0 →
   不撞默认域 0 的 main)。否则 live 跑会抢 `/Odometry` `/rslidar_points`, 触发 `lidar loop back` 雪崩。

注: `legacy/ros1/package.xml` 包名也是 `fast_lio_sam` (catkin 版), 已用 `legacy/COLCON_IGNORE`
屏蔽整个 `legacy/` 子树, 避免 colcon 撞 duplicate package。

### LIVE 建图 (Airy, driver 必须先起)
```bash
ros2 launch fast_lio_sam mapping_airy.launch.py
```
默认 yaml: `config/airy_test_no_extr_est.yaml`, `lidar_type=5`, 订阅 `/rslidar_points` + `/rslidar_imu_data`。
**注意**: 已删除的 `airy.yaml` 用 identity 外参, 在真实 Airy 硬件 (IMU 装反) 上会让 ba 爆炸到 +2.7 m/s² → 几公里漂飞.

### bag 回放建图
```bash
ros2 launch fast_lio_sam mapping_bag.launch.py \
    bag:=<rosbag2_dir> \
    config_file:=$(ros2 pkg prefix fast_lio_sam)/share/fast_lio_sam/config/airy_test_no_extr_est.yaml \
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

## 2026-05-26 修复总集 (按适用范围分层)

会话上下文: airy bag 跑出几公里 drift → 修外参 → loop 重影 → submap 污染 → z 反向. 全部 root cause 已 nail.

### A. 通用 (任何 LiDAR + IMU 组合, 改的是 fastlio 上游的 SAM/PGO 跟 IKF 默认值)

A1. **Loop closure 5 项联调** (默认配置对小室内一律出重影 — 因为 default 是大尺度 hokuyo / 室外调的):
- `src/laserMapping.cpp` ICP setMaxCorrespondenceDistance: 150 → **5** m. 150 让 ICP 跨房间错配
- `src/laserMapping.cpp` Loop noise 拆分: 原 `Vector6 << fitness×6` 旋转 std 高达 31°, 改 `rot=1e-6, trans=max(fitness,1e-4)`
- `src/laserMapping.cpp` Odom edge noise: `1e-4 m² → 5e-3 m²` (1cm → 7cm std), 默认 1cm 死压 loop, 改后 PGO 才能拉
- yaml `historyKeyframeSearchNum`: 25 → **5**. **这是最隐蔽的 bug** — submap 含 25 个邻居 KF, 跨 region 的 KF 把 drift 全污染进 target, ICP 算出 dz≈0, 实际差 45cm
- yaml `historyKeyframeFitnessScore`: 0.3 → **0.02**. 0.3 接受 ICP RMS 55cm 烂匹配 → 错 loop 拉乱

A2. **z-up convention fix** (`IMU_Processing.hpp::IMU_init`): 构造 `state.rot` 把 body-up 旋到 world +z, `state.grav = (0,0,-G_m_s2)`. 默认 fastlio 让 world frame 继承 IMU body 朝向, 装反或装歪都会让 raw 输出 (`/Odometry`, `scans.pcd`, `trajectory.pcd`) 用奇怪 z 方向, RViz/Nav2 要反复转. 数学等价但外部约定一致.

A3. **删除 `airy.yaml` 这种 identity-extrinsic config**, 改默认到含真外参的 yaml. 任何 LiDAR-IMU 组合都该校外参, 不能默认 identity.

### B. Airy LiDAR 特定 (这家硬件的特殊约定)

B1. **IMU linear_acceleration 单位是 g (不是 ROS 标准 m/s²)**. mean_acc.norm() ≈ 1.0. FAST-LIO 内部 `acc * G/|mean_acc|` 自动 normalize 能 handle, 不用手动 fix.

B2. **IMU 装反 (body z 朝下, LiDAR 朝上)**. config 必须用真实外参矩阵 (180° flip), identity 会让 ba 在 5s 内爆炸到 +2.7 m/s² → 几公里漂飞:
```yaml
extrinsic_T: [0.00425, 0.00418, -0.00446]
extrinsic_R: [-0.006915, -0.999975,  0.001559,
              -0.999950,  0.006903, -0.007273,
               0.007262, -0.001609, -0.999972]
```

### C. Airy + 机器狗特定 (高振动平台跟 handheld 差别大)

C1. **Odom edge translation noise 必须放松** (A1 列了改成 5e-3). 默认 1e-4 (1cm std) 在 handheld 都已偏紧, 在狗这种 step-impact 高振动平台直接飘. 这是 C1 跟 A1 重合的部分 — 哪怕只跑 handheld 也该改, 但跑狗这种平台**绝对必须**改.

C2. **后续可能要调 IMU cov** (`mapping.acc_cov`/`gyr_cov`): 默认 0.1, 狗高动态时 (gyro 1-2 rad/s, acc ±4g shock) 可能要 0.3-0.5. 当前 bag 还算温和, 没踩到. 跑跑跳跳的 bag 上 drift 还顶不住时再调.

C3. **狗有大量"停一会儿转一圈"行为**, 一个 bag 里多次原地旋转扫描. 这种采集 mode 让 `historyKeyframeSearchNum` 默认 25 的 submap 跨多个停留点 (各 z 略不同), 污染特别严重. 改 5 是这个采集模式的必需 (handheld 一次性扫一个房间问题不大).

## 已知小毛病 / TODO

- **shutdown segfault**: ROS2 port 退出时偶尔 `terminate called after throwing an instance of 'std::system_error'`。PCD 已落盘后才发生, 不丢数据, 但需要 fix。怀疑 spin 循环里 `rclcpp::shutdown()` 后还有 publisher/subscriber 析构。
- **`wrapper/bag_io.cc` 禁用中**: 依赖内部 `lightning` framework (IMUPtr / Vec3d / global ToSec), 不在 repo 里。`#include` 注释了, CMakeLists 不编。离线回放走 `ros2 bag play --clock` 路径 (`mapping_bag.launch.py`)。
- **PGO 静止欠定**: SAM 因子图在纯静止数据上会抛 `IndeterminantLinearSystemException`。算法本性, 不是 bug, 但要在文档里 highlight。
- **`scan_line` 与硬件**: Airy 96/192 模式可切, 但 yaml 里要手动改 (rslidar_sdk DIFOP install_mode 决定实际值)。当前 `airy_test_no_extr_est.yaml` 写 96。
- **`launch` 退出时不调 `saveMap`**: `src/laserMapping.cpp` 主循环退出后没自动 call `saveMapService`, 导致 PCD 用的是前端 IEKF 状态 (可能漂飞), 不是 PGO 修正后的. 临时方案: 用 `bin/run_with_savemap.sh` 在 SIGINT 前手动 `ros2 service call /save_map`. 长期方案: 在 main 末尾或 SigHandle 后加 `if (savePCD) saveMap();` 调用 (line 2545 那行注释).
- **gravity 微调**: 2026-05-26 IMU init 已修正粗 z-up 朝向 (见上面 A2). 但 IMU init 那 0.1s 静态拟合还会有几度 tilt, `tools/align_floor/align_floor.py` 后处理可以再校到亚度.

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

## 上机定位栈 (2026-05-11 起)

**建图用 fast-lio-sam**, **上机定位用另一栈**, 两套独立:

```
/home/chenguojun/fls_ws/src/
├── FAST_LIO_SAM/           ← 本仓库 (mapping)
├── livox_ros_driver2/      ← Livox driver (msg-only stub)
├── ndt_omp_ros2/           ← NDT_OMP backend (rsasaki 用; **新方案不再依赖**)
├── lidar_localization_ros2/ ← rsasaki0109 fork (NDT 实验, **轨迹有 NDT 噪声, 已弃用**)
└── open3d_loc/             ← deepglint/FAST_LIO_LOCALIZATION_HUMANOID humble + Airy 适配 ← ★主用
```

**当前主方案 (2026-05-12 verified, 架构 B): fast-lio-sam localization_mode**

经过详细对比 (见 GitHub issues #27, #28), 选定 **scan-to-prior 单层架构**:
- `fast-lio-sam` 启动时预加载 prior PCD 进 ikd-Tree (固定, 不再 incremental update)
- IKF 直接 scan-to-prior 匹配, 出 `/Odometry` 直接在 prior frame
- 不需要 open3d_loc 的 ICP correction (它跟 fastlio loc_mode 协同会双重 transform, 反而破坏)
- 启动时 yaml `localization.init_pos_xyz/init_rot_quat` 给个粗 hint (±0.5m), 运行时 `/initialpose` (来自 relocation_node 或 RViz) 精修到 cm 级

精度 (bag2-on-bag1+bag2 merged prior, vs transformed bag2 GT):
- median NN 5-15cm (干净 prior 区域 1-3cm)
- 95th 40cm, max 50cm
- end-to-end loop closure 6cm

冷启动 cm 级流程:
```bash
ros2 launch open3d_loc airy_fastlio_loc_cold_start.launch.py rate:=1.0
# yaml hint 偏 0.5m → fastlio 5s dm 级 → relocation_node ICP refine → /initialpose → fastlio 1s 拉到 cm 级
```

依赖: `~/open3d019` (从 isl-org/Open3D v0.19 releases prebuilt cxx11-abi tarball 解), `libc++-dev libc++abi-dev` (apt).

**架构 A (备选, 弃用): fast-lio-sam mapping_mode + open3d_loc ICP correction** 已弃用. 见 issue #28: open3d_loc ICP 在跨 bag z-degeneracy 拉 7m, 修了 ICP init bug 但仍受 fastlio ikd-Tree growing 累积错几何影响 + "两个 map 打架". 架构 B 更干净更准.

**bag2-on-bag1 实测**:
| 指标 | rsasaki NDT (旧) | open3d_loc (新) |
|---|---|---|
| /pcl_pose density | 245 / 152s (1.6Hz) | **1301 / 132s (10Hz)** |
| Median 速度 | 3.3 m/s (噪声主导) | **0.94 m/s (真实步行)** |
| z std | 2.29m | **1.16m** |
| 轨迹形态 | xy 乱跳 | 平滑跟 corridor |

**rsasaki 路线弃用原因**: NDT_OMP 在 bag1 prior + bag2 scan 上 fitness median 0.6 但对应 RMS ~0.8m, 帧间跳 1-2m. Open3D ICP 同数据 fitness 0.6-0.9 / rmse 0.27m, 是质的差距. 弃用. 留 lidar_localization_ros2 包当 fallback / 参考.

**关键 3 件套 (rsasaki 时代用过, open3d_loc 不需要了)**:
- `lidar_localization_ros2` (上游): NDT_OMP scan-to-map 跟踪, 接受 `/initialpose` (RViz click 或程式发) — 已弃用
- `scripts/auto_init_pose.py`: 启动时用 ScanContext++ (`pyscancontext` pip 装) 把 bag1 226 KFs 建 DB, 头几帧自动匹配 → 发 `/initialpose` — 可复用做 init
- `scripts/imu_reframe.py`: Airy IMU adapter (rsasaki 时代必须). **open3d_loc 不需要** (fast-lio-sam 直接吃原始 Airy IMU)

**统一 launch**:
```bash
ros2 launch lidar_localization_ros2 airy_localization.launch.py \
    param_yaml:=.../airy_bag2_on_bag1.yaml \
    prior_kf_dir:=~/Downloads/LOAM_173924/pcd \
    prior_pose_json:=~/Downloads/LOAM_173924/pose.json \
    extrinsic_yaml:=~/results/airy_real_ext.yaml \
    bag:=~/bags/airy_20260510_175730 rate:=2.0 bag_start_delay_sec:=8.0
# 上机 live mode: bag:='' use_sim_time:=false
```

### 部署陷阱 (再加 4 个, 7-10):

7. **base_link → rslidar 静态 TF 必须 = LiDAR-IMU 外参** (R_imu_lidar from yaml, ~180° flip), **不是 identity**. 因 fastlio mapping 时 `state.rot = I, state.pos = 0` at t=0, 保存的 prior 在 "world = IMU at t=0" frame, 跟 LiDAR 差一个 R_imu_lidar.

8. **Airy IMU 三件 ROS-不标准, 必须经 `imu_reframe.py` 桥接**:
   - **单位**: Airy 输出 `linear_acceleration` 是 **g-units** (~0.99 g), ROS 标准是 m/s² (~9.81). 节点 `auto_scale_acc=true` 自动检测 < 5 时 ×9.80665
   - **mounting**: IMU 装反 (z 向下), LiDAR 朝上. 用 yaml extrinsic_R 的转置 = R_lidar_imu 把读数旋转到 LiDAR upright 系. 节点参数 `rotate_to_base_link=true`
   - **acc 符号约定**: rsasaki0109 内部用 `a_world = R @ a_body + g_world` (不是 textbook 的 `- g_world`), 必须 `negate_acc=true`. 静态时 acc 才能正确归零
   - frame_id 改成 `base_link` (避开 TF 二次旋转)

9. **`/initialpose` 类型必须 `PoseWithCovarianceStamped`** — 不是 PoseStamped. RViz "2D Pose Estimate" 默认就发这个. CLI 测试时用 `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped ...`.

10. **`set_initial_pose: true` 模式下 yaml 的 init_pose 只用一次, 运行时 /initialpose 被忽略**. 想自动重定位必须 `set_initial_pose: false`, 让 auto_init_pose 在 SC 匹配后再发 /initialpose.

11. **跨 bag GT init 必须用 scan-vs-prior ICP 推, 不能用 Procrustes-via-trajectory-files** (bag2-on-bag1 最大坑):
   - **教训**: 上次 session 用 Procrustes(bag1 `trajectory.pcd` ↔ bag2 `trajectory_in_bag1RAW.pcd`)
     算 bag2 t=0 init pose. mean residual 0 看着完美, 实际**错了 5m** (z 错 4.33m, yaw 错 20°).
     根因: 输入的 `trajectory_in_bag1RAW.pcd` 本身是用错的整体 ICP 算出来的, 错传进 Procrustes.
   - **正解**: 用 `scripts/derive_init_via_scan_icp.py`: 拿 bag2 第一帧 LiDAR scan, 跟 bag1 prior PCD 直接
     Open3D multi-scale ICP. 要求 fitness >0.9, inlier_rmse <0.5m 才算 OK. 输出 T_map_baselink
     的 (x,y,z, qx,qy,qz,qw) 粘进 yaml.
   - **症状**: init 错时 NDT 一直在拼命修正, 把 pose 拉到真实 z. 之前误判成 "NDT z-degeneracy / 6DoF
     局部最小", 加 z-filter prior / use_imu=false / threshold 调大 / local_map_crop 都治标. **根因就是 init 错**.
   - 一旦 init 对了 (scan-vs-prior ICP fitness 0.975 / inlier_rmse 0.36m), 上面那些复杂调参不需要,
     full prior + NDT + IMU 都能稳跟.

12. **跨 bag NDT IMU 副作用 (在 init 已对的前提下还要小心)**:
   - `use_imu: true` 跨 bag 可能让 deskew 把扫描扭歪 (Airy 外参未精校 ~5cm/帧). 同 bag 完美, 跨 bag 谨慎.
   - `imu_prediction_correction_guard_yaw_deg` 默认 4°, 跨 bag IMU 30s 累积漂 25° → 一次 NDT 大修正
     就把 IMU preintegration 永久禁掉. 想保留 IMU 放宽到 30°. 根因还是外参未精校.
   - `enable_local_map_crop: true`, `local_map_radius: 30.0` 跨 bag 强烈建议开. 不开 3.7M 点全图 NDT
     单帧 >10s, executor 卡死 IMU 灌不进来.

14. **vs-GT eval 揭穿了"看起来对"的定位** (2026-05-12 末发现, 未解决):
   - 合并 prior 跑通后 (fitness 0.994, z=-1m 真实地板, 86% 覆盖), 我以为定位就 OK 了.
     但用 bag2 self-trajectory.pcd 当 GT 做对比, **median 3D 误差 8m**, 95th 17m.
   - 形态: ICP 校正一次 → pose 拉回 1-2m → fast-lio 漂 2.5s → 又错 5-15m → ICP 再拉回. 反复.
   - 用户一句话点醒: "2.5s 飘 5m, 这是 odometry 问题". fast-lio 正常应该 cm/min 漂,
     2 m/s 漂移率 = 完全坏掉. 提高 ICP 频率治标不治本.
   - **下次接手要查的根因方向**: (1) fastlio_airy_for_loc.yaml extrinsic/IMU cov 是否准, (2) bag2
     mapping 当时 yaml 跟现在用的一不一样, (3) use_sim_time 是否传到 fastlio_mapping, (4)
     scan_bodyframe_pub_en:true 是否触发 fastlio 副作用, (5) 单独跑 fastlio 不接 open3d_loc 看
     /Odometry 是否同样漂.
   - **见 `fls_ws/src/open3d_loc/HANDOFF.md`** 完整诊断 + 接手计划.

13. **合并两次 mapping PCD 必须做 yaw sweep, 不能信任单次 ICP** (2026-05-12 大坑):
   - bag1 + bag2 各自 fastlio-mapping 出的世界 frame 起始 yaw 任意 (取决于操作员开始扫描时的朝向),
     两个 frame 之间可能差几十度 yaw 甚至 180° 左右.
   - **单次 whole-map ICP 几乎必定卡 yaw 局部最优** (走廊 / 矩形大堂这种重复几何尤其严重).
     上次 bag1↔bag2 单次 ICP 给 fitness 0.67, 看似合理, 实际是 yaw=0 stuck.
     甚至我一度认为是反射 (`diag(-1,-1,-1)` 给 fitness 0.73), 完全错了.
   - **正解**: ICP 之前 yaw sweep, 每 10°-30° 试一次, 取 fitness 最高的当 init, 然后 refine.
     bag1↔bag2 yaw sweep 发现真值 yaw=+210° (相当于 -150°), 一次跑出 fitness **0.994 / rmse 0.25m**.
   - 用户直觉很关键: 把两个 map 用不同颜色叠在 xy plot 上, 肉眼看墙体走向是否平行. 平行 = ICP 成功.
   - 实现见 `open3d_loc/scripts/merge_bag1_bag2_priors.py` (yaw sweep + 多尺度 ICP refine).
   - **症状**: 合并 prior 看似 OK 但 bag2 定位输出的 z 系统性 -4m 偏移 / 轨迹"穿墙". 这不是
     NDT z-degeneracy, 是 prior 本身就被合并错了, 把 bag2 投到 bag1 frame 的错位置.

### bag1-self / bag2-on-bag1 实测 (2026-05-11 ~ 12)
| 场景 (rsasaki NDT 栈) | init pose 来源 | /pcl_pose / score-over | 评价 |
|---|---|---|---|
| bag1-self (no IMU) | identity (同源) | 1821 / 0 | ✅ baseline |
| bag1-self + IMU | identity | dense / 0 | ✅ IMU 修复 verified |
| bag2 + IMU + SC auto-init | ScanContext (yaw 错 5.76°) | sparse / 1-11 | 中等 |
| bag2 + GT init (Procrustes 错的) + IMU + zfilt | **错的 init**, z 错 4m | 522 / 0 | ❌ 看似工作 |
| bag2 + GT init (scan-ICP) + IMU + crop30 + full prior | scan-vs-prior ICP | 245 / 0 | ✅ 算法跑通, 但 NDT 帧间噪声 0.8m, 轨迹"zigzag" |

| 场景 (open3d_loc 栈) | prior | 关键 fitness | bag2 z 中位 | 覆盖率 / 穿墙率 | 评价 |
|---|---|---|---|---|---|
| bag1-only prior | bag1 GlobalMap.pcd | 0.61 (live) | -5.5m | 20% / 70% | ❌ bag2 走 bag1 没看的区域 |
| merged (单 ICP, proper-rot init) | bag1+bag2 fitness 0.67 | 0.84 (live) | -5.7m | 49% (假高) / 43% | ❌ 合并 ICP 卡 yaw 局部最优 |
| merged (反射 seed) | bag1+bag2 fitness 0.73 | 0.89 (live) | -4.0m | 27% / 43% | ❌ 误以为是反射, 还是错 |
| **merged (yaw sweep + proper rot)** | bag1+bag2 fitness 0.994 | 0.74 (live) | **-0.98m** ✅ | **86% / 6.9%** | ✅ **最终方案**, z 在真实地板高度 |

### auto_init_pose 输出格式

### auto_init_pose 输出格式
ScanContext 匹配 bag1 KF index + delta_yaw, 节点取 `pose.json` 该 KF 的 LiDAR pose (kf.json 存的是 cloudKeyPoses6D 即 LiDAR-pose-in-world), 套上 delta_yaw Rz 修正, 再用 LiDAR-IMU 外参逆变换到 base_link pose (IMU pose), 发到 `/initialpose`. 实测 bag2 → bag1 匹配 KF 6, dist=0.06, delta_yaw=5.76°.

## 相关上游

- [hku-mars/FAST_LIO `ROS2`](https://github.com/hku-mars/FAST_LIO/tree/ROS2) - 官方 FAST-LIO ROS2
- [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) - 社区 ROS2 端口
- [rohrschacht/FAST_LIO_SLAM_ros2](https://github.com/rohrschacht/FAST_LIO_SLAM_ros2) - SAM/PGO ROS2 移植参考
- [TixiaoShan/LIO-SAM `ros2`](https://github.com/TixiaoShan/LIO-SAM/tree/ros2) - LIO-SAM 官方 ROS2
- [RuanJY/robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio) - Robosense PointXYZIRT 适配参考 (PR #11 借鉴了它的 Point 结构)
- [rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2) - 上机定位栈 (NDT_OMP + RViz /initialpose, ROS2 Humble)
- [rsasaki0109/ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) - NDT_OMP backend
- [gisbi-kim/scancontext-pybind](https://github.com/gisbi-kim/scancontext-pybind) - ScanContext++ Python 绑定 (auto_init_pose 用它做 place recognition)
