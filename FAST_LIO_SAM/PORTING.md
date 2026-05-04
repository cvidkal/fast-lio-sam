# FAST-LIO-SAM ROS2 Humble 移植规划

> Tracking issue: #1

## 总目标

把 `fast-lio-sam` 从 ROS1 (Noetic, catkin, roscpp) 移植到 ROS2 (Humble, ament_cmake, rclcpp)，**保留 SAM/PGO 回环检测**这一核心能力，让它能直接用 `ros2 launch` 跑实时建图和 `ros2 bag play` 回放建图。

## 移植阶段拆分

每个阶段对应一个独立 commit (或一组小 commit)，方便 review：

| 阶段 | 内容 | 编译能过 | 单测能过 | PR 状态 |
|---|---|---|---|---|
| **1** | 脚手架 + 移植规划 + 把 ROS1 备份到 `legacy/ros1/` | ❌ 占位 | n/a | 当前 |
| **2** | `package.xml` (ament) + `CMakeLists.txt` (ament_cmake) + `Pose6D.msg` / `save_map.srv` / `save_pose.srv` 用 rosidl 生成 | ❌（未连源码） | n/a | 待 |
| **3** | `preprocess` + `IMU_Processing` + `common_lib` + `ikd-Tree` 转 rclcpp/纯 C++（前两者强相关 ROS2 类型，后两者基本无 ROS 依赖） | ❌（laserMapping 没动） | n/a | 待 |
| **4** | `laserMapping.cpp` 主节点（含 SAM/PGO） 转 rclcpp + tf2 + service | ⚠️ 需要 GTSAM | n/a | 待 |
| **5** | `*.launch` → `*.launch.py`，至少 velodyne16/airy 两个能起 | ✅ | ⚠️ 需要 bag | 待 |
| **6** | 烟测：跑 LIO-SAM 公开数据集 (rosbag2 sqlite3) → 出地图 | ✅ | ✅ | 待 |

## ROS1 → ROS2 接口对照表

| ROS1 概念 | ROS2 等价 | 备注 |
|---|---|---|
| `ros::init(argc, argv, name)` | `rclcpp::init(argc, argv)` + `auto node = rclcpp::Node::make_shared(name)` | |
| `ros::NodeHandle nh; nh.param("k", v, def)` | `node->declare_parameter("k", def); v = node->get_parameter("k").as_<T>()` | ROS2 必须 declare |
| `ros::Publisher pub = nh.advertise<T>("topic", q)` | `auto pub = node->create_publisher<T>("topic", q)` | |
| `pub.publish(msg)` | `pub->publish(msg)` | |
| `ros::Subscriber sub = nh.subscribe(...)` | `auto sub = node->create_subscription<T>("topic", q, cb)` | |
| `ros::ServiceServer srv = nh.advertiseService(...)` | `auto srv = node->create_service<T>("srv", cb)` | cb 签名变了 |
| `ros::Time::now()` | `node->now()` 或 `rclcpp::Clock().now()` | 注意 sim_time |
| `ros::Rate r(10); r.sleep()` | `rclcpp::Rate r(10); r.sleep()` | 几乎一样 |
| `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` | header 也换 |
| `tf::TransformListener` | `tf2_ros::Buffer + TransformListener` | API 大变 |
| `ros::spin()` | `rclcpp::spin(node)` | |
| `ROS_INFO("...")` | `RCLCPP_INFO(node->get_logger(), "...")` | |
| `<launch>` XML | `*.launch.py` Python | |
| `*.msg` / `*.srv` (catkin `message_generation`) | 同名文件 + `rosidl_default_generators` | 文件内容兼容 |
| `pcl_ros::transformPointCloud` | `pcl_ros::transformPointCloud` (ROS2 同名 API) | header 换 `pcl_ros/transforms.hpp` |
| `livox_ros_driver::CustomMsg` | `livox_ros_driver2::msg::CustomMsg` | API v2 |
| `nav_msgs::Path` | `nav_msgs::msg::Path` | 所有 msg 多一层 `::msg::` |

## 文件级移植清单

### 删 / 移 (legacy/ros1/)
- [x] `launch/*.launch` → `legacy/ros1/launch/`
- [x] `package.xml` (catkin 版) → `legacy/ros1/package.xml`
- [ ] `docker/Dockerfile` (基于 noetic) → `legacy/ros1/docker/Dockerfile` (后续如需做 ROS2 docker 再加)

### 重写 (ament + rclcpp)
- [ ] `package.xml` — ament_cmake build_type, depend 替换
- [ ] `CMakeLists.txt` — 新建，参照 hku-mars/FAST_LIO ROS2 分支
- [ ] `msg/Pose6D.msg` — 字段保持，加到 rosidl 生成
- [ ] `srv/save_map.srv`, `srv/save_pose.srv` — 同上

### 改造 (改头 / 改调用)
- [ ] `src/preprocess.h` — 替换 ros/livox 头，PCL 头不变
- [ ] `src/preprocess.cpp` — 同上 + Subscriber 写法、time 转换
- [ ] `src/IMU_Processing.hpp` — header-only，主要是 `ros::Time` → `rclcpp::Time`
- [ ] `src/laserMapping.cpp` — 大头，137 处 ROS1 调用，逐个翻
- [ ] `src/GNSS_Processing.hpp` — GeographicLib 不变，ROS msg 头换
- [ ] `wrapper/ros_utils.h`、`wrapper/bag_io.{cc,h}` — bag IO 重写为 rosbag2_cpp（如果保留这一层抽象）

### 不变 (纯 C++ / 第三方)
- `include/Exp_mat.h`, `so3_math.h`, `common_lib.h` — 逻辑层，可能小改
- `include/ikd-Tree/` — 第三方树结构，无 ROS 依赖
- `include/IKFoM_toolkit/` (如果有) — 同上

### 删
- [ ] `include/matplotlibcpp.h` — 调试用 Python 绑定，ROS2 移植期间不需要

## 关键决策

1. **保留 GTSAM 依赖** (SAM 模块需要)。`apt install ros-humble-gtsam` (实测 Humble 没有官方包，需要自己 apt install libgtsam-dev 或源码编译)。
2. **Livox 驱动用 livox_ros_driver2** (ROS2 版本)，不用 v1。
3. **bag 处理**：`rosbag2_cpp` API 用于读写 ROS2 bag (sqlite3 / mcap)；不打算同时支持 ROS1 bag 读写。
4. **保留 `legacy/ros1/`**：完整保留 ROS1 入口，不破坏现有 docker 用户，issue #1 close 后可以单独再考虑删。

## 参考实现

- [hku-mars/FAST_LIO `ROS2` 分支](https://github.com/hku-mars/FAST_LIO/tree/ROS2) — FAST-LIO 核心的官方 ROS2 端口，preprocess / IMU / laserMapping 三个模块的 rclcpp 写法可直接参考
- [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) — 社区版 FAST-LIO ROS2，CMakeLists / launch.py 写得规整
- [rohrschacht/FAST_LIO_SLAM_ros2](https://github.com/rohrschacht/FAST_LIO_SLAM_ros2) — FAST-LIO + Scan Context 的 ROS2 移植，**SAM 模块的 PGO 接口替换可直接借鉴**
- [TixiaoShan/LIO-SAM `ros2` 分支](https://github.com/TixiaoShan/LIO-SAM/tree/ros2) — LIO-SAM 官方 ROS2，GTSAM 接入参考

## 进度记录

- 2026-05-03: 阶段 1 完成（脚手架）
- 2026-05-03: 阶段 2 完成 (ament_cmake + rosidl + placeholder, colcon 47s 通过)
- 2026-05-03: 阶段 3 完成 (preprocess + IMU_Processing + GNSS + common_lib 转 rclcpp)

## 阶段 4 详细拆分 (laserMapping.cpp, 2620 LOC)

为了让 stage 4 的 PR 不变成无法 review 的巨型 diff, 拟分 4 个 sub-commit:

### 4a · sed pass (mechanical, 不可编译)
全部用 sed/perl 能做的批量替换. 已经在本 worktree 实验过, 共 236 行 diff. 内容:
- `<ros/ros.h>` → `<rclcpp/rclcpp.hpp>` 等所有 include 重写
- 所有 `sensor_msgs::Imu`/`PointCloud2`/`NavSatFix`/`nav_msgs::Odometry`/`Path`/`geometry_msgs::Vector3` 等 → `*::msg::*`
- `livox_ros_driver::CustomMsg` → `livox_ros_driver2::msg::CustomMsg`
- `ROS_INFO/WARN/ERROR/...` → `RCLCPP_*` (用 `rclcpp::get_logger("fastlio_mapping")`)
- `ros::Time().fromSec(x)` → `rclcpp::Time(static_cast<int64_t>(x*1e9), RCL_ROS_TIME)`
- `ros::Time` → `rclcpp::Time`, `ros::Rate` → `rclcpp::Rate`
- `ros::ok()/shutdown()` → `rclcpp::ok()/shutdown()`
- `header.stamp.toSec()` → `rclcpp::Time(...).seconds()`

### 4b · 全局 Publisher / Subscriber / Service 类型重写 (manual)
`ros::Publisher`/`ros::Subscriber`/`ros::ServiceServer` 在 ROS2 都是模板, 必须知道消息类型. 全局变量需要修改:

| 原 (ROS1) | 改成 (ROS2) | 备注 |
|---|---|---|
| `ros::Publisher pubX` | `rclcpp::Publisher<T>::SharedPtr pubX` | T 从 `nh.advertise<T>(..)` 推断 |
| `ros::Subscriber sub` | `rclcpp::Subscription<T>::SharedPtr sub` | T 从回调参数推断 |
| `ros::ServiceServer srvX` | `rclcpp::Service<T>::SharedPtr srvX` | T 从 srv 文件推断 |
| `pub.publish(msg)` | `pub->publish(msg)` | 替换 . → -> |

具体每个 publisher 的类型, 见 `nh.advertise<T>` 调用 (在 `main` 里都集中). 大致清单:
- `pubHistoryKeyFrames`, `pubIcpKeyFrames`, `pubRecentKeyFrames`, `pubRecentKeyFrame`, `pubCloudRegisteredRaw`, `pubLaserCloudSurround`, `pubOptimizedGlobalMap` → `sensor_msgs::msg::PointCloud2`
- `pubLoopConstraintEdge` → `visualization_msgs::msg::MarkerArray`
- `pubGnssPath`, `pubPathUpdate` → `nav_msgs::msg::Path`
- 函数参数 `const ros::Publisher&` → `const rclcpp::Publisher<T>::SharedPtr&` (按调用点的类型确定)

### 4c · main() / NodeHandle / 参数 重写 (manual, 重头戏)
在 `main()` 里集中改造:
1. `ros::init(argc, argv, "laserMapping")` + `ros::NodeHandle nh` → `rclcpp::init(argc, argv); auto node = rclcpp::Node::make_shared("laserMapping")`
2. **每一个 `nh.param<T>("k", v, def)`** → 必须 `node->declare_parameter<T>("k", def);` 然后 `v = node->get_parameter("k").as_<T>()`. 一共 ~40 处. 建议封装成 `template<typename T> T param(rclcpp::Node::SharedPtr&, const std::string&, const T&)` 减少 boilerplate
3. `nh.advertise<T>("topic", q)` → `node->create_publisher<T>("topic", q)`
4. `nh.subscribe("topic", q, cb)` → `node->create_subscription<T>("topic", q, cb)`
5. `nh.advertiseService("name", &fn)` → `node->create_service<T>("name", std::bind(fn, _1, _2))`. **service 回调签名变了**: ROS2 是 `void(const std::shared_ptr<Request>, std::shared_ptr<Response>)`, 不再返回 bool
6. `ros::spinOnce()` → `rclcpp::spin_some(node)`
7. `tf::TransformBroadcaster br;` → `tf2_ros::TransformBroadcaster br(node);`
8. `tf::createQuaternionMsgFromRollPitchYaw(...)` → `tf2::Quaternion q; q.setRPY(...);` 然后 `tf2::toMsg(q)`

### 4d · ikd_Tree + GTSAM + CMakeLists 整合, 试编译
1. 在 CMakeLists 里:
   - `find_package(GTSAM REQUIRED)` (apt: `ros-humble-gtsam`)
   - `add_executable` 加 `src/laserMapping.cpp` + `include/ikd-Tree/ikd_Tree.cpp`
   - `target_link_libraries` 加 `gtsam`
   - 把 `wrapper/bag_io.cc` 也编进去 (它已经有 `#ifdef ROS1` 双支持, 只要别 define ROS1 就走 ROS2 分支)
2. CMakeLists 的依赖也要加: `rosbag2_cpp`, `visualization_msgs`, `tf2_geometry_msgs`
3. `colcon build` 看错误, 逐个修
4. **典型错误预测**:
   - tf2 quaternion 转换的细节差异
   - `auto& msg = *msg_in` 这种取引用解 `ConstSharedPtr` 的写法可能要改
   - `pcl::fromROSMsg` 在 ROS2 里 header path 是 `pcl_conversions/pcl_conversions.h` (一致)
   - GTSAM 的 nav 模块在 Humble 版本可能少了某些 factor, 看运行时报错

## 阶段 5: launch.py
- `legacy/ros1/launch/mapping_velodyne16.launch` → `launch/mapping_velodyne16.launch.py`
- `legacy/ros1/launch/mapping_rs.launch` → `launch/mapping_airy.launch.py` (顺便 rename)
- 用 `Node(package='fast_lio_sam', executable='fastlio_mapping', parameters=[YAML])`
- RViz config `rviz_cfg/*.rviz` 大概率能直接用 (Display 类型在 ROS2 都向后兼容)

## 阶段 6: 烟测
- 下载一个 LIO-SAM 公开数据集 bag (`ros2 bag convert` 把 ROS1 .bag 转到 ROS2 sqlite3, 或者拿现成的)
- `ros2 launch fast_lio_sam mapping_velodyne16.launch.py bag:=<path>`
- 看是否出 odometry / 地图

## 估时

| 阶段 | 估时 |
|---|---|
| 4a (sed) | 已做, 5min |
| 4b (Pub/Sub/Srv 类型) | 1h |
| 4c (main/参数/服务回调) | 2-3h |
| 4d (CMake/编译/调试) | 1-3h, 强烈依赖 GTSAM 编译质量 |
| 5 (launch.py) | 30min |
| 6 (烟测) | 1h, 依赖 bag 是否能拿到 |


