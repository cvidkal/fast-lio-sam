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

- 2026-05-03: 阶段 1 完成（脚手架），后续阶段开 PR 时更新此节
