cmake_minimum_required(VERSION 3.16)
project(fast_lio_sam VERSION 0.1.0 LANGUAGES C CXX)  # rosidl 会生成 C 代码

# ============================================================
# 编译器选项
# ============================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)              # ROS2 Humble 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 原 ROS1 代码用 ROOT_DIR 宏定位 Log/ 目录, 沿用同一约定.
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# ============================================================
# ament + rosidl
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# ----- stage 3 启用 -----
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters kdtree search registration)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenMP REQUIRED)
# ----- stage 4 启用 -----
find_package(GTSAM REQUIRED)              # SAM/PGO 模块 (apt: ros-humble-gtsam, 4.2+)
find_package(visualization_msgs REQUIRED) # 闭环 marker
find_package(rosbag2_cpp REQUIRED)        # wrapper/bag_io.cc 离线 bag 回放

# ============================================================
# rosidl 接口生成 (msg + srv)
#   保留 ROS1 时期相同字段, 只是包名/路径换成 ROS2
# ============================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Pose6D.msg
  srv/SaveMap.srv
  srv/SavePose.srv
  srv/SetInitPose.srv
  srv/Relocalize.srv
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ============================================================
# stage 4: 完整 fastlio_mapping
#   src/laserMapping.cpp + src/preprocess.cpp + include/ikd-Tree/ikd_Tree.cpp
#   wrapper/bag_io.cc (有 #ifdef ROS1 双支持, 我们走 ROS2 分支)
#
# 注意: 不再编译 laser_mapping_node_placeholder.cpp (它的活儿被
#       laserMapping.cpp 替代了)
# ============================================================
add_executable(fastlio_mapping
  src/laserMapping.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
  # wrapper/bag_io.cc        # 依赖内部 lightning 框架, ROS2 port 暂时不编;
                             # 离线回放走 `ros2 bag play --clock`
)
target_include_directories(fastlio_mapping PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(fastlio_mapping
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosbag2_cpp
)
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  gtsam
  Geographic   # apt: libgeographic-dev (WGS84 -> ENU 转换, 用于 GNSS_Processing)
)

# 让节点用上我们生成的接口 (Pose6D / SaveMap / SavePose)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fastlio_mapping ${cpp_typesupport_target})

# ============================================================
# 安装
# ============================================================
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)
install(DIRECTORY rviz_cfg/
  DESTINATION share/${PROJECT_NAME}/rviz_cfg
  OPTIONAL
)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  OPTIONAL
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
  OPTIONAL
)

# ============================================================
# 测试 (lint + 后续 gtest)
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint / cppcheck 在原 ROS1 代码里很多 warning, 暂时跳过
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
