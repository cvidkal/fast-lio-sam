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
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters kdtree search)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenMP REQUIRED)              # FAST-LIO 在 preprocess + laserMapping 用了 omp_get_wtime / #pragma omp
# ----- stage 4 启用 -----
# find_package(GTSAM REQUIRED)             # SAM 模块, apt: ros-humble-gtsam
# find_package(rosbag2_cpp REQUIRED)       # wrapper/bag_io 改造时

# ============================================================
# rosidl 接口生成 (msg + srv)
#   保留 ROS1 时期相同字段, 只是包名/路径换成 ROS2
# ============================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Pose6D.msg
  srv/SaveMap.srv
  srv/SavePose.srv
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ============================================================
# stage 3: 把 preprocess + ImuProcess (header-only, compiled-via-placeholder)
#          编进可执行. laserMapping 真主体在 stage 4 加进来.
# ============================================================
add_executable(fastlio_mapping
  src/laser_mapping_node_placeholder.cpp
  src/preprocess.cpp
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
  pcl_conversions
  pcl_ros
  livox_ros_driver2
)
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# 让节点也能用我们自己生成的接口 (Pose6D/SaveMap/SavePose)
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

# launch / rviz_cfg 在 stage 5 落地后再 install
# install(DIRECTORY launch/      DESTINATION share/${PROJECT_NAME}/launch)
# install(DIRECTORY rviz_cfg/    DESTINATION share/${PROJECT_NAME}/rviz)

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
