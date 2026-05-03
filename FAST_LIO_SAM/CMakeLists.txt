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

# ----- stage 3+ 才启用 -----
# find_package(pcl_conversions REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(PCL 1.10 REQUIRED COMPONENTS common io filters kdtree search)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(livox_ros_driver2 REQUIRED)
# find_package(GTSAM REQUIRED)             # SAM 模块需要, ros-humble-gtsam 没有官方包, 用 libgtsam-dev
# find_package(rosbag2_cpp REQUIRED)        # wrapper/bag_io 改造时启用

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
# 主可执行文件 (stage 2: 仅 placeholder)
#
#   stage 4 会替换为真正的 laserMapping 移植, 届时把
#   src/preprocess.cpp / src/laserMapping.cpp 等加进来.
# ============================================================
add_executable(fastlio_mapping
  src/laser_mapping_node_placeholder.cpp
)
ament_target_dependencies(fastlio_mapping
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
# 让 placeholder 也能用我们自己生成的接口 (Pose6D/SaveMap/SavePose), 验证生成链
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
