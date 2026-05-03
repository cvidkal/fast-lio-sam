// SPDX-License-Identifier: BSD-3-Clause
//
// fastlio_mapping placeholder (stage 2/6 of the ROS2 port).
//
// At this stage we just want to prove that:
//   - the ament_cmake build chain works
//   - rosidl_generate_interfaces wires Pose6D / SaveMap / SavePose correctly
//   - executable links against rclcpp + the generated typesupport
//
// In stage 4 this file gets replaced by the actual port of laserMapping.cpp.
// For now it spins an empty node so we can `ros2 run fast_lio_sam fastlio_mapping`
// and see the binary boots.

#include <chrono>

#include "rclcpp/rclcpp.hpp"

// 触发 typesupport 链接, 验证 rosidl 生成链工作
#include "fast_lio_sam/msg/pose6_d.hpp"
#include "fast_lio_sam/srv/save_map.hpp"
#include "fast_lio_sam/srv/save_pose.hpp"


class FastLioMappingPlaceholder : public rclcpp::Node {
 public:
  FastLioMappingPlaceholder() : rclcpp::Node("fastlio_mapping") {
    RCLCPP_INFO(get_logger(),
        "fast-lio-sam ROS2 port - stage 2/6 placeholder. "
        "See PORTING.md for roadmap.");
    // 验证生成接口确实可实例化
    auto pose = fast_lio_sam::msg::Pose6D{};
    (void)pose;
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastLioMappingPlaceholder>();
  RCLCPP_INFO(node->get_logger(),
      "TODO stage 4: replace with laserMapping port");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
