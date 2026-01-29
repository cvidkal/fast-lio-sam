//
// Created by xiang on 25-3-24.
//

#ifndef LIGHTNING_ROS_UTILS_H
#define LIGHTNING_ROS_UTILS_H

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

namespace lightning {

#ifdef ROS1
inline double ToSec(const ros::Time &time) { return time.toSec(); }
inline uint64_t ToNanoSec(const ros::Time &time) { return time.toNSec(); }
#else
inline double ToSec(const builtin_interfaces::msg::Time &time) { return double(time.sec) + 1e-9 * time.nanosec; }
inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time &time) { return time.sec * 1e9 + time.nanosec; }
#endif

}  // namespace lightning

#endif  // LIGHTNING_ROS_UTILS_H
