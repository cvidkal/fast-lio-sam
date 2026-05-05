// SPDX-License-Identifier: BSD-3-Clause
//
// Preprocess: 把不同雷达 (Livox / Velodyne / Ouster / RoboSense) 输出的原始点云
// 整理成 FAST-LIO 的 PointCloudXYZI (curvature 字段塞相对时间, ms).
//
// 这是 ROS2 版本. 由 stage 3/6 (refs #1) 从 ROS1 移植.
#ifndef FAST_LIO_SAM_PREPROCESS_H_
#define FAST_LIO_SAM_PREPROCESS_H_

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>


#define IS_VALID(a) ((std::abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE {
    LIVOX = 1,
    VELO16,        // 2: Velodyne 16-line 兼容布局 (float relative time + ring)
    OUST64,        // 3: Ouster
    RS128,         // 4: 老 suteng_msgs 的 rslidar_ros::Point (float time, RELATIVE)
    RSLIDAR_NEW,   // 5: 现行 rslidar_sdk v1.5+ PointXYZIRT (double timestamp, ABSOLUTE)
                   //    覆盖 Helios / Airy / Bpearl / E1 / RS-LiDAR-M1 等
};
enum Feature {
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};  // 未判断, 可能平面, 平面, 跳跃边, 平面交接边, 细线
enum Surround { Prev, Next };
enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };  // 未判断, 接近0度, 接近180度, 接近远端, 接近近端

enum LIVOX_TYPE { LIVOX_CUS = 1, LIVOX_ROS, LIVOX_ROS_SKYLAND };

// 用于记录每个点的距离, 角度, 特征种类等属性
struct orgtype {
    double range;      // 平面距离
    double dista;      // 与后一个点的间距平方
    double angle[2];   // cos(当前点指向前一点或后一点的向量, ray)
    double intersect;  // 当前点与相邻两点的夹角 cos 值
    E_jump edj[2];     // 点前后两个方向的 edge_jump 类型
    Feature ftype;
    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

namespace livox_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;                  // 4D 点坐标 + 对齐填充
    float intensity;                  // Reflectivity
    uint8_t tag;                      // Livox point tag
    uint8_t line;                     // Laser line id
    uint8_t reflectivity;             // reflectivity, 0~255
    uint32_t offset_time;             // offset time relative to the base time
    PCL_ADD_RGB;                      // RGB
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 内存对齐
};
struct EIGEN_ALIGN16 PointSkyland {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livox_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint8_t, tag, tag)(std::uint8_t, line,
                                                              line)(std::uint8_t, reflectivity,
                                                                    reflectivity)(std::uint32_t, offset_time,
                                                                                  offset_time)(float, rgb, rgb))
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::PointSkyland,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, time, time)(std::uint16_t, ring,
                                                                                                        ring))

namespace rslidar_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          curvature)(float, time, normal_x)(std::uint16_t,
                                                                                                            ring, ring))

// 现行 rslidar_sdk v1.5+ 在 POINT_TYPE=XYZIRT 下输出的标准点结构.
// 字段命名/类型/语义和老的 rslidar_ros::Point 都不一样:
//   timestamp 是 double 绝对 UNIX 秒, ring 是 uint16.
// 覆盖 Helios / Airy / Bpearl / E1 / M1 / Ruby Plus 等所有现行 RoboSense 雷达.
namespace robosense_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (double, timestamp, timestamp))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on

/**
 * 6D 位姿点云结构定义.
 */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;


class Preprocess {
   public:
    Preprocess();
    ~Preprocess();

    // ---- ROS2 接口 ----
    void process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg,
                 PointCloudXYZI::Ptr &pcl_out);
    void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                 PointCloudXYZI::Ptr &pcl_out);
    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    // ---- 数据成员 ----
    PointCloudXYZI pl_full, pl_corn, pl_surf;       // 全部点 / 角点 / 面点
    // 上限 256 兼容 192 线 (Airy 是 96/192 线模式可切换).
    // 单帧最多分线后, 192 行就够了, 256 留一点冗余.
    static constexpr int kMaxScanLines = 256;
    PointCloudXYZI pl_buff[kMaxScanLines];
    std::vector<orgtype> typess[kMaxScanLines];
    int lidar_type;
    int livox_type;
    int point_filter_num;
    int N_SCANS;
    int SCAN_RATE;
    double blind;                                   // xy 平面距离, 小于此阈值不计算特征
    bool feature_enabled;
    bool given_offset_time;

   private:
    void avia_handler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg);
    void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void livox_ros_skyland_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void rs_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    // 现行 rslidar_sdk PointXYZIRT (double timestamp) 路径, 见 LID_TYPE::RSLIDAR_NEW
    void rslidar_new_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void give_feature(PointCloudXYZI &pl, std::vector<orgtype> &types);
    int plane_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, uint &i_nex,
                    Eigen::Vector3d &curr_direct);
    bool small_plane(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex,
                     Eigen::Vector3d &curr_direct);
    bool edge_jump_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, Surround nor_dir);

    int group_size;  // 计算平面特征时需要的最少局部点数
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};

#endif  // FAST_LIO_SAM_PREPROCESS_H_
