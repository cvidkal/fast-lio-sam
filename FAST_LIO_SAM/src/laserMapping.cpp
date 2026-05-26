// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// #include <Python.h>  // ROS2 port: matplotlibcpp 移除
#include <geometry_msgs/msg/vector3.hpp>
#include <ikd-Tree/ikd_Tree.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omp.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <so3_math.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <mutex>
#include <pcl/search/impl/search.hpp>
#include <thread>

#include "IMU_Processing.hpp"
#include "preprocess.h"

// gstam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// gnss
#include "GNSS_Processing.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// save map
#include "fast_lio_sam/srv/save_map.hpp"
#include "fast_lio_sam/srv/save_pose.hpp"

// save data in kitti format
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

// ROS2 port: wrapper/bag_io 依赖内部 lightning 框架 (IMUPtr / Vec3d 等),
// 暂时禁用. 离线回放用 `ros2 bag play --clock` 即可, 见 launch/*.launch.py.
// #include "bag_io.h"
#include <thread>

// using namespace gtsam;

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

// ROS2 tf2 broadcaster (在 main() 里用 node 初始化, publish_odometry 中使用).
std::unique_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN],
    s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
int data_mode = 0;  // 0: default, 1: rosbag, 2: live
string bag_path = "";
/**************************/

float res_last[100000] = {0.0};  // 残差，点到面距离平方和
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;
condition_variable sig_main_loop_finished;
mutex mtx_main_loop;
int main_loop_flag = 0;
int lidar_buffer_flag = 0;
// RosbagIO bag_io;       // 见 #include 注释, ROS2 port 屏蔽
std::thread* rosbag_thread = nullptr;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1,
    pcd_index = 0;
bool point_selected_surf[100000] = {0};  // 是否为平面特征点
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>> pointSearchInd_surf;
vector<BoxPointType> cub_needrm;  // ikd-tree中，地图需要移除的包围盒序列
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;                // 记录lidar时间
deque<PointCloudXYZI::Ptr> lidar_buffer;  // 记录特征提取或间隔采样后的lidar（特征）数据
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());   // 畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());  // 畸变纠正后降采样的单帧点云，w系
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));  // 特征点在地图中对应点的，局部平面参数,w系
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));  // 对应点法相量？
PointCloudXYZI::Ptr _featsArray;                                   // ikd-tree中，map需要移除的点云

pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap;   // 未使用

KD_TREE ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);  // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);   // R lidar to imu (imu = r * lidar + t)

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;  // 状态，噪声维度，输入
state_ikfom state_point;
vect3 pos_lid;  // world系下lidar坐标

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

/*back end*/
vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;  // 历史所有关键帧的角点集合（降采样）
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;    // 历史所有关键帧的平面点集合（降采样）

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());          // 历史关键帧位姿（位置）
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());  // 历史关键帧位姿
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D(
    new pcl::PointCloud<PointTypePose>());  //  存储fastlio 未优化的位姿
pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());  //  gnss 轨迹

// voxel filter paprams
float odometrySurfLeafSize;
float mappingCornerLeafSize;
float mappingSurfLeafSize;

float z_tollerance;
float rotation_tollerance;

// CPU Params
int numberOfCores = 4;
double mappingProcessInterval;

/*loop clousre*/
bool startFlag = true;
bool loopClosureEnableFlag;
float loopClosureFrequency;  //   回环检测频率
int surroundingKeyframeSize;
float historyKeyframeSearchRadius;    // 回环检测 radius kdtree搜索半径
float historyKeyframeSearchTimeDiff;  //  帧间时间阈值
int historyKeyframeSearchNum;         //   回环时多少个keyframe拼成submap
float historyKeyframeFitnessScore;    // icp 匹配阈值
bool potentialLoopFlag = false;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;  // loop history keyframe submap
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

bool aLoopIsClosed = false;
map<int, int> loopIndexContainer;  // from new to old
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

nav_msgs::msg::Path globalPath;

// 局部关键帧构建的map点云，对应kdtree，用于scan-to-map找相邻点
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

// 降采样
pcl::VoxelGrid<PointType> downSizeFilterCorner;
// pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterICP;
pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of scan-to-map optimization

float transformTobeMapped[6];  //  当前帧的位姿(world系下)

std::mutex mtx;
std::mutex mtxLoopInfo;

// Surrounding map
float surroundingkeyframeAddingDistThreshold;   //  判断是否为关键帧的距离阈值
float surroundingkeyframeAddingAngleThreshold;  //  判断是否为关键帧的角度阈值
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

// gtsam
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values optimizedEstimate;
gtsam::ISAM2* isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOptimizedGlobalMap;  // 最终优化地图

bool recontructKdTree = false;
int updateKdtreeCount = 0;         //  每100次更新一次
bool visulize_IkdtreeMap = false;  //  visual iktree submap

// gnss
double last_timestamp_gnss = -1.0;
deque<nav_msgs::msg::Odometry> gnss_buffer;
geometry_msgs::msg::PoseStamped msg_gnss_pose;
string gnss_topic;
bool useImuHeadingInitialization;
bool useGpsElevation;    //  是否使用gps高层优化
float gpsCovThreshold;   //   gps方向角和高度差的协方差阈值
float poseCovThreshold;  //  位姿协方差阈值  from isam2

M3D Gnss_R_wrt_Lidar(Eye3d);  // gnss  与 imu 的外参
V3D Gnss_T_wrt_Lidar(Zero3d);
bool gnss_inited = false;  //  是否完成gnss初始化
shared_ptr<GnssProcess> p_gnss(new GnssProcess());
GnssProcess gnss_data;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGnssPath;
nav_msgs::msg::Path gps_path;
vector<double> extrinT_Gnss2Lidar(3, 0.0);
vector<double> extrinR_Gnss2Lidar(9, 0.0);

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

// saveMap
rclcpp::Service<fast_lio_sam::srv::SaveMap>::SharedPtr srvSaveMap;
rclcpp::Service<fast_lio_sam::srv::SavePose>::SharedPtr srvSavePose;
bool savePCD;             // 是否保存地图
string savePCDDirectory;  // 保存路径

/**
 * 更新里程计轨迹
 */
void updatePath(const PointTypePose& pose_in) {
    string odometryFrame = "camera_init";
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(static_cast<int64_t>((pose_in.time) * 1e9), RCL_ROS_TIME);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf2::Quaternion q;
    q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

/**
 * 对点云cloudIn进行变换transformIn，返回结果点云， 修改liosam, 考虑到外参的表示
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                    PointTypePose* transformIn) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle
    // 转换到rotation_matrix 不合适，注释 Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x,
    // transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);  //  获取  body2lidar  外参
    T_b_lidar.pretranslate(state_point.offset_T_L_I);

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
                                                    transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b;  //   world2body
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d T_w_lidar = T_w_b * T_b_lidar;  //  T_w_lidar  转换矩阵

    Eigen::Isometry3d transCur = T_w_lidar;

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x =
            transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y =
            transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z =
            transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

/**
 * Eigen格式的位姿变换
 */
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) {
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch,
                                  thisPoint.yaw);
}

/**
 * Eigen格式的位姿变换
 */
Eigen::Affine3f trans2Affine3f(float transformIn[]) {
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1],
                                  transformIn[2]);
}

/**
 * 位姿格式变换
 */
PointTypePose trans2PointTypePose(float transformIn[]) {
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

/**
 * 发布thisCloud，返回thisCloud对应msg格式
 */
sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud,
                                      rclcpp::Time thisStamp, std::string thisFrame) {
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0) thisPub->publish(tempCloud);
    return tempCloud;
}

/**
 * 点到坐标系原点距离
 */
float pointDistance(PointType p) { return sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

/**
 * 两点之间距离
 */
float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

/**
 * 初始化
 */
void allocateMemory() {
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }
}

//  eulerAngle 2 Quaterniond
Eigen::Quaterniond EulerToQuat(float roll_, float pitch_, float yaw_) {
    Eigen::Quaterniond q;  //   四元数 q 和 -q 是相等的
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll;
    q.normalize();
    return q;
}

// 将更新的pose赋值到 transformTobeMapped
void getCurPose(state_ikfom cur_state) {
    //  欧拉角是没有群的性质，所以从SO3还是一般的rotation matrix 转换过来的结果一样
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2, 1, 0);  //  yaw pitch roll  单位：弧度
    // V3D eulerAngle  =  SO3ToEuler(cur_state.rot)/57.3 ;     //   fastlio 自带  roll pitch yaw  单位: 度，旋转顺序 zyx

    // transformTobeMapped[0] = eulerAngle(0);                //  roll     使用 SO3ToEuler 方法时，顺序是 rpy
    // transformTobeMapped[1] = eulerAngle(1);                //  pitch
    // transformTobeMapped[2] = eulerAngle(2);                //  yaw

    transformTobeMapped[0] = eulerAngle(2);     //  roll  使用 eulerAngles(2,1,0) 方法时，顺序是 ypr
    transformTobeMapped[1] = eulerAngle(1);     //  pitch
    transformTobeMapped[2] = eulerAngle(0);     //  yaw
    transformTobeMapped[3] = cur_state.pos(0);  //  x
    transformTobeMapped[4] = cur_state.pos(1);  //   y
    transformTobeMapped[5] = cur_state.pos(2);  // z
}

/**
 * rviz展示闭环边
 */
void visualizeLoopClosure() {
    rclcpp::Time timeLaserInfoStamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);  //  时间戳
    string odometryFrame = "camera_init";

    if (loopIndexContainer.empty()) return;

    visualization_msgs::msg::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // 闭环边
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::msg::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge->publish(markerArray);
}

/**
 * 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
 */
bool saveFrame() {
    if (cloudKeyPoses3D->points.empty()) return true;

    // 前一帧位姿
    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    // 当前帧位姿
    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);
    // Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4],
    // transformTobeMapped[5],
    //                                                     transformTobeMapped[0], transformTobeMapped[1],
    //                                                     transformTobeMapped[2]);

    // 位姿变换增量
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);  //  获取上一帧 相对 当前帧的 位姿

    // 旋转和平移量都较小，当前帧不设为关键帧
    if (abs(roll) < surroundingkeyframeAddingAngleThreshold && abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;
    return true;
}

/**
 * 添加激光里程计因子
 */
void addOdomFactor() {
    if (cloudKeyPoses3D->points.empty()) {
        // 第一帧初始化先验因子
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12)
                .finished());  // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2,
                               // 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    } else {
        // 添加激光里程计因子. xy 7cm std, z 7cm std (回到对称, v3 z 放松反而让 xy 也飘).
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-5, 1e-5, 1e-5, 5e-3, 5e-3, 5e-3).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());  /// pre
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                    // cur
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
                                                          poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

/**
 * 添加闭环因子
 */
void addLoopFactor() {
    if (loopIndexQueue.empty()) return;

    // 闭环队列
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first;  //   cur
        int indexTo = loopIndexQueue[i].second;   //    pre
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

/**
 * 添加GPS因子
 */
void addGPSFactor() {
    if (gnss_buffer.empty()) return;
    // 如果没有关键帧，或者首尾关键帧距离小于5m，不添加gps因子
    if (cloudKeyPoses3D->points.empty()) return;
    else {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0) return;
    }
    // 位姿协方差很小，没必要加入GPS数据进行校正
    if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold) return;
    static PointType lastGPSPoint;  // 最新的gps数据
    while (!gnss_buffer.empty()) {
        // 删除当前帧0.2s之前的里程计
        if (rclcpp::Time(gnss_buffer.front().header.stamp).seconds() < lidar_end_time - 0.05) {
            gnss_buffer.pop_front();
        }
        // 超过当前帧0.2s之后，退出
        else if (rclcpp::Time(gnss_buffer.front().header.stamp).seconds() > lidar_end_time + 0.05) {
            break;
        } else {
            nav_msgs::msg::Odometry thisGPS = gnss_buffer.front();
            gnss_buffer.pop_front();
            // GPS噪声协方差太大，不能用
            float noise_x = thisGPS.pose.covariance[0];  //  x 方向的协方差
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];  //   z(高层)方向的协方差
            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold) continue;
            // GPS里程计位置
            float gps_x = thisGPS.pose.pose.position.x;
            float gps_y = thisGPS.pose.pose.position.y;
            float gps_z = thisGPS.pose.pose.position.z;
            if (!useGpsElevation)  //  是否使用gps的高度
            {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
            }

            // (0,0,0)无效数据
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) continue;
            // 每隔5m添加一个GPS里程计
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0) continue;
            else
                lastGPSPoint = curGPSPoint;
            // 添加GPS因子
            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);
            aLoopIsClosed = true;
            RCLCPP_INFO(rclcpp::get_logger("fastlio_mapping"), "GPS Factor Added");
            break;
        }
    }
}

void saveKeyFramesAndFactor() {
    //  计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    if (saveFrame() == false) return;
    // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
    addOdomFactor();
    // GPS因子 (UTM -> WGS84)
    addGPSFactor();
    // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
    addLoopFactor();
    // 执行优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    if (aLoopIsClosed == true)  // 有回环因子，多update几次
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    // 优化结果
    isamCurrentEstimate = isam->calculateBestEstimate();
    // 当前帧位姿结果
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

    // cloudKeyPoses3D加入当前帧位置
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    // 索引
    thisPose3D.intensity = cloudKeyPoses3D->size();  //  使用intensity作为该帧点云的index
    cloudKeyPoses3D->push_back(thisPose3D);          //  新关键帧帧放入队列中

    // cloudKeyPoses6D加入当前帧位姿
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloudKeyPoses6D->push_back(thisPose6D);

    // 位姿协方差
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // ESKF状态和方差  更新
    state_ikfom state_updated = kf.get_x();  //  获取cur_pose (还没修正)
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(),
                        latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(),
                                       latestEstimate.rotation().yaw());

    //  更新状态量
    state_updated.pos = pos;
    state_updated.rot = q;
    state_point = state_updated;  // 对state_point进行更新，state_point可视化用到
    // if(aLoopIsClosed == true )
    kf.change_x(state_updated);  //  对cur_pose 进行isam2优化后的修正

    // TODO:  P的修正有待考察，按照yanliangwang的做法，修改了p，会跑飞
    // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P(); // 获取当前的状态估计的协方差矩阵
    // P_updated.setIdentity();
    // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // kf.change_P(P_updated);

    // 当前帧激光角点、平面点，降采样集合
    // pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    // pcl::copyPointCloud(*feats_undistort,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame);  // 存储关键帧,没有降采样的点云

    // 保存特征点降采样集合
    // cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    updatePath(thisPose6D);  //  可视化update后的path
}

void recontructIKdTree() {
    if (recontructKdTree && updateKdtreeCount > 0) {
        /*** if path is too large, the rvis will crash ***/
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kdtree查找最近一帧关键帧相邻的关键帧集合
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        mtx.lock();
        kdtreeGlobalMapPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMapPoses->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius,
                                           pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            subMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);  //  subMap的pose集合
        // 降采样
        pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
        downSizeFilterSubMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
                                                 globalMapVisualizationPoseDensity);  // for global map visualization
        downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
        downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);  //  subMap poses  downsample
        // 提取局部相邻关键帧对应的特征点云
        for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i) {
            // 距离过大
            if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) >
                globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
            // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
            // &cloudKeyPoses6D->points[thisKeyInd]);
            *subMapKeyFrames += *transformPointCloud(
                surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);  //  fast_lio only use  surfCloud
        }
        // 降采样，发布
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;  // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
                                                     globalMapVisualizationLeafSize);  // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

        std::cout << "subMapKeyFramesDS sizes  =  " << subMapKeyFramesDS->points.size() << std::endl;

        ikdtree.reconstruct(subMapKeyFramesDS->points);
        updateKdtreeCount = 0;
        RCLCPP_INFO(rclcpp::get_logger("fastlio_mapping"), "Reconstructed  ikdtree ");
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        std::cout << "featsFromMapNum  =  " << featsFromMapNum << "\t" << " kdtree_size_st   =  " << kdtree_size_st
                  << std::endl;
    }
    updateKdtreeCount++;
}

/**
 * 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
 */
void correctPoses() {
    if (cloudKeyPoses3D->points.empty()) return;

    if (aLoopIsClosed == true) {
        // 清空里程计轨迹
        globalPath.poses.clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i) {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
            updatePath(cloudKeyPoses6D->points[i]);
        }
        // 清空局部map， reconstruct  ikdtree submap
        recontructIKdTree();
        RCLCPP_INFO(rclcpp::get_logger("fastlio_mapping"), "ISMA2 Update");
        aLoopIsClosed = false;
    }
}

// 回环检测三大要素
//  1.设置最小时间差，太近没必要
//  2.控制回环的频率，避免频繁检测，每检测一次，就做一次等待
//  3.根据当前最小距离重新计算等待时间
bool detectLoopClosureDistance(int* latestID, int* closestID) {
    // 当前关键帧帧
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;  //  当前关键帧索引
    int loopKeyPre = -1;

    // 当前帧已经添加过闭环对应关系，不再继续添加
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) return false;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;                         //  候选关键帧索引
    std::vector<float> pointSearchSqDisLoop;                     //  候选关键帧距离
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);  //  历史帧构建kdtree
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop,
                                        pointSearchSqDisLoop, 0);
    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - lidar_end_time) > historyKeyframeSearchTimeDiff) {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    RCLCPP_INFO(rclcpp::get_logger("fastlio_mapping"), "Find loop clousre frame ");
    return true;
}

/**
 * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
 */
void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum) {
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    auto surfcloud_keyframes_size = surfCloudKeyFrames.size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) continue;

        if (keyNear < 0 || keyNear >= surfcloud_keyframes_size) continue;

        // *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear],
        // &copy_cloudKeyPoses6D->points[keyNear]); 注意：cloudKeyPoses6D 存储的是 T_w_b ,
        // 而点云是lidar系下的，构建icp的submap时，需要通过外参数T_b_lidar 转换 , 参考pointBodyToWorld 的转换
        *nearKeyframes += *transformPointCloud(
            surfCloudKeyFrames[keyNear],
            &copy_cloudKeyPoses6D->points[keyNear]);  //  fast-lio 没有进行特征提取，默认点云就是surf
    }

    if (nearKeyframes->empty()) return;

    // 降采样
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void performLoopClosure() {
    rclcpp::Time timeLaserInfoStamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);  //  时间戳
    string odometryFrame = "camera_init";

    if (cloudKeyPoses3D->points.empty() == true) {
        return;
    }

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // 当前关键帧索引，候选闭环匹配帧索引
    int loopKeyCur;
    int loopKeyPre;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) {
        return;
    }

    // 提取
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());  //  cue keyframe
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());  //   history keyframe submap
    {
        // 提取当前关键帧特征点集合，降采样
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);  //  将cur keyframe 转换到world系下
        // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
                              historyKeyframeSearchNum);  //  选取historyKeyframeSearchNum个keyframe拼成submap
        // 如果特征点较少，返回
        // if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        //     return;
        // 发布闭环匹配关键帧局部map
        if (pubHistoryKeyFrames->get_subscription_count() != 0)
            publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings.
    // 原本 setMaxCorrespondenceDistance(150) 是 giseop 留给大尺度场景的, 但在 FAST-LIO
    // 有 ~3m drift 时, 150m 让 ICP 把 cur 帧的点匹配到 prev 子图里完全错的对应,
    // 看上去 fitness OK 实际 align 偏 3m, 再加上 loop noise 用 fitness × 6 (= 31° 旋转
    // std), GTSAM 几乎忽略这种 loop, 出现重影.
    // 5m 经验值: 既能 cover 几米 drift, 又不会跨房间错配.
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(5.0);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // scan-to-map，调用icp匹配
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // 未收敛，或者匹配不够好
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) return;

    std::cout << "icp  success  " << std::endl;

    // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    if (pubIcpKeyFrames->get_subscription_count() != 0) {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // 闭环优化前当前帧位姿
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // 闭环优化后当前帧位姿
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);  //  获取上一帧 相对 当前帧的 位姿
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // 闭环匹配帧的位姿
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    // Loop noise: GTSAM Pose3 noise 顺序 (rx, ry, rz, x, y, z), 单位 rad²/m².
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    float trans_var = std::max(noiseScore, 1e-4f);  // floor 1cm std
    float rot_var   = 1e-6f;                        // ~0.057° std
    Vector6 << rot_var, rot_var, rot_var, trans_var, trans_var, trans_var;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    // 详细 log: 闭哪两个 KF + ICP 出的相对 z 差 + 时间差
    float dz_icp = correctionLidarFrame(2, 3);
    float dt_kf = (copy_cloudKeyPoses6D->points[loopKeyCur].time - copy_cloudKeyPoses6D->points[loopKeyPre].time);
    std::cout << "LOOP cur=" << loopKeyCur << " <-> pre=" << loopKeyPre
              << "  dt=" << dt_kf << "s"
              << "  fitness=" << noiseScore
              << "  dz_correction=" << dz_icp
              << "  curZ=" << copy_cloudKeyPoses6D->points[loopKeyCur].z
              << " preZ=" << copy_cloudKeyPoses6D->points[loopKeyPre].z
              << std::endl;

    // 添加闭环因子需要的数据
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    loopIndexContainer[loopKeyCur] = loopKeyPre;  //   使用hash map 存储回环对
}

// 回环检测线程
void loopClosureThread() {
    if (loopClosureEnableFlag == false) {
        std::cout << "loopClosureEnableFlag   ==  false " << endl;
        return;
    }

    rclcpp::Rate rate(loopClosureFrequency);  //   回环频率
    while (rclcpp::ok() && startFlag) {
        rate.sleep();
        performLoopClosure();    //  回环检测
        visualizeLoopClosure();  // rviz展示闭环边
    }
}

void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE* fp) {
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                             // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));     // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));     // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));        // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));        // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);  // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const* const pi, PointType* const po, state_ikfom& s) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// 按当前body(lidar)的状态，将局部点转换到世界系下
void pointBodyToWorld(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    // world <-- imu <-- lidar
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const* const pi, PointType* const po)  //  lidar2world
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void test_RGBpointBodyToWorld(PointType const* const pi, PointType* const po, Eigen::Vector3d pos,
                              Eigen::Matrix3d rotation)  //  lidar2world
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(rotation * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

// 根据lidar的FoV分割场景
BoxPointType LocalMap_Points;  // ikd-tree中,局部地图的包围盒角点
bool Localmap_Initialized = false;
void lasermap_fov_segment() {
    cub_needrm.clear();  // 清空需要移除的区域
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);  // X轴分界点转换到w系下
    V3D pos_LiD = pos_lid;                                // global系lidar位置

    // 初始化局部地图包围盒角点，以为w系下lidar位置为中心
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];  // 各个方向与局部地图边界的距离
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

        // 与某个方向上的边界距离太小，标记需要移除need_move
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }

    // 不需要移除则直接返回
    if (!need_move) return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;  // 新的局部地图角点
    float mov_dist =
        max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        // 与包围盒最小值角点距离
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);  // 移除较远包围盒
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
        RCLCPP_ERROR(rclcpp::get_logger("fastlio_mapping"), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    lidar_buffer_flag = 1;
    mtx_buffer.unlock();

    if (data_mode == 1) {
        sig_buffer.notify_all();
        std::unique_lock<std::mutex> lk(mtx_main_loop);
        sig_main_loop_finished.wait(lk, []() { return main_loop_flag == 1; });
    }
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;  // 标记是否已经进行了时间补偿
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
        RCLCPP_ERROR(rclcpp::get_logger("fastlio_mapping"), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;  //????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    // 特征提取或间隔采样
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);  // 储存处理后的lidar特征
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    lidar_buffer_flag = 1;
    mtx_buffer.unlock();
    if (data_mode == 1) {
        sig_buffer.notify_all();
        std::unique_lock<std::mutex> lk(mtx_main_loop);
        sig_main_loop_finished.wait(lk, []() { return main_loop_flag == 1; });
    }
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in) {
    publish_count++;
    sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));

    // lidar 和 imu时间差过大，且开启 时间同步, 纠正当前输入imu的时间
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        // 对输入imu时间，纠正为 时间差 + 原始时间
        const double corrected = timediff_lidar_wrt_imu + rclcpp::Time(msg_in->header.stamp).seconds();
        msg->header.stamp = rclcpp::Time(static_cast<int64_t>(corrected * 1e9), RCL_ROS_TIME);
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;  // update imu time

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
}

void gnss_cbk(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg_in) {
    //  RCLCPP_INFO(rclcpp::get_logger("fastlio_mapping"), "GNSS DATA IN ");
    double timestamp = rclcpp::Time(msg_in->header.stamp).seconds();

    mtx_buffer.lock();

    // 没有进行时间纠正
    if (timestamp < last_timestamp_gnss) {
        RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = rclcpp::Time(msg_in->header.stamp).seconds();
    gnss_data.status = msg_in->status.status;
    gnss_data.service = msg_in->status.service;
    gnss_data.pose_cov[0] = msg_in->position_covariance[0];
    gnss_data.pose_cov[1] = msg_in->position_covariance[4];
    gnss_data.pose_cov[2] = msg_in->position_covariance[8];

    mtx_buffer.unlock();

    if (!gnss_inited) {  //  初始化位置
        gnss_data.InitOriginPosition(msg_in->latitude, msg_in->longitude, msg_in->altitude);
        gnss_inited = true;
    } else {  //   初始化完成
        gnss_data.UpdateXYZ(msg_in->latitude, msg_in->longitude,
                            msg_in->altitude);  //  WGS84 -> ENU  ???  调试结果好像是 NED 北东地

        Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
        gnss_pose(0, 3) = gnss_data.local_N;   //    北
        gnss_pose(1, 3) = gnss_data.local_E;   //     东
        gnss_pose(2, 3) = -gnss_data.local_U;  //    地

        Eigen::Isometry3d gnss_to_lidar(Gnss_R_wrt_Lidar);
        gnss_to_lidar.pretranslate(Gnss_T_wrt_Lidar);
        gnss_pose = gnss_to_lidar * gnss_pose;  //  gnss 转到 lidar 系下, （当前Gnss_T_wrt_Lidar，只是一个大致的初值）

        nav_msgs::msg::Odometry gnss_data_enu;
        // add new message to buffer:
        gnss_data_enu.header.stamp = rclcpp::Time(static_cast<int64_t>((gnss_data.time) * 1e9), RCL_ROS_TIME);
        gnss_data_enu.pose.pose.position.x = gnss_pose(0, 3);  // gnss_data.local_E ;   北
        gnss_data_enu.pose.pose.position.y = gnss_pose(1, 3);  // gnss_data.local_N;    东
        gnss_data_enu.pose.pose.position.z = gnss_pose(2, 3);  //  地

        gnss_data_enu.pose.pose.orientation.x = geoQuat.x;  //  gnss 的姿态不可观，所以姿态只用于可视化，取自imu
        gnss_data_enu.pose.pose.orientation.y = geoQuat.y;
        gnss_data_enu.pose.pose.orientation.z = geoQuat.z;
        gnss_data_enu.pose.pose.orientation.w = geoQuat.w;

        gnss_data_enu.pose.covariance[0] = gnss_data.pose_cov[0];
        gnss_data_enu.pose.covariance[7] = gnss_data.pose_cov[1];
        gnss_data_enu.pose.covariance[14] = gnss_data.pose_cov[2];

        gnss_buffer.push_back(gnss_data_enu);

        // visial gnss path in rviz:
        msg_gnss_pose.header.frame_id = "camera_init";
        msg_gnss_pose.header.stamp = rclcpp::Time(static_cast<int64_t>((gnss_data.time) * 1e9), RCL_ROS_TIME);

        msg_gnss_pose.pose.position.x = gnss_pose(0, 3);
        msg_gnss_pose.pose.position.y = gnss_pose(1, 3);
        msg_gnss_pose.pose.position.z = gnss_pose(2, 3);

        gps_path.poses.push_back(msg_gnss_pose);

        //  save_gnss path
        PointTypePose thisPose6D;
        thisPose6D.x = msg_gnss_pose.pose.position.x;
        thisPose6D.y = msg_gnss_pose.pose.position.y;
        thisPose6D.z = msg_gnss_pose.pose.position.z;
        thisPose6D.intensity = 0;
        thisPose6D.roll = 0;
        thisPose6D.pitch = 0;
        thisPose6D.yaw = 0;
        thisPose6D.time = lidar_end_time;
        gnss_cloudKeyPoses6D->push_back(thisPose6D);
    }
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool sync_packages(MeasureGroup& meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();          // lidar指针指向最旧的lidar数据
        meas.lidar_beg_time = time_buffer.front();  // 记录最早时间

        // 更新结束时刻的时间
        if (meas.lidar->points.size() <= 1)  // time too little 时间太短，点数不足
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;  // 记录lidar结束时间为 起始时间 + 单帧扫描时间
            RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "Too few input point cloud!\n");
        } else if (meas.lidar->points.back().curvature / double(1000) <
                   0.5 * lidar_mean_scantime)  // 最后一个点的时间 小于 单帧扫描时间的一半
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;  // 记录lidar结束时间为 起始时间 + 单帧扫描时间
        } else {
            scan_num++;
            lidar_end_time =
                meas.lidar_beg_time + meas.lidar->points.back().curvature /
                                          double(1000);  // 结束时间设置为 起始时间 + 最后一个点的时间（相对）
            // 动态更新每帧lidar数据平均扫描时间
            lidar_mean_scantime +=
                (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();  // 最旧 IMU 时间
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))  // 记录 imu 数据, imu 时间小于当前帧 lidar 结束时间
    {
        imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());  // 记录当前lidar帧内的imu数据到meas.imu
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental() {
    PointVector PointToAdd;             // 需要加入到ikd-tree中的点云
    PointVector PointNoNeedDownsample;  // 加入ikd-tree时，不需要降采样的点云
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    // 根据点与所在包围盒中心点的距离，分类是否需要降采样
    for (int i = 0; i < feats_down_size; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector& points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);  // 当前点与box中心的距离

            // 判断最近点在x、y、z三个方向上，与中心的距离，判断是否加入时需要降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(
                    feats_down_world->points[i]);  // 若三个方向距离都大于地图珊格半轴长，无需降采样
                continue;
            }

            // 判断当前点的 NUM_MATCH_POINTS 个邻近点与包围盒中心的范围
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) <
                    dist)  // 如果邻近点到中心的距离 小于 当前点到中心的距离，则不需要添加当前点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);  // 加入点时需要降采样
    ikdtree.Add_Points(PointNoNeedDownsample, false);       // 加入点时不需要降采样
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull)  //    将稠密点云从 imu convert to  world
{
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull_body)  //   发布body系(imu)下的点云
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudMap) {
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T& out) {
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);  // rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    // ROS2 tf2 broadcaster: g_tf_broadcaster 是全局, 在 main() 里用 node 初始化
    geometry_msgs::msg::TransformStamped tfs;
    tfs.header = odomAftMapped.header;
    tfs.header.frame_id = "camera_init";
    tfs.child_frame_id = "body";
    tfs.transform.translation.x = odomAftMapped.pose.pose.position.x;
    tfs.transform.translation.y = odomAftMapped.pose.pose.position.y;
    tfs.transform.translation.z = odomAftMapped.pose.pose.position.z;
    tfs.transform.rotation = odomAftMapped.pose.pose.orientation;
    if (g_tf_broadcaster) g_tf_broadcaster->sendTransform(tfs);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pubPath) {
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);

        //  save  unoptimized pose
        V3D rot_ang(Log(state_point.rot.toRotationMatrix()));  //   旋转向量
        PointTypePose thisPose6D;
        thisPose6D.x = msg_body_pose.pose.position.x;
        thisPose6D.y = msg_body_pose.pose.position.y;
        thisPose6D.z = msg_body_pose.pose.position.z;
        thisPose6D.roll = rot_ang(0);
        thisPose6D.pitch = rot_ang(1);
        thisPose6D.yaw = rot_ang(2);
        fastlio_unoptimized_cloudKeyPoses6D->push_back(thisPose6D);
    }
}

void publish_path_update(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pubPath) {
    rclcpp::Time timeLaserInfoStamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);  //  时间戳
    string odometryFrame = "camera_init";
    if (pubPath->get_subscription_count() != 0) {
        /*** if path is too large, the rvis will crash ***/
        static int kkk = 0;
        kkk++;
        if (kkk % 10 == 0) {
            // path.poses.push_back(globalPath);
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
    }
}

//  发布gnss 轨迹
void publish_gnss_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pubPath) {
    gps_path.header.stamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    gps_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        pubPath->publish(gps_path);
    }
}

/*定义pose结构体*/
struct pose {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
};

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out);  //  使用std::ios::out 可实现覆盖
    if (!ofs) {
        std::cout << "open csv file error " << std::endl;
        return false;
    }
    return true;
}

/* write2txt   format  KITTI*/
void WriteText(std::ofstream& ofs, pose data) {
    ofs << std::fixed << data.R(0, 0) << " " << data.R(0, 1) << " " << data.R(0, 2) << " " << data.t[0] << " "
        << data.R(1, 0) << " " << data.R(1, 1) << " " << data.R(1, 2) << " " << data.t[1] << " " << data.R(2, 0) << " "
        << data.R(2, 1) << " " << data.R(2, 2) << " " << data.t[2] << std::endl;
}

void savePoseService(const std::shared_ptr<fast_lio_sam::srv::SavePose::Request> req, std::shared_ptr<fast_lio_sam::srv::SavePose::Response> res) {
    pose pose_gnss;
    pose pose_optimized;
    pose pose_without_optimized;

    std::ofstream file_pose_gnss;
    std::ofstream file_pose_optimized;
    std::ofstream file_pose_without_optimized;

    string savePoseDirectory;
    cout << "****************************************************" << endl;
    cout << "Saving poses to pose files ..." << endl;
    if (req->destination.empty()) savePoseDirectory = std::getenv("HOME") + savePCDDirectory;
    else
        savePoseDirectory = std::getenv("HOME") + req->destination;
    cout << "Save destination: " << savePoseDirectory << endl;

    // create file
    CreateFile(file_pose_gnss, savePoseDirectory + "/gnss_pose.txt");
    CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose.txt");
    CreateFile(file_pose_without_optimized, savePoseDirectory + "/without_optimized_pose.txt");

    //  save optimize data
    for (int i = 0; i < cloudKeyPoses6D->size(); i++) {
        pose_optimized.t =
            Eigen::Vector3d(cloudKeyPoses6D->points[i].x, cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z);
        pose_optimized.R = Exp(double(cloudKeyPoses6D->points[i].roll), double(cloudKeyPoses6D->points[i].pitch),
                               double(cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_optimized, pose_optimized);
    }
    cout << "Sucess global optimized  poses to pose files ..." << endl;

    for (int i = 0; i < fastlio_unoptimized_cloudKeyPoses6D->size(); i++) {
        pose_without_optimized.t = Eigen::Vector3d(fastlio_unoptimized_cloudKeyPoses6D->points[i].x,
                                                   fastlio_unoptimized_cloudKeyPoses6D->points[i].y,
                                                   fastlio_unoptimized_cloudKeyPoses6D->points[i].z);
        pose_without_optimized.R = Exp(double(fastlio_unoptimized_cloudKeyPoses6D->points[i].roll),
                                       double(fastlio_unoptimized_cloudKeyPoses6D->points[i].pitch),
                                       double(fastlio_unoptimized_cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_without_optimized, pose_without_optimized);
    }
    cout << "Sucess unoptimized  poses to pose files ..." << endl;

    for (int i = 0; i < gnss_cloudKeyPoses6D->size(); i++) {
        pose_gnss.t = Eigen::Vector3d(gnss_cloudKeyPoses6D->points[i].x, gnss_cloudKeyPoses6D->points[i].y,
                                      gnss_cloudKeyPoses6D->points[i].z);
        pose_gnss.R = Exp(double(gnss_cloudKeyPoses6D->points[i].roll), double(gnss_cloudKeyPoses6D->points[i].pitch),
                          double(gnss_cloudKeyPoses6D->points[i].yaw));
        WriteText(file_pose_gnss, pose_gnss);
    }
    cout << "Sucess gnss  poses to pose files ..." << endl;

    file_pose_gnss.close();
    file_pose_optimized.close();
    file_pose_without_optimized.close();
    res->success = true;
}

/**
 * 输出HBA格式的数据
 */
void outputHBA(const vector<pcl::PointCloud<PointType>::Ptr>& surfCloudKeyFrames,
               const pcl::PointCloud<PointTypePose>::Ptr& cloudKeyPoses6D, const string& saveMapDirectory) {
    // 创建pcd目录
    string pcdDir = saveMapDirectory + "/pcd";
    (void)system((string("mkdir -p ") + pcdDir).c_str());

    // 保存每个关键帧的点云
    for (size_t i = 0; i < surfCloudKeyFrames.size(); ++i) {
        string pcdFile = pcdDir + "/" + to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(pcdFile, *surfCloudKeyFrames[i]);
    }

    // 创建pose.json文件
    string poseFile = saveMapDirectory + "/pose.json";
    FILE* file = fopen(poseFile.c_str(), "w");
    if (!file) {
        cout << "Failed to open pose.json for writing" << endl;
        return;
    }

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
        // 获取body系下的位姿
        PointTypePose pose_body = cloudKeyPoses6D->points[i];

        // 构造T_w_b (world to body)
        Eigen::Affine3f T_w_b_ = pcl::getTransformation(pose_body.x, pose_body.y, pose_body.z, pose_body.roll,
                                                        pose_body.pitch, pose_body.yaw);
        Eigen::Isometry3d T_w_b;
        T_w_b.matrix() = T_w_b_.matrix().cast<double>();

        // 构造T_b_lidar (body to lidar)
        Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);
        T_b_lidar.pretranslate(state_point.offset_T_L_I);

        // 计算T_w_lidar = T_w_b * T_b_lidar
        Eigen::Isometry3d T_w_lidar = T_w_b * T_b_lidar;

        // 提取平移
        Eigen::Vector3d translation = T_w_lidar.translation();

        // 提取旋转并转换为四元数
        Eigen::Quaterniond quat(T_w_lidar.rotation());

        // 写入JSON格式
        fprintf(file, "%lf %lf %lf %lf %lf %lf %lf\n", translation.x(), translation.y(), translation.z(), quat.w(),
                quat.x(), quat.y(), quat.z());
    }
    fclose(file);

    cout << "HBA format output completed: " << surfCloudKeyFrames.size() << " PCD files and pose.json saved." << endl;
}

/**
 * 保存全局关键帧特征点集合
 */
void saveMapService(const std::shared_ptr<fast_lio_sam::srv::SaveMap::Request> req, std::shared_ptr<fast_lio_sam::srv::SaveMap::Response> res) {
    string saveMapDirectory;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    if (req->destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
    else
        saveMapDirectory = std::getenv("HOME") + req->destination;
    cout << "Save destination: " << saveMapDirectory << endl;
    // 这个代码太坑了！！注释掉
    //   int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    //   unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
    // 保存历史关键帧位姿
    pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);       // 关键帧位置
    pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);  // 关键帧位姿
    // 提取历史关键帧角点、平面点集合
    //   pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    //   pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

    // 注意：拼接地图时，keyframe是lidar系，而fastlio更新后的存到的cloudKeyPoses6D 关键帧位姿是body系下的，需要把
    // cloudKeyPoses6D  转换为T_world_lidar 。 T_world_lidar = T_world_body * T_body_lidar , T_body_lidar 是外参
    for (int i = 0; i < (int)cloudKeyPoses6D->size(); i++) {
        //   *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }

    if (req->resolution != 0) {
        cout << "\n\nSave resolution: " << req->resolution << endl;

        // 降采样
        // downSizeFilterCorner.setInputCloud(globalCornerCloud);
        // downSizeFilterCorner.setLeafSize(req->resolution, req->resolution, req->resolution);
        // downSizeFilterCorner.filter(*globalCornerCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
        // 降采样
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req->resolution, req->resolution, req->resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
    } else {
        //   downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);           //  稠密点云地图
    }

    // 保存到一起，全局关键帧特征点集合
    //   *globalMapCloud += *globalCornerCloud;
    *globalMapCloud += *globalSurfCloud;
    pcl::io::savePCDFileBinary(saveMapDirectory + "/filterGlobalMap.pcd", *globalSurfCloudDS);   //  滤波后地图
    int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);  //  稠密地图
    res->success = ret == 0;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;

    // visial optimize global map on viz
    rclcpp::Time timeLaserInfoStamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    string odometryFrame = "camera_init";
    publishCloud(pubOptimizedGlobalMap, globalSurfCloudDS, timeLaserInfoStamp, odometryFrame);

    // 输出HBA格式的数据
    outputHBA(surfCloudKeyFrames, cloudKeyPoses6D, saveMapDirectory);

    // res->success 已经在 ret==0 处赋好, 这里 void 收尾
}

void saveMap() {
    auto req = std::make_shared<fast_lio_sam::srv::SaveMap::Request>();
    auto res = std::make_shared<fast_lio_sam::srv::SaveMap::Response>();
    saveMapService(req, res);
    if (!res->success) {
        cout << "Fail to save map" << endl;
    }
}

/**
 * 发布局部关键帧map的特征点云
 */
void publishGlobalMap() {
    /*** if path is too large, the rvis will crash ***/
    rclcpp::Time timeLaserInfoStamp = rclcpp::Time(static_cast<int64_t>((lidar_end_time) * 1e9), RCL_ROS_TIME);
    string odometryFrame = "camera_init";
    if (pubLaserCloudSurround->get_subscription_count() == 0) return;

    if (cloudKeyPoses3D->points.empty() == true) return;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    ;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kdtree查找最近一帧关键帧相邻的关键帧集合
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap,
                                  pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
                                                globalMapVisualizationPoseDensity);  // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    // 提取局部相邻关键帧对应的特征点云
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
        // 距离过大
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
        // &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(
            surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);  //  fast_lio only use  surfCloud
    }
    // 降采样，发布
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;  // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
                                                 globalMapVisualizationLeafSize);  // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
}

// 构造H矩阵
void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)  // 判断每个点的对应邻域是否符合平面点的假设
    {
        PointType& point_body = feats_down_body->points[i];    // lidar系下坐标
        PointType& point_world = feats_down_world->points[i];  // lidar数据点在world系下坐标

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);                      // lidar系下坐标
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);  // w系下坐标
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto& points_near = Nearest_Points[i];

        if (ekfom_data.converge)  //  如果收敛了
        {
            /** Find the closest surfaces in the map **/
            // world系下从ikdtree找5个最近点用于平面拟合
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // 最近点数大于NUM_MATCH_POINTS，且最大距离小于等于5,point_selected_surf设置为true
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                     : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                  : true;
        }

        // 不符合平面特征
        if (!point_selected_surf[i]) continue;

        VF(4)
        pabcd;                           //  plane 参数  a b c d
        point_selected_surf[i] = false;  // 二次筛选平面点
        // 拟合局部平面，返回：是否有内点大于距离阈值
        if (esti_plane(pabcd, points_near, 0.1f)) {
            // plane distance
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s =
                1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());  // 筛选条件 1 - 0.9 * （点到平面距离 / 点到lidar原点距离）

            if (s > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;  // 以intensity记录点到面残差
                res_last[i] = abs(pd2);              // 残差，距离
            }
        }
    }

    effct_feat_num = 0;  // 有效匹配点数

    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];  // body系 平面特征点
            corr_normvect->points[effct_feat_num] = normvec->points[i];          // world系 平面参数
            total_residual += res_last[i];                                       // 残差和
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;  // 残差均值 （距离）
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  // 定义H维度
    ekfom_data.h.resize(effct_feat_num);                  // 有效方程个数

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType& laser_p = laserCloudOri->points[i];  // lidar系 平面特征点
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;  // 当前状态imu系下 点坐标
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);  // 当前状态imu系下 点坐标反对称矩阵

        /*** get the normal vector of closest surface/corner ***/
        const PointType& norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);  // 对应局部法相量, world系下

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);  // 将对应局部法相量旋转到imu系下 corr_normal_I
        V3D A(point_crossmat * C);            // 残差对角度求导系数 P(IMU)^ [R(imu <-- w) * normal_w]
        // 添加数据到矩阵
        if (extrinsic_est_en) {
            // B = lidar_p^ R(L <-- I) * corr_normal_I
            // B = lidar_p^ R(L <-- I) * R(I <-- W) * normal_W
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);  // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B),
                VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}
// livox的pointcloud2格式数据的回调函数,将数据引入buffer中
void livox_ros_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    mtx_buffer.lock();  // 加锁
    scan_count++;

    // 如果当前扫描的lidar数据比上一次早,则lidar数据有误,需要将lidar数据缓存队列清空
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
        RCLCPP_ERROR(rclcpp::get_logger("fastlio_mapping"), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();  // 记录最后一个雷达时间
    // 如果不需要进行时间同步,而imu时间戳和雷达时间戳相差大于10s,则输出错误信息
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }
    // time_sync_en为true时,当imu时间戳和雷达时间戳相差大于1s时,进行时间同步
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;  // ????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());
    p_pre->process(msg, ptr);                          // 点云预处理
    lidar_buffer.push_back(ptr);                       // 点云存入缓冲区
    time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());  // 时间存入缓冲区
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();  // 记录最后一个雷达时间
    lidar_buffer_flag = 1;
    mtx_buffer.unlock();  // 解锁
    if (data_mode == 1) {
        sig_buffer.notify_all();
        std::unique_lock<std::mutex> lk(mtx_main_loop);
        sig_main_loop_finished.wait(lk, []() { return main_loop_flag == 1; });
    }
}

int main(int argc, char** argv) {
    // allocateMemory();
    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("laserMapping");
    g_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    // ROS2 参数声明 + 读取小帮手
    auto declare_param = [&node](auto && name, auto && def, auto * dst) {
        using T = std::decay_t<decltype(def)>;
        node->template declare_parameter<T>(name, def);
        node->get_parameter(name, *dst);
    };
    auto declare_param_def = [&node](auto && name, auto && def) {
        using T = std::decay_t<decltype(def)>;
        node->template declare_parameter<T>(name, def);
        // 在 generic lambda 里 T 是 dependent type, 需要 template 消歧
        return node->get_parameter(name).template get_value<T>();
    };

    declare_param("publish.path_en",                path_en,            &path_en);
    declare_param("publish.scan_publish_en",        scan_pub_en,        &scan_pub_en);
    declare_param("publish.dense_publish_en",       dense_pub_en,       &dense_pub_en);
    declare_param("publish.scan_bodyframe_pub_en",  scan_body_pub_en,   &scan_body_pub_en);
    declare_param("max_iteration",                  4,                  &NUM_MAX_ITERATIONS);
    declare_param("map_file_path",                  std::string(""),    &map_file_path);
    declare_param("common.lid_topic",               std::string("/livox/lidar"), &lid_topic);
    declare_param("common.imu_topic",               std::string("/livox/imu"),   &imu_topic);
    declare_param("common.time_sync_en",            false,              &time_sync_en);
    declare_param("common.data_mode",               0,                  &data_mode);
    declare_param("common.bag_path",                std::string(""),    &bag_path);
    declare_param("filter_size_corner",             0.5,                &filter_size_corner_min);
    declare_param("filter_size_surf",               0.5,                &filter_size_surf_min);
    declare_param("filter_size_map",                0.5,                &filter_size_map_min);
    declare_param("cube_side_length",               200.0,              &cube_len);
    declare_param("mapping.det_range",              300.0f,             &DET_RANGE);
    declare_param("mapping.fov_degree",             180.0,              &fov_deg);
    declare_param("mapping.gyr_cov",                0.1,                &gyr_cov);
    declare_param("mapping.acc_cov",                0.1,                &acc_cov);
    declare_param("mapping.b_gyr_cov",              0.0001,             &b_gyr_cov);
    declare_param("mapping.b_acc_cov",              0.0001,             &b_acc_cov);
    declare_param("preprocess.blind",               0.01,               &p_pre->blind);
    declare_param("preprocess.lidar_type",          int(LIVOX),         &p_pre->lidar_type);
    declare_param("preprocess.livox_type",          int(LIVOX_CUS),     &p_pre->livox_type);
    declare_param("preprocess.scan_line",           16,                 &p_pre->N_SCANS);
    declare_param("preprocess.scan_rate",           10,                 &p_pre->SCAN_RATE);
    declare_param("point_filter_num",               2,                  &p_pre->point_filter_num);
    declare_param("feature_extract_enable",         false,              &p_pre->feature_enabled);
    declare_param("runtime_pos_log_enable",         false,              &runtime_pos_log);
    declare_param("mapping.extrinsic_est_en",       true,               &extrinsic_est_en);
    declare_param("pcd_save.pcd_save_en",           false,              &pcd_save_en);
    declare_param("pcd_save.interval",              -1,                 &pcd_save_interval);
    // 默认: 零平移 + 单位旋转, 防止跑空 ros2 run 时 [0..2] / [0..8] 越界 segfault
    extrinT = declare_param_def("mapping.extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
    extrinR = declare_param_def("mapping.extrinsic_R",
                                std::vector<double>{1.0, 0.0, 0.0,
                                                    0.0, 1.0, 0.0,
                                                    0.0, 0.0, 1.0});
    cout << "p_pre->lidar_type " << p_pre->lidar_type << endl;

    declare_param("odometrySurfLeafSize",           0.2f, &odometrySurfLeafSize);
    declare_param("mappingCornerLeafSize",          0.2f, &mappingCornerLeafSize);
    declare_param("mappingSurfLeafSize",            0.2f, &mappingSurfLeafSize);

    declare_param("z_tollerance",                   FLT_MAX, &z_tollerance);
    declare_param("rotation_tollerance",            FLT_MAX, &rotation_tollerance);

    declare_param("numberOfCores",                  2, &numberOfCores);
    declare_param("mappingProcessInterval",         0.15, &mappingProcessInterval);

    // save keyframes
    declare_param("surroundingkeyframeAddingDistThreshold",  20.0f, &surroundingkeyframeAddingDistThreshold);
    declare_param("surroundingkeyframeAddingAngleThreshold", 0.2f,  &surroundingkeyframeAddingAngleThreshold);
    declare_param("surroundingKeyframeDensity",              1.0f,  &surroundingKeyframeDensity);
    declare_param("surroundingKeyframeSearchRadius",         50.0f, &surroundingKeyframeSearchRadius);

    // loop closure
    declare_param("loopClosureEnableFlag",          false, &loopClosureEnableFlag);
    declare_param("loopClosureFrequency",           1.0f,  &loopClosureFrequency);
    declare_param("surroundingKeyframeSize",        50,    &surroundingKeyframeSize);
    declare_param("historyKeyframeSearchRadius",    10.0f, &historyKeyframeSearchRadius);
    declare_param("historyKeyframeSearchTimeDiff",  30.0f, &historyKeyframeSearchTimeDiff);
    declare_param("historyKeyframeSearchNum",       25,    &historyKeyframeSearchNum);
    declare_param("historyKeyframeFitnessScore",    0.3f,  &historyKeyframeFitnessScore);

    // gnss
    declare_param("common.gnss_topic",   std::string("/gps/fix"), &gnss_topic);
    extrinR_Gnss2Lidar = declare_param_def(
        "mapping.extrinR_Gnss2Lidar",
        std::vector<double>{1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0});
    extrinT_Gnss2Lidar = declare_param_def(
        "mapping.extrinT_Gnss2Lidar", std::vector<double>{0.0, 0.0, 0.0});
    declare_param("useImuHeadingInitialization", false, &useImuHeadingInitialization);
    declare_param("useGpsElevation",             false, &useGpsElevation);
    declare_param("gpsCovThreshold",             2.0f,  &gpsCovThreshold);
    declare_param("poseCovThreshold",            25.0f, &poseCovThreshold);

    // Visualization
    declare_param("globalMapVisualizationSearchRadius",  1e3f,  &globalMapVisualizationSearchRadius);
    declare_param("globalMapVisualizationPoseDensity",   10.0f, &globalMapVisualizationPoseDensity);
    declare_param("globalMapVisualizationLeafSize",      1.0f,  &globalMapVisualizationLeafSize);

    // visual ikdtree map
    declare_param("visulize_IkdtreeMap", false, &visulize_IkdtreeMap);

    // reconstruct ikdtree
    declare_param("recontructKdTree", false, &recontructKdTree);

    // savMap
    declare_param("savePCD",          false,                              &savePCD);
    declare_param("savePCDDirectory", std::string("/Downloads/LOAM/"),    &savePCDDirectory);

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    // downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(
        surroundingKeyframeDensity, surroundingKeyframeDensity,
        surroundingKeyframeDensity);  // for surrounding key poses of scan-to-map optimization

    // ISAM2参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    path.header.stamp = node->now();
    path.header.frame_id = "camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
                           aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));  // 重复？
    memset(res_last, -1000.0f, sizeof(res_last));

    // 设置imu和lidar外参和imu参数等
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));  // 加速度协方差
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // 设置gnss外参数
    Gnss_T_wrt_Lidar << VEC_FROM_ARRAY(extrinT_Gnss2Lidar);
    Gnss_R_wrt_Lidar << MAT_FROM_ARRAY(extrinR_Gnss2Lidar);

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    /// 初始化，其中h_share_model定义了·平面搜索和残差计算
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    bool debug_record = false;
    ofstream fout_pre, fout_out, fout_dbg;
    FILE* fp;
    if (debug_record) {
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out) cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;
    }

    /*** ROS subscribe initialization ***/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    if (data_mode != 1) {
        if (p_pre->lidar_type == LIVOX) {
            if (p_pre->livox_type == LIVOX_CUS) {
                sub_pcl_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    lid_topic, rclcpp::SensorDataQoS().keep_last(200000), livox_pcl_cbk);
            } else {
                sub_pcl_pc = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    lid_topic, rclcpp::SensorDataQoS().keep_last(200000), livox_ros_cbk);
            }
        } else {
            sub_pcl_pc = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::SensorDataQoS().keep_last(200000), standard_pcl_cbk);
        }
        sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS().keep_last(200000), imu_cbk);
    } else {
        // ROS2 port: data_mode==1 (内置 RosbagIO) 暂时禁用. 离线回放请用
        //   ros2 bag play --clock <bag_dir>
        // + 同时 launch 这个节点 use_sim_time:=true
        RCLCPP_FATAL(rclcpp::get_logger("fastlio_mapping"),
                     "data_mode=1 (built-in bag replay) is disabled in the ROS2 port. "
                     "Use 'ros2 bag play --clock <bag>' alongside this node instead.");
        return -1;
    }

    auto pubLaserCloudFull =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
    auto pubLaserCloudFull_body =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
    auto pubLaserCloudEffect =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);    // no used
    auto pubLaserCloudMap =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);         // no used
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
    auto pubPath          = node->create_publisher<nav_msgs::msg::Path>("/path", 100000);
    auto pubPathUpdate    = node->create_publisher<nav_msgs::msg::Path>("fast_lio_sam/path_update", 100000);

    pubGnssPath           = node->create_publisher<nav_msgs::msg::Path>("/gnss_path", 100000);
    pubLaserCloudSurround = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "fast_lio_sam/mapping/keyframe_submap", 1);
    pubOptimizedGlobalMap = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "fast_lio_sam/mapping/map_global_optimized", 1);

    // loop closure
    pubHistoryKeyFrames = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "fast_lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "fast_lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
    pubLoopConstraintEdge = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fast_lio_sam/mapping/loop_closure_constraints", 1);

    // gnss
    auto sub_gnss = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic, rclcpp::SensorDataQoS().keep_last(200000), gnss_cbk);

    // saveMap
    srvSaveMap  = node->create_service<fast_lio_sam::srv::SaveMap>("/save_map", &saveMapService);
    // savePose
    srvSavePose = node->create_service<fast_lio_sam::srv::SavePose>("/save_pose", &savePoseService);

    // 回环检测线程
    std::thread loopthread(&loopClosureThread);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    bool status = rclcpp::ok();
    while (status) {
        if (flg_exit) break;
        rclcpp::spin_some(node);

        if (data_mode == 1) {
            // 等待sig_buffer通知
            std::unique_lock<std::mutex> lk(mtx_buffer);
            sig_buffer.wait(lk, []() { return lidar_buffer_flag == 1 || flg_exit; });
            lidar_buffer_flag = 0;
            main_loop_flag = 0;
        }

        /// 在Measure内，储存当前lidar数据及lidar扫描时间内对应的imu数据序列
        if (sync_packages(Measures)) {
            // 第一帧lidar数据
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;  // 记录第一帧绝对时间
                p_imu->first_lidar_time = first_lidar_time;  // 记录第一帧绝对时间
                flg_first_scan = false;
                if (data_mode == 1) {
                    main_loop_flag = 1;
                    sig_main_loop_finished.notify_all();
                }
                continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            // 根据imu数据序列和lidar数据，向前传播纠正点云的畸变, 此前已经完成间隔采样或特征提取
            //  feats_undistort 为畸变纠正之后的点云,lidar系
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();                                                // 前向传播后body的状态预测值
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;  // global系 lidar位置

            if (feats_undistort->empty() || (feats_undistort == NULL)) {
                RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "No point, skip this scan!\n");
                if (data_mode == 1) {
                    main_loop_flag = 1;
                    sig_main_loop_finished.notify_all();
                }
                continue;
            }

            // 检查当前lidar数据时间，与最早lidar数据时间是否足够
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();  // 根据lidar在W系下的位置，重新确定局部地图的包围盒角点，移除远端的点

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();  // 当前帧降采样后点数

            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr) {
                if (feats_down_size > 5) {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++) {
                        pointBodyToWorld(&(feats_down_body->points[i]),
                                         &(feats_down_world->points[i]));  // point转到world系下
                    }
                    // world系下对当前帧降采样后的点云，初始化lkd-tree
                    ikdtree.Build(feats_down_world->points);
                }
                if (data_mode == 1) {
                    main_loop_flag = 1;
                    sig_main_loop_finished.notify_all();
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num:
            // "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5) {
                RCLCPP_WARN(rclcpp::get_logger("fastlio_mapping"), "No point, skip this scan!\n");
                main_loop_flag = 1;
                sig_main_loop_finished.notify_all();
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // lidar --> imu
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            if (debug_record) {
                fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose()
                         << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                         << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                         << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav
                         << endl;
            }

            if (visulize_IkdtreeMap)  // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                publish_map(pubLaserCloudMap);
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true;  //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);  // 预测、更新
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;  // world系下lidar坐标
            geoQuat.x = state_point.rot.coeffs()[0];                                 // world系下当前imu的姿态四元数
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            getCurPose(state_point);  //   更新transformTobeMapped
            /*back end*/
            // 1.计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
            // 2.添加激光里程计因子、GPS因子、闭环因子
            // 3.执行因子图优化
            // 4.得到当前帧优化后的位姿，位姿协方差
            // 5.添加cloudKeyPoses3D，cloudKeyPoses6D，更新transformTobeMapped，添加当前关键帧的角点、平面点集合
            saveKeyFramesAndFactor();
            // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹， 重构ikdtree
            correctPoses();
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en) {
                publish_path(pubPath);
                publish_gnss_path(pubGnssPath);      //   发布gnss轨迹
                publish_path_update(pubPathUpdate);  //   发布经过isam2优化后的路径
                static int jjj = 0;
                jjj++;
                if (jjj % 100 == 0) {
                    publishGlobalMap();  //  发布局部点云特征地图
                }
            }
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFull);           //   发布world系下的点云
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);  //  发布imu系下的点云

            // if(savePCD)  saveMap();

            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log) {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp =
                    aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve =
                    aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf(
                    "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave "
                    "ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
                    t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                    aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                if (debug_record) {
                    fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose()
                             << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                             << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                             << state_point.bg.transpose() << " " << state_point.ba.transpose() << " "
                             << state_point.grav << " " << feats_undistort->points.size() << endl;
                    dump_lio_state_to_log(fp);
                }
            }
        }

        status = rclcpp::ok();
        rate.sleep();

        if (data_mode == 1) {
            main_loop_flag = 1;
            sig_main_loop_finished.notify_all();
        }
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        string dir_name = string(ROOT_DIR) + "PCD/";
        if (!std::filesystem::exists(dir_name)) {
            std::filesystem::create_directories(dir_name);
        }
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << all_points_dir << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (debug_record) {
        fout_out.close();
        fout_pre.close();
        fout_dbg.close();
    }

    if (runtime_pos_log) {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE* fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2,
                "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, "
                "tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++) {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]),
                    s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]),
                    int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    startFlag = false;
    loopthread.join();  //  分离线程

    // ROS2 移植析构顺序修复:
    // 局部 `node` (line 2130) 在 main return 时先析构, 但下面这些全局 SharedPtr 持有
    // node 引用; 它们在 main return 之后才析构, 此时 node context 已死,
    // rmw 删 datawriter / service 时崩 (rmw_publish.cpp:62 / rmw_service.cpp:104, SIGSEGV).
    // 显式按"逆创建顺序"释放, 让 node 最后死.
    pubLoopConstraintEdge.reset();
    pubIcpKeyFrames.reset();
    pubHistoryKeyFrames.reset();
    pubRecentKeyFrame.reset();
    pubRecentKeyFrames.reset();
    pubCloudRegisteredRaw.reset();
    pubOptimizedGlobalMap.reset();
    pubLaserCloudSurround.reset();
    pubGnssPath.reset();
    srvSavePose.reset();
    srvSaveMap.reset();
    g_tf_broadcaster.reset();

    rclcpp::shutdown();
    return 0;
}
