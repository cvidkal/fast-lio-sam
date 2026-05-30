#!/usr/bin/env python3
"""
init_pose_service.py — ROS2 节点, 提供 /relocalize 服务.

行为:
  - 启动时加载 prior map PCD (一次性)
  - 订阅 /rslidar_points + /rslidar_imu_data, 维护 ~3s 滑窗 buffer
  - service /relocalize 被调时:
      1. 从 buffer 取最近 N 秒数据
      2. 调用 init_pose_algo.compute_init_pose() (open3d FPFH+RANSAC+GICP + IMU gravity)
      3. 若 push_to_fastlio == true, 调 /set_init_pose service 把 pose 注入 fastlio_mapping
      4. 返回 PoseWithCovariance + fitness + rmse

参数 (ros2 args):
  --ros-args -p prior_map:=<pcd_path>
             -p extrinsic_yaml:=<airy_real_ext.yaml>
             -p window_seconds:=2.0
             -p voxel:=0.3
             -p lidar_topic:=/rslidar_points
             -p imu_topic:=/rslidar_imu_data
             -p gyro_threshold:=0.05

参考: Day 2c 计划. 算法核心见 init_pose_algo.py.
"""

import math
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu

# init_pose_algo 在同目录, 直接同进程 import
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import init_pose_algo as algo  # noqa: E402

# 用项目 srv
from fast_lio_sam.srv import Relocalize, SetInitPose


class InitPoseService(Node):
    def __init__(self):
        super().__init__("init_pose_service")

        # ---- Parameters ----
        self.declare_parameter("prior_map", "")
        self.declare_parameter("extrinsic_yaml", "")
        self.declare_parameter("window_seconds", 2.0)
        self.declare_parameter("voxel", 0.3)
        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter("imu_topic", "/rslidar_imu_data")
        self.declare_parameter("gyro_threshold", 0.05)
        self.declare_parameter("set_init_pose_service", "/set_init_pose")
        self.declare_parameter("ransac_max_iter", 100000)

        self.prior_map_path = self.get_parameter("prior_map").value
        self.extrinsic_yaml = self.get_parameter("extrinsic_yaml").value
        self.window_seconds = float(self.get_parameter("window_seconds").value)
        self.voxel = float(self.get_parameter("voxel").value)
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.gyro_threshold = float(self.get_parameter("gyro_threshold").value)
        self.set_pose_srv_name = self.get_parameter("set_init_pose_service").value
        self.ransac_max_iter = int(self.get_parameter("ransac_max_iter").value)

        if not self.prior_map_path or not Path(self.prior_map_path).exists():
            self.get_logger().error(f"prior_map missing or not found: {self.prior_map_path!r}")
            raise SystemExit(2)
        if not self.extrinsic_yaml or not Path(self.extrinsic_yaml).exists():
            self.get_logger().error(f"extrinsic_yaml missing or not found: {self.extrinsic_yaml!r}")
            raise SystemExit(2)

        # ---- Load extrinsic ----
        self.ext = algo.Extrinsic.from_yaml(Path(self.extrinsic_yaml))
        self.get_logger().info(
            f"extrinsic R det={np.linalg.det(self.ext.R_lidar_imu):+.4f}, t={self.ext.t_lidar_imu}")

        # ---- Load prior map (in main thread, blocking) ----
        t0 = time.time()
        import open3d as o3d
        self.prior_pcd = o3d.io.read_point_cloud(self.prior_map_path)
        self.prior_xyz = np.asarray(self.prior_pcd.points, dtype=np.float32)
        self.get_logger().info(
            f"prior_map loaded: {len(self.prior_xyz)} pts from {self.prior_map_path} ({time.time()-t0:.2f}s)")

        # ---- Buffers ----
        self._lidar_lock = threading.Lock()
        self._imu_lock = threading.Lock()
        self._lidar_buf: Deque[Tuple[float, np.ndarray]] = deque()    # (ts_sec, xyz[N,3])
        self._imu_buf: Deque[Tuple[float, np.ndarray, np.ndarray]] = deque()

        # ---- Subscribers (BEST_EFFORT — driver typically RELIABLE pub, BE sub matches OK) ----
        qos_be_keep_last_100 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=100)
        qos_be_keep_last_1000 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1000)

        self.sub_lidar = self.create_subscription(
            PointCloud2, self.lidar_topic, self._lidar_cb, qos_be_keep_last_100)
        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, qos_be_keep_last_1000)

        # ---- Service: /relocalize ----
        self.srv_relocalize = self.create_service(
            Relocalize, "/relocalize", self._relocalize_cb)

        # ---- Client: /set_init_pose ----
        self.cli_set_pose = self.create_client(SetInitPose, self.set_pose_srv_name)

        self.get_logger().info(
            f"ready. window={self.window_seconds}s voxel={self.voxel}m  "
            f"sub: {self.lidar_topic} {self.imu_topic}  "
            f"srv: /relocalize  client: {self.set_pose_srv_name}")

    # ---------------- subscriber callbacks ----------------
    def _lidar_cb(self, msg: PointCloud2):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            xyz, _ = algo._decode_pointcloud2(msg)
        except Exception as e:
            self.get_logger().warn(f"lidar decode failed: {e}")
            return
        with self._lidar_lock:
            self._lidar_buf.append((ts, xyz))
            # 老数据踢出 (保留 window_seconds + 1s margin)
            cutoff = ts - (self.window_seconds + 1.0)
            while self._lidar_buf and self._lidar_buf[0][0] < cutoff:
                self._lidar_buf.popleft()

    def _imu_cb(self, msg: Imu):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z], dtype=np.float64)
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z], dtype=np.float64)
        with self._imu_lock:
            self._imu_buf.append((ts, acc, gyro))
            cutoff = ts - (self.window_seconds + 1.0)
            while self._imu_buf and self._imu_buf[0][0] < cutoff:
                self._imu_buf.popleft()

    # ---------------- service callback ----------------
    def _relocalize_cb(self, req: Relocalize.Request, res: Relocalize.Response):
        t0 = time.time()
        self.get_logger().info(
            f"/relocalize called: hint=({req.hint.position.x:+.3f},{req.hint.position.y:+.3f},{req.hint.position.z:+.3f}) "
            f"search_radius={req.search_radius:.2f} use_imu_gravity={req.use_imu_gravity} push={req.push_to_fastlio}")

        # 1. snapshot buffers (under lock)
        with self._lidar_lock:
            lidar_snap = list(self._lidar_buf)
        with self._imu_lock:
            imu_snap = list(self._imu_buf)

        if not lidar_snap:
            res.success = False
            res.message = "no lidar in buffer (driver not publishing? or just started)"
            self.get_logger().error(res.message)
            return res
        if req.use_imu_gravity and not imu_snap:
            res.success = False
            res.message = "no imu in buffer (use_imu_gravity=true requires IMU samples)"
            self.get_logger().error(res.message)
            return res

        # 2. cut to window_seconds
        t_latest = lidar_snap[-1][0]
        t_start = t_latest - self.window_seconds
        lidar_window = [(ts, xyz) for (ts, xyz) in lidar_snap if ts >= t_start]
        imu_window = [(ts, a, g) for (ts, a, g) in imu_snap if ts >= t_start]
        self.get_logger().info(
            f"window: {len(lidar_window)} lidar + {len(imu_window)} imu over last {self.window_seconds:.2f}s")

        # 3. gravity
        if req.use_imu_gravity and imu_window:
            # adapt to existing algo.estimate_gravity_in_lidar API which expects (ts, acc, gyro) tuples
            try:
                g_lidar, n_used = algo.estimate_gravity_in_lidar(imu_window, self.ext, self.gyro_threshold)
            except Exception as e:
                res.success = False
                res.message = f"gravity estimation failed: {e}"
                self.get_logger().error(res.message)
                return res
            R_gravity = algo.gravity_align_rotation(g_lidar)
            tilt = math.degrees(math.acos(np.clip(-g_lidar[2], -1, 1)))
            self.get_logger().info(
                f"gravity tilt={tilt:.2f}°  ({n_used}/{len(imu_window)} low-motion samples)")
        else:
            R_gravity = np.eye(3)
            self.get_logger().info("gravity skipped (use_imu_gravity=false or no imu)")

        # 4. accumulate source cloud, gravity-align
        all_xyz = np.concatenate([m[1] for m in lidar_window], axis=0)
        src_ga = (R_gravity @ all_xyz.T).T.astype(np.float32)

        # IMU<->LiDAR 变换. /relocalize 输入 hint 与 /set_init_pose 输出 pose 都是
        # "IMU 在 prior frame 的 pose" 语义, 内部 registration 用 LiDAR 系, 需要转换.
        T_imu_lidar = np.eye(4)
        T_imu_lidar[:3, :3] = self.ext.R_lidar_imu
        T_imu_lidar[:3, 3]  = self.ext.t_lidar_imu

        # 5. parse hint (语义: IMU 在 prior frame 的 pose)
        hint_T_for_reg = None
        h = req.hint
        if abs(h.position.x) + abs(h.position.y) + abs(h.position.z) > 1e-9 or \
                abs(h.orientation.x) + abs(h.orientation.y) + abs(h.orientation.z) + abs(h.orientation.w - 1.0) > 1e-6:
            T_hint_imu = np.eye(4)
            T_hint_imu[:3, 3] = [h.position.x, h.position.y, h.position.z]
            T_hint_imu[:3, :3] = algo.quat_xyzw_to_matrix(np.array([
                h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w]))
            # IMU pose -> LiDAR pose -> gravity-aligned LiDAR pose (registration 接受这个)
            T_hint_lidar = T_hint_imu @ T_imu_lidar
            T_ga_inv = np.eye(4); T_ga_inv[:3, :3] = R_gravity.T
            hint_T_for_reg = T_hint_lidar @ T_ga_inv

        # 6. registration
        try:
            r = algo.register_global_then_refine(
                src_ga, self.prior_xyz,
                voxel=self.voxel, hint_T=hint_T_for_reg,
                ransac_max_iter=self.ransac_max_iter,
            )
        except Exception as e:
            res.success = False
            res.message = f"registration failed: {e}"
            self.get_logger().error(res.message)
            return res

        # 7. compose final pose: T_priormap_lidar -> T_priormap_imu
        T_ga_from_lidar = np.eye(4)
        T_ga_from_lidar[:3, :3] = R_gravity
        T_priormap_lidar = r["T_target_source"] @ T_ga_from_lidar
        T_final = T_priormap_lidar @ np.linalg.inv(T_imu_lidar)

        pos, quat = algo.matrix_to_pos_quat(T_final)

        # 8. fill response
        res.success = True
        res.pose.pose.position.x = float(pos[0])
        res.pose.pose.position.y = float(pos[1])
        res.pose.pose.position.z = float(pos[2])
        res.pose.pose.orientation.x = float(quat[0])
        res.pose.pose.orientation.y = float(quat[1])
        res.pose.pose.orientation.z = float(quat[2])
        res.pose.pose.orientation.w = float(quat[3])
        # 简单的 cov 估计: 用 rmse 作为 xyz σ, 用一个保守的角度 σ
        sigma_xy = max(r["rmse"], 0.05)
        sigma_z = max(r["rmse"], 0.05)
        sigma_rpy = math.radians(2.0)  # 默认 ~2° 不确定性
        cov = np.zeros(36)
        cov[0] = sigma_xy ** 2
        cov[7] = sigma_xy ** 2
        cov[14] = sigma_z ** 2
        cov[21] = sigma_rpy ** 2
        cov[28] = sigma_rpy ** 2
        cov[35] = sigma_rpy ** 2
        res.pose.covariance = cov.tolist()

        res.fitness = float(r["fitness"])
        res.rmse = float(r["rmse"])
        res.message = (
            f"{r['method']} fitness={r['fitness']:.3f} rmse={r['rmse']:.3f}m "
            f"src={r['n_src_down']} tgt={r['n_tgt_down']}  total_time={time.time()-t0:.2f}s")
        self.get_logger().info(f"/relocalize done: {res.message}")
        self.get_logger().info(
            f"  pos=[{pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}] "
            f"quat=[{quat[0]:+.4f},{quat[1]:+.4f},{quat[2]:+.4f},{quat[3]:+.4f}]")

        # 9. optionally push to fastlio_mapping via /set_init_pose
        if req.push_to_fastlio:
            push_ok = self._push_to_fastlio(pos, quat, r["fitness"])
            if not push_ok:
                res.success = False
                res.message += " (push_to_fastlio FAILED)"
        return res

    def _push_to_fastlio(self, pos, quat, score) -> bool:
        if not self.cli_set_pose.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{self.set_pose_srv_name} not available (fastlio_mapping not running?)")
            return False
        req = SetInitPose.Request()
        req.pose.position.x = float(pos[0])
        req.pose.position.y = float(pos[1])
        req.pose.position.z = float(pos[2])
        req.pose.orientation.x = float(quat[0])
        req.pose.orientation.y = float(quat[1])
        req.pose.orientation.z = float(quat[2])
        req.pose.orientation.w = float(quat[3])
        req.score = float(score)
        fut = self.cli_set_pose.call_async(req)
        # spin until done (we're inside a service cb but use a separate spin briefly)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().error(f"{self.set_pose_srv_name} timed out")
            return False
        r = fut.result()
        if not r.success:
            self.get_logger().error(f"{self.set_pose_srv_name} returned failure: {r.message}")
            return False
        self.get_logger().info(f"{self.set_pose_srv_name}: {r.message}")
        return True


def main():
    rclpy.init()
    node = InitPoseService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
