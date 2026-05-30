#!/usr/bin/env python3
"""
init_pose_algo.py — 计算 bag2 在 bag1 prior map frame 下的初始 pose.

Pipeline:
  1. 读 bag (mcap / rosbag2) 前 N 秒的 /rslidar_points + /rslidar_imu_data
  2. 从 IMU acc 取低运动段平均, 解出 LiDAR 系下的 gravity vector
  3. 解 R_gravity, 把 LiDAR 系的 +z 转到真实 up. 这一步锁定 roll/pitch
  4. 累积前 N 帧 LiDAR 成密集 source 云, 应用 R_gravity 转到 gravity-aligned 系
  5. Voxel downsample source + target (prior map)
  6. Estimate normals + FPFH features
  7. 若 hint 提供: GICP 从 hint 局部精修
     若 hint 全零: RANSAC + FPFH 全局粗对齐 → GICP 精修
  8. 输出: 4x4 pose, fitness, RMSE
  9. (可选) 调 ROS2 /set_init_pose service 把 pose 写进 fastlio_mapping 的 IKF

依赖: open3d, numpy, rosbags (已装), pyyaml.

仅算法 + CLI, 不绑 ROS2 节点 (那是 Day 2c 的事).
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


# ============================================================
# Bag reading (rosbags package — pure Python, no ROS2 install needed at runtime)
# ============================================================
def read_bag_first_n_seconds(bag_path: Path, seconds: float,
                             lidar_topic: str = "/rslidar_points",
                             imu_topic: str = "/rslidar_imu_data"):
    """
    返回前 `seconds` 秒内的 (lidar_msgs, imu_msgs).
    每个 lidar_msg = (timestamp_ns, np.ndarray[N, 3] xyz, np.ndarray[N] timestamp_per_point_sec)
    每个 imu_msg   = (timestamp_ns, acc[3], gyro[3])  m/s^2 / rad/s
    """
    from rosbags.highlevel import AnyReader

    lidar = []
    imu = []
    t_start_ns = None

    with AnyReader([bag_path]) as reader:
        connections = list(reader.connections)
        topic_names = {c.topic for c in connections}
        if lidar_topic not in topic_names:
            raise RuntimeError(f"bag has no {lidar_topic}. topics: {sorted(topic_names)}")
        if imu_topic not in topic_names:
            raise RuntimeError(f"bag has no {imu_topic}. topics: {sorted(topic_names)}")

        wanted = [c for c in connections if c.topic in (lidar_topic, imu_topic)]
        for conn, ts_ns, raw in reader.messages(connections=wanted):
            if t_start_ns is None:
                t_start_ns = ts_ns
            if (ts_ns - t_start_ns) * 1e-9 > seconds:
                break
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == imu_topic:
                imu.append((ts_ns,
                            np.array([msg.linear_acceleration.x,
                                      msg.linear_acceleration.y,
                                      msg.linear_acceleration.z], dtype=np.float64),
                            np.array([msg.angular_velocity.x,
                                      msg.angular_velocity.y,
                                      msg.angular_velocity.z], dtype=np.float64)))
            elif conn.topic == lidar_topic:
                xyz, t_per_pt = _decode_pointcloud2(msg)
                lidar.append((ts_ns, xyz, t_per_pt))

    return lidar, imu


def _decode_pointcloud2(msg) -> tuple:
    """
    把 rosbags 反序列化出来的 PointCloud2 → (xyz[N,3], timestamp_per_point[N]).
    支持 rslidar_sdk v1.5+ PointXYZIRT (x,y,z,intensity,ring,timestamp double).
    intensity / ring 字段我们目前用不上.
    """
    # 取 fields 中的 x/y/z/timestamp offset + datatype
    field_map = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in field_map for k in ("x", "y", "z")):
        raise RuntimeError(f"PointCloud2 missing x/y/z. fields={list(field_map.keys())}")

    point_step = int(msg.point_step)
    width = int(msg.width)
    height = int(msg.height)
    n = width * height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float64)

    data = bytes(msg.data) if not isinstance(msg.data, (bytes, bytearray)) else msg.data
    arr = np.frombuffer(data, dtype=np.uint8).reshape(n, point_step)

    def slice_field(name, np_dtype, byte_size):
        off, _dt = field_map[name]
        return np.frombuffer(arr[:, off:off + byte_size].tobytes(), dtype=np_dtype)

    x = slice_field("x", np.float32, 4)
    y = slice_field("y", np.float32, 4)
    z = slice_field("z", np.float32, 4)
    xyz = np.stack([x, y, z], axis=1)

    if "timestamp" in field_map:
        off, _dt = field_map["timestamp"]
        # double timestamp = 8 bytes (rslidar_sdk new) — absolute ROS time per point
        t_per_pt = np.frombuffer(arr[:, off:off + 8].tobytes(), dtype=np.float64)
    elif "time" in field_map:
        off, _dt = field_map["time"]
        # float relative time per point (Velodyne / old rslidar) — relative to frame start
        t_per_pt = np.frombuffer(arr[:, off:off + 4].tobytes(), dtype=np.float32).astype(np.float64)
    else:
        t_per_pt = np.zeros(n, dtype=np.float64)

    # 过滤 NaN / 0 点
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.1)
    return xyz[valid].astype(np.float32), t_per_pt[valid]


# ============================================================
# Extrinsic loading (LiDAR-IMU, from airy_real_ext.yaml-style file)
# ============================================================
@dataclass
class Extrinsic:
    R_lidar_imu: np.ndarray  # 3x3, 把向量从 LiDAR 系转到 IMU 系: v_I = R * v_L
    t_lidar_imu: np.ndarray  # 3
    """
    YAML 字段 mapping.extrinsic_T / extrinsic_R 在 fastlio 里语义是 "IMU 系下 LiDAR 原点
    的位置 / LiDAR 系到 IMU 系的旋转". 见 IMU_Processing.hpp 的 set_extrinsic.
    """

    @staticmethod
    def from_yaml(p: Path) -> "Extrinsic":
        with open(p) as fp:
            d = yaml.safe_load(fp)
        # 兼容 fastlio yaml: /**:\n  ros__parameters:\n    mapping:\n      ...
        mp = d.get("/**", {}).get("ros__parameters", {}).get("mapping", {})
        if not mp:
            raise RuntimeError(f"{p}: no /**.ros__parameters.mapping section")
        R = np.array(mp["extrinsic_R"], dtype=np.float64).reshape(3, 3)
        T = np.array(mp["extrinsic_T"], dtype=np.float64).reshape(3)
        return Extrinsic(R_lidar_imu=R, t_lidar_imu=T)


# ============================================================
# IMU gravity estimation (-> R_gravity to align LiDAR z with world up)
# ============================================================
def estimate_gravity_in_lidar(imu_msgs, extrinsic: Extrinsic, gyro_thresh: float = 0.05):
    """
    输入 IMU 样本 (低运动段筛 ||gyro|| < gyro_thresh rad/s), 平均 acc -> g_imu (IMU 系下重力).
    转到 LiDAR 系: g_lidar = R_lidar_imu^T @ g_imu  (因为 R 是 lidar->imu).

    返回:
      g_lidar (3,), unit vector (向下方向)
      n_used (int): 用了几个样本
    """
    if not imu_msgs:
        raise RuntimeError("no IMU samples in window")
    accs = np.stack([m[1] for m in imu_msgs])   # (N, 3) m/s^2 in IMU frame
    gyros = np.stack([m[2] for m in imu_msgs])  # (N, 3) rad/s
    gyro_norm = np.linalg.norm(gyros, axis=1)
    mask = gyro_norm < gyro_thresh
    if mask.sum() < max(5, len(imu_msgs) // 4):
        # 退化: 大部分时间都在动. 用全部样本平均 (会有偏差, 用户应在 bag 开头静止)
        mask = np.ones(len(imu_msgs), dtype=bool)

    g_imu = -accs[mask].mean(axis=0)   # acc 读到的是 -g (重力反向), 翻号得 g
    # 兼容两种单位:
    #   - 标准 ROS sensor_msgs/Imu: m/s^2, ||g|| ~ 9.81
    #   - rslidar_sdk Airy IMU: g (gravitational units), ||g|| ~ 1.0
    # 算法只用 unit vector 的方向, 所以单位无所谓; 但偏差太大说明 bag 开头在剧烈运动.
    g_norm = np.linalg.norm(g_imu)
    if not (0.5 < g_norm < 1.5 or 8.0 < g_norm < 12.0):
        print(f"[warn] |g| in IMU frame = {g_norm:.4f} (expected ~1.0 g or ~9.81 m/s^2); "
              f"bag 开头可能不静止 / IMU 装反", file=sys.stderr)
    else:
        unit_hint = "g-units" if g_norm < 2.0 else "m/s^2"
        print(f"[gravity] |g_in_imu| = {g_norm:.4f} ({unit_hint})")
    # 转到 LiDAR 系
    g_lidar = extrinsic.R_lidar_imu.T @ g_imu
    return g_lidar / np.linalg.norm(g_lidar), int(mask.sum())


def rotation_from_two_vectors(v_from: np.ndarray, v_to: np.ndarray) -> np.ndarray:
    """Rodrigues: 求 R, 使 R @ v_from = v_to. v_from, v_to 应为单位向量."""
    v_from = v_from / np.linalg.norm(v_from)
    v_to = v_to / np.linalg.norm(v_to)
    cos_a = np.clip(np.dot(v_from, v_to), -1.0, 1.0)
    if cos_a > 1 - 1e-9:
        return np.eye(3)
    if cos_a < -1 + 1e-9:
        # 180°: 找个跟 v_from 不平行的轴
        axis = np.cross(v_from, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(v_from, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + 2 * K @ K
    axis = np.cross(v_from, v_to)
    sin_a = np.linalg.norm(axis)
    axis /= sin_a
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)


def gravity_align_rotation(g_lidar_unit: np.ndarray) -> np.ndarray:
    """
    返回 R_gl 使得: 点 p 在 gravity-aligned 系下 = R_gl @ p_in_lidar.
    要求: R_gl @ g_lidar = (0, 0, -1)  (重力指向 -z, 即 +z 向上).
    """
    return rotation_from_two_vectors(g_lidar_unit, np.array([0.0, 0.0, -1.0]))


# ============================================================
# Open3D registration: FPFH + RANSAC + GICP
# ============================================================
def _yaw_rotation(theta_rad: float) -> np.ndarray:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def register_yaw_grid_then_refine(
        source_xyz: np.ndarray, target_xyz: np.ndarray,
        voxel: float = 0.3,
        yaw_step_deg: float = 10.0,
        coarse_voxel: float = None,
        gicp_iter_coarse: int = 30,
        gicp_iter_fine: int = 120):
    """
    4-DOF 全局对齐: 假定 source 和 target 都已 gravity-aligned (+z = up).
    在 [0, 360) 范围内以 yaw_step_deg 步长扫 yaw, 每个 candidate:
       1. 用 yaw 旋转 source, 平移到 (target center - rotated source center)
       2. 在 coarse_voxel 上 GICP 评分
    取 fitness 最高的, 进 fine voxel GICP 精修.
    确定性 + 利用 gravity prior, 比 RANSAC+FPFH 在缺乏明显几何特征的场景更稳.
    """
    import open3d as o3d
    if coarse_voxel is None:
        coarse_voxel = voxel * 2.0

    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(source_xyz.astype(np.float64))
    tgt = o3d.geometry.PointCloud()
    tgt.points = o3d.utility.Vector3dVector(target_xyz.astype(np.float64))

    src_coarse = src.voxel_down_sample(coarse_voxel)
    tgt_coarse = tgt.voxel_down_sample(coarse_voxel)
    src_coarse.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=coarse_voxel * 2, max_nn=30))
    tgt_coarse.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=coarse_voxel * 2, max_nn=30))

    src_arr = np.asarray(src_coarse.points)
    tgt_arr = np.asarray(tgt_coarse.points)
    src_mean = src_arr.mean(axis=0)
    tgt_mean = tgt_arr.mean(axis=0)

    yaws = np.arange(0.0, 360.0, yaw_step_deg)
    best = None
    t0 = time.time()
    for yaw_deg in yaws:
        yaw_rad = math.radians(yaw_deg)
        Rz = _yaw_rotation(yaw_rad)
        # 用 mean alignment 给一个 reasonable translation init
        t_init = tgt_mean - Rz @ src_mean
        T_init = np.eye(4)
        T_init[:3, :3] = Rz
        T_init[:3, 3] = t_init

        try:
            r = o3d.pipelines.registration.registration_generalized_icp(
                src_coarse, tgt_coarse, max_correspondence_distance=coarse_voxel * 2.0,
                init=T_init,
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=gicp_iter_coarse),
            )
        except Exception:
            continue
        if best is None or r.fitness > best["fitness"]:
            best = {"T": np.asarray(r.transformation), "fitness": r.fitness,
                    "rmse": r.inlier_rmse, "yaw_deg": yaw_deg}
    t_coarse = time.time() - t0
    if best is None:
        return None
    print(f"[yaw grid] {len(yaws)} candidates @ voxel={coarse_voxel:.2f}m  best yaw_init={best['yaw_deg']:.0f}°  "
          f"fitness={best['fitness']:.4f} rmse={best['rmse']:.4f}  t={t_coarse:.2f}s")

    # Fine refinement
    src_fine = src.voxel_down_sample(voxel)
    tgt_fine = tgt.voxel_down_sample(voxel)
    src_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    tgt_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    t0 = time.time()
    r = o3d.pipelines.registration.registration_generalized_icp(
        src_fine, tgt_fine, max_correspondence_distance=voxel * 1.5,
        init=best["T"],
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=gicp_iter_fine),
    )
    t_fine = time.time() - t0
    print(f"[gicp fine] voxel={voxel:.2f}m  src={len(src_fine.points)} tgt={len(tgt_fine.points)}  "
          f"fitness={r.fitness:.4f} rmse={r.inlier_rmse:.4f}  t={t_fine:.2f}s")

    return {
        "T_target_source": np.asarray(r.transformation, dtype=np.float64),
        "fitness": float(r.fitness),
        "rmse": float(r.inlier_rmse),
        "method": "yaw_grid+gicp",
        "n_src_down": len(src_fine.points),
        "n_tgt_down": len(tgt_fine.points),
        "yaw_init_deg": float(best["yaw_deg"]),
    }


def register_global_then_refine(
        source_xyz: np.ndarray, target_xyz: np.ndarray,
        voxel: float = 0.3,
        hint_T: np.ndarray = None,
        hint_search_radius: float = 5.0,
        ransac_voxel: float = None,
        ransac_max_iter: int = 1000000,
        ransac_confidence: float = 0.999,
        rng_seed: int = 0):
    """
    返回 dict { 'T_target_source' (4x4), 'fitness', 'rmse', 'method', 'n_src_down', 'n_tgt_down', ... }.

    分两段:
      1. RANSAC+FPFH 在较大 voxel (ransac_voxel, 默认 = voxel*2) 上做全局粗对齐
      2. GICP 在 voxel 上做精修

    粗对齐用大 voxel 出于两个原因: (a) FPFH 描述子在大尺度上更稳, (b) RANSAC 搜索空间小.
    精修用小 voxel 保几何精度.
    """
    import open3d as o3d
    if ransac_voxel is None:
        ransac_voxel = voxel * 2.0

    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(source_xyz.astype(np.float64))
    tgt = o3d.geometry.PointCloud()
    tgt.points = o3d.utility.Vector3dVector(target_xyz.astype(np.float64))

    # 精修用 voxel
    src_fine = src.voxel_down_sample(voxel)
    tgt_fine = tgt.voxel_down_sample(voxel)
    src_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    tgt_fine.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))

    if hint_T is None:
        # 粗对齐用大 voxel
        src_ds = src.voxel_down_sample(ransac_voxel)
        tgt_ds = tgt.voxel_down_sample(ransac_voxel)
        src_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=ransac_voxel * 2, max_nn=30))
        tgt_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=ransac_voxel * 2, max_nn=30))

        fpfh_radius = ransac_voxel * 5
        src_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            src_ds, o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=100))
        tgt_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            tgt_ds, o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=100))

        t0 = time.time()
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_ds, tgt_ds, src_fpfh, tgt_fpfh, mutual_filter=True,
            max_correspondence_distance=ransac_voxel * 1.5,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(ransac_voxel * 1.5),
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                ransac_max_iter, ransac_confidence),
        )
        init_T = result.transformation
        ransac_t = time.time() - t0
        print(f"[ransac] voxel={ransac_voxel:.2f}m  src={len(src_ds.points)} tgt={len(tgt_ds.points)} fpfh_r={fpfh_radius:.2f}m  "
              f"fitness={result.fitness:.4f} rmse={result.inlier_rmse:.4f} t={ransac_t:.2f}s")
        if result.fitness < 0.05:
            print(f"[ransac] WARNING: low fitness, GICP refinement will likely fail. "
                  f"Consider giving a --hint or increasing --ransac-voxel.")
    else:
        init_T = hint_T
        print(f"[hint] using user-provided init, GICP only")

    # GICP refine (在精细 voxel 上)
    t0 = time.time()
    result_icp = o3d.pipelines.registration.registration_generalized_icp(
        src_fine, tgt_fine, max_correspondence_distance=voxel * 1.5,
        init=init_T,
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=120),
    )
    gicp_t = time.time() - t0
    print(f"[gicp ] voxel={voxel:.2f}m  src={len(src_fine.points)} tgt={len(tgt_fine.points)}  "
          f"fitness={result_icp.fitness:.4f} rmse={result_icp.inlier_rmse:.4f} t={gicp_t:.2f}s")

    return {
        "T_target_source": np.asarray(result_icp.transformation, dtype=np.float64),
        "fitness": float(result_icp.fitness),
        "rmse": float(result_icp.inlier_rmse),
        "method": "ransac+gicp" if hint_T is None else "hint+gicp",
        "n_src_down": len(src_fine.points),
        "n_tgt_down": len(tgt_fine.points),
    }


# ============================================================
# Helpers: 4x4 ↔ pos + quat (xyzw), yaml dump
# ============================================================
def matrix_to_pos_quat(T: np.ndarray) -> tuple:
    """4x4 -> (pos[3], quat_xyzw[4])"""
    R = T[:3, :3]
    t = T[:3, 3]
    # quat = R -> xyzw via scipy-free implementation
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = 2.0 * math.sqrt(tr + 1.0)
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return t.copy(), np.array([qx, qy, qz, qw])


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = q
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(qy*qy + qz*qz), s*(qx*qy - qz*qw),   s*(qx*qz + qy*qw)],
        [s*(qx*qy + qz*qw),     1 - s*(qx*qx + qz*qz), s*(qy*qz - qx*qw)],
        [s*(qx*qz - qy*qw),     s*(qy*qz + qx*qw),   1 - s*(qx*qx + qy*qy)],
    ])


# ============================================================
# Main pipeline
# ============================================================
def main():
    ap = argparse.ArgumentParser(description="Compute init pose for bag2 in bag1 prior map frame")
    ap.add_argument("--bag", type=Path, required=True, help="rosbag2 directory (mcap/sqlite3)")
    ap.add_argument("--prior", type=Path, required=True, help="prior map PCD (bag1 gravity-aligned recommended)")
    ap.add_argument("--extrinsic-yaml", type=Path, required=True, help="airy_real_ext.yaml style file for LiDAR-IMU R/T")
    ap.add_argument("--seconds", type=float, default=2.0, help="how many sec of bag head to use for init")
    ap.add_argument("--voxel", type=float, default=0.3, help="voxel size for downsample + FPFH radius scaling")
    ap.add_argument("--lidar-topic", default="/rslidar_points")
    ap.add_argument("--imu-topic", default="/rslidar_imu_data")
    ap.add_argument("--hint", default="",
                    help="optional initial guess as x,y,z,qx,qy,qz,qw (语义: IMU 在 prior frame 的 pose). 给了 hint 走 GICP only")
    ap.add_argument("--method", choices=["yaw_grid", "ransac"], default="yaw_grid",
                    help="global init method when no hint: yaw_grid (4DOF, deterministic, default) or ransac (6DOF FPFH+RANSAC)")
    ap.add_argument("--yaw-step-deg", type=float, default=10.0,
                    help="yaw grid step (deg). 10° = 36 candidates")
    ap.add_argument("--no-gravity", action="store_true", help="skip IMU gravity alignment (yaw_grid 必须有 gravity, 不能跟 --no-gravity 一起用)")
    ap.add_argument("--out", type=Path, default=None, help="output yaml path (default: print only)")
    ap.add_argument("--push-to-fastlio", action="store_true",
                    help="after computing, call ROS2 service /set_init_pose to inject into fastlio_mapping")
    ap.add_argument("--debug-dump", type=Path, default=None,
                    help="dir to dump source/target downsampled PCDs + transformed result for inspection")
    args = ap.parse_args()

    print(f"=== init_pose_algo ===")
    print(f"bag           : {args.bag}")
    print(f"prior         : {args.prior}")
    print(f"extrinsic_yaml: {args.extrinsic_yaml}")
    print(f"seconds head  : {args.seconds}")
    print(f"voxel         : {args.voxel}")

    ext = Extrinsic.from_yaml(args.extrinsic_yaml)
    print(f"R_lidar_imu det = {np.linalg.det(ext.R_lidar_imu):+.4f} (expect ±1)")

    # 1. read bag head
    t0 = time.time()
    lidar_msgs, imu_msgs = read_bag_first_n_seconds(
        args.bag, args.seconds, args.lidar_topic, args.imu_topic)
    print(f"[bag] {len(lidar_msgs)} lidar / {len(imu_msgs)} imu msgs in first {args.seconds}s "
          f"(read {time.time()-t0:.2f}s)")
    if not lidar_msgs:
        print("ERROR: no LiDAR frames", file=sys.stderr)
        sys.exit(1)

    # 2. IMU gravity
    if args.no_gravity:
        g_lidar = np.array([0.0, 0.0, -1.0])
        R_gravity = np.eye(3)
        print("[gravity] disabled (--no-gravity)")
    else:
        g_lidar, n_used = estimate_gravity_in_lidar(imu_msgs, ext)
        R_gravity = gravity_align_rotation(g_lidar)
        tilt_deg = math.degrees(math.acos(np.clip(-g_lidar[2], -1.0, 1.0)))
        print(f"[gravity] g_in_lidar = {g_lidar} (tilt vs LiDAR-z = {tilt_deg:.2f}°)"
              f", used {n_used}/{len(imu_msgs)} low-motion IMU samples")

    # 3. accumulate source cloud (gravity-aligned LiDAR frame)
    all_xyz = np.concatenate([m[1] for m in lidar_msgs], axis=0)
    print(f"[accum] {len(all_xyz)} raw points from {len(lidar_msgs)} frames")
    src_gravity_aligned = (R_gravity @ all_xyz.T).T.astype(np.float32)

    # 4. load prior
    t0 = time.time()
    import open3d as o3d
    prior_pcd = o3d.io.read_point_cloud(str(args.prior))
    prior_xyz = np.asarray(prior_pcd.points, dtype=np.float32)
    print(f"[prior] {len(prior_xyz)} points loaded ({time.time()-t0:.2f}s)")

    # ---- IMU<->LiDAR 变换 ----
    # fastlio 约定: state.pos/rot 是 IMU 在 world(prior_map) 的 pose.
    # 算法内部用 LiDAR 点云配准, 得 T_priormap_lidar, 最后要转 IMU pose 推给 /set_init_pose.
    # 关系 (见 laserMapping.cpp pointBodyToWorld):
    #   p_world = R_imu_world @ (R_L_I @ p_lidar + t_L_I) + t_imu_world
    # 其中 R_L_I = extrinsic_R, t_L_I = extrinsic_T (LiDAR -> IMU). 推:
    #   T_priormap_imu = T_priormap_lidar @ inv([R_L_I | t_L_I])
    T_imu_lidar = np.eye(4)
    T_imu_lidar[:3, :3] = ext.R_lidar_imu
    T_imu_lidar[:3, 3]  = ext.t_lidar_imu

    # 5. registration
    # hint 输入语义: IMU 在 prior frame 的 pose (与 service 接口一致).
    # 内部 registration 是 LiDAR 配准, hint 要转回 LiDAR-pose 形式, 且要在 gravity-aligned 系下表达.
    hint_T_for_reg = None
    if args.hint:
        parts = [float(x) for x in args.hint.split(",")]
        if len(parts) != 7:
            print("ERROR: --hint must be x,y,z,qx,qy,qz,qw (语义: IMU pose in prior frame)", file=sys.stderr)
            sys.exit(2)
        T_hint_imu = np.eye(4)
        T_hint_imu[:3, 3] = parts[:3]
        T_hint_imu[:3, :3] = quat_xyzw_to_matrix(np.array(parts[3:]))
        # T_priormap_imu  -> T_priormap_lidar  -> T_priormap_(gravity_aligned_lidar)
        T_hint_lidar = T_hint_imu @ T_imu_lidar
        T_ga_inv = np.eye(4); T_ga_inv[:3, :3] = R_gravity.T
        hint_T_for_reg = T_hint_lidar @ T_ga_inv

    if hint_T_for_reg is not None:
        result = register_global_then_refine(src_gravity_aligned, prior_xyz,
                                             voxel=args.voxel, hint_T=hint_T_for_reg)
    elif args.method == "yaw_grid":
        if args.no_gravity:
            print("ERROR: --method=yaw_grid requires gravity alignment, drop --no-gravity", file=sys.stderr)
            sys.exit(2)
        result = register_yaw_grid_then_refine(src_gravity_aligned, prior_xyz,
                                               voxel=args.voxel,
                                               yaw_step_deg=args.yaw_step_deg)
        if result is None:
            print("ERROR: yaw_grid registration failed (no candidates converged)", file=sys.stderr)
            sys.exit(3)
    else:  # ransac
        result = register_global_then_refine(src_gravity_aligned, prior_xyz,
                                             voxel=args.voxel)

    # 6. Compose: T_priormap_lidar = T_priormap_(gravity_aligned_lidar) @ T_(gravity_aligned_lidar)_lidar
    #    R_gravity 已经把 lidar 系转成 gravity-aligned 系, 所以从 lidar 系出发要先套 R_gravity
    T_ga_from_lidar = np.eye(4)
    T_ga_from_lidar[:3, :3] = R_gravity
    T_priormap_lidar = result["T_target_source"] @ T_ga_from_lidar

    # 7. LiDAR pose -> IMU pose (service / IKF 用的语义)
    T_final = T_priormap_lidar @ np.linalg.inv(T_imu_lidar)

    pos, quat = matrix_to_pos_quat(T_final)
    rpy = _rotmat_to_rpy(T_final[:3, :3])
    print(f"\n=== RESULT ===")
    print(f"method   : {result['method']}")
    print(f"fitness  : {result['fitness']:.4f}")
    print(f"rmse     : {result['rmse']:.4f}  m")
    print(f"src down : {result['n_src_down']} pts")
    print(f"tgt down : {result['n_tgt_down']} pts")
    print(f"pos (xyz)            : [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]")
    print(f"quat (xyzw)          : [{quat[0]:+.6f}, {quat[1]:+.6f}, {quat[2]:+.6f}, {quat[3]:+.6f}]")
    print(f"rpy deg (zyx convent): roll={math.degrees(rpy[0]):+.2f}  pitch={math.degrees(rpy[1]):+.2f}  yaw={math.degrees(rpy[2]):+.2f}")

    # 7. Output yaml
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w") as fp:
            yaml.safe_dump({
                "method": result["method"],
                "fitness": result["fitness"],
                "rmse": result["rmse"],
                "init_pos_xyz": pos.tolist(),
                "init_rot_quat": quat.tolist(),
                "T_4x4": T_final.tolist(),
                "rpy_deg": [math.degrees(r) for r in rpy],
                "voxel": args.voxel,
                "seconds": args.seconds,
            }, fp)
        print(f"wrote {args.out}")

    # 8. Optional debug dumps
    if args.debug_dump:
        args.debug_dump.mkdir(parents=True, exist_ok=True)
        # source in prior frame: 用 LiDAR-pose 变换 (T_priormap_lidar), 不是 IMU-pose
        src_xform = (T_priormap_lidar @ np.hstack([all_xyz, np.ones((len(all_xyz), 1))]).T).T[:, :3]
        _save_pcd(args.debug_dump / "source_raw.pcd", all_xyz)
        _save_pcd(args.debug_dump / "source_in_prior.pcd", src_xform)
        # downsampled too
        _save_pcd(args.debug_dump / "source_ds.pcd", np.asarray(o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(src_gravity_aligned.astype(np.float64))).voxel_down_sample(args.voxel).points))
        print(f"debug dumps in {args.debug_dump}")

    # 9. Optional push to fastlio
    if args.push_to_fastlio:
        ok = _push_to_fastlio(pos, quat, result["fitness"])
        if not ok:
            sys.exit(3)


def _rotmat_to_rpy(R: np.ndarray) -> tuple:
    """Z-Y-X intrinsic (yaw, pitch, roll), return (roll, pitch, yaw) in rad."""
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _save_pcd(path: Path, xyz: np.ndarray):
    import open3d as o3d
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    o3d.io.write_point_cloud(str(path), p)


def _push_to_fastlio(pos: np.ndarray, quat: np.ndarray, score: float) -> bool:
    """调 ROS2 /set_init_pose service. 仅 --push-to-fastlio 时用."""
    import subprocess
    req = (
        '{pose: {position: {x: %.6f, y: %.6f, z: %.6f}, '
        'orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}, score: %.4f}'
    ) % (pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3], score)
    cmd = ["ros2", "service", "call", "/set_init_pose", "fast_lio_sam/srv/SetInitPose", req]
    print(f"[push] $ {' '.join(cmd[:4])} <req>")
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except FileNotFoundError:
        print("ERROR: ros2 CLI not found. Source /opt/ros/humble/setup.bash first.", file=sys.stderr)
        return False
    print(r.stdout)
    if r.returncode != 0 or "success=True" not in r.stdout:
        print(f"[push] FAILED ({r.returncode}): {r.stderr}", file=sys.stderr)
        return False
    print("[push] /set_init_pose accepted, fastlio_mapping IKF updated")
    return True


if __name__ == "__main__":
    main()
