#!/usr/bin/env python3
"""
align_floor — FAST-LIO 输出 PCD/轨迹的 gravity 后校正.

为什么需要:
  FAST-LIO 在 IMU 初始化阶段用平均 acc 估计重力方向, 然后把世界系 z 轴
  对齐到这个方向. 如果 init 那 0.1 秒里 IMU 本身没水平 (机器狗站歪了 /
  采集人手持没端平), 整个建图坐标系就跟真实重力差几度.

  具体征兆:
    - GlobalMap.pcd 里地面拟合出的法向量与 +z 偏 1°+
    - 关键帧轨迹 z 与 (x, y) 有线性关系 (整体倾斜)
    - PCD 喂给 nav2 后地图看着 "歪"

修复思路:
  1) 拿 GlobalMap.pcd 里靠下的那部分点 (推测是地面)
  2) RANSAC 拟合一个平面, 取它的法向量 n
  3) 用 Rodrigues 公式构造把 n 旋到 +z 的旋转 R
  4) 把整云和轨迹都 @ R, 再平移让地面 z=0

输入: GlobalMap.pcd + trajectory.pcd (FAST-LIO-SAM 的 saveMap 输出)
输出: GlobalMap_aligned.pcd + trajectory_aligned.pcd + 4x4 变换矩阵 yaml

依赖: numpy + (没了, PCD 自己读 binary)

用法:
  python3 align_floor.py <input_dir>          # 输入目录里要有 GlobalMap.pcd
  python3 align_floor.py <input_dir> -o <out>
  python3 align_floor.py <input_dir> --floor-z-range -3.0 -2.0
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path

import numpy as np


# ============================================================
def read_pcd_binary(path: Path) -> tuple[list[str], np.ndarray]:
    """简易 PCD 读: 只支持 binary, FIELDS x y z intensity ..., float32."""
    with open(path, "rb") as f:
        fields, sizes = [], []
        while True:
            line = f.readline().decode("utf-8", errors="replace")
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(s) for s in line.split()[1:]]
            elif line.startswith("DATA"):
                if "binary" not in line:
                    raise ValueError(f"only DATA binary supported, got: {line.strip()}")
                break
        rec = sum(sizes)
        if any(s != 4 for s in sizes):
            raise ValueError(f"only float32 (size=4) supported, got sizes={sizes}")
        raw = f.read()
    n = len(raw) // rec
    arr = np.frombuffer(raw[: n * rec], dtype=np.float32).reshape(-1, rec // 4)
    return fields, arr


def write_pcd_binary(path: Path, fields: list[str], arr: np.ndarray) -> None:
    n = len(arr)
    nfields = len(fields)
    header = (
        f"# .PCD v0.7 - aligned by align_floor.py\n"
        f"VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(['4'] * nfields)}\n"
        f"TYPE {' '.join(['F'] * nfields)}\n"
        f"COUNT {' '.join(['1'] * nfields)}\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(arr.astype(np.float32).tobytes())


# ============================================================
def fit_plane_ransac(
    pts: np.ndarray,
    n_iter: int = 500,
    inlier_thresh: float = 0.05,
    seed: int = 42,
    min_normal_z: float = 0.7,
) -> tuple[np.ndarray, float, int]:
    """RANSAC 拟合平面, 返回 (法向 n, d, 内点数). n 已归一化, 默认指向 +z.

    min_normal_z: 只接受法向近竖直 (|n_z| >= 该值) 的平面. 防止在室内把竖直墙面
    当地面 — "最低 20% z" 的地面候选带里, 长墙的底部点往往比地面点还多, 纯"最大
    内点平面" RANSAC 会锁到墙 (法向水平), 再翻成 +z 得到 ~86° 的假地面, 把整张图
    旋歪. 地面残留倾斜一般只有几度, 默认 0.7 (允许到 45°) 极安全且能干净拒绝墙."""
    rng = np.random.default_rng(seed)
    best_inl = 0
    best = None
    for _ in range(n_iter):
        idx = rng.choice(len(pts), 3, replace=False)
        p1, p2, p3 = pts[idx]
        n = np.cross(p2 - p1, p3 - p1)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        if abs(float(n[2])) < min_normal_z:
            continue  # 不是近水平面 (地面), 跳过 — 拒绝墙/家具立面
        d = -n @ p1
        dist = np.abs(pts @ n + d)
        inl = (dist < inlier_thresh).sum()
        if inl > best_inl:
            best_inl = inl
            best = (n.copy(), float(d))
    if best is None:
        raise RuntimeError(
            f"RANSAC 找不到近水平地面 (|n_z| >= {min_normal_z}). "
            f"点云可能没 z-up, 或地面候选带里全是墙 — 试 --floor-z-range 手动缩小地面 z 区间."
        )
    n, d = best
    if n[2] < 0:
        n, d = -n, -d
    return n, d, best_inl


def rodrigues_align(n: np.ndarray, target: np.ndarray = np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """构造把 n 旋到 target 的 3x3 旋转矩阵."""
    v = np.cross(n, target)
    s = np.linalg.norm(v)
    c = float(n @ target)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s**2)


# ============================================================
# 多层场景检测 (基于轨迹 z, 不是空间点云 z)
# ============================================================
def detect_floor_layers(traj_z: np.ndarray, gap_threshold: float = 1.5) -> dict:
    """
    根据"采集设备走过的 z" 检测是否多层.

    思路: 把 keyframe 的 z 排序, 看相邻间隔. 一层楼内 z 是连续的 (走路时 ±0.5m
    起伏), 两层楼之间会有一段没走过的 z 区域 -> 排序 z 序列里出现一个大 gap.
    阈值默认 1.5m (典型一层楼 2.7m, 取一半作为安全裕度).

    比"空间点云 z 范围" 鲁棒得多:
      - 点云 z 包含天花板 / 家具上沿 / 楼梯井, 干扰多
      - 轨迹 z = 设备实际经过的高度, 自然反映楼层结构

    返回:
        {
          n_layers: int,
          layer_centers: [z, ...] (median of each cluster, sorted ascending),
          layer_z_ranges: [(min, max), ...],
          max_gap_m: float,          # 排序 z 序列里最大相邻间隔
          is_multi_floor: bool,
        }
    """
    z = np.sort(traj_z.astype(np.float64))
    if len(z) < 2:
        v = float(z[0]) if len(z) else 0.0
        return {
            "n_layers": 1,
            "layer_centers": [v],
            "layer_z_ranges": [(v, v)],
            "max_gap_m": 0.0,
            "is_multi_floor": False,
        }
    gaps = np.diff(z)
    big = np.where(gaps > gap_threshold)[0]
    splits = [0, *(big + 1).tolist(), len(z)]
    centers, ranges = [], []
    for a, b in zip(splits[:-1], splits[1:]):
        seg = z[a:b]
        centers.append(float(np.median(seg)))
        ranges.append((float(seg.min()), float(seg.max())))
    return {
        "n_layers": len(centers),
        "layer_centers": centers,
        "layer_z_ranges": ranges,
        "max_gap_m": float(gaps.max()),
        "is_multi_floor": len(centers) > 1,
    }


# ============================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input_dir", type=Path, help="包含 GlobalMap.pcd / trajectory.pcd 的目录")
    ap.add_argument("-o", "--output-dir", type=Path, default=None, help="输出目录 (默认 <input_dir>/aligned)")
    ap.add_argument(
        "--floor-z-range",
        type=float,
        nargs=2,
        default=None,
        metavar=("Z_MIN", "Z_MAX"),
        help="拟合地面的 z 范围 (默认: 自动取最低 20%% 的点)",
    )
    ap.add_argument("--ransac-thresh", type=float, default=0.05, help="RANSAC 内点距离阈值 (m, 默认 0.05)")
    ap.add_argument("--ransac-iter", type=int, default=500, help="RANSAC 迭代数 (默认 500)")
    ap.add_argument(
        "--floor-gap-threshold",
        type=float,
        default=1.5,
        help="轨迹 z 排序后相邻间隔 > 此值 -> 视为多层 (m, 默认 1.5)",
    )
    ap.add_argument(
        "--strict-single-floor",
        action="store_true",
        help="检测到多层时报错并退出 (默认仅 warn 后继续)",
    )
    args = ap.parse_args()

    in_dir: Path = args.input_dir
    out_dir: Path = args.output_dir or (in_dir / "aligned")
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1) 读云 + 轨迹
    cloud_path = in_dir / "GlobalMap.pcd"
    traj_path = in_dir / "trajectory.pcd"
    if not cloud_path.exists():
        print(f"[ERR] {cloud_path} 不存在 — 先 ros2 service call /save_map", file=sys.stderr)
        return 1
    print(f"[INFO] reading {cloud_path}")
    fields, arr = read_pcd_binary(cloud_path)
    xyz = arr[:, :3].astype(np.float64)
    print(f"       {len(xyz):,} 点, fields={fields}")

    traj_arr = None
    traj_fields = None
    if traj_path.exists():
        print(f"[INFO] reading {traj_path}")
        traj_fields, traj_arr = read_pcd_binary(traj_path)
        print(f"       {len(traj_arr):,} 关键帧")

        # 1.5) 多层检测 (基于轨迹 z, 不是空间点云 z)
        layers = detect_floor_layers(traj_arr[:, 2], gap_threshold=args.floor_gap_threshold)
        print(
            f"[INFO] 楼层检测: 轨迹 z 簇数 = {layers['n_layers']}, "
            f"max_gap = {layers['max_gap_m']:.2f}m"
        )
        if layers["is_multi_floor"]:
            msg = (
                "[WARN] 检测到多层场景:\n"
                f"       轨迹 z 序列里有 {layers['n_layers']} 个簇 (间隔 > "
                f"{args.floor_gap_threshold:.2f}m)\n"
                f"       簇中心 (z, m): "
                + ", ".join(f"{c:+.2f}" for c in layers["layer_centers"])
                + "\n"
                "       align_floor 当前只校一个重力方向 + 一个 z=0 基准.\n"
                "       结果: 重力 OK (重力跨层一致), 但 z=0 会绑到 RANSAC\n"
                "             点最多的那一层. 其他层的 z 不会归零.\n"
                "       多层正确做法: 按轨迹 z 簇切分, 每层独立处理 -- 暂未实现."
            )
            print(msg, file=sys.stderr)
            if args.strict_single_floor:
                print("[ERR] --strict-single-floor 拒绝继续", file=sys.stderr)
                return 2

    # 2) 选地面候选点
    if args.floor_z_range:
        z_lo, z_hi = args.floor_z_range
    else:
        z_sorted = np.sort(xyz[:, 2])
        z_lo = float(z_sorted[0])
        z_hi = float(z_sorted[len(z_sorted) // 5])  # 最低 20%
    gnd_mask = (xyz[:, 2] >= z_lo) & (xyz[:, 2] <= z_hi)
    gnd = xyz[gnd_mask]
    print(f"[INFO] 地面候选: z ∈ [{z_lo:+.2f}, {z_hi:+.2f}], {len(gnd):,} 点")

    # 3) RANSAC 平面
    n, d, inl = fit_plane_ransac(gnd, n_iter=args.ransac_iter, inlier_thresh=args.ransac_thresh)
    tilt_deg = np.degrees(np.arccos(np.clip(float(n[2]), -1.0, 1.0)))
    print(f"[INFO] 地面法向 n = [{n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f}]")
    print(f"       内点 {inl:,} / {len(gnd):,} ({100 * inl / len(gnd):.1f}%)")
    print(f"       与 +z 夹角 (重力倾斜): {tilt_deg:.3f}°")

    # 4) 构造旋转
    R = rodrigues_align(n)

    # 5) apply
    xyz_aligned = xyz @ R.T
    floor_z_aligned = float((gnd @ R.T)[:, 2].mean())
    xyz_aligned[:, 2] -= floor_z_aligned
    arr_out = arr.copy()
    arr_out[:, :3] = xyz_aligned.astype(np.float32)

    write_pcd_binary(out_dir / "GlobalMap.pcd", fields, arr_out)
    print(f"[OK] {out_dir / 'GlobalMap.pcd'}")

    if traj_arr is not None:
        traj_xyz = traj_arr[:, :3].astype(np.float64) @ R.T
        traj_xyz[:, 2] -= floor_z_aligned
        traj_out = traj_arr.copy()
        traj_out[:, :3] = traj_xyz.astype(np.float32)
        write_pcd_binary(out_dir / "trajectory.pcd", traj_fields, traj_out)
        print(f"[OK] {out_dir / 'trajectory.pcd'}")
        # 打印轨迹 z 是否变水平了
        z = traj_xyz[:, 2]
        print(f"[INFO] 轨迹 z (after align): mean={z.mean():+.3f}m std={z.std():.3f}m span={z.ptp():.3f}m")

    # 6) 输出变换 yaml
    T = np.eye(4)
    T[:3, :3] = R
    T[2, 3] = -floor_z_aligned  # post-rotate translation in z
    yaml_str = (
        "# align_floor.py 输出: aligned = R @ raw + t\n"
        f"# 拟合地面法向 (raw 系下) = [{n[0]:+.6f}, {n[1]:+.6f}, {n[2]:+.6f}]\n"
        f"# 重力倾斜角 = {tilt_deg:.4f}°\n"
        "rotation:\n"
    )
    for row in R:
        yaml_str += f"  - [{row[0]:+.9f}, {row[1]:+.9f}, {row[2]:+.9f}]\n"
    yaml_str += f"translation: [0.0, 0.0, {-floor_z_aligned:+.9f}]\n"
    (out_dir / "alignment.yaml").write_text(yaml_str)
    print(f"[OK] {out_dir / 'alignment.yaml'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
