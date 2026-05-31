#!/usr/bin/env python3
"""
footprint_filter — per-scan 操作员自身排除.

跟 pcd_to_occgrid 里的 \"--footprint-radius\" (全局空间最近 KF 启发) 不同, 这个
直接在 LiDAR 局部系做几何切除:

    for 每个 keyframe i:
        raw_i = pcd/<i>.pcd            # LiDAR 系
        sensor_dist = sqrt(raw_i.x^2 + raw_i.y^2)
        keep = sensor_dist >= sensor_radius
        # 可选: 切掉 LiDAR 上方 (头 / 低天花板)
        if sensor_z_above is not None:
            keep &= (raw_i.z <= sensor_z_above)
        # 用 PGO 修正后的 6D 位姿 + IMU↔LiDAR 外参变换到世界系
        T_w_lidar = T_w_b(transformations.pcd[i]) @ T_b_lidar(extrinsic.yaml)
        world_i = T_w_lidar @ raw_i[keep]
        accumulate

输入:
  - SaveMap 输出目录 (含 pcd/, transformations.pcd)
  - LiDAR↔IMU 外参 yaml (跟 LIO config 同 schema, 取 mapping.extrinsic_T/R)

输出:
  - 一个 \"已经清掉操作员\" 的 GlobalMap.pcd, 可直接喂给 align_floor + pcd_to_occgrid

依赖: numpy.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path
from typing import Tuple

import numpy as np


# ============================================================
# Yaml 读 extrinsic_T / extrinsic_R (省得拉 PyYAML 依赖, 手写解析).
# 接受两种格式:
#   1. LIO config yaml (含 /**: ros__parameters: 包裹)
#   2. airy_extrinsic.py 输出 (mapping: extrinsic_T:..., extrinsic_R:...)
# ============================================================
def parse_extrinsic_yaml(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """返回 (T_b_lidar 3x3 R, 3x1 t)."""
    text = Path(path).read_text()
    # 找 extrinsic_T / extrinsic_R, 取第一处出现的就行
    R = None
    T = None
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("extrinsic_T") and ":" in line:
            # 可能在同一行 [a, b, c], 也可能下面几行
            after = line.split(":", 1)[1].strip()
            T = _parse_floats(after, lines, i)
        elif line.startswith("extrinsic_R") and ":" in line:
            after = line.split(":", 1)[1].strip()
            R = _parse_floats(after, lines, i)
        if T is not None and R is not None:
            break
        i += 1
    if R is None or T is None:
        raise ValueError(f"yaml 里没找到 extrinsic_T / extrinsic_R: {path}")
    if len(T) != 3 or len(R) != 9:
        raise ValueError(f"extrinsic_T 应该 3 个数 (得 {len(T)}), extrinsic_R 应该 9 个 (得 {len(R)})")
    return np.array(R, dtype=np.float64).reshape(3, 3), np.array(T, dtype=np.float64)


def _parse_floats(after_colon: str, lines, start_idx: int) -> list[float]:
    """把 [a, b, c] 或者跨多行的 list 抽出 floats."""
    chunk = after_colon
    if "[" in chunk and "]" in chunk:
        return _split_inline(chunk)
    # 跨行: 收集到 ] 为止
    if "[" in chunk:
        chunk_full = chunk
        i = start_idx + 1
        while "]" not in chunk_full and i < len(lines):
            chunk_full += " " + lines[i].strip()
            i += 1
        return _split_inline(chunk_full)
    raise ValueError(f"解析失败: {after_colon!r}")


def _split_inline(s: str) -> list[float]:
    s = s[s.index("[") + 1 : s.rindex("]")]
    return [float(x.strip()) for x in s.replace(",", " ").split() if x.strip()]


# ============================================================
# transformations.pcd 解析: 字段 [x y z intensity roll pitch yaw time]
# x..yaw 是 float32, time 是 double (8 bytes). 一行 36 byte.
# ============================================================
def read_transformations_pcd(path: Path) -> np.ndarray:
    """返回 shape=(N, 7) 的数组, 列 [x, y, z, roll, pitch, yaw, time]."""
    f = open(path, "rb")
    while True:
        line = f.readline()
        if line.startswith(b"DATA"):
            break
    raw = f.read()
    dtype = np.dtype([
        ("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4"),
        ("roll", "f4"), ("pitch", "f4"), ("yaw", "f4"),
        ("time", "f8"),
    ])
    arr = np.frombuffer(raw, dtype=dtype)
    out = np.zeros((len(arr), 7), dtype=np.float64)
    out[:, 0] = arr["x"]
    out[:, 1] = arr["y"]
    out[:, 2] = arr["z"]
    out[:, 3] = arr["roll"]
    out[:, 4] = arr["pitch"]
    out[:, 5] = arr["yaw"]
    out[:, 6] = arr["time"]
    return out


# ============================================================
# 简易 PCD 读 / 写 (binary, x y z + 其他 float32 列)
# ============================================================
def read_pcd_xyz(path: Path) -> np.ndarray:
    """返回 (N, M) 数组, M 是字段数. 假设全部 float32."""
    f = open(path, "rb")
    fields = []
    sizes = []
    while True:
        line = f.readline().decode("ascii", "replace")
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("SIZE"):
            sizes = [int(s) for s in line.split()[1:]]
        elif line.startswith("DATA"):
            break
    if not all(s == 4 for s in sizes):
        raise ValueError(f"非纯 float32 PCD: {path} sizes={sizes}")
    raw = f.read()
    rec = sum(sizes)
    n = len(raw) // rec
    return np.frombuffer(raw[: n * rec], dtype=np.float32).reshape(n, len(fields))


def write_pcd_binary(path: Path, points: np.ndarray, fields: list[str]) -> None:
    """fields 长度 = points.shape[1], 全 float32."""
    n, m = points.shape
    assert len(fields) == m
    header = (
        "# .PCD v0.7 - footprint_filter output\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(['4']*m)}\n"
        f"TYPE {' '.join(['F']*m)}\n"
        f"COUNT {' '.join(['1']*m)}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.astype(np.float32).tobytes())


# ============================================================
# 6D pose -> 4x4 矩阵.
# 跟 LIO-SAM / FAST-LIO-SAM 的 pcl::getTransformation 对齐:
#   T = Translate(x, y, z) * Rz(yaw) * Ry(pitch) * Rx(roll)
# ============================================================
def pose6d_to_matrix(x, y, z, roll, pitch, yaw) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


# ============================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("savemap_dir", type=Path,
                    help="SaveMap service 输出目录 (含 pcd/, transformations.pcd)")
    ap.add_argument("--extrinsic-yaml", type=Path, required=True,
                    help="LIO config yaml 或 airy_extrinsic 输出 yaml, "
                         "提供 mapping.extrinsic_T/R (IMU->LiDAR)")
    ap.add_argument("-o", "--output", type=Path, required=True,
                    help="输出 PCD 路径 (将被 align_floor / pcd_to_occgrid 当 GlobalMap 用)")
    ap.add_argument("--sensor-radius", type=float, default=0.5,
                    help="传感器水平半径 m. LiDAR 局部系 sqrt(x^2+y^2) < r 的点丢掉. 默认 0.5")
    ap.add_argument("--sensor-z-above", type=float, default=None, metavar="Z",
                    help="(可选) LiDAR 局部 z 上限 m, > z 的点丢掉 (头顶 / 低天花板)")
    ap.add_argument("--sensor-z-below", type=float, default=None, metavar="Z",
                    help="(可选) LiDAR 局部 z 下限 m, < z 的点丢掉 (脚下噪点 / 地下室)")
    ap.add_argument("--limit-keyframes", type=int, default=None,
                    help="(debug) 只处理前 N 个 keyframe")
    args = ap.parse_args()

    # --- 读外参 ---
    R_b_lidar, t_b_lidar = parse_extrinsic_yaml(args.extrinsic_yaml)
    T_b_lidar = np.eye(4)
    T_b_lidar[:3, :3] = R_b_lidar
    T_b_lidar[:3, 3] = t_b_lidar
    print(f"[INFO] T_b_lidar (extrinsic from {args.extrinsic_yaml.name}):")
    print(f"       t = {t_b_lidar}")
    print(f"       R = {R_b_lidar.flatten()[:3]} ... {R_b_lidar.flatten()[-3:]}")

    # --- 读 transformations ---
    pcd_dir = args.savemap_dir / "pcd"
    trans_path = args.savemap_dir / "transformations.pcd"
    if not pcd_dir.is_dir():
        print(f"[ERR] {pcd_dir} 不存在 (SaveMap pcd/ 子目录)", file=sys.stderr)
        return 2
    if not trans_path.is_file():
        print(f"[ERR] {trans_path} 不存在", file=sys.stderr)
        return 2

    poses = read_transformations_pcd(trans_path)
    print(f"[INFO] {len(poses)} keyframes in transformations.pcd")

    n_kf = len(poses)
    if args.limit_keyframes:
        n_kf = min(n_kf, args.limit_keyframes)
        print(f"[INFO] --limit-keyframes: only processing first {n_kf}")

    # --- 处理每个 KF ---
    accum = []
    fields_out = ["x", "y", "z", "intensity"]  # 简化: 只保留 4 列
    total_in, total_out = 0, 0
    sample_print = 5
    for i in range(n_kf):
        kf_pcd = pcd_dir / f"{i}.pcd"
        if not kf_pcd.is_file():
            print(f"[WARN] {kf_pcd} 缺失, 跳过", file=sys.stderr)
            continue
        try:
            local = read_pcd_xyz(kf_pcd)
        except Exception as e:
            print(f"[WARN] 读 {kf_pcd} 失败: {e}", file=sys.stderr)
            continue
        # local 是 LiDAR 系, 列 [x y z intensity ...]
        x, y, z = local[:, 0], local[:, 1], local[:, 2]
        sensor_dist = np.hypot(x, y)
        keep = sensor_dist >= args.sensor_radius
        if args.sensor_z_above is not None:
            keep &= (z <= args.sensor_z_above)
        if args.sensor_z_below is not None:
            keep &= (z >= args.sensor_z_below)
        kept = local[keep]
        total_in += len(local)
        total_out += len(kept)

        # 变换到世界系: T_w_lidar = T_w_b * T_b_lidar
        x_w, y_w, z_w, r, p, yw, _ = poses[i]
        T_w_b = pose6d_to_matrix(x_w, y_w, z_w, r, p, yw)
        T_w_lidar = T_w_b @ T_b_lidar
        # 变换 (扩列 1, 应用 4x4, 取 3 列)
        ones = np.ones((len(kept), 1), dtype=np.float64)
        homo = np.hstack([kept[:, :3].astype(np.float64), ones])
        world = (T_w_lidar @ homo.T).T[:, :3]
        # 拼回 intensity
        out = np.column_stack([world.astype(np.float32), kept[:, 3]])
        accum.append(out)

        if i < sample_print:
            print(f"  KF {i:>4}: {len(local):>6} -> {len(kept):>6} "
                  f"({100*len(kept)/max(1,len(local)):5.1f}% kept)  "
                  f"pose ({x_w:+6.2f}, {y_w:+6.2f}, {z_w:+5.2f})")
        elif i == sample_print:
            print(f"  ... ({n_kf-sample_print} more)")

    print(f"\n[OK] 总计 {total_in:,} -> {total_out:,} pts ({100*total_out/max(1,total_in):.1f}% kept)")
    print(f"     drop rate: {100*(1 - total_out/max(1,total_in)):.1f}%  (越高越多操作员被识别)")
    if not accum:
        print("[ERR] 没产出任何点, 检查输入", file=sys.stderr)
        return 2

    points_world = np.concatenate(accum, axis=0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_pcd_binary(args.output, points_world, fields_out)
    print(f"[OK] 写出 {args.output}  ({len(points_world):,} pts)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
