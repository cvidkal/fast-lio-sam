#!/usr/bin/env python3
"""
pcd_to_occgrid — 离线把 FAST-LIO 输出的 3D 点云 (.pcd) 投影成 2D 占据栅格,
                 直接生成 nav2 可用的 map.pgm + map.yaml.

为什么需要这一步:
  - FAST-LIO2 输出的 scans.pcd 是稠密 3D 地图, 适合可视化和狗本体避障
  - nav2 / amcl 全局规划目前只能吃 2D 占据栅格
  - 思路:
      1) 取 3D 地图中机器狗"身体高度"那一层 (默认 0.10m ~ 1.50m)
      2) 把这层点 xy 投到地面网格里, 有点的格子 = 障碍 (黑)
      3) 没观测到但被遮挡的格子 = 未知 (灰), 没观测且无遮挡 = 自由 (白)
      4) 形态学清理一下毛刺
      5) 输出 occupancy_grid_msgs 兼容的 PGM + YAML

可选增强:
  --auto-floor       自动从 z 直方图找地面 (取最低 5% 点的中位 z), 之后
                     z_min/z_max 都是相对地面的高度. 不用再手动量地面 z.
  --raycast TRAJ_PCD 传入 trajectory.pcd (FAST-LIO-SAM SaveMap 输出),
                     对每个关键帧做 polar raycast 把 LiDAR 看到的可通行区域
                     标 free. 极大提高 nav2 用得上的 free 比例.

依赖:
  pip install numpy pillow scipy
  (PCD 解析自己实现, 不依赖 open3d / pcl)

用法:
  ./pcd_to_occgrid.py scans.pcd \
      -o /home/nvidia/mou/dog/dog_mapping_ws/maps/airy_room \
      --resolution 0.05 \
      --z-min 0.10 --z-max 1.50

  # 推荐: 用 SaveMap 输出的 GlobalMap.pcd + trajectory.pcd, 开 raycast
  ./pcd_to_occgrid.py /tmp/save_dir/GlobalMap.pcd \
      -o maps/airy_room \
      --auto-floor \
      --raycast /tmp/save_dir/trajectory.pcd
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path

import numpy as np


# ============================================================
# PCD 解析 (支持 ASCII + binary, 不支持 binary_compressed)
# ============================================================
def load_pcd_xyz(path: str | os.PathLike) -> np.ndarray:
    """读取 PCD, 仅提取 x/y/z, 返回 shape=(N,3) 的 float32 数组."""
    path = Path(path)
    with open(path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'PCD 头未读完: {path}')
            line_s = line.decode('ascii', errors='replace').strip()
            header_lines.append(line_s)
            if line_s.startswith('DATA'):
                data_mode = line_s.split()[1]
                break
        data_pos = f.tell()

        meta = {}
        for ln in header_lines:
            if ln.startswith('#') or ' ' not in ln:
                continue
            k, *v = ln.split()
            meta[k] = v
        fields  = meta['FIELDS']
        sizes   = list(map(int, meta['SIZE']))
        types_  = meta['TYPE']
        counts  = list(map(int, meta['COUNT']))
        n_pts   = int(meta['POINTS'][0])

        try:
            ix, iy, iz = fields.index('x'), fields.index('y'), fields.index('z')
        except ValueError as e:
            raise ValueError(f'PCD 没有 x/y/z 字段: {fields}') from e
        if not (sizes[ix] == sizes[iy] == sizes[iz] == 4 and
                types_[ix] == types_[iy] == types_[iz] == 'F'):
            raise ValueError('xyz 不是 float32, 暂不支持')

        if data_mode == 'ascii':
            arr = np.loadtxt(path, skiprows=len(header_lines), dtype=np.float64)
            return arr[:, [ix, iy, iz]].astype(np.float32)

        if data_mode == 'binary':
            # 计算每点字节数和 xyz 偏移
            field_bytes = [s * c for s, c in zip(sizes, counts)]
            point_bytes = sum(field_bytes)
            offsets = np.cumsum([0] + field_bytes[:-1])
            ox, oy, oz = offsets[ix], offsets[iy], offsets[iz]

            f.seek(data_pos)
            raw = np.frombuffer(f.read(point_bytes * n_pts),
                                dtype=np.uint8).reshape(n_pts, point_bytes)
            xs = np.frombuffer(raw[:, ox:ox+4].tobytes(), dtype=np.float32)
            ys = np.frombuffer(raw[:, oy:oy+4].tobytes(), dtype=np.float32)
            zs = np.frombuffer(raw[:, oz:oz+4].tobytes(), dtype=np.float32)
            return np.stack([xs, ys, zs], axis=1)

        raise ValueError(f'不支持的 PCD DATA 模式: {data_mode}')


# ============================================================
# 投影 + 形态学
# ============================================================
def project_to_grid(
    xyz: np.ndarray,
    resolution: float,
    z_min: float,
    z_max: float,
    floor_z: float | None,
) -> tuple[np.ndarray, tuple[float, float]]:
    """
    返回:
        grid: shape=(H,W) uint8, 0=free, 100=occupied, 255=unknown(临时)
        origin_xy: 地图左下角对应的世界坐标 (x_min, y_min)
    """
    if floor_z is not None:
        # 把地面附近的点 (用作 free 标记) 也保留
        keep_mask = (xyz[:, 2] >= floor_z) & (xyz[:, 2] <= z_max)
    else:
        keep_mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)

    pts = xyz[keep_mask]
    if pts.size == 0:
        raise RuntimeError('裁剪后没有点, 检查 z 范围')

    # 障碍层: z_min ~ z_max
    obs_mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    # 地面层: floor_z ~ z_min (用来标 free)
    if floor_z is not None:
        free_mask = (pts[:, 2] >= floor_z) & (pts[:, 2] < z_min)
    else:
        free_mask = np.zeros(len(pts), dtype=bool)

    x_min = float(np.min(xyz[:, 0])) - 1.0
    y_min = float(np.min(xyz[:, 1])) - 1.0
    x_max = float(np.max(xyz[:, 0])) + 1.0
    y_max = float(np.max(xyz[:, 1])) + 1.0

    W = int(np.ceil((x_max - x_min) / resolution))
    H = int(np.ceil((y_max - y_min) / resolution))
    grid = np.full((H, W), 255, dtype=np.uint8)  # unknown

    # 先标 free
    if free_mask.any():
        fp = pts[free_mask]
        ix = np.clip(((fp[:, 0] - x_min) / resolution).astype(int), 0, W - 1)
        iy = np.clip(((fp[:, 1] - y_min) / resolution).astype(int), 0, H - 1)
        grid[iy, ix] = 0

    # 再标 occupied (覆盖 free)
    op = pts[obs_mask]
    ix = np.clip(((op[:, 0] - x_min) / resolution).astype(int), 0, W - 1)
    iy = np.clip(((op[:, 1] - y_min) / resolution).astype(int), 0, H - 1)
    grid[iy, ix] = 100

    return grid, (x_min, y_min)


def morph_clean(grid: np.ndarray, dilate_iter: int = 1) -> np.ndarray:
    """简单的形态学: 膨胀障碍, 把孤立 free 点连成块. 不依赖 scipy 也能跑."""
    out = grid.copy()
    occ = (out == 100)
    if dilate_iter > 0:
        try:
            from scipy.ndimage import binary_dilation
            occ = binary_dilation(occ, iterations=dilate_iter)
        except ImportError:
            pass
    out[occ] = 100
    return out


# ============================================================
# 自动找地面 z (从 z 直方图最低 5% 取中位)
# ============================================================
def auto_detect_floor_z(xyz: np.ndarray, low_pct: float = 5.0) -> float:
    """取最低 low_pct% 点的中位 z 作为地面."""
    z = xyz[:, 2]
    cutoff = np.percentile(z, low_pct)
    floor = float(np.median(z[z <= cutoff]))
    return floor


# ============================================================
# Raycast: 沿轨迹做 polar 扫描, 给 free space 涂白
# ============================================================
def raycast_free_from_trajectory(
    grid: np.ndarray,
    origin_xy: tuple[float, float],
    resolution: float,
    traj_xy: np.ndarray,
    n_angles: int = 720,
    max_range_m: float = 30.0,
) -> np.ndarray:
    """
    对每个轨迹点 (x, y), 在 grid 上做 polar 扫描:
      把每个 angle bin 上 "kf 到最近 occupied" 之间的格子标 free.
    occupied 格子保持不变.

    Args:
        grid:        project_to_grid 输出的 (H, W) 图; 100=occupied, 0=free, 255=unknown
        origin_xy:   (x_min, y_min) 世界坐标 → 像素 (0, 0)
        resolution:  m / pixel
        traj_xy:     (N, 2) 关键帧世界坐标
        n_angles:    极坐标 bin 数 (默认 720, 即 0.5°/bin)
        max_range_m: 雷达 raycast 最大距离 (默认 30m)
    """
    H, W = grid.shape
    x_min, y_min = origin_xy
    max_range_px = int(max_range_m / resolution)

    # 找所有 occupied 格子坐标 (一次性)
    occ_iy, occ_ix = np.where(grid == 100)
    if len(occ_ix) == 0:
        return grid

    free_mark = np.zeros_like(grid, dtype=bool)

    # kf -> 像素
    kf_ix = np.clip(((traj_xy[:, 0] - x_min) / resolution).astype(int), 0, W - 1)
    kf_iy = np.clip(((traj_xy[:, 1] - y_min) / resolution).astype(int), 0, H - 1)

    for k in range(len(traj_xy)):
        cx, cy = int(kf_ix[k]), int(kf_iy[k])
        # 取 kf 周围 max_range 内的 occupied
        dx = occ_ix - cx
        dy = occ_iy - cy
        dist = np.sqrt(dx * dx + dy * dy)
        in_range = dist < max_range_px
        if not in_range.any():
            continue
        sub_x = occ_ix[in_range]
        sub_y = occ_iy[in_range]
        sub_dx = dx[in_range]
        sub_dy = dy[in_range]
        sub_dist = dist[in_range]

        # 按 angle 划 bin, 每个 bin 取最近的 occupied
        angles = np.arctan2(sub_dy, sub_dx)
        ang_bins = ((angles + np.pi) / (2 * np.pi) * n_angles).astype(np.int32)
        ang_bins = np.clip(ang_bins, 0, n_angles - 1)

        nearest_dist = np.full(n_angles, np.inf)
        nearest_idx = np.full(n_angles, -1, dtype=np.int32)
        for i, b in enumerate(ang_bins):
            if sub_dist[i] < nearest_dist[b]:
                nearest_dist[b] = sub_dist[i]
                nearest_idx[b] = i

        # 对每个 bin 的最近终点画 Bresenham 线 (不含终点)
        for i in nearest_idx[nearest_idx >= 0]:
            x1, y1 = int(sub_x[i]), int(sub_y[i])
            _bresenham_mark(cx, cy, x1, y1, free_mark)

    out = grid.copy()
    # 标 free, 但不覆盖 occupied
    apply = free_mark & (out != 100)
    out[apply] = 0
    return out


def _bresenham_mark(x0: int, y0: int, x1: int, y1: int, out: np.ndarray) -> None:
    """从 (x0, y0) 到 (x1, y1) 不含终点的格子置 True. out shape=(H, W)."""
    H, W = out.shape
    dx = x1 - x0
    dy = y1 - y0
    n = max(abs(dx), abs(dy))
    if n == 0:
        return
    sx = dx / n
    sy = dy / n
    xs = (x0 + np.arange(n) * sx).astype(np.int32)
    ys = (y0 + np.arange(n) * sy).astype(np.int32)
    valid = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
    out[ys[valid], xs[valid]] = True


# ============================================================
# 写出 nav2 兼容的 PGM + YAML
# ============================================================
def save_map(
    grid: np.ndarray,
    origin_xy: tuple[float, float],
    resolution: float,
    out_prefix: str | os.PathLike,
) -> None:
    """
    写 nav2 map_server 标准格式:
        out_prefix.pgm  : 8bit 灰度, 0=占据(黑), 254=自由(白), 205=未知(灰)
        out_prefix.yaml : 含 resolution / origin / 阈值
    """
    from PIL import Image

    out_prefix = Path(out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    # nav2 习俗: 黑色 = 占据, 白色 = 自由
    img = np.full(grid.shape, 205, dtype=np.uint8)   # unknown
    img[grid == 0]   = 254                           # free
    img[grid == 100] = 0                             # occupied

    # PGM 的 (0,0) 在左上, ROS 地图原点在左下, 需要上下翻转
    img = np.flipud(img)

    pgm_path = str(out_prefix) + '.pgm'
    yaml_path = str(out_prefix) + '.yaml'
    Image.fromarray(img, mode='L').save(pgm_path)

    yaml_text = (
        f'image: {os.path.basename(pgm_path)}\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin_xy[0]:.4f}, {origin_xy[1]:.4f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.25\n'
        f'mode: trinary\n'
    )
    with open(yaml_path, 'w') as f:
        f.write(yaml_text)

    print(f'[OK] 写出: {pgm_path}')
    print(f'[OK] 写出: {yaml_path}')
    print(f'      尺寸: {grid.shape[1]} x {grid.shape[0]} px')
    print(f'      分辨率: {resolution} m/px')
    print(f'      原点 (左下): {origin_xy}')


# ============================================================
def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('pcd', help='输入 PCD 文件 (FAST-LIO 输出的 scans.pcd 或 SaveMap 的 GlobalMap.pcd)')
    p.add_argument('-o', '--output', required=True,
                   help='输出文件前缀, 例: maps/airy_room (会得到 .pgm + .yaml)')
    p.add_argument('--resolution', type=float, default=0.05,
                   help='栅格分辨率 m/cell (默认 0.05, 即 5cm)')
    p.add_argument('--z-min', type=float, default=0.10,
                   help='障碍层 z 下界 (默认 0.10m, 排除地面). 如果开 --auto-floor, 此值视作"地面以上 X m"')
    p.add_argument('--z-max', type=float, default=1.50,
                   help='障碍层 z 上界 (默认 1.50m). 如果开 --auto-floor, 此值视作"地面以上 X m"')
    p.add_argument('--floor-z', type=float, default=-0.20,
                   help='地面层 z 下界, 该层用来标 free 区域 (默认 -0.20m). 用 --auto-floor 自动覆盖此值.')
    p.add_argument('--auto-floor', action='store_true',
                   help='自动从 z 直方图找地面 (取最低 5%% 点的中位 z), z-min/z-max/floor-z 全部相对地面.')
    p.add_argument('--raycast', metavar='TRAJ_PCD', default=None,
                   help='传入 trajectory.pcd, 对每个关键帧做 polar raycast 把可通行区域标 free.')
    p.add_argument('--raycast-max-range', type=float, default=30.0,
                   help='raycast 最大距离 m (默认 30, 跟 Airy 等 LiDAR 室内有效距离一致)')
    p.add_argument('--raycast-angles', type=int, default=720,
                   help='raycast 极坐标 bin 数 (默认 720, 即 0.5°/bin)')
    p.add_argument('--dilate', type=int, default=1,
                   help='障碍膨胀次数 (默认 1, 给 nav2 留点 inflation 缓冲)')
    args = p.parse_args()

    print(f'[INFO] 加载 PCD: {args.pcd}')
    xyz = load_pcd_xyz(args.pcd)
    print(f'       共 {xyz.shape[0]:,} 点')
    print(f'       x: [{xyz[:,0].min():.2f}, {xyz[:,0].max():.2f}]')
    print(f'       y: [{xyz[:,1].min():.2f}, {xyz[:,1].max():.2f}]')
    print(f'       z: [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]')

    # 自动找地面
    if args.auto_floor:
        gnd = auto_detect_floor_z(xyz)
        print(f'[INFO] auto_floor: 地面 z ≈ {gnd:.3f} m')
        z_min = gnd + args.z_min
        z_max = gnd + args.z_max
        floor_z = gnd + args.floor_z
        print(f'       (相对地面 +z_min={args.z_min}, +z_max={args.z_max}, floor_z={args.floor_z})')
    else:
        z_min, z_max, floor_z = args.z_min, args.z_max, args.floor_z

    print(f'[INFO] 投影到 2D 占据栅格 res={args.resolution} z=[{z_min:.2f},{z_max:.2f}]')
    grid, origin = project_to_grid(
        xyz,
        resolution=args.resolution,
        z_min=z_min,
        z_max=z_max,
        floor_z=floor_z,
    )

    if args.raycast:
        print(f'[INFO] 加载轨迹: {args.raycast}')
        traj = load_pcd_xyz(args.raycast)
        print(f'       共 {len(traj)} 关键帧, raycast max_range={args.raycast_max_range}m angles={args.raycast_angles}')
        grid = raycast_free_from_trajectory(
            grid, origin, args.resolution,
            traj_xy=traj[:, :2],
            n_angles=args.raycast_angles,
            max_range_m=args.raycast_max_range,
        )
        n_free = int((grid == 0).sum())
        n_occ = int((grid == 100).sum())
        print(f'       raycast 完成: occupied={n_occ:,}  free={n_free:,}')

    grid = morph_clean(grid, dilate_iter=args.dilate)
    save_map(grid, origin, args.resolution, args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
