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

依赖:
  pip install numpy pillow scipy
  (PCD 解析自己实现, 不依赖 open3d / pcl)
用法:
  ./pcd_to_occgrid.py scans.pcd \
      -o /home/nvidia/mou/dog/dog_mapping_ws/maps/airy_room \
      --resolution 0.05 \
      --z-min 0.10 --z-max 1.50
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
    p.add_argument('pcd', help='输入 PCD 文件 (FAST-LIO 输出的 scans.pcd)')
    p.add_argument('-o', '--output', required=True,
                   help='输出文件前缀, 例: maps/airy_room (会得到 .pgm + .yaml)')
    p.add_argument('--resolution', type=float, default=0.05,
                   help='栅格分辨率 m/cell (默认 0.05, 即 5cm)')
    p.add_argument('--z-min', type=float, default=0.10,
                   help='障碍层 z 下界 (默认 0.10m, 排除地面)')
    p.add_argument('--z-max', type=float, default=1.50,
                   help='障碍层 z 上界 (默认 1.50m, 排除天花板/吊顶)')
    p.add_argument('--floor-z', type=float, default=-0.20,
                   help='地面层 z 下界, 该层用来标 free 区域 (默认 -0.20m)')
    p.add_argument('--dilate', type=int, default=1,
                   help='障碍膨胀次数 (默认 1, 给 nav2 留点 inflation 缓冲)')
    args = p.parse_args()

    print(f'[INFO] 加载 PCD: {args.pcd}')
    xyz = load_pcd_xyz(args.pcd)
    print(f'       共 {xyz.shape[0]:,} 点')
    print(f'       x: [{xyz[:,0].min():.2f}, {xyz[:,0].max():.2f}]')
    print(f'       y: [{xyz[:,1].min():.2f}, {xyz[:,1].max():.2f}]')
    print(f'       z: [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]')

    print(f'[INFO] 投影到 2D 占据栅格 res={args.resolution} z=[{args.z_min},{args.z_max}]')
    grid, origin = project_to_grid(
        xyz,
        resolution=args.resolution,
        z_min=args.z_min,
        z_max=args.z_max,
        floor_z=args.floor_z,
    )
    grid = morph_clean(grid, dilate_iter=args.dilate)
    save_map(grid, origin, args.resolution, args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
