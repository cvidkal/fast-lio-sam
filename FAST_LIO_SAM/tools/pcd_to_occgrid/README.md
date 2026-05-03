# pcd_to_occgrid — 3D 点云地图 → 2D 占据栅格

把 FAST-LIO-SAM 输出的稠密 3D PCD 投影成 [nav2 map_server](https://navigation.ros.org/configuration/packages/configuring-map-server.html) 兼容的 PGM + YAML，用于 2D 全局规划。

> 解决 issue #6.

## 为什么需要

| 用途 | 适合的地图 |
|---|---|
| 机器狗本体避障 (桌子、低悬挂物、楼梯) | **3D 点云**，因为有"上下"概念 |
| 全局路径规划 (nav2 / amcl) | **2D 占据栅格**，nav2 目前只吃这个 |

LIO 跑出来的天然是 3D，本工具做离线投影。

## 安装

```bash
pip install numpy pillow scipy
# 或者只装最小依赖, scipy 仅用于膨胀, 没有它会自动跳过
pip install numpy pillow
```

## 用法

```bash
python3 pcd_to_occgrid.py /path/to/scans.pcd \
    -o /path/to/maps/airy_room \
    --resolution 0.05 \
    --z-min 0.10 --z-max 1.50 \
    --floor-z -0.20 \
    --dilate 1
```

完成后产物：

```
maps/airy_room.pgm     # 8bit 灰度: 0=占据, 254=自由, 205=未知
maps/airy_room.yaml    # nav2 标准描述
```

直接用 nav2 加载：

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=maps/airy_room.yaml
```

## 参数说明

| 参数 | 默认 | 含义 |
|---|---|---|
| `--resolution` | 0.05 | 栅格分辨率 m/cell (建议 0.05 室内 / 0.10 室外) |
| `--z-min` | 0.10 | 障碍层下界, 排除地面噪点 |
| `--z-max` | 1.50 | 障碍层上界, 排除天花板/吊顶 |
| `--floor-z` | -0.20 | 地面层下界, 该层用于标记 free 区域 |
| `--dilate` | 1 | 障碍膨胀次数, 给 nav2 inflation 留余量 |

## 算法

```
对每个 z ∈ [z_min, z_max] 的点  -> 投影到 (x,y) 格子 -> 标记 occupied (0)
对每个 z ∈ [floor_z, z_min) 的点 -> 投影到 (x,y) 格子 -> 标记 free (254)
其余格子 -> unknown (205)
障碍做 morphological dilation
上下翻转后写 PGM (因为 ROS map 原点在左下, PGM 在左上)
```

## 输入限制

- 支持 PCD ASCII / binary
- **不支持** binary_compressed (lz4 压缩的 PCD)
- 必须有 x/y/z 字段, 类型 float32

## 性能

| 点数 | 处理时间 (Jetson Orin) |
|---|---|
| 100 K  | < 1 s |
| 1 M    | ~ 5 s |
| 10 M   | ~ 30 s |

## 已知限制

- 没做 ray-casting，所以走廊外侧"看不到"的区域会被标 unknown 而不是 free。如果需要更好的 free space 推断，可以接 OctoMap。
