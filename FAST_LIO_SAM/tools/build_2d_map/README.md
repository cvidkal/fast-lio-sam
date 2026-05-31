# build_2d_map — 一条命令做出 nav2 兼容 PGM/YAML

把 FAST-LIO-SAM 的 SaveMap 输出转成 nav2 全局规划用的 2D 占据栅格，封装了：

1. **align_floor** — RANSAC 拟合地面，旋转 + 平移让 z=0 = 真地面、z 轴 = 真重力
2. **pcd_to_occgrid** — z 切片 + 投影 + 沿轨迹做 polar raycast 填 free space

> 解决 issue #21.

## 为什么要封装

3 步分开跑非常容易踩坑：

- **漏 align**：z=0 是 LIO init 位置（LiDAR 杆顶），切片 `[0.2, 1.5]` 切到空中，PGM 几乎全空
- **顺序错**：raycast 用的轨迹和点云不在同一坐标系，free space 全错位
- **路径乱**：中间产物到处放

## 用法

```bash
# 最简: SaveMap 输出 ($HOME/Downloads/LOAM/) 转 nav2 map
python3 build_2d_map.py ~/Downloads/LOAM/ -o ~/maps/airy_room

# 然后:
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/airy_room.yaml
```

## 选项

```bash
build_2d_map.py SAVEMAP_DIR -o OUT_PREFIX [选项]

# 跳过 gravity-align (点云已经对齐时)
--no-align

# 跳过 raycast (只标 occupied + 地面切片 free)
--no-raycast

# 切片 (默认地面以上 0.2-1.5m, 适合人/狗高度)
--z-min 0.10 --z-max 1.50

# 自定义分辨率
--resolution 0.05

# 调 raycast 距离 (默认 30m)
--raycast-range 30.0

# 保留中间产物 (debug 用), 输出到 OUT_PREFIX.aligned/
--keep-intermediate

# 手动指定地面 z (RANSAC 自动失败时的 fallback)
--floor-z-range -3.0 -2.0
```

## 输出

```
~/maps/airy_room.pgm           # nav2 standard PGM (8bit gray)
~/maps/airy_room.yaml          # nav2 map_server YAML
~/maps/airy_room.summary.txt   # 跑了什么, 切片范围, raycast 状态

# --keep-intermediate 时还有:
~/maps/airy_room.aligned/
    GlobalMap.pcd              # gravity-aligned 点云
    trajectory.pcd             # gravity-aligned 关键帧
    transformations.pcd        # gravity-aligned 6D 位姿
    step2_align.log            # align_floor 日志 + RANSAC 结果
    step3_occgrid.log          # pcd_to_occgrid 日志 + 切片统计
```

## 工作流图

```
LIO 跑完 (mapping_bag.launch.py / mapping_airy.launch.py)
    ↓
SaveMap service 触发 (launch shutdown 时自动, 见 commit b5f4c88)
    ↓
$HOME/Downloads/LOAM/{GlobalMap,trajectory,transformations}.pcd
    ↓ build_2d_map.py
$out_prefix.pgm + $out_prefix.yaml
    ↓ ros2 run nav2_map_server map_server
nav2 + amcl 全局规划
```

## 常见问题

**Q: PGM 空白 / 几乎全 unknown**

A: 可能没 align, 或者切片范围不对.
- 先看 `summary.txt` 的 `align: ON/OFF`. 应该是 ON
- 看中间日志 `step2_align.log` 里的 `tilt vs +z` 行, 如果 > 5°, 数据本身有问题
- 看 `step3_occgrid.log` 里 `z: [...]` 范围, 如果切片把整个范围切空了, 调 `--z-min`/`--z-max`

**Q: 走廊远端是 unknown 不是 free**

A: raycast 只对**击中 occupied 的射线**填 free. 远端没墙的方向（开阔地）保持 unknown 是有意为之 — 没观测就不假设.
- 如果场景就是没墙, 用 `--no-raycast` + 调 `--floor-z` 让地面观测当 free
- 或者单独跑 OctoMap 处理开阔区

**Q: 跑很慢 / 卡住**

A: raycast 是 pure python (issue #16 跟踪性能). 大场景可以:
- 调小 `--raycast-range` (默认 30m → 试 15m)
- `--no-raycast` 跳过这步, 牺牲 free 覆盖率换速度

## 限制

- 只支持单层场景. 多层楼要先按 z 拆点云 (例如 `pdal pipeline` filter), 分别跑
- 假设场景有大块平地. 纯户外起伏地形会让 RANSAC 拟合错, 用 `--floor-z-range` 手动指定 (issue #17)
- 真 3D raycast 在 issue #20 跟踪
