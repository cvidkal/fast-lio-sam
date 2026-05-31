# tools/ — 离线后处理工具

FAST-LIO-SAM 跑完 + `ros2 service call /save_map` 后, 这里的脚本把 SaveMap 输出
(`GlobalMap.pcd` / `trajectory.pcd` / `transformations.pcd` + `pcd/`) 加工成可用产物。

## 2D 占据栅格图 (nav2 / amcl) —— 从这里开始

**一条命令: [`build_2d_map/build_2d_map.py`](build_2d_map/README.md)**

```bash
# 默认 (手持采集)
python3 tools/build_2d_map/build_2d_map.py ~/Downloads/LOAM -o ~/maps/room
# 机器狗 (雷达低 + 体型): 用 dog profile
python3 tools/build_2d_map/build_2d_map.py <SaveMap目录> -o ~/maps/dog_room --profile dog
# 加载
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/room.yaml
```

它内部串 3 步 (一般不用单独跑):

| 步骤 | 脚本 | 作用 |
|---|---|---|
| 1 | [`align_floor/`](align_floor/README.md) | RANSAC 拟合地面 → 旋转使 z=0=真地面 (修手持重力倾斜). 只接受近竖直法向, 不会锁到墙. |
| 2 | [`pcd_to_occgrid/`](pcd_to_occgrid/README.md) | z 切片 + 投影 + 沿轨迹 polar raycast 填 free + 可选 footprint 排除 + autocrop 裁空白边 |
| (可选) | [`footprint_filter/`](footprint_filter/README.md) | per-scan 在传感器局部系按外参切除操作员/机身 (比 pcd_to_occgrid 的全局 `--footprint-radius` 更准) |

> ⚠️ 不要用其它包里的 `generate_occupancy_map_from_pcd.py` 之类脚本生成 airy/dog 2D 图
> —— 那类 full-map height-delta 启发式在带地面噪声的 LOAM 点云上几乎全判障碍, 不可用。

## 其它

- [`airy_extrinsic/`](airy_extrinsic/) — 从 RoboSense Airy DIFOP 解 IMU↔LiDAR 外参, 出 config / footprint_filter 用的 yaml
- `init_pose/` — 定位初始位姿工具
