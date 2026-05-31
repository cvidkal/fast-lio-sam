# footprint_filter — per-scan 操作员/支架自身排除

把 SaveMap 输出的每个 keyframe 单独的 surf cloud (`pcd/<i>.pcd`，**LiDAR 局部系**) 在传感器局部坐标里做几何切除：

```
sensor_dist = sqrt(point.x² + point.y²)   # LiDAR 系
keep = sensor_dist >= sensor_radius
# (可选) 切传感器上下方
keep &= (point.z <= sensor_z_above)
keep &= (point.z >= sensor_z_below)
# 用 PGO 修正后的 6D 位姿 + IMU↔LiDAR 外参变到世界系
T_w_lidar = T_w_b(transformations.pcd) * T_b_lidar(extrinsic.yaml)
world_kept = T_w_lidar @ point[keep]
```

## 跟 `pcd_to_occgrid --footprint-radius` 的区别

| 维度 | `--footprint-radius` (in pcd_to_occgrid) | `footprint_filter` |
|---|---|---|
| 输入 | 已合并的 `GlobalMap.pcd` | 每个 KF 的原始 `pcd/<i>.pcd` |
| 距离参考 | 切片点 → 最近 trajectory 关键帧 (xy, KDTree) | 切片点 → **观测它的那个 LiDAR** (LIDAR 系自带) |
| 准确性 | 启发式 — PGO 修正 KF 位姿后，"最近 trajectory 帧" 可能不是真正观测它的帧 | 100% 准 — LiDAR 局部系下点-传感器距离不依赖位姿对齐 |
| 半径选择 | 紧凑 (~0.3m), 怕误吃近距离真墙 | 宽 (~0.5m) 也安全, 因为只剔除"传感器辐射 0.5m 内"的圆盘 |
| 依赖 | 只要 `GlobalMap.pcd` + `trajectory.pcd` | 还要 `pcd/` 子目录 + `transformations.pcd` + 真外参 yaml |
| 计算量 | KDTree 一次 | 810 个 KF 每个变换一次 (~10s) |

per-scan 算法上更"对"。如果场景能拿到完整 SaveMap 输出 + 真外参，**优先用 per-scan**。

## 用法

### 直接用工具

```bash
python3 footprint_filter.py \
    ~/Downloads/LOAM/ \
    --extrinsic-yaml ../airy_extrinsic/examples/airy_300DBEDB0075.yaml \
    -o /tmp/GlobalMap_filtered.pcd \
    --sensor-radius 0.5
```

### 在 build_2d_map 里串起来

```bash
python3 build_2d_map.py ~/Downloads/LOAM/ -o ~/maps/airy \
    --per-scan-filter \
    --per-scan-extrinsic-yaml ../airy_extrinsic/examples/airy_300DBEDB0075.yaml
```

会先跑 footprint_filter 输出 filtered cloud，再喂给 align_floor + pcd_to_occgrid。

## 可选参数

- `--sensor-radius FLOAT` — 水平半径 (默认 0.5m). 操作员典型 footprint 半径
- `--sensor-z-above FLOAT` — LiDAR 局部 z 上限. 切掉头顶 / 低天花板. **注意 LiDAR 局部系的 z 朝向跟传感器型号有关**:
  - RoboSense Airy 半球形: z 朝上 (cap), 地面 z<0 看不到 (盲区), 操作员身体也在盲区
  - 某些机械式 LiDAR: z 居中, ±15° 视场
  - 默认不设 (None), 不切 z, 避免误伤
- `--sensor-z-below FLOAT` — 同上, 下界

## 限制

- 假设 `pcd/<i>.pcd` 是 **LiDAR 局部系**. 这是 FAST-LIO-SAM `surfCloudKeyFrames[i]` 的约定 (`*feats_undistort` 在 lidar 系)
- 真外参从 yaml 读. 如果用了 `extrinsic_est_en: true`, 在线估计的外参没存盘, 这个工具拿不到 — 用 yaml 里的 default extrinsic 会偏
- 不补偿 LIO 漂移. 如果 IEKF 在某一段 drift, 后续 KF 的位姿不准, 变换出来的世界系点云就乱. 这个 issue 在 [#22](https://github.com/cvidkal/fast-lio-sam/issues/22) 跟踪
- 暂只输出 `[x y z intensity]` 4 列, 丢掉 normal / curvature 等. 大多数下游工具不需要

## 何时不该用这工具

- LiDAR 是上半球或下半球形 (Airy 半球) **且操作员身体本来就在盲区**: 那么 sensor-near 点其实是支架 / 头顶 / GPS 天线, 仍可以切, 但收益较小 (~10% drop)
- 没有 `pcd/` 子目录 (旧 SaveMap 输出 or 自己写的 service): 退回 `pcd_to_occgrid --footprint-radius` 启发式
- 没拿到真外参 (`extrinsic_est_en: true` 在线估计): 退到全局空间法
