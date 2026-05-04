# rosbag2 离线建图工作流

> 本文针对 ROS2 Humble。涉及的所有命令默认在 Jetson Orin (aarch64) / Ubuntu 22.04 上跑过。

## 总览

```
        ┌──────────────┐
  录    │ rslidar_sdk  │  /rslidar_points (XYZIRT)
  ───>  │ (driver)     │  /rslidar_imu_data
        └──────────────┘
              │
              │ ros2 bag record
              ↓
        ┌──────────────┐
  存    │ rosbag2 dir  │
        │ (sqlite3)    │
        └──────────────┘
              │
              │ ros2 launch fast_lio_sam mapping_bag.launch.py
              ↓
        ┌──────────────┐
  跑    │ fastlio_     │ → /Odometry  /cloud_registered  /path
  ───>  │ mapping      │   (落 PCD 在 src/FAST_LIO_SAM/PCD/)
        │ (this PR)    │
        └──────────────┘
              │
              │ python3 tools/pcd_to_occgrid (PR #7)
              ↓
        ┌──────────────┐
  转    │ map.pgm      │  ← nav2 map_server 直接吃
        │ map.yaml     │
        └──────────────┘
```

## 录 bag

### 必备前置

`rslidar_sdk` 必须 **`POINT_TYPE=XYZIRT` + `ENABLE_IMU_DATA_PARSE=ON`** 重编。否则点云缺 `ring` / `timestamp` per-point 字段，LIO 去畸变直接废 (实测漂移 200m)。检查方式：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
ros2 topic echo /rslidar_points --once --field fields | head -10
# 期望看到 "name='ring'" 和 "name='timestamp'"
```

### 录制

直接 `ros2 bag record`：

```bash
mkdir -p ~/bags
ros2 bag record -o ~/bags/airy_$(date +%Y%m%d_%H%M%S) \
    /rslidar_points /rslidar_imu_data
# 走完后 Ctrl+C, rosbag2 自动收尾
```

或用 systemd（同用户跑 driver + 录制，避开 FastDDS shm 跨用户问题）：

```bash
sudo systemctl start airy-record.service
# 走一圈
sudo systemctl stop airy-record.service
ls -lt ~/mou/dog/bags/ | head -3
```

### 录制建议

- **前 5 秒静止**: IMU 偏置初始化要静止数据
- **慢走 30-60 秒**: < 1 m/s, 别急加减速急转弯, 别让 IMU 饱和
- **可选回到原点**: 如果走一圈回原点, SAM/PGO 回环检测能验证轨迹一致性

## 体检 bag

回放前先验 bag 是否合格：

```bash
pip install rosbags  # 一次性
python3 /home/cgj/Codes/fast-lio-sam/FAST_LIO_SAM/scripts/bag_inspect.py \
    /path/to/bag_dir
```

输出会告诉你:
- 有几条 LiDAR / IMU 消息, 频率多少
- 点云字段齐不齐 (ring + timestamp 是关键)
- header.stamp 有没有倒退
- LiDAR 和 IMU 时间区间重叠多少 (< 30% 是致命, < 70% 警告)

致命问题 → 不要跑 launch, 先修 bag 或重录.

## 回放建图

### 基础用法

```bash
source /home/cgj/Codes/fast-lio-sam_ws/install/setup.bash
ros2 launch fast_lio_sam mapping_bag.launch.py \
    bag:=/path/to/rosbag2_dir \
    config_file:=/path/to/airy_via_bridge.yaml \
    rate:=1.0
```

参数:

| 参数 | 默认 | 说明 |
|---|---|---|
| `bag` | (必填) | rosbag2 目录路径 |
| `config_file` | `velodyne16.yaml` | LIO YAML, 例如 `airy_via_bridge.yaml` |
| `rate` | `1.0` | 回放倍速, Jetson Orin 建议 `0.5` 给算力留头 |
| `stop_service` | `''` | 回放前 stop 的 systemd 单位; 空则不动 |
| `grace_sec` | `5` | bag 退出后等多少秒再 SIGINT fastlio (落 PCD) |
| `rviz` | `false` | 是否启动 RViz |

### 避开"双源混乱"

如果你的 LiDAR 服务（例如 `airy-lidar.service`）此刻还在 LIVE 跑，**它会和 bag 同时往 `/rslidar_points` 发**，下游的 fastlio 会看到 stamp 来回跳几百秒，触发 `lidar loop back` 雪崩。

两种解决方法:

**A. 让 launch 自动停**（需要 sudoers NOPASSWD，见下文）:

```bash
ros2 launch fast_lio_sam mapping_bag.launch.py \
    bag:=/path/to/bag stop_service:=airy-lidar.service
```

launch 会:
1. `sudo -n systemctl stop airy-lidar.service` (回放前)
2. 起 fastlio + bag play
3. bag 退出 → `sudo -n systemctl start airy-lidar.service` (恢复)
4. 关闭 launch, fastlio 落 PCD

**B. 手动停**:

```bash
sudo systemctl stop airy-lidar.service
ros2 launch fast_lio_sam mapping_bag.launch.py bag:=/path/to/bag
sudo systemctl start airy-lidar.service
```

### 配置 NOPASSWD sudo (一次性)

让 `stop_service` 自动 stop/start 不弹密码，加一条 sudoers:

```bash
sudo visudo -f /etc/sudoers.d/airy-lidar-systemctl
```

写:
```
# 允许 cgj 不密码 (重)启 airy-lidar.service
cgj ALL=(ALL) NOPASSWD: /bin/systemctl stop airy-lidar.service, /bin/systemctl start airy-lidar.service, /bin/systemctl restart airy-lidar.service
```

`Ctrl+O` 保存。验证:

```bash
sudo -n systemctl is-active airy-lidar.service   # 应直接出结果, 不弹密码
```

## 收尾 / 看图

### PCD 在哪

fastlio_mapping 落 PCD 到 `${FAST_LIO_SAM_SOURCE_DIR}/PCD/scans.pcd`。运行时 launch 会打 banner 告诉你具体路径。

把它移到合理位置:

```bash
mkdir -p ~/maps
mv /home/cgj/Codes/fast-lio-sam/FAST_LIO_SAM/PCD/scans.pcd \
   ~/maps/airy_$(date +%Y%m%d_%H%M%S).pcd
```

### 转 2D nav2 占据栅格

```bash
python3 /home/cgj/Codes/fast-lio-sam/FAST_LIO_SAM/tools/pcd_to_occgrid/pcd_to_occgrid.py \
    ~/maps/airy_xxx.pcd \
    -o ~/maps/airy_xxx_2d \
    --resolution 0.05 --z-min 0.10 --z-max 1.50
```

得到 `airy_xxx_2d.pgm` + `airy_xxx_2d.yaml`，nav2 直接加载:

```bash
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=$HOME/maps/airy_xxx_2d.yaml
```

## 排错速查表

| 症状 | 可能原因 | 处理 |
|---|---|---|
| `ros2 topic list` 看到 topic, 但 `topic echo` 没数据 | FastDDS shm 跨用户死锁 | `export FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only.xml>` |
| fastlio 持续 `lidar loop back` | bag + 实时 driver 同时发 → stamp 来回跳 | 回放前 stop driver, 见上 |
| fastlio 持续 `No Effective Points` | 点云缺 ring/timestamp 字段 | rslidar_sdk 切 XYZIRT, 重编 + 重启 |
| fastlio 没收到点云 | bridge / driver QoS 不匹配 | 检查发布端是 `RELIABLE` 还是 `BEST_EFFORT`, sub 端要 `<=` 它 |
| PCD 一直没落盘 | fastlio 没收到 SIGINT, 或 `pcd_save_en=false` | launch 默认会发 SIGINT; YAML 里 `pcd_save: pcd_save_en: true` |
| bag 跑完 launch 还不退 | `OnProcessExit` 没触发 | 检查 grace_sec 是否合理; 实在不行 Ctrl+C |
| `sudo -n systemctl ...` 提示要密码 | sudoers 没配 NOPASSWD | 见上面 "配置 NOPASSWD sudo" |

## 相关文档

- [`docs/STAGE6_SMOKE.md`](STAGE6_SMOKE.md) — ROS2 移植烟测报告 + 5 个部署 gotcha 详细分析
- [`tools/pcd_to_occgrid/README.md`](../tools/pcd_to_occgrid/README.md) — 3D PCD → 2D nav2 占据栅格工具
- [`tools/airy_extrinsic/README.md`](../tools/airy_extrinsic/README.md) — Airy DIFOP 外参解析工具
