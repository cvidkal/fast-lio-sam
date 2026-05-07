# align_floor — FAST-LIO 输出 PCD 的 gravity 后校正

## 为什么需要

FAST-LIO 在 **IMU 初始化阶段** (前几十个 IMU 样本, ~0.1s) 用平均加速度估计重力方向, 并把世界系 z 轴对齐到这个方向. 如果初始化那段时间 IMU 本身没保持水平 (机器狗站歪了 / 采集人手持没端平), 那么整个建图世界系跟真实重力差几度.

具体征兆 :

- `GlobalMap.pcd` 拟合地面 → 法向量与 +z 偏 1° 以上
- `trajectory.pcd` 整体在 z 上有线性趋势 (沿 x/y 走时 z 慢慢升降)
- 喂给 nav2 后地图看着歪 / 切片时墙体不齐

FAST-LIO 自身没法重置这个 — IMU init 一旦完成, 重力方向就锁死. 唯一办法是**后处理把整云重新转一次**.

## 用法

前提 : 你已经跑过 `ros2 service call /save_map ...`, 拿到 `GlobalMap.pcd` + `trajectory.pcd`.

```bash
python3 tools/align_floor/align_floor.py /path/to/save_dir
# 默认输出到 /path/to/save_dir/aligned/
```

可选参数 :

```text
-o OUT, --output-dir OUT    输出目录 (默认 <input_dir>/aligned)
--floor-z-range Z_MIN Z_MAX 拟合地面用的 z 区间 (默认: 自动取最低 20%)
--ransac-thresh 0.05        RANSAC 内点距离阈值 (m)
--ransac-iter 500           RANSAC 迭代数
```

输出 :

- `aligned/GlobalMap.pcd` — 旋转 + 平移过的全局地图, 地面 z ≈ 0
- `aligned/trajectory.pcd` — 同步处理的关键帧轨迹
- `aligned/alignment.yaml` — 4×4 变换矩阵 + 计算的倾斜角

## 算法

1. 在原云里取 **最低 20% 的 z 那部分** (推测是地面)
2. **RANSAC** 拟合一个平面, 取它的法向量 `n` (确保指向 +z)
3. **Rodrigues** 公式构造把 `n` 转到 `[0, 0, 1]` 的旋转 `R`
4. 对所有点和轨迹做 `p_aligned = R @ p`
5. 平移让地面 `z=0`

整个过程**不**重新跑 LIO, 也不需要重新喂 IMU 数据 — 纯几何操作.

## 限制

- 默认假设 **地面是单一平面**. 多层楼/复杂场景请用 `--floor-z-range` 手动指定单一楼层
- 只校正 **重力 (roll/pitch)**, 不校正 yaw — yaw 没绝对参考
- 输入 PCD 必须是 **binary** 格式 + float32 字段 (FAST-LIO 默认输出就是这样)
