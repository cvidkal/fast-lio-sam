#!/usr/bin/env python3
"""
bag_inspect.py — 检查 rosbag2 是否能给 FAST-LIO-SAM 喂出有意义的地图.

LIO 算法对输入 bag 的硬性要求, 不满足建图必定崩:
  1. 至少有一个 sensor_msgs/msg/PointCloud2 话题
  2. 至少有一个 sensor_msgs/msg/Imu 话题
  3. 点云字段: x, y, z 必有; intensity / ring / time(or timestamp) 强烈推荐
     (无 ring/timestamp 时去畸变会失效, 见 docs/STAGE6_SMOKE.md)
  4. IMU 频率 >= 50 Hz, 越高越稳 (一般 100-500 Hz)
  5. PointCloud header.stamp 严格单调递增
  6. PointCloud 和 IMU 时间区间重叠 >90%

用法:
  python3 bag_inspect.py /path/to/rosbag2_dir [--lid-topic /rslidar_points] [--imu-topic /rslidar_imu_data]

依赖:
  pip install rosbags  (无需 ROS2 也能跑)
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter, defaultdict


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag', help='rosbag2 目录 (内含 metadata.yaml + .db3 / .mcap)')
    p.add_argument('--lid-topic', default=None,
                   help='点云话题 (不填则自动选第一个 PointCloud2)')
    p.add_argument('--imu-topic', default=None,
                   help='IMU 话题 (不填则自动选第一个 Imu)')
    p.add_argument('--max-msgs', type=int, default=None,
                   help='只看前 N 条消息 (调试用, 默认全扫)')
    args = p.parse_args()

    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        print('[ERR] 需要 rosbags 库:  pip install rosbags', file=sys.stderr)
        return 2

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    issues = []
    warnings = []
    print(f'[inspect] {args.bag}\n')

    with Reader(args.bag) as reader:
        # ===== 1. topics =====
        topics = {c.topic: c.msgtype for c in reader.connections}
        print('Topics:')
        for t, ty in sorted(topics.items()):
            print(f'  {t}  -> {ty}')

        pc_topics = [t for t, ty in topics.items()
                     if ty == 'sensor_msgs/msg/PointCloud2']
        imu_topics = [t for t, ty in topics.items()
                      if ty == 'sensor_msgs/msg/Imu']

        lid_topic = args.lid_topic
        if lid_topic is None:
            if not pc_topics:
                issues.append('没有 PointCloud2 话题')
                lid_topic = None
            else:
                lid_topic = pc_topics[0]
                if len(pc_topics) > 1:
                    warnings.append(
                        f'多个 PointCloud2 话题 {pc_topics}, 自动用 {lid_topic}, '
                        f'要换的话用 --lid-topic 指定'
                    )

        imu_topic = args.imu_topic
        if imu_topic is None:
            if not imu_topics:
                issues.append('没有 Imu 话题 — LIO 跑不了')
                imu_topic = None
            else:
                imu_topic = imu_topics[0]
                if len(imu_topics) > 1:
                    warnings.append(
                        f'多个 Imu 话题 {imu_topics}, 自动用 {imu_topic}'
                    )

        print(f'\n选定:')
        print(f'  lidar : {lid_topic}')
        print(f'  imu   : {imu_topic}')

        if lid_topic is None or imu_topic is None:
            _print_summary(issues, warnings)
            return 1

        # ===== 2. 详细扫描 =====
        lid_stamps = []
        imu_stamps = []
        lid_field_names = None
        lid_first_widths = []
        n_msgs = 0
        max_msgs = args.max_msgs

        for conn, _, raw in reader.messages(
                connections=[c for c in reader.connections
                             if c.topic in (lid_topic, imu_topic)]):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            if conn.topic == lid_topic:
                lid_stamps.append(t)
                if lid_field_names is None:
                    lid_field_names = [(f.name, f.offset, f.datatype) for f in msg.fields]
                if len(lid_first_widths) < 5:
                    lid_first_widths.append((msg.width, msg.height, msg.point_step))
            else:
                imu_stamps.append(t)

            n_msgs += 1
            if max_msgs and n_msgs >= max_msgs:
                break

    # ===== 3. 报告 =====
    print(f'\n点云: {len(lid_stamps)} 条')
    if lid_first_widths:
        w, h, s = lid_first_widths[0]
        print(f'  首帧: width={w} height={h} step={s} ({w*h} 点)')
    print(f'  字段: {lid_field_names}')

    field_names = {f[0] for f in (lid_field_names or [])}
    if 'ring' not in field_names:
        warnings.append('点云缺 ring 字段 — LIO 的 scan-line 检测会退化')
    if 'time' not in field_names and 'timestamp' not in field_names and 't' not in field_names:
        warnings.append('点云缺 per-point time/timestamp — 去畸变失效, 地图会飘')
    if 'intensity' not in field_names:
        warnings.append('点云缺 intensity (低优先, 不影响建图但 RViz 看不出灰度)')

    print(f'\nIMU: {len(imu_stamps)} 条')

    # ===== 4. 单调性 =====
    def _check_mono(stamps, name):
        if len(stamps) < 2:
            return
        loop = sum(1 for i in range(1, len(stamps)) if stamps[i] < stamps[i-1])
        if loop:
            issues.append(f'{name} header.stamp 非单调 ({loop} 处倒退) - bag 可能录串了')

    _check_mono(lid_stamps, '点云')
    _check_mono(imu_stamps, 'IMU')

    # ===== 5. 频率 =====
    def _stats(stamps, name, want_min_hz):
        if len(stamps) < 2:
            return
        s = sorted(stamps)
        dur = s[-1] - s[0]
        hz = (len(s) - 1) / dur if dur > 0 else 0
        diffs = [s[i+1] - s[i] for i in range(len(s)-1)]
        diffs.sort()
        med = diffs[len(diffs)//2]
        print(f'  {name}: 总时长={dur:.2f}s 平均={hz:.1f} Hz 中位间隔={med*1000:.1f} ms')
        if hz < want_min_hz:
            warnings.append(f'{name} 频率 {hz:.1f} Hz < 推荐 {want_min_hz} Hz')

    print('\n频率:')
    _stats(lid_stamps, '点云', 5.0)
    _stats(imu_stamps, 'IMU ', 50.0)

    # ===== 6. 时间重叠 =====
    if lid_stamps and imu_stamps:
        l0, l1 = min(lid_stamps), max(lid_stamps)
        i0, i1 = min(imu_stamps), max(imu_stamps)
        overlap = max(0.0, min(l1, i1) - max(l0, i0))
        union = max(l1, i1) - min(l0, i0)
        ratio = overlap / union if union > 0 else 0
        print(f'\n时间重叠: {overlap:.2f}s / {union:.2f}s = {ratio*100:.0f}%')
        if ratio < 0.3:
            issues.append(f'IMU 与点云时间区间重叠 {ratio*100:.0f}% (<30%) — IMU 和点云基本不重合, LIO 没法初始化')
        elif ratio < 0.7:
            warnings.append(f'IMU 与点云时间区间重叠 {ratio*100:.0f}% (<70%) — bag 头尾几秒可能没数据, IMU 初始化或末尾建图可能受影响')

    return _print_summary(issues, warnings)


def _print_summary(issues, warnings) -> int:
    print('\n' + '=' * 60)
    if issues:
        print(f'❌ 致命问题 ({len(issues)}):')
        for i in issues:
            print(f'  - {i}')
    if warnings:
        print(f'⚠️  警告 ({len(warnings)}):')
        for w in warnings:
            print(f'  - {w}')
    if not issues and not warnings:
        print('✅ 所有检查通过, bag 可以喂给 FAST-LIO-SAM')
    elif not issues:
        print('🟡 有警告但能跑, 建图质量可能下降')
    else:
        print('🛑 有致命问题, 不要跑了, 先修')
    print('=' * 60)
    return 1 if issues else 0


if __name__ == '__main__':
    sys.exit(main())
