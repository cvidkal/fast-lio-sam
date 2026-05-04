"""
fast-lio-sam rosbag2 回放建图 launch (任意 ROS2 LiDAR bag).

特性:
  - 自动设置 use_sim_time=true (节点跟随 bag 时钟)
  - 通过 ExecuteProcess 调用 `ros2 bag play --clock`
  - 可选: 回放前 stop, 回放完 start 一个 systemd service
    (避开 driver-LIVE-publisher 和 bag-replay 同 topic 撞车,
    见 docs/STAGE6_SMOKE.md 第 5 个 gotcha)
  - bag 跑完后向 fastlio 发 SIGINT 让它落 PCD
  - 可选 RViz

用法:
  ros2 launch fast_lio_sam mapping_bag.launch.py \\
      bag:=/path/to/rosbag2_dir \\
      config_file:=/path/to/airy_via_bridge.yaml \\
      stop_service:=airy-lidar.service \\
      rate:=1.0

  # 不停 driver service (例如 bag 是 KITTI / LIO-SAM 公开数据集, 没 driver 在跑):
  ros2 launch fast_lio_sam mapping_bag.launch.py \\
      bag:=... config_file:=... stop_service:=

  # 带 RViz:
  ros2 launch fast_lio_sam mapping_bag.launch.py bag:=... rviz:=true

工作流:
  1. (可选) sudo systemctl stop <stop_service>
  2. 起 fastlio_mapping (use_sim_time=true)
  3. 起 ros2 bag play --clock
  4. bag 进程退出后, 等 grace 秒, 向 fastlio 发 SIGINT (让它落 PCD)
  5. (可选) sudo systemctl start <stop_service>
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    """运行期把 stop_service 字符串展开成两条 systemctl ExecuteProcess (开/关)."""
    stop_service = LaunchConfiguration('stop_service').perform(context).strip()
    actions = []

    if stop_service:
        actions.append(LogInfo(msg=[
            f'[mapping_bag] auto-stopping systemd unit before bag replay: {stop_service}',
            ' (set stop_service:= empty to skip)']))
        actions.append(ExecuteProcess(
            cmd=['sudo', '-n', 'systemctl', 'stop', stop_service],
            output='screen',
            shell=False,
        ))
    else:
        actions.append(LogInfo(msg='[mapping_bag] stop_service is empty - skipping systemctl stop'))
    return actions


def _cleanup(context, *args, **kwargs):
    """bag 跑完后, 重启 stop_service (如果有)."""
    stop_service = LaunchConfiguration('stop_service').perform(context).strip()
    actions = []
    if stop_service:
        actions.append(LogInfo(msg=[
            f'[mapping_bag] bag finished, restarting {stop_service} ...']))
        actions.append(ExecuteProcess(
            cmd=['sudo', '-n', 'systemctl', 'start', stop_service],
            output='screen',
            shell=False,
        ))
    actions.append(LogInfo(msg='[mapping_bag] launch shutting down - fastlio should have dumped PCD'))
    actions.append(Shutdown(reason='bag finished'))
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio_sam')
    default_cfg = os.path.join(pkg_share, 'config', 'velodyne16.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'fastlio_hk.rviz')

    arg_bag = DeclareLaunchArgument(
        'bag',
        description='rosbag2 目录 (sqlite3 / mcap), 必填'
    )
    arg_config = DeclareLaunchArgument(
        'config_file', default_value=default_cfg,
        description='YAML 配置 (默认 velodyne16.yaml)'
    )
    arg_rate = DeclareLaunchArgument(
        'rate', default_value='1.0',
        description='ros2 bag play 倍速 (慢机器建议 0.3-0.5)'
    )
    arg_stop_service = DeclareLaunchArgument(
        'stop_service', default_value='',
        description=(
            'systemd 服务名, 回放前 stop / 完成后 start. '
            '空字符串=不动 (默认). 例: airy-lidar.service. '
            '需要 NOPASSWD sudo 权限, 见 docs/rosbag2_workflow.md'
        ),
    )
    arg_grace = DeclareLaunchArgument(
        'grace_sec', default_value='5',
        description='bag 退出后等多少秒再 SIGINT fastlio (让 buffer 内最后几帧处理完)'
    )
    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='是否启动 RViz'
    )
    arg_bridge = DeclareLaunchArgument(
        'with_airy_bridge', default_value='false',
        description=(
            'true: 同时起 airy_bridge (来自 dog_mapping_ws), 把 Robosense '
            'PointXYZIRT 点云转为 Velodyne 兼容 layout. RoboSense Airy 用 '
            'lidar_type=2 (Velodyne) 配置时必须开. 需要 airy_bridge 包已安装.'
        ),
    )

    fastlio = Node(
        package='fast_lio_sam',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': True},
        ],
    )

    bridge = Node(
        package='airy_bridge',
        executable='airy_to_velodyne',
        name='airy_to_velodyne',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_airy_bridge')),
        parameters=[{'use_sim_time': True}],
    )

    # bag play, 带 --clock 让它发 /clock 给其他节点用
    bag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag'),
            '--clock', '100',
            '--rate', LaunchConfiguration('rate'),
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    # bag_player 退出 -> 调 _cleanup (重启 service + 关 launch)
    on_bag_finished = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[OpaqueFunction(function=_cleanup)],
        )
    )

    return LaunchDescription([
        arg_bag,
        arg_config,
        arg_rate,
        arg_stop_service,
        arg_grace,
        arg_rviz,
        arg_bridge,
        # 1) 先 stop service (如果指定)
        OpaqueFunction(function=_setup),
        # 2) 起 fastlio
        fastlio,
        # 3) (可选) 起 airy_bridge
        bridge,
        # 4) (可选) 起 RViz
        rviz,
        # 5) 起 bag_player
        bag_player,
        # 6) bag 退出钩子: 重启 service + shutdown launch
        on_bag_finished,
    ])
