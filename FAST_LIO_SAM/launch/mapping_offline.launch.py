"""
fast-lio-sam 离线 bag 直读建图 (data_mode=1, 不依赖 ros2 bag play).

与 mapping_bag.launch.py 的区别:
  - 不起 `ros2 bag play`; 节点自己用 rosbag2_cpp::Reader 顺序读 bag, 逐帧 lock-step
    喂给 SLAM, 零丢帧, 跑满 CPU (可快于或慢于实时, 取决于机器).
  - bag 读完后节点自行落 PCD 并退出, launch 监听到节点退出后 shutdown.
  - 无需 use_sim_time / --clock / 停 driver service (没有实时时钟一说).

存储后端 (sqlite3 / mcap) 自动识别, 无需指定.

用法:
  ros2 launch fast_lio_sam mapping_offline.launch.py \\
      bag:=/path/to/rosbag2_dir \\
      config_file:=/path/to/airy_test_no_extr_est.yaml

  # 带 RViz:
  ros2 launch fast_lio_sam mapping_offline.launch.py bag:=... config_file:=... rviz:=true

注意:
  - config_file 里的 common.lid_topic / common.imu_topic 必须与 bag 内真实 topic 一致,
    否则 reader 取不到消息, 主循环空转直到 bag 读完即退出 (日志会打印各 topic 计数).
  - PCD 落盘由 config 里的 pcd_save / savePCD 相关项控制, 与在线建图一致.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='是否启动 RViz'
    )

    # data_mode / bag_path 用 CLI 覆盖 config, 这样同一份 config 既能在线又能离线.
    fastlio = Node(
        package='fast_lio_sam',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'common.data_mode': 1},
            {'common.bag_path': LaunchConfiguration('bag')},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 节点退出 (bag 读完 + 落 PCD 后自行 SIGINT 退出) -> 关 launch.
    on_node_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=fastlio,
            on_exit=[
                LogInfo(msg='[mapping_offline] fastlio 已退出 (PCD 已落盘), 关闭 launch'),
                Shutdown(reason='offline bag finished'),
            ],
        )
    )

    return LaunchDescription([
        arg_bag,
        arg_config,
        arg_rviz,
        fastlio,
        rviz,
        on_node_exit,
    ])
