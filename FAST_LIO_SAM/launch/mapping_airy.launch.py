"""
fast-lio-sam 实时建图 launch (RoboSense Airy, 原生 PointXYZIRT 路径).
配合 rslidar_sdk 的 POINT_TYPE=XYZIRT + ENABLE_IMU_DATA_PARSE=ON 编译,
直接订阅 /rslidar_points + /rslidar_imu_data, 不需要 airy_bridge 中转.

  # 实时:
  ros2 launch fast_lio_sam mapping_airy.launch.py

  # bag 回放 (use mapping_bag.launch.py instead):
  ros2 launch fast_lio_sam mapping_bag.launch.py \\
       bag:=<path> config_file:=<...>/airy.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio_sam')
    default_cfg = os.path.join(pkg_share, 'config', 'airy.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'fastlio_hk.rviz')

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='LIVE 建图设 false; bag 回放设 true (但建议直接用 mapping_bag.launch.py)'
    )
    arg_config = DeclareLaunchArgument(
        'config_file', default_value=default_cfg,
        description='YAML 配置 (默认 airy.yaml)'
    )
    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz'
    )
    arg_rviz_cfg = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz,
        description='RViz 配置文件'
    )

    fastlio = Node(
        package='fast_lio_sam',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_config,
        arg_rviz,
        arg_rviz_cfg,
        fastlio,
        rviz,
    ])
