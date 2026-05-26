"""
fast-lio-sam 实时建图 launch (Velodyne 16 线).
ROS2 port. 用法:

  # 实时:
  ros2 launch fast_lio_sam mapping_velodyne16.launch.py

  # 回放 rosbag2:
  ros2 launch fast_lio_sam mapping_velodyne16.launch.py use_sim_time:=true
  # 然后另开一个 shell:
  ros2 bag play --clock <bag_dir>
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio_sam')
    default_cfg = os.path.join(pkg_share, 'config', 'velodyne16.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'fastlio_hk.rviz')

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='True 跟随 ros2 bag 时钟; 实时建图设 false'
    )
    arg_config = DeclareLaunchArgument(
        'config_file', default_value=default_cfg,
        description='YAML 配置 (可换 mid360.yaml / airy_test_no_extr_est.yaml 等)'
    )
    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz'
    )
    arg_rviz_cfg = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz,
        description='RViz 配置文件路径'
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
