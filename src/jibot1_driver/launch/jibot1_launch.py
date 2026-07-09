"""
Jibot1 机械臂 Launch 文件

启动方式:
  ros2 launch jibot1_driver jibot1_launch.py
  ros2 launch jibot1_driver jibot1_launch.py port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('jibot1_driver')
    params_file = os.path.join(pkg_dir, 'config', 'jibot1_params.yaml')

    return LaunchDescription([
        # 可通过命令行覆盖串口
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyS3',
            description='ESP32 串口设备路径'),

        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='串口波特率'),

        # 驱动节点
        Node(
            package='jibot1_driver',
            executable='jibot1_node',
            name='jibot1_driver',
            output='screen',
            parameters=[
                params_file,
                {
                    'port': LaunchConfiguration('port'),
                    'baud': LaunchConfiguration('baud'),
                },
            ],
        ),

        # 拾取放置节点
        Node(
            package='jibot1_driver',
            executable='pick_place',
            name='jibot1_pick_place',
            output='screen',
            parameters=[
                params_file,
            ],
        ),
    ])
