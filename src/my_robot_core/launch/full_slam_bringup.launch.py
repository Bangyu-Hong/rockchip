#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全系统一键启动 —— 未知地图（SLAM 建图）模式
============================================
适用场景：第一次进入新场地，边建图边搜球。

启动内容：
  底盘驱动   base_driver              → /cmd_vel、/odom、/imu/data、odom→base_footprint
  EKF        planar_ekf_node          → 可选 /odometry/filtered、odom→base_footprint
  激光雷达   sllidar_c1               → /scan
  静态TF     base_footprint→base_link→laser
  SLAM       slam_toolbox             → /map（实时建图）
  Nav2       navigation_launch        → 路径规划（无 AMCL）
  深度相机   orbbec gemini            → /camera/...
  视觉节点   yolo_3d_node             → /target_ball_position_stamped
  机械臂驱动 jibot1_driver            → /jibot1/raw_cmd
  抓取节点   jibot1_pick_place        → /jibot1/pick_place
  主控节点   main_control             → 状态机（原地旋转搜索）
  RViz       nav_slam.rviz            → 建图 / 导航调试界面

建图完成后保存地图：
  ros2 run nav2_map_server map_saver_cli -f /userdata/ros2_ws/maps/my_room

命令行覆盖串口（默认 /dev/ttyS3）：
  ros2 launch my_robot_core full_slam_bringup.launch.py port:=/dev/ttyUSB0

无界面启动：
  ros2 launch my_robot_core full_slam_bringup.launch.py rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from my_robot_core.laser_filter_node import DEFAULT_TILTED_SLOPE_SCAN_RANGE_M
from my_robot_core.slam_scan_matching_guard import (
    DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD,
)


def generate_launch_description():
    # ── 包路径 ──────────────────────────────────────────────────────────────
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    sllidar_ros2_dir  = get_package_share_directory('sllidar_ros2')
    nav2_bringup_dir  = get_package_share_directory('nav2_bringup')
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    jibot1_driver_dir = get_package_share_directory('jibot1_driver')

    slam_params_file  = os.path.join(my_robot_core_dir, 'config', 'my_slam_params.yaml')
    teb_params_file   = os.path.join(my_robot_core_dir, 'config', 'nav2_teb_params.yaml')
    ekf_params_file = os.path.join(my_robot_core_dir, 'config', 'ekf_planar_params.yaml')
    jibot1_params_file = os.path.join(jibot1_driver_dir, 'config', 'jibot1_params.yaml')
    camera_frame_id = 'robot_camera_color_optical_frame'

    # ── 命令行参数 ──────────────────────────────────────────────────────────
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyS3',
        description='机械臂 ESP32 串口设备路径'
    )
    declare_baud = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='机械臂串口波特率'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否随整机建图入口自动启动 RViz'
    )
    declare_attitude_in_degrees = DeclareLaunchArgument(
        'attitude_in_degrees',
        default_value='false',
        description='下位机上报 roll/pitch 是否为角度制；false 表示弧度制'
    )
    declare_enable_ekf = DeclareLaunchArgument(
        'enable_ekf',
        default_value='false',
        description='是否启动 2D EKF；默认关闭，底盘串口数据直接发布 /odom 和 odom->base_footprint'
    )
    declare_base_driver_publish_tf = DeclareLaunchArgument(
        'base_driver_publish_tf',
        default_value='true',
        description='是否让 base_driver 直接发布 odom TF；启用 EKF 时应显式设为 false'
    )
    declare_ekf_use_visual_odom = DeclareLaunchArgument(
        'ekf_use_visual_odom',
        default_value='false',
        description='是否让 EKF 低权重融合 /rgbd/odom 的视觉速度辅助；默认关闭'
    )
    declare_ekf_visual_twist_std = DeclareLaunchArgument(
        'ekf_visual_twist_std',
        default_value='0.35',
        description='EKF 融合 RGB-D 视觉速度的标准差；越大占比越低'
    )
    declare_enable_laser_tilt_gate = DeclareLaunchArgument(
        'enable_laser_tilt_gate',
        default_value='true',
        description='是否启用基于 /base/tilt 的激光门控'
    )
    declare_tilt_roll_offset = DeclareLaunchArgument(
        'tilt_roll_offset_deg',
        default_value='0.0',
        description='激光门控使用的 roll 静态零偏，单位 deg'
    )
    declare_tilt_pitch_offset = DeclareLaunchArgument(
        'tilt_pitch_offset_deg',
        default_value='0.0',
        description='激光门控使用的 pitch 静态零偏，单位 deg'
    )
    declare_tilted_safety_max_range = DeclareLaunchArgument(
        'tilted_safety_max_range',
        default_value=str(DEFAULT_TILTED_SLOPE_SCAN_RANGE_M),
        description='车体倾斜时近距雷达保留的最大扫描距离，单位 m'
    )
    declare_tilted_scan_mode = DeclareLaunchArgument(
        'tilted_scan_mode',
        default_value='all_maps',
        description='倾斜时雷达分流模式：all_maps=所有地图仅吃近距，safety_only=仅局部安全雷达吃近距'
    )
    declare_enable_slam_scan_matching_guard = DeclareLaunchArgument(
        'enable_slam_scan_matching_guard',
        default_value=str(DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD).lower(),
        description='倾斜时是否自动关闭 slam_toolbox use_scan_matching，水平后再打开'
    )
    declare_enable_rgbd_odom = DeclareLaunchArgument(
        'enable_rgbd_odom',
        default_value='false',
        description='是否启动 RGB-D 视觉里程计；默认关闭以降低板端负载'
    )
    declare_startup_tilt_bootstrap_enabled = DeclareLaunchArgument(
        'startup_tilt_bootstrap_enabled',
        default_value='true',
        description='启动阶段轻微不水平时是否短暂放行 /scan_filtered'
    )
    declare_startup_tilt_bootstrap_max_frames = DeclareLaunchArgument(
        'startup_tilt_bootstrap_max_frames',
        default_value='3',
        description='启动阶段最多放行到 /scan_filtered 的帧数'
    )
    declare_startup_tilt_bootstrap_max_duration = DeclareLaunchArgument(
        'startup_tilt_bootstrap_max_duration_sec',
        default_value='2.0',
        description='启动阶段放行 /scan_filtered 的最长时间，单位 s'
    )
    declare_startup_tilt_bootstrap_max_angle = DeclareLaunchArgument(
        'startup_tilt_bootstrap_max_angle_deg',
        default_value='5.0',
        description='启动阶段允许放行 /scan_filtered 的最大轻微倾角，单位 deg'
    )

    # ── 1. 底盘驱动 ─────────────────────────────────────────────────────────
    base_driver_node = Node(
        package='my_robot_core',
        executable='base_driver',
        name='base_driver',
        output='screen',
        parameters=[{
            'attitude_in_degrees': ParameterValue(
                LaunchConfiguration('attitude_in_degrees'),
                value_type=bool,
            ),
            'publish_tf': ParameterValue(
                LaunchConfiguration('base_driver_publish_tf'),
                value_type=bool,
            ),
        }],
    )

    ekf_node = Node(
        package='my_robot_core',
        executable='planar_ekf_node',
        name='planar_ekf_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ekf')),
        parameters=[
            ekf_params_file,
            {
                'use_visual_odom': ParameterValue(
                    LaunchConfiguration('ekf_use_visual_odom'),
                    value_type=bool,
                ),
                'visual_twist_std': ParameterValue(
                    LaunchConfiguration('ekf_visual_twist_std'),
                    value_type=float,
                ),
            },
        ],
    )

    # ── 2. 激光雷达（SLAMTEC C1，固定串口别名 /dev/rplidar）────────────────
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # ── 3. 静态 TF 骨架 ─────────────────────────────────────────────────────
    # base_footprint（地面投影）→ base_link（当前实车量测：离地 100mm）
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_link_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.10',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
        output='screen',
    )

    # base_link → laser（雷达反装，yaw=π；前方 9cm，相对 base_link 高 5cm）
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_to_laser_tf',
        arguments=[
            '--x', '0.09', '--y', '0.0', '--z', '0.05',
            '--yaw', '3.14159', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        output='screen',
    )

    laser_filter_node = Node(
        package='my_robot_core',
        executable='laser_filter',
        name='laser_filter_node',
        output='screen',
        parameters=[{
            'enable_tilt_gate': ParameterValue(
                LaunchConfiguration('enable_laser_tilt_gate'),
                value_type=bool,
            ),
            'roll_offset_deg': ParameterValue(
                LaunchConfiguration('tilt_roll_offset_deg'),
                value_type=float,
            ),
            'pitch_offset_deg': ParameterValue(
                LaunchConfiguration('tilt_pitch_offset_deg'),
                value_type=float,
            ),
            'tilted_safety_max_range': ParameterValue(
                LaunchConfiguration('tilted_safety_max_range'),
                value_type=float,
            ),
            'tilted_scan_mode': ParameterValue(
                LaunchConfiguration('tilted_scan_mode'),
                value_type=str,
            ),
            'startup_tilt_bootstrap_enabled': ParameterValue(
                LaunchConfiguration('startup_tilt_bootstrap_enabled'),
                value_type=bool,
            ),
            'startup_tilt_bootstrap_max_frames': ParameterValue(
                LaunchConfiguration('startup_tilt_bootstrap_max_frames'),
                value_type=int,
            ),
            'startup_tilt_bootstrap_max_duration_sec': ParameterValue(
                LaunchConfiguration('startup_tilt_bootstrap_max_duration_sec'),
                value_type=float,
            ),
            'startup_tilt_bootstrap_max_angle_deg': ParameterValue(
                LaunchConfiguration('startup_tilt_bootstrap_max_angle_deg'),
                value_type=float,
            ),
        }],
    )

    # ── 4. SLAM 建图 ─────────────────────────────────────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    slam_scan_matching_guard_node = Node(
        package='my_robot_core',
        executable='slam_scan_matching_guard',
        name='slam_scan_matching_guard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_slam_scan_matching_guard')),
        parameters=[{
            'roll_offset_deg': ParameterValue(
                LaunchConfiguration('tilt_roll_offset_deg'),
                value_type=float,
            ),
            'pitch_offset_deg': ParameterValue(
                LaunchConfiguration('tilt_pitch_offset_deg'),
                value_type=float,
            ),
            'level_scan_matching_enabled': True,
            'tilted_scan_matching_enabled': False,
        }],
    )

    # ── 5. Nav2 路径规划（navigation_launch，无 AMCL，SLAM 直接提供地图）──
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file':  teb_params_file,
        }.items()
    )

    # ── 6. 深度相机（Orbbec Gemini）─────────────────────────────────────────
    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'gemini.launch.xml')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'enable_d2c_viewer': 'false',
            'enable_publish_extrinsic': 'false',
        }.items(),
    )

    rgbd_odom_node = Node(
        package='robot_vision',
        executable='rgbd_visual_odometry',
        name='rgbd_visual_odometry_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rgbd_odom')),
        parameters=[{
            'publish_tf': False,
            'odom_topic': '/rgbd/odom',
        }],
    )

    # ── 7. 视觉检测节点（YOLO + 3D 定位）───────────────────────────────────
    yolo_3d_node = Node(
        package='robot_vision',
        executable='yolo_3d_node',
        name='npu_3d_tracker_node',
        output='screen',
        parameters=[{'camera_frame_id': camera_frame_id}],
    )

    # ── 8. 机械臂底层驱动（串口通信，参数与 jibot1_launch.py 保持一致）─────
    jibot1_node = Node(
        package='jibot1_driver',
        executable='jibot1_node',
        name='jibot1_driver',
        output='screen',
        parameters=[
            jibot1_params_file,
            {
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            },
        ],
    )

    # ── 9. 抓取动作节点 ──────────────────────────────────────────────────────
    pick_place_node = Node(
        package='jibot1_driver',
        executable='pick_place',
        name='jibot1_pick_place',
        output='screen',
        parameters=[
            jibot1_params_file,
        ],
    )

    # ── 10. 主控状态机（SLAM 模式：原地旋转搜索，不使用巡逻路点）─────────
    main_control_node = Node(
        package='my_robot_core',
        executable='main_control',
        name='main_control_node',
        output='screen',
        parameters=[{
            'use_patrol': False,
            'camera_frame_id': camera_frame_id,
        }],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_core_dir, 'launch', 'rviz_nav_slam.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # 命令行参数声明（必须在最前面）
        declare_port,
        declare_baud,
        declare_rviz,
        declare_attitude_in_degrees,
        declare_enable_ekf,
        declare_base_driver_publish_tf,
        declare_ekf_use_visual_odom,
        declare_ekf_visual_twist_std,
        declare_enable_laser_tilt_gate,
        declare_tilt_roll_offset,
        declare_tilt_pitch_offset,
        declare_tilted_safety_max_range,
        declare_tilted_scan_mode,
        declare_enable_slam_scan_matching_guard,
        declare_enable_rgbd_odom,
        declare_startup_tilt_bootstrap_enabled,
        declare_startup_tilt_bootstrap_max_frames,
        declare_startup_tilt_bootstrap_max_duration,
        declare_startup_tilt_bootstrap_max_angle,
        # 基础硬件
        base_driver_node,
        ekf_node,
        sllidar_launch,
        base_link_tf,
        laser_tf,
        laser_filter_node,
        # 建图
        slam_toolbox_node,
        slam_scan_matching_guard_node,
        # 导航
        nav2_launch,
        # 感知
        camera_launch,
        rgbd_odom_node,
        yolo_3d_node,
        # 机械臂
        jibot1_node,
        pick_place_node,
        # 主控（最后启动，等待所有依赖就绪）
        main_control_node,
        # RViz 调试界面
        rviz_launch,
    ])
