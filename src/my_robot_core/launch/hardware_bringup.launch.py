import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from my_robot_core.laser_filter_node import DEFAULT_TILTED_SLOPE_SCAN_RANGE_M

def generate_launch_description():
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    ekf_params_file = os.path.join(my_robot_core_dir, 'config', 'ekf_planar_params.yaml')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否随硬件入口自动启动 RViz'
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
        description='是否让 EKF 融合 /rgbd/odom；视觉里程计稳定前保持 false'
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
            },
        ],
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # 🌟 骨架一：底盘投影(footprint) 到 底盘中心(link)
    # 当前实车量测：base_link 比 base_footprint 高 0.10m
    base_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_link_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.10',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
            '--frame-id', 'base_footprint', 
            '--child-frame-id', 'base_link'
        ],
        output='screen'
    )

    # 🌟 骨架二：底盘中心(link) 到 雷达(laser)
    # 当前实车量测：雷达在 base_link 前方 0.09m，相对 base_link 高 0.05m，反装 yaw=π
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_to_laser_tf',
        arguments=[
            '--x', '0.09', '--y', '0.0', '--z', '0.05',
            '--yaw', '3.14159', '--pitch', '0.0', '--roll', '0.0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'laser'
        ],
        output='screen'
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

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_core_dir, 'launch', 'rviz_nav_slam.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_rviz,
        declare_attitude_in_degrees,
        declare_enable_ekf,
        declare_base_driver_publish_tf,
        declare_ekf_use_visual_odom,
        declare_enable_laser_tilt_gate,
        declare_tilt_roll_offset,
        declare_tilt_pitch_offset,
        declare_tilted_safety_max_range,
        declare_tilted_scan_mode,
        declare_startup_tilt_bootstrap_enabled,
        declare_startup_tilt_bootstrap_max_frames,
        declare_startup_tilt_bootstrap_max_duration,
        declare_startup_tilt_bootstrap_max_angle,
        base_driver_node,
        ekf_node,
        sllidar_launch,
        base_link_tf_node,  # 添加骨架一
        laser_tf_node,      # 添加骨架二
        laser_filter_node,
        rviz_launch
    ])
