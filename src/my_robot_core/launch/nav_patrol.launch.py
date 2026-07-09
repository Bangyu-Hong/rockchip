import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from my_robot_core.laser_filter_node import DEFAULT_TILTED_SLOPE_SCAN_RANGE_M

def generate_launch_description():
    # 1. 找到各个包的绝对路径
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') # 系统大佬的包

    # 2. 指定你的配置文件和【已经建好的地图】路径
    teb_params_file = os.path.join(my_robot_core_dir, 'config', 'nav2_teb_params.yaml')
    
    # ⚠️ 注意：这里写死了你保存的地图路径。确保该目录下有这两个文件：my_room.yaml 和 my_room.pgm
    map_file = '/userdata/ros2_ws/maps/my_room.yaml'

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否通过 hardware_bringup 自动启动 RViz'
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

    # ========================================================
    # 3. 开始召唤 (纯硬件 + 完整版 Nav2)
    # ========================================================

    # [终端 1]: 启动躯体与感觉 (调用 hardware_bringup.launch.py，绝对不能带 SLAM！)
    hardware_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_core_dir, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'attitude_in_degrees': LaunchConfiguration('attitude_in_degrees'),
            'enable_ekf': LaunchConfiguration('enable_ekf'),
            'base_driver_publish_tf': LaunchConfiguration('base_driver_publish_tf'),
            'ekf_use_visual_odom': LaunchConfiguration('ekf_use_visual_odom'),
            'enable_laser_tilt_gate': LaunchConfiguration('enable_laser_tilt_gate'),
            'tilt_roll_offset_deg': LaunchConfiguration('tilt_roll_offset_deg'),
            'tilt_pitch_offset_deg': LaunchConfiguration('tilt_pitch_offset_deg'),
            'tilted_safety_max_range': LaunchConfiguration('tilted_safety_max_range'),
            'tilted_scan_mode': LaunchConfiguration('tilted_scan_mode'),
            'startup_tilt_bootstrap_enabled': LaunchConfiguration(
                'startup_tilt_bootstrap_enabled'
            ),
            'startup_tilt_bootstrap_max_frames': LaunchConfiguration(
                'startup_tilt_bootstrap_max_frames'
            ),
            'startup_tilt_bootstrap_max_duration_sec': LaunchConfiguration(
                'startup_tilt_bootstrap_max_duration_sec'
            ),
            'startup_tilt_bootstrap_max_angle_deg': LaunchConfiguration(
                'startup_tilt_bootstrap_max_angle_deg'
            ),
        }.items()
    )

    # [终端 2]: 启动完整版领航员 Nav2 (调用 bringup_launch.py，自带 AMCL 定位和地图服务器)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'map': map_file,               # 👈 把地图喂给它
            'params_file': teb_params_file  # 👈 TEB 局部规划器参数
        }.items()
    )

    # 4. 打包发送给系统，点火！
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
        hardware_cmd,
        nav2_cmd
    ])
