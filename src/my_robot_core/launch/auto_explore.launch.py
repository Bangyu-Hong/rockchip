import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from my_robot_core.laser_filter_node import DEFAULT_TILTED_SLOPE_SCAN_RANGE_M
from my_robot_core.slam_scan_matching_guard import (
    DEFAULT_ENABLE_SLAM_SCAN_MATCHING_GUARD,
)

def generate_launch_description():
    # 1. 找到各个包的绝对路径
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    robot_vision_dir = get_package_share_directory('robot_vision')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') # 系统大佬的包
    # explore_lite_dir = get_package_share_directory('explore_lite') # 探索插件的包

    # 2. 找到我们自己写的 MPPI 配置文件
    teb_params_file = os.path.join(my_robot_core_dir, 'config', 'nav2_teb_params.yaml')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否通过 mapping_bringup 自动启动 RViz'
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
        description='是否让 EKF 低权重融合 /rgbd/odom 的视觉速度辅助；默认关闭，稳定后再显式开启'
    )
    declare_ekf_visual_twist_std = DeclareLaunchArgument(
        'ekf_visual_twist_std',
        default_value='0.35',
        description='EKF 融合 RGB-D 视觉速度的标准差；越大占比越低'
    )
    declare_enable_rgbd_odom = DeclareLaunchArgument(
        'enable_rgbd_odom',
        default_value='false',
        description='是否启动 Orbbec 相机和 RGB-D 视觉里程计；默认关闭以保证手动建图/Nav2 稳定'
    )
    declare_start_camera = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='启动 RGB-D 视觉里程计时是否同时启动 Orbbec 相机；相机已启动时设为 false'
    )
    declare_nav2_log_level = DeclareLaunchArgument(
        'nav2_log_level',
        default_value='info',
        description='传给 Nav2 navigation_launch.py 的日志等级；保持 info，避免 none 让 Nav2 节点启动即 abort'
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
    # 3. 开始召唤四大金刚 (相当于把你敲的 4 个终端命令写成代码)
    # ========================================================

    # [终端 1]: 启动躯体与大脑 (调用我们自己写的 mapping_bringup.launch.py)
    mapping_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_core_dir, 'launch', 'mapping_bringup.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'attitude_in_degrees': LaunchConfiguration('attitude_in_degrees'),
            'enable_ekf': LaunchConfiguration('enable_ekf'),
            'base_driver_publish_tf': LaunchConfiguration('base_driver_publish_tf'),
            'ekf_use_visual_odom': LaunchConfiguration('ekf_use_visual_odom'),
            'ekf_visual_twist_std': LaunchConfiguration('ekf_visual_twist_std'),
            'enable_laser_tilt_gate': LaunchConfiguration('enable_laser_tilt_gate'),
            'tilt_roll_offset_deg': LaunchConfiguration('tilt_roll_offset_deg'),
            'tilt_pitch_offset_deg': LaunchConfiguration('tilt_pitch_offset_deg'),
            'tilted_safety_max_range': LaunchConfiguration('tilted_safety_max_range'),
            'tilted_scan_mode': LaunchConfiguration('tilted_scan_mode'),
            'enable_slam_scan_matching_guard': LaunchConfiguration(
                'enable_slam_scan_matching_guard'
            ),
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

    # [终端 2]: 可选启动深度相机和 RGB-D 视觉里程计，发布 /rgbd/odom 给 EKF 试验性低权重融合
    rgbd_odom_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_vision_dir, 'launch', 'rgbd_odometry.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_rgbd_odom')),
        launch_arguments={
            'start_camera': LaunchConfiguration('start_camera'),
            'rviz': 'false',
            'odom_topic': '/rgbd/odom',
            'publish_rgbd_tf': 'false',
        }.items()
    )

    # [终端 3]: 启动领航员 Nav2 (调用系统的 navigation_launch.py，坚决不带 AMCL)
    # 并强制它吃下我们写的 MPPI 参数！
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': teb_params_file,
            'log_level': LaunchConfiguration('nav2_log_level'),
        }.items()
    )

    # [终端 4]: 启动自动探索插件
    # explore_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(explore_lite_dir, 'launch', 'explore.launch.py')
    #     )
    # )

    # 4. 打包发送给系统，同时点火！
    return LaunchDescription([
        declare_rviz,
        declare_attitude_in_degrees,
        declare_enable_ekf,
        declare_base_driver_publish_tf,
        declare_ekf_use_visual_odom,
        declare_ekf_visual_twist_std,
        declare_enable_rgbd_odom,
        declare_start_camera,
        declare_nav2_log_level,
        declare_enable_laser_tilt_gate,
        declare_tilt_roll_offset,
        declare_tilt_pitch_offset,
        declare_tilted_safety_max_range,
        declare_tilted_scan_mode,
        declare_enable_slam_scan_matching_guard,
        declare_startup_tilt_bootstrap_enabled,
        declare_startup_tilt_bootstrap_max_frames,
        declare_startup_tilt_bootstrap_max_duration,
        declare_startup_tilt_bootstrap_max_angle,
        mapping_cmd,
        rgbd_odom_cmd,
        nav2_cmd,
        # explore_cmd
    ])
