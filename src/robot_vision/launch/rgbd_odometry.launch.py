import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    robot_vision_dir = get_package_share_directory('robot_vision')
    default_rviz_config = os.path.join(
        robot_vision_dir,
        'rviz',
        'rgbd_odometry.rviz',
    )

    declare_start_camera = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='是否同时启动 Orbbec 相机；整机 bringup 已启动相机时设为 false'
    )
    declare_color_topic = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/color/image_raw',
        description='RGB 图像话题'
    )
    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='深度图话题'
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='彩色相机内参话题'
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/rgbd/odom',
        description='RGB-D 视觉里程计输出话题'
    )
    declare_publish_rgbd_tf = DeclareLaunchArgument(
        'publish_rgbd_tf',
        default_value='false',
        description='是否发布 rgbd_odom -> camera 光学坐标系 TF；默认只发 /rgbd/odom'
    )
    declare_flip_depth_horizontal = DeclareLaunchArgument(
        'flip_depth_horizontal',
        default_value='true',
        description='是否按当前相机经验水平翻转深度图以对齐 RGB'
    )
    declare_min_depth = DeclareLaunchArgument(
        'min_depth_m',
        default_value='0.2',
        description='可靠最小深度，单位 m'
    )
    declare_max_depth = DeclareLaunchArgument(
        'max_depth_m',
        default_value='2.5',
        description='可靠最大深度，单位 m'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='是否打开视觉里程计 RViz 调试界面'
    )
    declare_publish_debug_image = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='false',
        description='是否发布 RGB-D 视觉里程计特征调试图'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='视觉里程计 RViz 配置文件路径'
    )

    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'gemini.launch.xml')
        ),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    rgbd_odom_node = Node(
        package='robot_vision',
        executable='rgbd_visual_odometry',
        name='rgbd_visual_odometry_node',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'publish_tf': ParameterValue(
                LaunchConfiguration('publish_rgbd_tf'),
                value_type=bool,
            ),
            'publish_debug_image': ParameterValue(
                LaunchConfiguration('publish_debug_image'),
                value_type=bool,
            ),
            'flip_depth_horizontal': ParameterValue(
                LaunchConfiguration('flip_depth_horizontal'),
                value_type=bool,
            ),
            'min_depth_m': ParameterValue(
                LaunchConfiguration('min_depth_m'),
                value_type=float,
            ),
            'max_depth_m': ParameterValue(
                LaunchConfiguration('max_depth_m'),
                value_type=float,
            ),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rgbd_odometry_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_start_camera,
        declare_color_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        declare_odom_topic,
        declare_publish_rgbd_tf,
        declare_flip_depth_horizontal,
        declare_min_depth,
        declare_max_depth,
        declare_rviz,
        declare_rviz_config,
        declare_publish_debug_image,
        camera_launch,
        rgbd_odom_node,
        rviz_node,
    ])
