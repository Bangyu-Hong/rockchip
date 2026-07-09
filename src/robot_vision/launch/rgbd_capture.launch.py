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
    orbbec_launch_file = os.path.join(
        orbbec_camera_dir,
        'launch',
        'gemini.launch.xml',
    )

    declare_start_camera = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Whether to start the Orbbec Gemini camera driver',
    )
    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='/userdata/yolo_tracking/dataset_capture',
        description='Dataset output directory',
    )
    declare_session_name = DeclareLaunchArgument(
        'session_name',
        default_value='',
        description='Optional capture session name',
    )
    declare_frame_stride = DeclareLaunchArgument(
        'frame_stride',
        default_value='10',
        description='Save one training image every N synchronized RGB-D frames',
    )
    declare_save_depth_frames = DeclareLaunchArgument(
        'save_depth_frames',
        default_value='false',
        description='Whether to save matching raw depth PNG frames',
    )
    declare_max_frames = DeclareLaunchArgument(
        'max_frames',
        default_value='0',
        description='Stop after this many raw frames; 0 means unlimited until Ctrl+C',
    )
    declare_show_preview = DeclareLaunchArgument(
        'show_preview',
        default_value='false',
        description='Show OpenCV recording preview window',
    )
    declare_flip_depth_horizontal = DeclareLaunchArgument(
        'flip_depth_horizontal',
        default_value='true',
        description='Mirror saved depth frames to align with RGB frames',
    )
    declare_min_reliable_depth_m = DeclareLaunchArgument(
        'min_reliable_depth_m',
        default_value='0.2',
        description='Reliable lower depth bound recorded in camera_info.json',
    )
    declare_max_reliable_depth_m = DeclareLaunchArgument(
        'max_reliable_depth_m',
        default_value='2.5',
        description='Reliable upper depth bound recorded in camera_info.json',
    )

    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(orbbec_launch_file),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    recorder = Node(
        package='robot_vision',
        executable='rgbd_dataset_recorder',
        name='rgbd_dataset_recorder',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'session_name': LaunchConfiguration('session_name'),
            'frame_stride': ParameterValue(
                LaunchConfiguration('frame_stride'),
                value_type=int,
            ),
            'save_depth_frames': ParameterValue(
                LaunchConfiguration('save_depth_frames'),
                value_type=bool,
            ),
            'max_frames': ParameterValue(
                LaunchConfiguration('max_frames'),
                value_type=int,
            ),
            'show_preview': ParameterValue(
                LaunchConfiguration('show_preview'),
                value_type=bool,
            ),
            'flip_depth_horizontal': ParameterValue(
                LaunchConfiguration('flip_depth_horizontal'),
                value_type=bool,
            ),
            'min_reliable_depth_m': ParameterValue(
                LaunchConfiguration('min_reliable_depth_m'),
                value_type=float,
            ),
            'max_reliable_depth_m': ParameterValue(
                LaunchConfiguration('max_reliable_depth_m'),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        declare_start_camera,
        declare_output_dir,
        declare_session_name,
        declare_frame_stride,
        declare_save_depth_frames,
        declare_max_frames,
        declare_show_preview,
        declare_flip_depth_horizontal,
        declare_min_reliable_depth_m,
        declare_max_reliable_depth_m,
        camera_launch,
        recorder,
    ])
