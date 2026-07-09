import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 找到 orbbec_camera 的 launch 文件路径
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    orbbec_launch_file = os.path.join(orbbec_camera_dir, 'launch', 'gemini.launch.xml')

    # 2. 包含相机的 launch (这就相当于你在终端敲 ros2 launch orbbec_camera gemini.launch.xml)
    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(orbbec_launch_file),
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

    # 3. 启动你的 3D 视觉追踪节点
    yolo_tracker_node = Node(
        package='robot_vision',
        executable='yolo_3d_node',  # 或者 npu_3d_tracker，取决于你在 setup.py 里的命名
        name='npu_3d_tracker_node',
        output='screen'
    )

    
    return LaunchDescription([
        camera_launch,
        yolo_tracker_node
    ])
