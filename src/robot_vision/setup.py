from setuptools import find_packages, setup
import os                      # 👈 必须要有这一行
from glob import glob          # 👈 必须要有这一行

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🆕 把 launch 文件夹里的所有 .py 和 .xml 文件都拷贝到成品仓库里
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elf',
    maintainer_email='elf@todo.todo',
    description='ROS2 vision package for RGB-D ball detection and 3D target localization.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_3d_node = robot_vision.npu_3d_tracker:main',
            'rgbd_dataset_recorder = robot_vision.rgbd_dataset_recorder:main',
            'rgbd_visual_odometry = robot_vision.rgbd_visual_odometry_node:main',
        ],
    },
)
