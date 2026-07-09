import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jibot1_driver'
package_dir = os.path.dirname(os.path.realpath(__file__))


def source_file(*parts):
    """Return a source path relative to the active setup.py working directory."""
    return os.path.relpath(os.path.join(package_dir, *parts), os.getcwd())


def source_glob(*parts):
    return [
        os.path.relpath(path, os.getcwd())
        for path in glob(os.path.join(package_dir, *parts))
    ]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [source_file('resource', package_name)]),
        ('share/' + package_name, [source_file('package.xml')]),
        # launch 文件
        (os.path.join('share', package_name, 'launch'),
            source_glob('launch', '*launch.[pxy][yma]*')),
        # 配置文件
        (os.path.join('share', package_name, 'config'),
            source_glob('config', '*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 driver for Jibot1 ESP32 robotic arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jibot1_node = jibot1_driver.jibot1_node:main',
            'pick_place = jibot1_driver.pick_place_node:main',
        ],
    },
)
