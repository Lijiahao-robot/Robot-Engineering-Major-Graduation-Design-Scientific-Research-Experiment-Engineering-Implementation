from setuptools import setup
import os
from glob import glob

package_name = 'rm_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装启动文件
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='rm_robot@example.com',
    description='RoboMaster ROS2 Node Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 关键：注册可执行文件，后续直接用 ros2 run 启动
    entry_points={
        'console_scripts': [
            # 底盘控制节点
            'chassis_control = rm_robot.chassis_control:main',
            # 机械臂控制节点
            'arm_control = rm_robot.arm_control:main',
            # 视觉跟踪节点
            'vision_tracking = rm_robot.vision_tracking:main',
            # 串口桥节点（ROS2 ↔ STM32）
            'serial_bridge = rm_robot.serial_bridge:main',
        ],
    },
)
