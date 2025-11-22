from setuptools import setup
import os
from glob import glob

package_name = 'keyboard_mecanum'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@todo.todo',
    description='麦轮小车键控和二维码识别ROS2包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = keyboard_mecanum.keyboard_control_node:main',
            'qr_scanner = keyboard_mecanum.qr_scanner_node:main',
        ],
    },
)

