#!/usr/bin/env python3
"""
二维码识别启动文件
仅启动二维码识别节点
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 二维码识别节点
        Node(
            package='keyboard_mecanum',
            executable='qr_scanner',
            name='qr_scanner',
            output='screen',
            parameters=[{
                'camera_width': 640,
                'camera_height': 480,
                'publish_rate': 10.0,
            }],
        ),
    ])

