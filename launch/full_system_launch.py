#!/usr/bin/env python3
"""
完整系统启动文件
同时启动键控节点和二维码识别节点
适用于树莓派，启动监控时自动开启二维码识别
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 键控节点
        Node(
            package='keyboard_mecanum',
            executable='keyboard_control',
            name='keyboard_control',
            output='screen',
            parameters=[{
                'linear_speed_limit': 1.0,
                'angular_speed_limit': 5.0,
            }],
            # prefix='xterm -e',  # SSH环境下不需要，直接在当前终端运行
        ),
        
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

