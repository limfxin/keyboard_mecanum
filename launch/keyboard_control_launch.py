#!/usr/bin/env python3
"""
键控小车启动文件
启动键控节点（仅键控，不启动相机）
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
    ])

