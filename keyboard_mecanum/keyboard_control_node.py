#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
长按式键控小车节点 - 支持麦轮横移
按住按键持续移动，松开即停止
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import sys
import termios
import tty
import select

# 控制说明
msg = """
=====================================
麦轮小车键控系统 (长按模式)
=====================================
移动控制:
   u    i    o
   j    k    l
   m    ,    .

i/,  : 前进/后退
j/l  : 左转/右转
u/m  : 左前/左后
o/.  : 右前/右后

麦轮横移:
a    : 左横移
d    : 右横移

舵机S1控制:
1    : S1 向左转
2    : S1 回中位
3    : S1 向右转

速度调整:
q/z : 增加/降低最大速度 10%
w/x : 增加/降低线速度 10%
e/c : 增加/降低角速度 10%

停止:
空格键 或 k : 停止
CTRL-C : 退出程序

=====================================
"""

# 移动按键映射 (x, y, theta)
# x: 前后, y: 左右横移, theta: 旋转
moveBindings = {
    'i': (1, 0, 0),       # 前进
    'o': (1, 0, -1),      # 右前
    'j': (0, 0, 1),       # 左转
    'l': (0, 0, -1),      # 右转
    'u': (1, 0, 1),       # 左前
    ',': (-1, 0, 0),      # 后退
    '.': (-1, 0, -1),     # 右后
    'm': (-1, 0, 1),      # 左后
    'a': (0, 1, 0),       # 左横移 (真正麦轮横移)
    'd': (0, -1, 0),      # 右横移 (真正麦轮横移)
    'I': (1, 0, 0),
    'O': (1, 0, -1),
    'J': (0, 0, 1),
    'L': (0, 0, -1),
    'U': (1, 0, 1),
    'M': (-1, 0, 1),
    'A': (0, 1, 0),
    'D': (0, -1, 0),
}

# 速度调整按键映射
speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
    'Q': (1.1, 1.1),
    'Z': (0.9, 0.9),
    'W': (1.1, 1),
    'X': (0.9, 1),
    'E': (1, 1.1),
    'C': (1, 0.9),
}

# 舵机控制按键映射 (角度)
servoBindings = {
    '1': -45,  # S1向左
    '2': 0,    # S1中位
    '3': 45,   # S1向右
}


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_mecanum_control')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_s1_pub = self.create_publisher(Int32, 'servo_s1', 10)
        
        # 声明参数
        self.declare_parameter('linear_speed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)
        
        # 获取参数
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').value
        
        # 初始速度
        self.speed = 0.3
        self.turn = 1.5
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('麦轮键控节点已启动 (长按模式)')
        
    def getKey(self, timeout=0.1):
        """获取按键输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def vels(self):
        """返回当前速度信息"""
        return f"当前速度:\t线速度 {self.speed:.2f}\t角速度 {self.turn:.2f}"
    
    def run(self):
        """主循环"""
        try:
            print(msg)
            print(self.vels())
            
            # 当前移动状态
            x = 0
            y = 0
            th = 0
            
            while rclpy.ok():
                key = self.getKey(timeout=0.05)  # 50ms超时，提高响应速度
                
                # 处理移动按键
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    th = moveBindings[key][2]
                    
                # 处理速度调整按键
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    
                    # 限制最大速度
                    if self.speed > self.linear_speed_limit:
                        self.speed = self.linear_speed_limit
                        print("线速度已达上限!")
                    if self.turn > self.angular_speed_limit:
                        self.turn = self.angular_speed_limit
                        print("角速度已达上限!")
                    
                    print(self.vels())
                
                # 处理舵机控制按键
                elif key in servoBindings.keys():
                    servo_msg = Int32()
                    servo_msg.data = servoBindings[key]
                    self.servo_s1_pub.publish(servo_msg)
                    self.get_logger().info(f'舵机S1转到: {servoBindings[key]}°')
                
                # 处理停止按键
                elif key == ' ' or key == 'k' or key == 'K':
                    x = 0
                    y = 0
                    th = 0
                
                # 退出
                elif key == '\x03':  # Ctrl+C
                    break
                
                # 如果没有按键，则停止（这是长按模式的核心）
                elif key == '':
                    x = 0
                    y = 0
                    th = 0
                
                # 发布速度消息
                twist = Twist()
                twist.linear.x = self.speed * x
                twist.linear.y = self.speed * y  # 麦轮横移
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.turn * th
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'发生错误: {e}')
        finally:
            # 停止小车
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

