#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist
import sys, select, termios, tty

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/,  : forward/backward
j/l  : turn left/right
a/d  : simulated strafe left/right (模拟横移)
u/m  : forward-left/backward-left
o/.  : forward-right/backward-right
h    : toggle strafe simulation mode

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

注意：当前使用"旋转+前进"模拟横移
真正横移需要升级ESP32固件

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0),      # forward
    'o': (1, 0, -1),     # forward + turn right
    'j': (0, 0, 1),      # turn left
    'l': (0, 0, -1),     # turn right
    'u': (1, 0, 1),      # forward + turn left
    ',': (-1, 0, 0),     # backward
    '.': (-1, 0, -1),    # backward + turn right
    'm': (-1, 0, 1),     # backward + turn left
    'a': (0.3, 0, 0.8),  # 模拟左横移：小幅前进+左转
    'd': (0.3, 0, -0.8), # 模拟右横移：小幅前进+右转
    'I': (1, 0, 0),
    'O': (1, 0, -1),
    'J': (0, 0, 1),
    'L': (0, 0, -1),
    'U': (1, 0, 1),
    'M': (-1, 0, 1),
    'A': (0.3, 0, 0.8),
    'D': (0.3, 0, -0.8),
}

speedBindings = {
    'Q': (1.1, 1.1),
    'Z': (.9, .9),
    'W': (1.1, 1),
    'X': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}



class Yahboom_Keybord(Node):
	def __init__(self,name):
		super().__init__(name)
		self.pub = self.create_publisher(Twist,'cmd_vel',1000)
		self.declare_parameter("linear_speed_limit",1.0)
		self.declare_parameter("angular_speed_limit",5.0)
		self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.settings = termios.tcgetattr(sys.stdin)
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist: key = sys.stdin.read(1)
		else: key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)		
	
def main():
	rclpy.init()
	yahboom_keyboard = Yahboom_Keybord("yahboom_keyboard_ctrl")
	(speed, turn) = (0.2, 1.0)
	(x, th) = (0, 0)
	status = 0
	stop = False
	count = 0
	strafe_mode = 1  # 模拟横移模式: 1=弧线, 2=原地转
	twist = Twist()
	
	# 横移模拟参数
	strafe_configs = {
		1: {'forward': 0.3, 'turn': 0.8},   # 模式1: 小幅前进+转向（弧线横移）
		2: {'forward': 0.0, 'turn': 1.0},   # 模式2: 纯旋转（原地转向）
	}
	
	try:
		print(msg)
		print(yahboom_keyboard.vels(speed, turn))
		print("Strafe simulation mode: {} (press 'h' to switch)".format(strafe_mode))
		while (1):
			key = yahboom_keyboard.getKey()
			if key == "h" or key == "H":
				strafe_mode = 2 if strafe_mode == 1 else 1
				print("Strafe mode switched to: {}".format(strafe_mode))
				# 更新 a/d 键映射
				cfg = strafe_configs[strafe_mode]
				moveBindings['a'] = (cfg['forward'], 0, cfg['turn'])
				moveBindings['d'] = (cfg['forward'], 0, -cfg['turn'])
				moveBindings['A'] = (cfg['forward'], 0, cfg['turn'])
				moveBindings['D'] = (cfg['forward'], 0, -cfg['turn'])
			if key == "s" or key == "S":
				print ("stop keyboard control: {}".format(not stop))
				stop = not stop
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][2]
				count = 0	
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				count = 0
				if speed > yahboom_keyboard.linenar_speed_limit: 
					speed = yahboom_keyboard.linenar_speed_limit
					print("Linear speed limit reached!")
				if turn > yahboom_keyboard.angular_speed_limit: 
					turn = yahboom_keyboard.angular_speed_limit
					print("Angular speed limit reached!")
				print(yahboom_keyboard.vels(speed, turn))
				if (status == 14): print(msg)
				status = (status + 1) % 15
			elif key == ' ': (x, th) = (0, 0)
			else:
				count = count + 1
				if count > 4: (x, th) = (0, 0)
				if (key == '\x03'): break
			twist.linear.x = speed * x
			twist.linear.y = 0.0
			twist.angular.z = turn * th
			if not stop: yahboom_keyboard.pub.publish(twist)
			if stop:yahboom_keyboard.pub.publish(Twist())
	except Exception as e: print(e)
	finally: yahboom_keyboard.pub.publish(Twist())
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, yahboom_keyboard.settings)
	yahboom_keyboard.destroy_node()
	rclpy.shutdown()
