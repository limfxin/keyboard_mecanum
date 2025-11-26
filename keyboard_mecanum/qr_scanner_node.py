#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二维码识别ROS2节点
适用于树莓派，使用rpicam-vid + OpenCV进行二维码识别
识别结果通过ROS2话题发布
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import subprocess
import os
import signal
import time


class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner_node')
        
        # 创建发布者 - 发布二维码识别结果
        self.qr_pub = self.create_publisher(String, 'qr_code', 10)
        
        # 声明参数
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('debug_mode', False)  # 调试模式 - 显示图像窗口
        
        # 获取参数
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 命名管道路径
        self.fifo_path = "/tmp/rpicam_fifo"
        
        # 初始化
        self.rpicam_process = None
        self.cap = None
        self.detector = cv2.QRCodeDetector()
        self.last_qr_data = ""  # 保存上一次的识别结果
        
        # 创建定时器 - 定期读取相机帧
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('二维码识别节点已启动')
        
        if self.debug_mode:
            self.get_logger().info('⚙️ 调试模式已启用 - 将显示图像窗口（需要图形界面）')
        else:
            self.get_logger().info('📡 正常模式 - 后台运行，适合SSH环境')
        
        # 初始化相机
        self.init_camera()
        
    def init_camera(self):
        """初始化树莓派相机"""
        try:
            # 创建命名管道
            if os.path.exists(self.fifo_path):
                os.unlink(self.fifo_path)
            os.mkfifo(self.fifo_path)
            
            self.get_logger().info('正在启动 rpicam-vid...')
            
            # 启动 rpicam-vid 进程
            self.rpicam_process = subprocess.Popen(
                [
                    "rpicam-vid",
                    "--width", str(self.camera_width),
                    "--height", str(self.camera_height),
                    "--codec", "mjpeg",
                    "--output", self.fifo_path,
                    "--timeout", "0",  # 无限运行
                    "--nopreview",
                    "--autofocus-mode", "continuous",
                    "--lens-position", "0.0"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # 检查进程是否启动成功
            time.sleep(0.5)
            if self.rpicam_process.poll() is not None:
                stderr_output = self.rpicam_process.stderr.read().decode('utf-8', errors='ignore')
                self.get_logger().error(f'rpicam-vid 启动失败: {stderr_output}')
                return False
            
            self.get_logger().info('rpicam-vid 进程已启动')
            
            # 等待管道建立连接
            time.sleep(1)
            
            # 打开视频流
            self.get_logger().info('正在打开视频流...')
            self.cap = cv2.VideoCapture(self.fifo_path)
            
            if not self.cap.isOpened():
                self.get_logger().error('无法打开视频流')
                return False
            
            self.get_logger().info('✅ 相机初始化成功，开始识别二维码')
            return True
            
        except Exception as e:
            self.get_logger().error(f'相机初始化失败: {e}')
            return False
    
    def timer_callback(self):
        """定时器回调 - 读取帧并识别二维码"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        
        if not ret:
            return
        
        # 图像预处理增强二维码识别
        # 1. 转灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 2. 直方图均衡化 - 增强对比度
        enhanced = cv2.equalizeHist(gray)
        
        # 3. 高斯模糊去噪（可选，用于显示效果）
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        # 尝试多种预处理结果检测
        data = ""
        bbox = None
        
        # 先用原始彩色图检测
        try:
            data, bbox, _ = self.detector.detectAndDecode(frame)
        except cv2.error:
            pass  # 忽略OpenCV内部错误
        
        # 如果失败，尝试增强后的灰度图
        if not data:
            try:
                data, bbox, _ = self.detector.detectAndDecode(enhanced)
            except cv2.error:
                pass  # 忽略OpenCV内部错误
        
        # 如果检测到二维码且内容与上次不同
        if bbox is not None and data and data != self.last_qr_data:
            self.last_qr_data = data
            
            # 发布识别结果
            msg = String()
            msg.data = data
            self.qr_pub.publish(msg)
            
            # 打印到控制台
            self.get_logger().info(f'✅ 检测到二维码: {data}')
        
        # 调试模式 - 显示图像窗口（仅在有图形界面时）
        if self.debug_mode:
            try:
                cv2.imshow('QR Scanner - Original', frame)
                cv2.imshow('QR Scanner - Enhanced', enhanced)
                cv2.waitKey(1)
            except cv2.error:
                # SSH环境下无法显示窗口，忽略错误
                pass
    
    def cleanup(self):
        """清理资源"""
        self.get_logger().info('正在清理资源...')
        
        if self.cap:
            self.cap.release()
        
        if self.rpicam_process and self.rpicam_process.poll() is None:
            self.rpicam_process.terminate()
            self.rpicam_process.wait()
        
        if os.path.exists(self.fifo_path):
            try:
                os.unlink(self.fifo_path)
            except:
                pass
        
        cv2.destroyAllWindows()
        self.get_logger().info('清理完成')
    
    def __del__(self):
        """析构函数"""
        self.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

