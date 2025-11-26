import cv2
import numpy as np
import subprocess
import os
import signal
import time

# 创建命名管道（FIFO）
fifo_path = "/tmp/rpicam_fifo"
if os.path.exists(fifo_path):
    os.unlink(fifo_path)
os.mkfifo(fifo_path)

# 检查显示环境
display = os.environ.get('DISPLAY')
if not display:
    print("⚠️  警告：未检测到 DISPLAY 环境变量")
    print("   如果通过 SSH 运行，可能需要设置 X11 转发或使用 VNC")
    print("   继续运行，但 cv2.imshow() 可能无法显示窗口...")

# 启动 rpicam-vid 进程，输出到命名管道
# 使用 MJPEG 格式，这样 OpenCV 可以读取
print("📹 正在启动 rpicam-vid...")
rpicam_process = subprocess.Popen(
    [
        "rpicam-vid",
        "--width", "640",
        "--height", "480",
        "--codec", "mjpeg",
        "--output", fifo_path,
        "--timeout", "0",  # 无限运行
        "--nopreview",
        "--autofocus-mode", "continuous",  # 连续自动对焦
        "--lens-position", "0.0"  # 0.0表示自动，或设置固定值如5.0-10.0对焦远处
    ],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

# 检查进程是否启动成功
time.sleep(0.5)
if rpicam_process.poll() is not None:
    # 进程已经退出，读取错误信息
    stderr_output = rpicam_process.stderr.read().decode('utf-8', errors='ignore')
    stdout_output = rpicam_process.stdout.read().decode('utf-8', errors='ignore')
    print(f"❌ 错误：rpicam-vid 进程启动失败！")
    print(f"   退出码: {rpicam_process.returncode}")
    if stderr_output:
        print(f"   错误信息: {stderr_output}")
    if stdout_output:
        print(f"   输出信息: {stdout_output}")
    os.unlink(fifo_path)
    exit(1)

print("✅ rpicam-vid 进程已启动")

# 等待一下让管道建立连接
print("⏳ 等待管道建立连接...")
time.sleep(1)

# 使用 OpenCV 从命名管道读取视频流
print("📷 正在打开视频流...")
cap = cv2.VideoCapture(fifo_path)

# 创建 QR 检测器
detector = cv2.QRCodeDetector()

if not cap.isOpened():
    print("❌ 错误：无法打开视频流")
    # 读取进程错误信息
    if rpicam_process.poll() is not None:
        stderr_output = rpicam_process.stderr.read().decode('utf-8', errors='ignore')
        if stderr_output:
            print(f"   rpicam-vid 错误: {stderr_output}")
    rpicam_process.terminate()
    rpicam_process.wait()
    os.unlink(fifo_path)
    exit(1)

print("✅ 视频流已打开")
print("🎥 相机已启动，按 'q' 键退出")
print("   开始读取帧...")

def cleanup():
    """清理资源"""
    if cap:
        cap.release()
    if rpicam_process.poll() is None:
        rpicam_process.terminate()
        rpicam_process.wait()
    if os.path.exists(fifo_path):
        os.unlink(fifo_path)
    cv2.destroyAllWindows()

# 注册信号处理，确保程序退出时清理资源
signal.signal(signal.SIGINT, lambda s, f: (cleanup(), exit(0)))
signal.signal(signal.SIGTERM, lambda s, f: (cleanup(), exit(0)))

frame_count = 0
try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            frame_count += 1
            if frame_count == 1:
                print("⚠️  警告：无法读取帧，重试...")
            elif frame_count % 30 == 0:  # 每30次重试输出一次
                print(f"⚠️  仍然无法读取帧（已重试 {frame_count} 次）...")
                # 检查进程是否还在运行
                if rpicam_process.poll() is not None:
                    stderr_output = rpicam_process.stderr.read().decode('utf-8', errors='ignore')
                    print(f"❌ rpicam-vid 进程已退出（退出码: {rpicam_process.returncode}）")
                    if stderr_output:
                        print(f"   错误信息: {stderr_output}")
                    break
            time.sleep(0.1)
            continue
        
        # 成功读取第一帧
        if frame_count == 0:
            print(f"✅ 成功读取第一帧！图像尺寸: {frame.shape}")
        
        frame_count += 1
        
        # 图像预处理增强二维码识别
        # 1. 转灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 2. 直方图均衡化 - 增强对比度
        enhanced = cv2.equalizeHist(gray)
        
        # 3. 高斯模糊去噪（可选，用于显示效果）
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        # 尝试多种预处理结果检测
        # 先用原始彩色图检测
        data, bbox, _ = detector.detectAndDecode(frame)
        
        # 如果失败，尝试增强后的灰度图
        if not data:
            data, bbox, _ = detector.detectAndDecode(enhanced)
        
        if bbox is not None and data:
            print(f"\n✅ 检测到二维码: {data}")
            # 绘制边框
            pts = bbox.astype(int).reshape(-1, 1, 2)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
        
        # 尝试显示窗口（如果失败会继续运行但不显示）
        try:
            # 只显示原始彩色图像
            cv2.imshow('QR Scanner', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n👋 用户退出")
                break
        except cv2.error as e:
            if frame_count == 1:
                print(f"⚠️  无法显示窗口（可能是无图形界面环境）: {e}")
                print("   程序将继续运行，但不会显示图像窗口")
                print("   二维码检测结果仍会输出到终端")
        
except KeyboardInterrupt:
    print("\n⚠️  收到中断信号")
except Exception as e:
    print(f"\n❌ 发生错误: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("🧹 正在清理资源...")
    cleanup()
    print("✅ 清理完成")

