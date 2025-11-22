# 麦轮小车键控系统

> 💡 **ROS2 Jazzy用户**: 本系统完全支持Jazzy！请查看 [ROS2_JAZZY说明.md](ROS2_JAZZY说明.md)

## ✨ 功能特点

### 1. **长按式键控** 🎮
- ✅ 按住按键持续移动，松开立即停止
- ✅ 与开源键控不同，无需按停止键
- ✅ 更直观的控制体验

### 2. **麦轮横移支持** 🚗
- ✅ 支持真正的横移（a/d键）
- ✅ 支持前进、后退、左右转
- ✅ 支持斜向移动（u/o/m/.键）

### 3. **舵机控制** 🎯
- ✅ 独立按键控制S1舵机
- ✅ 1键: 向左转(-45°)
- ✅ 2键: 回中位(0°)
- ✅ 3键: 向右转(45°)

### 4. **二维码识别** 📷
- ✅ 基于ROS2节点，运行在树莓派
- ✅ 实时识别二维码
- ✅ 识别结果变化时自动发布到话题
- ✅ 可在终端直接查看识别结果

---

## 📦 安装

### 1. 在树莓派上安装依赖

```bash
# 安装OpenCV和相关依赖
sudo apt update
sudo apt install -y python3-opencv python3-numpy

# 确保rpicam-vid已安装（树莓派OS通常自带）
which rpicam-vid
```

### 2. 编译ROS2包

```bash
# 进入工作空间
cd ~/ros2_ws/src

# 复制keyboard_control目录到工作空间
cp -r /path/to/7503_car/keyboard_control ./

# 编译
cd ~/ros2_ws
colcon build --packages-select keyboard_mecanum

# 加载环境
source install/setup.bash
```

---

## 🚀 使用方法

### 方式1: 仅启动键控（推荐）

```bash
ros2 launch keyboard_mecanum keyboard_control_launch.py
```

### 方式2: 仅启动二维码识别

```bash
ros2 launch keyboard_mecanum qr_scanner_launch.py
```

### 方式3: 同时启动键控和二维码识别

```bash
ros2 launch keyboard_mecanum full_system_launch.py
```

### 方式4: 手动启动各个节点

```bash
# 终端1: 启动键控
ros2 run keyboard_mecanum keyboard_control

# 终端2: 启动二维码识别
ros2 run keyboard_mecanum qr_scanner
```

---

## ⌨️ 键控说明

### 移动控制
```
   u    i    o
   j    k    l
   m    ,    .
```

- `i` / `,` : 前进 / 后退
- `j` / `l` : 左转 / 右转
- `u` / `m` : 左前 / 左后
- `o` / `.` : 右前 / 右后

### 麦轮横移
- `a` : 左横移（真正横移）
- `d` : 右横移（真正横移）

### 舵机S1控制
- `1` : S1 向左转 (-45°)
- `2` : S1 回中位 (0°)
- `3` : S1 向右转 (45°)

### 速度调整
- `q` / `z` : 增加/降低最大速度 10%
- `w` / `x` : 增加/降低线速度 10%
- `e` / `c` : 增加/降低角速度 10%

### 停止
- `空格键` 或 `k` : 立即停止
- `Ctrl+C` : 退出程序

---

## 📡 ROS2 话题

### 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 小车速度控制 |
| `/servo_s1` | `std_msgs/Int32` | 舵机S1角度控制 |
| `/qr_code` | `std_msgs/String` | 二维码识别结果 |

### 订阅话题

二维码识别节点不订阅任何话题，仅发布识别结果。

---

## 🧪 测试

### 测试键控功能

```bash
# 1. 启动键控
ros2 run keyboard_mecanum keyboard_control

# 2. 在另一个终端查看cmd_vel话题
ros2 topic echo /cmd_vel

# 3. 按键测试
# - 按住i键，应该看到持续的前进速度
# - 松开i键，应该立即停止
# - 按住a键，应该看到横移速度（linear.y非零）
# - 按1/2/3键，检查舵机话题
ros2 topic echo /servo_s1
```

### 测试二维码识别

```bash
# 1. 启动二维码识别节点
ros2 run keyboard_mecanum qr_scanner

# 2. 在另一个终端订阅二维码话题
ros2 topic echo /qr_code

# 3. 用手机生成二维码，放在相机前
# 4. 识别到新二维码时，会在终端显示并发布到话题
```

### 同时测试所有功能

```bash
# 1. 启动完整系统
ros2 launch keyboard_mecanum full_system_launch.py

# 2. 按键控制小车移动
# 3. 同时观察二维码识别结果
ros2 topic echo /qr_code
```

---

## 🔧 参数配置

### 键控节点参数

在launch文件中修改：

```python
parameters=[{
    'linear_speed_limit': 1.0,      # 最大线速度 (m/s)
    'angular_speed_limit': 5.0,     # 最大角速度 (rad/s)
}]
```

### 二维码识别参数

```python
parameters=[{
    'camera_width': 640,            # 相机宽度
    'camera_height': 480,           # 相机高度
    'publish_rate': 10.0,           # 发布频率 (Hz)
}]
```

---

## 📋 与开源键控的区别

| 特性 | 开源键控 | 本系统 |
|-----|---------|--------|
| 控制方式 | 按一次持续移动 | 长按移动，松开停止 |
| 停止方式 | 需按k键停止 | 松开按键自动停止 |
| 横移支持 | 模拟横移（旋转+前进） | 真正麦轮横移 |
| 舵机控制 | 需单独发布话题 | 集成在键控中（1/2/3键） |
| 二维码识别 | 无 | 集成ROS2节点 |

---

## 🎯 典型应用场景

### 场景1: 手动遥控

```bash
ros2 launch keyboard_mecanum keyboard_control_launch.py
```

### 场景2: 巡逻 + 二维码识别

```bash
# 启动完整系统
ros2 launch keyboard_mecanum full_system_launch.py

# 在另一个终端监控识别结果
ros2 topic echo /qr_code
```

### 场景3: 自动记录识别结果

```bash
# 启动系统
ros2 launch keyboard_mecanum full_system_launch.py

# 录制识别结果到日志
ros2 topic echo /qr_code >> qr_log.txt
```

---

## 🛠️ 故障排查

### 问题1: 键控节点无法启动

**症状**: 提示 `No module named 'termios'`

**解决**: 这是正常的，termios只能在Linux上运行。在Windows上需要通过SSH连接到树莓派运行。

### 问题2: 二维码识别节点启动失败

**症状**: `rpicam-vid 启动失败`

**解决**: 
```bash
# 检查相机是否连接
libcamera-hello

# 检查rpicam-vid是否安装
which rpicam-vid

# 如果未安装，更新系统
sudo apt update && sudo apt upgrade
```

### 问题3: 相机无法打开

**症状**: `无法打开视频流`

**解决**:
```bash
# 1. 检查相机是否启用
sudo raspi-config
# -> Interface Options -> Camera -> Enable

# 2. 重启树莓派
sudo reboot

# 3. 测试相机
libcamera-hello -t 0
```

### 问题4: 舵机不响应

**症状**: 按1/2/3键无反应

**解决**:
```bash
# 1. 检查舵机话题是否发布
ros2 topic echo /servo_s1

# 2. 检查ESP32是否连接并订阅该话题
ros2 topic info /servo_s1

# 3. 手动测试舵机
ros2 topic pub --once /servo_s1 std_msgs/msg/Int32 "{data: 45}"
```

### 问题5: 麦轮横移无效

**症状**: 按a/d键小车不横移

**解决**:
```bash
# 检查ESP32固件是否支持麦轮横移
# 确保ESP32固件处理了cmd_vel.linear.y参数

# 查看发送的速度命令
ros2 topic echo /cmd_vel
# 应该看到 linear.y 不为0
```

---

## 📚 参考文档

- [odom_publisher 舵机控制说明](../odom_publisher/舵机控制说明.md)
- [ROS2 文档](https://docs.ros.org/) (支持 Jazzy/Humble/Iron)
- [树莓派相机文档](https://www.raspberrypi.com/documentation/computers/camera_software.html)

---

## 🎉 完整功能列表

| 功能 | 状态 | 说明 |
|-----|------|------|
| ✅ 长按式键控 | 完成 | 按住移动，松开停止 |
| ✅ 麦轮横移 | 完成 | 支持a/d键真正横移 |
| ✅ 舵机S1控制 | 完成 | 1/2/3键控制 |
| ✅ 二维码识别 | 完成 | 实时识别并发布 |
| ✅ ROS2集成 | 完成 | 完整的话题通信 |
| ✅ 树莓派支持 | 完成 | 优化树莓派运行 |

---

**版本**: v1.0.0  
**更新时间**: 2024-11-22  
**测试平台**: 树莓派 + ROS2 (Humble/Jazzy) + ESP32-S3

