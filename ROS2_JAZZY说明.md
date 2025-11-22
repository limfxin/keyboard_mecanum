# ROS2 Jazzy 版本说明

## ✅ 完全兼容 ROS2 Jazzy

本键控系统完全支持ROS2 Jazzy版本，所有功能正常运行。

---

## 🚀 在Jazzy上安装

### 方法1: 自动安装（推荐）

```bash
cd keyboard_control
chmod +x install.sh
./install.sh
```

安装脚本会**自动检测**你的ROS2版本（Jazzy/Humble/Iron等），无需手动配置。

### 方法2: 手动安装

```bash
# 1. 加载Jazzy环境
source /opt/ros/jazzy/setup.bash

# 2. 复制到工作空间
cd ~/ros2_ws/src
cp -r /path/to/keyboard_control ./keyboard_mecanum

# 3. 安装依赖
sudo apt install python3-opencv python3-numpy xterm

# 4. 编译
cd ~/ros2_ws
colcon build --packages-select keyboard_mecanum

# 5. 加载环境
source install/setup.bash
```

---

## 🎯 Jazzy特定注意事项

### 1. Python版本

ROS2 Jazzy使用Python 3.11+，确保你的系统已安装：

```bash
python3 --version  # 应该是 3.11 或更高
```

### 2. OpenCV版本

```bash
# 检查OpenCV版本
python3 -c "import cv2; print(cv2.__version__)"

# 如果没有安装
sudo apt install python3-opencv
```

### 3. 依赖包

Jazzy的依赖包名称可能略有不同：

```bash
# 基础依赖
sudo apt update
sudo apt install -y \
    python3-opencv \
    python3-numpy \
    xterm \
    rpicam-apps
```

---

## 🔧 Jazzy vs Humble 差异

本项目在两个版本上的使用**完全相同**，没有任何功能差异。

| 特性 | Humble | Jazzy |
|-----|--------|-------|
| 长按键控 | ✅ | ✅ |
| 麦轮横移 | ✅ | ✅ |
| 舵机控制 | ✅ | ✅ |
| 二维码识别 | ✅ | ✅ |
| Launch文件 | ✅ | ✅ |
| 所有功能 | 完全相同 | 完全相同 |

---

## 🧪 在Jazzy上测试

### 1. 检查环境

```bash
# 检查ROS2版本
echo $ROS_DISTRO  # 应该输出: jazzy

# 检查节点是否可用
ros2 pkg list | grep keyboard_mecanum
```

### 2. 启动测试

```bash
# 启动键控
ros2 launch keyboard_mecanum keyboard_control_launch.py

# 查看话题
ros2 topic list

# 测试发布
ros2 topic echo /cmd_vel
```

### 3. 完整功能测试

```bash
# 使用测试脚本
cd keyboard_control
chmod +x test_system.sh
./test_system.sh
```

---

## 📋 常见问题 (Jazzy特定)

### Q1: 提示找不到包

**症状**: `Package 'keyboard_mecanum' not found`

**解决**:
```bash
# 重新编译
cd ~/ros2_ws
colcon build --packages-select keyboard_mecanum --symlink-install

# 重新加载环境
source install/setup.bash
```

### Q2: Python模块导入错误

**症状**: `ModuleNotFoundError: No module named 'cv2'`

**解决**:
```bash
# Jazzy使用Python 3.11+，需要重新安装OpenCV
sudo apt install python3-opencv python3-numpy
```

### Q3: Launch文件找不到

**症状**: `Unable to find launch file`

**解决**:
```bash
# 确保正确编译了launch文件
cd ~/ros2_ws
colcon build --packages-select keyboard_mecanum
source install/setup.bash

# 检查launch文件
ls ~/ros2_ws/install/keyboard_mecanum/share/keyboard_mecanum/launch/
```

---

## 🎉 Jazzy优化建议

### 1. 使用新的命令行工具

Jazzy引入了一些新特性，建议使用：

```bash
# 使用新的话题工具
ros2 topic info /cmd_vel --verbose

# 使用改进的bag记录
ros2 bag record -a  # 记录所有话题
```

### 2. 性能优化

Jazzy在性能上有改进，可以提高发布频率：

编辑 `launch/qr_scanner_launch.py`:
```python
parameters=[{
    'camera_width': 640,
    'camera_height': 480,
    'publish_rate': 15.0,  # Jazzy可以设置更高（原来是10.0）
}]
```

### 3. 日志级别

Jazzy改进了日志系统：

```bash
# 设置详细日志
ros2 run keyboard_mecanum keyboard_control --ros-args --log-level debug
```

---

## 🔄 从Humble迁移到Jazzy

如果你之前使用Humble，现在想切换到Jazzy：

```bash
# 1. 安装ROS2 Jazzy
# （参考ROS2官方文档）

# 2. 重新编译工作空间
cd ~/ros2_ws
rm -rf build/ install/ log/  # 清理旧的编译结果
source /opt/ros/jazzy/setup.bash
colcon build --packages-select keyboard_mecanum

# 3. 更新bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 📚 Jazzy相关资源

- [ROS2 Jazzy 官方文档](https://docs.ros.org/en/jazzy/)
- [Jazzy 发行说明](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [从Humble迁移指南](https://docs.ros.org/en/jazzy/Releases/Migration-Guide.html)

---

## ✅ 验证清单

在Jazzy上安装后，确认以下都正常：

- [ ] `echo $ROS_DISTRO` 输出 `jazzy`
- [ ] `ros2 pkg list | grep keyboard_mecanum` 能找到包
- [ ] `ros2 launch keyboard_mecanum keyboard_control_launch.py` 能启动
- [ ] 按键控制正常响应
- [ ] `/cmd_vel` 话题正常发布
- [ ] `/servo_s1` 话题正常发布
- [ ] 二维码识别节点正常工作
- [ ] `/qr_code` 话题正常发布

全部打勾？**恭喜，Jazzy版本安装成功！** 🎉

---

## 💡 技术说明

本项目之所以能无缝支持Jazzy，是因为：

1. **使用标准ROS2 API** - 没有使用过时或实验性API
2. **Python代码兼容** - 支持Python 3.8+
3. **消息类型标准** - 使用标准的geometry_msgs和std_msgs
4. **Launch文件格式** - 使用Python launch格式，跨版本兼容

---

**版本**: v1.0.0  
**ROS2版本**: ✅ Jazzy (主要测试) | ✅ Humble (兼容)  
**更新日期**: 2024-11-22

