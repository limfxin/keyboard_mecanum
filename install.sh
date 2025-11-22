#!/bin/bash
# 麦轮小车键控系统 - 安装脚本
# 适用于树莓派 + ROS2 (Humble/Jazzy等)

echo "========================================="
echo "  麦轮小车键控系统 - 安装脚本"
echo "========================================="
echo ""

# 检查是否在Linux上
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "❌ 错误: 此脚本只能在Linux上运行"
    echo "   请在树莓派上执行此脚本"
    exit 1
fi

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  警告: 未检测到ROS2环境"
    echo "   正在尝试自动检测ROS2安装..."
    
    # 尝试常见的ROS2版本
    for distro in jazzy humble iron galactic; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source /opt/ros/$distro/setup.bash
            echo "✅ ROS2环境已加载: $ROS_DISTRO"
            break
        fi
    done
    
    if [ -z "$ROS_DISTRO" ]; then
        echo "❌ 错误: 找不到ROS2安装"
        echo "   请先安装ROS2 (支持Jazzy/Humble/Iron/Galactic)"
        exit 1
    fi
else
    echo "✅ 检测到ROS2环境: $ROS_DISTRO"
fi

echo ""
echo "1️⃣  安装系统依赖..."
sudo apt update
sudo apt install -y python3-opencv python3-numpy xterm

echo ""
echo "2️⃣  检查相机工具..."
if command -v rpicam-vid &> /dev/null; then
    echo "✅ rpicam-vid 已安装"
else
    echo "⚠️  rpicam-vid 未找到，尝试安装..."
    sudo apt install -y rpicam-apps
fi

echo ""
echo "3️⃣  检查ROS2工作空间..."
if [ -z "$ROS_WS" ]; then
    ROS_WS="$HOME/ros2_ws"
    echo "   使用默认工作空间: $ROS_WS"
else
    echo "   使用工作空间: $ROS_WS"
fi

# 创建工作空间（如果不存在）
if [ ! -d "$ROS_WS/src" ]; then
    echo "   创建ROS2工作空间..."
    mkdir -p "$ROS_WS/src"
fi

echo ""
echo "4️⃣  复制包到工作空间..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TARGET_DIR="$ROS_WS/src/keyboard_mecanum"

if [ -d "$TARGET_DIR" ]; then
    echo "   ⚠️  目标目录已存在，删除旧版本..."
    rm -rf "$TARGET_DIR"
fi

echo "   从 $SCRIPT_DIR"
echo "   到 $TARGET_DIR"

mkdir -p "$TARGET_DIR"
cp -r "$SCRIPT_DIR"/* "$TARGET_DIR/"

echo ""
echo "5️⃣  编译ROS2包..."
cd "$ROS_WS"
colcon build --packages-select keyboard_mecanum

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
else
    echo "❌ 编译失败，请检查错误信息"
    exit 1
fi

echo ""
echo "6️⃣  测试相机..."
echo "   将打开相机5秒进行测试..."
sleep 1
libcamera-hello -t 5000 2>&1 | head -n 10

echo ""
echo "========================================="
echo "  ✅ 安装完成！"
echo "========================================="
echo ""
echo "📝 下一步操作："
echo ""
echo "1. 加载环境变量："
echo "   source $ROS_WS/install/setup.bash"
echo ""
echo "2. 启动键控（推荐）："
echo "   ros2 launch keyboard_mecanum keyboard_control_launch.py"
echo ""
echo "3. 启动完整系统（键控+二维码）："
echo "   ros2 launch keyboard_mecanum full_system_launch.py"
echo ""
echo "4. 查看详细文档："
echo "   cat $TARGET_DIR/使用指南.md"
echo ""
echo "========================================="

# 添加到bashrc（可选）
echo ""
read -p "是否将环境变量添加到 ~/.bashrc? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    BASHRC_LINE="source $ROS_WS/install/setup.bash"
    if ! grep -q "$BASHRC_LINE" ~/.bashrc; then
        echo "$BASHRC_LINE" >> ~/.bashrc
        echo "✅ 已添加到 ~/.bashrc"
        echo "   下次登录时会自动加载ROS2环境"
    else
        echo "✅ ~/.bashrc 中已存在该配置"
    fi
fi

echo ""
echo "🎉 完成！祝使用愉快！"

