#!/bin/bash
# 麦轮小车键控系统 - 快速测试脚本

echo "========================================="
echo "  麦轮小车键控系统 - 功能测试"
echo "========================================="
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: 未检测到ROS2环境"
    echo "   请先运行: source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "✅ ROS2环境: $ROS_DISTRO"
echo ""

# 测试菜单
while true; do
    echo "请选择要测试的功能:"
    echo ""
    echo "1) 测试键控节点"
    echo "2) 测试二维码识别节点"
    echo "3) 测试完整系统（键控+二维码）"
    echo "4) 查看所有话题"
    echo "5) 监控 /cmd_vel 话题"
    echo "6) 监控 /servo_s1 话题"
    echo "7) 监控 /qr_code 话题"
    echo "8) 手动发送舵机命令"
    echo "9) 手动发送速度命令"
    echo "0) 退出"
    echo ""
    read -p "请输入选项 (0-9): " choice
    
    case $choice in
        1)
            echo ""
            echo "🎮 启动键控节点..."
            echo "   按键说明:"
            echo "   - i/,  : 前进/后退"
            echo "   - j/l  : 左转/右转"
            echo "   - a/d  : 左横移/右横移"
            echo "   - 1/2/3: 舵机控制"
            echo "   - Ctrl+C: 退出"
            echo ""
            sleep 2
            ros2 run keyboard_mecanum keyboard_control
            ;;
        2)
            echo ""
            echo "📷 启动二维码识别节点..."
            echo "   识别结果将显示在终端"
            echo "   按 Ctrl+C 退出"
            echo ""
            sleep 2
            ros2 run keyboard_mecanum qr_scanner
            ;;
        3)
            echo ""
            echo "🚀 启动完整系统..."
            echo "   将同时启动键控和二维码识别"
            echo "   按 Ctrl+C 退出"
            echo ""
            sleep 2
            ros2 launch keyboard_mecanum full_system_launch.py
            ;;
        4)
            echo ""
            echo "📋 查看所有话题..."
            ros2 topic list
            echo ""
            read -p "按回车继续..."
            ;;
        5)
            echo ""
            echo "🔍 监控 /cmd_vel 话题..."
            echo "   按 Ctrl+C 退出"
            echo ""
            sleep 1
            ros2 topic echo /cmd_vel
            ;;
        6)
            echo ""
            echo "🔍 监控 /servo_s1 话题..."
            echo "   按 Ctrl+C 退出"
            echo ""
            sleep 1
            ros2 topic echo /servo_s1
            ;;
        7)
            echo ""
            echo "🔍 监控 /qr_code 话题..."
            echo "   按 Ctrl+C 退出"
            echo ""
            sleep 1
            ros2 topic echo /qr_code
            ;;
        8)
            echo ""
            echo "🎯 手动发送舵机命令"
            echo ""
            echo "请选择角度:"
            echo "1) -45° (左)"
            echo "2) 0°   (中)"
            echo "3) 45°  (右)"
            echo "4) 自定义"
            echo ""
            read -p "请输入选项: " angle_choice
            
            case $angle_choice in
                1) angle=-45 ;;
                2) angle=0 ;;
                3) angle=45 ;;
                4) 
                    read -p "请输入角度 (-90 到 90): " angle
                    ;;
                *)
                    echo "❌ 无效选项"
                    continue
                    ;;
            esac
            
            echo "发送舵机命令: $angle°"
            ros2 topic pub --once /servo_s1 std_msgs/msg/Int32 "{data: $angle}"
            echo "✅ 命令已发送"
            echo ""
            read -p "按回车继续..."
            ;;
        9)
            echo ""
            echo "🚗 手动发送速度命令"
            echo ""
            echo "请选择测试:"
            echo "1) 前进 (0.3 m/s)"
            echo "2) 后退 (0.3 m/s)"
            echo "3) 左横移 (0.3 m/s)"
            echo "4) 右横移 (0.3 m/s)"
            echo "5) 左转 (1.0 rad/s)"
            echo "6) 右转 (1.0 rad/s)"
            echo "7) 停止"
            echo ""
            read -p "请输入选项: " vel_choice
            
            case $vel_choice in
                1) cmd='"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"' ;;
                2) cmd='"{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"' ;;
                3) cmd='"{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"' ;;
                4) cmd='"{linear: {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"' ;;
                5) cmd='"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"' ;;
                6) cmd='"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"' ;;
                7) cmd='"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"' ;;
                *)
                    echo "❌ 无效选项"
                    continue
                    ;;
            esac
            
            echo "发送速度命令..."
            eval "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist $cmd"
            echo "✅ 命令已发送"
            echo ""
            read -p "按回车继续..."
            ;;
        0)
            echo ""
            echo "👋 再见！"
            exit 0
            ;;
        *)
            echo ""
            echo "❌ 无效选项，请重新选择"
            echo ""
            ;;
    esac
    
    echo ""
    echo "========================================="
    echo ""
done

