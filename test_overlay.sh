#!/bin/bash
# 验证 overlay workspace 是否正常工作
# 使用方法: bash test_overlay.sh

echo "============================================"
echo "  ROS Overlay Workspace 验证脚本"
echo "============================================"

# 1. 检查 workspace 路径
echo ""
echo "[1/5] 检查 source 环境..."

WS1="/home/abot/abot_ws/devel/setup.bash"
WS2="/home/abot/freeze_ros/ros_shoot_2026/devel/setup.bash"

if [ -f "$WS1" ]; then
    source "$WS1"
    echo "  [OK] 原始 workspace: $WS1"
else
    echo "  [FAIL] 找不到原始 workspace: $WS1"
    exit 1
fi

if [ -f "$WS2" ]; then
    source "$WS2"
    echo "  [OK] overlay workspace: $WS2"
else
    echo "  [FAIL] 找不到 overlay workspace: $WS2"
    exit 1
fi

# 2. 验证包能找到
echo ""
echo "[2/5] 验证 package 路径..."

check_pkg() {
    local pkg=$1
    local expected=$2
    local path=$(rospack find "$pkg" 2>/dev/null)
    if [ -z "$path" ]; then
        echo "  [FAIL] $pkg 找不到"
    elif echo "$path" | grep -q "$expected"; then
        echo "  [OK] $pkg -> $path"
    else
        echo "  [WARN] $pkg -> $path (期望在 $expected)"
    fi
}

# overlay 里的包 (应从你的 workspace 加载)
check_pkg "robot_slam"    "freeze_ros/ros_shoot_2026"
check_pkg "abot_slam"     "freeze_ros/ros_shoot_2026"
check_pkg "TTS_audio"     "freeze_ros/ros_shoot_2026"

# 原始 workspace 的包 (应通过 overlay 继承)
check_pkg "abot_bringup"  "abot_ws"
check_pkg "shoot_cmd"     "abot_ws"
check_pkg "track_tag"     "abot_ws"

echo ""
echo "[3/5] 检查 Python 依赖..."

# TTS_audio 需要 websockets
if python3 -c "import websockets" 2>/dev/null; then
    echo "  [OK] websockets"
else
    echo "  [WARN] websockets 未安装, TTS_audio 会失败"
fi

# 2026_shoot_demo.py 需要 funasr
if python3 -c "import funasr" 2>/dev/null; then
    echo "  [OK] funasr"
else
    echo "  [INFO] funasr 未安装 (仅语音识别需要)"
fi

echo ""
echo "[4/5] 检查关键节点是否可以找到..."

check_node() {
    local pkg=$1
    local node=$2
    local result=$(rosrun "$pkg" --list 2>/dev/null | grep -w "$node")
    if [ -n "$result" ]; then
        echo "  [OK] $pkg/$node"
    else
        echo "  [INFO] $pkg/$node (未找到可执行文件，可能是Python脚本)"
    fi
}

check_node "shoot_cmd"     "shoot_control"
check_node "track_tag"     "ar_track"

echo ""
echo "[5/5] 检查关键 topic 和 service 类型..."

# 验证消息类型是否可用
check_type() {
    local type=$1
    if rosmsg show "$type" >/dev/null 2>&1; then
        echo "  [OK] $type"
    else
        echo "  [WARN] $type"
    fi
}

check_type "geometry_msgs/Twist"
check_type "std_msgs/String"
check_type "ar_track_alvar_msgs/AlvarMarkers"
check_type "TTS_audio/StringService"

echo ""
echo "============================================"
echo "  验证完成! (需要先启动 roscore 才能跑完整测试)"
echo "============================================"
