#!/bin/bash
# 验证 freeze_ros workspace 是否正常工作
# 使用方法: bash test_overlay.sh

echo "============================================"
echo "  ROS freeze_ros Workspace 验证脚本"
echo "============================================"

# 1. 检查 workspace 路径
echo ""
echo "[1/5] 检查 source 环境..."

ROS_SETUP="/opt/ros/melodic/setup.bash"
FREEZE_SETUP="/home/abot/freeze_ros/ros_shoot_2026/devel/setup.bash"

if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
    echo "  [OK] ROS underlay: $ROS_SETUP"
else
    echo "  [FAIL] 找不到 ROS underlay: $ROS_SETUP"
    exit 1
fi

if [ -f "$FREEZE_SETUP" ]; then
    source "$FREEZE_SETUP"
    echo "  [OK] freeze_ros workspace: $FREEZE_SETUP"
else
    echo "  [FAIL] 找不到 freeze_ros workspace: $FREEZE_SETUP"
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

# 比赛链路源码包 (必须全部从 freeze_ros 加载)
check_pkg "robot_slam"    "freeze_ros/ros_shoot_2026"
check_pkg "abot_slam"     "freeze_ros/ros_shoot_2026"
check_pkg "TTS_audio"     "freeze_ros/ros_shoot_2026"
check_pkg "abot_bringup"  "freeze_ros/ros_shoot_2026"
check_pkg "abot_imu"      "freeze_ros/ros_shoot_2026"
check_pkg "abot_model"    "freeze_ros/ros_shoot_2026"
check_pkg "lidar_filters" "freeze_ros/ros_shoot_2026"
check_pkg "shoot_cmd"     "freeze_ros/ros_shoot_2026"
check_pkg "track_tag"     "freeze_ros/ros_shoot_2026"
check_pkg "find_object_2d" "freeze_ros/ros_shoot_2026"

# ROS Melodic 系统依赖 (应由 apt 安装)
check_pkg "map_server"       "/opt/ros/melodic"
check_pkg "move_base"        "/opt/ros/melodic"
check_pkg "amcl"             "/opt/ros/melodic"
check_pkg "tf"               "/opt/ros/melodic"
check_pkg "xacro"            "/opt/ros/melodic"
check_pkg "laser_filters"    "/opt/ros/melodic"
check_pkg "rplidar_ros"      "/opt/ros/melodic"
check_pkg "usb_cam"          "/opt/ros/melodic"
check_pkg "ar_track_alvar"   "/opt/ros/melodic"
check_pkg "ar_track_alvar_msgs" "/opt/ros/melodic"
check_pkg "robot_pose_ekf"   "/opt/ros/melodic"
check_pkg "imu_filter_madgwick" "/opt/ros/melodic"
check_pkg "joint_state_publisher" "/opt/ros/melodic"
check_pkg "robot_state_publisher" "/opt/ros/melodic"

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
echo "[4/5] 检查 C++ 节点编译产物..."

check_node() {
    local pkg=$1
    local node=$2
    local binary="/home/abot/freeze_ros/ros_shoot_2026/devel/lib/$pkg/$node"
    if [ -x "$binary" ]; then
        echo "  [OK] $pkg/$node"
    else
        echo "  [WARN] $pkg/$node 未找到: $binary"
    fi
}

check_node "shoot_cmd"     "shoot_control"
check_node "track_tag"     "ar_track"
echo "  [INFO] shoot_cmd/shoot_control 是旧射击链路产物，2026 主控不要同时启动它"

echo ""
echo "[5/5] 检查关键文件、topic 和 service 类型..."

MAP_DIR="/home/abot/freeze_ros/ros_shoot_2026/src/robot_slam/maps"
for map_file in my_lab_freeze.yaml my_lab_freeze.pgm; do
    if [ -f "$MAP_DIR/$map_file" ]; then
        echo "  [OK] $MAP_DIR/$map_file"
    else
        echo "  [FAIL] $MAP_DIR/$map_file"
    fi
done

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

if rossrv show "TTS_audio/StringService" >/dev/null 2>&1; then
    echo "  [OK] TTS_audio/StringService"
else
    echo "  [WARN] TTS_audio/StringService"
fi

echo ""
echo "============================================"
echo "  验证完成! (需要先启动 roscore 才能跑完整测试)"
echo "============================================"
