# Project Map — abot ROS Workspace

> 生成日期: 2026-05-14
> ROS 版本: Melodic (Ubuntu 18.04)
> 机器人: "abot" — Ackermann 底盘 + 射击机构

## 目录

1. [工程总览](#1-工程总览)
2. [Package 一览](#2-package-一览)
3. [Launch 启动链](#3-launch-启动链)
4. [Topic 数据流](#4-topic-数据流)
5. [关键串口协议](#5-关键串口协议)
6. [各子系统详解](#6-各子系统详解)
7. [现存问题与建议](#7-现存问题与建议)

---

## 1. 工程总览

这是一个 ROS Melodic 工作空间，包含约 **20 个 packages**，分为以下几层：

```
应用层 (Python)     robot_slam, abot_vlm, tracker_pkg, color_pkg, face_pkg
                      ↕ topic ↕
导航/SLAM 层         move_base, gmapping, cartographer, hector_slam, amcl
                      ↕ topic ↕
驱动层 (C++)         abot_driver, shoot_control, abot_imu, cam_track, track_tag
                      ↕ 串口/硬件
硬件层               RPLidar, USB Camera, 电机驱动板, 射击舵机, IMU
```

**构建方式**: `catkin_make` (Melodic 标准)
**工作空间路径** (目标机): `/home/abot/abot_ws/`
**Python 解释器**: Python 3.9 conda (`/home/abot/anaconda3/envs/py39/bin/python`) 或 `#!/usr/bin/env python3`

---

## 2. Package 一览

### 2.1 底盘驱动 (abot_base)

| 子包 | 语言 | 文件 | 功能 |
|---|---|---|---|
| **abot_bringup** | C++ | `main.cpp`, `base_driver.cpp`, `serial_transport.cpp` | 底盘电机驱动主节点 `abot_driver`；读取 `base_params.yaml`；自定义 serial 协议 (8N1/9600) |
| | Python | `odom_ekf.py` | EKF 里程计融合 (订阅 `robot_pose_ekf/odom_combined` → 发布 `/odom`) |
| | Python | `shoot.py`, `cofig.py` | 简单射击脚本、参数配置工具 |
| **abot_imu** | C++ | `abot_imu.cpp` | IMU 原始数据读取节点 |
| **abot_model** | URDF | `urdf/abot_model.urdf` | 机器人 URDF 模型、Gazebo 仿真 launch |
| **lidar_filters** | Launch | `box_filter_example.launch` | 激光雷达范围过滤器 (搭配 `laser_filters`) |

### 2.2 导航/SLAM

| 包 | 语言 | 功能 |
|---|---|---|
| **abot_slam** | Launch/YAML | SLAM launch: gmapping, cartographer, hector；move_base 参数: DWA, costmap, 全局/局部规划器 |
| **robot_slam** | Python/C++ | **核心应用包**: 多目标点导航任务 (2022~2025)、射击演示、语音导航、VLM 识别集成 |
| **hector_slam** (3rd Party) | C++ | Hector SLAM 库: hector_mapping, geotiff, trajectory_server 等 |

### 2.3 射击系统

| 包 | 语言 | 文件 | 功能 |
|---|---|---|---|
| **shoot_cmd** | C++ | `shoot_control.cpp`, `control_center.cpp`, `SerialPort.cpp` | 串口射击控制。订阅 `/shoot` topic。协议: `0x55 0x01 0x12...` 发射, `0x55 0x01 0x11...` 停止 |

### 2.4 视觉感知

| 包 | 语言 | 功能 |
|---|---|---|
| **cam_track** | C++ | 摄像头目标跟踪，PID 控制，发布 `/cmd_vel` |
| **track_tag** | C++ | AR 标签跟踪 (`ar_track_alvar`)。订阅 `/ar_pose_marker` → P 控制器 → 发布 `/cmd_vel` + `/shoot` |
| **abot_find** | Launch | `find_object_2d` 包装: 基于特征匹配 (SIFT/SURF) 的目标检测 |
| **tracker_pkg** | Python | KCF + 卡尔曼滤波跟踪、KLT 光流跟踪 |
| **abot_object_detect** | Python | HSV 颜色检测 (球)、Haar 级联人脸检测、行人检测 |
| **color_pkg** | Python | 火焰检测 (红色阈值)、巡线 |
| **face_pkg** | Python | 人脸检测/识别 (LBPH) |
| **opencv_demo** | C++/Python | 大量 OpenCV 示例 launch (canny, hough, camshift, 光流等) |

### 2.5 视觉语言模型 (VLM)

| 包 | 语言 | 文件 | 功能 |
|---|---|---|---|
| **abot_vlm** | Python | `vlm_node.py` | 订阅 `/usb_cam/image_raw`，调用 Yi-Vision API (零一万物)，发布 `/vision_result` |

### 2.6 语音

| 包 | 语言 | 文件 | 功能 |
|---|---|---|---|
| **robot_voice** | C++ | `tts_subscribe.cpp`, `iat_publish.cpp`, `voice_assistant.cpp` | 语音合成 (订阅 `/robot_voice/tts_topic`)、语音识别 (iFlytek MSC SDK)、语音助手 |
| **TTS_audio** | Python | `TTS.py` | WebSocket 方式 TTS 服务 (ROS Service `StringService`) |
| **voice_demo** | Python | demo 脚本 | 语音功能测试 |

### 2.7 其他

| 包 | 语言 | 功能 |
|---|---|---|
| **lidar_follower** | Python | 基于激光雷达的行人跟随 |
| **imu_filter** | C++ | Madgwick/Mahony 姿态解算滤波器 |
| **user_demo** | C++/Python | 用户自定义任务示例 mission_node |

---

## 3. Launch 启动链

### 3.1 底盘启动 (基础版)

```
robot.launch
├── bringup.launch
│   └── abot_driver (C++)  ← 加载 base_params.yaml
├── model.launch
│   ├── joint_state_publisher
│   └── robot_state_publisher
└── rplidar.launch
    ├── rplidarNode (/dev/rplidar, 115200)
    └── box_filter_example.launch → scan_to_scan_filter_chain
```

### 3.2 底盘启动 (带 IMU)

```
robot_with_imu.launch
├── bringup_with_imu.launch
│   ├── abot_driver  ← 加载 base_params_with_imu.yaml
│   ├── abot_imu
│   ├── static_transform_publisher (imu_link ← laser_link)
│   ├── imu_filter_madgwick
│   ├── robot_pose_ekf  ← 融合 wheel_odom + imu/data
│   └── odom_ekf.py  ← robot_pose_ekf/odom_combined → /odom
├── model.launch
└── rplidar.launch
```

### 3.3 SLAM 建图

```
# Gmapping
gmapping.launch
├── slam_gmapping (gmapping)
└── move_base.launch.xml

# Hector
hector.launch
└── hector_mapping

# Cartographer
cartographer.launch
└── cartographer_node + rviz
```

### 3.4 导航 (基于已有地图)

```
navigation.launch / navigation_shoot.launch
├── map_server  ← 加载 shoot.yaml / my_lab.yaml
└── move_base.launch.xml  +  amcl.launch.xml
```

### 3.5 多目标点任务 (逐年演进)

```
multi_goal_2022.launch  → navigation_multi_goals_2022.py
multi_goal_2023.launch  → navigation_multi_goals_2023.py
multi_goal_2024.launch  → navigation_multi_goals_4.py
multi_goal_2025.launch  → navigation_multi_goals_2025.py
multi_goal_vlm.launch   → navigation_multi_goals_vlm.py
multi_goal_shoot_2024   → shoot_object_2024.py
multi_goal_shoot_2025   → shoot_object_2025.py
4x5_demo.launch         → 4x5_demo.py
```

### 3.6 射击演示完整流程

```
# 1. 启动底盘 + 激光雷达
roslaunch abot_bringup robot.launch

# 2. 启动导航 (加载射击场地地图)
roslaunch robot_slam navigation_shoot.launch

# 3. 启动摄像头
roslaunch robot_slam camera_my.launch

# 4a. 射击演示 - 基于 AR 标签
rosrun shoot_cmd shoot_control    # 射击执行节点
roslaunch track_tag ar_track_camera.launch  # AR 标签检测
roslaunch track_tag track_command.launch     # 标签跟踪 + 射击决策

# 4b. 射击演示 - 基于 VLM (2025)
roslaunch abot_vlm vlm_node.launch  # Yi-Vision API
roslaunch robot_slam multi_goal_vlm.launch

# 4c. 射击演示 - 基于语音
rosrun robot_slam 2026_shoot_demo.py   # FunASR 语音识别
rosrun robot_slam 2026_nav_end.py      # 语义解析 + 导航目标发布
```

---

## 4. Topic 数据流

### 4.1 核心数据流总图

```
                    ┌─────────────────────────────────────┐
                    │          /usb_cam/image_raw          │
                    │          (Camera: usb_cam)           │
                    └──────────┬──────────┬────────────┬───┘
                               │          │            │
                    ┌──────────▼──┐  ┌────▼────┐  ┌───▼──────────┐
                    │ ar_track_   │  │abot_vlm │  │ cam_track    │
                    │ alvar       │  │vlm_node │  │ track_tag    │
                    │ (AR tag)    │  │(Yi-Vis) │  │tracker_pkg   │
                    └──────┬──────┘  └────┬────┘  └──────┬───────┘
                           │              │              │
              ┌────────────▼──┐   ┌───────▼──────┐       │
              │/ar_pose_marker│   │/vision_result │       │
              └────────┬──────┘   └───┬───────────┘       │
                       │              │                   │
              ┌────────▼────┐  ┌──────▼───────┐           │
              │ ar_track    │  │ navigation_  │           │
              │ (track_tag) │  │ multi_goals_ │           │
              └──┬──────┬───┘  │ vlm.py       │           │
                 │      │      └──────┬────────┘           │
                 │      │             │                    │
         ┌───────▼───┐  │     ┌──────▼──────┐             │
         │  /shoot    │  │     │ /shoot      │             │
         │  (String)  │  │     │ (publish)   │             │
         └───┬───────┘  │     └─────────────┘             │
             │          │                                  │
    ┌────────▼──┐  ┌────▼──────────────────────┐           │
    │shoot_     │  │        /cmd_vel           │◄──────────┘
    │control    │  │        (Twist)            │
    │(serial)   │  └────────┬──────────────────┘
    └───────────┘           │
                            ▼
                    ┌────────────────┐
                    │  abot_driver    │
                    │  (serial→电机)  │
                    └────────────────┘
```

### 4.2 Topic 详细清单

| Topic | Type | Pub Node | Sub Node | 说明 |
|---|---|---|---|---|
| `/cmd_vel` | Twist | cam_track, track_tag, lidar_follower, tracker_pkg, move_base, control_center | abot_driver | **核心**: 速度命令枢纽 |
| `/shoot` | String | track_tag, shoot_2025.py, navigate.cpp | shoot_control | 射击指令: "shoot" / "stopshoot" |
| `/move_base_simple/goal` | PoseStamped | mission.py, navigation_*.py, control_center | move_base | 导航目标点 |
| `/move_base/status` | GoalStatusArray | move_base | control_center, navigation scripts | 导航状态反馈 |
| `/ar_pose_marker` | AlvarMarkers | ar_track_alvar | track_tag, control_center | AR 标签位姿 |
| `/usb_cam/image_raw` | Image | usb_cam_node | vlm_node, ar_track_alvar, find_object_2d, trackers | 摄像头图像 |
| `/vision_result` | String | vlm_node | navigation_vlm.py | VLM 识别结果 |
| `/chinese_topic` | String | 2026_shoot_demo.py | 2026_nav_end.py | ASR 中文识别文本 |
| `/nav_end_topic` | Int32 | 2026_nav_end.py | navigation scripts | 导航终点 ID (11/12/13 对应一二三) |
| `/operator_topic` | String | 2026_nav_end.py | navigation scripts | 数学运算符 (+-*/) |
| `/initialpose` | PoseWithCovarianceStamped | mission.py | amcl | 初始位姿估计 |
| `/robot_voice/tts_topic` | String | navigation scripts | tts_subscribe | TTS 语音合成请求 |
| `/snowman/ask` | String | 语音前端 | control_center | 语音命令触发导航 |
| `/voiceWords` | String | 2026_nav_end.py | voice_assistant | 语音反馈文本 |
| `/mission/arrived` | String | mission.py | track_tag | 导航到达信号 |
| `/scan` | LaserScan | rplidarNode | gmapping, hector, amcl, costmap, lidar_follower | 激光雷达数据 |
| `/odom` | Odometry | odom_ekf.py | move_base, amcl, cartographer | 里程计 |
| `/wheel_odom` | Odometry | abot_driver | robot_pose_ekf | 原始轮式里程计 |
| `/imu/data` | Imu | imu_filter_madgwick | robot_pose_ekf | 滤波后 IMU 数据 |
| `/map` | OccupancyGrid | map_server / slam_gmapping | move_base, amcl | 占据栅格地图 |

### 4.3 里程计数据流

```
abot_driver (编码器) → /wheel_odom ─┐
                                    ├→ robot_pose_ekf → /robot_pose_ekf/odom_combined
abot_imu → imu_filter_madgwick → /imu/data ─┘
                                                      ↓
                                              odom_ekf.py → /odom
```

### 4.4 射击数据流 (3 种触发方式)

**方式 A — AR 标签跟踪**:
```
usb_cam → ar_track_alvar → /ar_pose_marker → ar_track → /cmd_vel + /shoot
```

**方式 B — VLM 视觉大模型**:
```
usb_cam → vlm_node (Yi-Vision) → /vision_result → navigation_multi_goals_vlm.py → (计算) → /shoot
```

**方式 C — 语音控制 (2026 demo)**:
```
麦克风 → 2026_shoot_demo.py (FunASR/Paraformer) → /chinese_topic
    → 2026_nav_end.py (关键词解析: 一二三、加减乘除) → /nav_end_topic + /operator_topic
    → navigation script → /move_base_simple/goal → move_base → (到达后) → /shoot
```

---

## 5. 关键串口协议

### 5.1 电机控制 (abot_driver → STM32)

- 端口: `/dev/abot` (通过 udev 规则)
- 波特率: 9600 8N1
- 协议: 自定义 simple_dataframe (主从模式)

### 5.2 射击控制 (shoot_control → 舵机)

- 端口: `/dev/shoot`
- 波特率: 9600 8N1
- 指令格式 (固定 8 字节):

| 指令 | 数据 |
|---|---|
| 发射 | `0x55 0x01 0x12 0x00 0x00 0x00 0x01 0x69` |
| 停止 | `0x55 0x01 0x11 0x00 0x00 0x00 0x01 0x68` |

### 5.3 激光雷达

- 端口: `/dev/rplidar`
- 波特率: 115200
- 帧名: `laser_link`

---

## 6. 各子系统详解

### 6.1 导航子系统

**move_base 参数架构**:
```
params/
├── costmap_common_params_abot.yaml   # 通用 costmap (机器人半径、障碍物层)
├── global_costmap_params.yaml         # 全局地图 (静态地图层)
├── local_costmap_params.yaml          # 局部地图 (滚动窗口)
├── global_planner_params.yaml         # 全局规划器 (A* / Dijkstra)
├── dwa_local_planner_params.yaml      # DWA 本地规划器 (加速度、速度限制)
└── move_base_params.yaml              # move_base 通用参数
```

**SLAM 方案对比**:
- **gmapping**: 常用，需里程计，粒子滤波，适合小场景
- **hector_slam**: 无需里程计，依赖高帧率激光雷达，适合不平坦地形
- **cartographer**: Google 方案，闭环检测，用于 zoo_2Dlidar_localication

**多目标点任务机制**: `navigation_multi_goals_*.py` 脚本从参数服务器读取坐标列表，
依次向 `/move_base_simple/goal` 发送 `PoseStamped`，等到达后发下一个。

### 6.2 VLM 子系统

- 模型: **Yi-Vision** (零一万物 lingyiwanwu)
- API 端: `https://api.lingyiwanwu.com/v1`
- 工作流:
  1. 订阅 `/usb_cam/image_raw`
  2. 收到拍照信号 (`/top_view_shot_node/im_flag` 参数 = 1)
  3. 保存图片 → 编码 base64 → 调用 Yi-Vision API
  4. 解析结果 → 发布 `/vision_result`
- 典型 prompt: *"图片中有一个计算式，请计算一下结果..."*

### 6.3 语音子系统

- **语音识别**: iFlytek MSC SDK (iat_publish) 或 FunASR Paraformer 模型 (2026_shoot_demo.py)
- **语音合成**: iFlytek MSC SDK (tts_subscribe) 或 WebSocket TTS (TTS_audio)
- **语音命令流**: 语音 → ASR → 中文文本 → 关键词匹配 → 导航/射击

### 6.4 跟踪子系统

- **AR 标签跟踪**: `track_tag/ar_track.cpp` — P 控制器，根据 AR 标签 x/z 偏差计算角速度/线速度
- **KCF 跟踪**: `tracker_pkg/kcf_kalman_tracker.py` — KCF + 卡尔曼滤波融合
- **KLT 光流**: `tracker_pkg/lk_tracker.py` — Lucas-Kanade 光流特征跟踪
- **激光跟人**: `lidar_follower/follower.py` — 根据激光点云聚类最近点，PID 控制跟随

---

## 7. 现存问题与建议

### 7.1 工程问题

| 问题 | 位置 | 建议 |
|---|---|---|
| **重复代码** | `robot_slam/` 下逐年复制的 `navigation_multi_goals_20XX.py` | 抽成通用导航框架 + YAML 配置文件，消除代码复制 |
| **Python 2/3 混用** | `shoot_2025.py` 使用 `#!/usr/bin/env python2` + `reload(sys)` | 统一到 Python 3 (Melodic 官方支持 Python 3 需额外配置) |
| **硬编码路径** | 多处硬编码 `/home/abot/abot_ws/src/...` | 使用 `rospy.get_param` 或 `rospack` 获取路径 |
| **noetic 迁移准备** | `tf` vs `tf2` 混用 | Melodic 支持 `tf2`，建议逐步迁移 |
| **包名冲突** | `imu_filter` 包名是 `filter` (package.xml) 但目录名是 `imu_filter` | 统一命名 |
| **串口设备名硬编码** | `/dev/shoot` 硬编码在 shoot_demo, shoot_2025.py | 通过 rosparam 配置 |
| **缺失 package.xml 依赖** | 多个包 (abot_vlm, robot_slam) 缺少 `build_depend`/`exec_depend` | 补充完整依赖声明 |
| **缺失 CMakeLists 构建** | abot_slam, abot_vlm, robot_slam 等只有 `catkin_python_setup()` 但未启用 | Python 包通过 CMakeLists 的 `catkin_install_python` 统一安装 |
| **CLAUDE.md 已存在** | 根目录已有 CLAUDE.md | 内容维持当前版本，此 `project_map.md` 为补充文档 |

### 7.2 架构建议

1. **增加状态机管理** — 当前导航 + 射击逻辑分散在多个脚本中，建议引入状态机 (如 `smach`) 统一管理 IDLE → NAV → TRACK → SHOOT → BACK 状态切换

2. **统一射击决策层** — 三种触发方式 (AR/VLM/语音) 各自独立决策，建议加一层仲裁节点，根据当前模式自动选择触发源

3. **参数集中化** — 导航目标点、PID 参数、串口配置分散在各 launch/yaml 中，建议集中到 `config/` 目录

4. **串口抽象层** — `shoot_cmd/SerialPort.cpp` 和 `abot_bringup/serial_transport.cpp` 功能重叠，可统一

5. **日志完善** — 当前 `ROS_INFO`/`print` 混用，建议统一使用 `rosconsole` + 日志级别
