# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS Melodic workspace for an Ackermann-style robot ("abot") with shooting capability. The robot performs SLAM, navigation, object tracking, voice interaction, visual-language-model (VLM) recognition, and projectile firing.

## Build & Run

```bash
# Build the workspace
cd ~/abot_ws
catkin_make

# Source the workspace
source devel/setup.bash

# Launch robot base + lidar
roslaunch abot_bringup robot.launch

# Launch navigation with shooting map
roslaunch robot_slam navigation_shoot.launch

# Launch individual nodes
rosrun shoot_cmd shoot_control
rosrun cam_track cam_track_node
rosrun track_tag ar_track
```

ROS Melodic target on Ubuntu 18.04. Python scripts use shebangs pointing to `/home/abot/anaconda3/envs/py39/bin/python` (Python 3.9 conda env) or `/usr/bin/env python3`.

## Package Architecture

| Package | Language | Purpose |
|---|---|---|
| **abot_base** | C++/Python | Robot base driver (serial motor control), IMU, lidar filters, URDF model. `abot_driver` is the main node. |
| **robot_slam** | Python | High-level navigation: multi-goal missions, speech-triggered nav, AR-tag shooting demos. Entry points: `mission.py`, `2026_shoot_demo.py`, `2026_nav_end.py` |
| **shoot_cmd** | C++ | Serial-port shooting mechanism control. Publishes `/shoot` topic. Protocol: `0x55 0x01 0x12...` to fire, `0x55 0x01 0x11...` to stop. |
| **cam_track** | C++ | Camera-based person/object tracking, publishes to `/cmd_vel` |
| **track_tag** | C++ | AR tag tracking via `ar_track_alvar`. Subscribes `/ar_pose_marker`, publishes `/cmd_vel` and `/shoot` |
| **abot_vlm** | Python | Vision-Language Model using Yi-Vision (lingyiwanwu) API. Subscribes `/usb_cam/image_raw`, publishes `vision_result` |
| **robot_voice** | C++ | Voice recognition/synthesis via iFlytek MSC SDK. Nodes: `tts_subscribe`, `iat_publish`, `voice_assistant` |
| **TTS_audio** | Python | WebSocket-based TTS service using ROS service `StringService` |
| **abot_slam** | Launch/Python | SLAM launch files (gmapping, cartographer, hector) and navigation mission scripts |
| **tracker_pkg** | Python | Visual tracking (KCF + Kalman filter, KLT optical flow) |
| **lidar_follower** | Python | Lidar-based person following |
| **color_pkg** | Python | Color detection (fire detection, line following) |
| **face_pkg** | Python | Face detection/recognition |
| **imu_filter** | C++ | Madgwick/Mahony IMU filter |
| **abot_find** | Launch | find_object_2d wrapper for object detection via feature matching |
| **abot_object_detect** | Python | HSV color tool, face Haar cascade, people detection |
| **hector_slam** | C++ | Hector SLAM library (3rd party) |

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | Twist | Velocity commands |
| `/shoot` | String | Fire control ("shoot" / "stopshoot") |
| `/move_base_simple/goal` | PoseStamped | Navigation goal |
| `/move_base/status` | GoalStatusArray | Navigation status |
| `/ar_pose_marker` | AlvarMarkers | AR tag poses |
| `/usb_cam/image_raw` | Image | Camera feed |
| `/vision_result` | String | VLM recognition result |
| `/chinese_topic` | String | ASR recognition text |
| `/nav_end_topic` | Int32 | Navigation endpoint ID |
| `/operator_topic` | String | Math operator for calculation |
| `/mission/arrived` | String | Waypoint arrival signal |
| `/voiceWords` | String | Voice feedback text |
| `/snowman/ask` | String | Voice command trigger |
| `/robot_voice/tts_topic` | String | TTS playback request |
| `/initialpose` | PoseWithCovarianceStamped | Initial pose estimate |

## Code Conventions

- **C++ nodes** follow `main()` with `ros::init` + `ros::spin()` pattern
- **Python nodes** use `rospy.init_node` + `rospy.spin()` with shebang line
- Parameters loaded from yaml files or via `nh.param<>` / `rospy.get_param()`
- Launch files organize robot subsystems (base, nav, camera, shoot)
- C++ code uses ROS Melodic APIs (`tf`, not `tf2` in most packages)
- Serial communication to motor driver uses custom `SerialPort`/`serial_transport` classes (8N1, 9600 baud default)
- P-controller pattern for tracking (position error -> velocity via P gain + saturation)
