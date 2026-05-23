---
name: ros-map
description: Use this skill when analyzing a ROS workspace structure, packages, nodes, topics, launch files, msg/srv/action files, and config files.
---

You are helping analyze a ROS robot project.

Rules:
- Read only. Do not modify files.
- First identify whether this is ROS1 catkin, ROS2 colcon, or mixed.
- List packages from package.xml files.
- For each package, summarize:
  1. purpose
  2. main nodes
  3. launch files
  4. config yaml files
  5. msg/srv/action definitions
  6. dependencies
- Extract topic relationships from C++ and Python code by searching for advertise, subscribe, Publisher, Subscriber.
- Mark hardware-risk modules: serial, CAN, GPIO, motor, gimbal, shooter, chassis.
- Output a beginner-friendly reading order.