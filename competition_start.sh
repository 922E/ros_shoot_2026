#!/bin/bash
### 2026 射击挑战赛 - 一键启动 ###
### 所有 launch 显式指向 freeze_ros，避免加载旧 workspace 中的同名 package。 ###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /opt/ros/melodic/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH; roslaunch $HOME/freeze_ros/ros_shoot_2026/src/abot_base/abot_bringup/launch/robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH; roslaunch $HOME/freeze_ros/ros_shoot_2026/src/robot_slam/launch/navigation_freeze.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH; roslaunch $HOME/freeze_ros/ros_shoot_2026/src/track_tag/launch/usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH; roslaunch $HOME/freeze_ros/ros_shoot_2026/src/track_tag/launch/ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH; roslaunch $HOME/freeze_ros/ros_shoot_2026/src/abot_find/launch/find_object_2d_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source /opt/ros/melodic/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH; roslaunch $HOME/freeze_ros/ros_shoot_2026/src/robot_slam/launch/competition_2026.launch; exec bash"'
