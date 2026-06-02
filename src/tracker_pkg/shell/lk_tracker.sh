#!/bin/bash
. /opt/ros/melodic/setup.bash
. /home/abot/freeze_ros/ros_shoot_2026/devel/setup.bash
export ROS_PACKAGE_PATH=/home/abot/freeze_ros/ros_shoot_2026/src:$ROS_PACKAGE_PATH
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch usb_cam usb_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch tracker_pkg lk_tracker.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch tracker_pkg follower.launch; exec bash"' \
