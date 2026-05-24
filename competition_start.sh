### 2026 射击挑战赛 - 一键启动 ###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; roslaunch robot_slam navigation_freeze.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/abot_ws/devel/setup.bash; source ~/freeze_ros/ros_shoot_2026/devel/setup.bash; roslaunch robot_slam competition_2026.launch; exec bash"'
