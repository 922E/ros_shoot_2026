#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
import threading
import os
music1_path="/home/abot/abot_ws/src/robot_slam/mp3/feng.mp3"

def chinese_callback(msg):
    keywords_fast = ['染', '毒', '地', '带']
    keywords_slow = ['泥', '泞', '路', '段']

    for keyword in keywords_fast:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '染毒地带', accelerating the robot for 3 seconds")
            os.system('mplayer %s' % music1_path)
           
            accelerate_robot(1.5, 3)
            
            break

    for keyword in keywords_slow:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '泥泞路段', slowing down the robot for 6 seconds")
            os.system('mplayer %s' % music1_path)
            slow_down_robot(0.5, 6)
           
            break

def accelerate_robot(speed, duration):
    twist = Twist()
    twist.linear.x = speed
    global cmd_vel_pub2
    cmd_vel_pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)

    while rospy.Time.now() - start_time < rospy.Duration(duration):
        cmd_vel_pub2.publish(twist)
        rate.sleep()

    twist.linear.x = 0.0
    cmd_vel_pub2.publish(twist)

def slow_down_robot(speed, duration):
    twist = Twist()
    twist.linear.x = speed
    global cmd_vel_pub3
    cmd_vel_pub3 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)

    while rospy.Time.now() - start_time < rospy.Duration(duration):
        cmd_vel_pub3.publish(twist)
        rate.sleep()

    twist.linear.x = 0.0
    cmd_vel_pub3.publish(twist)

def chinese_subscriber():
    global cmd_vel_pub2
    global cmd_vel_pub3
    rospy.init_node('chinese_subscriber', anonymous=True)
    rospy.Subscriber("chinese_topic", String, chinese_callback)
    rospy.loginfo("Chinese subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    chinese_subscriber()
