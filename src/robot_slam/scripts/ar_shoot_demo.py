#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# 上面这行是为了告诉操作系统，这是一个Python脚本，可以直接运行

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
import math
import serial
import time
from std_msgs.msg import String
serialPort = "/dev/shoot"
baudRate = 9600
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
# 定义Yaw阈值
Yaw_th = 0.07
Min_y = -0.36
Max_y = -0.29
class ARTracker:
    def __init__(self):
        # 初始化ROS节点，命名为'ar_tracker_node'，并设置为匿名节点
        rospy.init_node('ar_tracker_node', anonymous=True)
        # 创建一个订阅者，订阅AR标记的消息，消息类型为AlvarMarkers，回调函数为ar_cb
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        # 创建一个发布者，用于发布Twist类型的消息到/cmd_vel话题
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

    # AR标记消息的回调函数
    def ar_cb(self, data):
        global ar_x_0, ar_x_0_abs, Yaw_th,ar_y_0,Min_y,Max_y
        # 获取所有AR标记
        ar_markers = data
        # 遍历接收到的所有AR标记
        for marker in data.markers:
            if marker.id==1 :
              ar_x_0 = marker.pose.pose.position.x
              ar_y_0 = marker.pose.pose.position.y
	      print "X"
              print ar_x_0
	      print "Y"
              print ar_y_0
	      ar_x_0_abs = abs(ar_x_0)
	      if ar_x_0_abs >= Yaw_th :
		msg = Twist()
                msg.angular.z = -1 * ar_x_0
                self.pub.publish(msg)
                #print(ar_x_0_abs)
	      elif Min_y <= ar_y_0 <= Max_y:
	        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
           	print "shoot"
           	rospy.sleep(0.1)
           	ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                rospy.sleep(2)

if __name__ == '__main__':
    try:
        # 创建ARTracker对象
        ar_tracker = ARTracker()
        # 进入ROS事件循环
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

