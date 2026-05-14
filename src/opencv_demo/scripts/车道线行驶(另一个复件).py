#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LaneFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Apply Canny edge detection
        edges = cv2.Canny(res, 50, 150)

        # Apply Hough transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, maxLineGap=200)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 < 320 and x2 < 320:  # 只处理图像左侧的车道线
                    # 计算车道线的中点y坐标
                    mid_y = (y1 + y2) / 2
                    # 控制机器人向左移动，使得车道线中点保持在屏幕左侧
                    move_cmd = 0.01 * (mid_y - 320)  # 240 是屏幕中点的y坐标
                    self.twist.linear.x = 0.1  # 前进速度
                    self.twist.angular.z = -move_cmd  # 因为麦克纳姆轮底盘支持全向移动，所以这里是负号
                    self.cmd_vel_pub.publish(self.twist)
                    break  # 只处理第一条检测到的左侧车道线

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    lf = LaneFollower()
    rospy.spin()

