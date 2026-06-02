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
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([30,255,255])
	#lower_red = np.array([0,50,0])
        #upper_red = np.array([12,255,255])


        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	#mask = cv2.inRange(hsv, lower_red, upper_red)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

        # Apply Canny edge detection
        edges = cv2.Canny(res, 50, 150)

        # Apply Hough transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, maxLineGap=200)

        if lines is not None:
            # 计算车道线的平均位置
            x_pos = []
            y_pos = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_pos.append((x1 + x2) / 2)
                y_pos.append((y1 + y2) / 2)
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            x_pos_avg = sum(x_pos) / len(x_pos)
            y_pos_avg = sum(y_pos) / len(y_pos)
            # 计算机器人需要偏转的角度
            angle = -(float(x_pos_avg) - float(cv_image.shape[1] / 2)) * 0.01
            # 计算机器人需要前进的距离
            distance = 0.2 * (float(cv_image.shape[0]) - float(y_pos_avg))
            # 更新cmd_vel消息
            self.twist.linear.x = min(max(distance, -0.1), 0.1)
            self.twist.angular.z = min(max(angle, -0.25), 0.25)
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    lf = LaneFollower()
    rospy.spin()
