#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower')  # 初始化ROS节点
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)  # 订阅图像消息
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # 发布运动控制消息
        self.error_pub = rospy.Publisher('/lane_error', Float32, queue_size=1)  # 发布车道偏差信息

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")  # 将ROS图像消息转换为OpenCV格式
        except Exception as e:
            print(e)
        
        yellow_mask = self.detect_yellow(cv_image)  # 检测黄色车道线
        center_offset, lane_image = self.calculate_center_offset(cv_image, yellow_mask)  # 计算车道线中心偏移和绘制车道线图像
        twist_msg = self.calculate_twist(center_offset)  # 计算运动控制消息
        
        self.cmd_vel_pub.publish(twist_msg)  # 发布运动控制消息
        error_msg = Float32()
        error_msg.data = center_offset
        self.error_pub.publish(error_msg)  # 发布车道偏差信息

        cv2.imshow("Lane Image", lane_image)  # 在屏幕上显示车道线图像
        cv2.waitKey(1)

    def detect_yellow(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return yellow_mask

    def calculate_center_offset(self, image, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lane_image = np.zeros_like(image)
        center_offset = 0
        if len(contours) > 1:  # 假设检测到两条车道线
           largest_contour1 = max(contours, key=cv2.contourArea)
           contours.remove(largest_contour1)  # 移除第一条车道线
           largest_contour2 = max(contours, key=cv2.contourArea)  # 寻找第二条车道线
           moment1 = cv2.moments(largest_contour1)
           moment2 = cv2.moments(largest_contour2)
           if moment1["m00"] != 0 and moment2["m00"] != 0:
              cx1 = int(moment1["m10"] / moment1["m00"])  # 第一条车道线中心点坐标
              cx2 = int(moment2["m10"] / moment2["m00"])  # 第二条车道线中心点坐标
              height, width = mask.shape[:2]
              center_offset = (cx1 + cx2 - width) / (width)  # 计算车道线中心偏移
              cv2.drawContours(lane_image, [largest_contour1, largest_contour2], -1, (0, 255, 0), 2)  # 绘制两条车道线
              cv2.circle(lane_image, (cx1, int(height/2)), 5, (0, 0, 255), -1)  # 标记第一条车道线中心点
              cv2.circle(lane_image, (cx2, int(height/2)), 5, (0, 0, 255), -1)  # 标记第二条车道线中心点
        return center_offset, lane_image

    def calculate_twist(self, center_offset):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # 前进速度
        twist_msg.linear.y = center_offset * 0.5  # y方向速度调整居中
        twist_msg.angular.z = center_offset * 0.5  # 转向调整
        return twist_msg

if __name__ == '__main__':
    lane_follower = LaneFollower()
    rospy.spin()

