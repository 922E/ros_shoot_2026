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
	    # 使用cv_bridge将ROS图像消息转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # 将BGR图像转换成HSV图像
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 定义黄色在HSV颜色空间中的范围
        lower_yellow = np.array([20,100,100]) # 黄色的下限
        upper_yellow = np.array([30,255,255]) # 黄色的上限

        # 对HSV图像进行阈值处理，提取出黄色部分
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 将mask和原始图像进行按位与操作
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

        # 应用Canny边缘检测
        edges = cv2.Canny(res, 50, 150)

        # 应用霍夫变换检测直线
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, maxLineGap=200)

        if lines is not None:
            # 计算车道线的平均位置
            x_pos = []
            y_pos = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_pos.append((x1 + x2) / 2)
                y_pos.append((y1 + y2) / 2)
		#在图像上绘制检测到的直线
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 绘制红色车道线中心点
            center_x = int(sum(x_pos) / len(x_pos))
            center_y = int(sum(y_pos) / len(y_pos))
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

            x_pos_avg = sum(x_pos) / len(x_pos)
            y_pos_avg = sum(y_pos) / len(y_pos)
            # 计算机器人需要偏转的角度
            angle = -(float(x_pos_avg) - float(cv_image.shape[1] / 2)) * 0.01
            # 计算机器人需要前进的距离，使用y方向速度调整车道线居中
            distance = 0.2 * (float(cv_image.shape[0]) - float(y_pos_avg))
            # 更新cmd_vel消息
            self.twist.linear.x = min(max(distance, -0.1), 0.1)
            self.twist.linear.y = angle * 0.07  # 使用y方向速度调整车道线居中
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    lf = LaneFollower()
    rospy.spin()

