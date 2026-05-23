#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import actionlib
import serial
import time
from std_msgs.msg import String
serialPort = "/dev/shoot"
baudRate = 9600
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from geometry_msgs.msg import Twist
from geometry_msgs.msg  import Point
from std_msgs.msg import String, Int32
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import os
time = 0
id = 255
flog0 = 0
flog1 = 0
flog2 = 255
count = 0
move_flog = 0
point_msg = None
case = 0
Yaw_th = 0.1  # 旋转靶角度阈值，可根据实际情况调整
Yaw_th1 = 0.1  # 移动靶角度阈值，可根据实际情况调整
Min_y = -0.1  # 旋转靶 y 坐标最小值，可根据实际情况调整
Max_y = 0.1   # 旋转靶 y 坐标最大值，可根据实际情况调整
target_id_rotating = None
target_id_moving = None
# 新增标志位
should_attack_circular = False
should_attack_rotating = False
should_attack_moving = False


class navigation_demo:
    def __init__(self):
        # 初始化发布者和订阅者
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.circular_target)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.rotating_target)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.moving_target)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        self.target_id_rotating_sub = rospy.Subscriber('target_id_rotating', Int32, self.target_id_rotating_callback)
        self.target_id_moving_sub = rospy.Subscriber('target_id_moving', Int32, self.target_id_moving_callback)

    def end(self):
        global time
        msg = Twist()
        msg.linear.x = -0.3
        msg.linear.y = -0.3
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        while time <= 10:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time = time + 1

    def circular_target(self, data):
        global id, flog0, flog1, flog2, count, move_flog, point_msg, case, should_attack_circular
        if not should_attack_circular:
            return
        id = 255
        point_msg = data
        flog0 = point_msg.x - 320
        flog1 = abs(flog0)
        if abs(flog1) > 10 and point_msg.z == 34 and flog2 >= 255 and case == 0:
            msg = Twist()
            msg.angular.z = -0.02 * flog0
            self.pub.publish(msg)
            print(flog1)
        elif abs(flog1) <= 15 and point_msg.z == 34 and flog2 >= 255:
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print "shoot"
            rospy.sleep(0.09)
            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
            flog2 = flog2 - 1
            should_attack_circular = False

    def rotating_target(self, data):
        global id, flog, msg, ar_x,  ar_x_0_abs, ar_y_0, Min_y, Max_y, should_attack_rotating
        if not should_attack_rotating:
            return
        ar_markers = data
        for marker in data.markers:
            if marker.id == target_id_rotating : 
                ar_x_0 = marker.pose.pose.position.x
                ar_y_0 = marker.pose.pose.position.y
                ar_x_0_abs = abs(ar_x_0)
                if ar_x_0_abs >= Yaw_th:
                    msg = Twist()
                    msg.angular.z = -1 * ar_x_0
                    self.pub.publish(msg)
                    print(ar_x_0_abs)
                elif ar_y_0 <= Max_y and ar_y_0 >= Min_y:
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print "shoot"
                    rospy.sleep(0.1)
                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    rospy.sleep(2)
                    should_attack_rotating = False

    def moving_target(self, data):
        global id, flog, msg, ar_x,  ar_x_0_abs, should_attack_moving
        if not should_attack_moving:
            return
        ar_markers = data
        for marker in data.markers:
            if marker.id == target_id_moving :  
                ar_x_0 = marker.pose.pose.position.x
                ar_x_0_abs = abs(ar_x_0)
                print(ar_x_0_abs)
                if ar_x_0_abs >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.95 * ar_x_0
                    self.pub.publish(msg)
                elif ar_x_0_abs < Yaw_th1:
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print "shoot"
                    rospy.sleep(0.1)
                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    should_attack_moving = False

    def target_id_rotating_callback(self, msg):
        global target_id_rotating
        target_id_rotating = msg.data
        rospy.loginfo("收到 target_id_rotating: %d", target_id_rotating)

    def target_id_moving_callback(self, msg):
        global target_id_moving
        target_id_moving = msg.data
        rospy.loginfo("收到 target_id_moving: %d", target_id_moving)

    def set_pose(self, p):
        if self.move_base is None:
            return False
        x, y, th = p
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True


if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    # 替换中文逗号为英文逗号
    goalListX = goalListX.replace(u'，', ',')
    goalListY = goalListY.replace(u'，', ',')
    goalListYaw = goalListYaw.replace(u'，', ',')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print('Please 1 to continue: ')
    input = raw_input()
    print(goals)
    r = rospy.Rate(1)

    def publish_audio():
        # 创建发布者对象，发布到 audio_topic 主题，消息类型为 String
        publisher = rospy.Publisher('audio_topic', String, queue_size=10)
        # 消息发布次数
        publish_count = 2
        # 循环直到发布次数达到设定值
        while not rospy.is_shutdown() and publish_count > 0:
            # 创建要发布的字符串消息
            audio_data = "audio message"  # 这里可以替换成您要发布的消息内容
            # 发布消息
            publisher.publish(audio_data)
            # 打印消息发布状态
            rospy.loginfo("Published: %s", audio_data)
            # 减少发布次数
            publish_count -= 1
            # 保持循环频率
            rate = rospy.Rate(1)  # 确保 rate 在循环内部定义，以便每次循环都能更新频率
            rate.sleep()

    navi = navigation_demo()
    if input == '1':
        # 语音识别打击目标
        publish_audio()
        rospy.sleep(18)

        # 打击环形靶
        navi.goto(goals[0])
        global should_attack_circular
        should_attack_circular = True

        # 打击旋转靶
        navi.goto(goals[1])
        global should_attack_rotating
        should_attack_rotating = True

        # 打击移动靶
        navi.goto(goals[2])
        global should_attack_moving
        should_attack_moving = True

        # 回到终点
        navi.goto(goals[3])
        navi.end()
        r.sleep()
        #navi.goto(goals[1])
        #rospy.sleep(5)
        #navi.goto(goals[2])
        #rospy.sleep(5)
        #navi.goto(goals[3])
        #rospy.sleep(5)

