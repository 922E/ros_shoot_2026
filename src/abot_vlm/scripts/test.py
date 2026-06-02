#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool, SetBoolResponse

vlm_id = 0

def vision_result_callback(msg):
    global vlm_id
    vlm_id1 = ['1']
    vlm_id2 = ['2']
    vlm_id3 = ['3']
    vlm_id4 = ['4']
    vlm_id5 = ['5']
    vlm_id6 = ['6']
    vlm_id7 = ['7']
    vlm_id8 = ['8']
    
    for keyword in vlm_id1:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '1点', accelerating the robot for 3 seconds")
            vlm_id = 1
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id2:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '2点', accelerating the robot for 3 seconds")
            vlm_id = 2
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id3:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '3点', accelerating the robot for 3 seconds")
            vlm_id = 3
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id4:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '4点', accelerating the robot for 3 seconds")
            vlm_id = 4
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id5:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '5点', accelerating the robot for 3 seconds")
            vlm_id = 5
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id6:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '6点', accelerating the robot for 3 seconds")
            vlm_id = 6
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id7:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '7点', accelerating the robot for 3 seconds")
            vlm_id = 7
            vlm_id_pub.publish(vlm_id)
            break
    for keyword in vlm_id8:
        if keyword in msg.data:
            rospy.loginfo("Detected keyword related to '8点', accelerating the robot for 3 seconds")
            vlm_id = 8
            vlm_id_pub.publish(vlm_id)
            break

def main():
    global vlm_id_pub
    rospy.init_node('vision_result_subscriber_node', anonymous=True)
    
    # 订阅 vision_result 话题
    rospy.Subscriber('vision_result', String, vision_result_callback)
    
    # 创建发布者，发布 vlm_id 到 vlm_id_topic 话题
    vlm_id_pub = rospy.Publisher('vlm_id_topic', Int32, queue_size=10)
    
    rospy.loginfo('视觉结果订阅节点启动成功！')
    
    rospy.spin()

if __name__ == '__main__':
    main()
