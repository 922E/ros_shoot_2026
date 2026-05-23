#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32

# 初始化全局变量
NAV_END = None
OPERATOR = None

# 定义运算符映射字典
OPERATORS = {
    "加法": "+",
    "减法": "-",
    "乘法": "*",
    "除法": "/"
}

# 定义关键词映射字典
KEYWORDS = {
    "一": 11,
    "二": 12,
    "三": 13
}

def chinese_callback(msg):
    """
    处理接收到的中文消息，检测关键词和运算符。
    :param msg: 接收到的消息
    """
    global NAV_END, OPERATOR, arrive_pub  # 声明全局变量

    # 检测关键词
    for keyword, value in KEYWORDS.items():
        if keyword in msg.data:
            NAV_END = value
            rospy.loginfo(f"检测到关键词: {keyword}, 设置 nav_end = {NAV_END}")
            arrive_str = "终点为十{}".format(keyword)
            arrive_pub.publish(arrive_str)
            break

    # 发布 nav_end 的值
    try:
        nav_end_pub.publish(NAV_END)
    except Exception as e:
        rospy.logerr(f"发布 nav_end 时出错: {e}")

    # 检测运算符
    for operator_keyword, operator_symbol in OPERATORS.items():
        if operator_keyword in msg.data:
            OPERATOR = operator_symbol
            rospy.loginfo(f"检测到运算符关键词: {operator_keyword}, 运算符为: {OPERATOR}")
            arrive_str = "运算符为{}".format(operator_keyword)
            arrive_pub.publish(arrive_str)
            break

    # 发布检测到的运算符
    if OPERATOR is not None:
        try:
            operator_pub.publish(OPERATOR)
        except Exception as e:
            rospy.logerr(f"发布运算符时出错: {e}")

def chinese_subscriber():
    """
    初始化 ROS 节点，创建发布者和订阅者。
    """
    global nav_end_pub, operator_pub, arrive_pub  # 声明全局变量
    rospy.init_node('chinese_subscriber', anonymous=True)

    # 创建发布者
    arrive_pub = rospy.Publisher('/voiceWords',String,queue_size=10)
    nav_end_pub = rospy.Publisher('nav_end_topic', Int32, queue_size=10)
    operator_pub = rospy.Publisher('operator_topic', String, queue_size=10)

    # 创建订阅者
    rospy.Subscriber("chinese_topic", String, chinese_callback)

    rospy.loginfo("Chinese subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        chinese_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 节点被中断")
