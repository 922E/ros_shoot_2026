#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32

# 初始化全局变量
vlm_id = 0
# 定义一个字典，将关键词和对应的 id 以及日志信息关联起来
keyword_mapping = {
    '1': (1, "Detected keyword related to '1点'"),
    '2': (2, "Detected keyword related to '2点'"),
    '3': (3, "Detected keyword related to '3点'"),
    '4': (4, "Detected keyword related to '4点'"),
    '5': (5, "Detected keyword related to '5点'"),
    '6': (6, "Detected keyword related to '6点'"),
    '7': (7, "Detected keyword related to '7点'"),
    '8': (8, "Detected keyword related to '8点'")
}


def vision_result_callback(msg):
    global vlm_id
    for keyword, (new_vlm_id, log_msg) in keyword_mapping.items():
        if keyword in msg.data:
            rospy.loginfo(log_msg)
            vlm_id = new_vlm_id
            try:
                # 尝试发布 vlm_id
                vlm_id_pub.publish(vlm_id)
            except Exception as e:
                rospy.logerr(f"Failed to publish vlm_id: {e}")
            break


def main():
    global vlm_id_pub
    # 初始化 ROS 节点
    rospy.init_node('vision_result_subscriber_node', anonymous=True)
    # 订阅 vision_result 话题
    rospy.Subscriber('vision_result', String, vision_result_callback)
    # 创建发布者，发布 vlm_id 到 vlm_id_topic 话题
    vlm_id_pub = rospy.Publisher('vlm_id_topic', Int32, queue_size=10)
    rospy.loginfo('视觉结果订阅节点启动成功！')
    try:
        # 进入 ROS 主循环
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 节点被中断")


if __name__ == '__main__':
    main()

