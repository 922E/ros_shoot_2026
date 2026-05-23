#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tkinter as tk
from time import sleep
import threading
import os
music1_path="/home/abot/abot_ws/src/robot_slam/mp3/feng.mp3"
def create_flashing_window(color, duration):
    def flash():
        root = tk.Tk()
        root.attributes('-fullscreen', True)
        canvas = tk.Canvas(root, width=2000, height=2000)
        canvas.pack()
        canvas.configure(bg=color)

        start_time = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start_time) < duration:
            canvas.configure(bg=color)
            root.update()
            sleep(0.5)  # 间隔0.5秒
            canvas.configure(bg="white")
            root.update()
            sleep(0.5)

        root.destroy()
    
    thread = threading.Thread(target=flash)
    thread.start()

def chinese_callback(msg):
    keywords_fast = ['染', '毒', '地', '带']
    keywords_slow = ['泥', '泞', '路', '段']

    for keyword in keywords_fast:
        if keyword in msg.data:
            create_flashing_window("red", 4)  # 红色背景闪烁
            
            break

    for keyword in keywords_slow:
        if keyword in msg.data:
            create_flashing_window("green", 4)  # 绿色背景闪烁
           
            break



def chinese_subscriber1():
    rospy.init_node('chinese_subscriber1', anonymous=True)
    rospy.Subscriber("chinese_topic1", String, chinese_callback)
    rospy.loginfo("Chinese subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    chinese_subscriber1()
