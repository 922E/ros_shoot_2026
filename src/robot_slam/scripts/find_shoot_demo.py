#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, Twist
import serial
import time
from std_msgs.msg import String
serialPort = "/dev/shoot"
baudRate = 9600
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)

class object_position:
    def __init__(self):
        rospy.init_node('object_position_node', anonymous=True)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

    def find_cb(self, data):
        global flog0, flog1
        point_msg = data
        flog0 = point_msg.x - 320
        flog1 = abs(flog0)
        if abs(flog1) > 0.5:
            msg = Twist()
            msg.angular.z = -0.01 * flog0
            self.pub.publish(msg)
        elif abs(flog1) <= 0.5:
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print ('打印射击')
            time.sleep(0.08)
            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

if __name__ == '__main__':
    try:
        object_position = object_position()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

