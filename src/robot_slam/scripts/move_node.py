#!/usr/bin/env python
#coding: utf-8
import rospy
from geometry_msgs.msg import Twist

def move_robot(linear_x, angular_z):
    rospy.init_node('move_robot_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  

    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        linear_x = 0.2   
        angular_z = 0.5  
        move_robot(linear_x, angular_z)
    except rospy.ROSInterruptException:
        pass



