import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def chinese_callback(msg):
    if '染'or'毒' or '地' or '带'  in msg.data:
        rospy.loginfo("Detected keyword '前方通过染毒地带', accelerating the robot for 3 seconds")

        # 创建速度消息
        twist = Twist()
        twist.linear.x = 1.5  # 设置线速度为1.5米每秒

        # 发布速度消息
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # 记录当前时间
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz的频率发布消息

        # 持续发布消息3秒
        while rospy.Time.now() - start_time < rospy.Duration(3):
            cmd_vel_pub.publish(twist)
            rate.sleep()

        # 停止运动
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
    else:
        rospy.loginfo("Keyword '好' not detected")

def chinese_subscriber():
    rospy.init_node('chinese_subscriber', anonymous=True)
    rospy.Subscriber("chinese_topic", String, chinese_callback)
    rospy.loginfo("Chinese subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    chinese_subscriber()


