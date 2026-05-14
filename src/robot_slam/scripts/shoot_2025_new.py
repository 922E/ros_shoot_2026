#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
机器人比赛主控节点
功能：
1. 接收语音识别目标ID
2. 导航至指定目标点
3. 控制射击机构打击目标
4. 状态机管理比赛流程
"""

import rospy
import actionlib
import serial
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, Int32
from tf_conversions import transformations
from math import pi

# 串口通信配置
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600

# 全局常量配置
YAW_THRESHOLD_ROTATING = 0.1    # 旋转靶角度阈值
YAW_THRESHOLD_MOVING = 0.1      # 移动靶角度阈值 
TARGET_Y_RANGE = (-0.1, 0.1)    # 有效射击Y轴范围

class ShootingCompetition:
    """射击比赛主控制类"""
    
    def __init__(self):
        # 初始化ROS组件
        self._init_ros_components()
        
        # 初始化运动控制
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        
        # 初始化动作客户端
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        
        # 初始化状态变量
        self.current_stage = "WAIT_START"  # 当前比赛阶段
        self.target_ids = {
            'rotating': None, 
            'moving': None
        }
        
        # 初始化串口
        self._init_serial()

    def _init_ros_components(self):
        """初始化ROS发布者和订阅者"""
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.audio_pub = rospy.Publisher('audio_topic', String, queue_size=10)  # 新增音频发布者
        
        # 订阅目标ID
        rospy.Subscriber('target_id_rotating', Int32, self._rotating_id_callback)
        rospy.Subscriber('target_id_moving', Int32, self._moving_id_callback)
        
        # 订阅传感器数据
        rospy.Subscriber('/object_position', Point, self._circular_target_handler)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._ar_marker_handler)

    def _init_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, parity="N", bytesize=8, timeout=1)
            rospy.loginfo("串口初始化成功")
        except Exception as e:
            rospy.logerr("串口初始化失败: %s", str(e))
            raise

    def publish_audio(self):
        """
        语音识别触发函数
        发布音频指令到指定话题
        """
        publish_count = 2
        while not rospy.is_shutdown() and publish_count > 0:
            audio_data = "start_recognition"  # 启动语音识别指令
            self.audio_pub.publish(audio_data)
            rospy.loginfo("已发送语音识别指令: %s", audio_data)
            publish_count -= 1
            rate = rospy.Rate(1)
            rate.sleep()

    def _rotating_id_callback(self, msg):
        """旋转靶ID回调函数"""
        self.target_ids['rotating'] = msg.data
        rospy.loginfo("收到旋转靶ID: %d", msg.data)

    def _moving_id_callback(self, msg):
        """移动靶ID回调函数"""
        self.target_ids['moving'] = msg.data
        rospy.loginfo("收到移动靶ID: %d", msg.data)

    def _fire(self):
        """执行射击动作"""
        try:
            self.ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')  # 激发指令
            rospy.sleep(0.1)
            self.ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')  # 复位指令
        except Exception as e:
            rospy.logerr("射击机构控制失败: %s", str(e))

    def _circular_target_handler(self, data):
        """环形靶处理回调函数"""
        if self.current_stage != "CIRCULAR_STAGE":
            return

        # 计算水平偏移量
        x_offset = data.x - 320  # 320为图像中心水平坐标
        if abs(x_offset) > 10 and data.z == 34:  # 检测到有效目标
            # 调整机器人朝向
            twist = Twist()
            twist.angular.z = -0.02 * x_offset
            self.cmd_vel_pub.publish(twist)
        elif abs(x_offset) <= 15:  # 进入有效射击范围
            self._fire()
            self.current_stage = "ROTATING_STAGE"  # 进入下一阶段

    def _ar_marker_handler(self, data):
        """AR标记处理回调函数"""
        # 处理旋转靶逻辑
        if self.current_stage == "ROTATING_STAGE":
            self._process_rotating_target(data.markers)
        
        # 处理移动靶逻辑
        elif self.current_stage == "MOVING_STAGE":
            self._process_moving_target(data.markers)

    def _process_rotating_target(self, markers):
        """处理旋转靶逻辑"""
        target_id = self.target_ids['rotating']
        for marker in markers:
            if marker.id == target_id:
                x_pos = marker.pose.pose.position.x
                y_pos = marker.pose.pose.position.y
                
                # 角度调整
                if abs(x_pos) >= YAW_THRESHOLD_ROTATING:
                    twist = Twist()
                    twist.angular.z = -1 * x_pos
                    self.cmd_vel_pub.publish(twist)
                # 位置验证
                elif TARGET_Y_RANGE[0] <= y_pos <= TARGET_Y_RANGE[1]:
                    self._fire()
                    self.current_stage = "MOVING_STAGE"  # 进入下一阶段

    def _process_moving_target(self, markers):
        """处理移动靶逻辑"""
        target_id = self.target_ids['moving']
        for marker in markers:
            if marker.id == target_id:
                x_pos = marker.pose.pose.position.x
                # 动态调整
                if abs(x_pos) >= YAW_THRESHOLD_MOVING:
                    twist = Twist()
                    twist.angular.z = -0.95 * x_pos
                    self.cmd_vel_pub.publish(twist)
                else:
                    self._fire()
                    self.current_stage = "FINISH_STAGE"  # 比赛结束

    def _navigate_to_goal(self, position, timeout=60):
        """导航到指定目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        
        # 转换四元数
        q = transformations.quaternion_from_euler(0.0, 0.0, position[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)
        return self.move_base.wait_for_result(rospy.Duration(timeout))

    def run_competition(self, goals):
        """执行完整比赛流程"""
        # 等待启动指令
        rospy.loginfo("等待启动指令...")
        while not rospy.is_shutdown() and self.current_stage == "WAIT_START":
            user_input = raw_input("请输入1开始比赛: ")
            if user_input == '1':
                self.current_stage = "CIRCULAR_STAGE"
                break

        try:
            # ==== 新增语音识别流程 ====
            self.publish_audio()
            rospy.loginfo("等待语音识别结果...")
            rospy.sleep(18)  # 等待语音识别完成

            # 阶段1: 打击环形靶
            if self._navigate_to_goal(goals[0]):
                rospy.loginfo("已到达环形靶位置")
                while self.current_stage == "CIRCULAR_STAGE":
                    rospy.sleep(0.1)

            # 阶段2: 打击旋转靶
            if self._navigate_to_goal(goals[1]):
                rospy.loginfo("已到达旋转靶位置")
                while self.current_stage == "ROTATING_STAGE":
                    rospy.sleep(0.1)

            # 阶段3: 打击移动靶
            if self._navigate_to_goal(goals[2]):
                rospy.loginfo("已到达移动靶位置")
                while self.current_stage == "MOVING_STAGE":
                    rospy.sleep(0.1)

            # 返回终点
            self._navigate_to_goal(goals[3])
            rospy.loginfo("比赛完成！")

        except KeyboardInterrupt:
            self.move_base.cancel_all_goals()
            rospy.loginfo("比赛已终止")

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('shooting_competition')
    
    # 加载目标点参数
    goal_config = {
        'x': rospy.get_param('~goalListX', '2.0,2.0,3.0,0.0'),
        'y': rospy.get_param('~goalListY', '2.0,4.0,1.0,0.0'), 
        'yaw': rospy.get_param('~goalListYaw', '0,90,180,0')
    }
    
    # 处理中文逗号并转换为坐标列表
    goals = [
        [float(x), float(y), float(yaw)] 
        for x, y, yaw in zip(
            goal_config['x'].replace(u'，', ',').split(','),
            goal_config['y'].replace(u'，', ',').split(','),
            goal_config['yaw'].replace(u'，', ',').split(',')
        )
    ]
    
    # 启动比赛
    try:
        competition = ShootingCompetition()
        competition.run_competition(goals)
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭")

