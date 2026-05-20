#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026 射击挑战赛 — 比赛主控节点
状态机: WAIT_START → VOICE_RECV → NAV_LOOP (遍历 route_points) → FINISH
从 competition_2026_route.yaml 加载路线配置
"""

import rospy
import yaml
import math
import os
import rospkg
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, Int32
from tf_conversions import transformations
import actionlib


class CompetitionControl:
    def __init__(self):
        rospy.init_node('competition_control')

        # 加载路线配置
        route_path = rospy.get_param('~route_config',
                                     self._default_route_path())
        self.route_points, self.global_params = self._load_route(route_path)
        rospy.loginfo("加载路线配置: %d 个点", len(self.route_points))

        # 状态机
        self.state = 'WAIT_START'
        self.current_point_index = 0
        self.state_change_time = rospy.Time.now()

        # 语音识别结果
        self.target_id_rotating = None
        self.target_id_moving = None

        # 感知数据
        self.latest_ar_markers = None
        self.latest_object_point = None

        # 射击状态
        self.shoot_done = False

        # 机器人当前位置
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.tf_listener = tf.TransformListener()

        # ------- 发布者 -------
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.shoot_pub = rospy.Publisher('/shoot', String, queue_size=10)
        self.audio_pub = rospy.Publisher('audio_topic', String, queue_size=10)
        self.state_pub = rospy.Publisher('/competition_state', String, queue_size=10)

        # ------- 订阅者 -------
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._ar_callback)
        rospy.Subscriber('/object_position', Point, self._object_callback)
        rospy.Subscriber('target_id_rotating', Int32, self._rotating_id_cb)
        rospy.Subscriber('target_id_moving', Int32, self._moving_id_cb)

        # ------- move_base 客户端 -------
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 服务...")
        if not self.move_base.wait_for_server(rospy.Duration(30)):
            rospy.logwarn("move_base 服务连接超时")

        rospy.loginfo("competition_control 初始化完成, 状态: %s", self.state)

    # ===================== 配置加载 =====================

    def _default_route_path(self):
        rp = rospkg.RosPack()
        return os.path.join(rp.get_path('robot_slam'),
                            'config', 'competition_2026_route.yaml')

    def _load_route(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data['route_points'], data.get('global', {})

    # ===================== 回调 =====================

    def _ar_callback(self, msg):
        self.latest_ar_markers = msg.markers

    def _object_callback(self, msg):
        self.latest_object_point = msg

    def _rotating_id_cb(self, msg):
        self.target_id_rotating = msg.data
        rospy.loginfo("收到旋转靶 ID: %d", msg.data)

    def _moving_id_cb(self, msg):
        self.target_id_moving = msg.data
        rospy.loginfo("收到移动靶 ID: %d", msg.data)

    # ===================== 导航 =====================

    def _navigate_to(self, x, y, yaw, timeout=60.0):
        """使用 move_base 导航到目标点, 返回是否成功"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("导航至: (%.3f, %.3f, yaw=%.2f)", x, y, yaw)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn("导航超时")
            return False
        state = self.move_base.get_state()
        success = (state == GoalStatus.SUCCEEDED)
        if not success:
            rospy.logwarn("导航失败, status=%d", state)
        return success

    def _cancel_nav(self):
        self.move_base.cancel_all_goals()

    # ===================== 位置判定 =====================

    def _distance_to(self, x, y):
        """计算当前位置到目标点的欧氏距离 (仅用于 relay/end)"""
        return math.hypot(self.robot_x - x, self.robot_y - y)

    def _is_in_task_zone(self, zone):
        """检查机器人 footprint 是否完全在任务点区域内"""
        if zone is None:
            return self._distance_to(self.route_points[self.current_point_index]['x'],
                                     self.route_points[self.current_point_index]['y']) < 0.05
        return (zone['x_min'] <= self.robot_x <= zone['x_max'] and
                zone['y_min'] <= self.robot_y <= zone['y_max'])

    # ===================== 射击 =====================

    def _fire(self):
        """通过 /shoot topic 触发一次射击"""
        self.shoot_pub.publish(String("shoot"))
        rospy.sleep(0.1)
        self.shoot_pub.publish(String("stopshoot"))
        rospy.loginfo("射击!")

    def _shoot_sequence(self, target_type, point):
        """执行瞄准 + 射击流程"""
        timeout = rospy.Time.now() + rospy.Duration(
            self.global_params.get('shoot_timeout', 15.0))
        rospy.loginfo("开始射击流程, 目标类型: %s", target_type)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if target_type == 'circular':
                done = self._aim_circular()
            elif target_type == 'rotating':
                done = self._aim_rotating()
            elif target_type == 'moving':
                done = self._aim_moving()
            else:
                rospy.logerr("未知的目标类型: %s", target_type)
                return False

            if done:
                rospy.sleep(self.global_params.get('shoot_cooldown', 2.0))
                return True
            rate.sleep()

        rospy.logwarn("射击超时")
        return False

    def _aim_circular(self):
        """环形靶瞄准 (模板匹配)"""
        if self.latest_object_point is None:
            return False

        data = self.latest_object_point
        offset_x = data.x - 320  # 图像中心 X=320
        threshold = 10

        if data.z == 34 and abs(offset_x) > threshold:  # 检测到环形靶
            twist = Twist()
            twist.angular.z = -0.02 * offset_x
            self.cmd_vel_pub.publish(twist)
            return False
        elif data.z == 34 and abs(offset_x) <= threshold:
            self._fire()
            return True
        return False

    def _aim_rotating(self):
        """旋转靶瞄准 (AR 标签)"""
        markers = self.latest_ar_markers
        if markers is None or self.target_id_rotating is None:
            return False

        for marker in markers:
            if marker.id == self.target_id_rotating:
                ax = marker.pose.pose.position.x
                ay = marker.pose.pose.position.y

                if abs(ax) >= 0.1:  # 角度阈值
                    twist = Twist()
                    twist.angular.z = -1.0 * ax
                    self.cmd_vel_pub.publish(twist)
                    return False
                elif -0.1 <= ay <= 0.1:  # Y 方向在有效范围内
                    self._fire()
                    return True
        return False

    def _aim_moving(self):
        """移动靶瞄准 (AR 标签, 高响应)"""
        markers = self.latest_ar_markers
        if markers is None or self.target_id_moving is None:
            return False

        for marker in markers:
            if marker.id == self.target_id_moving:
                ax = marker.pose.pose.position.x

                if abs(ax) >= 0.1:
                    twist = Twist()
                    twist.angular.z = -0.95 * ax
                    self.cmd_vel_pub.publish(twist)
                    return False
                else:
                    self._fire()
                    return True
        return False

    # ===================== 终点滑入 =====================

    def _slide_into_end(self, duration=3.0):
        """用速度指令滑入终点 (导航无法到达角落)"""
        rospy.loginfo("滑入终点区域...")
        twist = Twist()
        twist.linear.x = -0.15
        twist.linear.y = -0.15  # 斜向后滑

        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > duration:
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # 停止
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("滑入完成")

    # ===================== 语音触发 =====================

    def _trigger_voice(self):
        """触发语音识别流程"""
        rospy.loginfo("触发语音识别...")
        rate = rospy.Rate(1)
        for _ in range(2):
            self.audio_pub.publish(String("start_recognition"))
            rate.sleep()

    # ===================== 状态机 =====================

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self._update_robot_pose()
            self.state_pub.publish(String(self.state))

            if self.state == 'WAIT_START':
                self._handle_wait_start()
            elif self.state == 'VOICE_RECV':
                self._handle_voice_recv()
            elif self.state == 'NAV_LOOP':
                self._handle_nav_loop()
            elif self.state == 'FINISH':
                self._handle_finish()
                break

            rate.sleep()

    def _update_robot_pose(self):
        """通过 tf 更新机器人当前位置"""
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0), rospy.Duration(0.1))
            trans, _ = self.tf_listener.lookupTransform('map', 'base_link',
                                                         rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass  # tf 不可用时保持上次值

    def _handle_wait_start(self):
        rospy.loginfo_throttle(5, "等待启动... (输入 1 开始比赛)")
        try:
            user_input = raw_input("请输入 1 开始: ")
        except NameError:
            user_input = input("请输入 1 开始: ")
        if user_input == '1':
            self.state = 'VOICE_RECV'
            self.state_change_time = rospy.Time.now()
            rospy.loginfo("比赛开始!")

    def _handle_voice_recv(self):
        self._trigger_voice()
        rospy.sleep(18)  # 等待语音识别 + 目标ID解析
        self.state = 'NAV_LOOP'
        self.current_point_index = 0
        self.state_change_time = rospy.Time.now()
        rospy.loginfo("开始导航循环, 共 %d 个 route point",
                      len(self.route_points))

    def _handle_nav_loop(self):
        if self.current_point_index >= len(self.route_points):
            self.state = 'FINISH'
            return

        point = self.route_points[self.current_point_index]
        ptype = point.get('type', 'relay')
        name = point.get('name', 'unknown')
        x, y, yaw = point['x'], point['y'], point.get('yaw', 0.0)

        if ptype == 'relay':
            # 中继点: 导航 → 判近 → 下一个
            self._navigate_to(x, y, yaw)
            threshold = self.global_params.get('relay_close_threshold', 0.10)
            if self._distance_to(x, y) < threshold:
                rospy.loginfo("到达中继点: %s", name)
                self.current_point_index += 1

        elif ptype == 'task':
            # 任务点: 导航 → footprint 判定 → 射击 → 下一个
            target_type = point.get('target_type', 'circular')
            nav_ok = self._navigate_to(x, y, yaw,
                                       self.global_params.get('task_timeout', 60.0))
            if not nav_ok:
                rospy.logwarn("任务点导航未达到, 尝试射击")
            # 检查是否在任务区域内
            zone = point.get('task_zone', None)
            if self._is_in_task_zone(zone):
                rospy.loginfo("进入任务区域: %s", name)
                shoot_ok = self._shoot_sequence(target_type, point)
                if shoot_ok:
                    rospy.loginfo("任务完成: %s", name)
                    self.current_point_index += 1
                else:
                    rospy.logwarn("射击未完成, 跳过: %s", name)
                    self.current_point_index += 1  # 超时后也跳过

        elif ptype == 'end':
            # 终点: 导航 → 判近 → 滑入 → 完成
            self._navigate_to(x, y, yaw)
            threshold = self.global_params.get('end_close_threshold', 0.15)
            if self._distance_to(x, y) < threshold:
                rospy.loginfo("到达终点附近, 开始滑入")
                self._cancel_nav()
                self._slide_into_end()
                self.state = 'FINISH'

        self.state_change_time = rospy.Time.now()

    def _handle_finish(self):
        rospy.loginfo("========== 比赛完成! ==========")
        self._cancel_nav()
        self.cmd_vel_pub.publish(Twist())


if __name__ == '__main__':
    try:
        ctrl = CompetitionControl()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
