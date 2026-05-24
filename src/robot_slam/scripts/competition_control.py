#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
2026 Shooting Competition - Main Control Node
State: WAIT_START -> VOICE_RECV -> NAV_LOOP -> FINISH
Loads route config from competition_2026_route.yaml
"""

import rospy
import yaml
import math
import os
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, Int32
import actionlib


def _safe(s):
    """Encode unicode to utf-8 bytes for Python 2 logging safety"""
    if isinstance(s, unicode):
        return s.encode('utf-8')
    return str(s)


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class CompetitionControl:
    def __init__(self):
        rospy.init_node('competition_control')

        route_path = rospy.get_param('~route_config',
                                     self._default_route_path())
        self.route_points, self.global_params, self.target_ids = self._load_route(route_path)
        rospy.loginfo("Loaded route: %d points", len(self.route_points))

        self.state = 'WAIT_START'
        self.current_point_index = 0
        self.state_change_time = rospy.Time.now()

        # 先用默认值，语音模块会覆盖
        self.target_id_rotating = None
        self.target_id_moving = None
        self.latest_ar_markers = None
        self.latest_object_point = None
        self.shoot_done = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.shoot_pub = rospy.Publisher('/shoot', String, queue_size=10)
        self.audio_pub = rospy.Publisher('audio_topic', String, queue_size=10)
        self.state_pub = rospy.Publisher('/competition_state', String, queue_size=10)
        self.init_pose_pub = rospy.Publisher('/initialpose',
                                             PoseWithCovarianceStamped, queue_size=5)

        # Subscribers
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._ar_callback)
        rospy.Subscriber('/object_position', Point, self._object_callback)
        rospy.Subscriber('target_id_rotating', Int32, self._rotating_id_cb)
        rospy.Subscriber('target_id_moving', Int32, self._moving_id_cb)

        # move_base client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        if not self.move_base.wait_for_server(rospy.Duration(30)):
            rospy.logwarn("move_base connection timeout")

        rospy.loginfo("competition_control init OK, state: %s", self.state)

    # ===================== Config =====================

    def _default_route_path(self):
        pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_path, 'config', 'competition_2026_route.yaml')

    def _load_route(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data['route_points'], data.get('global', {}), data.get('target_ids', {})

    # ===================== Callbacks =====================

    def _ar_callback(self, msg):
        self.latest_ar_markers = msg.markers

    def _object_callback(self, msg):
        self.latest_object_point = msg

    def _rotating_id_cb(self, msg):
        self.target_id_rotating = msg.data
        rospy.loginfo("Rotating target ID: %d", msg.data)

    def _moving_id_cb(self, msg):
        self.target_id_moving = msg.data
        rospy.loginfo("Moving target ID: %d", msg.data)

    # ===================== Navigation =====================

    def _navigate_to(self, x, y, yaw, timeout=60.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Nav to: (%.3f, %.3f, yaw=%.2f)", x, y, yaw)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn("Nav timeout")
            return False
        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("Nav failed, status=%d", state)
            return False
        return True

    def _cancel_nav(self):
        self.move_base.cancel_all_goals()

    # ===================== Position =====================

    def _distance_to(self, x, y):
        return math.hypot(self.robot_x - x, self.robot_y - y)

    def _is_in_task_zone(self, zone):
        if zone is None:
            return self._distance_to(self.route_points[self.current_point_index]['x'],
                                     self.route_points[self.current_point_index]['y']) < 0.05
        return (zone['x_min'] <= self.robot_x <= zone['x_max'] and
                zone['y_min'] <= self.robot_y <= zone['y_max'])

    # ===================== Shooting =====================

    def _fire(self):
        self.shoot_pub.publish(String("shoot"))
        rospy.sleep(0.1)
        self.shoot_pub.publish(String("stopshoot"))
        rospy.loginfo("FIRED!")

    def _shoot_sequence(self, target_type, point):
        timeout = rospy.Time.now() + rospy.Duration(
            self.global_params.get('shoot_timeout', 15.0))
        rospy.loginfo("Shooting: %s", target_type)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if target_type == 'circular':
                done = self._aim_circular()
            elif target_type == 'rotating':
                done = self._aim_rotating()
            elif target_type == 'moving':
                done = self._aim_moving()
            else:
                rospy.logerr("Unknown target type: %s", target_type)
                return False

            if done:
                rospy.sleep(self.global_params.get('shoot_cooldown', 2.0))
                return True
            rate.sleep()

        rospy.logwarn("Shoot timeout")
        return False

    def _aim_circular(self):
        if self.latest_object_point is None:
            return False
        data = self.latest_object_point
        offset_x = data.x - 320
        threshold = 10
        if data.z == 34 and abs(offset_x) > threshold:
            twist = Twist()
            twist.angular.z = -0.02 * offset_x
            self.cmd_vel_pub.publish(twist)
            return False
        elif data.z == 34 and abs(offset_x) <= threshold:
            self._fire()
            return True
        return False

    def _aim_rotating(self):
        markers = self.latest_ar_markers
        if markers is None or self.target_id_rotating is None:
            return False
        for marker in markers:
            if marker.id == self.target_id_rotating:
                ax = marker.pose.pose.position.x
                ay = marker.pose.pose.position.y
                if abs(ax) >= 0.1:
                    twist = Twist()
                    twist.angular.z = -1.0 * ax
                    self.cmd_vel_pub.publish(twist)
                    return False
                elif -0.1 <= ay <= 0.1:
                    self._fire()
                    return True
        return False

    def _aim_moving(self):
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

    # ===================== Endpoint slide-in =====================

    def _slide_into_end(self, duration=3.0):
        rospy.loginfo("Sliding into endpoint...")
        twist = Twist()
        twist.linear.x = -0.15
        twist.linear.y = -0.15
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > duration:
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Slide complete")

    # ===================== Voice trigger =====================

    def _trigger_voice(self):
        rospy.loginfo("Triggering voice recognition...")
        rate = rospy.Rate(1)
        for _ in range(2):
            self.audio_pub.publish(String("start_recognition"))
            rate.sleep()

    # ===================== State Machine =====================

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
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0), rospy.Duration(0.1))
            trans, _ = self.tf_listener.lookupTransform('map', 'base_link',
                                                         rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass
        except Exception:
            pass

    def _handle_wait_start(self):
        rospy.loginfo_throttle(5, "Waiting... (press 1 to start)")
        user_input = raw_input("Input 1 to start: ")
        if user_input == '1':
            self.state = 'VOICE_RECV'
            self.state_change_time = rospy.Time.now()
            rospy.loginfo("Competition started!")

    def _set_initial_pose(self):
        """将第一个 route point 设为 AMCL 初始位姿"""
        if len(self.route_points) == 0:
            return
        p = self.route_points[0]
        x, y, yaw = p['x'], p['y'], p.get('yaw', 0.0)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        # 协方差默认值，表示中等置信度
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        for _ in range(3):
            self.init_pose_pub.publish(msg)
            rospy.sleep(0.2)
        rospy.loginfo("Initial pose set: (%.3f, %.3f, yaw=%.2f)", x, y, yaw)

    def _handle_voice_recv(self):
        self._trigger_voice()
        rospy.sleep(18)
        # 如果语音没启动，使用 YAML 中的默认 ID
        if self.target_id_rotating is None:
            self.target_id_rotating = self.target_ids.get('rotating', None)
        if self.target_id_moving is None:
            self.target_id_moving = self.target_ids.get('moving', None)
        rospy.loginfo("Target IDs - rotating: %s, moving: %s",
                      self.target_id_rotating, self.target_id_moving)
        self._set_initial_pose()
        rospy.sleep(1)
        self.state = 'NAV_LOOP'
        self.current_point_index = 0
        self.state_change_time = rospy.Time.now()
        rospy.loginfo("NAV_LOOP: %d route points", len(self.route_points))

    def _handle_nav_loop(self):
        if self.current_point_index >= len(self.route_points):
            self.state = 'FINISH'
            return

        point = self.route_points[self.current_point_index]
        ptype = point.get('type', 'relay')
        name = _safe(point.get('name', 'unknown'))
        x, y, yaw = point['x'], point['y'], point.get('yaw', 0.0)

        if ptype == 'relay':
            threshold = self.global_params.get('relay_close_threshold', 0.10)
            if self._distance_to(x, y) < threshold:
                rospy.loginfo("Already at %s", name)
            else:
                self._navigate_to(x, y, yaw, timeout=30.0)
                if self._distance_to(x, y) < threshold:
                    rospy.loginfo("Reached relay: %s", name)
                else:
                    rospy.logwarn("Skip relay: %s", name)
            self.current_point_index += 1

        elif ptype == 'task':
            target_type = point.get('target_type', 'circular')
            self._navigate_to(x, y, yaw,
                              self.global_params.get('task_timeout', 60.0))
            zone = point.get('task_zone', None)
            if self._is_in_task_zone(zone):
                rospy.loginfo("In task zone: %s", name)
                shoot_ok = self._shoot_sequence(target_type, point)
                if shoot_ok:
                    rospy.loginfo("Task done: %s", name)
                else:
                    rospy.logwarn("Shoot failed: %s", name)
            else:
                rospy.logwarn("Not in task zone: %s, skip", name)
            self.current_point_index += 1

        elif ptype == 'end':
            self._navigate_to(x, y, yaw)
            threshold = self.global_params.get('end_close_threshold', 0.15)
            if self._distance_to(x, y) < threshold:
                rospy.loginfo("Near endpoint, sliding in")
                self._cancel_nav()
            else:
                rospy.logwarn("Not near endpoint, slide anyway")
            self._slide_into_end()
            self.state = 'FINISH'

        self.state_change_time = rospy.Time.now()

    def _handle_finish(self):
        rospy.loginfo("========== COMPETITION FINISHED! ==========")
        self._cancel_nav()
        self.cmd_vel_pub.publish(Twist())


if __name__ == '__main__':
    try:
        ctrl = CompetitionControl()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
