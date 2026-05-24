#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import actionlib
import serial
import yaml
import os
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, Int32
from math import pi
import tf

# Target thresholds (same as shoot_2025.py)
Yaw_th = 0.1
Yaw_th1 = 0.1
Min_y = -0.1
Max_y = 0.1

# Global state flags (same as shoot_2025.py)
point_msg = None
target_id_rotating = None
target_id_moving = None
should_attack_circular = False
should_attack_rotating = False
should_attack_moving = False


class CompetitionControl:
    def __init__(self):
        rospy.init_node('competition_control')

        # Load route config
        route_path = rospy.get_param('~route_config',
                                     self._default_route_path())
        self.route_points, self.global_params, self.target_ids = \
            self._load_route(route_path)
        rospy.loginfo("Loaded route: %d points", len(self.route_points))

        self.state = 'WAIT_START'
        self.current_point_index = 0

        # Publishers (match shoot_2025.py)
        self.set_pose_pub = rospy.Publisher('/initialpose',
                                            PoseWithCovarianceStamped,
                                            queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.audio_pub = rospy.Publisher('audio_topic', String, queue_size=10)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        self.state_pub = rospy.Publisher('/competition_state', String,
                                         queue_size=10)

        # Subscribers (match shoot_2025.py callback-driven pattern)
        self.find_sub = rospy.Subscriber('/object_position', Point,
                                         self.circular_target)
        self.ar_sub1 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
                                         self.rotating_target)
        self.ar_sub2 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
                                         self.moving_target)
        self.target_id_rotating_sub = rospy.Subscriber('target_id_rotating',
                                                       Int32,
                                                       self._rotating_id_cb)
        self.target_id_moving_sub = rospy.Subscriber('target_id_moving',
                                                     Int32,
                                                     self._moving_id_cb)

        # move_base client (match shoot_2025.py)
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                       MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))

        # Serial port (init inside class, not at module level)
        self.ser = None
        try:
            self.ser = serial.Serial(port="/dev/shoot", baudrate=9600,
                                     parity="N", bytesize=8, stopbits=1)
            rospy.loginfo("Serial port /dev/shoot opened")
        except Exception:
            rospy.logwarn("/dev/shoot not available, shoot disabled")

        # TF listener
        self.tf_listener = tf.TransformListener()
        self.robot_x = 0.0
        self.robot_y = 0.0

        rospy.loginfo("competition_control init OK, state: %s", self.state)

    # ===================== Config =====================

    def _default_route_path(self):
        pkg_path = os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_path, 'config',
                            'competition_2026_route.yaml')

    def _load_route(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data['route_points'], data.get('global', {}), \
            data.get('target_ids', {})

    # ===================== Callbacks (match shoot_2025.py) =====================

    def _rotating_id_cb(self, msg):
        global target_id_rotating
        target_id_rotating = msg.data

    def _moving_id_cb(self, msg):
        global target_id_moving
        target_id_moving = msg.data

    def circular_target(self, data):
        """环形靶瞄准回调 (same as shoot_2025.py)"""
        global point_msg, should_attack_circular
        if not should_attack_circular:
            return
        point_msg = data
        offset_x = data.x - 320
        if abs(offset_x) > 10 and data.z == 34:
            msg = Twist()
            msg.angular.z = -0.02 * offset_x
            self.pub.publish(msg)
        elif abs(offset_x) <= 10 and data.z == 34:
            self._fire()
            rospy.loginfo("Circular target hit!")
            should_attack_circular = False

    def rotating_target(self, data):
        """旋转靶瞄准回调 (same as shoot_2025.py)"""
        global target_id_rotating, should_attack_rotating
        global Yaw_th, Min_y, Max_y
        if not should_attack_rotating:
            return
        for marker in data.markers:
            if marker.id == target_id_rotating:
                ax = marker.pose.pose.position.x
                ay = marker.pose.pose.position.y
                if abs(ax) >= Yaw_th:
                    msg = Twist()
                    msg.angular.z = -1.0 * ax
                    self.pub.publish(msg)
                elif Min_y <= ay <= Max_y:
                    self._fire()
                    rospy.sleep(2)
                    rospy.loginfo("Rotating target hit!")
                    should_attack_rotating = False

    def moving_target(self, data):
        """移动靶瞄准回调 (same as shoot_2025.py)"""
        global target_id_moving, should_attack_moving, Yaw_th1
        if not should_attack_moving:
            return
        for marker in data.markers:
            if marker.id == target_id_moving:
                ax = marker.pose.pose.position.x
                if abs(ax) >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.95 * ax
                    self.pub.publish(msg)
                else:
                    self._fire()
                    rospy.sleep(0.1)
                    rospy.loginfo("Moving target hit!")
                    should_attack_moving = False

    # ===================== Navigation =====================

    def set_pose(self, x, y, yaw_deg):
        """Set AMCL initial pose (same as shoot_2025.py)"""
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = self._quat_from_euler(0.0, 0.0, yaw_deg / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.set_pose_pub.publish(pose)

    def goto(self, x, y, yaw_deg, timeout=60.0):
        """Navigate using actionlib (same as shoot_2025.py)"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = self._quat_from_euler(0.0, 0.0, yaw_deg / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Nav to: (%.3f, %.3f)", x, y)
        self.move_base.send_goal(goal, self._done_cb, self._active_cb,
                                 self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not result:
            self.move_base.cancel_goal()
            rospy.logwarn("Nav timeout")
            return False
        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Nav succeeded")
            return True
        rospy.logwarn("Nav failed, status=%d", state)
        return False

    def _done_cb(self, status, result):
        pass

    def _active_cb(self):
        pass

    def _feedback_cb(self, feedback):
        pass

    def cancel(self):
        self.move_base.cancel_all_goals()

    def _quat_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    # ===================== Position =====================

    def _update_robot_pose(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0),
                                              rospy.Duration(0.5))
            trans, _ = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
        except Exception:
            pass

    def _close_enough(self, x, y, threshold):
        return math.hypot(self.robot_x - x, self.robot_y - y) < threshold

    def _in_task_zone(self, zone):
        if zone is None:
            return False
        return (zone['x_min'] <= self.robot_x <= zone['x_max'] and
                zone['y_min'] <= self.robot_y <= zone['y_max'])

    # ===================== Shooting =====================

    def _fire(self):
        """Fire using serial (same as shoot_2025.py)"""
        if self.ser is None:
            rospy.logwarn("Serial not available, cannot fire")
            return
        self.ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
        rospy.sleep(0.09)
        self.ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

    def _wait_for_shoot(self, flag_name, timeout=15.0):
        """Wait until shoot flag becomes False (callback-driven)"""
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            global should_attack_circular, should_attack_rotating
            global should_attack_moving
            if flag_name == 'circular' and not should_attack_circular:
                return True
            if flag_name == 'rotating' and not should_attack_rotating:
                return True
            if flag_name == 'moving' and not should_attack_moving:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()

    # ===================== Endpoint slide-in =====================

    def _slide_into_end(self, duration=3.0):
        rospy.loginfo("Sliding into endpoint...")
        msg = Twist()
        msg.linear.x = -0.15
        msg.linear.y = -0.15
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > duration:
                break
            self.pub.publish(msg)
            rate.sleep()
        self.pub.publish(Twist())

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

    def _handle_wait_start(self):
        global target_id_rotating, target_id_moving
        rospy.loginfo("STEP: Set '2D Pose Estimate' in RViz, align laser")
        user_input = raw_input("Press Enter after setting initial pose: ")
        # Apply YAML default IDs if voice is not running
        if target_id_rotating is None:
            target_id_rotating = self.target_ids.get('rotating', None)
        if target_id_moving is None:
            target_id_moving = self.target_ids.get('moving', None)
        rospy.loginfo("IDs - rotating:%s moving:%s",
                      target_id_rotating, target_id_moving)
        self.state = 'VOICE_RECV'
        rospy.loginfo("Competition started!")

    def _handle_voice_recv(self):
        global target_id_rotating, target_id_moving
        self._trigger_voice()
        rospy.sleep(18)
        # Use YAML defaults if voice didn't update IDs
        if target_id_rotating is None:
            target_id_rotating = self.target_ids.get('rotating', None)
        if target_id_moving is None:
            target_id_moving = self.target_ids.get('moving', None)
        rospy.loginfo("IDs - rotating:%s moving:%s",
                      target_id_rotating, target_id_moving)
        self.state = 'NAV_LOOP'
        self.current_point_index = 0
        rospy.loginfo("NAV_LOOP: %d points", len(self.route_points))

    def _handle_nav_loop(self):
        if self.current_point_index >= len(self.route_points):
            self.state = 'FINISH'
            return

        point = self.route_points[self.current_point_index]
        ptype = point.get('type', 'relay')
        name = str(point.get('name', 'unknown'))
        x, y = point['x'], point['y']
        yaw_deg = point.get('yaw', 0.0)  # degrees

        rospy.loginfo("[%d/%d] %s target(%.3f,%.3f) robot(%.3f,%.3f)",
                      self.current_point_index + 1, len(self.route_points),
                      name, x, y, self.robot_x, self.robot_y)

        if ptype == 'relay':
            threshold = self.global_params.get('relay_close_threshold', 0.10)
            if self._close_enough(x, y, threshold):
                rospy.loginfo("Already at relay: %s", name)
            else:
                self.goto(x, y, yaw_deg, timeout=30.0)
                self._update_robot_pose()
                if self._close_enough(x, y, threshold):
                    rospy.loginfo("Reached relay: %s", name)
                else:
                    rospy.logwarn("Skip relay: %s", name)
            self.current_point_index += 1

        elif ptype == 'task':
            target_type = point.get('target_type', 'circular')
            self.goto(x, y, yaw_deg,
                      self.global_params.get('task_timeout', 60.0))
            self._update_robot_pose()
            zone = point.get('task_zone', None)
            if self._in_task_zone(zone):
                rospy.loginfo("In task zone: %s", name)
                if target_type == 'circular':
                    global should_attack_circular
                    should_attack_circular = True
                    shoot_ok = self._wait_for_shoot('circular',
                        self.global_params.get('shoot_timeout', 15.0))
                elif target_type == 'rotating':
                    global should_attack_rotating
                    should_attack_rotating = True
                    shoot_ok = self._wait_for_shoot('rotating',
                        self.global_params.get('shoot_timeout', 15.0))
                elif target_type == 'moving':
                    global should_attack_moving
                    should_attack_moving = True
                    shoot_ok = self._wait_for_shoot('moving',
                        self.global_params.get('shoot_timeout', 15.0))
                else:
                    shoot_ok = False
                if shoot_ok:
                    rospy.loginfo("Task done: %s", name)
                else:
                    rospy.logwarn("Shoot timeout: %s", name)
            else:
                rospy.logwarn("Not in task zone: %s, skip", name)
            self.current_point_index += 1

        elif ptype == 'end':
            self.goto(x, y, yaw_deg)
            threshold = self.global_params.get('end_close_threshold', 0.15)
            if self._close_enough(x, y, threshold):
                rospy.loginfo("Near endpoint, sliding in")
                self.cancel()
            else:
                rospy.logwarn("Not near endpoint, slide anyway")
            self._slide_into_end()
            self.state = 'FINISH'

    def _handle_finish(self):
        rospy.loginfo("========== COMPETITION FINISHED! ==========")
        self.cancel()
        self.pub.publish(Twist())


if __name__ == '__main__':
    try:
        ctrl = CompetitionControl()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
