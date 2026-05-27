#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import actionlib
import serial
import yaml
import os
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String, Int32
from math import pi
import tf


def _safe(s):
    """Safe str for Python 2: encode unicode to utf-8 bytes"""
    if isinstance(s, unicode):
        return s.encode('utf-8')
    return str(s)

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
        self.route_points, self.global_params, self.target_ids, \
            self.start_pose = self._load_route(route_path)
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

        # 2D Nav Goal publisher (same as RViz, proven working)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped, queue_size=5)
        # actionlib only for cancel
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
            data.get('target_ids', {}), data.get('start_pose', {})

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

    def goto(self, x, y, yaw_deg, timeout=60.0, tol=0.15):
        """Navigate using /move_base_simple/goal. tol=0.05 for task, 0.12 for relay"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        q = self._quat_from_euler(0.0, 0.0, yaw_deg / 180.0 * pi)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        rospy.loginfo("Nav to: (%.3f, %.3f) tol=%.3f", x, y, tol)
        self.goal_pub.publish(goal)

        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_robot_pose()
            d = math.hypot(self.robot_x - x, self.robot_y - y)
            if d < tol:
                ryaw = self._get_robot_yaw()
                rospy.loginfo("[ARRIVE] OK dist=%.3f tol=%.3f robot=(%.3f,%.3f,%.0fdeg)",
                              d, tol, self.robot_x, self.robot_y, ryaw * 180.0 / pi)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("[ARRIVE] TIMEOUT dist=%.3f tol=%.3f robot=(%.3f,%.3f)",
                              d, tol, self.robot_x, self.robot_y)
                self.move_base.cancel_all_goals()
                return False
            rate.sleep()

    def _rotate_to_yaw(self, yaw_deg, tol_deg=10.0, timeout=5.0):
        """Rotate in place to target yaw. Blocks until done or timeout."""
        target_yaw = yaw_deg / 180.0 * pi
        rospy.loginfo("[ROTATE] to %.0fdeg from %.0fdeg",
                      yaw_deg, self._get_robot_yaw() * 180.0 / pi)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            err = self._get_robot_yaw_error(target_yaw)
            if abs(err) < tol_deg / 180.0 * pi:
                self.pub.publish(Twist())
                rospy.loginfo("[ROTATE] OK robot_yaw=%.0fdeg", self._get_robot_yaw() * 180.0 / pi)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[ROTATE] timeout err=%.0fdeg", err * 180.0 / pi)
                return False
            msg = Twist()
            msg.angular.z = max(-0.5, min(0.5, err * 1.0))
            self.pub.publish(msg)
            rate.sleep()
        return False

    def _fine_adjust_to_pose(self, x, y, target_yaw_deg=None,
                              pos_tol=0.05, yaw_tol=10.0, timeout=10.0):
        """Fix position + yaw. Rotate first if yaw error is large."""
        target_yaw = target_yaw_deg / 180.0 * pi if target_yaw_deg is not None else None
        ryaw = self._get_robot_yaw()
        rospy.loginfo("[FINE_ADJUST] target=(%.3f,%.3f,%.0fdeg) tol_xy=%.3f tol_yaw=%.0fdeg robot_yaw=%.0fdeg",
                      x, y, target_yaw_deg if target_yaw_deg is not None else -999,
                      pos_tol, yaw_tol, ryaw * 180.0 / pi)

        # If yaw error > 30deg, rotate toward target yaw first
        if target_yaw is not None:
            err = self._get_robot_yaw_error(target_yaw)
            if abs(err) > math.radians(30):
                rospy.logwarn("[FINE_ADJUST] yaw off by %.0fdeg, rotate first",
                              err * 180.0 / pi)
                self._rotate_to_yaw(target_yaw_deg, tol_deg=15.0)

        start = rospy.Time.now()
        rate = rospy.Rate(10)

        # Step 1: fix position. Lock yaw — abort xy if yaw drifts > 15deg
        while not rospy.is_shutdown():
            self._update_robot_pose()
            dx = x - self.robot_x
            dy = y - self.robot_y
            d = math.hypot(dx, dy)
            if d < pos_tol:
                self.pub.publish(Twist())
                rospy.loginfo("[FINE_ADJUST] xy OK dist=%.3f", d)
                break
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[FINE_ADJUST] xy timeout dist=%.3f", d)
                break
            # Abort xy if yaw has drifted from target
            if target_yaw is not None:
                yaw_drift = abs(self._get_robot_yaw_error(target_yaw)) * 180.0 / pi
                if yaw_drift > 15.0:
                    self.pub.publish(Twist())
                    rospy.logwarn("[FINE_ADJUST] yaw drifted %.0fdeg, abort xy", yaw_drift)
                    break
            target_dir = math.atan2(dy, dx)
            yaw_err = self._get_robot_yaw_error(target_dir)
            msg = Twist()
            if abs(yaw_err) > 0.3:
                msg.angular.z = max(-0.5, min(0.5, yaw_err * 1.0))
            else:
                msg.linear.x = max(-0.08, min(0.08, d * 0.3))
                msg.angular.z = max(-0.3, min(0.3, yaw_err * 0.5))
            self.pub.publish(msg)
            rate.sleep()

        # Step 2: fix yaw to target
        if target_yaw is not None:
            if not self._rotate_to_yaw(target_yaw_deg, tol_deg=yaw_tol):
                pass  # warn already logged

        return d < pos_tol if 'd' in dir() else False

    def _get_robot_yaw(self):
        """Get current robot yaw from TF. Returns radians, or 0 on failure."""
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0), rospy.Duration(0.3))
            _, rot = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time(0))
            import tf.transformations as tft
            _, _, yaw = tft.euler_from_quaternion(rot)
            return yaw
        except Exception:
            return 0.0

    def _get_robot_yaw_error(self, target_yaw):
        """Get yaw error from current TF orientation to target_yaw"""
        robot_yaw = self._get_robot_yaw()
        err = target_yaw - robot_yaw
        while err > pi:
            err -= 2 * pi
        while err < -pi:
            err += 2 * pi
        return err

    def _log_pose(self, tag, name, tx, ty, tyaw_deg):
        """Log target vs robot pose with yaw"""
        ryaw = self._get_robot_yaw()
        d = math.hypot(self.robot_x - tx, self.robot_y - ty)
        yaw_err_deg = (tyaw_deg / 180.0 * pi - ryaw) * 180.0 / pi if tyaw_deg is not None else 0
        rospy.loginfo("[%s] %s target=(%.3f,%.3f,%.0fdeg) robot=(%.3f,%.3f,%.0fdeg) dist=%.3f yaw_err=%.0fdeg",
                      tag, name, tx, ty, tyaw_deg,
                      self.robot_x, self.robot_y, ryaw * 180.0 / pi,
                      d, yaw_err_deg)

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

    def _in_task_zone(self, x, y):
        """Check if robot is within 0.10m of task point center (dynamic zone)"""
        return math.hypot(self.robot_x - x, self.robot_y - y) < 0.10

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

    def _slide_to_point(self, x, y, speed=0.08, timeout=6.0):
        """Slide toward target with low-speed cmd_vel, bypassing costmap"""
        rospy.loginfo("[END] Sliding to (%.3f,%.3f) speed=%.3f", x, y, speed)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_robot_pose()
            dx = x - self.robot_x
            dy = y - self.robot_y
            d = math.hypot(dx, dy)
            if d < 0.05:
                self.pub.publish(Twist())
                rospy.loginfo("[END] Slide complete, dist=%.3f", d)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[END] Slide timeout, dist=%.3f", d)
                return False
            msg = Twist()
            msg.linear.x = max(-speed, min(speed, dx * 0.5))
            msg.linear.y = max(-speed, min(speed, dy * 0.5))
            self.pub.publish(msg)
            rate.sleep()
        return False

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
        # Auto-set initial pose from config
        sp = self.start_pose
        if sp:
            self.set_pose(sp['x'], sp['y'], sp.get('yaw', 0.0))
            rospy.loginfo("Initial pose set: (%.3f, %.3f)",
                          sp['x'], sp['y'])
        # Apply YAML default IDs if voice is not running
        if target_id_rotating is None:
            target_id_rotating = self.target_ids.get('rotating', None)
        if target_id_moving is None:
            target_id_moving = self.target_ids.get('moving', None)
        rospy.loginfo("IDs - rotating:%s moving:%s",
                      target_id_rotating, target_id_moving)
        rospy.loginfo("Place robot at start mark, press Enter")
        raw_input("Press Enter to start competition: ")
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
        name = _safe(point.get('name', 'unknown'))
        x, y = point['x'], point['y']
        yaw_deg = point.get('yaw', 0.0)  # degrees

        rospy.loginfo("[%d/%d] %s target(%.3f,%.3f) robot(%.3f,%.3f)",
                      self.current_point_index + 1, len(self.route_points),
                      name, x, y, self.robot_x, self.robot_y)

        if ptype == 'relay':
            # Fast pass-through with optional yaw correction
            threshold = self.global_params.get('relay_close_threshold', 0.25)
            if not self._close_enough(x, y, threshold):
                self.goto(x, y, yaw_deg, timeout=20.0, tol=threshold)
            # If relay has yaw, rotate in place before next point
            relay_yaw = point.get('yaw')
            if relay_yaw is not None and abs(self._get_robot_yaw_error(
                    relay_yaw / 180.0 * pi)) > 0.26:
                self._rotate_to_yaw(relay_yaw)
            self.current_point_index += 1

        elif ptype == 'task':
            target_type = point.get('target_type', 'circular')
            task_yaw = point.get('yaw', yaw_deg)
            ryaw = self._get_robot_yaw()
            rospy.loginfo("[TASK_PRE] %s target=(%.3f,%.3f,%.0fdeg) robot=(%.3f,%.3f,%.0fdeg)",
                          name, x, y, task_yaw,
                          self.robot_x, self.robot_y, ryaw * 180.0 / pi)
            # Step 1: rough nav with loose tol, short timeout
            rough_tol = 0.12
            self.goto(x, y, yaw_deg, timeout=15.0, tol=rough_tol)
            self.cancel()
            # Fast accept: if already close enough, skip fine_adjust
            self._update_robot_pose()
            d = math.hypot(self.robot_x - x, self.robot_y - y)
            yaw_err = abs(self._get_robot_yaw_error(
                task_yaw / 180.0 * pi)) * 180.0 / pi
            if d < 0.12 and yaw_err < 15.0:
                rospy.loginfo("[FAST_ACCEPT] dist=%.3f yaw_err=%.0fdeg, skip fine",
                              d, yaw_err)
            else:
                # Step 2: fine adjust position + yaw
                self._fine_adjust_to_pose(x, y, target_yaw_deg=task_yaw)
            if self._in_task_zone(x, y):
                self._log_pose("TASK", name, x, y, task_yaw)
                if self.ser is None:
                    # Dry-run: skip shoot wait
                    rospy.logwarn("[TASK] Dry-run, skip shoot: %s", name)
                    shoot_ok = True
                elif target_type == 'circular':
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
                    rospy.loginfo("[TASK] Done: %s", name)
                else:
                    rospy.logwarn("[TASK] Shoot failed: %s", name)
            else:
                rospy.logwarn("[TASK] Not in zone: %s robot=(%.3f,%.3f)",
                              name, self.robot_x, self.robot_y)
            self.current_point_index += 1

        elif ptype == 'end':
            # Navigate to pre-point (safe distance from wall), then slide in
            pre = point.get('pre_point', None)
            if pre:
                self.goto(pre['x'], pre['y'], pre.get('yaw', yaw_deg),
                          timeout=60.0, tol=0.12)
                self._update_robot_pose()
                rospy.loginfo("[END] Pre-point reached, now sliding to endpoint")
            else:
                rospy.loginfo("[END] No pre-point, sliding from current position")
            self.cancel()
            self._slide_to_point(x, y)
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
