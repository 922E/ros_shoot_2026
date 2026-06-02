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

# Shooter task tolerances (fine)
SHOOT_XY_TOL = 0.06        # fine_adjust_xy success threshold (m)
SHOOT_TASK_DIST_TOL = 0.08 # TASK entry max distance (m)
TASK_NAV_TOL = 0.14        # rough move_base handoff; fine adjust owns final xy
TASK_NAV_TIMEOUT = 10.0    # avoid waiting on the last few centimeters
FINE_ADJUST_MAX_SPEED = 0.14
FINE_ADJUST_GAIN = 0.65

# Relay arrival: near target y-line + target-centered x corridor
RELAY_Y_TOL = 0.08         # abs(robot_y - target_y) <= this → y_ok
RELAY_X_TOL = 0.14         # abs(robot_x - target_x) <= this → x_ok
SAFE_X_MIN = 0.15          # hard field safety lower bound
SAFE_X_MAX = 0.45          # hard field safety upper bound
RELAY_YAW_TOL = 20.0       # preferred relay departure yaw tolerance (deg)
RELAY_YAW_HARD_LIMIT = 45.0  # do not enter next segment above this error
RELAY_ROTATE_TIMEOUT = 5.0 # enough for a large heading correction (s)
ROTATE_RETRY_LIMIT = 1     # one bounded attempt; hard limit decides continue/stop
ROTATE_CMD_SIGN = 1.0      # auto-flipped if yaw error grows during rotation
RELAY_DRIVE_TIMEOUT = 10.0 # cmd_vel corridor traversal timeout (s)
RELAY_MAX_VX = 0.14        # map x correction speed while crossing corridor
RELAY_MAX_VY = 0.30        # map y traversal speed through narrow corridor
RELAY_MAX_WZ = 0.18        # weak yaw hold, avoid in-corridor large turns

# back_y_only: retreat to safe x corridor while keeping y near task line
BACK_X_TOL = 0.16          # helper point x tolerance around configured x
BACK_NAV_TOL = 0.12        # move_base only needs to reach cmd_vel handoff range
BACK_Y_TOL = 0.08          # keep y close to target during back point
BACK_Y_TIMEOUT = 6.0       # bounded goto before short cmd_vel correction
BACK_YAW_TOL = 20.0        # coarse yaw alignment at helper points (deg)
BACK_ROTATE_TIMEOUT = 3.0  # keep helper-point alignment bounded (s)
BACK_PUSH_TOL = 0.06       # cmd_vel helper correction target tolerance (m)
BACK_PUSH_TIMEOUT = 2.0    # short push, cheaper than another move_base retry
BACK_PUSH_SPEED = 0.18     # quick handoff correction after move_base

# End slide
END_ACCEPT_TOL = 0.04      # end zone needs a tighter final center lock
END_PRE_TIMEOUT = 12.0     # don't wait 60s for a tight pre-point
END_PRE_SPEED = 0.18       # cmd_vel fallback speed to pre-point
END_SLIDE_SPEED = 0.18     # final slide speed into 40x40cm end zone
END_SLIDE_TIMEOUT = 12.0
END_RETRY_SPEED = 0.10     # slow final settle if the first slide times out
END_RETRY_TIMEOUT = 3.0

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
        self.circular_stable_frames = 0
        self.rotating_stable_frames = 0
        self.moving_stable_frames = 0
        self.last_shoot_ok = None

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
        except Exception as exc:
            rospy.logwarn("/dev/shoot not available, shoot disabled: %s",
                          str(exc))

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
        target_id = self.target_ids.get('circular', 34)
        aim_center_x = 320.0
        fire_threshold_px = 10.0
        stable_frames_required = 1
        kp_far = 0.02
        kp_near = 0.012
        near_threshold_px = 40.0
        max_wz = 0.65
        min_wz = 0.08
        search_wz = self.global_params.get('circular_search_wz', 0.12)

        if not should_attack_circular:
            self.circular_stable_frames = 0
            return
        point_msg = data

        if data.z != target_id:
            self.circular_stable_frames = 0
            msg = Twist()
            if data.z == 255:
                msg.angular.z = search_wz
            self.pub.publish(msg)
            rospy.logwarn_throttle(
                1.0,
                "Circular target waiting: detected_id=%.0f expected=%d x=%.1f search_wz=%.3f",
                data.z, target_id, data.x, msg.angular.z)
            return

        offset_x = data.x - aim_center_x
        abs_offset_x = abs(offset_x)

        if abs_offset_x > fire_threshold_px:
            self.circular_stable_frames = 0
            kp = kp_near if abs_offset_x <= near_threshold_px else kp_far
            angular_z = -kp * offset_x
            angular_z = max(-max_wz, min(max_wz, angular_z))
            if abs(angular_z) < min_wz:
                angular_z = min_wz if angular_z > 0 else -min_wz
            msg = Twist()
            msg.angular.z = angular_z
            self.pub.publish(msg)
            rospy.loginfo_throttle(
                1.0,
                "Circular target aiming: id=%.0f offset_x=%.1f wz=%.3f",
                data.z, offset_x, angular_z)

        else:
            self.circular_stable_frames += 1
            self.pub.publish(Twist())
            rospy.loginfo("Circular target stable frames: %d/%d",
                          self.circular_stable_frames,
                          stable_frames_required)
            if self.circular_stable_frames < stable_frames_required:
                return
            fire_ok = self._fire()
            self.last_shoot_ok = fire_ok
            if fire_ok:
                rospy.loginfo("Circular target pulse sent")
            else:
                rospy.logerr("Circular target pulse failed")
            self.circular_stable_frames = 0
            should_attack_circular = False

    def rotating_target(self, data):
        """旋转靶瞄准回调 (same as shoot_2025.py)"""
        global target_id_rotating, should_attack_rotating
        x_threshold = self.global_params.get('rotating_x_threshold', 0.1)
        y_min = self.global_params.get('rotating_y_min', -0.1)
        y_max = self.global_params.get('rotating_y_max', 0.1)
        stable_frames_required = int(
            self.global_params.get('rotating_stable_frames_required', 2))
        kp_far = self.global_params.get('rotating_kp_far', 0.55)
        kp_near = self.global_params.get('rotating_kp_near', 0.35)
        near_threshold = 0.18
        max_wz = self.global_params.get('rotating_max_wz', 0.30)
        min_wz = self.global_params.get('rotating_min_wz', 0.04)
        shoot_cooldown = 0.2

        if not should_attack_rotating:
            self.rotating_stable_frames = 0
            return

        found_target = False
        for marker in data.markers:
            if marker.id == target_id_rotating:
                found_target = True
                ax = marker.pose.pose.position.x
                ay = marker.pose.pose.position.y
                abs_ax = abs(ax)
                y_ok = y_min <= ay <= y_max

                if abs_ax >= x_threshold:
                    self.rotating_stable_frames = 0
                    kp = kp_near if abs_ax <= near_threshold else kp_far
                    angular_z = -kp * ax
                    angular_z = max(-max_wz, min(max_wz, angular_z))
                    if abs(angular_z) < min_wz:
                        angular_z = min_wz if angular_z > 0 else -min_wz
                    msg = Twist()
                    msg.angular.z = angular_z
                    self.pub.publish(msg)
                    rospy.loginfo_throttle(
                        1.0,
                        "Rotating target aiming: id=%d x=%.3f y=%.3f wz=%.3f",
                        marker.id, ax, ay, angular_z)

                elif y_ok:
                    self.rotating_stable_frames += 1
                    self.pub.publish(Twist())
                    rospy.loginfo("Rotating target stable frames: %d/%d",
                                  self.rotating_stable_frames,
                                  stable_frames_required)
                    if self.rotating_stable_frames < stable_frames_required:
                        return
                    fire_ok = self._fire()
                    self.last_shoot_ok = fire_ok
                    rospy.sleep(shoot_cooldown)
                    if fire_ok:
                        rospy.loginfo("Rotating target pulse sent")
                    else:
                        rospy.logerr("Rotating target pulse failed")
                    self.rotating_stable_frames = 0
                    should_attack_rotating = False

                else:
                    self.rotating_stable_frames = 0
                    self.pub.publish(Twist())
                    rospy.logwarn_throttle(
                        1.0,
                        "Rotating target y outside window: id=%d x=%.3f y=%.3f expected=[%.3f,%.3f]",
                        marker.id, ax, ay, y_min, y_max)
                return

        if not found_target:
            self.rotating_stable_frames = 0
            self.pub.publish(Twist())
            rospy.logwarn_throttle(
                1.0,
                "Rotating target waiting: expected_id=%s visible_ids=%s",
                str(target_id_rotating),
                str([marker.id for marker in data.markers]))

    def moving_target(self, data):
        """移动靶瞄准回调 (same as shoot_2025.py)"""
        global target_id_moving, should_attack_moving
        x_threshold = 0.1
        stable_frames_required = 2
        kp_far = 0.95
        kp_near = 0.7
        near_threshold = 0.18
        max_wz = 0.6
        min_wz = 0.08
        shoot_cooldown = 0.1

        if not should_attack_moving:
            self.moving_stable_frames = 0
            return

        found_target = False
        for marker in data.markers:
            if marker.id == target_id_moving:
                found_target = True
                ax = marker.pose.pose.position.x
                abs_ax = abs(ax)

                if abs_ax >= x_threshold:
                    self.moving_stable_frames = 0
                    kp = kp_near if abs_ax <= near_threshold else kp_far
                    angular_z = -kp * ax
                    angular_z = max(-max_wz, min(max_wz, angular_z))
                    if abs(angular_z) < min_wz:
                        angular_z = min_wz if angular_z > 0 else -min_wz
                    msg = Twist()
                    msg.angular.z = angular_z
                    self.pub.publish(msg)
                    rospy.loginfo_throttle(
                        1.0,
                        "Moving target aiming: id=%d x=%.3f wz=%.3f",
                        marker.id, ax, angular_z)

                else:
                    self.moving_stable_frames += 1
                    self.pub.publish(Twist())
                    rospy.loginfo("Moving target stable frames: %d/%d",
                                  self.moving_stable_frames,
                                  stable_frames_required)
                    if self.moving_stable_frames < stable_frames_required:
                        return
                    fire_ok = self._fire()
                    self.last_shoot_ok = fire_ok
                    rospy.sleep(shoot_cooldown)
                    if fire_ok:
                        rospy.loginfo("Moving target pulse sent")
                    else:
                        rospy.logerr("Moving target pulse failed")
                    self.moving_stable_frames = 0
                    should_attack_moving = False
                return

        if not found_target:
            self.moving_stable_frames = 0
            self.pub.publish(Twist())
            rospy.logwarn_throttle(
                1.0,
                "Moving target waiting: expected_id=%s visible_ids=%s",
                str(target_id_moving),
                str([marker.id for marker in data.markers]))

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
        cmd_sign = ROTATE_CMD_SIGN
        initial_abs_err = None
        sign_checked = False
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            err = self._get_robot_yaw_error(target_yaw)
            abs_err = abs(err)
            if initial_abs_err is None:
                initial_abs_err = abs_err
            if abs(err) < tol_deg / 180.0 * pi:
                self.pub.publish(Twist())
                rospy.loginfo("[ROTATE] OK robot_yaw=%.0fdeg", self._get_robot_yaw() * 180.0 / pi)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[ROTATE] timeout err=%.0fdeg", err * 180.0 / pi)
                return False
            if (not sign_checked and
                    (rospy.Time.now() - start).to_sec() > 0.7):
                sign_checked = True
                if abs_err > initial_abs_err + 0.15:
                    cmd_sign *= -1.0
                    rospy.logwarn("[ROTATE] yaw error growing, flip cmd sign")
            msg = Twist()
            msg.angular.z = max(-0.5, min(0.5, err * cmd_sign))
            self.pub.publish(msg)
            rate.sleep()
        return False

    def _rotate_to_yaw_with_retry(self, yaw_deg, tol_deg, timeout,
                                  retry_limit):
        """Rotate with bounded retries. Used before entering narrow gaps."""
        for attempt in range(retry_limit):
            ok = self._rotate_to_yaw(yaw_deg, tol_deg=tol_deg,
                                     timeout=timeout)
            if ok:
                return True
            rospy.logwarn("[ROTATE] retry %d/%d failed",
                          attempt + 1, retry_limit)
        return False

    def _fine_adjust_xy(self, x, y, pos_tol=None, target_yaw_deg=None,
                         timeout=10.0):
        """Fix xy position with holonomic translation; do not rotate to aim.

        Navigation owns position only. The shooting module owns final aiming,
        so this controller converts map-frame position error into base_link
        linear.x/y and keeps angular.z at zero.
        """
        if pos_tol is None:
            pos_tol = SHOOT_XY_TOL

        rospy.loginfo("[FINE_ADJUST_XY] target=(%.3f,%.3f) tol=%.3f robot=(%.3f,%.3f)",
                      x, y, pos_tol, self.robot_x, self.robot_y)

        for attempt in range(3):
            if rospy.is_shutdown():
                self.pub.publish(Twist())
                return False
            self._update_robot_pose()
            d = math.hypot(self.robot_x - x, self.robot_y - y)
            if d <= pos_tol:
                self.pub.publish(Twist())
                rospy.loginfo("[FINE_ADJUST_XY] OK dist=%.3f attempt=%d", d, attempt)
                return True

            start = rospy.Time.now()
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self._update_robot_pose()
                dx = x - self.robot_x
                dy = y - self.robot_y
                d = math.hypot(dx, dy)
                if d <= pos_tol:
                    self.pub.publish(Twist())
                    rospy.loginfo("[FINE_ADJUST_XY] OK dist=%.3f attempt=%d", d, attempt)
                    return True
                if (rospy.Time.now() - start).to_sec() > timeout:
                    self.pub.publish(Twist())
                    rospy.logwarn("[FINE_ADJUST_XY] timeout dist=%.3f attempt=%d", d, attempt)
                    break

                robot_yaw = self._get_robot_yaw()
                cos_yaw = math.cos(robot_yaw)
                sin_yaw = math.sin(robot_yaw)
                vx_map = max(-FINE_ADJUST_MAX_SPEED,
                             min(FINE_ADJUST_MAX_SPEED,
                                 dx * FINE_ADJUST_GAIN))
                vy_map = max(-FINE_ADJUST_MAX_SPEED,
                             min(FINE_ADJUST_MAX_SPEED,
                                 dy * FINE_ADJUST_GAIN))
                msg = Twist()
                msg.linear.x = cos_yaw * vx_map + sin_yaw * vy_map
                msg.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map
                msg.angular.z = 0.0
                self.pub.publish(msg)
                rate.sleep()

            if rospy.is_shutdown():
                self.pub.publish(Twist())
                return False

        self.pub.publish(Twist())
        rospy.logerr("[FINE_ADJUST_XY] FAIL after 3 attempts")
        return False

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

    def _stop_motion(self, duration=0.15):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist())
            if (rospy.Time.now() - start).to_sec() >= duration:
                break
            rate.sleep()

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

    def _x_hard_safe(self):
        return SAFE_X_MIN <= self.robot_x <= SAFE_X_MAX

    def _back_arrived(self, target_x, target_y):
        x_safe = self._x_hard_safe()
        x_ok = abs(self.robot_x - target_x) <= BACK_X_TOL
        y_ok = abs(self.robot_y - target_y) <= BACK_Y_TOL
        return x_safe and x_ok, y_ok, x_safe and x_ok and y_ok

    def _relay_arrived(self, target_x, target_y):
        x_safe = self._x_hard_safe()
        x_ok = abs(self.robot_x - target_x) <= RELAY_X_TOL
        y_ok = abs(self.robot_y - target_y) <= RELAY_Y_TOL
        return x_safe and x_ok, y_ok, x_safe and x_ok and y_ok

    def _in_task_zone(self, x, y):
        """Check if robot is within TASK entry distance"""
        return math.hypot(self.robot_x - x, self.robot_y - y) <= SHOOT_TASK_DIST_TOL

    # ===================== Shooting =====================

    def _fire(self):
        """Fire using serial (same as shoot_2025.py)"""
        if self.ser is None:
            rospy.logwarn("Serial not available, cannot fire")
            return False
        fire_command = b'\x55\x01\x12\x00\x00\x00\x01\x69'
        stop_command = b'\x55\x01\x11\x00\x00\x00\x01\x68'
        try:
            fire_written = self.ser.write(fire_command)
            self.ser.flush()
            rospy.sleep(0.09)
            stop_written = self.ser.write(stop_command)
            self.ser.flush()
        except Exception as exc:
            rospy.logerr("Shooter serial write failed: %s", str(exc))
            return False
        if fire_written != len(fire_command) or \
                stop_written != len(stop_command):
            rospy.logerr("Shooter short write: fire=%d/%d stop=%d/%d",
                         fire_written, len(fire_command),
                         stop_written, len(stop_command))
            return False
        rospy.loginfo("Shooter pulse sent: fire=%d stop=%d",
                      fire_written, stop_written)
        return True

    def _wait_for_shoot(self, flag_name, timeout=15.0):
        """Wait until shoot flag becomes False (callback-driven)"""
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            global should_attack_circular, should_attack_rotating
            global should_attack_moving
            if flag_name == 'circular' and not should_attack_circular:
                return self.last_shoot_ok is True
            if flag_name == 'rotating' and not should_attack_rotating:
                return self.last_shoot_ok is True
            if flag_name == 'moving' and not should_attack_moving:
                return self.last_shoot_ok is True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("Shooter aiming timeout: %s", flag_name)
                self._reset_attack_state(flag_name)
                return False
            rate.sleep()

    def _reset_attack_state(self, flag_name):
        """Disable one visual attack mode and stop residual rotation."""
        global should_attack_circular, should_attack_rotating
        global should_attack_moving
        if flag_name == 'circular':
            should_attack_circular = False
            self.circular_stable_frames = 0
        elif flag_name == 'rotating':
            should_attack_rotating = False
            self.rotating_stable_frames = 0
        elif flag_name == 'moving':
            should_attack_moving = False
            self.moving_stable_frames = 0
        self.pub.publish(Twist())

    # ===================== Endpoint slide-in =====================

    def _slide_to_point(self, x, y, speed=0.08, timeout=6.0, tol=None,
                        tag="END"):
        """Slide toward target with low-speed cmd_vel, bypassing costmap."""
        if tol is None:
            tol = END_ACCEPT_TOL
        rospy.loginfo("[%s] Sliding to (%.3f,%.3f) speed=%.3f",
                      tag, x, y, speed)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_robot_pose()
            dx = x - self.robot_x
            dy = y - self.robot_y
            d = math.hypot(dx, dy)
            if d <= tol:
                self.pub.publish(Twist())
                rospy.loginfo("[%s] Slide accept, dist=%.3f tol=%.3f", tag, d, tol)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[%s] Slide timeout, dist=%.3f", tag, d)
                return d <= tol
            yaw = self._get_robot_yaw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            vx_map = max(-speed, min(speed, dx * 0.8))
            vy_map = max(-speed, min(speed, dy * 0.8))
            msg = Twist()
            msg.linear.x = cos_yaw * vx_map + sin_yaw * vy_map
            msg.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map
            self.pub.publish(msg)
            rate.sleep()
        return False

    def _cmd_vel_to_y(self, target_y, speed=-0.10, timeout=5.0):
        """Drive robot in y direction with cmd_vel until robot_y <= target_y.
        Uses map-frame y direction for holonomic robot facing yaw≈0."""
        rospy.loginfo("[CMD_VEL_Y] driving to y<=%.3f from y=%.3f",
                      target_y, self.robot_y)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_robot_pose()
            if self.robot_y <= target_y:
                self.pub.publish(Twist())
                rospy.loginfo("[CMD_VEL_Y] OK y=%.3f", self.robot_y)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[CMD_VEL_Y] timeout y=%.3f", self.robot_y)
                return False
            msg = Twist()
            msg.linear.y = speed
            self.pub.publish(msg)
            rate.sleep()
        return False

    def _drive_relay_corridor(self, target_x, target_y, yaw_deg,
                              timeout=RELAY_DRIVE_TIMEOUT):
        """Cross a short narrow corridor with holonomic cmd_vel.

        The corridor is only about two robot lengths, so using move_base here
        tends to add heading changes. This keeps x in the safe lane, drives
        toward the relay y-line, and applies only weak yaw hold.
        """
        rospy.loginfo("[RELAY_DRIVE] target=(%.3f,%.3f) yaw=%.0fdeg",
                      target_x, target_y, yaw_deg)
        start = rospy.Time.now()
        target_yaw = yaw_deg / 180.0 * pi
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_robot_pose()
            x_safe, y_ok, relay_ok = self._relay_arrived(target_x, target_y)
            if relay_ok:
                self.pub.publish(Twist())
                rospy.loginfo("[RELAY_DRIVE] OK x=%.3f y=%.3f",
                              self.robot_x, self.robot_y)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.pub.publish(Twist())
                rospy.logwarn("[RELAY_DRIVE] timeout x=%.3f(x_safe=%s) y=%.3f(y_ok=%s)",
                              self.robot_x, x_safe, self.robot_y, y_ok)
                return False

            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            yaw = self._get_robot_yaw()
            yaw_err = self._get_robot_yaw_error(target_yaw)

            vx_map = max(-RELAY_MAX_VX, min(RELAY_MAX_VX, dx * 1.0))
            vy_map = max(-RELAY_MAX_VY, min(RELAY_MAX_VY, dy * 1.6))
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            msg = Twist()
            msg.linear.x = cos_yaw * vx_map + sin_yaw * vy_map
            msg.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map
            msg.angular.z = max(-RELAY_MAX_WZ,
                                min(RELAY_MAX_WZ, yaw_err * 0.35))
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

        # Skip 18s voice wait if IDs already known from YAML config
        if target_id_rotating is not None and target_id_moving is not None:
            rospy.loginfo("IDs already known (rotating=%s moving=%s), skip voice wait",
                          target_id_rotating, target_id_moving)
        else:
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

        if ptype == 'back_y_only':
            # Back point must put the robot in the safe x corridor before
            # the next narrow-gap relay is allowed to run.
            rospy.loginfo("[BACK_Y] goto (%.3f,%.3f)", x, y)
            back_ok = False
            for attempt in range(2):
                self.goto(x, y, yaw_deg, timeout=BACK_Y_TIMEOUT,
                          tol=BACK_NAV_TOL)
                self.cancel()
                self._update_robot_pose()
                x_safe, y_ok, back_ok = self._back_arrived(x, y)
                rospy.loginfo("[BACK_Y] check attempt=%d x=%.3f(x_safe=%s) y=%.3f(y_ok=%s)",
                              attempt + 1, self.robot_x, x_safe,
                              self.robot_y, y_ok)
                if back_ok:
                    break
                rospy.logwarn("[BACK_Y] not safe, retry goto")

            if back_ok:
                rospy.loginfo("[BACK_Y] OK x=%.3f y=%.3f target_y=%.3f",
                              self.robot_x, self.robot_y, y)
                d_back = math.hypot(self.robot_x - x, self.robot_y - y)
                if d_back > BACK_PUSH_TOL:
                    rospy.loginfo("[BACK_Y] short cmd push dist=%.3f", d_back)
                    self._slide_to_point(x, y, speed=BACK_PUSH_SPEED,
                                         timeout=BACK_PUSH_TIMEOUT,
                                         tol=BACK_PUSH_TOL,
                                         tag="BACK_PUSH")
                    self._update_robot_pose()
                self._stop_motion(duration=0.1)
                yaw_ok = self._rotate_to_yaw(
                    yaw_deg, tol_deg=BACK_YAW_TOL,
                    timeout=BACK_ROTATE_TIMEOUT)
                if not yaw_ok:
                    yaw_err = abs(self._get_robot_yaw_error(
                        yaw_deg / 180.0 * pi)) * 180.0 / pi
                    rospy.logwarn("[BACK_Y] yaw not fully aligned err=%.0fdeg, continue to relay",
                                  yaw_err)
            else:
                rospy.logerr("[BACK_Y] FAIL x=%.3f y=%.3f target_y=%.3f, stop before narrow gap",
                             self.robot_x, self.robot_y, y)
                self.state = 'FINISH'
                return

            self.current_point_index += 1

        elif ptype == 'relay':
            # Short corridor: use holonomic cmd_vel instead of move_base to
            # avoid extra heading changes inside a 65cm narrow segment.
            self.cancel()
            self._stop_motion(duration=0.1)
            relay_ok = self._drive_relay_corridor(x, y, yaw_deg)
            if not relay_ok:
                rospy.logerr("[RELAY] FAIL x=%.3f y=%.3f target_y=%.3f, stop before next task",
                             self.robot_x, self.robot_y, y)
                self.state = 'FINISH'
                return

            self._update_robot_pose()
            next_idx = self.current_point_index + 1
            if next_idx < len(self.route_points):
                next_point = self.route_points[next_idx]
                heading_deg = yaw_deg
                yaw_err = abs(self._get_robot_yaw_error(
                    heading_deg / 180.0 * pi)) * 180.0 / pi
                rospy.loginfo("[RELAY] before %s yaw_err=%.0fdeg",
                              _safe(next_point.get('name', 'unknown')),
                              yaw_err)
                if yaw_err > RELAY_YAW_HARD_LIMIT:
                    rospy.logwarn("[RELAY] yaw above hard limit, coarse rotate")
                    rotate_ok = self._rotate_to_yaw_with_retry(
                        heading_deg, tol_deg=RELAY_YAW_TOL,
                        timeout=RELAY_ROTATE_TIMEOUT,
                        retry_limit=ROTATE_RETRY_LIMIT)
                    yaw_err = abs(self._get_robot_yaw_error(
                        heading_deg / 180.0 * pi)) * 180.0 / pi
                    if not rotate_ok and yaw_err > RELAY_YAW_HARD_LIMIT:
                        rospy.logerr("[RELAY] yaw FAIL err=%.0fdeg, stop before next segment",
                                     yaw_err)
                        self.state = 'FINISH'
                        return
                self._update_robot_pose()
                x_safe, y_ok, relay_ok = self._relay_arrived(x, y)
                if not relay_ok:
                    rospy.logerr("[RELAY] drift after rotate x=%.3f(x_safe=%s) y=%.3f(y_ok=%s)",
                                 self.robot_x, x_safe, self.robot_y, y_ok)
                    self.state = 'FINISH'
                    return

            self.current_point_index += 1

        elif ptype == 'task':
            target_type = point.get('target_type', 'circular')
            task_yaw = point.get('yaw', yaw_deg)
            ryaw = self._get_robot_yaw()
            rospy.loginfo("[TASK_PRE] %s target=(%.3f,%.3f,%.0fdeg) robot=(%.3f,%.3f,%.0fdeg)",
                          name, x, y, task_yaw,
                          self.robot_x, self.robot_y, ryaw * 180.0 / pi)

            # Step 1: rough nav
            self.goto(x, y, yaw_deg, timeout=TASK_NAV_TIMEOUT,
                      tol=TASK_NAV_TOL)
            self.cancel()
            if rospy.is_shutdown():
                return

            # Step 2: pre-check. Navigation owns position only; the shooting
            # module owns final yaw/aiming through visual feedback.
            self._update_robot_pose()
            d_pre = math.hypot(self.robot_x - x, self.robot_y - y)

            if d_pre <= SHOOT_TASK_DIST_TOL:
                rospy.loginfo("[TASK] position OK after goto (dist=%.3f), skip fine",
                              d_pre)
            else:
                self._fine_adjust_xy(x, y)

            # Step 3: final position check before TASK.
            self._update_robot_pose()
            d_final = math.hypot(self.robot_x - x, self.robot_y - y)
            in_zone = d_final <= SHOOT_TASK_DIST_TOL

            if in_zone:
                self._log_pose("TASK", name, x, y, task_yaw)
                if self.ser is None:
                    rospy.logwarn("[TASK] Dry-run, skip shoot: %s", name)
                    shoot_ok = True
                elif target_type == 'circular':
                    global should_attack_circular
                    self.last_shoot_ok = None
                    should_attack_circular = True
                    shoot_ok = self._wait_for_shoot('circular',
                        self.global_params.get('shoot_timeout', 15.0))
                elif target_type == 'rotating':
                    global should_attack_rotating
                    self.last_shoot_ok = None
                    should_attack_rotating = True
                    shoot_ok = self._wait_for_shoot('rotating',
                        self.global_params.get('shoot_timeout', 15.0))
                elif target_type == 'moving':
                    global should_attack_moving
                    self.last_shoot_ok = None
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
                rospy.logerr("[TASK] FAIL %s: dist=%.3f(tol=%.3f)",
                              name, d_final, SHOOT_TASK_DIST_TOL)
            self.current_point_index += 1

        elif ptype == 'end':
            # Navigate to pre-point (safe distance from wall), then slide in
            pre = point.get('pre_point', None)
            if pre:
                pre_ok = self.goto(pre['x'], pre['y'], pre.get('yaw', yaw_deg),
                                   timeout=END_PRE_TIMEOUT, tol=0.12)
                self.cancel()
                self._update_robot_pose()
                if not pre_ok:
                    rospy.logwarn("[END] pre-point move_base timeout, cmd_vel fallback")
                    self._slide_to_point(pre['x'], pre['y'],
                                         speed=END_PRE_SPEED,
                                         timeout=END_SLIDE_TIMEOUT,
                                         tol=0.12,
                                         tag="END_PRE")
                rospy.loginfo("[END] Pre-point handled, now sliding to endpoint")
            else:
                rospy.loginfo("[END] No pre-point, sliding from current position")
            self.cancel()
            end_ok = self._slide_to_point(x, y, speed=END_SLIDE_SPEED,
                                          timeout=END_SLIDE_TIMEOUT,
                                          tol=END_ACCEPT_TOL,
                                          tag="END")
            if not end_ok:
                rospy.logwarn("[END] final center not locked, slow retry")
                self._slide_to_point(x, y, speed=END_RETRY_SPEED,
                                     timeout=END_RETRY_TIMEOUT,
                                     tol=END_ACCEPT_TOL,
                                     tag="END_RETRY")
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
