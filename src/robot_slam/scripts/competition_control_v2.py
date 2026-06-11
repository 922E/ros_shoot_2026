#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import os
import re

import actionlib
import rospy
import serial
import tf
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import Int32, String

try:
    from TTS_audio.srv import StringService
except Exception:
    StringService = None


def _safe(s):
    if isinstance(s, unicode):
        return s.encode('utf-8')
    return str(s)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _angle_norm(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# v2 route-control parameters: faster than the old script, but limits long
# lateral movement so the car does not drift sideways while finding targets.
TASK_FAST_TOL = 0.09
TASK_FINE_TOL = 0.055
TASK_FAST_TIMEOUT = 8.0
TASK_FINE_TIMEOUT = 3.0
TASK_MAX_VX = 0.36
TASK_MAX_VY = 0.10
TASK_FINE_VX = 0.18
TASK_FINE_VY = 0.06

BACK_TOL = 0.10
BACK_TIMEOUT = 6.0
BACK_MAX_VX = 0.34
BACK_MAX_VY = 0.08

CORRIDOR_TOL = 0.10
CORRIDOR_TIMEOUT = 8.0
CORRIDOR_YAW_DEG = -90.0
CORRIDOR_MAX_VX = 0.42
CORRIDOR_MAX_VY = 0.08

END_TOL = 0.045
END_PRE_TOL = 0.11
END_PRE_TIMEOUT = 7.0
END_TIMEOUT = 6.0
END_MAX_VX = 0.24
END_MAX_VY = 0.08

DRIVE_GAIN_X = 1.2
DRIVE_GAIN_Y = 1.0
MIN_VX = 0.06
MIN_VY = 0.035
YAW_TOL_DEG = 8.0
YAW_HOLD_GAIN = 1.2
ROTATE_TIMEOUT = 4.0
ROTATE_MAX_WZ = 0.75
ROTATE_MIN_WZ = 0.12

SHOOT_TIMEOUT = 12.0


class CompetitionControlV2(object):
    def __init__(self):
        rospy.init_node('competition_control_v2')

        route_path = rospy.get_param('~route_config',
                                     self._default_route_path())
        self.route_points, self.global_params, self.config_target_ids, \
            self.start_pose = self._load_route(route_path)

        self.state = 'INIT'
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.target_id_rotating = None
        self.target_id_moving = None
        self.voice_text = u''
        self.voice_wakeup_ok = False

        self.voice_required = rospy.get_param('~voice_required', True)
        self.allow_default_target_ids = rospy.get_param(
            '~allow_default_target_ids', False)
        self.require_wakeup_word = rospy.get_param('~require_wakeup_word',
                                                   True)
        self.voice_wait_timeout = float(rospy.get_param(
            '~voice_wait_timeout', 18.0))
        self.voice_retry_limit = int(rospy.get_param('~voice_retry_limit', 2))
        self.tts_enabled = rospy.get_param('~tts_enabled', True)
        self.tts_client = None

        self.should_attack_circular = False
        self.should_attack_rotating = False
        self.should_attack_moving = False
        self.circular_stable_frames = 0
        self.rotating_stable_frames = 0
        self.moving_stable_frames = 0
        self.last_shoot_ok = False

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped, queue_size=5)
        self.pose_pub = rospy.Publisher('/initialpose',
                                        PoseWithCovarianceStamped,
                                        queue_size=5)
        self.audio_pub = rospy.Publisher('audio_topic', String, queue_size=5)
        self.voice_pub = rospy.Publisher('/voiceWords', String, queue_size=5)
        self.state_pub = rospy.Publisher('/competition_state', String,
                                         queue_size=5)

        rospy.Subscriber('/object_position', Point, self._circular_cb)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._ar_cb)
        rospy.Subscriber('chinese_topic', String, self._voice_text_cb)
        rospy.Subscriber('target_id_rotating', Int32, self._rotating_id_cb)
        rospy.Subscriber('target_id_moving', Int32, self._moving_id_cb)

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient('move_base',
                                                      MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(20))

        self.ser = None
        try:
            self.ser = serial.Serial(port='/dev/shoot', baudrate=9600,
                                     parity='N', bytesize=8, stopbits=1)
            rospy.loginfo('[V2] Serial /dev/shoot opened')
        except Exception:
            rospy.logwarn('[V2] /dev/shoot not available, dry-run shooting')

        rospy.loginfo('[V2] loaded %d route points', len(self.route_points))

    def _say(self, text):
        if not self.tts_enabled or StringService is None:
            return False
        try:
            if self.tts_client is None:
                rospy.wait_for_service('tts_service', timeout=1.0)
                self.tts_client = rospy.ServiceProxy('tts_service',
                                                     StringService)
            response = self.tts_client(_safe(text))
            rospy.loginfo('[V2][TTS] %s', _safe(response.data))
            return True
        except Exception as exc:
            rospy.logwarn("[V2][TTS] unavailable, skip '%s': %s",
                          _safe(text), str(exc))
            return False

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

    # ===================== Voice =====================

    def _as_unicode(self, text):
        if isinstance(text, unicode):
            return text
        return str(text).decode('utf-8', 'ignore')

    def _compact_text(self, text):
        return re.sub(u'\\s+', u'', self._as_unicode(text))

    def _reset_voice(self):
        self.voice_text = u''
        self.voice_wakeup_ok = False
        self.target_id_rotating = None
        self.target_id_moving = None

    def _has_wakeup_word(self, text):
        if not self.require_wakeup_word:
            return True
        text = self._compact_text(text)
        words = [u'开始比赛', u'开始', u'启动', u'出发', u'小车', u'机器人']
        return any(word in text for word in words)

    def _mapped_id_near_word(self, text, words, mapping):
        text = self._compact_text(text)
        for word in words:
            pos = text.find(word)
            while pos >= 0:
                around = text[max(0, pos - 6):pos + len(word) + 8]
                for token, value in mapping:
                    if token in around:
                        return value
                pos = text.find(word, pos + 1)
        return None

    def _parse_target_pair(self, text):
        text = self._compact_text(text)
        for noise in [u'2026', u'二零二六', u'二〇二六']:
            text = text.replace(noise, u'')
        for label in [u'第二靶', u'第二个靶', u'第三靶', u'第三个靶']:
            text = text.replace(label, u'')

        number_map = {
            u'一': 1, u'1': 1, u'二': 2, u'2': 2,
            u'三': 3, u'3': 3, u'四': 4, u'4': 4,
            u'五': 5, u'5': 5, u'六': 6, u'6': 6,
            u'七': 7, u'7': 7, u'八': 8, u'8': 8,
        }
        rotating_id = None
        moving_id = None
        for ch in text:
            value = number_map.get(ch, None)
            if value is None:
                continue
            if rotating_id is None and 1 <= value <= 5:
                rotating_id = value
            elif moving_id is None and 6 <= value <= 8:
                moving_id = value
            if rotating_id is not None and moving_id is not None:
                break
        return rotating_id, moving_id

    def _voice_ready(self):
        if self.require_wakeup_word and not self.voice_wakeup_ok:
            return False
        return self.target_id_rotating is not None and \
            self.target_id_moving is not None

    def _voice_text_cb(self, msg):
        text = self._as_unicode(msg.data)
        self.voice_text = text
        rospy.loginfo('[V2][VOICE] text: %s', _safe(text))

        if self._has_wakeup_word(text):
            self.voice_wakeup_ok = True
            rospy.loginfo('[V2][VOICE] wakeup OK')

        rotating_id = self._mapped_id_near_word(
            text,
            [u'旋转靶', u'旋转', u'第二靶', u'第二个靶'],
            [(u'一', 1), (u'1', 1), (u'二', 2), (u'2', 2),
             (u'三', 3), (u'3', 3), (u'四', 4), (u'4', 4),
             (u'五', 5), (u'5', 5)])
        moving_id = self._mapped_id_near_word(
            text,
            [u'移动靶', u'移动', u'第三靶', u'第三个靶'],
            [(u'六', 6), (u'6', 6), (u'七', 7), (u'7', 7),
             (u'八', 8), (u'8', 8)])

        if rotating_id is None or moving_id is None:
            pair = self._parse_target_pair(text)
            if rotating_id is None:
                rotating_id = pair[0]
            if moving_id is None:
                moving_id = pair[1]

        if rotating_id is not None:
            self.target_id_rotating = rotating_id
        if moving_id is not None:
            self.target_id_moving = moving_id

        if self._voice_ready():
            msg = u'语音确认，旋转靶{}号，移动靶{}号'.format(
                self.target_id_rotating, self.target_id_moving)
            self.voice_pub.publish(String(_safe(msg)))
            rospy.loginfo('[V2][VOICE] IDs rotating=%s moving=%s',
                          self.target_id_rotating, self.target_id_moving)

    def _rotating_id_cb(self, msg):
        self.target_id_rotating = msg.data

    def _moving_id_cb(self, msg):
        self.target_id_moving = msg.data

    def _trigger_voice(self):
        rospy.loginfo('[V2][VOICE] trigger ASR')
        rate = rospy.Rate(1)
        for _ in range(2):
            self.audio_pub.publish(String('start_recognition'))
            rate.sleep()

    def _wait_voice(self):
        if not self.voice_required:
            if self.allow_default_target_ids:
                self.target_id_rotating = self.config_target_ids.get(
                    'rotating', None)
                self.target_id_moving = self.config_target_ids.get('moving',
                                                                   None)
                return self._voice_ready() or (
                    self.target_id_rotating is not None and
                    self.target_id_moving is not None)
            return False

        for attempt in range(self.voice_retry_limit):
            self._reset_voice()
            rospy.loginfo('[V2][VOICE] attempt %d/%d',
                          attempt + 1, self.voice_retry_limit)
            self._trigger_voice()
            start = rospy.Time.now()
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                if self._voice_ready():
                    return True
                if (rospy.Time.now() - start).to_sec() > \
                        self.voice_wait_timeout:
                    break
                rate.sleep()
            rospy.logwarn('[V2][VOICE] timeout')

        if self.allow_default_target_ids:
            self.target_id_rotating = self.config_target_ids.get('rotating',
                                                                 None)
            self.target_id_moving = self.config_target_ids.get('moving', None)
            rospy.logwarn('[V2][VOICE] fallback to YAML IDs')
            return self.target_id_rotating is not None and \
                self.target_id_moving is not None
        return False

    # ===================== Pose / Motion =====================

    def _set_state(self, state):
        self.state = state
        self.state_pub.publish(String(state))
        rospy.loginfo('[V2][STATE] %s', state)

    def _set_initial_pose(self):
        sp = self.start_pose
        if not sp:
            return
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = sp['x']
        pose.pose.pose.position.y = sp['y']
        q = self._quat_from_yaw(sp.get('yaw', 0.0) / 180.0 * math.pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.pose_pub.publish(pose)

    def _quat_from_yaw(self, yaw):
        return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    def _update_pose(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0),
                                              rospy.Duration(0.25))
            trans, _ = self.tf_listener.lookupTransform('map', 'base_link',
                                                        rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            return True
        except Exception:
            return False

    def _yaw(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0),
                                              rospy.Duration(0.25))
            _, rot = self.tf_listener.lookupTransform('map', 'base_link',
                                                      rospy.Time(0))
            import tf.transformations as tft
            _, _, yaw = tft.euler_from_quaternion(rot)
            return yaw
        except Exception:
            return 0.0

    def _yaw_error(self, yaw_deg):
        return _angle_norm(yaw_deg / 180.0 * math.pi - self._yaw())

    def _stop(self, duration=0.12):
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(Twist())
            if (rospy.Time.now() - start).to_sec() >= duration:
                break
            rate.sleep()

    def _cancel_nav(self):
        try:
            self.move_base.cancel_all_goals()
        except Exception:
            pass

    def _rotate_to(self, yaw_deg, timeout=ROTATE_TIMEOUT):
        rospy.loginfo('[V2][ROTATE] to %.0fdeg', yaw_deg)
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            err = self._yaw_error(yaw_deg)
            if abs(err) <= YAW_TOL_DEG / 180.0 * math.pi:
                self._stop(0.08)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                self._stop(0.08)
                rospy.logwarn('[V2][ROTATE] timeout err=%.1fdeg',
                              err * 180.0 / math.pi)
                return False
            wz = _clamp(err * 1.6, -ROTATE_MAX_WZ, ROTATE_MAX_WZ)
            if abs(wz) < ROTATE_MIN_WZ:
                wz = ROTATE_MIN_WZ if wz > 0 else -ROTATE_MIN_WZ
            cmd = Twist()
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)
            rate.sleep()
        return False

    def _drive_to(self, x, y, yaw_deg, tol, timeout, max_vx, max_vy, tag):
        rospy.loginfo('[V2][%s] target=(%.3f,%.3f,%.0fdeg) tol=%.3f',
                      tag, x, y, yaw_deg, tol)
        self._rotate_to(yaw_deg)
        start = rospy.Time.now()
        stable = 0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._update_pose()
            dx = x - self.robot_x
            dy = y - self.robot_y
            dist = math.hypot(dx, dy)
            yaw = self._yaw()
            yaw_err = _angle_norm(yaw_deg / 180.0 * math.pi - yaw)
            if dist <= tol and abs(yaw_err) <= 12.0 / 180.0 * math.pi:
                stable += 1
                if stable >= 3:
                    self._stop(0.08)
                    rospy.loginfo('[V2][%s] OK dist=%.3f robot=(%.3f,%.3f)',
                                  tag, dist, self.robot_x, self.robot_y)
                    return True
            else:
                stable = 0

            if (rospy.Time.now() - start).to_sec() > timeout:
                self._stop(0.08)
                rospy.logwarn('[V2][%s] timeout dist=%.3f robot=(%.3f,%.3f)',
                              tag, dist, self.robot_x, self.robot_y)
                return dist <= tol * 1.4

            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            ex_body = cos_yaw * dx + sin_yaw * dy
            ey_body = -sin_yaw * dx + cos_yaw * dy

            vx = _clamp(ex_body * DRIVE_GAIN_X, -max_vx, max_vx)
            vy = _clamp(ey_body * DRIVE_GAIN_Y, -max_vy, max_vy)
            if abs(ex_body) > tol and abs(vx) < MIN_VX:
                vx = MIN_VX if ex_body > 0 else -MIN_VX
            if abs(ey_body) > tol and abs(vy) < MIN_VY:
                vy = MIN_VY if ey_body > 0 else -MIN_VY

            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = _clamp(yaw_err * YAW_HOLD_GAIN, -0.35, 0.35)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        return False

    # ===================== Shooting =====================

    def _fire(self):
        if self.ser is None:
            rospy.logwarn('[V2][SHOOT] serial disabled, dry-run fire')
            return False
        try:
            fire_bytes = self.ser.write(
                b'\x55\x01\x12\x00\x00\x00\x01\x69')
            self.ser.flush()
            rospy.sleep(0.09)
            stop_bytes = self.ser.write(
                b'\x55\x01\x11\x00\x00\x00\x01\x68')
            self.ser.flush()
            ok = fire_bytes == 8 and stop_bytes == 8
            if ok:
                rospy.loginfo('[V2][SHOOT] pulse sent: fire=%d stop=%d',
                              fire_bytes, stop_bytes)
            else:
                rospy.logerr('[V2][SHOOT] incomplete serial write: fire=%s stop=%s',
                             str(fire_bytes), str(stop_bytes))
            return ok
        except Exception as exc:
            rospy.logerr('[V2][SHOOT] serial write failed: %s', str(exc))
            return False

    def _wait_shoot_done(self, target_type, timeout=SHOOT_TIMEOUT):
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if target_type == 'circular' and not self.should_attack_circular:
                return self.last_shoot_ok
            if target_type == 'rotating' and not self.should_attack_rotating:
                return self.last_shoot_ok
            if target_type == 'moving' and not self.should_attack_moving:
                return self.last_shoot_ok
            if (rospy.Time.now() - start).to_sec() > timeout:
                self.should_attack_circular = False
                self.should_attack_rotating = False
                self.should_attack_moving = False
                self._stop(0.1)
                rospy.logwarn('[V2][SHOOT] timeout target=%s', target_type)
                return False
            rate.sleep()
        return False

    def _shoot_task(self, target_type):
        self._stop(0.12)
        self.last_shoot_ok = False
        if target_type == 'circular':
            self.should_attack_circular = True
        elif target_type == 'rotating':
            self.should_attack_rotating = True
        elif target_type == 'moving':
            self.should_attack_moving = True
        else:
            return False
        return self._wait_shoot_done(target_type)

    def _circular_cb(self, data):
        if not self.should_attack_circular:
            self.circular_stable_frames = 0
            return

        target_id = self.config_target_ids.get('circular', 34)
        center_x = 320.0
        threshold = 9.0
        search_wz = self.global_params.get('circular_search_wz', 0.12)
        offset = data.x - center_x
        abs_offset = abs(offset)

        if data.z != target_id:
            self.circular_stable_frames = 0
            cmd = Twist()
            if data.z == 255:
                cmd.angular.z = search_wz
            self.cmd_pub.publish(cmd)
            rospy.logwarn_throttle(
                1.0,
                '[V2][SHOOT] circular waiting id=%.0f expected=%d search_wz=%.3f',
                data.z, target_id, cmd.angular.z)
            return

        if abs_offset > threshold:
            self.circular_stable_frames = 0
            kp = 0.012 if abs_offset < 45.0 else 0.02
            cmd = Twist()
            cmd.angular.z = _clamp(-kp * offset, -0.65, 0.65)
            if abs(cmd.angular.z) < 0.08:
                cmd.angular.z = 0.08 if cmd.angular.z > 0 else -0.08
            self.cmd_pub.publish(cmd)
            return

        self.circular_stable_frames += 1
        self.cmd_pub.publish(Twist())
        if self.circular_stable_frames >= 1:
            self.last_shoot_ok = self._fire()
            if self.last_shoot_ok:
                rospy.loginfo('[V2][SHOOT] circular hit')
            else:
                rospy.logerr('[V2][SHOOT] circular pulse failed')
            self.circular_stable_frames = 0
            self.should_attack_circular = False

    def _ar_cb(self, data):
        if self.should_attack_rotating:
            self._aim_ar(data, 'rotating', self.target_id_rotating)
        if self.should_attack_moving:
            self._aim_ar(data, 'moving', self.target_id_moving)

    def _aim_ar(self, data, target_type, target_id):
        if target_id is None:
            self.cmd_pub.publish(Twist())
            return

        marker = None
        for item in data.markers:
            if item.id == target_id:
                marker = item
                break
        if marker is None:
            if target_type == 'rotating':
                self.rotating_stable_frames = 0
            else:
                self.moving_stable_frames = 0
            self.cmd_pub.publish(Twist())
            return

        ax = marker.pose.pose.position.x
        ay = marker.pose.pose.position.y
        threshold = 0.085
        y_ok = True
        if target_type == 'rotating':
            threshold = self.global_params.get('rotating_x_threshold', 0.1)
            y_min = self.global_params.get('rotating_y_min', -0.1)
            y_max = self.global_params.get('rotating_y_max', 0.1)
            y_ok = y_min <= ay <= y_max

        if abs(ax) > threshold:
            if target_type == 'rotating':
                self.rotating_stable_frames = 0
            else:
                self.moving_stable_frames = 0
            kp = 0.65 if abs(ax) < 0.18 else 1.0
            max_wz = 0.58 if target_type == 'rotating' else 0.62
            cmd = Twist()
            cmd.angular.z = _clamp(-kp * ax, -max_wz, max_wz)
            if abs(cmd.angular.z) < 0.07:
                cmd.angular.z = 0.07 if cmd.angular.z > 0 else -0.07
            self.cmd_pub.publish(cmd)
            return

        if not y_ok:
            self.cmd_pub.publish(Twist())
            self.rotating_stable_frames = 0
            return

        self.cmd_pub.publish(Twist())
        if target_type == 'rotating':
            self.rotating_stable_frames += 1
            stable_frames_required = int(self.global_params.get(
                'rotating_stable_frames_required', 2))
            if self.rotating_stable_frames < stable_frames_required:
                return
            self.last_shoot_ok = self._fire()
            rospy.sleep(0.18)
            if self.last_shoot_ok:
                rospy.loginfo('[V2][SHOOT] rotating hit id=%s', target_id)
            else:
                rospy.logerr('[V2][SHOOT] rotating pulse failed id=%s',
                             target_id)
            self.rotating_stable_frames = 0
            self.should_attack_rotating = False
        else:
            self.moving_stable_frames += 1
            if self.moving_stable_frames < 2:
                return
            self.last_shoot_ok = self._fire()
            rospy.sleep(0.10)
            if self.last_shoot_ok:
                rospy.loginfo('[V2][SHOOT] moving hit id=%s', target_id)
            else:
                rospy.logerr('[V2][SHOOT] moving pulse failed id=%s',
                             target_id)
            self.moving_stable_frames = 0
            self.should_attack_moving = False

    # ===================== Route actions =====================

    def _go_task(self, point):
        x, y = point['x'], point['y']
        yaw = point.get('yaw', 0.0)
        name = _safe(point.get('name', 'task'))
        rospy.loginfo('[V2][TASK] %s', name)
        ok_fast = self._drive_to(x, y, yaw, TASK_FAST_TOL,
                                 TASK_FAST_TIMEOUT, TASK_MAX_VX,
                                 TASK_MAX_VY, 'TASK_FAST')
        ok_fine = self._drive_to(x, y, yaw, TASK_FINE_TOL,
                                 TASK_FINE_TIMEOUT, TASK_FINE_VX,
                                 TASK_FINE_VY, 'TASK_FINE')
        return ok_fast or ok_fine

    def _go_back(self, point):
        return self._drive_to(point['x'], point['y'], point.get('yaw', 0.0),
                              BACK_TOL, BACK_TIMEOUT, BACK_MAX_VX,
                              BACK_MAX_VY, 'BACK')

    def _go_relay(self, point):
        return self._drive_to(point['x'], point['y'], CORRIDOR_YAW_DEG,
                              CORRIDOR_TOL, CORRIDOR_TIMEOUT,
                              CORRIDOR_MAX_VX, CORRIDOR_MAX_VY,
                              'CORRIDOR')

    def _go_end(self, point):
        pre = point.get('pre_point', None)
        if pre:
            self._drive_to(pre['x'], pre['y'], point.get('yaw', 0.0),
                           END_PRE_TOL, END_PRE_TIMEOUT, CORRIDOR_MAX_VX,
                           CORRIDOR_MAX_VY, 'END_PRE')
        return self._drive_to(point['x'], point['y'], CORRIDOR_YAW_DEG,
                              END_TOL, END_TIMEOUT, END_MAX_VX,
                              END_MAX_VY, 'END')

    def _run_route(self):
        for index, point in enumerate(self.route_points):
            ptype = point.get('type', 'relay')
            target_type = point.get('target_type', '')
            self._set_state('ROUTE_%d_%s' % (index + 1, ptype))
            rospy.loginfo('[V2][ROUTE] %d/%d %s type=%s',
                          index + 1, len(self.route_points),
                          _safe(point.get('name', 'unknown')), ptype)

            ok = True
            if ptype == 'task':
                ok = self._go_task(point)
                if ok:
                    ok = self._shoot_task(target_type)
            elif ptype == 'back_y_only':
                ok = self._go_back(point)
            elif ptype == 'relay':
                ok = self._go_relay(point)
            elif ptype == 'end':
                ok = self._go_end(point)
            else:
                ok = self._drive_to(point['x'], point['y'],
                                    point.get('yaw', 0.0), CORRIDOR_TOL,
                                    CORRIDOR_TIMEOUT, TASK_MAX_VX,
                                    TASK_MAX_VY, 'POINT')

            if not ok:
                rospy.logwarn('[V2][ROUTE] point failed, continue carefully')
        return True

    # ===================== Main =====================

    def run(self):
        self._set_state('WAIT_START')
        self._set_initial_pose()
        rospy.loginfo('[V2] Place robot at start mark, press Enter')
        raw_input('Press Enter to start competition_v2: ')
        self._say(u'比赛开始')

        self._set_state('VOICE')
        if not self._wait_voice():
            rospy.logerr('[V2] voice failed, stop before route')
            self._set_state('FINISH')
            self._stop(0.2)
            return

        rospy.loginfo('[V2] target IDs rotating=%s moving=%s',
                      self.target_id_rotating, self.target_id_moving)
        self._set_state('RUN_ROUTE')
        self._cancel_nav()
        self._run_route()
        self._set_state('FINISH')
        self._stop(0.3)
        rospy.loginfo('[V2] competition finished')
        self._say(u'比赛结束')


if __name__ == '__main__':
    try:
        CompetitionControlV2().run()
    except rospy.ROSInterruptException:
        pass
