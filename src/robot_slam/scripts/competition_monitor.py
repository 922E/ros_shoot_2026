#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026 射击挑战赛 — 比赛监控节点
监控: 2分钟倒计时 / 30s启动检测 / 20s卡死检测 / 围挡边界检测
"""

import rospy
import math
import tf
from std_msgs.msg import String


class CompetitionMonitor:
    def __init__(self):
        rospy.init_node('competition_monitor')

        # 参数
        self.total_time = rospy.get_param('~total_time', 120.0)
        self.start_motion_timeout = rospy.get_param('~start_motion_timeout', 30.0)
        self.stuck_timeout = rospy.get_param('~stuck_timeout', 20.0)
        self.field_half = rospy.get_param('~field_half_size', 1.8)

        # 状态
        self.competition_started = False
        self.competition_start_time = None
        self.last_motion_time = None
        self.last_state_change_time = None
        self.current_state = None

        # 机器人位置
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_x = 0.0
        self.last_y = 0.0

        # tf
        self.tf_listener = tf.TransformListener()

        # 发布告警
        self.warning_pub = rospy.Publisher('/voiceWords', String, queue_size=10)

        # 订阅状态机状态
        rospy.Subscriber('/competition_state', String, self._state_callback)

        rospy.loginfo("competition_monitor 初始化完成")
        rospy.loginfo("总限时: %.0fs | 启动超时: %.0fs | 卡死超时: %.0fs | 场地半宽: %.1fm",
                      self.total_time, self.start_motion_timeout,
                      self.stuck_timeout, self.field_half)

    def _state_callback(self, msg):
        new_state = msg.data
        if new_state != self.current_state:
            self.current_state = new_state
            self.last_state_change_time = rospy.Time.now()
            rospy.loginfo("比赛状态变更: %s", new_state)
            if new_state == 'NAV_LOOP' and not self.competition_started:
                self.competition_started = True
                self.competition_start_time = rospy.Time.now()

    def _update_pose(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_link',
                                              rospy.Time(0), rospy.Duration(0.1))
            trans, _ = self.tf_listener.lookupTransform('map', 'base_link',
                                                         rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass

    def _check_boundary(self):
        """围挡/边界检测"""
        if abs(self.robot_x) > self.field_half or abs(self.robot_y) > self.field_half:
            rospy.logerr("警告: 超出场地边界! (%.2f, %.2f)", self.robot_x, self.robot_y)
            return True
        return False

    def _check_stuck(self):
        """状态卡死检测"""
        if self.current_state == 'WAIT_START':
            return False
        if self.last_state_change_time is None:
            return False
        elapsed = (rospy.Time.now() - self.last_state_change_time).to_sec()
        if elapsed > self.stuck_timeout:
            rospy.logerr("警告: 状态卡死! 当前状态 %s 已持续 %.1fs",
                         self.current_state, elapsed)
            return True
        return False

    def _check_total_time(self):
        """总时间检测"""
        if not self.competition_started or self.competition_start_time is None:
            return False
        elapsed = (rospy.Time.now() - self.competition_start_time).to_sec()
        remaining = self.total_time - elapsed
        if remaining < 30:
            rospy.logwarn_throttle(5, "剩余时间: %.0f 秒", remaining)
        if remaining <= 0:
            rospy.logerr("比赛时间到!")
            return True
        return False

    def _check_motion(self):
        """运动检测"""
        dx = abs(self.robot_x - self.last_x)
        dy = abs(self.robot_y - self.last_y)
        moved = (dx > 0.005 or dy > 0.005)

        if moved and self.last_motion_time is None:
            self.last_motion_time = rospy.Time.now()

        return moved

    def run(self):
        rate = rospy.Rate(5)  # 5Hz 足够

        while not rospy.is_shutdown():
            self._update_pose()

            # 检查运动 (始终更新)
            moved = self._check_motion()
            self.last_x = self.robot_x
            self.last_y = self.robot_y

            if self.competition_started:
                if self._check_boundary():
                    self.warning_pub.publish(String("超出场地边界"))

                if self._check_stuck():
                    self.warning_pub.publish(String("状态卡死"))

                if self._check_total_time():
                    self.warning_pub.publish(String("比赛时间到"))

            rate.sleep()


if __name__ == '__main__':
    try:
        mon = CompetitionMonitor()
        mon.run()
    except rospy.ROSInterruptException:
        pass
