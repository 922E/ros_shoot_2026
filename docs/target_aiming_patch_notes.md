# Target Aiming Patch Notes

This note extracts the updated target aiming logic from:

```text
src/robot_slam/scripts/competition_control.py
```

Use it when another teammate needs to apply the same changes manually.

## 1. Add Stable Frame Counters

In `CompetitionControl.__init__()`, find:

```python
self.state = 'WAIT_START'
self.current_point_index = 0
```

Add these three lines immediately after it:

```python
self.circular_stable_frames = 0
self.rotating_stable_frames = 0
self.moving_stable_frames = 0
```

The result should look like:

```python
self.state = 'WAIT_START'
self.current_point_index = 0
self.circular_stable_frames = 0
self.rotating_stable_frames = 0
self.moving_stable_frames = 0
```

## 2. Replace `circular_target()`

```python
def circular_target(self, data):
    """环形靶瞄准回调 (same as shoot_2025.py)"""
    global point_msg, should_attack_circular
    target_id = 34
    aim_center_x = 320.0
    fire_threshold_px = 10.0
    stable_frames_required = 1
    kp_far = 0.02
    kp_near = 0.012
    near_threshold_px = 40.0
    max_wz = 0.65
    min_wz = 0.08

    if not should_attack_circular:
        self.circular_stable_frames = 0
        return
    point_msg = data

    if data.z != target_id:
        self.circular_stable_frames = 0
        self.pub.publish(Twist())
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

    else:
        self.circular_stable_frames += 1
        self.pub.publish(Twist())
        rospy.loginfo("Circular target stable frames: %d/%d",
                      self.circular_stable_frames,
                      stable_frames_required)
        if self.circular_stable_frames < stable_frames_required:
            return
        self._fire()
        rospy.loginfo("Circular target hit!")
        self.circular_stable_frames = 0
        should_attack_circular = False
```

## 3. Replace `rotating_target()`

```python
def rotating_target(self, data):
    """旋转靶瞄准回调 (same as shoot_2025.py)"""
    global target_id_rotating, should_attack_rotating
    x_threshold = 0.1
    y_min = -0.1
    y_max = 0.1
    stable_frames_required = 2
    kp_far = 1.0
    kp_near = 0.65
    near_threshold = 0.18
    max_wz = 0.55
    min_wz = 0.06
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

            elif y_ok:
                self.rotating_stable_frames += 1
                self.pub.publish(Twist())
                rospy.loginfo("Rotating target stable frames: %d/%d",
                              self.rotating_stable_frames,
                              stable_frames_required)
                if self.rotating_stable_frames < stable_frames_required:
                    return
                self._fire()
                rospy.sleep(shoot_cooldown)
                rospy.loginfo("Rotating target hit!")
                self.rotating_stable_frames = 0
                should_attack_rotating = False
            else:
                self.rotating_stable_frames = 0
                self.pub.publish(Twist())
            return

    if not found_target:
        self.rotating_stable_frames = 0
        self.pub.publish(Twist())
```

## 4. Replace `moving_target()`

```python
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
            else:
                self.moving_stable_frames += 1
                self.pub.publish(Twist())
                rospy.loginfo("Moving target stable frames: %d/%d",
                              self.moving_stable_frames,
                              stable_frames_required)
                if self.moving_stable_frames < stable_frames_required:
                    return
                self._fire()
                rospy.sleep(shoot_cooldown)
                rospy.loginfo("Moving target hit!")
                self.moving_stable_frames = 0
                should_attack_moving = False
            return

    if not found_target:
        self.moving_stable_frames = 0
        self.pub.publish(Twist())
```

## 5. Current Confirmation Strategy

```text
circular target: 1 stable frame
rotating target: 2 stable frames
moving target: 2 stable frames
```

The first target is static, so it uses one-frame confirmation for speed.
The rotating and moving targets use two-frame confirmation to reduce false shots while keeping response time short.
