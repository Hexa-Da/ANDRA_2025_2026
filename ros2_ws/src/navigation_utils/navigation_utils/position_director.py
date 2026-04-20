#!/usr/bin/env python3

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class PositionDirector(Node):
    """
    Corridor centering controller based on LaserScan.
    It drives forward while minimizing left/right distance error.
    """

    def __init__(self) -> None:
        super().__init__('position_director')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_publish_hz', 20.0)
        self.declare_parameter('scan_timeout_s', 0.35)
        self.declare_parameter('linear_speed', 0.14)
        self.declare_parameter('steering_trim', -0.015)
        self.declare_parameter('max_angular_speed', 0.045)
        self.declare_parameter('kp_centering', 0.09)
        self.declare_parameter('ki_centering', 0.0)
        self.declare_parameter('kd_centering', 0.0)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('centering_deadband', 0.06)
        self.declare_parameter('angular_smoothing', 0.985)
        self.declare_parameter('integral_limit', 0.20)
        self.declare_parameter('derivative_smoothing', 0.92)
        self.declare_parameter('max_omega_step', 0.004)
        self.declare_parameter('max_centering_error', 0.18)
        self.declare_parameter('micro_correction_error_scale', 0.28)
        self.declare_parameter('two_stage_enabled', True)
        self.declare_parameter('two_stage_first_ratio', 0.35)
        self.declare_parameter('two_stage_observe_s', 0.55)
        self.declare_parameter('front_stop_distance', 0.55)
        self.declare_parameter('side_sector_deg', 35.0)
        self.declare_parameter('left_sector_center_deg', 90.0)
        self.declare_parameter('right_sector_center_deg', -90.0)
        self.declare_parameter('front_sector_deg', 15.0)
        self.declare_parameter('min_valid_range', 0.05)
        self.declare_parameter('max_valid_range', 8.0)
        self.declare_parameter('side_memory_timeout_s', 0.6)
        self.declare_parameter('use_initial_sum_reference', True)
        self.declare_parameter('side_margin_m', 0.10)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._cmd_publish_hz = float(self.get_parameter('cmd_publish_hz').value)
        self._scan_timeout_s = float(self.get_parameter('scan_timeout_s').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._steering_trim = float(self.get_parameter('steering_trim').value)
        self._max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self._kp_centering = float(self.get_parameter('kp_centering').value)
        self._ki_centering = float(self.get_parameter('ki_centering').value)
        self._kd_centering = float(self.get_parameter('kd_centering').value)
        self._invert_steering = bool(self.get_parameter('invert_steering').value)
        self._centering_deadband = float(self.get_parameter('centering_deadband').value)
        self._angular_smoothing = float(self.get_parameter('angular_smoothing').value)
        self._integral_limit = float(self.get_parameter('integral_limit').value)
        self._derivative_smoothing = float(self.get_parameter('derivative_smoothing').value)
        self._front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self._left_sector_center_rad = math.radians(
            float(self.get_parameter('left_sector_center_deg').value)
        )
        self._right_sector_center_rad = math.radians(
            float(self.get_parameter('right_sector_center_deg').value)
        )
        self._side_sector_rad = math.radians(float(self.get_parameter('side_sector_deg').value))
        self._front_sector_rad = math.radians(float(self.get_parameter('front_sector_deg').value))
        self._min_valid_range = float(self.get_parameter('min_valid_range').value)
        self._max_valid_range = float(self.get_parameter('max_valid_range').value)
        self._side_memory_timeout_s = float(self.get_parameter('side_memory_timeout_s').value)
        self._use_initial_sum_reference = bool(
            self.get_parameter('use_initial_sum_reference').value
        )
        self._side_margin_m = float(self.get_parameter('side_margin_m').value)
        self._max_omega_step = float(self.get_parameter('max_omega_step').value)
        self._max_centering_error = float(self.get_parameter('max_centering_error').value)
        self._micro_correction_error_scale = float(
            self.get_parameter('micro_correction_error_scale').value
        )
        self._two_stage_enabled = bool(self.get_parameter('two_stage_enabled').value)
        self._two_stage_first_ratio = float(self.get_parameter('two_stage_first_ratio').value)
        self._two_stage_observe_s = float(self.get_parameter('two_stage_observe_s').value)

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._scan_sub = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._last_log_time = self.get_clock().now()
        self._last_scan_time = self.get_clock().now()
        self._last_omega = 0.0
        self._integral_error = 0.0
        self._last_error = 0.0
        self._last_error_derivative = 0.0
        self._last_time_ns = None
        self._desired_cmd = Twist()
        self._last_left_dist = None
        self._last_right_dist = None
        self._last_left_time_ns = 0
        self._last_right_time_ns = 0
        self._stage1_active = False
        self._stage1_start_ns = 0
        self._stage1_direction = 0
        self._stage1_omega = 0.0
        self._initial_corridor_sum = None

        publish_period = 1.0 / max(1.0, self._cmd_publish_hz)
        self._publish_timer = self.create_timer(publish_period, self._publish_loop)

        self.get_logger().info(
            f'Corridor mode started (scan={self._scan_topic}, cmd_vel={cmd_topic}).'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_time = self.get_clock().now()
        now_ns = self.get_clock().now().nanoseconds
        left_dist = self._sector_median(
            msg, center_angle=self._left_sector_center_rad, half_width=self._side_sector_rad
        )
        right_dist = self._sector_median(
            msg, center_angle=self._right_sector_center_rad, half_width=self._side_sector_rad
        )
        front_dist = self._sector_min(msg, center_angle=0.0, half_width=self._front_sector_rad)

        # Keep a short memory of side distances to handle transient NaN sectors.
        if left_dist is not None:
            self._last_left_dist = left_dist
            self._last_left_time_ns = now_ns
        elif self._last_left_dist is not None:
            age_s = (now_ns - self._last_left_time_ns) / 1e9
            if age_s <= self._side_memory_timeout_s:
                left_dist = self._last_left_dist

        if right_dist is not None:
            self._last_right_dist = right_dist
            self._last_right_time_ns = now_ns
        elif self._last_right_dist is not None:
            age_s = (now_ns - self._last_right_time_ns) / 1e9
            if age_s <= self._side_memory_timeout_s:
                right_dist = self._last_right_dist

        cmd = Twist()

        if front_dist is None:
            # No front data: stop for safety.
            self._reset_pid_state()
            self._desired_cmd = Twist()
            return

        if front_dist < self._front_stop_distance:
            self.get_logger().warn(
                f'Obstacle ahead at {front_dist:.2f} m (< {self._front_stop_distance:.2f} m), stopping.'
            )
            self._reset_pid_state()
            self._desired_cmd = Twist()
            return

        cmd.linear.x = self._linear_speed

        if left_dist is not None and right_dist is not None:
            if self._use_initial_sum_reference and self._initial_corridor_sum is None:
                self._initial_corridor_sum = left_dist + right_dist
                self.get_logger().info(
                    f'Initial corridor sum locked at {self._initial_corridor_sum:.2f} m'
                )

            if self._use_initial_sum_reference and self._initial_corridor_sum is not None:
                target_half = 0.5 * self._initial_corridor_sum
                lower_bound = target_half - self._side_margin_m

                left_deficit = lower_bound - left_dist
                right_deficit = lower_bound - right_dist

                # positive error => turn left, negative error => turn right
                if left_deficit > 0.0 and right_deficit > 0.0:
                    # Both sides too close: correct toward the side with larger margin.
                    if left_dist > right_dist:
                        raw_error = right_deficit
                    else:
                        raw_error = -left_deficit
                elif left_deficit > 0.0:
                    raw_error = -left_deficit
                elif right_deficit > 0.0:
                    raw_error = right_deficit
                else:
                    self._reset_pid_state()
                    self._last_omega = 0.0
                    cmd.angular.z = 0.0
                    self._desired_cmd = cmd
                    self._throttled_log(front_dist, left_dist, right_dist, cmd.angular.z)
                    return

                error = max(-self._max_centering_error, min(self._max_centering_error, raw_error))
            else:
                # Fallback to symmetric centering around left/right distances.
                wall_gap_error = left_dist - right_dist
                if abs(wall_gap_error) <= self._centering_deadband:
                    self._reset_pid_state()
                    self._last_omega = 0.0
                    cmd.angular.z = 0.0
                    self._desired_cmd = cmd
                    self._throttled_log(front_dist, left_dist, right_dist, cmd.angular.z)
                    return

                signed_excess_gap = math.copysign(
                    abs(wall_gap_error) - self._centering_deadband,
                    wall_gap_error,
                )
                corridor_width = max(0.2, left_dist + right_dist)
                error = signed_excess_gap / corridor_width
                error = max(-self._max_centering_error, min(self._max_centering_error, error))

            # Micro-correction mode: avoid amplitude growth by using a bounded
            # proportional command (no integral accumulation, no derivative kick).
            e_scale = max(1e-3, self._micro_correction_error_scale)
            soft_error = math.tanh(error / e_scale)
            omega_full = self._kp_centering * soft_error
            omega = omega_full

            if self._two_stage_enabled:
                first_ratio = max(0.1, min(0.9, self._two_stage_first_ratio))
                now_ns = self.get_clock().now().nanoseconds
                direction = 1 if omega_full > 0.0 else -1

                if not self._stage1_active or direction != self._stage1_direction:
                    self._stage1_active = True
                    self._stage1_start_ns = now_ns
                    self._stage1_direction = direction
                    self._stage1_omega = first_ratio * omega_full

                elapsed_s = max(0.0, (now_ns - self._stage1_start_ns) / 1e9)
                if elapsed_s < max(0.05, self._two_stage_observe_s):
                    omega = self._stage1_omega
                else:
                    # After observation window, release toward full correction.
                    self._stage1_active = False
                    omega = omega_full

            omega = max(-self._max_angular_speed, min(self._max_angular_speed, omega))

            self._last_error = error
            if self._invert_steering:
                omega = -omega

            alpha = max(0.0, min(0.99, self._angular_smoothing))
            omega = alpha * self._last_omega + (1.0 - alpha) * omega
            omega = self._limit_rate(omega, self._last_omega, self._max_omega_step)
            self._last_omega = omega
            cmd.angular.z = omega
        else:
            self._reset_pid_state()
            cmd.angular.z = 0.0

        # Mechanical trim to compensate persistent drift
        # (negative value = slight right turn in ROS convention).
        cmd.angular.z += self._steering_trim
        cmd.angular.z = max(-self._max_angular_speed, min(self._max_angular_speed, cmd.angular.z))

        self._desired_cmd = cmd
        self._throttled_log(front_dist, left_dist, right_dist, cmd.angular.z)

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_loop(self) -> None:
        age_s = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
        if age_s > self._scan_timeout_s:
            self._cmd_pub.publish(Twist())
            return
        self._cmd_pub.publish(self._desired_cmd)

    def _reset_pid_state(self) -> None:
        self._integral_error = 0.0
        self._last_error = 0.0
        self._last_error_derivative = 0.0
        self._last_omega = 0.0
        self._last_time_ns = None
        self._stage1_active = False
        self._stage1_start_ns = 0
        self._stage1_direction = 0
        self._stage1_omega = 0.0

    @staticmethod
    def _limit_rate(target: float, previous: float, max_step: float) -> float:
        step = target - previous
        if step > max_step:
            return previous + max_step
        if step < -max_step:
            return previous - max_step
        return target

    def _sector_min(self, scan: LaserScan, center_angle: float, half_width: float):
        vals: List[float] = []
        angle = scan.angle_min

        for rng in scan.ranges:
            rel = self._normalize_angle(angle - center_angle)
            if abs(rel) <= half_width and math.isfinite(rng):
                if self._min_valid_range <= rng <= self._max_valid_range:
                    vals.append(rng)
            angle += scan.angle_increment

        if not vals:
            return None
        return min(vals)

    def _sector_median(self, scan: LaserScan, center_angle: float, half_width: float):
        vals: List[float] = []
        angle = scan.angle_min

        for rng in scan.ranges:
            rel = self._normalize_angle(angle - center_angle)
            if abs(rel) <= half_width and math.isfinite(rng):
                if self._min_valid_range <= rng <= self._max_valid_range:
                    vals.append(rng)
            angle += scan.angle_increment

        if not vals:
            return None
        vals.sort()
        mid = len(vals) // 2
        if len(vals) % 2 == 0:
            return 0.5 * (vals[mid - 1] + vals[mid])
        return vals[mid]

    @staticmethod
    def _normalize_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def _throttled_log(self, front, left, right, omega):
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds < 800_000_000:
            return
        self._last_log_time = now
        self.get_logger().info(
            f'front={front:.2f}m left={left if left is not None else float("nan"):.2f}m '
            f'right={right if right is not None else float("nan"):.2f}m omega={omega:.2f}rad/s'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
