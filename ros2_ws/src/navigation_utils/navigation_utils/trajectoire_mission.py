#!/usr/bin/env python3

import math
import time
from typing import List, Dict

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml


def quaternion_from_yaw(yaw: float):
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0),
    }


class TrajectoireMission(Node):
    """Send a sequence of map-frame goals to Nav2."""

    def __init__(self) -> None:
        super().__init__('trajectoire_mission')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('loop', False)
        self.declare_parameter('pause_s', 0.0)
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('emergency_stop_distance', 0.5)
        self.declare_parameter('emergency_front_sector_deg', 15.0) 
        self.declare_parameter('min_valid_range', 0.05)
        self.declare_parameter('max_valid_range', 8.0)

        self.waypoints_file = str(self.get_parameter('waypoints_file').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.pause_s = float(self.get_parameter('pause_s').value)
        action_name = str(self.get_parameter('action_name').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.emergency_front_sector_rad = math.radians(
            float(self.get_parameter('emergency_front_sector_deg').value)
        )
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.max_valid_range = float(self.get_parameter('max_valid_range').value)

        self._client = ActionClient(self, NavigateToPose, action_name)
        self._waypoints: List[Dict[str, float]] = []
        self._emergency_stop = False
        self._last_front_dist = None
        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, msg: LaserScan) -> None:
        front_dist = self._sector_min(msg, center_angle=0.0, half_width=self.emergency_front_sector_rad)
        self._last_front_dist = front_dist
        self._emergency_stop = front_dist is not None and front_dist < self.emergency_stop_distance

    def _sector_min(self, scan: LaserScan, center_angle: float, half_width: float):
        vals: List[float] = []
        angle = scan.angle_min
        for rng in scan.ranges:
            rel = math.atan2(math.sin(angle - center_angle), math.cos(angle - center_angle))
            if abs(rel) <= half_width and math.isfinite(rng):
                if self.min_valid_range <= rng <= self.max_valid_range:
                    vals.append(rng)
            angle += scan.angle_increment
        return min(vals) if vals else None

    def load_waypoints(self) -> bool:
        if not self.waypoints_file:
            self.get_logger().error('Missing parameter waypoints_file.')
            return False

        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(f'Cannot read waypoints file: {exc}')
            return False

        points = data.get('waypoints') if isinstance(data, dict) else None
        if not isinstance(points, list) or not points:
            self.get_logger().error("Invalid file format. Expected: {'waypoints': [...]} with items.")
            return False

        parsed: List[Dict[str, float]] = []
        for i, item in enumerate(points):
            if not isinstance(item, dict):
                self.get_logger().error(f'Waypoint #{i} is not an object.')
                return False
            try:
                parsed.append(
                    {
                        'x': float(item['x']),
                        'y': float(item['y']),
                        'yaw': float(item.get('yaw', 0.0)),
                    }
                )
            except Exception:
                self.get_logger().error(
                    f"Waypoint #{i} must contain at least numeric 'x' and 'y'."
                )
                return False

        self._waypoints = parsed
        self.get_logger().info(f'Loaded {len(self._waypoints)} waypoint(s) from {self.waypoints_file}')
        return True

    def run(self) -> int:
        if not self.load_waypoints():
            return 1

        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server is not available.')
            return 2

        lap = 0
        while rclpy.ok():
            lap += 1
            self.get_logger().info(f'Starting mission lap #{lap}')

            for idx, wp in enumerate(self._waypoints, start=1):
                self.get_logger().info(
                    f"[{idx}/{len(self._waypoints)}] Goal x={wp['x']:.2f}, y={wp['y']:.2f}, yaw={wp['yaw']:.2f}"
                )

                goal = NavigateToPose.Goal()
                goal.pose = PoseStamped()
                goal.pose.header.frame_id = self.frame_id
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = wp['x']
                goal.pose.pose.position.y = wp['y']
                q = quaternion_from_yaw(wp['yaw'])
                goal.pose.pose.orientation.x = q['x']
                goal.pose.pose.orientation.y = q['y']
                goal.pose.pose.orientation.z = q['z']
                goal.pose.pose.orientation.w = q['w']

                send_future = self._client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
                goal_handle = send_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().error('Goal rejected by Nav2.')
                    return 3

                result_future = goal_handle.get_result_async()
                while rclpy.ok():
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.2)

                    if self._emergency_stop:
                        self.get_logger().error(
                            f'Emergency stop: obstacle ahead at {self._last_front_dist:.2f} m. '
                            'Cancelling current goal and stopping mission.'
                        )
                        cancel_future = goal_handle.cancel_goal_async()
                        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                        return 6

                    if result_future.done():
                        break

                wrapped = result_future.result()
                if wrapped is None:
                    self.get_logger().error('No result received from Nav2.')
                    return 4

                if wrapped.status != 4:  # GoalStatus.STATUS_SUCCEEDED
                    self.get_logger().error(f'Goal failed with status={wrapped.status}')
                    return 5

                if self.pause_s > 0.0:
                    time.sleep(self.pause_s)

            if not self.loop:
                self.get_logger().info('Mission completed.')
                return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoireMission()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
