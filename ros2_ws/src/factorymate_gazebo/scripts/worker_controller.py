#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Dict, List, Tuple

Waypoint = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def worker_routes(worker_count: int = 9) -> Dict[str, List[Waypoint]]:
    if worker_count < 0 or worker_count > 20:
        raise ValueError('worker_count must be between 0 and 20')
    templates: List[List[Waypoint]] = [
        [(-22, -11), (-12, -11), (-12, -7), (-22, -7), (-22, -11)],
        [(-15, -4), (-5, -4), (-5, 0), (-15, 0), (-15, -4)],
        [(-7, 9), (3, 9), (3, 13), (-7, 13), (-7, 9)],
        [(0, -9), (10, -9), (10, -5), (0, -5), (0, -9)],
        [(8, 5), (18, 5), (18, 9), (8, 9), (8, 5)],
        [(16, -8), (22, -8), (22, 0), (16, 0), (16, -8)],
        [(21, 10), (12, 10), (12, 14), (21, 14), (21, 10)],
        [(-20, 7), (-10, 7), (-10, 3), (-20, 3), (-20, 7)],
        [(2, 13), (2, 4), (-2, 4), (-2, 13), (2, 13)],
    ]
    return {f'worker_{i:02d}': list(templates[(i - 1) % len(templates)]) for i in range(1, worker_count + 1)}


def unicycle_command(
    pose: Pose2D,
    target: Waypoint,
    *,
    max_linear: float = 0.8,
    max_angular: float = 1.2,
) -> Tuple[float, float]:
    x, y, yaw = pose
    tx, ty = target
    dx, dy = tx - x, ty - y
    distance = math.hypot(dx, dy)
    if distance < 0.05:
        return 0.0, 0.0
    desired = math.atan2(dy, dx)
    error = _wrap(desired - yaw)
    angular = max(-max_angular, min(max_angular, 2.0 * error))
    alignment = max(0.0, math.cos(error))
    linear = min(max_linear, 0.65 * distance, max_linear * alignment)
    return linear, angular


try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ImportError:  # Allows pure-logic unit tests outside ROS.
    rclpy = None
    Node = object


if rclpy is not None:
    def _yaw_from_quat(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)


    class WorkerController(Node):
        def __init__(self) -> None:
            super().__init__('factorymate_worker_controller')
            self.declare_parameter('worker_count', 9)
            self.declare_parameter('max_linear', 0.75)
            self.declare_parameter('max_angular', 1.1)
            count = int(self.get_parameter('worker_count').value)
            self.routes = worker_routes(count)
            self.indices = {name: 1 for name in self.routes}
            self.poses: Dict[str, Pose2D] = {}
            self.publishers = {}
            self.subscriptions = []
            for name in self.routes:
                self.publishers[name] = self.create_publisher(Twist, f'/workers/{name}/cmd_vel', 10)
                sub = self.create_subscription(Odometry, f'/workers/{name}/odom', lambda msg, n=name: self._odom(n, msg), 10)
                self.subscriptions.append(sub)
            self.timer = self.create_timer(0.1, self._tick)
            self.get_logger().info(f'controlling {count} warehouse workers')

        def _odom(self, name: str, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.poses[name] = (p.x, p.y, _yaw_from_quat(q))

        def _tick(self) -> None:
            max_linear = float(self.get_parameter('max_linear').value)
            max_angular = float(self.get_parameter('max_angular').value)
            for name, route in self.routes.items():
                pose = self.poses.get(name)
                if pose is None:
                    continue
                idx = self.indices[name]
                target = route[idx]
                if math.hypot(target[0] - pose[0], target[1] - pose[1]) < 0.45:
                    idx = (idx + 1) % len(route)
                    if idx == 0:
                        idx = 1
                    self.indices[name] = idx
                    target = route[idx]
                linear, angular = unicycle_command(pose, target, max_linear=max_linear, max_angular=max_angular)
                msg = Twist()
                msg.linear.x = linear
                msg.angular.z = angular
                self.publishers[name].publish(msg)


def main() -> None:
    if rclpy is None:
        raise RuntimeError('rclpy is required to run worker_controller.py')
    rclpy.init()
    node = WorkerController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
