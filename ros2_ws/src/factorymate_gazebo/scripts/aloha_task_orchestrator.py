#!/usr/bin/env python3
"""Odom-based pick-and-place orchestrator for Mobile ALOHA.

Drives with /mobile_aloha/odom feedback and /mobile_aloha/cmd_vel. Arm motions
use FollowJointTrajectory on the FL/FR action servers from aloha_action_bridge.

Mission (offsets from the first odom pose; works for world or relative odom):
  1. [DRIVE] pickup  +1.15 m forward
  2. [PICK-A] sleep -> reach (grippers open)
  3. [PICK-B] close grippers
  4. [GRASP] attach crate so it rides with the robot
  5. [DRIVE] drop-off +1.15 m forward, +1.6 m left
  6. [PLACE] open grippers, release crate, return to sleep

Do not declare use_sim_time here; pass it on the command line:
  ros2 run factorymate_gazebo aloha_task_orchestrator.py --ros-args -p use_sim_time:=true
"""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 9))

SLEEP: Tuple[float, ...] = (0.0, 1.70, -1.45, 0.0, 0.35, 0.0, 0.04, -0.04)
REACH: Tuple[float, ...] = (0.35, 0.55, -0.45, 0.0, 0.20, 0.0, 0.04, -0.04)
GRIPPER_OPEN = (0.04, -0.04)
GRIPPER_CLOSED = (0.00, 0.00)

Pose2D = Tuple[float, float, float]


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _duration(sec: float) -> Duration:
    whole = int(sec)
    d = Duration()
    d.sec = whole
    d.nanosec = int(round((sec - whole) * 1e9))
    return d


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def arm_names(prefix: str) -> List[str]:
    return [f'{prefix}_{j}' for j in ARM_JOINTS]


def with_gripper(joints: Sequence[float], gripper: Sequence[float]) -> List[float]:
    values = [float(v) for v in joints]
    values[6] = float(gripper[0])
    values[7] = float(gripper[1])
    return values


def mirrored_reach(prefix: str) -> List[float]:
    """FL uses +joint1 yaw; FR mirrors so the two arms do not collide."""
    values = [float(v) for v in REACH]
    if prefix == 'fr':
        values[0] = -abs(values[0])
    elif prefix == 'fl':
        values[0] = abs(values[0])
    return values


class AlohaTaskOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('aloha_task_orchestrator')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('odom_topic', '/mobile_aloha/odom')
        self.declare_parameter('cmd_vel_topic', '/mobile_aloha/cmd_vel')
        self.declare_parameter('fl_action', '/mobile_aloha/fl_arm_controller/follow_joint_trajectory')
        self.declare_parameter('fr_action', '/mobile_aloha/fr_arm_controller/follow_joint_trajectory')
        # Waypoints are offsets (meters / radians) from the first odom pose so
        # both world-frame odom and spawn-relative odom work.
        self.declare_parameter('pickup_forward', 1.15)
        self.declare_parameter('pickup_left', 0.0)
        self.declare_parameter('pickup_yaw_offset', 0.0)
        self.declare_parameter('dropoff_forward', 1.15)
        self.declare_parameter('dropoff_left', 1.6)
        self.declare_parameter('dropoff_yaw_offset', 1.57)
        self.declare_parameter('attach_topic', '/pick_crate/attach')
        self.declare_parameter('server_wait_sec', 60.0)
        self.declare_parameter('odom_wait_sec', 15.0)
        self.declare_parameter('drive_timeout_sec', 120.0)
        self.declare_parameter('arm_timeout_sec', 60.0)
        self.declare_parameter('reach_duration_sec', 8.0)
        self.declare_parameter('grip_duration_sec', 3.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('xy_tolerance', 0.12)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('heading_align', 0.18)
        self.declare_parameter('kp_linear', 0.7)
        self.declare_parameter('kp_angular', 1.6)
        self.declare_parameter('max_linear', 0.45)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('min_linear', 0.05)
        self.declare_parameter('min_angular', 0.08)

        self._cb = ReentrantCallbackGroup()
        self._pose: Optional[Pose2D] = None
        self._pose_lock = threading.Lock()
        self._cmd_pub = self.create_publisher(
            Twist, self._str('cmd_vel_topic'), 10
        )
        self._attach_pub = self.create_publisher(Bool, self._str('attach_topic'), 10)
        self.create_subscription(
            Odometry,
            self._str('odom_topic'),
            self._on_odom,
            qos_profile_sensor_data,
            callback_group=self._cb,
        )
        self._fl = ActionClient(
            self, FollowJointTrajectory, self._str('fl_action'), callback_group=self._cb
        )
        self._fr = ActionClient(
            self, FollowJointTrajectory, self._str('fr_action'), callback_group=self._cb
        )

    def _str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._pose_lock:
            self._pose = (float(p.x), float(p.y), float(yaw))

    def _current_pose(self) -> Optional[Pose2D]:
        with self._pose_lock:
            return self._pose

    def run_mission(self) -> bool:
        dry = self._bool('dry_run')
        if dry:
            self.get_logger().info('[DRY-RUN] Printing mission only; no cmd_vel or arm goals')
            pickup = (1.15, 0.0, 0.0)
            dropoff = (1.15, 1.6, 1.57)
        else:
            if not self._wait_for_odom():
                return False
            if not self._wait_for_arm_servers():
                return False
            start = self._current_pose()
            if start is None:
                return False
            pickup = self._offset_goal(
                start,
                self._float('pickup_forward'),
                self._float('pickup_left'),
                self._float('pickup_yaw_offset'),
            )
            dropoff = self._offset_goal(
                start,
                self._float('dropoff_forward'),
                self._float('dropoff_left'),
                self._float('dropoff_yaw_offset'),
            )
            self.get_logger().info(
                f'Start odom=({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})'
            )

        steps = (
            (f'[DRIVE] pickup → ({pickup[0]:.2f}, {pickup[1]:.2f}, yaw={pickup[2]:.2f})',
             lambda: self._drive_to(*pickup)),
            ('[PICK-A] Reaching...', self._pick_reach),
            ('[PICK-B] Closing grippers...', self._pick_close),
            ('[GRASP] Attaching crate...', self._attach_crate),
            (f'[DRIVE] drop-off → ({dropoff[0]:.2f}, {dropoff[1]:.2f}, yaw={dropoff[2]:.2f})',
             lambda: self._drive_to(*dropoff)),
            ('[PLACE-A] Opening grippers...', self._place_open),
            ('[PLACE-B] Releasing crate...', self._release_crate),
            ('[PLACE-C] Returning to sleep...', self._place_sleep),
        )
        for label, action in steps:
            self.get_logger().info(label)
            if dry:
                continue
            if not action():
                self._stop()
                self.get_logger().error(f'Failed at {label}')
                return False

        self._stop()
        self.get_logger().info('[DONE] Pick-and-place mission complete')
        return True

    def _offset_goal(
        self, start: Pose2D, forward: float, left: float, yaw_offset: float
    ) -> Pose2D:
        sx, sy, syaw = start
        x = sx + forward * math.cos(syaw) - left * math.sin(syaw)
        y = sy + forward * math.sin(syaw) + left * math.cos(syaw)
        return x, y, _wrap(syaw + yaw_offset)

    def _set_attached(self, attached: bool) -> None:
        msg = Bool()
        msg.data = attached
        for _ in range(5):
            self._attach_pub.publish(msg)
            time.sleep(0.05)

    def _attach_crate(self) -> bool:
        self._set_attached(True)
        time.sleep(0.4)
        return True

    def _release_crate(self) -> bool:
        self._set_attached(False)
        time.sleep(0.4)
        return True

    def _wait_for_odom(self) -> bool:
        timeout = self._float('odom_wait_sec')
        self.get_logger().info(f'Waiting for odom on {self._str("odom_topic")}...')
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            pose = self._current_pose()
            if pose is not None:
                self.get_logger().info(
                    f'Odom ready at ({pose[0]:.2f}, {pose[1]:.2f}, yaw={pose[2]:.2f})'
                )
                return True
            time.sleep(0.05)
        self.get_logger().error('Timed out waiting for /mobile_aloha/odom')
        return False

    def _wait_for_arm_servers(self) -> bool:
        timeout = self._float('server_wait_sec')
        for label, client in (
            ('FL FollowJointTrajectory', self._fl),
            ('FR FollowJointTrajectory', self._fr),
        ):
            self.get_logger().info(f'Waiting for {label}...')
            if not client.wait_for_server(timeout_sec=timeout):
                self.get_logger().error(f'Timed out waiting for {label}')
                return False
        return True

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _drive_to(self, x: float, y: float, yaw: float) -> bool:
        """Rotate to heading, drive the remaining distance, then face goal yaw."""
        rate_hz = max(5.0, self._float('control_rate_hz'))
        dt = 1.0 / rate_hz
        timeout = self._float('drive_timeout_sec')
        xy_tol = self._float('xy_tolerance')
        yaw_tol = self._float('yaw_tolerance')
        heading_align = self._float('heading_align')
        kp_lin = self._float('kp_linear')
        kp_ang = self._float('kp_angular')
        max_lin = self._float('max_linear')
        max_ang = self._float('max_angular')
        min_lin = self._float('min_linear')
        min_ang = self._float('min_angular')

        deadline = time.monotonic() + timeout
        phase = 'rotate'
        last_log = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            pose = self._current_pose()
            if pose is None:
                time.sleep(dt)
                continue
            px, py, pyaw = pose
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            heading_err = _wrap(bearing - pyaw)
            yaw_err = _wrap(yaw - pyaw)

            cmd = Twist()
            if dist > xy_tol:
                if abs(heading_err) > heading_align or phase == 'rotate':
                    phase = 'rotate'
                    cmd.angular.z = _clip(kp_ang * heading_err, max_ang)
                    if abs(cmd.angular.z) < min_ang:
                        cmd.angular.z = math.copysign(min_ang, heading_err)
                    if abs(heading_err) <= heading_align:
                        phase = 'drive'
                else:
                    phase = 'drive'
                    cmd.linear.x = _clip(kp_lin * dist, max_lin)
                    if cmd.linear.x < min_lin:
                        cmd.linear.x = min_lin
                    cmd.angular.z = _clip(0.4 * kp_ang * heading_err, max_ang)
            else:
                phase = 'final_yaw'
                if abs(yaw_err) <= yaw_tol:
                    self._stop()
                    self.get_logger().info(
                        f'[DRIVE] Arrived ({px:.2f}, {py:.2f}, yaw={pyaw:.2f})'
                    )
                    return True
                cmd.angular.z = _clip(kp_ang * yaw_err, max_ang)
                if abs(cmd.angular.z) < min_ang:
                    cmd.angular.z = math.copysign(min_ang, yaw_err)

            self._cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_log > 1.0:
                last_log = now
                self.get_logger().info(
                    f'[DRIVE] {phase} pose=({px:.2f},{py:.2f},{pyaw:.2f}) '
                    f'dist={dist:.2f} heading_err={heading_err:.2f}'
                )
            time.sleep(dt)

        self._stop()
        self.get_logger().error('[DRIVE] Timed out before reaching waypoint')
        return False

    def _pick_reach(self) -> bool:
        t_reach = self._float('reach_duration_sec')
        fl = with_gripper(mirrored_reach('fl'), GRIPPER_OPEN)
        fr = with_gripper(mirrored_reach('fr'), GRIPPER_OPEN)
        return self._send_arms({'fl': [(t_reach, fl)], 'fr': [(t_reach, fr)]})

    def _pick_close(self) -> bool:
        t_grip = self._float('grip_duration_sec')
        fl = with_gripper(mirrored_reach('fl'), GRIPPER_CLOSED)
        fr = with_gripper(mirrored_reach('fr'), GRIPPER_CLOSED)
        return self._send_arms({'fl': [(t_grip, fl)], 'fr': [(t_grip, fr)]})

    def _place_open(self) -> bool:
        t_grip = self._float('grip_duration_sec')
        fl = with_gripper(mirrored_reach('fl'), GRIPPER_OPEN)
        fr = with_gripper(mirrored_reach('fr'), GRIPPER_OPEN)
        return self._send_arms({'fl': [(t_grip, fl)], 'fr': [(t_grip, fr)]})

    def _place_sleep(self) -> bool:
        t_reach = self._float('reach_duration_sec')
        sleep = with_gripper(SLEEP, GRIPPER_OPEN)
        return self._send_arms({'fl': [(t_reach, sleep)], 'fr': [(t_reach, sleep)]})

    def _arm_goal(
        self, prefix: str, waypoints: Sequence[Tuple[float, Sequence[float]]]
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = arm_names(prefix)
        traj.header.stamp = self.get_clock().now().to_msg()
        for t_sec, positions in waypoints:
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.time_from_start = _duration(t_sec)
            traj.points.append(point)
        goal.trajectory = traj
        return goal

    def _send_arms(self, motions: Dict[str, Sequence[Tuple[float, Sequence[float]]]]) -> bool:
        clients = {'fl': self._fl, 'fr': self._fr}
        sends = []
        for prefix, waypoints in motions.items():
            send = clients[prefix].send_goal_async(self._arm_goal(prefix, waypoints))
            sends.append((prefix, send))

        handles = []
        timeout = self._float('arm_timeout_sec')
        for prefix, send in sends:
            if not self._wait_future(send, timeout):
                return False
            handle = send.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'{prefix} trajectory goal rejected')
                return False
            handles.append((prefix, handle))

        for prefix, handle in handles:
            result = handle.get_result_async()
            if not self._wait_future(result, timeout):
                handle.cancel_goal_async()
                return False
            wrapped = result.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f'{prefix} trajectory status={wrapped.status} '
                    f'error_code={wrapped.result.error_code}'
                )
                return False
        return True

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error('Action future timed out')
                return False
            time.sleep(0.05)
        return bool(future.done())


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = AlohaTaskOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    ok = False
    try:
        ok = node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().warn('Mission interrupted')
        node._stop()
    finally:
        try:
            node._stop()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
