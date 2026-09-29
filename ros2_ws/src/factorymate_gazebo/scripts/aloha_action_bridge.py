#!/usr/bin/env python3
"""FollowJointTrajectory servers for Mobile ALOHA's four Piper arms.

Each arm action is packed into the 33-element Float64MultiArray expected by
/mobile_aloha/arm_position_controller/commands (JointGroupPositionController).

Joint order matches factorymate_gazebo/config/mobile_aloha_controllers.yaml:
  fl_joint1..8, fr_joint1..8, bl_joint1..8, br_joint1..8, mid_camera_stand_joint
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional, Sequence

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_PREFIXES = ('fl', 'fr', 'bl', 'br')
JOINTS_PER_ARM = 8
COMMAND_JOINTS: List[str] = [
    f'{prefix}_joint{i}' for prefix in ARM_PREFIXES for i in range(1, JOINTS_PER_ARM + 1)
] + ['mid_camera_stand_joint']
N_CMD = len(COMMAND_JOINTS)  # 33
INDEX = {name: i for i, name in enumerate(COMMAND_JOINTS)}


def arm_joint_names(prefix: str) -> List[str]:
    return [f'{prefix}_joint{i}' for i in range(1, JOINTS_PER_ARM + 1)]


def _sec(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def _lerp(a: Sequence[float], b: Sequence[float], t: float) -> List[float]:
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    return [float(ai) + (float(bi) - float(ai)) * t for ai, bi in zip(a, b)]


def interpolate_positions(points: Sequence[JointTrajectoryPoint], t: float) -> List[float]:
    if not points:
        raise ValueError('empty trajectory')
    times = [_sec(p.time_from_start) for p in points]
    if t <= times[0]:
        return list(points[0].positions)
    if t >= times[-1]:
        return list(points[-1].positions)
    for i in range(1, len(points)):
        if t <= times[i]:
            dt = times[i] - times[i - 1]
            alpha = 0.0 if dt <= 1e-9 else (t - times[i - 1]) / dt
            return _lerp(points[i - 1].positions, points[i].positions, alpha)
    return list(points[-1].positions)


class AlohaActionBridge(Node):
    def __init__(self) -> None:
        super().__init__('aloha_action_bridge')
        self.declare_parameter('command_topic', '/mobile_aloha/arm_position_controller/commands')
        self.declare_parameter('joint_states_topic', '/mobile_aloha/joint_states')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('goal_tolerance', 0.15)  # relaxed for Gazebo physics lag
        # self.declare_parameter('use_sim_time', True)

        self._cb = ReentrantCallbackGroup()
        self._state: Dict[str, float] = {}
        self._cmd = [0.0] * N_CMD
        self._have_state = False

        js_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        rate = float(self.get_parameter('publish_rate').value)

        self.create_subscription(JointState, js_topic, self._on_state, 20, callback_group=self._cb)
        self._pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick, callback_group=self._cb)

        self._servers = []
        for prefix in ARM_PREFIXES:
            names = arm_joint_names(prefix)
            server = ActionServer(
                self,
                FollowJointTrajectory,
                f'/mobile_aloha/{prefix}_arm_controller/follow_joint_trajectory',
                execute_callback=lambda handle, p=prefix: self._execute(handle, p),
                goal_callback=lambda goal, n=tuple(names): self._validate(goal, n),
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._cb,
            )
            self._servers.append(server)

        cam_names = ('mid_camera_stand_joint',)
        self._servers.append(
            ActionServer(
                self,
                FollowJointTrajectory,
                '/mobile_aloha/camera_stand_controller/follow_joint_trajectory',
                execute_callback=lambda handle: self._execute(handle, 'cam'),
                goal_callback=lambda goal, n=cam_names: self._validate(goal, n),
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._cb,
            )
        )
        self.get_logger().info(
            f'ALOHA action bridge ready: 4 arms + camera stand -> {cmd_topic} ({N_CMD} joints)'
        )

    def _on_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._state[name] = float(pos)
        if not self._have_state:
            for i, name in enumerate(COMMAND_JOINTS):
                if name in self._state:
                    self._cmd[i] = self._state[name]
            if all(n in self._state for n in COMMAND_JOINTS):
                self._have_state = True

    def _tick(self) -> None:
        if not self._have_state:
            return
        msg = Float64MultiArray()
        msg.data = list(self._cmd)
        self._pub.publish(msg)

    def _validate(self, goal, names: Sequence[str]) -> GoalResponse:
        traj = goal.trajectory
        if not traj.points:
            return GoalResponse.REJECT
        if set(traj.joint_names) != set(names):
            self.get_logger().warn(
                f'Rejecting goal: expected {list(names)}, got {list(traj.joint_names)}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _ordered_points(self, traj, names: Sequence[str]) -> List[JointTrajectoryPoint]:
        lookup = {n: i for i, n in enumerate(traj.joint_names)}
        ordered: List[JointTrajectoryPoint] = []
        for src in traj.points:
            dst = JointTrajectoryPoint()
            dst.positions = [float(src.positions[lookup[n]]) for n in names]
            dst.time_from_start = src.time_from_start
            ordered.append(dst)
        return ordered

    def _apply(self, names: Sequence[str], positions: Sequence[float]) -> None:
        for name, pos in zip(names, positions):
            self._cmd[INDEX[name]] = float(pos)

    def _group_names(self, group: str) -> List[str]:
        if group == 'cam':
            return ['mid_camera_stand_joint']
        return arm_joint_names(group)

    def _execute(self, handle, group: str):
        names = self._group_names(group)
        traj = handle.request.trajectory
        points = self._ordered_points(traj, names)
        final = points[-1]
        duration = _sec(final.time_from_start)
        timeout_s = max(5.0, duration + 8.0)
        start_ns = self.get_clock().now().nanoseconds
        wall_start = time.monotonic()
        wall_cap = max(60.0, 20.0 * timeout_s)
        tol = float(self.get_parameter('goal_tolerance').value)

        while True:
            if handle.is_cancel_requested:
                if all(n in self._state for n in names):
                    self._apply(names, [self._state[n] for n in names])
                handle.canceled()
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)

            elapsed = (self.get_clock().now().nanoseconds - start_ns) * 1e-9
            if elapsed > timeout_s or (time.monotonic() - wall_start) > wall_cap:
                break
            self._apply(names, interpolate_positions(points, elapsed))
            if all(n in self._state for n in names):
                err = max(abs(self._state[n] - float(final.positions[i])) for i, n in enumerate(names))
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = list(names)
                fb.actual.positions = [self._state[n] for n in names]
                fb.desired.positions = list(final.positions)
                handle.publish_feedback(fb)
                if err < tol and elapsed >= duration:
                    handle.succeed()
                    return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
            time.sleep(0.02)

        if all(n in self._state for n in names):
            self._apply(names, [self._state[n] for n in names])
        handle.abort()
        return FollowJointTrajectory.Result(
            error_code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            error_string='Mobile ALOHA joint target timeout',
        )

    def destroy_node(self):
        for server in self._servers:
            server.destroy()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = AlohaActionBridge()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
