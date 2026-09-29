#!/usr/bin/env python3
"""Convert a Gazebo D435 depth image into a 2D LaserScan for Nav2.

Default source is the torso mid-left RGB-D camera (stable while the arms move).
Wrist cameras (fl_camera) are a poor Nav2 source because they rotate with the arm.

Equivalent depthimage_to_laserscan launch (if that package is installed):

  ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args \\
    -r depth:=/mobile_aloha/mid_l_camera/depth \\
    -r depth_camera_info:=/mobile_aloha/mid_l_camera/camera_info \\
    -r scan:=/mobile_aloha/scan \\
    -p output_frame:=mid_l_camera_link -p use_sim_time:=true
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class DepthToLaserScan(Node):
    def __init__(self) -> None:
        super().__init__('mobile_aloha_depth_to_laserscan')
        self.declare_parameter('depth_topic', '/mobile_aloha/mid_l_camera/depth')
        self.declare_parameter('camera_info_topic', '/mobile_aloha/mid_l_camera/camera_info')
        self.declare_parameter('scan_topic', '/mobile_aloha/scan')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('scan_height', 40)
        # self.declare_parameter('use_sim_time', True)

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self._info: Optional[CameraInfo] = None
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self._pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self.get_logger().info(
            f'Depth {depth_topic} + {info_topic} -> LaserScan {scan_topic}'
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        h, w = msg.height, msg.width
        encoding = (msg.encoding or '').lower()
        if encoding in ('32fc1', '32fc'):
            arr = np.frombuffer(msg.data, dtype=np.float32)
        elif encoding in ('16uc1', 'mono16'):
            arr = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) * 0.001
        elif encoding in ('8uc1', 'mono8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float32)
        else:
            # Gazebo Harmonic depth is typically 32FC1; fall back by step.
            if msg.step >= w * 4:
                arr = np.frombuffer(msg.data, dtype=np.float32)
            elif msg.step >= w * 2:
                arr = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) * 0.001
            else:
                self.get_logger().warn(f'Unsupported depth encoding {msg.encoding!r}', throttle_duration_sec=5.0)
                return None
        if arr.size < h * w:
            return None
        return arr[: h * w].reshape(h, w)

    def _on_depth(self, msg: Image) -> None:
        info = self._info
        if info is None or info.k[0] == 0.0:
            return
        depth = self._decode_depth(msg)
        if depth is None:
            return

        h, w = depth.shape
        fx = float(info.k[0])
        cx = float(info.k[2])
        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        band = max(1, int(self.get_parameter('scan_height').value) // 2)
        row0 = max(0, h // 2 - band)
        row1 = min(h, h // 2 + band)
        strip = depth[row0:row1, :]
        strip = np.where(np.isfinite(strip) & (strip > 0.0), strip, np.inf)
        z = np.min(strip, axis=0)

        cols = np.arange(w, dtype=np.float64)
        x = (cols - cx) / fx
        ranges = np.sqrt(1.0 + x * x) * z
        ranges = np.where((ranges >= range_min) & (ranges <= range_max), ranges, float('inf'))

        scan = LaserScan()
        scan.header = msg.header
        output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        if output_frame:
            scan.header.frame_id = output_frame
        elif not scan.header.frame_id:
            scan.header.frame_id = 'mid_l_camera_link'
        scan.angle_min = math.atan2(0 - cx, fx)
        scan.angle_max = math.atan2((w - 1) - cx, fx)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / max(w - 1, 1)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        self._pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = DepthToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
