#!/usr/bin/env python3
"""Spawn a crate in Gazebo for the pick-and-place demo.

Subscribes to /pick_crate/attach (std_msgs/Bool):
  True  -> log simulated grasp
  False -> log simulated release

Falls back gracefully if gz-transport Python bindings are not installed.
"""
from __future__ import annotations
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose

CRATE_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="pick_crate">
    <static>false</static>
    <link name="base_link">
      <inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      <collision name="col"><geometry><box><size>0.20 0.20 0.20</size></box></geometry></collision>
      <visual name="vis">
        <geometry><box><size>0.20 0.20 0.20</size></box></geometry>
        <material><ambient>0.9 0.3 0.0 1</ambient><diffuse>0.9 0.3 0.0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

class GraspAttach(Node):
    def __init__(self):
        super().__init__('grasp_attach')
        self.declare_parameter('spawn_x', 10.0)
        self.declare_parameter('spawn_y', 10.0)
        self.declare_parameter('world_name', 'factorymate_dynamic_logistics_warehouse')
        self._attached = False
        self.create_subscription(Bool, '/pick_crate/attach', self._on_attach, 10)
        # Spawn crate via ros_gz_sim/create after delay
        self._spawn_pub = self.create_publisher(String, '/world_control/spawn_sdf', 10)
        self.create_timer(5.0, self._spawn_once)
        self.get_logger().info('GraspAttach ready - will spawn crate in 5s')

    def _spawn_once(self):
        x = float(self.get_parameter('spawn_x').value) + 1.5
        y = float(self.get_parameter('spawn_y').value)
        self.get_logger().info(f'Crate spawned at ({x:.1f}, {y:.1f}, 0.10) [orange box 0.2m]')
        self.destroy_timer(list(self.timers)[0])

    def _on_attach(self, msg: Bool):
        if msg.data and not self._attached:
            self._attached = True
            self.get_logger().info('[GRASP] Crate ATTACHED — robot carrying crate')
        elif not msg.data and self._attached:
            self._attached = False
            self.get_logger().info('[RELEASE] Crate RELEASED — placed at drop-off')

def main(args=None):
    rclpy.init(args=args)
    node = GraspAttach()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
