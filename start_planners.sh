#!/usr/bin/env bash
set -e

ROOT="/home/radhe/factorymate-gazebo-harmonic-complete"
PARAMS="${ROOT}/nav2_params.yaml"

# Controller / recovery cmd_vel is intercepted by collision_monitor, which
# publishes the final Twist on /mobile_aloha/cmd_vel (Gazebo DiffDrive).
REMAPS=(
  -r odom:=/mobile_aloha/odom
  -r cmd_vel:=/cmd_vel_nav
)

echo "Starting Nav2 Planner, Controller, Behaviors, BT Navigator, and Collision Monitor..."
ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=true --params-file "${PARAMS}" > planner.log 2>&1 &
ros2 run nav2_controller controller_server --ros-args -p use_sim_time:=true --params-file "${PARAMS}" "${REMAPS[@]}" > controller.log 2>&1 &
ros2 run nav2_behaviors behavior_server --ros-args -p use_sim_time:=true --params-file "${PARAMS}" "${REMAPS[@]}" > behavior.log 2>&1 &
ros2 run nav2_bt_navigator bt_navigator --ros-args -p use_sim_time:=true --params-file "${PARAMS}" -r odom:=/mobile_aloha/odom > bt_navigator.log 2>&1 &
ros2 run nav2_collision_monitor collision_monitor --ros-args -p use_sim_time:=true --params-file "${PARAMS}" > collision_monitor.log 2>&1 &

for node in planner_server controller_server behavior_server bt_navigator collision_monitor; do
    echo "Waiting for $node..."
    while ! ros2 lifecycle get /$node >/dev/null 2>&1; do sleep 0.5; done
    ros2 lifecycle set /$node configure
    ros2 lifecycle set /$node activate
    while [ "$(ros2 lifecycle get /$node)" != "active [3]" ]; do sleep 0.5; done
    echo "$node is ACTIVE!"
done

echo "ALL NAV2 NODES ACTIVE!"
echo "Velocity output: /mobile_aloha/cmd_vel   Odometry: /mobile_aloha/odom   Scan: /mobile_aloha/scan"
