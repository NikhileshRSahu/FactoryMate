#!/usr/bin/env bash
set -e

# 1. Kill stale processes
echo "Killing stale processes..."
pkill -f "gz sim" || true
pkill -f "ruby.*gz" || true
pkill -f "rviz2" || true
pkill -f "robot_state_publisher" || true
pkill -f "map_server" || true
pkill -f "amcl" || true
pkill -f "controller_server" || true
pkill -f "planner_server" || true
pkill -f "behavior_server" || true
pkill -f "bt_navigator" || true
pkill -f "collision_monitor" || true
pkill -f "lifecycle_manager" || true
pkill -f "component_container" || true
sleep 3

# 2. Start Gazebo
echo "Starting Gazebo and FactoryMate..."
/home/radhe/factorymate-gazebo-harmonic-complete/scripts/run_factorymate_gazebo.sh --upstream-world --no-rviz > gazebo.log 2>&1 &

# 3. Wait for /clock
echo "Waiting for /clock..."
while ! timeout 5 ros2 topic echo /clock --once >/dev/null 2>&1; do
    sleep 0.5
done
echo "/clock is active!"

# 4. Wait for /factorymate/scan
echo "Waiting for /factorymate/scan..."
while ! timeout 5 ros2 topic echo /factorymate/scan --once >/dev/null 2>&1; do
    sleep 0.5
done
echo "/factorymate/scan is active!"

# 5. Map Server
echo "Starting Map Server..."
ros2 run nav2_map_server map_server \
    --ros-args -p yaml_filename:=/home/radhe/factorymate-gazebo-harmonic-complete/upstream/dynamic_logistics_warehouse/maps/005/map.yaml \
    -p use_sim_time:=true > map_server.log 2>&1 &

echo "Waiting for map_server to be available..."
while ! ros2 lifecycle get /map_server >/dev/null 2>&1; do
    sleep 0.5
done

echo "Configuring map_server..."
ros2 lifecycle set /map_server configure
echo "Activating map_server..."
ros2 lifecycle set /map_server activate

while [ "$(ros2 lifecycle get /map_server)" != "active [3]" ]; do
    sleep 0.5
done
echo "map_server is ACTIVE!"

# 6. AMCL
echo "Starting AMCL..."
ros2 run nav2_amcl amcl \
    --ros-args -p use_sim_time:=true --params-file /home/radhe/factorymate-gazebo-harmonic-complete/nav2_params.yaml > amcl.log 2>&1 &

echo "Waiting for amcl to be available..."
while ! ros2 lifecycle get /amcl >/dev/null 2>&1; do
    sleep 0.5
done

echo "Configuring AMCL..."
ros2 lifecycle set /amcl configure
echo "Activating AMCL..."
ros2 lifecycle set /amcl activate

while [ "$(ros2 lifecycle get /amcl)" != "active [3]" ]; do
    sleep 0.5
done
echo "AMCL is ACTIVE!"

# 7. Set initial pose (AMCL handles set_initial_pose from params now, but we can also publish it to be safe)
echo "Setting initial pose..."
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -8.0, y: -3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

# 8. Verify map -> odom
echo "Waiting for map -> odom TF..."
while ! timeout 2 ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1; do
    sleep 0.5
done
echo "map -> odom is active!"

# 9. Planner / Controller / Behavior / BT Navigator
echo "Starting Nav2 Planner, Controller, Behaviors, BT Navigator, and Collision Monitor..."
ros2 run nav2_planner planner_server --ros-args -p use_sim_time:=true --params-file /home/radhe/factorymate-gazebo-harmonic-complete/nav2_params.yaml > planner.log 2>&1 &
ros2 run nav2_controller controller_server --ros-args -p use_sim_time:=true --params-file /home/radhe/factorymate-gazebo-harmonic-complete/nav2_params.yaml > controller.log 2>&1 &
ros2 run nav2_behaviors behavior_server --ros-args -p use_sim_time:=true --params-file /home/radhe/factorymate-gazebo-harmonic-complete/nav2_params.yaml > behavior.log 2>&1 &
ros2 run nav2_bt_navigator bt_navigator --ros-args -p use_sim_time:=true --params-file /home/radhe/factorymate-gazebo-harmonic-complete/nav2_params.yaml > bt_navigator.log 2>&1 &
ros2 run nav2_collision_monitor collision_monitor --ros-args -p use_sim_time:=true --params-file /home/radhe/factorymate-gazebo-harmonic-complete/nav2_params.yaml > collision_monitor.log 2>&1 &

for node in planner_server controller_server behavior_server bt_navigator collision_monitor; do
    echo "Waiting for $node..."
    while ! ros2 lifecycle get /$node >/dev/null 2>&1; do sleep 0.5; done
    ros2 lifecycle set /$node configure
    ros2 lifecycle set /$node activate
    while [ "$(ros2 lifecycle get /$node)" != "active [3]" ]; do sleep 0.5; done
    echo "$node is ACTIVE!"
done

echo "ALL NAV2 NODES ACTIVE!"
