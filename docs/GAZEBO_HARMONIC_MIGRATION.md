# Dynamic Logistics Warehouse: Gazebo Classic / ROS1 → Gazebo Harmonic / ROS 2 Jazzy

## What is migrated

The valuable warehouse content is simulator data: SDF world structure, model directories, Collada meshes, textures, lights, props, and scripted actors. Those are retained as source material. The obsolete runtime wrapper is replaced.

| Old repository integration | FactoryMate Harmonic integration |
|---|---|
| `catkin` | `ament_cmake` |
| ROS1 `roslaunch` XML | ROS 2 Python launch |
| `gazebo_ros/empty_world.launch` | `ros_gz_sim` / Gazebo Harmonic |
| Gazebo Classic | Gazebo Harmonic |
| `gazebo_ros` / `gazebo_plugins` | `ros_gz` and Harmonic systems |
| ROS1 topic bridge conventions | ROS 2 `ros_gz_bridge` YAML |
| RViz1-era workflow | RViz2 |

## Why the old environment remains useful

`belal-ibrahim/dynamic_logistics_warehouse` is an extensive AWS RoboMaker-style warehouse and includes nine waypoint-driven actors, shelves, pallet jack, desks, trash cans, clutter, and multiple lights. The migration therefore does **not** discard the warehouse because its launch file is ROS1.

## Conversion strategy

`tools/import_dynamic_logistics_warehouse.py` takes a local upstream clone and performs a conservative port:

- model / mesh / texture bytes are copied unchanged;
- the original `warehouse.world` is retained as `dynamic_logistics_warehouse_classic.source.world`;
- the SDF version is normalized to 1.10;
- the world is renamed `factorymate_dynamic_logistics_warehouse`;
- plugins whose filename / name contains `libgazebo` or `gazebo_ros` are removed;
- Harmonic Physics, UserCommands, SceneBroadcaster, and Sensors systems are added;
- upstream `model://...` references are resolved by adding `models/upstream` to `GZ_SIM_RESOURCE_PATH`.

This avoids trying to translate ROS1 plugin APIs line-for-line. Standard SDF actors / trajectories and model content remain available to Harmonic, while simulator-specific ROS integration is replaced.

## Target world and robot

The shipped simulation uses only `dynamic_logistics_warehouse_harmonic.sdf` and the `mobile_aloha_sim` robot descriptions. Generic primitive FactoryMate warehouse SDFs and placeholder URDFs are not included.

Robot-specific completion should be validated against Mobile ALOHA:

1. SDF/URDF parsing;
2. static spawn and TF;
3. base joints and `/cmd_vel`;
4. odometry;
5. lidar/camera topics;
6. arm joint control;
7. gripper attachment/contact behavior;
8. Nav2;
9. CBF / ORCA safety;
10. ACT policy integration;
11. multi-AMR scaling.

## Simulator scope

This workspace is Gazebo Harmonic / ROS 2 Jazzy only. Isaac Sim repair bundles are not included.
