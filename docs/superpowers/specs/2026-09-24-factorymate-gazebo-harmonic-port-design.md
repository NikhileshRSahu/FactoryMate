# FactoryMate Gazebo Harmonic Dynamic Warehouse Port — Design

Date: 2026-09-24

## Goal

Create a lower-GPU FactoryMate simulation backend on Ubuntu 24.04 / ROS 2 Jazzy using Gazebo Harmonic, while preserving the existing FactoryMate ROS/autonomy interfaces and keeping the Isaac Sim backend intact for high-fidelity final demonstrations.

The dynamic warehouse assets and layout from `belal-ibrahim/dynamic_logistics_warehouse` will be reused as source material, but its ROS1 / Gazebo Classic launch path will not be carried forward.

## Success Criteria

1. One command launches the Gazebo Harmonic warehouse and ROS 2 integration.
2. The warehouse contains the original useful logistics assets and nine scripted moving actors, adapted to Harmonic where required.
3. FactoryMate can be spawned into the warehouse without changing higher-level ROS topics unnecessarily.
4. ROS 2 Jazzy receives simulation clock, TF, joint states, odometry, lidar, camera/depth data when those sensors exist in the FactoryMate model.
5. RViz can visualize the robot, TF, lidar, paths, costmaps, humans/obstacles, and navigation state without a custom Isaac USD-to-marker bridge.
6. Nav2 can use the simulation with `use_sim_time:=true`.
7. The design supports later insertion of ORCA/RVO, CBF safety, manipulator/gripper control, and multiple AMRs without coupling those systems to Gazebo internals.
8. Existing Isaac scripts remain available and unchanged except for optional documentation/launcher selection.

## Scope

### In scope

- Add a new Gazebo Harmonic backend alongside Isaac.
- Import/port the dynamic logistics warehouse world and reusable models.
- Replace ROS1/Gazebo Classic launch integration with ROS 2 Python launch and Gazebo Harmonic (`gz sim`).
- Use `ros_gz_bridge` / Gazebo ROS 2 systems instead of a bespoke world-visualization bridge.
- Preserve FactoryMate ROS-facing autonomy interfaces where practical.
- Add launch-time validation and smoke tests that do not require a GPU-rendered GUI.
- Add documentation for GUI and headless launch modes.

### Out of scope for the first port

- Rebuilding the entire FactoryMate robot model if its URDF/USD/meshes are absent from the supplied repair bundle.
- Isaac-level photorealistic rendering.
- Full ORCA/RVO crowd intelligence implementation in the first port.
- Full multi-AMR fleet scheduling in the first port.
- Retraining RL/ACT policies.

These remain integration steps after the backend/world port is structurally valid.

## Existing Context

The supplied FactoryMate archive is a compact repair/integration bundle. It includes Isaac bridge/launcher code, ROS 2 FactoryMate core code, RViz/Nav2 configuration, scripts, and tests, but it does not contain the complete 171 MB robot/world asset set previously described.

The source warehouse repository is a Gazebo simulation package derived from the AWS RoboMaker small warehouse and documents nine waypoint-driven actors plus warehouse props. Its launch file uses ROS1 `gazebo_ros/empty_world.launch`, so the world/assets are the reusable part; the launch/plugin integration must be replaced.

## Architecture

```text
                         FactoryMate
                             |
               +-------------+-------------+
               |                           |
        Gazebo Harmonic                 Isaac Sim
       everyday backend             high-fidelity backend
               |                           |
          ros_gz bridge              Isaac ROS bridge
               |                           |
               +------------ ROS 2 --------+
                             |
             +---------------+----------------+
             |               |                |
            Nav2          ORCA/RVO            CBF
             |               |                |
             +---------------+----------------+
                             |
                            RViz
```

The two backends should expose compatible ROS concepts rather than share simulator-specific code.

## Components

### 1. `factorymate_gazebo`

New ROS 2 package responsible only for Gazebo integration.

Proposed structure:

```text
ros2_ws/src/factorymate_gazebo/
  package.xml
  CMakeLists.txt
  launch/
    warehouse.launch.py
    factorymate_gazebo.launch.py
  worlds/
    dynamic_logistics_warehouse.sdf
  models/
    ...ported source assets...
  config/
    bridge.yaml
  scripts/
    validate_world.py
```

Responsibilities:

- Launch Gazebo Harmonic.
- Resolve model/resource paths.
- Start ROS-Gazebo bridges.
- Spawn FactoryMate if a compatible robot description/model is present.
- Expose clear launch arguments for GUI/headless mode, world, robot pose, and actor enablement.

### 2. Ported Dynamic Warehouse

The Classic `.world` is treated as source data, not executed directly in production.

Porting rules:

- Convert/normalize to an SDF version supported by Harmonic.
- Preserve visual meshes/material references where compatible.
- Prefer simple collision geometry for static racks and props.
- Keep static warehouse structures static.
- Preserve nine actors initially with deterministic scripted trajectories.
- Remove/replace Classic-only plugins.
- Avoid per-object ROS publishers for static scenery.

The first milestone keeps actor behavior scripted so that the world port can be verified independently. ORCA/RVO becomes a later crowd-control layer.

### 3. ROS-Gazebo Bridge

Use `ros_gz_bridge` for standard transport.

Expected interfaces include, where present:

```text
/clock
/tf
/tf_static
/joint_states
/cmd_vel
/odom
/scan
/camera/image_raw
/camera/camera_info
```

Topic naming should be aligned with existing FactoryMate/Nav2 configuration via bridge remaps or launch parameters rather than simulator-specific rewrites in autonomy nodes.

### 4. RViz

RViz remains the autonomy/debug frontend.

It should consume ROS 2 data directly and should not require the Isaac environment marker workaround when running the Gazebo backend. Static warehouse context for navigation should come from the map/costmap; dynamic actors should appear through lidar/costmaps and optional dedicated markers if needed for experiments.

### 5. Navigation and Safety

Nav2, ORCA/RVO, and CBF remain ROS-side systems.

They should not depend on whether Gazebo or Isaac is the active simulator beyond standard inputs such as odometry, TF, scan/point cloud, people/dynamic-obstacle information, and velocity commands.

## Backend Selection

Add a backend-selecting launcher/script without deleting the existing Isaac launch path.

Example intent:

```bash
./scripts/run_factorymate.sh --backend gazebo --stage dynamic
./scripts/run_factorymate.sh --backend isaac  --stage dynamic
```

If changing the existing launcher is too invasive, add `run_factorymate_gazebo.sh` first and unify launch selection only after both are validated.

The safer first implementation is the second option: separate launcher, shared ROS interfaces.

## Performance Design

To keep GPU/CPU load below Isaac:

- Use Ogre2 rendering only when GUI is requested.
- Support `-s` / headless server mode for automated tests.
- Keep warehouse static geometry static.
- Use low-complexity collision proxies rather than visual triangle meshes where feasible.
- Disable unnecessary cameras by launch argument.
- Keep scripted actor count configurable.
- Do not attach ROS plugins to every environment object.
- Use sensor update rates appropriate to the controller rather than maximum render rate.

## Human Actors

Milestone 1:

- Preserve nine source actors and waypoint scripts.
- Verify they move and collide/are sensed as intended.

Milestone 2:

- Replace purely scripted local motion with a ROS-side ORCA/RVO crowd controller.
- Keep goal/route intent separate from local collision avoidance.
- Publish human state in a simulator-independent message/interface for CBF evaluation.

## FactoryMate Robot Integration

The current supplied repair bundle does not include enough robot assets to recreate the full AMR/manipulator visually.

Therefore the port should support two modes:

1. `world_only:=true` — launches and validates the warehouse immediately.
2. `robot_model:=<path>` — spawns the actual FactoryMate model when the complete robot URDF/SDF/meshes are present.

No fake production robot should be silently substituted.

## Error Handling

Launch must fail clearly when:

- `gz` / Harmonic is unavailable.
- `ros_gz_bridge` is unavailable.
- required world/model resources cannot be resolved.
- a requested robot model path is missing.

Optional components (camera, actors, manipulator controllers) may be disabled explicitly, but not silently skipped when requested.

## Verification

### Static tests

- Parse SDF/XML.
- Assert expected warehouse world exists.
- Assert actor count is nine for the baseline port.
- Assert bridge config includes `/clock`, command velocity, odometry, and lidar mappings expected by FactoryMate.
- Assert launch files can be imported by Python.
- Shell syntax check launcher scripts.

### Headless smoke tests

When Gazebo Harmonic is installed:

- Start world headless.
- Verify server remains alive for a fixed smoke window.
- Verify `/clock` advances.
- Verify bridge nodes start.
- If robot asset is present, verify robot entity exists and odometry/TF topics become active.

### GUI acceptance

- Warehouse visually loads.
- Nine actors animate.
- FactoryMate appears at requested spawn pose when complete assets are provided.
- RViz shows TF and sensors.
- Sending velocity commands produces expected base motion.

## Migration Strategy

Phase 1: world-only Harmonic port and bridge skeleton.

Phase 2: actual FactoryMate robot asset spawn and sensors.

Phase 3: Nav2 and current autonomy stack.

Phase 4: ORCA/RVO dynamic crowd behavior and CBF integration.

Phase 5: multiple AMRs and manipulation/gripper validation.

This staged migration avoids repeating the previous failure mode where environment, crowd, navigation, manipulation, and safety were all debugged at once.

## Deliverable

The first deliverable is a new FactoryMate ZIP containing the Gazebo Harmonic backend and ported warehouse integration while preserving the Isaac backend. It will be clearly labeled as either:

- `world-port complete, robot assets required`, if the full FactoryMate robot asset set is still absent, or
- `full Gazebo backend`, if the required robot assets are supplied and pass smoke tests.
