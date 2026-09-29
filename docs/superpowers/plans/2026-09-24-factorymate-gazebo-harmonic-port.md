# FactoryMate Gazebo Harmonic Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a runnable ROS 2 Jazzy + Gazebo Harmonic FactoryMate warehouse backend that preserves the dynamic-logistics-warehouse concept, runs independently of Isaac Sim, exposes standard ROS interfaces, and packages as a one-command project ZIP.

**Architecture:** A new `factorymate_gazebo` ament package owns the Harmonic world, launch files, bridge configuration, crowd motion node, and validation. Static warehouse geometry is generated natively in SDF for low GPU/CPU cost; the nine workers are lightweight Gazebo entities controlled through bridged Twist topics. An upstream-asset importer is included so the original Belal/AWS visual models can replace procedural geometry without changing ROS interfaces. Existing FactoryMate ROS-side nodes remain separate.

**Tech Stack:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic / `gz-sim`, `ros_gz_sim`, `ros_gz_bridge`, Python 3, SDF 1.10, RViz2, pytest.

**Spec:** `docs/superpowers/specs/2026-09-24-factorymate-gazebo-harmonic-port-design.md`

## Global Constraints

- Keep the Isaac backend intact; add Gazebo as a separate backend.
- One command must launch the Harmonic backend.
- ROS 2 interfaces must use simulation time and standard topics where practical.
- The baseline warehouse must contain nine dynamic workers.
- Static geometry must use simple collision shapes; visual complexity must not drive physics cost.
- Do not silently substitute a fake production robot when the full FactoryMate model is absent.
- World-only mode must work without the production robot assets.
- Optional upstream Belal/AWS assets may enhance visuals without changing interfaces.

## Review Focus

- Missing Gazebo/ROS binaries must fail with actionable diagnostics rather than silent partial launch.
- World generation must remain deterministic and always produce exactly nine worker entities by default.
- Bridge configuration must not reverse ROS-to-Gazebo vs Gazebo-to-ROS direction for `/cmd_vel`, `/odom`, `/scan`, or `/clock`.
- Paths containing spaces must remain safe in shell launchers and resource setup.
- Disabling GUI or workers must not invalidate SDF or launch-file syntax.

---

### Task 1: Project skeleton and contract tests

**Files:**
- Create: `tests/test_harmonic_project.py`
- Create: `ros2_ws/src/factorymate_gazebo/package.xml`
- Create: `ros2_ws/src/factorymate_gazebo/CMakeLists.txt`

**Interfaces:**
- Produces: package identity `factorymate_gazebo`, installable `launch`, `worlds`, `config`, `rviz`, and `scripts` resources.

- [ ] Write tests asserting package format 3, `ament_cmake`, runtime deps (`ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `rviz2`), and install directives.
- [ ] Run `pytest tests/test_harmonic_project.py -q` and confirm failure because files are absent.
- [ ] Add package metadata and CMake install rules.
- [ ] Re-run the test and confirm pass.

### Task 2: Native Harmonic warehouse world

**Files:**
- Create: `factorymate_gazebo_tools/world_builder.py`
- Create: `ros2_ws/src/factorymate_gazebo/worlds/factorymate_dynamic_warehouse.sdf`
- Create: `tests/test_world.py`

**Interfaces:**
- Produces: `build_world(worker_count: int = 9) -> str` and deterministic SDF world named `factorymate_dynamic_warehouse`.

- [ ] Write tests for SDF 1.10, required Harmonic systems, 36 rack models, pickup/delivery/inspection zones, and exactly nine workers with DiffDrive systems.
- [ ] Run tests and confirm failure because builder/world are absent.
- [ ] Implement the builder with low-cost static rack geometry, floor/walls/lights, pallets/zones, nine colored worker entities, and per-worker DiffDrive plugins.
- [ ] Generate the checked-in SDF.
- [ ] Re-run tests and confirm pass.

### Task 3: ROS-Gazebo bridge and crowd controller

**Files:**
- Create: `ros2_ws/src/factorymate_gazebo/config/bridge.yaml`
- Create: `ros2_ws/src/factorymate_gazebo/scripts/worker_controller.py`
- Create: `tests/test_bridge_and_workers.py`

**Interfaces:**
- Consumes: worker names `worker_01`..`worker_09` from Task 2.
- Produces: bridge mappings for `/clock`, `/cmd_vel`, `/odom`, `/scan`, and nine worker command topics; deterministic waypoint velocity controller.

- [ ] Write tests for bridge topic directions/types and worker route generation.
- [ ] Run tests and confirm failure.
- [ ] Implement YAML and controller logic with configurable worker count and `use_sim_time`.
- [ ] Re-run tests and confirm pass.

### Task 4: Launch, RViz, and one-command startup

**Files:**
- Create: `ros2_ws/src/factorymate_gazebo/launch/factorymate_gazebo.launch.py`
- Create: `ros2_ws/src/factorymate_gazebo/rviz/factorymate_gazebo.rviz`
- Create: `scripts/run_factorymate_gazebo.sh`
- Create: `tests/test_launch_contract.py`

**Interfaces:**
- Consumes: world and bridge files from Tasks 2-3.
- Produces: launch arguments `gui`, `rviz`, `workers`, `robot_model`, `robot_x`, `robot_y`, `robot_yaw`; shell entrypoint.

- [ ] Write tests that parse launch source text and shell syntax, and assert explicit dependency checks for ROS Jazzy, `gz`, and `ros_gz_bridge`.
- [ ] Run tests and confirm failure.
- [ ] Implement launch and RViz configuration; shell launcher must source Jazzy, build workspace when requested, set `GZ_SIM_RESOURCE_PATH`, and launch.
- [ ] Re-run tests and `bash -n` and confirm pass.

### Task 5: Upstream Belal asset migration support

**Files:**
- Create: `tools/import_dynamic_logistics_warehouse.py`
- Create: `scripts/fetch_dynamic_logistics_assets.sh`
- Create: `tests/test_asset_importer.py`

**Interfaces:**
- Produces: `import_assets(source: Path, destination: Path) -> ImportReport` that copies compatible models and rewrites Classic-only package metadata without changing the native fallback world.

- [ ] Write tests using a miniature fixture repository with model.config, model.sdf, mesh/texture files, and a Classic package.xml.
- [ ] Run tests and confirm failure.
- [ ] Implement copy/validation and a fetch helper that clones the public upstream repo on a networked user machine.
- [ ] Re-run tests and confirm pass.

### Task 6: Documentation, compatibility checks, and package artifact

**Files:**
- Create: `README.md`
- Create: `docs/GAZEBO_HARMONIC_MIGRATION.md`
- Create: `tests/test_distribution.py`
- Modify: `docs/REPAIR_STATUS.md`

**Interfaces:**
- Produces: documented one-command workflow and self-contained distributable ZIP.

- [ ] Write distribution tests asserting no absolute build-machine paths, required files present, launcher executable, and upstream license/source attribution included.
- [ ] Run tests and confirm failure.
- [ ] Add documentation, migration notes, troubleshooting, robot-model integration instructions, and attribution.
- [ ] Run full `pytest -q`, XML parsing, Python compile, shell syntax, and ZIP integrity verification.
- [ ] Build `/mnt/data/factorymate-gazebo-harmonic-complete.zip`.
