#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT_DIR/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
GUI=true
RVIZ=true
WORKERS=true
BUILD=true
ROBOT_MODEL="$WS/src/mobile_aloha_sim/aloha_new_description/urdf/mobile_aloha_harmonic.urdf.xacro"
ROBOT_X="10.0"
ROBOT_Y="10.0"
ROBOT_YAW="0.0"
UPSTREAM_WORLD=true

usage() {
  cat <<'EOF'
Usage: ./scripts/run_factorymate_gazebo.sh [options]
  --headless                 Gazebo server only
  --no-rviz                  Do not start RViz2
  --no-workers               Leave worker entities stationary
  --no-build                 Reuse an existing colcon build
  --upstream-world           Use dynamic_logistics_warehouse_harmonic.sdf (default)
  --robot-model PATH         Spawn Mobile ALOHA URDF/Xacro (default: mobile_aloha_harmonic.urdf.xacro)
  --robot-x X --robot-y Y    Spawn position
  --robot-yaw RADIANS        Spawn yaw
EOF
}

while (($#)); do
  case "$1" in
    --headless) GUI=false; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --no-workers) WORKERS=false; shift ;;
    --no-build) BUILD=false; shift ;;
    --upstream-world) UPSTREAM_WORLD=true; WORKERS=false; shift ;;
    --robot-model) ROBOT_MODEL="${2:?missing model path}"; shift 2 ;;
    --robot-x) ROBOT_X="${2:?missing x}"; shift 2 ;;
    --robot-y) ROBOT_Y="${2:?missing y}"; shift 2 ;;
    --robot-yaw) ROBOT_YAW="${2:?missing yaw}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS 2 Jazzy setup not found at $ROS_SETUP" >&2
  echo "Install ROS 2 Jazzy for Ubuntu 24.04 before running FactoryMate." >&2
  exit 10
fi
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

command -v ros2 >/dev/null || { echo 'ERROR: ros2 command is unavailable.' >&2; exit 11; }
command -v gz >/dev/null || { echo 'ERROR: Gazebo Harmonic (gz) is unavailable.' >&2; exit 12; }
ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 || { echo 'ERROR: ros_gz_bridge is not installed.' >&2; exit 13; }
ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 || { echo 'ERROR: ros_gz_sim is not installed.' >&2; exit 14; }

if [[ -n "$ROBOT_MODEL" && ! -f "$ROBOT_MODEL" ]]; then
  echo "ERROR: requested robot model does not exist: $ROBOT_MODEL" >&2
  exit 15
fi

cd "$WS"
if [[ "$BUILD" == true ]]; then
  command -v colcon >/dev/null || { echo 'ERROR: colcon is unavailable.' >&2; exit 16; }
  colcon build --symlink-install --packages-select factorymate_gazebo
fi

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo 'ERROR: workspace is not built. Run without --no-build first.' >&2
  exit 17
fi
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

PKG_SHARE="$(ros2 pkg prefix --share factorymate_gazebo)"
export GZ_SIM_RESOURCE_PATH="$PKG_SHARE/models:$PKG_SHARE/models/upstream:${GZ_SIM_RESOURCE_PATH:-}"

# Ensure upstream models directory is always on the resource path so that
# model://aws_robomaker_warehouse_* URIs resolve and textures are found.
UPSTREAM_MODELS="$ROOT_DIR/upstream/dynamic_logistics_warehouse/models"
if [[ -d "$UPSTREAM_MODELS" ]]; then
  export GZ_SIM_RESOURCE_PATH="$UPSTREAM_MODELS:${GZ_SIM_RESOURCE_PATH:-}"
fi

LAUNCH_ARGS=(
  "gui:=$GUI"
  "rviz:=$RVIZ"
  "workers:=$WORKERS"
  "robot_x:=$ROBOT_X"
  "robot_y:=$ROBOT_Y"
  "robot_yaw:=$ROBOT_YAW"
)
if [[ -n "$ROBOT_MODEL" ]]; then
  LAUNCH_ARGS+=("robot_model:=$ROBOT_MODEL")
fi
if [[ "$UPSTREAM_WORLD" == true ]]; then
  UPSTREAM_SDF="$PKG_SHARE/worlds/dynamic_logistics_warehouse_harmonic.sdf"
  if [[ ! -f "$UPSTREAM_SDF" ]]; then
    echo "ERROR: imported upstream world is missing. Run ./scripts/fetch_dynamic_logistics_assets.sh first, then rebuild." >&2
    exit 18
  fi
  LAUNCH_ARGS+=("world:=$UPSTREAM_SDF")
fi

exec ros2 launch factorymate_gazebo factorymate_gazebo.launch.py "${LAUNCH_ARGS[@]}"
