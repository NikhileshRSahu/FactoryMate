#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_DIR="$ROOT_DIR/upstream/dynamic_logistics_warehouse"
DEST="$ROOT_DIR/ros2_ws/src/factorymate_gazebo"

command -v git >/dev/null || { echo 'ERROR: git is required to fetch upstream assets.' >&2; exit 20; }
command -v python3 >/dev/null || { echo 'ERROR: python3 is required to import upstream assets.' >&2; exit 21; }

mkdir -p "$(dirname "$UPSTREAM_DIR")"
if [[ -d "$UPSTREAM_DIR/.git" ]]; then
  git -C "$UPSTREAM_DIR" fetch --depth 1 origin main
  git -C "$UPSTREAM_DIR" reset --hard origin/main
else
  git clone --depth 1 https://github.com/belal-ibrahim/dynamic_logistics_warehouse.git "$UPSTREAM_DIR"
fi

python3 "$ROOT_DIR/tools/import_dynamic_logistics_warehouse.py" "$UPSTREAM_DIR" "$DEST"
echo 'Upstream warehouse assets imported. Re-run the FactoryMate launcher so colcon installs them.'
