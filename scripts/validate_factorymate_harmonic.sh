#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

python3 -m pytest -q
python3 -m py_compile \
  factorymate_gazebo_tools/world_builder.py \
  tools/import_dynamic_logistics_warehouse.py \
  ros2_ws/src/factorymate_gazebo/launch/factorymate_gazebo.launch.py \
  ros2_ws/src/factorymate_gazebo/scripts/worker_controller.py
bash -n scripts/run_factorymate_gazebo.sh
bash -n scripts/fetch_dynamic_logistics_assets.sh

python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
root = Path('ros2_ws/src/factorymate_gazebo')
ET.parse(root / 'package.xml')
world = ET.parse(root / 'worlds/dynamic_logistics_warehouse_harmonic.sdf').getroot()
w = world.find('world')
assert world.attrib['version'] == '1.10'
assert w is not None and w.attrib['name'] == 'factorymate_dynamic_logistics_warehouse'
assert not (root / 'worlds/factorymate_dynamic_warehouse.sdf').exists()
assert not (root / 'urdf/factorymate.urdf.xacro').exists()
print('XML/SDF structural validation: OK')
PY

if command -v gz >/dev/null 2>&1; then
  gz sdf -k ros2_ws/src/factorymate_gazebo/worlds/dynamic_logistics_warehouse_harmonic.sdf
else
  echo 'INFO: gz not installed in this environment; skipping sdformat runtime validation.'
fi

echo 'FactoryMate Harmonic static validation completed.'
