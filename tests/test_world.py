from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from factorymate_gazebo_tools.world_builder import build_world  # noqa: E402


def _parse(xml_text: str):
    return ET.fromstring(xml_text)


def test_world_contract_and_systems():
    root = _parse(build_world())
    assert root.tag == 'sdf'
    assert root.attrib['version'] == '1.10'
    world = root.find('world')
    assert world is not None
    assert world.attrib['name'] == 'factorymate_dynamic_warehouse'
    filenames = {p.attrib.get('filename') for p in world.findall('plugin')}
    assert 'gz-sim-physics-system' in filenames
    assert 'gz-sim-user-commands-system' in filenames
    assert 'gz-sim-scene-broadcaster-system' in filenames
    assert 'gz-sim-sensors-system' in filenames


def test_world_has_expected_factory_layout():
    world = _parse(build_world()).find('world')
    models = [m.attrib['name'] for m in world.findall('model')]
    assert len([n for n in models if n.startswith('shelf_r')]) == 36
    assert {'pickup_pallet', 'delivery_cell', 'inspection_cell'} <= set(models)
    assert {'north_wall', 'south_wall', 'east_wall', 'west_wall'} <= set(models)


def test_world_has_nine_dynamic_workers_with_diffdrive():
    world = _parse(build_world()).find('world')
    workers = [m for m in world.findall('model') if m.attrib['name'].startswith('worker_')]
    assert len(workers) == 9
    assert [m.attrib['name'] for m in workers] == [f'worker_{i:02d}' for i in range(1, 10)]
    for worker in workers:
        assert worker.find('static') is None or worker.findtext('static') == 'false'
        plugins = worker.findall('plugin')
        diff = [p for p in plugins if p.attrib.get('filename') == 'gz-sim-diff-drive-system']
        assert len(diff) == 1
        assert diff[0].findtext('topic') == f"/model/{worker.attrib['name']}/cmd_vel"


def test_worker_count_is_configurable_and_validated():
    world = _parse(build_world(worker_count=3)).find('world')
    workers = [m for m in world.findall('model') if m.attrib['name'].startswith('worker_')]
    assert len(workers) == 3
    try:
        build_world(worker_count=-1)
    except ValueError as exc:
        assert 'worker_count' in str(exc)
    else:
        raise AssertionError('negative worker_count must fail')


def test_generic_factorymate_world_placeholder_is_removed():
    worlds = ROOT / 'ros2_ws/src/factorymate_gazebo/worlds'
    assert not (worlds / 'factorymate_dynamic_warehouse.sdf').exists()
    assert not (worlds / 'factorymate_dynamic_warehouse_fixed.sdf').exists()
    assert (worlds / 'dynamic_logistics_warehouse_harmonic.sdf').exists()
