from pathlib import Path
import importlib.util
import math

import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ros2_ws/src/factorymate_gazebo'


def _load_bridge():
    return yaml.safe_load((PKG / 'config/bridge.yaml').read_text())


def test_core_bridge_directions_and_types():
    entries = _load_bridge()
    by_ros = {e['ros_topic_name']: e for e in entries}
    assert by_ros['/clock']['direction'] == 'GZ_TO_ROS'
    assert by_ros['/clock']['ros_type_name'] == 'rosgraph_msgs/msg/Clock'
    assert by_ros['/factorymate/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert by_ros['/factorymate/cmd_vel']['gz_type_name'] == 'gz.msgs.Twist'
    assert by_ros['/factorymate/odom']['direction'] == 'GZ_TO_ROS'
    assert by_ros['/factorymate/odom']['ros_type_name'] == 'nav_msgs/msg/Odometry'
    assert by_ros['/factorymate/scan']['direction'] == 'GZ_TO_ROS'
    assert by_ros['/factorymate/scan']['ros_type_name'] == 'sensor_msgs/msg/LaserScan'


def test_all_worker_command_topics_bridge_ros_to_gz():
    entries = _load_bridge()
    worker = [e for e in entries if e['ros_topic_name'].startswith('/workers/') and e['ros_topic_name'].endswith('/cmd_vel')]
    assert len(worker) == 9
    expected = {f'/workers/worker_{i:02d}/cmd_vel' for i in range(1, 10)}
    assert {e['ros_topic_name'] for e in worker} == expected
    assert all(e['direction'] == 'ROS_TO_GZ' for e in worker)
    assert all(e['gz_type_name'] == 'gz.msgs.Twist' for e in worker)


def _load_worker_module():
    path = PKG / 'scripts/worker_controller.py'
    spec = importlib.util.spec_from_file_location('worker_controller', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_worker_routes_are_deterministic_closed_loops():
    mod = _load_worker_module()
    routes1 = mod.worker_routes(9)
    routes2 = mod.worker_routes(9)
    assert routes1 == routes2
    assert set(routes1) == {f'worker_{i:02d}' for i in range(1, 10)}
    for points in routes1.values():
        assert len(points) >= 4
        assert points[0] == points[-1]


def test_unicycle_command_turns_toward_target_and_respects_limits():
    mod = _load_worker_module()
    linear, angular = mod.unicycle_command((0.0, 0.0, 0.0), (0.0, 2.0), max_linear=0.8, max_angular=1.2)
    assert 0.0 <= linear <= 0.8
    assert 0.0 < angular <= 1.2
    assert math.isclose(angular, 1.2, rel_tol=1e-6)
    linear2, angular2 = mod.unicycle_command((0.0, 0.0, 0.0), (3.0, 0.0), max_linear=0.8, max_angular=1.2)
    assert math.isclose(linear2, 0.8, rel_tol=1e-6)
    assert math.isclose(angular2, 0.0, abs_tol=1e-6)
