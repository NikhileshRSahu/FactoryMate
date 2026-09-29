from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ros2_ws/src/factorymate_gazebo'
LAUNCH = PKG / 'launch/factorymate_gazebo.launch.py'
RUNNER = ROOT / 'scripts/run_factorymate_gazebo.sh'


def test_launch_exposes_required_arguments_and_components():
    text = LAUNCH.read_text()
    for arg in ['gui', 'rviz', 'workers', 'robot_model', 'robot_x', 'robot_y', 'robot_yaw']:
        assert f"DeclareLaunchArgument('{arg}'" in text or f'DeclareLaunchArgument("{arg}"' in text
    assert 'ros_gz_sim' in text
    assert 'parameter_bridge' in text
    assert 'worker_controller.py' in text
    assert 'rviz2' in text
    assert "executable='create'" in text or 'executable="create"' in text


def test_runner_has_actionable_dependency_checks():
    text = RUNNER.read_text()
    assert '/opt/ros/jazzy/setup.bash' in text
    assert 'command -v gz' in text
    assert 'command -v ros2' in text
    assert 'ros2 pkg prefix ros_gz_bridge' in text
    assert 'colcon build' in text
    assert 'GZ_SIM_RESOURCE_PATH' in text
    assert 'factorymate_gazebo.launch.py' in text


def test_runner_shell_syntax_is_valid():
    result = subprocess.run(['bash', '-n', str(RUNNER)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_rviz_config_targets_factorymate_topics():
    text = (PKG / 'rviz/factorymate_gazebo.rviz').read_text()
    assert 'Fixed Frame: odom' in text
    assert '/factorymate/scan' in text
    assert '/plan' in text

def test_launch_can_select_imported_upstream_world():
    launch_text = LAUNCH.read_text()
    runner_text = RUNNER.read_text()
    assert "DeclareLaunchArgument('world'" in launch_text or 'DeclareLaunchArgument("world"' in launch_text
    assert "LaunchConfiguration('world')" in launch_text or 'LaunchConfiguration("world")' in launch_text
    assert 'dynamic_logistics_warehouse_harmonic.sdf' in launch_text
    assert '--upstream-world' in runner_text
    assert 'dynamic_logistics_warehouse_harmonic.sdf' in runner_text
    assert 'models/upstream' in runner_text
    assert 'mobile_aloha_harmonic.urdf.xacro' in runner_text
    assert 'factorymate.urdf.xacro' not in runner_text

def test_upstream_world_uses_its_native_actor_trajectories():
    text = RUNNER.read_text()
    assert '--upstream-world) UPSTREAM_WORLD=true; WORKERS=false;' in text
