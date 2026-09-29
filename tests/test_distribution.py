from pathlib import Path
import os

ROOT = Path(__file__).resolve().parents[1]


def test_readme_documents_all_supported_run_modes():
    text = (ROOT / 'README.md').read_text()
    assert './scripts/run_factorymate_gazebo.sh' in text
    assert './scripts/fetch_dynamic_logistics_assets.sh' in text
    assert '--upstream-world' in text
    assert '--robot-model' in text
    assert 'ROS 2 Jazzy' in text
    assert 'Gazebo Harmonic' in text


def test_migration_doc_explains_ros1_to_ros2_replacements():
    text = (ROOT / 'docs/GAZEBO_HARMONIC_MIGRATION.md').read_text()
    for old, new in [
        ('catkin', 'ament_cmake'),
        ('roslaunch', 'ROS 2 Python launch'),
        ('gazebo_ros', 'ros_gz'),
        ('Gazebo Classic', 'Gazebo Harmonic'),
    ]:
        assert old in text
        assert new in text


def test_third_party_attribution_is_explicit():
    text = (ROOT / 'THIRD_PARTY.md').read_text()
    assert 'belal-ibrahim/dynamic_logistics_warehouse' in text
    assert 'GPL-2.0' in text
    assert 'Apache-2.0' in text
    assert 'not bundled' in text.lower()


def test_isaac_backend_placeholders_are_removed():
    assert not (ROOT / 'backends').exists()


def test_runnable_scripts_are_executable_and_validate_script_exists():
    for rel in [
        'scripts/run_factorymate_gazebo.sh',
        'scripts/fetch_dynamic_logistics_assets.sh',
        'scripts/validate_factorymate_harmonic.sh',
    ]:
        path = ROOT / rel
        assert path.exists()
        assert os.access(path, os.X_OK)


def test_harmonic_runtime_files_have_no_build_machine_paths():
    banned = ['/mnt/data/', '/home/chirag/', '/home/oai/']
    roots = [ROOT / 'ros2_ws/src/factorymate_gazebo', ROOT / 'scripts', ROOT / 'tools', ROOT / 'factorymate_gazebo_tools']
    for base in roots:
        for path in base.rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:
                continue
            try:
                text = path.read_text()
            except UnicodeDecodeError:
                continue
            for value in banned:
                assert value not in text, f'{path} contains build-machine path {value}'
