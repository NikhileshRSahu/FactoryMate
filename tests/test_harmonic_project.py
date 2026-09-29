from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ros2_ws/src/factorymate_gazebo'


def test_package_is_ros2_jazzy_harmonic_package():
    package_xml = PKG / 'package.xml'
    assert package_xml.exists()
    root = ET.parse(package_xml).getroot()
    assert root.attrib.get('format') == '3'
    text = package_xml.read_text()
    assert '<buildtool_depend>ament_cmake</buildtool_depend>' in text
    for dep in ['ros_gz_sim', 'ros_gz_bridge', 'robot_state_publisher', 'rviz2']:
        assert f'<exec_depend>{dep}</exec_depend>' in text


def test_cmake_installs_runtime_resources():
    cmake = (PKG / 'CMakeLists.txt').read_text()
    for resource in ['launch', 'worlds', 'config', 'rviz', 'scripts']:
        assert resource in cmake
    assert 'ament_package()' in cmake
