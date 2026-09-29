from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('factorymate_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    default_world = os.path.join(pkg_share, 'worlds', 'dynamic_logistics_warehouse_harmonic.sdf')
    bridge = os.path.join(pkg_share, 'config', 'bridge.yaml')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'factorymate_gazebo.rviz')

    # The upstream AWS warehouse models live here; GZ_SIM_RESOURCE_PATH must
    # include this directory so that model:// URIs in the world SDF resolve.
    upstream_models_dir = os.path.join(pkg_share, 'models', 'upstream')
    existing_gz_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource = upstream_models_dir + ((':' + existing_gz_resource) if existing_gz_resource else '')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    workers = LaunchConfiguration('workers')
    robot_model = LaunchConfiguration('robot_model')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    gz_launch = os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')

    has_robot = PythonExpression(["'", robot_model, "' != ''"])

    robot_description = ParameterValue(Command(['xacro ', robot_model]), value_type=str)

    return LaunchDescription([
        # Ensure Gazebo can resolve model:// URIs to the upstream AWS warehouse models
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_resource),

        DeclareLaunchArgument('world', default_value=default_world, description='Absolute SDF world path'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo graphical client'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2 engineering view'),
        DeclareLaunchArgument('workers', default_value='true', description='Drive the nine dynamic workers'),
        DeclareLaunchArgument('robot_model', default_value='', description='Optional absolute URDF/Xacro path for Mobile ALOHA'),
        DeclareLaunchArgument('robot_x', default_value='-8.0'),
        DeclareLaunchArgument('robot_y', default_value='-3.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -v 3 ', world]}.items(),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -s -v 3 ', world]}.items(),
            condition=UnlessCondition(gui),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='factorymate_ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge}],
        ),

        Node(
            package='factorymate_gazebo',
            executable='worker_controller.py',
            name='factorymate_worker_controller',
            output='screen',
            parameters=[{'use_sim_time': True, 'worker_count': 9}],
            condition=IfCondition(workers),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='factorymate',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            condition=IfCondition(has_robot),
        ),

        TimerAction(period=3.0, condition=IfCondition(has_robot), actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_factorymate',
                output='screen',
                arguments=[
                    '-name', 'factorymate',
                    '-topic', '/factorymate/robot_description',
                    '-x', robot_x,
                    '-y', robot_y,
                    '-z', '0.20',
                    '-Y', robot_yaw,
                ],
            )
        ]),

        TimerAction(period=6.0, condition=IfCondition(has_robot), actions=[
            Node(
                package='controller_manager', executable='spawner', output='screen',
                arguments=['joint_state_broadcaster', '--controller-manager', '/factorymate/controller_manager'],
            )
        ]),

        TimerAction(period=6.8, condition=IfCondition(has_robot), actions=[
            Node(
                package='controller_manager', executable='spawner', output='screen',
                arguments=['diff_drive_controller', '--controller-manager', '/factorymate/controller_manager'],
            )
        ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])
