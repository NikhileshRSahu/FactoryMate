from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('factorymate_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    aloha_share = get_package_share_directory('aloha_new_description')
    tracer_share = get_package_share_directory('tracer2_description')
    realsense_share = get_package_share_directory('realsense2_description')

    default_world = os.path.join(pkg_share, 'worlds', 'dynamic_logistics_warehouse_harmonic.sdf')
    default_xacro = os.path.join(aloha_share, 'urdf', 'mobile_aloha_harmonic.urdf.xacro')
    bridge = os.path.join(pkg_share, 'config', 'mobile_aloha_bridge.yaml')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'factorymate_gazebo.rviz')

    # Gazebo Harmonic resolves package://mesh URIs by looking for
    # <pkg_name>/meshes/... under each GZ_SIM_RESOURCE_PATH entry.
    # Non-merged colcon installs put each package in its own share parent.
    resource_dirs = [
        os.path.dirname(aloha_share),
        os.path.dirname(tracer_share),
        os.path.dirname(realsense_share),
        os.path.join(pkg_share, 'models', 'upstream'),
        os.path.join(pkg_share, 'models'),
    ]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource = ':'.join([d for d in resource_dirs if d] + ([existing] if existing else []))

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    robot_model = LaunchConfiguration('robot_model')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    run_mission = LaunchConfiguration('run_mission')

    gz_launch = os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
    robot_description = ParameterValue(Command(['xacro ', robot_model]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource),

        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute SDF world path'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('robot_model', default_value=default_xacro,
                              description='Absolute path to the Mobile ALOHA xacro'),
        DeclareLaunchArgument('robot_x', default_value='10.0'),
        DeclareLaunchArgument('robot_y', default_value='10.0'),
        DeclareLaunchArgument('robot_z', default_value='0.1'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('run_mission', default_value='false',
                              description='Start the odom pick-and-place orchestrator'),

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
            name='mobile_aloha_ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge, 'use_sim_time': True}],
        ),

        # Gazebo Harmonic depth is R_FLOAT32; ros_gz_bridge parameter_bridge
        # drops it. ros_gz_image converts FLOAT32 -> sensor_msgs/Image 32FC1.
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='mobile_aloha_depth_image_bridge',
            output='screen',
            arguments=['/model/mobile_aloha/mid_l_camera/depth_image'],
            remappings=[
                (
                    '/model/mobile_aloha/mid_l_camera/depth_image',
                    '/mobile_aloha/mid_l_camera/depth',
                ),
            ],
            parameters=[{'use_sim_time': True}],
        ),

        # World-frame teleport for the demo crate (grasp attach).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='mobile_aloha_set_pose_bridge',
            output='screen',
            arguments=[
                '/world/factorymate_dynamic_logistics_warehouse/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            ],
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='mobile_aloha',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            remappings=[('joint_states', '/mobile_aloha/joint_states')],
        ),

        TimerAction(period=4.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_mobile_aloha',
                output='screen',
                arguments=[
                    '-name', 'mobile_aloha',
                    '-topic', '/mobile_aloha/robot_description',
                    '-x', robot_x,
                    '-y', robot_y,
                    '-z', robot_z,
                    '-Y', robot_yaw,
                ],
            )
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_pick_crate',
                output='screen',
                arguments=[
                    '-name', 'pick_crate',
                    '-file', os.path.join(pkg_share, 'models', 'pick_crate', 'model.sdf'),
                    '-x', '11.35',
                    '-y', robot_y,
                    '-z', '0.08',
                ],
            )
        ]),

        TimerAction(period=8.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['arm_position_controller', '--controller-manager', '/mobile_aloha/controller_manager'],
            )
        ]),

        Node(
            package='factorymate_gazebo',
            executable='depth_to_laserscan.py',
            name='mobile_aloha_depth_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'depth_topic': '/mobile_aloha/mid_l_camera/depth',
                'camera_info_topic': '/mobile_aloha/mid_l_camera/camera_info',
                'scan_topic': '/mobile_aloha/scan',
            }],
        ),

        TimerAction(period=9.0, actions=[
            Node(
                package='factorymate_gazebo',
                executable='aloha_action_bridge.py',
                name='aloha_action_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]),

        Node(
            package='factorymate_gazebo',
            executable='grasp_attach.py',
            name='grasp_attach',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'spawn_x': 10.0,
                'spawn_y': 10.0,
            }],
        ),

        TimerAction(period=14.0, actions=[
            Node(
                package='factorymate_gazebo',
                executable='aloha_task_orchestrator.py',
                name='aloha_task_orchestrator',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(run_mission),
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
