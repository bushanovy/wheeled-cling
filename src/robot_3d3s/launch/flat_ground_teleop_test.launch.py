#!/usr/bin/env python3
"""
Ground -> flat wall controlled teleop test.

This launch uses my_world.sdf with the flat_steel_panel active, spawns the
robot on the ground facing +X, disables the analytic C++ adhesion plugin, and
starts Python contact-only adhesion plus the climb safety controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_name = 'default'
    world_path = os.path.join(pkg, 'worlds', 'my_world.sdf')
    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    default_force_table = os.path.join(pkg, 'config', 'kmw100_force_table.csv')

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        (existing_gz_path + ':' + models_path) if existing_gz_path else models_path
    )
    existing_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
        (existing_plugin_path + ':' + plugins_path)
        if existing_plugin_path else plugins_path
    )

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)
    robot_desc = robot_desc.replace(
        '<enabled>true</enabled>',
        '<enabled>false</enabled>')

    run_sim = LaunchConfiguration('run_sim')
    auto_step_startup = LaunchConfiguration('auto_step_startup')
    startup_steps = LaunchConfiguration('startup_steps')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    start_controllers = LaunchConfiguration('start_controllers')
    start_adhesion = LaunchConfiguration('start_adhesion')
    use_climb_controller = LaunchConfiguration('use_climb_controller')
    force_table_csv = LaunchConfiguration('force_table_csv')
    wall_thickness_mm = LaunchConfiguration('wall_thickness_mm')
    adhesion_force = LaunchConfiguration('adhesion_force')
    adhesion_rate = LaunchConfiguration('adhesion_rate')
    gap_reference_mm = LaunchConfiguration('gap_reference_mm')
    safe_tilt_deg = LaunchConfiguration('safe_tilt_deg')
    cutoff_tilt_deg = LaunchConfiguration('cutoff_tilt_deg')

    gz_args = PythonExpression([
        "'-r -v 4 ", world_path, "' if '", run_sim,
        "' == 'true' else '-v 4 ", world_path, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_3d3s',
            '-string', robot_desc,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{world_name}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_3d3s/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/robot_3d3s/wheel_1_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_3_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen',
    )

    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    climb_controller = Node(
        package='robot_3d3s',
        executable='climb_surface_controller.py',
        name='climb_surface_controller',
        condition=IfCondition(use_climb_controller),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cmd_in_topic': '/climb_controller/cmd_vel_in',
            'cmd_out_topic': '/swerve_controller/cmd_vel',
            'status_topic': '/robot_3d3s/adhesion_status',
            'world_frame': 'world',
            'base_frame': 'base_link',
            'min_attached_wheels': 2,
            'max_safe_gap_mm': 30.0,
            'status_timeout_s': 1.0,
            'tf_timeout_s': 0.3,
            'cmd_timeout_s': 2.0,
            'wall_confirm_s': 0.3,
            'capture_speed_limit_mps': 0.04,
            'climb_speed_limit_mps': 0.08,
        }],
    )

    adhesion = Node(
        package='robot_3d3s',
        executable='magnetic_adhesion.py',
        name='magnetic_adhesion',
        condition=IfCondition(start_adhesion),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': 'plane',
            'adhesion_model': 'lookup',
            'force_n': ParameterValue(adhesion_force, value_type=float),
            'min_force_n': 0.0,
            'gap_reference_mm': ParameterValue(gap_reference_mm, value_type=float),
            'safe_tilt_deg': ParameterValue(safe_tilt_deg, value_type=float),
            'cutoff_tilt_deg': ParameterValue(cutoff_tilt_deg, value_type=float),
            'force_table_csv': force_table_csv,
            'wall_thickness_mm': ParameterValue(wall_thickness_mm, value_type=float),
            'wheel_radius_m': 0.05,
            'plane_point_x': 0.95,
            'plane_point_y': 0.0,
            'plane_point_z': 0.0,
            'normal_x': -1.0,
            'normal_y': 0.0,
            'normal_z': 0.0,
            'rate_hz': ParameterValue(adhesion_rate, value_type=float),
            'contact_only': True,
            'contact_timeout_s': 0.25,
            'world_name': world_name,
            'gazebo_pose_topic': f'/world/{world_name}/pose/info',
            'status_topic': '/robot_3d3s/adhesion_status',
            'status_period_s': 0.05,
            'service_timeout_ms': 1000,
            'process_timeout_s': 1.0,
            'diagnostic_period_s': 1.0,
        }],
    )

    def spawner(name, delay):
        return TimerAction(
            period=delay,
            condition=IfCondition(start_controllers),
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name, '--controller-manager-timeout', '30'],
                output='screen',
            )],
        )

    step_startup = TimerAction(
        period=11.0,
        condition=IfCondition(auto_step_startup),
        actions=[
            LogInfo(msg=[
                'Stepping paused Gazebo for ', startup_steps,
                ' iteration(s) so controllers and contact sensors initialize.'
            ]),
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', f'/world/{world_name}/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', ['multi_step: ', startup_steps],
                ],
                output='screen',
            ),
        ],
    )

    ready = TimerAction(
        period=15.0,
        actions=[LogInfo(msg=[
            '\nFLAT GROUND TELEOP TEST: my_world.sdf should show ground_plane, ',
            'flat_steel_panel, sun, and robot_3d3s only. The robot starts on ',
            'the ground facing +X. Press Play, then drive forward toward the ',
            'wall. Teleop:\n',
            'ros2 run robot_3d3s teleop_keyboard_swerve.py --ros-args ',
            '-p cmd_topic:=/climb_controller/cmd_vel_in\n',
        ])],
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_sim', default_value='false',
                              description='Start Gazebo running. Set false to start paused.'),
        DeclareLaunchArgument('auto_step_startup', default_value='true',
                              description='Step paused Gazebo briefly so ros2_control activates.'),
        DeclareLaunchArgument('startup_steps', default_value='50',
                              description='Number of paused startup physics steps.'),
        DeclareLaunchArgument('start_controllers', default_value='true',
                              description='Start joint_state_broadcaster and swerve_controller.'),
        DeclareLaunchArgument('start_adhesion', default_value='true',
                              description='Start contact-only Python adhesion.'),
        DeclareLaunchArgument('use_climb_controller', default_value='true',
                              description='Route teleop through climb safety controller.'),
        DeclareLaunchArgument('force_table_csv', default_value=default_force_table,
                              description='KMW100 force lookup CSV.'),
        DeclareLaunchArgument('wall_thickness_mm', default_value='10.0',
                              description='Flat wall thickness used for lookup.'),
        DeclareLaunchArgument('adhesion_force', default_value='900.0',
                              description='Fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('adhesion_rate', default_value='20.0',
                              description='Contact-only adhesion update rate in Hz.'),
        DeclareLaunchArgument('gap_reference_mm', default_value='1.0',
                              description='Reference gap for HKCM-style gap_decay model.'),
        DeclareLaunchArgument('safe_tilt_deg', default_value='8.0',
                              description='Tilt angle with no adhesion penalty.'),
        DeclareLaunchArgument('cutoff_tilt_deg', default_value='30.0',
                              description='Tilt angle where adhesion decays to zero.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Robot ground starting X.'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Robot ground starting Y.'),
        DeclareLaunchArgument('spawn_z', default_value='0.20',
                              description='Robot ground starting Z.'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Yaw 0 means teleop W drives toward +X / wall.'),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        world_tf,
        climb_controller,
        adhesion,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
        step_startup,
        ready,
    ])
