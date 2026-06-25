#!/usr/bin/env python3
"""
climb_test.launch.py — Flat-wall adhesion test for the 3D3S robot.

Spawns the robot against the vertical flat_steel_panel with the correct
orientation for climbing. The Kmw100AdhesionSystem Gazebo plugin applies
wheel adhesion every physics step; the Python magnetic_adhesion node is
kept as an optional fallback/debug path.

Terminal 1 (this file):
  ros2 launch robot_3d3s climb_test.launch.py

Terminal 2 (drive the robot):
  ros2 run robot_3d3s teleop_keyboard.py --ros-args -p gazebo:=true

Teleop keys on the wall:
  W / S   — climb UP / DOWN the wall
  A / D   — slide LEFT / RIGHT on the wall
  Q / E   — rotate robot body CW / CCW on the wall surface
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file        = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_path       = os.path.join(pkg, 'worlds', 'my_world.sdf')
    models_path      = os.path.join(pkg, 'models')
    plugins_path     = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
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

    run_sim = LaunchConfiguration('run_sim')
    start_adhesion = LaunchConfiguration('start_adhesion')
    start_controllers = LaunchConfiguration('start_controllers')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    adhesion_delay = LaunchConfiguration('adhesion_delay')
    adhesion_mode = LaunchConfiguration('adhesion_mode')
    adhesion_model = LaunchConfiguration('adhesion_model')
    adhesion_force = LaunchConfiguration('adhesion_force')
    min_force = LaunchConfiguration('min_force')
    gap_reference_mm = LaunchConfiguration('gap_reference_mm')
    safe_tilt_deg = LaunchConfiguration('safe_tilt_deg')
    cutoff_tilt_deg = LaunchConfiguration('cutoff_tilt_deg')
    gap_bias_mm = LaunchConfiguration('gap_bias_mm')
    max_lookup_gap_mm = LaunchConfiguration('max_lookup_gap_mm')
    force_table_csv = LaunchConfiguration('force_table_csv')
    wall_thickness_mm = LaunchConfiguration('wall_thickness_mm')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    adhesion_rate = LaunchConfiguration('adhesion_rate')
    axis_x = LaunchConfiguration('axis_x')
    axis_y = LaunchConfiguration('axis_y')
    surface_radius_m = LaunchConfiguration('surface_radius_m')
    plane_point_x = LaunchConfiguration('plane_point_x')
    plane_point_y = LaunchConfiguration('plane_point_y')
    normal_x = LaunchConfiguration('normal_x')
    normal_y = LaunchConfiguration('normal_y')
    normal_z = LaunchConfiguration('normal_z')
    plane_point_z = LaunchConfiguration('plane_point_z')

    gz_args = PythonExpression([
        "'-r -v 4 ", world_path, "' if '", run_sim,
        "' == 'true' else '-v 4 ", world_path, "'"
    ])

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # 2. Spawn robot on the +X side of the flat steel panel.
    #
    #    URDF wheel positions (in body frame):
    #      wheel_1_link ≈ (+0.414, 0,      +0.050)  ← "front" wheel
    #      wheel_2_link ≈ (-0.195, +0.337, +0.050)  ← back-left
    #      wheel_3_link ≈ (-0.195, -0.337, +0.050)  ← back-right
    #
    #    Orientation: roll=0, pitch=-π/2, yaw=0 (rotate 90° CCW about world Y)
    #      • body +X  → world +Z   (forward = climbing UP, W key)
    #      • body +Z  → world -X   (wheels stick out toward the wall)
    #      • body -Z  → world +X   (chassis back points away from the wall)
    #
    #    Climbing pose on the -X side of the flat wall.
    #    The panel back face is at x=0.95 and its outward normal is -X.
    #    Orientation R=0, P=-π/2, Y=0:
    #      body +X  → world +Z   (W key climbs UP)
    #      body +Z  → world -X   (wheels press toward the panel)
    #    The default pose below matches the manually tested Gazebo placement.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',   'robot_3d3s',
            '-string', robot_desc,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', spawn_roll, '-P', spawn_pitch, '-Y', spawn_yaw,
        ],
        output='screen',
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4. ROS <-> Gazebo clock bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_3d3s/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/robot_3d3s/wheel_1_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_3_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen',
    )

    # 5. Controller spawners
    def spawner(name, delay=5.0):
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

    # 6. Initial static TF for adhesion direction.
    #    The robot is spawned at this pose. Teleop in Gazebo mode does not
    #    publish a fake base TF, so this gives the adhesion node a correct
    #    starting transform; if TF is late, adhesion still uses its fallback.
    initial_robot_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint_initial',
        arguments=[
            '--x', spawn_x, '--y', spawn_y, '--z', spawn_z,
            '--roll', spawn_roll, '--pitch', spawn_pitch, '--yaw', spawn_yaw,
            '--frame-id', 'world', '--child-frame-id', 'base_footprint',
        ],
        output='screen',
    )

    # 7. Optional Python magnetic adhesion fallback/debug node. The default
    #    adhesion path is the Kmw100AdhesionSystem Gazebo plugin in the URDF.
    adhesion = TimerAction(
        period=adhesion_delay,
        condition=IfCondition(start_adhesion),
        actions=[Node(
            package='robot_3d3s',
            executable='magnetic_adhesion.py',
            name='magnetic_adhesion',
            output='screen',
            parameters=[{
                'mode': adhesion_mode,
                'adhesion_model': adhesion_model,
                'axis_x': ParameterValue(axis_x, value_type=float),
                'axis_y': ParameterValue(axis_y, value_type=float),
                'surface_radius_m': ParameterValue(surface_radius_m, value_type=float),
                'plane_point_x': ParameterValue(plane_point_x, value_type=float),
                'plane_point_y': ParameterValue(plane_point_y, value_type=float),
                'plane_point_z': ParameterValue(plane_point_z, value_type=float),
                'normal_x': ParameterValue(normal_x, value_type=float),
                'normal_y': ParameterValue(normal_y, value_type=float),
                'normal_z': ParameterValue(normal_z, value_type=float),
                'force_n': ParameterValue(adhesion_force, value_type=float),
                'min_force_n': ParameterValue(min_force, value_type=float),
                'gap_reference_mm': ParameterValue(gap_reference_mm, value_type=float),
                'safe_tilt_deg': ParameterValue(safe_tilt_deg, value_type=float),
                'cutoff_tilt_deg': ParameterValue(cutoff_tilt_deg, value_type=float),
                'gap_bias_mm': ParameterValue(gap_bias_mm, value_type=float),
                'max_lookup_gap_mm': ParameterValue(max_lookup_gap_mm, value_type=float),
                'force_table_csv': force_table_csv,
                'wall_thickness_mm': ParameterValue(wall_thickness_mm, value_type=float),
                'wheel_radius_m': ParameterValue(wheel_radius_m, value_type=float),
                'fallback_normal_x': ParameterValue(normal_x, value_type=float),
                'fallback_normal_y': ParameterValue(normal_y, value_type=float),
                'fallback_normal_z': ParameterValue(normal_z, value_type=float),
                'startup_delay_s': 0.0,
                'rate_hz': ParameterValue(adhesion_rate, value_type=float),
                'use_persistent_wrench': False,
                'contact_timeout_s': 0.25,
                'service_timeout_ms': 1000,
                'process_timeout_s': 1.0,
                'diagnostic_period_s': 1.0,
            }],
        )],
    )

    ready_message = TimerAction(
        period=15.0,
        actions=[LogInfo(msg=[
            '\nREADY CHECK: Gazebo is paused. Confirm the Gazebo log shows ',
            'Kmw100AdhesionSystem wheel surface/gap/force messages and all ',
            'controller spawners have finished. The Python magnetic_adhesion ',
            'node is disabled by default to avoid double-applying adhesion. ',
            'Then press Play in Gazebo.\n'
        ])],
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_sim', default_value='false',
                              description='Start Gazebo running. Set false to start paused for manual placement.'),
        DeclareLaunchArgument('start_adhesion', default_value='false',
                              description='Start optional Python adhesion fallback/debug node. The C++ Gazebo plugin is always loaded from the URDF.'),
        DeclareLaunchArgument('start_controllers', default_value='true',
                              description='Start joint_state, steering, and wheel controllers automatically.'),
        DeclareLaunchArgument('spawn_x', default_value='0.95',
                              description='Initial robot X position.'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Initial robot Y position.'),
        DeclareLaunchArgument('spawn_z', default_value='1.0',
                              description='Initial robot Z position.'),
        DeclareLaunchArgument('spawn_roll', default_value='2.9373',
                              description='Initial robot roll.'),
        DeclareLaunchArgument('spawn_pitch', default_value='-1.5708',
                              description='Initial robot pitch.'),
        DeclareLaunchArgument('spawn_yaw', default_value='-2.9373',
                              description='Initial robot yaw.'),
        DeclareLaunchArgument('adhesion_delay', default_value='1.0',
                              description='Seconds before starting adhesion. Increase for manual setup.'),
        DeclareLaunchArgument('adhesion_mode', default_value='plane',
                              description='Adhesion mode: cylinder or plane.'),
        DeclareLaunchArgument('adhesion_model', default_value='lookup',
                              description='Adhesion model: lookup or constant.'),
        DeclareLaunchArgument('adhesion_force', default_value='500.0',
                              description='Constant/fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('min_force', default_value='0.0',
                              description='Minimum adhesion force after lookup in Newtons.'),
        DeclareLaunchArgument('gap_reference_mm', default_value='1.0',
                              description='Reference gap for HKCM-style gap_decay model.'),
        DeclareLaunchArgument('safe_tilt_deg', default_value='8.0',
                              description='Tilt angle with no adhesion penalty.'),
        DeclareLaunchArgument('cutoff_tilt_deg', default_value='30.0',
                              description='Tilt angle where adhesion decays to zero.'),
        DeclareLaunchArgument('gap_bias_mm', default_value='0.0',
                              description='Bias added to estimated air gap before lookup.'),
        DeclareLaunchArgument('max_lookup_gap_mm', default_value='-1.0',
                              description='Cap lookup air gap in mm; negative disables.'),
        DeclareLaunchArgument('force_table_csv', default_value=default_force_table,
                              description='CSV lookup table for KMW100 adhesion force.'),
        DeclareLaunchArgument('wall_thickness_mm', default_value='10.0',
                              description='Steel wall thickness for lookup model.'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.05',
                              description='Magnetic wheel radius used for air-gap estimate.'),
        DeclareLaunchArgument('adhesion_rate', default_value='10.0',
                              description='Adhesion update rate in Hz.'),
        DeclareLaunchArgument('axis_x', default_value='0.55',
                              description='Cylinder axis X coordinate for the big vertical pipe.'),
        DeclareLaunchArgument('axis_y', default_value='0.55',
                              description='Cylinder axis Y coordinate for the big vertical pipe.'),
        DeclareLaunchArgument('surface_radius_m', default_value='0.0',
                              description='Cylinder outer radius for pipe mode.'),
        DeclareLaunchArgument('plane_point_x', default_value='0.95',
                              description='A point on the flat panel face, X coordinate.'),
        DeclareLaunchArgument('plane_point_y', default_value='0.0',
                              description='A point on the flat panel face, Y coordinate.'),
        DeclareLaunchArgument('plane_point_z', default_value='0.0',
                              description='A point on the flat panel face, Z coordinate.'),
        DeclareLaunchArgument('normal_x', default_value='-1.0',
                              description='Outward normal X for plane mode and cylinder fallback.'),
        DeclareLaunchArgument('normal_y', default_value='0.0',
                              description='Outward normal Y for plane mode and cylinder fallback.'),
        DeclareLaunchArgument('normal_z', default_value='0.0',
                              description='Outward normal Z for plane mode and fallback.'),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        initial_robot_tf,
        adhesion,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('steering_controller',     delay=10.0),
        spawner('wheel_controller',        delay=12.0),
        ready_message,
    ])
