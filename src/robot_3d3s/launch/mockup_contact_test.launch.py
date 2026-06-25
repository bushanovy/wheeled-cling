#!/usr/bin/env python3
"""
Mockup structure collision/contact test.

This is the controlled mockup pipe contact/climb test. It loads a mockup-only
world with the mockup_structure include enabled, spawns the robot beside it,
prints compact per-wheel contact reports, and uses contact-only COMSOL-table
adhesion through the Python magnetic_adhesion node.
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
    world_name = 'mockup_pipe_test'
    world_path = os.path.join(pkg, 'worlds', 'mockup_contact_test.sdf')
    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    default_force_table = os.path.join(pkg, 'config', 'kmw100_comsol_seed_table.csv')
    default_rviz_config = os.path.join(pkg, 'config', 'display.rviz')

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
    start_monitor = LaunchConfiguration('start_monitor')
    start_adhesion = LaunchConfiguration('start_adhesion')
    use_climb_controller = LaunchConfiguration('use_climb_controller')
    use_mockup_planner = LaunchConfiguration('use_mockup_planner')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    force_table_csv = LaunchConfiguration('force_table_csv')
    wall_thickness_mm = LaunchConfiguration('wall_thickness_mm')
    pipe_radius_m = LaunchConfiguration('pipe_radius_m')
    adhesion_force = LaunchConfiguration('adhesion_force')
    adhesion_rate = LaunchConfiguration('adhesion_rate')
    gap_reference_mm = LaunchConfiguration('gap_reference_mm')
    safe_tilt_deg = LaunchConfiguration('safe_tilt_deg')
    cutoff_tilt_deg = LaunchConfiguration('cutoff_tilt_deg')
    full_contact_depth_m = LaunchConfiguration('full_contact_depth_m')
    full_contact_points = LaunchConfiguration('full_contact_points')
    min_contact_fraction = LaunchConfiguration('min_contact_fraction')
    motion_smoothing = LaunchConfiguration('motion_smoothing')
    capture_speed_limit_mps = LaunchConfiguration('capture_speed_limit_mps')
    climb_speed_limit_mps = LaunchConfiguration('climb_speed_limit_mps')
    accel_limit_mps2 = LaunchConfiguration('accel_limit_mps2')
    yaw_accel_limit_radps2 = LaunchConfiguration('yaw_accel_limit_radps2')
    gap_slow_start_mm = LaunchConfiguration('gap_slow_start_mm')
    low_force_slow_n = LaunchConfiguration('low_force_slow_n')
    min_motion_scale = LaunchConfiguration('min_motion_scale')
    partial_contact_scale = LaunchConfiguration('partial_contact_scale')

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

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'robot_3d3s',
                '-string', robot_desc,
                '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        )],
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

    gazebo_model_tf = Node(
        package='robot_3d3s',
        executable='gazebo_model_tf_relay.py',
        name='gazebo_model_tf_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': '/model/robot_3d3s/pose',
            'parent_frame': 'world',
            'child_frame': 'base_footprint',
            'model_name': 'robot_3d3s',
            'fallback_x': ParameterValue(spawn_x, value_type=float),
            'fallback_y': ParameterValue(spawn_y, value_type=float),
            'fallback_z': ParameterValue(spawn_z, value_type=float),
            'fallback_yaw': ParameterValue(spawn_yaw, value_type=float),
        }],
    )

    contact_monitor = Node(
        package='robot_3d3s',
        executable='mockup_contact_monitor.py',
        name='mockup_contact_monitor',
        condition=IfCondition(start_monitor),
        output='screen',
        parameters=[{'use_sim_time': True}],
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
            'base_frame': 'base_footprint',
            'min_attached_wheels': 2,
            'max_safe_gap_mm': 30.0,
            'status_timeout_s': 1.0,
            'tf_timeout_s': 0.3,
            'cmd_timeout_s': 2.0,
            'wall_confirm_s': 0.3,
            'capture_speed_limit_mps': ParameterValue(capture_speed_limit_mps, value_type=float),
            'climb_speed_limit_mps': ParameterValue(climb_speed_limit_mps, value_type=float),
            'motion_smoothing': ParameterValue(motion_smoothing, value_type=bool),
            'accel_limit_mps2': ParameterValue(accel_limit_mps2, value_type=float),
            'yaw_accel_limit_radps2': ParameterValue(yaw_accel_limit_radps2, value_type=float),
            'gap_slow_start_mm': ParameterValue(gap_slow_start_mm, value_type=float),
            'low_force_slow_n': ParameterValue(low_force_slow_n, value_type=float),
            'min_motion_scale': ParameterValue(min_motion_scale, value_type=float),
            'partial_contact_scale': ParameterValue(partial_contact_scale, value_type=float),
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
            'mode': 'cylinder',
            'adhesion_model': 'lookup',
            'force_n': ParameterValue(adhesion_force, value_type=float),
            'min_force_n': 0.0,
            'gap_reference_mm': ParameterValue(gap_reference_mm, value_type=float),
            'safe_tilt_deg': ParameterValue(safe_tilt_deg, value_type=float),
            'cutoff_tilt_deg': ParameterValue(cutoff_tilt_deg, value_type=float),
            'force_table_csv': force_table_csv,
            'wall_thickness_mm': ParameterValue(wall_thickness_mm, value_type=float),
            'pipe_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'rate_hz': ParameterValue(adhesion_rate, value_type=float),
            'contact_only': True,
            'contact_timeout_s': 0.25,
            'full_contact_depth_m': ParameterValue(full_contact_depth_m, value_type=float),
            'full_contact_points': ParameterValue(full_contact_points, value_type=int),
            'min_contact_fraction': ParameterValue(min_contact_fraction, value_type=float),
            'world_name': world_name,
            'gazebo_pose_topic': f'/world/{world_name}/pose/info',
            'status_topic': '/robot_3d3s/adhesion_status',
            'status_period_s': 0.05,
            'service_timeout_ms': 1000,
            'process_timeout_s': 1.0,
            'diagnostic_period_s': 1.0,
        }],
    )

    mockup_planner = Node(
        package='robot_3d3s',
        executable='mockup_climb_planner.py',
        name='mockup_climb_planner',
        condition=IfCondition(use_mockup_planner),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_frame': 'world',
            'base_frame': 'base_footprint',
            'cmd_topic': '/climb_controller/cmd_vel_in',
            'status_topic': '/robot_3d3s/adhesion_status',
            'clicked_point_topic': '/clicked_point',
            'goal_pose_topic': '/goal_pose',
            'marker_topic': '/mockup_climb_markers',
            'default_target_x': ParameterValue(spawn_x, value_type=float),
            'mockup_front_y': 0.0,
            'mockup_contact_y': -0.03,
            'mockup_x_min': 0.0,
            'mockup_x_max': 3.7,
            'mockup_z_min': 0.0,
            'mockup_z_max': 3.11,
            'approach_speed_mps': 0.12,
            'climb_speed_mps': 0.11,
            'lateral_speed_mps': 0.05,
        }],
    )

    rviz = TimerAction(
        period=12.0,
        condition=IfCondition(use_rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )],
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
            '\nMOCKUP CONTACT TEST: Gazebo entity tree should show world ',
            world_name, ' with ground_plane, mockup_structure, sun, and robot_3d3s only. ',
            'Press Play, then drive toward the mockup with teleop. The robot starts near the left vertical ',
            'pipe at x=1.0, y=-0.75 and yaw=90deg, so W drives toward +Y ',
            'and into that pipe. Watch ',
            'mockup_contact_monitor for other_collision containing mockup_structure. ',
            'Contact-only magnetic_adhesion uses the COMSOL-style lookup table ',
            force_table_csv, '. Attachment is applied only after real wheel contact ',
            'is reported. For the safety controller, run:\n',
            'ros2 run robot_3d3s teleop_keyboard_swerve.py --ros-args ',
            '-p cmd_topic:=/climb_controller/cmd_vel_in\n'
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
        DeclareLaunchArgument('start_monitor', default_value='true',
                              description='Start compact wheel contact monitor.'),
        DeclareLaunchArgument('start_adhesion', default_value='true',
                              description='Start contact-only Python adhesion for mockup pipe tests.'),
        DeclareLaunchArgument('use_climb_controller', default_value='true',
                              description='Route teleop through climb safety controller.'),
        DeclareLaunchArgument('use_mockup_planner', default_value='true',
                              description='Enable RViz clicked-point mockup climb planner.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz for robot, mockup, and clicked goals.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='RViz configuration file.'),
        DeclareLaunchArgument('force_table_csv', default_value=default_force_table,
                              description='COMSOL-style KMW100 force lookup CSV.'),
        DeclareLaunchArgument('wall_thickness_mm', default_value='9.0',
                              description='Mockup pipe wall thickness used for lookup.'),
        DeclareLaunchArgument('pipe_radius_m', default_value='0.55',
                              description='Mockup pipe radius used for lookup.'),
        DeclareLaunchArgument('adhesion_force', default_value='650.0',
                              description='Minimum/fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('adhesion_rate', default_value='20.0',
                              description='Contact-only adhesion update rate in Hz.'),
        DeclareLaunchArgument('gap_reference_mm', default_value='1.0',
                              description='Reference gap for HKCM-style gap_decay model.'),
        DeclareLaunchArgument('safe_tilt_deg', default_value='8.0',
                              description='Tilt angle with no adhesion penalty.'),
        DeclareLaunchArgument('cutoff_tilt_deg', default_value='30.0',
                              description='Tilt angle where adhesion decays to zero.'),
        DeclareLaunchArgument('full_contact_depth_m', default_value='0.002',
                              description='Contact depth treated as full magnetic contact.'),
        DeclareLaunchArgument('full_contact_points', default_value='4',
                              description='Contact point count treated as full magnetic contact.'),
        DeclareLaunchArgument('min_contact_fraction', default_value='0.10',
                              description='Minimum fraction for fresh magnetic contact.'),
        DeclareLaunchArgument('motion_smoothing', default_value='true',
                              description='Limit command acceleration in the climb safety controller.'),
        DeclareLaunchArgument('capture_speed_limit_mps', default_value='0.07',
                              description='Max speed while confirming wall or pipe contact.'),
        DeclareLaunchArgument('climb_speed_limit_mps', default_value='0.14',
                              description='Max speed while attached to wall or pipe surfaces.'),
        DeclareLaunchArgument('accel_limit_mps2', default_value='0.35',
                              description='Linear acceleration limit for mockup teleop commands.'),
        DeclareLaunchArgument('yaw_accel_limit_radps2', default_value='1.20',
                              description='Yaw acceleration limit for mockup teleop commands.'),
        DeclareLaunchArgument('gap_slow_start_mm', default_value='15.0',
                              description='Wheel gap where the mockup controller starts slowing motion.'),
        DeclareLaunchArgument('low_force_slow_n', default_value='300.0',
                              description='Attached wheel force below this slows mockup motion.'),
        DeclareLaunchArgument('min_motion_scale', default_value='0.35',
                              description='Lowest automatic speed scale while still allowing recovery motion.'),
        DeclareLaunchArgument('partial_contact_scale', default_value='0.75',
                              description='Speed scale when fewer than min_attached_wheels have contact.'),
        DeclareLaunchArgument('spawn_x', default_value='1.00',
                              description='Robot starting X aligned with the left vertical pipe.'),
        DeclareLaunchArgument('spawn_y', default_value='-0.75',
                              description='Robot starting Y, just outside the mockup side.'),
        DeclareLaunchArgument('spawn_z', default_value='0.00',
                              description='Robot starting base_footprint Z; 0 puts the wheels on the ground.'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708',
                              description='Yaw 90 deg so teleop W drives toward +Y.'),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        world_tf,
        gazebo_model_tf,
        contact_monitor,
        adhesion,
        climb_controller,
        mockup_planner,
        rviz,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
        step_startup,
        ready,
    ])
