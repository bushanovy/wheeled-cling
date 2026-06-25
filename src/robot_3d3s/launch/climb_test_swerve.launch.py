#!/usr/bin/env python3
"""
Flat-wall magnetic adhesion test using the 3D3S swerve_controller.

Control flow:
  teleop_keyboard_swerve.py -> climb_surface_controller.py -> /swerve_controller/cmd_vel
  swerve_controller -> steering/wheel hardware interfaces

Adhesion flow:
  Kmw100AdhesionSystem Gazebo plugin -> per-physics-step wheel adhesion
  magnetic_adhesion.py is kept as an optional fallback/debug node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
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

    run_sim = LaunchConfiguration('run_sim')
    start_adhesion = LaunchConfiguration('start_adhesion')
    start_controllers = LaunchConfiguration('start_controllers')
    use_climb_controller = LaunchConfiguration('use_climb_controller')
    auto_step_startup = LaunchConfiguration('auto_step_startup')
    startup_steps = LaunchConfiguration('startup_steps')
    controller_switch_timeout = LaunchConfiguration('controller_switch_timeout')
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
    ground_z = LaunchConfiguration('ground_z')
    slide_angle_deg = LaunchConfiguration('slide_angle_deg')
    slide_start_x = LaunchConfiguration('slide_start_x')
    slide_start_z = LaunchConfiguration('slide_start_z')
    slide_wall_x = LaunchConfiguration('slide_wall_x')
    surface_margin_m = LaunchConfiguration('surface_margin_m')
    surface_hysteresis_m = LaunchConfiguration('surface_hysteresis_m')

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
            '-R', spawn_roll, '-P', spawn_pitch, '-Y', spawn_yaw,
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_3d3s/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_3d3s/adhesion_status@std_msgs/msg/String[gz.msgs.StringMsg',
            '/robot_3d3s/wheel_1_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_3_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[
            ('/model/robot_3d3s/adhesion_status', '/robot_3d3s/adhesion_status'),
        ],
        output='screen',
    )

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

    def spawner(name, delay):
        return TimerAction(
            period=delay,
            condition=IfCondition(start_controllers),
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    name,
                    '--controller-manager-timeout', '30',
                    '--switch-timeout', controller_switch_timeout,
                ],
                output='screen',
            )],
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
            'slide_angle_deg': ParameterValue(slide_angle_deg, value_type=float),
            'min_attached_wheels': 2,
            'max_safe_gap_mm': 30.0,
            'status_timeout_s': 1.0,
            'tf_timeout_s': 0.3,
            'cmd_timeout_s': 2.0,
            'wall_confirm_s': 0.3,
            'capture_speed_limit_mps': 0.05,
            'climb_speed_limit_mps': 0.10,
        }],
    )

    adhesion = TimerAction(
        period=adhesion_delay,
        condition=IfCondition(start_adhesion),
        actions=[Node(
            package='robot_3d3s',
            executable='magnetic_adhesion.py',
            name='magnetic_adhesion',
            output='screen',
            parameters=[{
                'use_sim_time': True,
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
                'ground_z': ParameterValue(ground_z, value_type=float),
                'slide_angle_deg': ParameterValue(slide_angle_deg, value_type=float),
                'slide_start_x': ParameterValue(slide_start_x, value_type=float),
                'slide_start_z': ParameterValue(slide_start_z, value_type=float),
                'slide_wall_x': ParameterValue(slide_wall_x, value_type=float),
                'surface_margin_m': ParameterValue(surface_margin_m, value_type=float),
                'surface_hysteresis_m': ParameterValue(surface_hysteresis_m, value_type=float),
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
        period=13.0,
        actions=[LogInfo(msg=[
            '\nREADY CHECK: Gazebo should still be paused. Confirm the ',
            'Gazebo log shows Kmw100AdhesionSystem wheel surface/gap/force ',
            'messages, and confirm the swerve_controller spawner finished. ',
            'The Python magnetic_adhesion node is disabled by default to avoid ',
            'double-applying adhesion forces. For surface-aware control, run ',
            'teleop with cmd_topic:=/climb_controller/cmd_vel_in. ',
            'Then press Play in Gazebo and drive.\n'
        ])],
    )

    step_startup = TimerAction(
        period=11.0,
        condition=IfCondition(auto_step_startup),
        actions=[
            LogInfo(msg=[
                'Stepping paused Gazebo for ', startup_steps,
                ' iteration(s) so ros2_control can activate, then remaining paused.'
            ]),
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/default/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', ['multi_step: ', startup_steps],
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_sim', default_value='false',
                              description='Start Gazebo running. Set false to start paused.'),
        DeclareLaunchArgument('start_adhesion', default_value='false',
                              description='Start optional Python adhesion fallback/debug node. The C++ Gazebo plugin is always loaded from the URDF.'),
        DeclareLaunchArgument('start_controllers', default_value='true',
                              description='Start joint_state_broadcaster and swerve_controller automatically.'),
        DeclareLaunchArgument('use_climb_controller', default_value='false',
                              description='Route teleop through the surface-aware climb controller.'),
        DeclareLaunchArgument('auto_step_startup', default_value='true',
                              description='Step paused Gazebo briefly so ros2_control can activate before manual Play.'),
        DeclareLaunchArgument('startup_steps', default_value='50',
                              description='Number of 1 ms Gazebo steps for paused startup activation.'),
        DeclareLaunchArgument('controller_switch_timeout', default_value='600.0',
                              description='Controller switch timeout. Long because Gazebo starts paused.'),
        DeclareLaunchArgument('spawn_x', default_value='0.955',
                              description='Initial robot X position, with 5 mm wheel preload into the flat panel.'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Initial robot Y position for the flat panel.'),
        DeclareLaunchArgument('spawn_z', default_value='1.0',
                              description='Initial robot Z position.'),
        DeclareLaunchArgument('spawn_roll', default_value='2.9373',
                              description='Initial robot roll.'),
        DeclareLaunchArgument('spawn_pitch', default_value='-1.5708',
                              description='Initial robot pitch.'),
        DeclareLaunchArgument('spawn_yaw', default_value='-2.9373',
                              description='Initial robot yaw.'),
        DeclareLaunchArgument('adhesion_delay', default_value='1.0',
                              description='Seconds before starting adhesion.'),
        DeclareLaunchArgument('adhesion_mode', default_value='plane',
                              description='Adhesion mode: plane for flat wall, slide for ground-to-wall slide, cylinder for pipe.'),
        DeclareLaunchArgument('adhesion_model', default_value='lookup',
                              description='Adhesion model: lookup or constant.'),
        DeclareLaunchArgument('adhesion_force', default_value='900.0',
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
                              description='Cylinder axis X coordinate for pipe mode.'),
        DeclareLaunchArgument('axis_y', default_value='0.55',
                              description='Cylinder axis Y coordinate for pipe mode.'),
        DeclareLaunchArgument('surface_radius_m', default_value='0.0',
                              description='Cylinder outer radius for pipe mode.'),
        DeclareLaunchArgument('plane_point_x', default_value='0.95',
                              description='A point on the flat panel face, X coordinate.'),
        DeclareLaunchArgument('plane_point_y', default_value='0.0',
                              description='A point on the flat panel face, Y coordinate.'),
        DeclareLaunchArgument('plane_point_z', default_value='0.0',
                              description='A point on the flat panel face, Z coordinate.'),
        DeclareLaunchArgument('normal_x', default_value='-1.0',
                              description='Outward normal X for plane mode.'),
        DeclareLaunchArgument('normal_y', default_value='0.0',
                              description='Outward normal Y for plane mode.'),
        DeclareLaunchArgument('normal_z', default_value='0.0',
                              description='Outward normal Z for plane mode or fallback.'),
        DeclareLaunchArgument('ground_z', default_value='0.0',
                              description='Ground steel plate height for slide mode.'),
        DeclareLaunchArgument('slide_angle_deg', default_value='45.0',
                              description='Slide angle above the ground for slide mode.'),
        DeclareLaunchArgument('slide_start_x', default_value='-0.15',
                              description='Lower slide surface X coordinate for slide mode.'),
        DeclareLaunchArgument('slide_start_z', default_value='0.0',
                              description='Lower slide surface Z coordinate for slide mode.'),
        DeclareLaunchArgument('slide_wall_x', default_value='0.95',
                              description='Vertical wall face X coordinate for slide mode.'),
        DeclareLaunchArgument('surface_margin_m', default_value='0.08',
                              description='Surface transition margin for slide mode.'),
        DeclareLaunchArgument('surface_hysteresis_m', default_value='0.03',
                              description='Surface latch hysteresis for slide mode.'),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        initial_robot_tf,
        climb_controller,
        adhesion,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
        step_startup,
        ready_message,
    ])
