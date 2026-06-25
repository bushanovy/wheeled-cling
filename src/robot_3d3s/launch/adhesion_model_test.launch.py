#!/usr/bin/env python3
"""Vertical pipe adhesion test world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULTS = {
    'pipe_radius_m': 1.20,
    'pipe_length_m': 4.00,
    'pipe_center_x': 0.00,
    'pipe_center_y': 0.00,
    'pipe_center_z': 2.00,
    'spawn_x': 1.17,
    'spawn_y': 0.00,
    'spawn_z': 2.00,
    'spawn_roll': 3.1416,
    'spawn_pitch': -1.5708,
    'spawn_yaw': 0.0,
    'target_axis_m': 2.00,
    'target_angle_deg': 0.0,
    'robot_mass_kg': 25.0,
}


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_name = 'adhesion_model_test'
    world_path = os.path.join(pkg, 'worlds', 'adhesion_model_test.sdf')
    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    default_force_table = os.path.join(pkg, 'config', 'kmw100_comsol_seed_table.csv')
    default_rviz_config = os.path.join(pkg, 'config', 'display.rviz')
    defaults = DEFAULTS

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
        '<adhesion_model>gap_decay</adhesion_model>',
        '<adhesion_model>constant</adhesion_model>')
    robot_desc = robot_desc.replace(
        '<min_force_n>0.0</min_force_n>',
        '<min_force_n>300.0</min_force_n>')
    robot_desc = robot_desc.replace(
        '<max_gap_m>0.06</max_gap_m>',
        '<max_gap_m>0.12</max_gap_m>')
    robot_desc = robot_desc.replace(
        '<corner_capture_m>0.08</corner_capture_m>',
        '<corner_capture_m>0.14</corner_capture_m>')
    robot_desc = robot_desc.replace(
        '<enable_opposite_wall>true</enable_opposite_wall>',
        '<enable_opposite_wall>false</enable_opposite_wall>')
    robot_desc = robot_desc.replace(
        '<enable_wall>true</enable_wall>',
        '<enable_wall>false</enable_wall>')
    robot_desc = robot_desc.replace(
        '<enable_ground>true</enable_ground>',
        '<enable_ground>false</enable_ground>')
    robot_desc = robot_desc.replace(
        '<enable_slide>true</enable_slide>',
        '<enable_slide>false</enable_slide>')
    robot_desc = robot_desc.replace(
        '<enable_pipe>false</enable_pipe>',
        '<enable_pipe>true</enable_pipe>')
    robot_desc = robot_desc.replace(
        '<pipe_axis>x</pipe_axis>',
        '<pipe_axis>z</pipe_axis>')
    robot_desc = robot_desc.replace(
        '<pipe_center_x>0.0</pipe_center_x>',
        f'<pipe_center_x>{defaults["pipe_center_x"]:.2f}</pipe_center_x>')
    robot_desc = robot_desc.replace(
        '<pipe_center_y>0.0</pipe_center_y>',
        f'<pipe_center_y>{defaults["pipe_center_y"]:.2f}</pipe_center_y>')
    robot_desc = robot_desc.replace(
        '<pipe_center_z>2.00</pipe_center_z>',
        f'<pipe_center_z>{defaults["pipe_center_z"]:.2f}</pipe_center_z>')
    robot_desc = robot_desc.replace(
        '<pipe_radius_m>1.20</pipe_radius_m>',
        f'<pipe_radius_m>{defaults["pipe_radius_m"]:.2f}</pipe_radius_m>')
    robot_desc = robot_desc.replace(
        '<pipe_length_m>6.00</pipe_length_m>',
        f'<pipe_length_m>{defaults["pipe_length_m"]:.2f}</pipe_length_m>')
    robot_desc = robot_desc.replace(
        '<surface_margin_m>0.08</surface_margin_m>',
        '<surface_margin_m>0.12</surface_margin_m>')
    robot_desc = robot_desc.replace(
        '<status_topic>/model/robot_3d3s/adhesion_status</status_topic>',
        '<status_topic>/robot_3d3s/adhesion_status</status_topic>')

    run_sim = LaunchConfiguration('run_sim')
    auto_step_startup = LaunchConfiguration('auto_step_startup')
    startup_steps = LaunchConfiguration('startup_steps')
    start_controllers = LaunchConfiguration('start_controllers')
    start_adhesion = LaunchConfiguration('start_adhesion')
    start_monitor = LaunchConfiguration('start_monitor')
    use_surface_planner = LaunchConfiguration('use_surface_planner')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    rviz_config = LaunchConfiguration('rviz_config')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    force_table_csv = LaunchConfiguration('force_table_csv')
    wall_thickness_mm = LaunchConfiguration('wall_thickness_mm')
    pipe_radius_m = LaunchConfiguration('pipe_radius_m')
    adhesion_force = LaunchConfiguration('adhesion_force')
    adhesion_rate = LaunchConfiguration('adhesion_rate')
    full_contact_depth_m = LaunchConfiguration('full_contact_depth_m')
    full_contact_points = LaunchConfiguration('full_contact_points')
    min_contact_fraction = LaunchConfiguration('min_contact_fraction')
    analytic_capture_gap_mm = LaunchConfiguration('analytic_capture_gap_mm')
    boundary_hold_risk = LaunchConfiguration('boundary_hold_risk')
    friction_mu = LaunchConfiguration('friction_mu')
    traction_safety_factor = LaunchConfiguration('traction_safety_factor')
    min_side_contact_fraction = LaunchConfiguration('min_side_contact_fraction')
    allow_guarded_low_contact = LaunchConfiguration('allow_guarded_low_contact')
    target_angle_deg = LaunchConfiguration('target_angle_deg')
    robot_mass_kg = LaunchConfiguration('robot_mass_kg')

    gz_args = PythonExpression([
        "('-r ' if '", run_sim, "' == 'true' else '') + ",
        "('-v 4 ' if '", use_gazebo_gui,
        "' == 'true' else '-s -v 4 ') + '", world_path, "'"
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
            '-R', spawn_roll,
            '-P', spawn_pitch,
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
            f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_3d3s/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/robot_3d3s/wheel_1_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_3_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/adhesion_status@std_msgs/msg/String[gz.msgs.StringMsg',
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

    contact_monitor = Node(
        package='robot_3d3s',
        executable='mockup_contact_monitor.py',
        name='adhesion_contact_monitor',
        condition=IfCondition(start_monitor),
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    gazebo_model_tf = Node(
        package='robot_3d3s',
        executable='gazebo_model_tf_relay.py',
        name='gazebo_model_tf_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': f'/world/{world_name}/dynamic_pose/info',
            'parent_frame': 'world',
            'child_frame': 'base_footprint',
            'model_name': 'robot_3d3s',
            'fallback_transform_index': 0,
            'fallback_x': ParameterValue(spawn_x, value_type=float),
            'fallback_y': ParameterValue(spawn_y, value_type=float),
            'fallback_z': ParameterValue(spawn_z, value_type=float),
            'fallback_roll': ParameterValue(spawn_roll, value_type=float),
            'fallback_pitch': ParameterValue(spawn_pitch, value_type=float),
            'fallback_yaw': ParameterValue(spawn_yaw, value_type=float),
        }],
    )

    scene_markers = Node(
        package='robot_3d3s',
        executable='adhesion_scene_rviz.py',
        name='adhesion_scene_rviz',
        condition=IfCondition(use_rviz),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_topic': '/adhesion_model_markers',
            'vertical_pipe_radius_m': defaults['pipe_radius_m'],
            'vertical_pipe_length_m': defaults['pipe_length_m'],
            'vertical_pipe_x': defaults['pipe_center_x'],
            'vertical_pipe_y': defaults['pipe_center_y'],
            'vertical_pipe_z': defaults['pipe_center_z'],
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
            'cylinder_axis': 'z',
            'axis_x': defaults['pipe_center_x'],
            'axis_y': defaults['pipe_center_y'],
            'axis_z': defaults['pipe_center_z'],
            'surface_radius_m': defaults['pipe_radius_m'],
            'adhesion_model': 'lookup',
            'force_n': ParameterValue(adhesion_force, value_type=float),
            'min_force_n': 0.0,
            'force_table_csv': force_table_csv,
            'wall_thickness_mm': ParameterValue(wall_thickness_mm, value_type=float),
            'pipe_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'rate_hz': ParameterValue(adhesion_rate, value_type=float),
            'contact_only': True,
            'contact_timeout_s': 0.25,
            'analytic_capture_gap_mm': ParameterValue(analytic_capture_gap_mm, value_type=float),
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

    surface_planner = Node(
        package='robot_3d3s',
        executable='surface_goal_planner.py',
        name='surface_goal_planner',
        condition=IfCondition(use_surface_planner),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_frame': 'world',
            'base_frame': 'base_footprint',
            'cmd_topic': '/swerve_controller/cmd_vel',
            'status_topic': '/robot_3d3s/adhesion_status',
            'clicked_point_topic': '/clicked_point',
            'goal_pose_topic': '/goal_pose',
            'marker_topic': '/surface_goal_markers',
            'debug_topic': '/surface_goal_planner/debug',
            'hold_steering_topic': '/swerve_controller/hold_steering',
            'surface_mode': 'cylinder',
            'cylinder_axis': 'z',
            'axis_x': defaults['pipe_center_x'],
            'axis_y': defaults['pipe_center_y'],
            'axis_z': defaults['pipe_center_z'],
            'surface_radius_m': defaults['pipe_radius_m'],
            'surface_length_m': defaults['pipe_length_m'],
            'target_axis_m': defaults['target_axis_m'],
            'target_angle_deg': ParameterValue(target_angle_deg, value_type=float),
            'wait_for_goal': True,
            'approach_axis_m': defaults['target_axis_m'],
            'approach_theta_deg': 0.0,
            'boundary_hold_risk': ParameterValue(boundary_hold_risk, value_type=float),
            'friction_mu': ParameterValue(friction_mu, value_type=float),
            'robot_mass_kg': ParameterValue(robot_mass_kg, value_type=float),
            'safety_factor': ParameterValue(traction_safety_factor, value_type=float),
            'min_side_contact_fraction': ParameterValue(min_side_contact_fraction, value_type=float),
            'allow_guarded_low_contact': ParameterValue(allow_guarded_low_contact, value_type=bool),
            'wheel_torque_limit_nm': 40.0,
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

    pipe_grid = Node(
        package='robot_3d3s',
        executable='experimental_polygon_grid.py',
        name='pipe_surface_grid',
        condition=IfCondition(use_rviz),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_frame': 'world',
            'marker_topic': '/surface_goal_markers',
            'point_cloud_topic': '/surface_goal_pick_points',
            'grid_profile': 'configured_pipe',
            'grid_namespace': 'pipe_surface_grid',
            'publish_hz': 5.0,
            'axial_lines': 16,
            'rings': 24,
            'ring_segments': 72,
            'line_width_m': 0.010,
            'hit_target_enabled': True,
            'hit_target_alpha': 0.08,
            'surface_alpha': 0.0,
            'surface_axis_segments': 24,
            'surface_ring_segments': 72,
            'point_cloud_axis_samples': 56,
            'point_cloud_ring_samples': 112,
            'cylinder_axis': 'z',
            'axis_x': defaults['pipe_center_x'],
            'axis_y': defaults['pipe_center_y'],
            'axis_z': defaults['pipe_center_z'],
            'surface_radius_m': defaults['pipe_radius_m'],
            'surface_length_m': defaults['pipe_length_m'],
        }],
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
            '\nVERTICAL PIPE TEST: centered steel_vertical_pipe at x=',
            f'{defaults["pipe_center_x"]:.2f}',
            ', y=', f'{defaults["pipe_center_y"]:.2f}',
            '. The vertical pipe radius is ', f'{defaults["pipe_radius_m"]:.2f}',
            ' m, length is ', f'{defaults["pipe_length_m"]:.2f}',
            ' m, and its bottom touches the ground. Robot spawn is x=',
            spawn_x, ', y=', spawn_y, ', z=', spawn_z,
            ', roll=', spawn_roll, ', pitch=', spawn_pitch,
            ', yaw=', spawn_yaw, '.\n'
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
        DeclareLaunchArgument('start_adhesion', default_value='false',
                              description='Start contact-only Python adhesion. The Gazebo plugin handles the vertical pipe by default.'),
        DeclareLaunchArgument('start_monitor', default_value='true',
                              description='Start compact wheel contact monitor.'),
        DeclareLaunchArgument('use_surface_planner', default_value='true',
                              description='Enable unified surface goal planner for the vertical pipe.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz with adhesion model scene markers.'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='false',
                              description='Start Gazebo GUI. False runs Gazebo server only and visualizes in RViz.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='RViz configuration file.'),
        DeclareLaunchArgument('force_table_csv', default_value=default_force_table,
                              description='COMSOL-style KMW100 force lookup CSV.'),
        DeclareLaunchArgument('wall_thickness_mm', default_value='9.0',
                              description='Steel wall thickness used for lookup.'),
        DeclareLaunchArgument('pipe_radius_m', default_value=f'{defaults["pipe_radius_m"]:.2f}',
                              description='Pipe radius used for lookup.'),
        DeclareLaunchArgument('adhesion_force', default_value='900.0',
                              description='Fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('adhesion_rate', default_value='20.0',
                              description='Contact-only adhesion update rate in Hz.'),
        DeclareLaunchArgument('full_contact_depth_m', default_value='0.002',
                              description='Contact depth treated as full magnetic contact.'),
        DeclareLaunchArgument('full_contact_points', default_value='4',
                              description='Contact point count treated as full magnetic contact.'),
        DeclareLaunchArgument('min_contact_fraction', default_value='0.10',
                              description='Minimum fraction for fresh magnetic contact.'),
        DeclareLaunchArgument('analytic_capture_gap_mm', default_value='30.0',
                              description='Air gap where cylinder magnets can analytically capture before contact.'),
        DeclareLaunchArgument('boundary_hold_risk', default_value='0.78',
                              description='Planner stops when adhesion boundary risk reaches this value.'),
        DeclareLaunchArgument('friction_mu', default_value='1.0',
                              description='Wheel/surface friction coefficient for planner traction estimates.'),
        DeclareLaunchArgument('traction_safety_factor', default_value='1.5',
                              description='Safety factor applied to required tangential load.'),
        DeclareLaunchArgument('min_side_contact_fraction', default_value='0.45',
                              description='Minimum average contact fraction for steep pipe motion.'),
        DeclareLaunchArgument('allow_guarded_low_contact', default_value='true',
                              description='Allow slow guarded pipe motion when contact is partial but force margin is OK.'),
        DeclareLaunchArgument('target_angle_deg', default_value=f'{defaults["target_angle_deg"]:.1f}',
                              description='Initial vertical pipe target angle around the pipe.'),
        DeclareLaunchArgument('robot_mass_kg', default_value=f'{defaults["robot_mass_kg"]:.1f}',
                              description='Robot mass used by planner traction estimates. Gazebo inertial masses still come from URDF.'),
        DeclareLaunchArgument('spawn_x', default_value=f'{defaults["spawn_x"]:.2f}',
                              description='Robot starting X.'),
        DeclareLaunchArgument('spawn_y', default_value=f'{defaults["spawn_y"]:.2f}',
                              description='Robot starting Y.'),
        DeclareLaunchArgument('spawn_z', default_value=f'{defaults["spawn_z"]:.2f}',
                              description='Robot starting Z.'),
        DeclareLaunchArgument('spawn_roll', default_value=f'{defaults["spawn_roll"]:.4f}',
                              description='Robot starting roll. +X side of the vertical pipe uses pi.'),
        DeclareLaunchArgument('spawn_pitch', default_value=f'{defaults["spawn_pitch"]:.4f}',
                              description='Robot starting pitch. +X side of the vertical pipe uses -pi/2.'),
        DeclareLaunchArgument('spawn_yaw', default_value=f'{defaults["spawn_yaw"]:.4f}',
                              description='Yaw 0 means teleop W drives toward +X.'),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        world_tf,
        contact_monitor,
        gazebo_model_tf,
        scene_markers,
        adhesion,
        surface_planner,
        pipe_grid,
        rviz,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
        step_startup,
        ready,
    ])
