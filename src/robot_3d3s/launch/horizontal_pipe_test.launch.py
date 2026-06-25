#!/usr/bin/env python3
"""
Large horizontal pipe curvature test.

This launch avoids the imported mockup mesh and loads a primitive steel pipe
with a 1.2 m radius and horizontal X axis. The robot starts on the +Y side of
the pipe by default, then surface_goal_planner.py can command motion around the
circular cross-section.
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


DEFAULTS = {
    'wall_thickness_mm': 9.0,
    'pipe_radius_m': 1.20,
    'pipe_center_z': 2.00,
    'spawn_x': -1.99,
    'spawn_y': 1.29,
    'spawn_z': 2.00,
    'spawn_roll': -1.5708,
    'spawn_pitch': 1.5708,
    'spawn_yaw': 0.0,
    'side_start_theta_deg': 0.0,
    'startup_side_hold_s': 15.0,
    'robot_mass_kg': 25.0,
    'target_x': -1.99,
    'target_angle_deg': 90.0,
}


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_name = 'horizontal_pipe_test'
    world_path = os.path.join(pkg, 'worlds', 'horizontal_pipe_test.sdf')
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
        '<pipe_center_z>1.20</pipe_center_z>',
        '<pipe_center_z>2.00</pipe_center_z>')
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
    start_monitor = LaunchConfiguration('start_monitor')
    start_adhesion = LaunchConfiguration('start_adhesion')
    start_attachment_guard = LaunchConfiguration('start_attachment_guard')
    use_pipe_planner = LaunchConfiguration('use_pipe_planner')
    surface_graph_yaml = LaunchConfiguration('surface_graph_yaml')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    rviz_config = LaunchConfiguration('rviz_config')
    force_table_csv = LaunchConfiguration('force_table_csv')
    wall_thickness_mm = LaunchConfiguration('wall_thickness_mm')
    pipe_radius_m = LaunchConfiguration('pipe_radius_m')
    pipe_center_x = LaunchConfiguration('pipe_center_x')
    pipe_center_y = LaunchConfiguration('pipe_center_y')
    pipe_center_z = LaunchConfiguration('pipe_center_z')
    pipe_length_m = LaunchConfiguration('pipe_length_m')
    target_x = LaunchConfiguration('target_x')
    target_angle_deg = LaunchConfiguration('target_angle_deg')
    approach_theta_deg = LaunchConfiguration('approach_theta_deg')
    approach_speed_mps = LaunchConfiguration('approach_speed_mps')
    axis_speed_mps = LaunchConfiguration('axis_speed_mps')
    curve_speed_mps = LaunchConfiguration('curve_speed_mps')
    adhesion_force = LaunchConfiguration('adhesion_force')
    adhesion_model = LaunchConfiguration('adhesion_model')
    scale_constant_by_contact_fraction = LaunchConfiguration('scale_constant_by_contact_fraction')
    adhesion_min_force = LaunchConfiguration('adhesion_min_force')
    adhesion_rate = LaunchConfiguration('adhesion_rate')
    contact_timeout_s = LaunchConfiguration('contact_timeout_s')
    gap_reference_mm = LaunchConfiguration('gap_reference_mm')
    safe_tilt_deg = LaunchConfiguration('safe_tilt_deg')
    cutoff_tilt_deg = LaunchConfiguration('cutoff_tilt_deg')
    full_contact_depth_m = LaunchConfiguration('full_contact_depth_m')
    full_contact_points = LaunchConfiguration('full_contact_points')
    min_contact_fraction = LaunchConfiguration('min_contact_fraction')
    analytic_capture_gap_mm = LaunchConfiguration('analytic_capture_gap_mm')
    attachment_guard_offset_m = LaunchConfiguration('attachment_guard_offset_m')
    attachment_guard_tolerance_m = LaunchConfiguration('attachment_guard_tolerance_m')
    attachment_guard_max_correction_m = LaunchConfiguration('attachment_guard_max_correction_m')
    attachment_guard_latch_timeout_s = LaunchConfiguration('attachment_guard_latch_timeout_s')
    attachment_guard_capture_tolerance_m = LaunchConfiguration('attachment_guard_capture_tolerance_m')
    attachment_guard_align_orientation = LaunchConfiguration('attachment_guard_align_orientation')
    attachment_guard_position_gain = LaunchConfiguration('attachment_guard_position_gain')
    attachment_guard_orientation_gain = LaunchConfiguration('attachment_guard_orientation_gain')
    attachment_guard_method = LaunchConfiguration('attachment_guard_method')
    attachment_guard_stiffness = LaunchConfiguration('attachment_guard_stiffness')
    attachment_guard_max_force = LaunchConfiguration('attachment_guard_max_force')
    attachment_guard_orientation_stiffness = LaunchConfiguration('attachment_guard_orientation_stiffness')
    attachment_guard_max_torque = LaunchConfiguration('attachment_guard_max_torque')
    attachment_guard_emergency_error_m = LaunchConfiguration('attachment_guard_emergency_error_m')
    startup_side_hold_s = LaunchConfiguration('startup_side_hold_s')
    side_start_theta_deg = LaunchConfiguration('side_start_theta_deg')
    attachment_settle_s = LaunchConfiguration('attachment_settle_s')
    boundary_hold_risk = LaunchConfiguration('boundary_hold_risk')
    min_side_contact_fraction = LaunchConfiguration('min_side_contact_fraction')
    allow_guarded_low_contact = LaunchConfiguration('allow_guarded_low_contact')
    traction_safety_factor = LaunchConfiguration('traction_safety_factor')
    friction_mu = LaunchConfiguration('friction_mu')
    robot_mass_kg = LaunchConfiguration('robot_mass_kg')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

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

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
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
            f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
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

    contact_monitor = Node(
        package='robot_3d3s',
        executable='mockup_contact_monitor.py',
        name='horizontal_pipe_contact_monitor',
        condition=IfCondition(start_monitor),
        output='screen',
        parameters=[{'use_sim_time': True}],
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
            'cylinder_axis': 'x',
            'axis_x': ParameterValue(pipe_center_x, value_type=float),
            'axis_y': ParameterValue(pipe_center_y, value_type=float),
            'axis_z': ParameterValue(pipe_center_z, value_type=float),
            'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'adhesion_model': adhesion_model,
            'force_n': ParameterValue(adhesion_force, value_type=float),
            'min_force_n': ParameterValue(adhesion_min_force, value_type=float),
            'scale_constant_by_contact_fraction': ParameterValue(
                scale_constant_by_contact_fraction, value_type=bool),
            'gap_reference_mm': ParameterValue(gap_reference_mm, value_type=float),
            'safe_tilt_deg': ParameterValue(safe_tilt_deg, value_type=float),
            'cutoff_tilt_deg': ParameterValue(cutoff_tilt_deg, value_type=float),
            'force_table_csv': force_table_csv,
            'wall_thickness_mm': ParameterValue(wall_thickness_mm, value_type=float),
            'pipe_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'rate_hz': ParameterValue(adhesion_rate, value_type=float),
            'contact_only': True,
            'magnetic_surface_allowlist': 'horizontal_pipe',
            'contact_timeout_s': ParameterValue(contact_timeout_s, value_type=float),
            'analytic_capture_gap_mm': ParameterValue(analytic_capture_gap_mm, value_type=float),
            'allow_tf_analytic_capture': True,
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

    pipe_planner = Node(
        package='robot_3d3s',
        executable='surface_goal_planner.py',
        name='surface_goal_planner',
        condition=IfCondition(use_pipe_planner),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_frame': 'world',
            'base_frame': 'base_footprint',
            'cmd_topic': '/swerve_controller/cmd_vel',
            'status_topic': '/robot_3d3s/adhesion_status',
            'clicked_point_topic': '/clicked_point',
            'goal_pose_topic': '/goal_pose',
            'marker_topic': '/horizontal_pipe_markers',
            'debug_topic': '/surface_goal_planner/debug',
            'legacy_debug_topic': '/horizontal_pipe_planner/debug',
            'hold_steering_topic': '/swerve_controller/hold_steering',
            'surface_graph_yaml': surface_graph_yaml,
            'surface_mode': 'cylinder',
            'cylinder_axis': 'x',
            'axis_x': ParameterValue(pipe_center_x, value_type=float),
            'axis_y': ParameterValue(pipe_center_y, value_type=float),
            'axis_z': ParameterValue(pipe_center_z, value_type=float),
            'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'surface_length_m': ParameterValue(pipe_length_m, value_type=float),
            'target_axis_m': ParameterValue(target_x, value_type=float),
            'target_angle_deg': ParameterValue(target_angle_deg, value_type=float),
            'wait_for_goal': True,
            'approach_theta_deg': ParameterValue(approach_theta_deg, value_type=float),
            'approach_speed_mps': ParameterValue(approach_speed_mps, value_type=float),
            'axis_speed_mps': ParameterValue(axis_speed_mps, value_type=float),
            'curve_speed_mps': ParameterValue(curve_speed_mps, value_type=float),
            'attachment_settle_s': ParameterValue(attachment_settle_s, value_type=float),
            'boundary_hold_risk': ParameterValue(boundary_hold_risk, value_type=float),
            'friction_mu': ParameterValue(friction_mu, value_type=float),
            'robot_mass_kg': ParameterValue(robot_mass_kg, value_type=float),
            'safety_factor': ParameterValue(traction_safety_factor, value_type=float),
            'min_side_contact_fraction': ParameterValue(min_side_contact_fraction, value_type=float),
            'allow_guarded_low_contact': ParameterValue(allow_guarded_low_contact, value_type=bool),
            'wheel_torque_limit_nm': 40.0,
        }],
    )

    attachment_guard = TimerAction(
        period=6.5,
        condition=IfCondition(start_attachment_guard),
        actions=[Node(
            package='robot_3d3s',
            executable='surface_attachment_guard.py',
            name='surface_attachment_guard',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'enabled': True,
                'world_name': world_name,
                'model_name': 'robot_3d3s',
                'base_link_name': 'base_link',
                'world_frame': 'world',
                'base_frame': 'base_footprint',
                'status_topic': '/robot_3d3s/adhesion_status',
                'surface_mode': 'cylinder',
                'cylinder_axis': 'x',
                'axis_x': ParameterValue(pipe_center_x, value_type=float),
                'axis_y': ParameterValue(pipe_center_y, value_type=float),
                'axis_z': ParameterValue(pipe_center_z, value_type=float),
                'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
                'surface_length_m': ParameterValue(pipe_length_m, value_type=float),
                'base_surface_offset_m': ParameterValue(attachment_guard_offset_m, value_type=float),
                'radial_tolerance_m': ParameterValue(attachment_guard_tolerance_m, value_type=float),
                'max_correction_m': ParameterValue(attachment_guard_max_correction_m, value_type=float),
                'correction_method': attachment_guard_method,
                'position_correction_gain': ParameterValue(attachment_guard_position_gain, value_type=float),
                'orientation_correction_gain': ParameterValue(attachment_guard_orientation_gain, value_type=float),
                'attachment_stiffness_n_per_m': ParameterValue(attachment_guard_stiffness, value_type=float),
                'max_attachment_force_n': ParameterValue(attachment_guard_max_force, value_type=float),
                'orientation_stiffness_nm_per_rad': ParameterValue(attachment_guard_orientation_stiffness, value_type=float),
                'max_orientation_torque_nm': ParameterValue(attachment_guard_max_torque, value_type=float),
                'emergency_pose_error_m': ParameterValue(attachment_guard_emergency_error_m, value_type=float),
                'allow_emergency_pose_correction': False,
                'min_attached_wheels': 2,
                'geometric_capture': True,
                'capture_tolerance_m': ParameterValue(attachment_guard_capture_tolerance_m, value_type=float),
                'latch_after_contact': True,
                'latch_timeout_s': ParameterValue(attachment_guard_latch_timeout_s, value_type=float),
                'startup_hold_s': ParameterValue(startup_side_hold_s, value_type=float),
                'startup_hold_theta_deg': ParameterValue(side_start_theta_deg, value_type=float),
                'align_orientation': ParameterValue(attachment_guard_align_orientation, value_type=bool),
                'orientation_tolerance_deg': 3.0,
            }],
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
            'marker_topic': '/horizontal_pipe_markers',
            'point_cloud_topic': '/horizontal_pipe_pick_points',
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
            'cylinder_axis': 'x',
            'axis_x': ParameterValue(pipe_center_x, value_type=float),
            'axis_y': ParameterValue(pipe_center_y, value_type=float),
            'axis_z': ParameterValue(pipe_center_z, value_type=float),
            'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'surface_length_m': ParameterValue(pipe_length_m, value_type=float),
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
            '\nHORIZONTAL PIPE TEST: world ', world_name,
            ' contains ground_plane, horizontal_pipe, sun, and robot_3d3s. ',
            'The pipe radius is ', pipe_radius_m,
            ' m and the X-axis length is ', pipe_length_m,
            ' m. The robot spawn pose is x=', spawn_x,
            ', y=', spawn_y, ', z=', spawn_z,
            ', roll=', spawn_roll, ', pitch=', spawn_pitch,
            ', yaw=', spawn_yaw,
            ', side theta=', side_start_theta_deg,
            ' deg, startup side hold=', startup_side_hold_s, ' s',
            '. RViz uses the same spawn pose until live Gazebo TF arrives. ',
            'surface_goal_planner targets theta=', target_angle_deg,
            ' deg around the pipe while holding x=', target_x,
            '. The Gazebo KMW100 adhesion plugin applies pipe holding force every physics step; ',
            'the optional Python adhesion node is off by default for this scenario.\n'
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
        DeclareLaunchArgument('start_adhesion', default_value='false',
                              description='Start contact-only Python adhesion for the horizontal pipe. The Gazebo plugin handles holding force by default.'),
        DeclareLaunchArgument('start_attachment_guard', default_value='true',
                              description='Debug guard that keeps an attached robot projected onto the pipe surface.'),
        DeclareLaunchArgument('use_pipe_planner', default_value='true',
                              description='Enable unified surface goal planner.'),
        DeclareLaunchArgument('surface_graph_yaml', default_value='',
                              description='Optional YAML surface graph. Empty keeps configured cylinder behavior.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz for robot and pipe markers.'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='false',
                              description='Start Gazebo GUI. False runs Gazebo server only and visualizes in RViz.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='RViz configuration file.'),
        DeclareLaunchArgument('force_table_csv', default_value=default_force_table,
                              description='COMSOL-style KMW100 force lookup CSV.'),
        DeclareLaunchArgument('wall_thickness_mm', default_value=f'{defaults["wall_thickness_mm"]:.2f}',
                              description='Pipe wall thickness used for lookup.'),
        DeclareLaunchArgument('pipe_radius_m', default_value=f'{defaults["pipe_radius_m"]:.3f}',
                              description='Horizontal pipe radius in meters.'),
        DeclareLaunchArgument('pipe_center_x', default_value='0.0',
                              description='A point on the pipe axis, X coordinate.'),
        DeclareLaunchArgument('pipe_center_y', default_value='0.0',
                              description='Pipe axis Y coordinate.'),
        DeclareLaunchArgument('pipe_center_z', default_value=f'{defaults["pipe_center_z"]:.3f}',
                              description='Pipe axis Z coordinate. Matches the SDF center height.'),
        DeclareLaunchArgument('pipe_length_m', default_value='6.00',
                              description='Pipe length along world X in meters.'),
        DeclareLaunchArgument('target_x', default_value=f'{defaults["target_x"]:.2f}',
                              description='Planner target along the pipe axis.'),
        DeclareLaunchArgument('target_angle_deg', default_value=f'{defaults["target_angle_deg"]:.1f}',
                              description='Planner target angle around pipe; 90 deg is the top.'),
        DeclareLaunchArgument('approach_theta_deg', default_value=f'{defaults["side_start_theta_deg"]:.1f}',
                              description='Cylinder approach angle before confirmed contact; 0 deg is the +Y side, -90 deg is the bottom.'),
        DeclareLaunchArgument('approach_speed_mps', default_value='0.08',
                              description='Slow radial approach speed while waiting for confirmed pipe attachment.'),
        DeclareLaunchArgument('axis_speed_mps', default_value='0.08',
                              description='Max planner speed along the active surface axis.'),
        DeclareLaunchArgument('curve_speed_mps', default_value='0.14',
                              description='Max planner speed around the pipe or polygon local-v direction.'),
        DeclareLaunchArgument('adhesion_model', default_value='constant',
                              description='Adhesion model: constant, lookup, or gap_decay. Horizontal side-start defaults to constant for KMW100 rated-force tests.'),
        DeclareLaunchArgument('adhesion_force', default_value='900.0',
                              description='Minimum/fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('scale_constant_by_contact_fraction', default_value='false',
                              description='When false, constant mode uses full force_n whenever attached/captured instead of reducing by contact fraction.'),
        DeclareLaunchArgument('adhesion_min_force', default_value='150.0',
                              description='Minimum lookup force per wheel once contact or analytic capture is valid.'),
        DeclareLaunchArgument('adhesion_rate', default_value='40.0',
                              description='Contact-only adhesion update rate in Hz.'),
        DeclareLaunchArgument('contact_timeout_s', default_value='1.00',
                              description='How long a wheel contact remains fresh before adhesion is removed.'),
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
        DeclareLaunchArgument('analytic_capture_gap_mm', default_value='120.0',
                              description='Air gap where cylinder magnets can analytically capture before contact.'),
        DeclareLaunchArgument('attachment_guard_offset_m', default_value='0.09',
                              description='Base-frame radial offset from pipe visual radius while guard is active. Positive keeps the robot outside the pipe.'),
        DeclareLaunchArgument('attachment_guard_tolerance_m', default_value='0.012',
                              description='Radial drift allowed before the attachment guard corrects the pose.'),
        DeclareLaunchArgument('attachment_guard_max_correction_m', default_value='0.08',
                              description='Maximum radial correction per guard tick.'),
        DeclareLaunchArgument('attachment_guard_latch_timeout_s', default_value='3.0',
                              description='Seconds the guard remains active after transient contact loss.'),
        DeclareLaunchArgument('attachment_guard_capture_tolerance_m', default_value='0.25',
                              description='Allow the guard to capture the robot geometrically when its base is this close to the pipe radius.'),
        DeclareLaunchArgument('attachment_guard_align_orientation', default_value='true',
                              description='Align base +Z to the pipe radial normal while the debug guard is active.'),
        DeclareLaunchArgument('attachment_guard_position_gain', default_value='0.45',
                              description='Fraction of each attachment-guard position correction applied per tick. Lower values reduce visual jumping.'),
        DeclareLaunchArgument('attachment_guard_orientation_gain', default_value='0.35',
                              description='Fraction of each attachment-guard orientation correction applied per tick. Lower values reduce visual jumping.'),
        DeclareLaunchArgument('attachment_guard_method', default_value='wrench',
                              description='Attachment guard correction method: wrench is smooth, pose is a hard rescue/set-pose correction.'),
        DeclareLaunchArgument('attachment_guard_stiffness', default_value='5000.0',
                              description='Smooth attachment force spring stiffness in N/m for wrench mode.'),
        DeclareLaunchArgument('attachment_guard_max_force', default_value='900.0',
                              description='Maximum smooth attachment force applied to base_link in wrench mode.'),
        DeclareLaunchArgument('attachment_guard_orientation_stiffness', default_value='70.0',
                              description='Smooth alignment torque stiffness in Nm/rad for wrench mode.'),
        DeclareLaunchArgument('attachment_guard_max_torque', default_value='60.0',
                              description='Maximum smooth alignment torque in wrench mode.'),
        DeclareLaunchArgument('attachment_guard_emergency_error_m', default_value='0.22',
                              description='If radial error exceeds this distance, pose correction may rescue the robot.'),
        DeclareLaunchArgument('side_start_theta_deg', default_value=f'{defaults["side_start_theta_deg"]:.1f}',
                              description='Horizontal pipe side-start angle. For the X-axis pipe, 0 deg is +Y side and -90 deg is bottom.'),
        DeclareLaunchArgument('startup_side_hold_s', default_value=f'{defaults["startup_side_hold_s"]:.1f}',
                              description='Seconds after first live TF that the attachment guard holds the robot at side_start_theta_deg.'),
        DeclareLaunchArgument('attachment_settle_s', default_value='0.75',
                              description='Planner hold time after first stable pipe attachment before curving.'),
        DeclareLaunchArgument('boundary_hold_risk', default_value='0.78',
                              description='Planner stops when adhesion boundary risk reaches this value.'),
        DeclareLaunchArgument('friction_mu', default_value='1.0',
                              description='Wheel/surface friction coefficient used by the planner safety gate.'),
        DeclareLaunchArgument('robot_mass_kg', default_value=f'{defaults["robot_mass_kg"]:.1f}',
                              description='Robot mass used by planner traction/inertia margin estimates. Gazebo inertial masses still come from URDF.'),
        DeclareLaunchArgument('traction_safety_factor', default_value='1.5',
                              description='Safety factor for pipe-side tangential gravity load.'),
        DeclareLaunchArgument('min_side_contact_fraction', default_value='0.45',
                              description='Minimum average wheel contact fraction allowed on steep pipe targets.'),
        DeclareLaunchArgument('allow_guarded_low_contact', default_value='true',
                              description='Allow side-pipe motion with low contact fraction while the attachment guard is active.'),
        DeclareLaunchArgument('spawn_x', default_value=f'{defaults["spawn_x"]:.2f}',
                              description='Robot starting X, inside the pipe length.'),
        DeclareLaunchArgument('spawn_y', default_value=f'{defaults["spawn_y"]:.2f}',
                              description='Robot starting Y on the +Y side of the pipe.'),
        DeclareLaunchArgument('spawn_z', default_value=f'{defaults["spawn_z"]:.2f}',
                              description='Robot starting base_footprint Z at the pipe side-start height.'),
        DeclareLaunchArgument('spawn_roll', default_value=f'{defaults["spawn_roll"]:.4f}',
                              description='Robot starting roll in radians. +Y side of the horizontal pipe uses -pi/2.'),
        DeclareLaunchArgument('spawn_pitch', default_value=f'{defaults["spawn_pitch"]:.4f}',
                              description='Robot starting pitch in radians. +Y side of the horizontal pipe uses +pi/2.'),
        DeclareLaunchArgument('spawn_yaw', default_value=f'{defaults["spawn_yaw"]:.4f}',
                              description='Robot starting yaw in radians.'),
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        world_tf,
        gazebo_model_tf,
        contact_monitor,
        adhesion,
        attachment_guard,
        pipe_planner,
        pipe_grid,
        rviz,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
        step_startup,
        ready,
    ])
