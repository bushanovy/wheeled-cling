#!/usr/bin/env python3
"""Experimental polygon pipe test.

Loads the measured lab polygon as primitive Gazebo geometry and starts the
robot on the outer -X side of the left vertical pipe.  The adhesion plugin and
surface planner are configured for that left vertical pipe by default, while
the full polygon remains present for collision and visual context.
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
    'horizontal_wall_thickness_mm': 6.0,
    'vertical_wall_thickness_mm': 9.5,
    'pipe_radius_m': 0.4572,
    'pipe_length_m': 3.000,
    'pipe_center_x': -1.5656,
    'pipe_center_y': 0.0,
    'pipe_center_z': 1.500,
    'spawn_x': -1.9928,
    'spawn_y': 0.0,
    'spawn_z': 0.700,
    'spawn_roll': -1.5708,
    'spawn_pitch': 0.0,
    'spawn_yaw': 1.5708,
    'side_start_theta_deg': 180.0,
    'startup_side_hold_s': 15.0,
    'target_x': 1.40,
    'target_angle_deg': 180.0,
    'robot_mass_kg': 25.0,
}


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_name = 'experimental_polygon_test'
    world_path = os.path.join(pkg, 'worlds', 'experimental_polygon_test.sdf')
    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    default_force_table = os.path.join(pkg, 'config', 'kmw100_comsol_seed_table.csv')
    default_rviz_config = os.path.join(pkg, 'config', 'display.rviz')
    default_surface_graph_yaml = os.path.join(pkg, 'config', 'experimental_polygon_graph.yaml')
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
        '<max_gap_m>0.14</max_gap_m>')
    robot_desc = robot_desc.replace(
        '<corner_capture_m>0.08</corner_capture_m>',
        '<corner_capture_m>0.10</corner_capture_m>')
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
        f'<pipe_center_x>{defaults["pipe_center_x"]:.3f}</pipe_center_x>')
    robot_desc = robot_desc.replace(
        '<pipe_center_y>0.0</pipe_center_y>',
        f'<pipe_center_y>{defaults["pipe_center_y"]:.3f}</pipe_center_y>')
    robot_desc = robot_desc.replace(
        '<pipe_center_z>2.00</pipe_center_z>',
        f'<pipe_center_z>{defaults["pipe_center_z"]:.3f}</pipe_center_z>')
    robot_desc = robot_desc.replace(
        '<pipe_radius_m>1.20</pipe_radius_m>',
        f'<pipe_radius_m>{defaults["pipe_radius_m"]:.3f}</pipe_radius_m>')
    robot_desc = robot_desc.replace(
        '<pipe_length_m>6.00</pipe_length_m>',
        f'<pipe_length_m>{defaults["pipe_length_m"]:.5f}</pipe_length_m>')
    robot_desc = robot_desc.replace(
        '<surface_margin_m>0.08</surface_margin_m>',
        '<surface_margin_m>0.08</surface_margin_m>')
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
    use_surface_planner = LaunchConfiguration('use_surface_planner')
    surface_graph_yaml = LaunchConfiguration('surface_graph_yaml')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    rviz_config = LaunchConfiguration('rviz_config')
    force_table_csv = LaunchConfiguration('force_table_csv')
    wall_thickness_mm = LaunchConfiguration('wall_thickness_mm')
    vertical_wall_thickness_mm = LaunchConfiguration('vertical_wall_thickness_mm')
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
    adhesion_rate = LaunchConfiguration('adhesion_rate')
    contact_timeout_s = LaunchConfiguration('contact_timeout_s')
    analytic_capture_gap_mm = LaunchConfiguration('analytic_capture_gap_mm')
    full_contact_depth_m = LaunchConfiguration('full_contact_depth_m')
    full_contact_points = LaunchConfiguration('full_contact_points')
    min_contact_fraction = LaunchConfiguration('min_contact_fraction')
    attachment_guard_method = LaunchConfiguration('attachment_guard_method')
    startup_side_hold_s = LaunchConfiguration('startup_side_hold_s')
    side_start_theta_deg = LaunchConfiguration('side_start_theta_deg')
    attachment_settle_s = LaunchConfiguration('attachment_settle_s')
    boundary_hold_risk = LaunchConfiguration('boundary_hold_risk')
    friction_mu = LaunchConfiguration('friction_mu')
    traction_safety_factor = LaunchConfiguration('traction_safety_factor')
    min_side_contact_fraction = LaunchConfiguration('min_side_contact_fraction')
    allow_guarded_low_contact = LaunchConfiguration('allow_guarded_low_contact')
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
        name='experimental_polygon_contact_monitor',
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
            'cylinder_axis': 'z',
            'axis_x': ParameterValue(pipe_center_x, value_type=float),
            'axis_y': ParameterValue(pipe_center_y, value_type=float),
            'axis_z': ParameterValue(pipe_center_z, value_type=float),
            'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'adhesion_model': 'lookup',
            'force_n': ParameterValue(adhesion_force, value_type=float),
            'min_force_n': 0.0,
            'force_table_csv': force_table_csv,
            'wall_thickness_mm': ParameterValue(wall_thickness_mm, value_type=float),
            'pipe_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'rate_hz': ParameterValue(adhesion_rate, value_type=float),
            'contact_only': True,
            'magnetic_surface_allowlist': 'experimental_polygon',
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
            'legacy_debug_topic': '/horizontal_pipe_planner/debug',
            'hold_steering_topic': '/swerve_controller/hold_steering',
            'show_experimental_polygon': True,
            'experimental_polygon_goal_selection': True,
            'surface_graph_yaml': surface_graph_yaml,
            'surface_mode': 'cylinder',
            'cylinder_axis': 'z',
            'axis_x': ParameterValue(pipe_center_x, value_type=float),
            'axis_y': ParameterValue(pipe_center_y, value_type=float),
            'axis_z': ParameterValue(pipe_center_z, value_type=float),
            'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'surface_length_m': ParameterValue(pipe_length_m, value_type=float),
            'target_axis_m': ParameterValue(target_x, value_type=float),
            'target_angle_deg': ParameterValue(target_angle_deg, value_type=float),
            'wait_for_goal': True,
            'approach_axis_m': ParameterValue(spawn_z, value_type=float),
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
            'wheel_radius_m': 0.05,
            'wheel_goal_projection': True,
            'wheel_steering_radius_m': 0.38905,
            'wheel_torque_limit_nm': 40.0,
        }],
    )

    attachment_guard = Node(
        package='robot_3d3s',
        executable='surface_attachment_guard.py',
        name='surface_attachment_guard',
        condition=IfCondition(start_attachment_guard),
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
            'experimental_polygon_surface_selection': True,
            'cylinder_axis': 'z',
            'axis_x': ParameterValue(pipe_center_x, value_type=float),
            'axis_y': ParameterValue(pipe_center_y, value_type=float),
            'axis_z': ParameterValue(pipe_center_z, value_type=float),
            'surface_radius_m': ParameterValue(pipe_radius_m, value_type=float),
            'surface_length_m': ParameterValue(pipe_length_m, value_type=float),
            'base_surface_offset_m': -0.03,
            'radial_tolerance_m': 0.010,
            'max_correction_m': 0.08,
            'correction_method': attachment_guard_method,
            'position_correction_gain': 0.45,
            'orientation_correction_gain': 0.35,
            'attachment_stiffness_n_per_m': 2500.0,
            'attachment_damping_n_per_mps': 650.0,
            'max_attachment_force_n': 700.0,
            'orientation_stiffness_nm_per_rad': 25.0,
            'max_orientation_torque_nm': 20.0,
            'emergency_pose_error_m': 0.22,
            'min_attached_wheels': 2,
            'geometric_capture': True,
            'capture_tolerance_m': 0.35,
            'latch_after_contact': True,
            'latch_timeout_s': 3.0,
            'startup_hold_s': ParameterValue(startup_side_hold_s, value_type=float),
            'startup_hold_theta_deg': ParameterValue(side_start_theta_deg, value_type=float),
            'align_orientation': True,
            'orientation_tolerance_deg': 3.0,
        }],
    )

    scene_markers = Node(
        package='robot_3d3s',
        executable='experimental_polygon_rviz.py',
        name='experimental_polygon_rviz',
        condition=IfCondition(use_rviz),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_frame': 'world',
            'marker_topic': '/experimental_polygon_markers',
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
            '\nEXPERIMENTAL POLYGON TEST: world ', world_name,
            ' contains the solid dark-red no-grid polygon. Vertical pipes: OD 914.40 mm, ',
            'wall thickness ', vertical_wall_thickness_mm,
            ' mm. Horizontal pipes: OD 254.00 mm, wall thickness ',
            f'{defaults["horizontal_wall_thickness_mm"]:.2f}',
            ' mm. Active left vertical pipe center z=', pipe_center_z,
            ' m, radius=', pipe_radius_m, ' m, length=', pipe_length_m,
            ' m. Robot side-start pose is x=', spawn_x, ', y=', spawn_y,
            ', z=', spawn_z, ', roll=', spawn_roll, ', pitch=', spawn_pitch,
            ', yaw=', spawn_yaw, '. Planner target x=', target_x,
            ', theta=', target_angle_deg,
            ' deg. RViz Publish Point is projected to the nearest reachable wheel',
            ' steering axis using 389.05 mm wheel spacing and 100 mm wheels.\n'
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
                              description='Start optional Python lookup adhesion. The Gazebo plugin handles holding force by default.'),
        DeclareLaunchArgument('start_attachment_guard', default_value='true',
                              description='Debug guard that keeps an attached robot projected onto the left vertical pipe.'),
        DeclareLaunchArgument('use_surface_planner', default_value='true',
                              description='Enable unified surface goal planner.'),
        DeclareLaunchArgument('surface_graph_yaml',
                              default_value=default_surface_graph_yaml,
                              description='Surface graph YAML for the planner. '
                              'Set to empty string to use the built-in experimental '
                              'polygon graph; the graph can be swapped at runtime by '
                              'changing this parameter or calling '
                              '~/reload_surface_graph.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz for robot and pipe markers.'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='false',
                              description='Start Gazebo GUI. False runs Gazebo server only and visualizes in RViz.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='RViz configuration file.'),
        DeclareLaunchArgument('force_table_csv', default_value=default_force_table,
                              description='COMSOL-style KMW100 force lookup CSV.'),
        DeclareLaunchArgument('wall_thickness_mm',
                              default_value=f'{defaults["vertical_wall_thickness_mm"]:.2f}',
                              description='Active left vertical pipe wall thickness used for adhesion lookup, in mm.'),
        DeclareLaunchArgument('vertical_wall_thickness_mm',
                              default_value=f'{defaults["vertical_wall_thickness_mm"]:.2f}',
                              description='Vertical pipe wall thickness for the polygon metadata/log, in mm.'),
        DeclareLaunchArgument('pipe_radius_m', default_value=f'{defaults["pipe_radius_m"]:.3f}',
                              description='Active left vertical pipe radius in meters.'),
        DeclareLaunchArgument('pipe_center_x', default_value=f'{defaults["pipe_center_x"]:.3f}',
                              description='A point on the left vertical pipe axis, X coordinate.'),
        DeclareLaunchArgument('pipe_center_y', default_value=f'{defaults["pipe_center_y"]:.3f}',
                              description='Left vertical pipe axis Y coordinate.'),
        DeclareLaunchArgument('pipe_center_z', default_value=f'{defaults["pipe_center_z"]:.3f}',
                              description='Left vertical pipe axis center Z coordinate.'),
        DeclareLaunchArgument('pipe_length_m', default_value=f'{defaults["pipe_length_m"]:.5f}',
                              description='Left vertical pipe height in meters.'),
        DeclareLaunchArgument('target_x', default_value=f'{defaults["target_x"]:.2f}',
                              description='Planner target height along the vertical pipe axis.'),
        DeclareLaunchArgument('target_angle_deg', default_value=f'{defaults["target_angle_deg"]:.1f}',
                              description='Planner target angle around pipe; 90 deg is the top.'),
        DeclareLaunchArgument('approach_theta_deg', default_value=f'{defaults["side_start_theta_deg"]:.1f}',
                              description='Cylinder approach angle before confirmed contact; 180 deg is the outer -X side of the left pipe.'),
        DeclareLaunchArgument('approach_speed_mps', default_value='0.06',
                              description='Radial approach speed while waiting for confirmed polygon pipe attachment.'),
        DeclareLaunchArgument('axis_speed_mps', default_value='0.08',
                              description='Max planner speed along the active surface axis.'),
        DeclareLaunchArgument('curve_speed_mps', default_value='0.12',
                              description='Max planner speed around the pipe or polygon local-v direction.'),
        DeclareLaunchArgument('adhesion_force', default_value='900.0',
                              description='Fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('adhesion_rate', default_value='40.0',
                              description='Optional Python adhesion update rate in Hz.'),
        DeclareLaunchArgument('contact_timeout_s', default_value='1.00',
                              description='How long a wheel contact remains fresh before Python adhesion is removed.'),
        DeclareLaunchArgument('analytic_capture_gap_mm', default_value='150.0',
                              description='Air gap where cylinder magnets can analytically capture before contact.'),
        DeclareLaunchArgument('full_contact_depth_m', default_value='0.002',
                              description='Contact depth treated as full magnetic contact.'),
        DeclareLaunchArgument('full_contact_points', default_value='4',
                              description='Contact point count treated as full magnetic contact.'),
        DeclareLaunchArgument('min_contact_fraction', default_value='0.10',
                              description='Minimum fraction for fresh magnetic contact.'),
        DeclareLaunchArgument('attachment_guard_method', default_value='pose',
                              description='Attachment guard correction method: pose is the stable default; wrench is optional for smooth force-only tests.'),
        DeclareLaunchArgument('side_start_theta_deg', default_value=f'{defaults["side_start_theta_deg"]:.1f}',
                              description='Vertical pipe side-start angle. 180 deg is the outer -X side of the left pipe.'),
        DeclareLaunchArgument('startup_side_hold_s', default_value=f'{defaults["startup_side_hold_s"]:.1f}',
                              description='Seconds after first live TF that the attachment guard holds the robot at side_start_theta_deg.'),
        DeclareLaunchArgument('attachment_settle_s', default_value='0.75',
                              description='Planner hold time after first stable pipe attachment before curving.'),
        DeclareLaunchArgument('boundary_hold_risk', default_value='0.78',
                              description='Planner stops when adhesion boundary risk reaches this value.'),
        DeclareLaunchArgument('friction_mu', default_value='1.0',
                              description='Wheel/surface friction coefficient used by the planner safety gate.'),
        DeclareLaunchArgument('robot_mass_kg', default_value=f'{defaults["robot_mass_kg"]:.1f}',
                              description='Robot mass used by planner traction/inertia margin estimates.'),
        DeclareLaunchArgument('traction_safety_factor', default_value='1.5',
                              description='Safety factor for pipe-side tangential gravity load.'),
        DeclareLaunchArgument('min_side_contact_fraction', default_value='0.35',
                              description='Minimum average wheel contact fraction allowed on steep pipe targets.'),
        DeclareLaunchArgument('allow_guarded_low_contact', default_value='true',
                              description='Allow side-pipe motion with low contact fraction while the attachment guard is active.'),
        DeclareLaunchArgument('spawn_x', default_value=f'{defaults["spawn_x"]:.2f}',
                              description='Robot starting X in the wheel-contact band on the outer side wall of the left vertical pipe.'),
        DeclareLaunchArgument('spawn_y', default_value=f'{defaults["spawn_y"]:.3f}',
                              description='Robot starting Y on the left vertical pipe side wall.'),
        DeclareLaunchArgument('spawn_z', default_value=f'{defaults["spawn_z"]:.3f}',
                              description='Robot starting base_footprint Z along the left vertical pipe.'),
        DeclareLaunchArgument('spawn_roll', default_value=f'{defaults["spawn_roll"]:.4f}',
                              description='Robot starting roll in radians for the -X side of the vertical pipe.'),
        DeclareLaunchArgument('spawn_pitch', default_value=f'{defaults["spawn_pitch"]:.4f}',
                              description='Robot starting pitch in radians for the -X side of the vertical pipe.'),
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
        scene_markers,
        surface_planner,
        rviz,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
        step_startup,
        ready,
    ])
