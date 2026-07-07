#!/usr/bin/env python3
"""
Horizontal pipe curvature test.

This launch avoids the imported mockup mesh and generates a primitive steel
pipe from launch arguments. The robot starts on the +Y side of the pipe by
default, then surface_goal_planner.py can command motion around the circular
cross-section.
"""

import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULTS = {
    'wall_thickness_mm': 9.0,
    'pipe_radius_m': 1.20,
    'wheel_diameter_mm': 100.0,
    'wheel_width_m': 0.05,
    'pipe_center_z': 2.00,
    'spawn_x': -1.60,
    'spawn_y': 1.29,
    'spawn_z': 2.00,
    'spawn_roll': -1.5708,
    'spawn_pitch': 1.5708,
    'spawn_yaw': 0.0,
    'spawn_location': 'top',
    'side_start_theta_deg': 90.0,
    'side_start_heading_deg': 180.0,
    'startup_side_hold_s': 15.0,
    'robot_mass_kg': 25.0,
    'target_x': 1.60,
    'target_angle_deg': -90.0,
}


def _float_arg(context, launch_config):
    return float(launch_config.perform(context))


def _bool_arg(context, launch_config):
    return launch_config.perform(context).lower() in ('1', 'true', 'yes', 'on')


def _side_spawn_pose(radius_m, center_y, center_z, theta_deg, offset_m):
    theta = math.radians(theta_deg)
    radial_distance = radius_m + offset_m
    return (
        center_y + radial_distance * math.cos(theta),
        center_z + radial_distance * math.sin(theta),
    )


def _spawn_theta_deg(location, custom_theta_deg):
    location = str(location).strip().lower()
    if location == 'top':
        return 90.0
    if location == 'side':
        return 0.0
    if location == 'bottom':
        return -90.0
    return custom_theta_deg


def _normalize(vector):
    mag = math.sqrt(sum(value * value for value in vector))
    if mag <= 1e-12:
        return None
    return tuple(value / mag for value in vector)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _rpy_from_basis(x_axis, y_axis, z_axis):
    # Rotation matrix with basis vectors as columns, using Rz(yaw)*Ry(pitch)*Rx(roll).
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    sy = -m20
    if abs(sy) < 0.999999:
        pitch = math.asin(sy)
        roll = math.atan2(m21, m22)
        yaw = math.atan2(m10, m00)
    else:
        pitch = math.asin(sy)
        roll = 0.0
        yaw = math.atan2(-m01, m11)
    return roll, pitch, yaw


def _side_spawn_rpy(theta_deg, heading_offset_deg):
    theta = math.radians(theta_deg)
    heading = math.radians(heading_offset_deg)
    radial = (0.0, math.cos(theta), math.sin(theta))
    pipe_axis = (1.0, 0.0, 0.0)
    x_axis = _normalize(_cross(radial, pipe_axis))
    if x_axis is None:
        return 0.0, 0.0, 0.0
    z_axis = radial
    y_axis = _normalize(_cross(z_axis, x_axis))
    if y_axis is None:
        return 0.0, 0.0, 0.0
    c = math.cos(heading)
    s = math.sin(heading)
    x_axis, y_axis = (
        (
            c * x_axis[0] + s * y_axis[0],
            c * x_axis[1] + s * y_axis[1],
            c * x_axis[2] + s * y_axis[2],
        ),
        (
            -s * x_axis[0] + c * y_axis[0],
            -s * x_axis[1] + c * y_axis[1],
            -s * x_axis[2] + c * y_axis[2],
        ),
    )
    return _rpy_from_basis(x_axis, y_axis, z_axis)


def _horizontal_pipe_world_sdf(radius_m, length_m, center_x, center_y, center_z):
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="horizontal_pipe_test">
    <gravity>0 0 -9.8</gravity>
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>250</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>0.01</contact_max_correcting_vel>
          <contact_surface_layer>0.0001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <material>
            <ambient>0.45 0.45 0.45 1</ambient>
            <diffuse>0.45 0.45 0.45 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="horizontal_pipe">
      <pose>{center_x:.6f} {center_y:.6f} {center_z:.6f} 0 0 0</pose>
      <static>true</static>
      <link name="pipe_link">
        <collision name="pipe_collision">
          <pose>0 0 0 0 1.57079632679 0</pose>
          <geometry>
            <cylinder>
              <radius>{radius_m:.6f}</radius>
              <length>{length_m:.6f}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.6</mu>
                <mu2>1.6</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>250000</kp>
                <kd>80</kd>
                <max_vel>0.20</max_vel>
                <min_depth>0.006</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="pipe_visual">
          <pose>0 0 0 0 1.57079632679 0</pose>
          <geometry>
            <cylinder>
              <radius>{radius_m:.6f}</radius>
              <length>{length_m:.6f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.55 0.58 0.60 1</ambient>
            <diffuse>0.68 0.72 0.74 1</diffuse>
            <specular>0.35 0.35 0.35 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


def _configured_robot_description(
        urdf_file, controllers_yaml, pipe_center_x, pipe_center_y,
        pipe_center_z, pipe_radius_m, pipe_length_m, adhesion_model,
        adhesion_force_n, adhesion_min_force_n, force_table_csv,
        wall_thickness_mm, wheel_radius_m, wheel_width_m,
        gap_reference_mm, adhesion_max_gap_mm):
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)
    robot_desc = robot_desc.replace(
        '<adhesion_model>gap_decay</adhesion_model>',
        f'<adhesion_model>{adhesion_model}</adhesion_model>')
    robot_desc = robot_desc.replace(
        '<force_n>900.0</force_n>',
        f'<force_n>{adhesion_force_n:.6f}</force_n>')
    robot_desc = robot_desc.replace(
        '<min_force_n>0.0</min_force_n>',
        f'<min_force_n>{adhesion_min_force_n:.6f}</min_force_n>')
    robot_desc = robot_desc.replace(
        '<force_table_csv></force_table_csv>',
        f'<force_table_csv>{force_table_csv}</force_table_csv>')
    robot_desc = robot_desc.replace(
        '<wall_thickness_mm>10.0</wall_thickness_mm>',
        f'<wall_thickness_mm>{wall_thickness_mm:.6f}</wall_thickness_mm>')
    robot_desc = robot_desc.replace(
        '<wheel_radius_m>0.05</wheel_radius_m>',
        f'<wheel_radius_m>{wheel_radius_m:.6f}</wheel_radius_m>')
    robot_desc = robot_desc.replace(
        '<wheel_width_m>0.05</wheel_width_m>',
        f'<wheel_width_m>{wheel_width_m:.6f}</wheel_width_m>')
    robot_desc = robot_desc.replace(
        '<cylinder radius="0.05" length="0.05"/>',
        f'<cylinder radius="{wheel_radius_m:.6f}" length="{wheel_width_m:.6f}"/>')
    wheel_visual_scale = wheel_radius_m / 0.05 if wheel_radius_m > 0.0 else 1.0
    for wheel_idx in (1, 2, 3):
        robot_desc = robot_desc.replace(
            f'<mesh filename="package://robot_3d3s/meshes/wheel_{wheel_idx}_link.STL" />',
            (
                f'<mesh filename="package://robot_3d3s/meshes/wheel_{wheel_idx}_link.STL" '
                f'scale="{wheel_visual_scale:.6f} {wheel_visual_scale:.6f} '
                f'{wheel_visual_scale:.6f}" />'
            ))
    robot_desc = robot_desc.replace(
        '<gap_reference_mm>3.0</gap_reference_mm>',
        f'<gap_reference_mm>{gap_reference_mm:.6f}</gap_reference_mm>')
    robot_desc = robot_desc.replace(
        '<max_gap_m>0.06</max_gap_m>',
        f'<max_gap_m>{adhesion_max_gap_mm * 0.001:.6f}</max_gap_m>')
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
        '<surface_margin_m>0.08</surface_margin_m>',
        '<surface_margin_m>0.12</surface_margin_m>')
    robot_desc = robot_desc.replace(
        '<status_topic>/model/robot_3d3s/adhesion_status</status_topic>',
        '<status_topic>/robot_3d3s/adhesion_status</status_topic>')
    robot_desc = robot_desc.replace(
        '<pipe_center_x>0.0</pipe_center_x>',
        f'<pipe_center_x>{pipe_center_x:.6f}</pipe_center_x>')
    robot_desc = robot_desc.replace(
        '<pipe_center_y>0.0</pipe_center_y>',
        f'<pipe_center_y>{pipe_center_y:.6f}</pipe_center_y>')
    robot_desc = robot_desc.replace(
        '<pipe_center_z>2.00</pipe_center_z>',
        f'<pipe_center_z>{pipe_center_z:.6f}</pipe_center_z>')
    robot_desc = robot_desc.replace(
        '<pipe_radius_m>1.20</pipe_radius_m>',
        f'<pipe_radius_m>{pipe_radius_m:.6f}</pipe_radius_m>')
    robot_desc = robot_desc.replace(
        '<pipe_length_m>6.00</pipe_length_m>',
        f'<pipe_length_m>{pipe_length_m:.6f}</pipe_length_m>')
    return robot_desc


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    world_name = 'horizontal_pipe_test'
    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    default_force_table = os.path.join(pkg, 'config', 'kmw100_comsol_seed_table.csv')
    default_rviz_config = os.path.join(pkg, 'config', 'horizontal_pipe.rviz')
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
    wheel_diameter_mm = LaunchConfiguration('wheel_diameter_mm')
    wheel_width_m = LaunchConfiguration('wheel_width_m')
    pipe_radius_m = LaunchConfiguration('pipe_radius_m')
    pipe_center_x = LaunchConfiguration('pipe_center_x')
    pipe_center_y = LaunchConfiguration('pipe_center_y')
    pipe_center_z = LaunchConfiguration('pipe_center_z')
    pipe_length_m = LaunchConfiguration('pipe_length_m')
    auto_side_spawn = LaunchConfiguration('auto_side_spawn')
    spawn_location = LaunchConfiguration('spawn_location')
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
    adhesion_max_gap_mm = LaunchConfiguration('adhesion_max_gap_mm')
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
    side_start_heading_deg = LaunchConfiguration('side_start_heading_deg')
    attachment_settle_s = LaunchConfiguration('attachment_settle_s')
    boundary_hold_risk = LaunchConfiguration('boundary_hold_risk')
    min_side_contact_fraction = LaunchConfiguration('min_side_contact_fraction')
    allow_guarded_low_contact = LaunchConfiguration('allow_guarded_low_contact')
    traction_safety_factor = LaunchConfiguration('traction_safety_factor')
    goal_hold_steering_strategy = LaunchConfiguration('goal_hold_steering_strategy')
    goal_hold_publish_steering = LaunchConfiguration('goal_hold_publish_steering')
    friction_mu = LaunchConfiguration('friction_mu')
    robot_mass_kg = LaunchConfiguration('robot_mass_kg')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    def resolved_spawn_pose(context):
        spawn_x_value = _float_arg(context, spawn_x)
        spawn_y_value = _float_arg(context, spawn_y)
        spawn_z_value = _float_arg(context, spawn_z)
        if _bool_arg(context, auto_side_spawn):
            theta_deg = _spawn_theta_deg(
                spawn_location.perform(context),
                _float_arg(context, side_start_theta_deg))
            spawn_y_value, spawn_z_value = _side_spawn_pose(
                radius_m=_float_arg(context, pipe_radius_m),
                center_y=_float_arg(context, pipe_center_y),
                center_z=_float_arg(context, pipe_center_z),
                theta_deg=theta_deg,
                offset_m=_float_arg(context, attachment_guard_offset_m),
            )
            roll_value, pitch_value, yaw_value = _side_spawn_rpy(
                theta_deg=theta_deg,
                heading_offset_deg=_float_arg(context, side_start_heading_deg),
            )
        else:
            roll_value = _float_arg(context, spawn_roll)
            pitch_value = _float_arg(context, spawn_pitch)
            yaw_value = _float_arg(context, spawn_yaw)
        return (
            f'{spawn_x_value:.6f}',
            f'{spawn_y_value:.6f}',
            f'{spawn_z_value:.6f}',
            f'{roll_value:.6f}',
            f'{pitch_value:.6f}',
            f'{yaw_value:.6f}',
        )

    def launch_gazebo_with_configured_pipe(context):
        radius = _float_arg(context, pipe_radius_m)
        length = _float_arg(context, pipe_length_m)
        center_x = _float_arg(context, pipe_center_x)
        center_y = _float_arg(context, pipe_center_y)
        center_z = _float_arg(context, pipe_center_z)
        world_sdf = _horizontal_pipe_world_sdf(
            radius, length, center_x, center_y, center_z)
        world_path = (
            f'/tmp/robot_3d3s_horizontal_pipe_test_'
            f'r{radius:.3f}_l{length:.3f}_z{center_z:.3f}_{os.getpid()}.sdf')
        with open(world_path, 'w') as f:
            f.write(world_sdf)

        gz_args = (
            ('-r ' if run_sim.perform(context).lower() == 'true' else '') +
            ('-v 4 ' if use_gazebo_gui.perform(context).lower() == 'true' else '-s -v 4 ') +
            world_path)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        )]

    def spawn_configured_robot(context):
        radius = _float_arg(context, pipe_radius_m)
        length = _float_arg(context, pipe_length_m)
        center_x = _float_arg(context, pipe_center_x)
        center_y = _float_arg(context, pipe_center_y)
        center_z = _float_arg(context, pipe_center_z)
        spawn_x_value, spawn_y_value, spawn_z_value, roll, pitch, yaw = (
            resolved_spawn_pose(context))
        robot_desc = _configured_robot_description(
            urdf_file, controllers_yaml, center_x, center_y,
            center_z, radius, length,
            adhesion_model.perform(context),
            _float_arg(context, adhesion_force),
            _float_arg(context, adhesion_min_force),
            force_table_csv.perform(context),
            _float_arg(context, wall_thickness_mm),
            _float_arg(context, wheel_diameter_mm) * 0.0005,
            _float_arg(context, wheel_width_m),
            _float_arg(context, gap_reference_mm),
            _float_arg(context, adhesion_max_gap_mm))
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
            ),
            Node(
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
                    'fallback_x': float(spawn_x_value),
                    'fallback_y': float(spawn_y_value),
                    'fallback_z': float(spawn_z_value),
                    'fallback_roll': float(roll),
                    'fallback_pitch': float(pitch),
                    'fallback_yaw': float(yaw),
                }],
            ),
            LogInfo(msg=[
                'Resolved horizontal-pipe robot spawn: x=', spawn_x_value,
                ', y=', spawn_y_value, ', z=', spawn_z_value,
                ', roll=', roll, ', pitch=', pitch, ', yaw=', yaw,
                ', auto_side_spawn=', auto_side_spawn,
                ', radius=', pipe_radius_m,
                ', spawn location=', spawn_location,
                ', side theta=', side_start_theta_deg,
                ', heading offset=', side_start_heading_deg,
                ', offset=', attachment_guard_offset_m,
            ]),
            TimerAction(
                period=5.0,
                actions=[Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'robot_3d3s',
                        '-string', robot_desc,
                        '-x', spawn_x_value,
                        '-y', spawn_y_value,
                        '-z', spawn_z_value,
                        '-R', roll,
                        '-P', pitch,
                        '-Y', yaw,
                    ],
                    output='screen',
                )],
            ),
        ]

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
            'route_lookahead_m': 0.36,
            'goal_slowdown_distance_m': 0.90,
            'goal_hold_steering_strategy': goal_hold_steering_strategy,
            'goal_hold_publish_steering': ParameterValue(
                goal_hold_publish_steering, value_type=bool),
            'surface_cmd_smoothing_tau_s': 0.35,
            'body_heading_max_rate_rad_s': 0.65,
            'body_heading_deadband_rad': 0.15,
            'bayesian_posterior_tau_s': 0.35,
            'edge_margin_m': 0.18,
            'edge_slow_min_scale': 0.35,
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
                'surface_heading_offset_deg': ParameterValue(side_start_heading_deg, value_type=float),
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
            # Higher grid density plus an opaque solid CYLINDER hit target
            # make RViz Publish Point reliable while keeping the pipe visible.
            'axial_lines': 48,
            'rings': 64,
            'ring_segments': 192,
            'line_width_m': 0.014,
            'hit_target_enabled': True,
            'hit_target_alpha': 1.0,
            'surface_alpha': 0.0,
            'surface_axis_segments': 96,
            'surface_ring_segments': 224,
            'point_cloud_axis_samples': 112,
            'point_cloud_ring_samples': 224,
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
        DeclareLaunchArgument('start_attachment_guard', default_value='false',
                              description='Debug guard that keeps an attached robot projected onto the pipe surface. Keep false for physical fixed-wheel fall tests.'),
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
        DeclareLaunchArgument('wheel_diameter_mm', default_value=f'{defaults["wheel_diameter_mm"]:.1f}',
                              description='Magnetic wheel collision/adhesion diameter in mm. Use 100.0 for KWM100 or 120.0 for the larger wheel.'),
        DeclareLaunchArgument('wheel_width_m', default_value=f'{defaults["wheel_width_m"]:.3f}',
                              description='Wheel collision/adhesion width in meters.'),
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
        DeclareLaunchArgument('auto_side_spawn', default_value='true',
                              description='Compute spawn_y/spawn_z from pipe radius, center, side_start_theta_deg, and attachment_guard_offset_m. Set false to use manual spawn_y/spawn_z.'),
        DeclareLaunchArgument('spawn_location', default_value=defaults["spawn_location"],
                              description='Pipe spawn location for auto spawn: top, side, bottom, or custom. custom uses side_start_theta_deg.'),
        DeclareLaunchArgument('target_x', default_value=f'{defaults["target_x"]:.2f}',
                              description='Planner target along the pipe axis.'),
        DeclareLaunchArgument('target_angle_deg', default_value=f'{defaults["target_angle_deg"]:.1f}',
                              description='Planner target angle around pipe; 90 deg is the top and -90 deg is the bottom.'),
        DeclareLaunchArgument('approach_theta_deg', default_value=f'{defaults["side_start_theta_deg"]:.1f}',
                              description='Cylinder approach angle before confirmed contact; 0 deg is the +Y side, -90 deg is the bottom.'),
        DeclareLaunchArgument('approach_speed_mps', default_value='0.08',
                              description='Slow radial approach speed while waiting for confirmed pipe attachment.'),
        DeclareLaunchArgument('axis_speed_mps', default_value='0.25',
                              description='Max planner speed along the active surface axis.'),
        DeclareLaunchArgument('curve_speed_mps', default_value='0.25',
                              description='Max planner speed around the pipe or polygon local-v direction.'),
        DeclareLaunchArgument('adhesion_model', default_value='lookup',
                              description='Adhesion model: constant, lookup, or gap_decay. Lookup uses the configured KMW100 force table.'),
        DeclareLaunchArgument('adhesion_force', default_value='900.0',
                              description='Minimum/fallback adhesion force per wheel in Newtons.'),
        DeclareLaunchArgument('scale_constant_by_contact_fraction', default_value='false',
                              description='When false, constant mode uses full force_n whenever attached/captured instead of reducing by contact fraction.'),
        DeclareLaunchArgument('adhesion_min_force', default_value='0.0',
                              description='Minimum force floor per wheel. Keep 0.0 for physical fixed-wheel fall tests.'),
        DeclareLaunchArgument('adhesion_max_gap_mm', default_value='6.0',
                              description='Maximum wheel-to-surface air gap where the Gazebo adhesion plugin can apply force. Keep small for physical fixed-wheel fall tests.'),
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
        DeclareLaunchArgument('analytic_capture_gap_mm', default_value='0.0',
                              description='Air gap where cylinder magnets can analytically capture before contact.'),
        DeclareLaunchArgument('attachment_guard_offset_m', default_value='0.084',
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
        DeclareLaunchArgument('side_start_heading_deg', default_value=f'{defaults["side_start_heading_deg"]:.1f}',
                              description='Robot heading around the pipe normal at side spawn. 180 deg points the front of the 3-wheel footprint upward on the +Y side.'),
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
        DeclareLaunchArgument('goal_hold_steering_strategy', default_value='max_adhesion',
                              description='Steering hold after goal/stop: max_adhesion, least_vertical, or current. max_adhesion aligns wheel axles with pipe axis.'),
        DeclareLaunchArgument('goal_hold_publish_steering', default_value='true',
                              description='Publish zero-speed hold steering while attached so the robot settles into the configured hold strategy.'),
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
        OpaqueFunction(function=launch_gazebo_with_configured_pipe),
        OpaqueFunction(function=spawn_configured_robot),
        bridge,
        world_tf,
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
