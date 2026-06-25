#!/usr/bin/env python3
"""
nav2_wall_climb.launch.py - Stage-2b: edge-safe Nav2 on the REAL climbing robot.

Combines three things proven separately:
  * Gazebo wall-climb physics + magnetic adhesion (from flat_wall_climb.launch.py,
    whose world/adhesion/spawn generators are reused here via importlib - no copy),
  * the edge keepout (surface_keepout_publisher + nav2_flat_ground.yaml overlay),
  * the Nav2 -> climb_surface_controller cmd_vel pipeline.

Frames (the crux): odom_node gives a PLANAR odom->base_footprint (wall-tangent),
which Nav2 plans in. map->odom is the wall_plane transform Ry(-90) at the spawn
point, so world->...->base_link still carries the real -90 deg climbing pitch that
climb_surface_controller needs for its world->base mapping. We therefore do NOT run
climb_scene_rviz (it would publish a conflicting world->base_footprint TF).

Two safety layers:
  L1  Nav2 keepout StaticLayer -> planner cannot route off the wall (a priori).
  L2  climb_surface_controller RECOVERY_STOP when wheels detach / gap too large
      (reactive: an edge = wheels losing the surface).

Run it (GUI):
  ros2 launch robot_3d3s nav2_wall_climb.launch.py
  Press Play. In RViz use "Nav2 Goal" on the wall face (red outline). A goal past
  the top/side edge is refused; driving toward an edge that the robot reaches
  physically triggers RECOVERY_STOP.

Headless smoke test (no GUI):
  ros2 launch robot_3d3s nav2_wall_climb.launch.py headless:=true use_rviz:=false
"""

import importlib.util
import math
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def _load_helpers(pkg):
    """Import flat_wall_climb's world/adhesion/spawn helpers without copying them."""
    path = os.path.join(pkg, 'launch', 'flat_wall_climb.launch.py')
    spec = importlib.util.spec_from_file_location('flat_wall_climb_helpers', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('robot_3d3s')
    h = _load_helpers(pkg)

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    nav2_overlay_file = os.path.join(pkg, 'config', 'nav2_flat_ground.yaml')
    nav2_rviz_config = os.path.join(pkg, 'config', 'nav2.rviz')

    scene_path = LaunchConfiguration('scene_yaml').perform(context) \
        or os.path.join(pkg, 'config', 'climb_scene.yaml')
    wall_nav_path = os.path.join(pkg, 'config', 'wall_nav.yaml')

    with open(scene_path) as f:
        scene = yaml.safe_load(f)
    with open(wall_nav_path) as f:
        wnav = yaml.safe_load(f)

    world_name = scene.get('world_name', 'flat_wall')
    robot_name = scene.get('robot', {}).get('name', 'robot_3d3s')
    wall = scene['flat_wall']
    wp = [float(v) for v in wall['pose']]
    ws = [float(v) for v in wall['size']]
    face_x, _normal = h._wall_face_x_and_normal(wall)

    robot = scene.get('robot', {})
    preload = float(robot.get('preload_m', 0.0))
    # Spawn on the climbed face: body +X "forward" -> world +Z (up), pitch -90.
    spawn_x = face_x + preload
    spawn_y = wp[1]
    spawn_z = float(robot.get('start_height_z', wp[2]))
    spawn_pitch = -math.pi / 2.0

    # World SDF + adhesion plugin generated from the SAME scene YAML.
    world_sdf = h._build_world_sdf(scene)
    world_path = os.path.join(tempfile.gettempdir(), f'{world_name}_nav.sdf')
    with open(world_path, 'w') as f:
        f.write(world_sdf)

    with open(urdf_file) as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)
    adhesion_block = h._build_adhesion_plugin(scene, face_x, ws[0], robot_name)
    robot_desc = re.sub(
        r'<plugin\s+filename="librobot_3d3s_kmw100_adhesion_system\.so".*?</plugin>',
        lambda _m: adhesion_block, robot_desc, count=1, flags=re.DOTALL)

    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(
        p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), models_path] if p)
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(
        p for p in [os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), plugins_path] if p)

    run_sim = LaunchConfiguration('run_sim').perform(context) == 'true'
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    gz_flags = '-v 4'
    if run_sim:
        gz_flags = '-r ' + gz_flags
    if headless:
        gz_flags = '-s ' + gz_flags
    gz_args = f'{gz_flags} {world_path}'

    # ---- Gazebo + robot + bridges ----
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', robot_name, '-string', robot_desc,
                   '-x', f'{spawn_x}', '-y', f'{spawn_y}', '-z', f'{spawn_z}',
                   '-R', '0', '-P', f'{spawn_pitch}', '-Y', '0'])

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/model/{robot_name}/adhesion_status@std_msgs/msg/String[gz.msgs.StringMsg',
            '/robot_3d3s/wheel_1_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/robot_3d3s/wheel_3_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[(f'/model/{robot_name}/adhesion_status', '/robot_3d3s/adhesion_status')])

    # ---- TF: planar odom (Nav2) placed ON the wall via wall_plane ----
    odom_node = Node(package='robot_3d3s', executable='odom_node.py',
                     name='odom_node', output='screen',
                     parameters=[{'use_sim_time': True}])
    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'], output='screen')
    wpl = wnav['wall_plane']
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        arguments=['--x', str(wpl['xyz'][0]), '--y', str(wpl['xyz'][1]), '--z', str(wpl['xyz'][2]),
                   '--roll', str(wpl['rpy'][0]), '--pitch', str(wpl['rpy'][1]), '--yaw', str(wpl['rpy'][2]),
                   '--frame-id', 'map', '--child-frame-id', 'odom'], output='screen')

    # ---- Edge keepout (wall face) ----
    surf, keep = wnav['surface'], wnav['keepout']
    keepout_publisher = Node(
        package='robot_3d3s', executable='surface_keepout_publisher.py',
        name='surface_keepout_publisher', output='screen',
        parameters=[{
            'use_sim_time': True, 'frame_id': wnav.get('frame_id', 'odom'),
            'center_x': float(surf['center'][0]), 'center_y': float(surf['center'][1]),
            'size_x': float(surf['size'][0]), 'size_y': float(surf['size'][1]),
            'resolution': float(keep['resolution']), 'margin': float(keep['margin']),
            'edge_standoff': float(keep.get('edge_standoff', 0.0)),
            'publish_period': float(keep.get('publish_period', 0.0))}])

    # ---- Nav2 stack (with keepout overlay) ----
    params = [nav2_params_file, nav2_overlay_file, {'use_sim_time': True}]

    def nav(pkg_name, exe, remaps=None, name=None):
        return Node(package=pkg_name, executable=exe, name=name, output='screen',
                    parameters=params, remappings=(remaps or []))

    nav2_nodes = [
        nav('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav('nav2_smoother', 'smoother_server'),
        nav('nav2_planner', 'planner_server'),
        nav('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav('nav2_bt_navigator', 'bt_navigator'),
        nav('nav2_waypoint_follower', 'waypoint_follower'),
        nav('nav2_velocity_smoother', 'velocity_smoother',
            [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')], 'velocity_smoother'),
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': [
            'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
            'bt_navigator', 'waypoint_follower', 'velocity_smoother']}])

    # Start the whole Nav2 stack only AFTER TF + controllers are up. The costmaps
    # (global_frame odom, robot_base_frame base_footprint) need odom->base_footprint
    # to exist, which needs joint_state_broadcaster (spawned at 8 s) -> joint_states
    # -> odom_node. Bringing Nav2 up at t=0 makes the lifecycle manager race ahead of
    # TF: costmaps spew "Timed out waiting for transform", the controller fails to
    # configure/activate, and the terminal fills with errors. Delaying past the
    # controller spawners removes that race.
    nav2_start_delay = float(
        LaunchConfiguration('nav2_start_delay').perform(context))
    nav2_stack = TimerAction(
        period=nav2_start_delay, actions=[*nav2_nodes, lifecycle_manager])

    # ---- cmd_vel pipeline: Nav2 -> climb_surface_controller -> swerve ----
    cmd_vel_stamper = Node(
        package='robot_3d3s', executable='cmd_vel_stamper.py', name='cmd_vel_stamper',
        output='screen', remappings=[('cmd_vel_in', '/cmd_vel'),
                                      ('cmd_vel_out', '/climb_controller/cmd_vel_in')])
    climb_controller = Node(
        package='robot_3d3s', executable='climb_surface_controller.py',
        name='climb_surface_controller', output='screen',
        parameters=[{
            'use_sim_time': True,
            'cmd_in_topic': '/climb_controller/cmd_vel_in',
            'cmd_out_topic': '/swerve_controller/cmd_vel',
            'status_topic': '/robot_3d3s/adhesion_status',
            'world_frame': 'world', 'base_frame': 'base_link',
            'min_attached_wheels': 2, 'max_safe_gap_mm': 60.0,
            'climb_speed_limit_mps': 0.10, 'capture_speed_limit_mps': 0.05}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', nav2_rviz_config], parameters=[{'use_sim_time': True}], output='screen')

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager-timeout', '60'], output='screen')])

    # Step paused sim briefly so controllers can activate when run_sim:=false.
    step_startup = TimerAction(
        period=9.0, condition=IfCondition(LaunchConfiguration('auto_step_startup')),
        actions=[ExecuteProcess(cmd=[
            'gz', 'service', '-s', f'/world/{world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', 'multi_step: 80'], output='screen')])

    return [
        gz_sim, spawn_robot, robot_state_publisher, bridge,
        odom_node, world_to_map, map_to_odom, keepout_publisher,
        nav2_stack,
        cmd_vel_stamper, climb_controller,
        rviz,
        spawner('joint_state_broadcaster', 8.0),
        spawner('swerve_controller', 10.0),
        step_startup,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene_yaml', default_value='',
                              description='climb_scene.yaml path (defaults to package config).'),
        DeclareLaunchArgument('run_sim', default_value='true',
                              description='Start Gazebo running (true) or paused (false).'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='gz server only (no GUI) - for smoke tests.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 with the nav2 config.'),
        DeclareLaunchArgument('auto_step_startup', default_value='false',
                              description='Step paused Gazebo so controllers activate.'),
        DeclareLaunchArgument('nav2_start_delay', default_value='12.0',
                              description='Seconds to wait before starting the Nav2 '
                                          'stack, so TF + controllers are up first.'),
        OpaqueFunction(function=launch_setup),
    ])
