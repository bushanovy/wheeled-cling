"""
nav2_wall.launch.py - Stage-2 edge-safe motion planning on a VERTICAL WALL.

Reuses the entire Stage-1 stack. The only differences from nav2_flat_ground:
  * the keepout rectangle is the 3 m x 3 m wall face (from wall_nav.yaml), and
  * the odom frame is PLACED on the wall (map->odom = Ry(-90) at the spawn point)
    so the keepout, the planned path, and the robot render on the actual wall.

Why this works with an unmodified Nav2: odom_node.py integrates body-frame
velocity into a planar pose, so `odom` is the wall-TANGENT plane while climbing
("forward" = up). Nav2 plans in odom exactly as on flat ground; the wall's top
and side edges are just the keepout boundary.

RViz-only mode (default - proves the edge-safe planner on the wall):
  ros2 launch robot_3d3s nav2_wall.launch.py
  Fixed frame "map". The red outline = the wall face. A Nav2 Goal inside it
  plans up/along the wall; a goal past the top/side edge is refused.

Gazebo climbing physics (adhesion + climb controller) is Stage 2b - see
nav2_swerve.launch.py gazebo:=true use_climb_controller:=true.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    nav2_overlay_file = os.path.join(pkg, 'config', 'nav2_flat_ground.yaml')  # generic keepout overlay
    scene_file = os.path.join(pkg, 'config', 'wall_nav.yaml')
    nav2_rviz_config = os.path.join(pkg, 'config', 'nav2.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    with open(scene_file, 'r') as f:
        scene = yaml.safe_load(f)
    surf = scene['surface']
    keep = scene['keepout']
    wp = scene['wall_plane']
    frame_id = scene.get('frame_id', 'odom')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen')

    odom_node = Node(
        package='robot_3d3s', executable='odom_node.py', name='odom_node',
        output='screen')

    # world -> map identity; map -> odom = place odom ON the wall (Ry(-90)).
    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'], output='screen')
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        arguments=['--x', str(wp['xyz'][0]), '--y', str(wp['xyz'][1]), '--z', str(wp['xyz'][2]),
                   '--roll', str(wp['rpy'][0]), '--pitch', str(wp['rpy'][1]), '--yaw', str(wp['rpy'][2]),
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen')

    keepout_publisher = Node(
        package='robot_3d3s', executable='surface_keepout_publisher.py',
        name='surface_keepout_publisher', output='screen',
        parameters=[{
            'frame_id': frame_id,
            'center_x': float(surf['center'][0]),
            'center_y': float(surf['center'][1]),
            'size_x': float(surf['size'][0]),
            'size_y': float(surf['size'][1]),
            'resolution': float(keep['resolution']),
            'margin': float(keep['margin']),
            'edge_standoff': float(keep.get('edge_standoff', 0.0)),
            'publish_period': float(keep.get('publish_period', 0.0)),
        }])

    params = [nav2_params_file, nav2_overlay_file, {'use_sim_time': False}]

    def nav(pkg_name, exe, extra_remaps=None, name=None):
        return Node(package=pkg_name, executable=exe, name=name, output='screen',
                    parameters=params, remappings=(extra_remaps or []))

    controller_server = nav('nav2_controller', 'controller_server',
                            [('cmd_vel', 'cmd_vel_nav')])
    smoother_server = nav('nav2_smoother', 'smoother_server')
    planner_server = nav('nav2_planner', 'planner_server')
    behavior_server = nav('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')])
    bt_navigator = nav('nav2_bt_navigator', 'bt_navigator')
    waypoint_follower = nav('nav2_waypoint_follower', 'waypoint_follower')
    velocity_smoother = nav('nav2_velocity_smoother', 'velocity_smoother',
                            [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
                            name='velocity_smoother')

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': [
            'controller_server', 'smoother_server', 'planner_server',
            'behavior_server', 'bt_navigator', 'waypoint_follower',
            'velocity_smoother']}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', nav2_rviz_config], parameters=[{'use_sim_time': False}],
        output='screen')

    cmd_vel_rviz = Node(
        package='robot_3d3s', executable='cmd_vel_to_wheels.py', name='cmd_vel_to_wheels',
        parameters=[{'gazebo': False}], output='screen')

    return LaunchDescription([
        robot_state_publisher, odom_node, world_to_map, map_to_odom,
        keepout_publisher,
        controller_server, smoother_server, planner_server, behavior_server,
        bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager,
        rviz, cmd_vel_rviz,
    ])
