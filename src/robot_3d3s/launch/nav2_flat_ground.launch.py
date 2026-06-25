"""
nav2_flat_ground.launch.py - Stage-1 edge-safe motion planning on flat ground.

The robot navigates a finite traversable "plate" defined in flat_ground_nav.yaml.
surface_keepout_publisher.py turns that geometry into /keepout_map, and a Nav2
costmap StaticLayer (see nav2_flat_ground.yaml) marks everything beyond the plate
edges as lethal -> the planner cannot route the robot off the surface.

RViz-only mode (default - no physics, fast to iterate):
  ros2 launch robot_3d3s nav2_flat_ground.launch.py
  Set a "Nav2 Goal" inside the red outline -> robot drives there.
  Set one OUTSIDE the outline -> planner refuses / clamps to the edge.

Gazebo mode (real swerve physics on flat ground):
  ros2 launch robot_3d3s nav2_flat_ground.launch.py gazebo:=true

cmd_vel pipeline (matches nav2_swerve.launch.py):
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  RViz mode : /cmd_vel -> cmd_vel_to_wheels.py -> /joint_states (visualization)
  Gazebo    : /cmd_vel -> cmd_vel_stamper.py -> /swerve_controller/cmd_vel
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    nav2_overlay_file = os.path.join(pkg, 'config', 'nav2_flat_ground.yaml')
    scene_file = os.path.join(pkg, 'config', 'flat_ground_nav.yaml')
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    world_path = os.path.join(pkg, 'worlds', 'flat_ground.sdf')
    nav2_rviz_config = os.path.join(pkg, 'config', 'nav2.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc_gz = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)

    # flat_ground_nav.yaml is the single source of truth for plate + spawn.
    with open(scene_file, 'r') as f:
        scene = yaml.safe_load(f)
    surf = scene['surface']
    keep = scene['keepout']
    spawn = scene['robot']['spawn']
    frame_id = scene.get('frame_id', 'odom')

    gazebo = LaunchConfiguration('gazebo')
    gazebo_arg = DeclareLaunchArgument(
        'gazebo', default_value='false',
        description='true = Gazebo physics, false = RViz-only visualization')

    # ----- shared nodes -----
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    odom_node = Node(
        package='robot_3d3s', executable='odom_node.py', name='odom_node',
        output='screen')

    # Nav2 needs a map frame; keep it identical to odom (odom-only navigation).
    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'], output='screen')
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'], output='screen')

    keepout_publisher = Node(
        package='robot_3d3s', executable='surface_keepout_publisher.py',
        name='surface_keepout_publisher', output='screen',
        parameters=[{
            'use_sim_time': True,
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

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    params = [nav2_params_file, nav2_overlay_file, {'use_sim_time': True}]

    controller_server = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=params, remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')])
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', output='screen',
        parameters=params, remappings=tf_remaps)
    planner_server = Node(
        package='nav2_planner', executable='planner_server', output='screen',
        parameters=params, remappings=tf_remaps)
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', output='screen',
        parameters=params, remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')])
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', output='screen',
        parameters=params, remappings=tf_remaps)
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', output='screen',
        parameters=params, remappings=tf_remaps)
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=params,
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': [
            'controller_server', 'smoother_server', 'planner_server',
            'behavior_server', 'bt_navigator', 'waypoint_follower',
            'velocity_smoother']}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': True}], output='screen')

    # ----- RViz-only mode: visualize /cmd_vel by integrating to joint_states -----
    cmd_vel_rviz = Node(
        package='robot_3d3s', executable='cmd_vel_to_wheels.py', name='cmd_vel_to_wheels',
        parameters=[{'gazebo': False}], output='screen',
        condition=UnlessCondition(gazebo))

    # ----- Gazebo mode -----
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
        condition=IfCondition(gazebo))

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'robot_3d3s', '-string', robot_desc_gz,
                   '-x', str(spawn[0]), '-y', str(spawn[1]), '-z', str(spawn[2])],
        output='screen', condition=IfCondition(gazebo))

    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen', condition=IfCondition(gazebo))

    cmd_vel_stamper = Node(
        package='robot_3d3s', executable='cmd_vel_stamper.py', name='cmd_vel_stamper',
        output='screen', remappings=[('cmd_vel_in', '/cmd_vel'),
                                      ('cmd_vel_out', '/swerve_controller/cmd_vel')],
        condition=IfCondition(gazebo))

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager-timeout', '30'],
            output='screen', condition=IfCondition(gazebo))])

    return LaunchDescription([
        gazebo_arg,
        robot_state_publisher, odom_node, world_to_map, map_to_odom,
        keepout_publisher,
        controller_server, smoother_server, planner_server, behavior_server,
        bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager,
        rviz,
        cmd_vel_rviz,
        gz_sim, spawn_robot, gz_bridge, cmd_vel_stamper,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
    ])
