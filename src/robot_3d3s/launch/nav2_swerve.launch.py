"""
nav2_swerve.launch.py  -  Navigation2 launch for the 3D3S robot using swerve_controller.

Launches the full navigation stack in either RViz-only or Gazebo mode.

RViz mode (default - no physics, pure visualization):
  ros2 launch robot_3d3s nav2_swerve.launch.py

Gazebo mode (real physics):
  ros2 launch robot_3d3s nav2_swerve.launch.py gazebo:=true

What this launches
------------------
Both modes:
  - robot_state_publisher    (URDF -> TF)
  - odom_node                (/joint_states -> /odom + TF odom->base_footprint)
  - nav2 nodes               (planner, controller, bt_navigator, ...)
  - RViz2                    (with nav2 panel for setting goals)
  - static TF: map -> odom   (identity - nav2 needs the map frame)

RViz mode only:
  - cmd_vel_to_wheels        (/cmd_vel -> /joint_states visualization)

Gazebo only:
  - Gazebo Sim               (physics)
  - ros2_control controllers (joint_state_broadcaster + swerve_controller)
  - ros_gz_bridge            (/clock and optional adhesion status)
  - cmd_vel_stamper.py       (/cmd_vel Twist -> /swerve_controller/cmd_vel TwistStamped)

cmd_vel pipeline in Gazebo mode
-------------------------------
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  -> cmd_vel_stamper.py -> /swerve_controller/cmd_vel -> swerve_controller -> robot

With use_climb_controller:=true:
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  -> cmd_vel_stamper.py -> /climb_controller/cmd_vel_in
  -> climb_surface_controller.py -> /swerve_controller/cmd_vel
  -> swerve_controller -> robot

This keeps swerve_controller.yaml compatible with:
  use_stamped_vel: true
"""

import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prefixed_robot_description(robot_description, prefix):
    root = ET.fromstring(robot_description)
    root.set('name', f"{root.get('name', 'robot')}_visual")

    for link in root.findall('.//link'):
        if 'name' in link.attrib:
            link.set('name', prefix + link.get('name'))

    for joint in root.findall('.//joint'):
        if 'name' in joint.attrib:
            joint.set('name', prefix + joint.get('name'))
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and 'link' in parent.attrib:
            parent.set('link', prefix + parent.get('link'))
        if child is not None and 'link' in child.attrib:
            child.set('link', prefix + child.get('link'))

    for gazebo in root.findall('.//gazebo'):
        if 'reference' in gazebo.attrib:
            gazebo.set('reference', prefix + gazebo.get('reference'))

    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    urdf_file = os.path.join(pkg, 'urdf', 'robot_3d3s.urdf')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    world_path = os.path.join(pkg, 'worlds', 'my_world.sdf')
    nav2_rviz_config = os.path.join(pkg, 'config', 'nav2.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc_gz = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)
    visual_prefix = 'climb_visual_'
    robot_desc_visual = _prefixed_robot_description(robot_desc, visual_prefix)

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='true = use Gazebo physics, false = RViz-only',
    )
    use_climb_controller_arg = DeclareLaunchArgument(
        'use_climb_controller',
        default_value='false',
        description='Route Nav2 velocity through climb_surface_controller in Gazebo mode',
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.2')
    spawn_roll_arg = DeclareLaunchArgument('spawn_roll', default_value='0.0')
    spawn_pitch_arg = DeclareLaunchArgument('spawn_pitch', default_value='0.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    slide_angle_arg = DeclareLaunchArgument(
        'slide_angle_deg',
        default_value='45.0',
        description='Ramp angle used by climb_surface_controller',
    )
    gazebo = LaunchConfiguration('gazebo')
    use_climb_controller = LaunchConfiguration('use_climb_controller')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    slide_angle_deg = LaunchConfiguration('slide_angle_deg')
    direct_gazebo_cmd_condition = IfCondition(PythonExpression([
        "'", gazebo, "' == 'true' and '", use_climb_controller, "' == 'false'"
    ]))
    climb_gazebo_cmd_condition = IfCondition(PythonExpression([
        "'", gazebo, "' == 'true' and '", use_climb_controller, "' == 'true'"
    ]))

    # Shared nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    climb_visual_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='climb_visual',
        parameters=[{
            'robot_description': robot_desc_visual,
            'use_sim_time': True,
        }],
        remappings=[
            ('joint_states', '/joint_states_visual'),
        ],
        output='screen',
        condition=climb_gazebo_cmd_condition,
    )

    odom_node = Node(
        package='robot_3d3s',
        executable='odom_node.py',
        name='odom_node',
        output='screen',
    )

    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    params = [nav2_params_file, {'use_sim_time': True}]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    route_server = Node(
        package='nav2_route',
        executable='route_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=params,
        remappings=tf_remaps + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    cmd_vel_stamper_direct = Node(
        package='robot_3d3s',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper_direct',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/swerve_controller/cmd_vel'),
        ],
        condition=direct_gazebo_cmd_condition,
    )

    cmd_vel_stamper_climb = Node(
        package='robot_3d3s',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper_climb',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/climb_controller/cmd_vel_in'),
        ],
        condition=climb_gazebo_cmd_condition,
    )

    climb_controller = Node(
        package='robot_3d3s',
        executable='climb_surface_controller.py',
        name='climb_surface_controller',
        output='screen',
        condition=climb_gazebo_cmd_condition,
        parameters=[{
            'use_sim_time': True,
            'cmd_in_topic': '/climb_controller/cmd_vel_in',
            'cmd_out_topic': '/swerve_controller/cmd_vel',
            'status_topic': '/robot_3d3s/adhesion_status',
            'world_frame': 'world',
            'base_frame': 'base_link',
            'slide_angle_deg': ParameterValue(slide_angle_deg, value_type=float),
            'min_attached_wheels': 2,
            'max_safe_gap_mm': 60.0,
            'status_timeout_s': 1.0,
            'tf_timeout_s': 0.3,
            'cmd_timeout_s': 2.0,
            'wall_confirm_s': 0.2,
            'capture_speed_limit_mps': 0.05,
            'climb_speed_limit_mps': 0.10,
        }],
    )

    climb_rviz_goal_bridge = Node(
        package='robot_3d3s',
        executable='climb_rviz_goal_bridge.py',
        name='climb_rviz_goal_bridge',
        output='screen',
        condition=climb_gazebo_cmd_condition,
        parameters=[{
            'use_sim_time': True,
            'marker_frame': 'odom',
            'goal_frame': 'odom',
            'clicked_point_topic': '/clicked_point',
            'goal_topic': '/goal_pose',
            'marker_topic': '/climb_surface_markers',
            'spawn_x': ParameterValue(spawn_x, value_type=float),
            'spawn_y': ParameterValue(spawn_y, value_type=float),
            'spawn_z': ParameterValue(spawn_z, value_type=float),
            'slide_start_x': -0.15,
            'slide_start_z': 0.0,
            'slide_wall_x': 0.95,
            'slide_angle_deg': ParameterValue(slide_angle_deg, value_type=float),
            'panel_center_x': 1.0,
            'panel_center_z': 1.5,
            'panel_thickness': 0.10,
            'panel_width': 3.0,
            'panel_height': 3.0,
        }],
    )

    climb_rviz_robot_visualizer = Node(
        package='robot_3d3s',
        executable='climb_rviz_robot_visualizer.py',
        name='climb_rviz_robot_visualizer',
        output='screen',
        condition=climb_gazebo_cmd_condition,
        parameters=[{
            'use_sim_time': True,
            'visual_prefix': visual_prefix,
            'robot_description': robot_desc_visual,
            'robot_description_topic': '/robot_description_visual',
            'joint_states_in': '/joint_states',
            'joint_states_out': '/joint_states_visual',
            'odom_topic': '/odom',
            'parent_frame': 'odom',
            'spawn_x': ParameterValue(spawn_x, value_type=float),
            'spawn_y': ParameterValue(spawn_y, value_type=float),
            'spawn_z': ParameterValue(spawn_z, value_type=float),
            'slide_start_x': -0.15,
            'slide_start_z': 0.0,
            'slide_wall_x': 0.95,
            'slide_angle_deg': ParameterValue(slide_angle_deg, value_type=float),
            'panel_center_z': 1.5,
            'panel_height': 3.0,
        }],
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=params,
        remappings=tf_remaps,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'route_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'docking_server',
            ]},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # RViz-only mode: keep the old visualization bridge.
    cmd_vel_rviz = Node(
        package='robot_3d3s',
        executable='cmd_vel_to_wheels.py',
        name='cmd_vel_to_wheels',
        parameters=[{'gazebo': False}],
        output='screen',
        condition=UnlessCondition(gazebo),
    )

    # Gazebo mode
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
        condition=IfCondition(gazebo),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_3d3s',
            '-string', robot_desc_gz,
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', spawn_roll, '-P', spawn_pitch, '-Y', spawn_yaw,
        ],
        output='screen',
        condition=IfCondition(gazebo),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/robot_3d3s/adhesion_status@std_msgs/msg/String[gz.msgs.StringMsg',
        ],
        remappings=[
            ('/model/robot_3d3s/adhesion_status', '/robot_3d3s/adhesion_status'),
        ],
        output='screen',
        condition=IfCondition(gazebo),
    )

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '30'],
            output='screen',
            condition=IfCondition(gazebo),
        )])

    return LaunchDescription([
        gazebo_arg,
        use_climb_controller_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_roll_arg,
        spawn_pitch_arg,
        spawn_yaw_arg,
        slide_angle_arg,

        # Shared
        robot_state_publisher,
        climb_visual_state_publisher,
        world_to_map,
        map_to_odom,
        odom_node,

        # Nav2 nodes
        controller_server,
        smoother_server,
        planner_server,
        route_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        cmd_vel_stamper_direct,
        cmd_vel_stamper_climb,
        climb_controller,
        climb_rviz_goal_bridge,
        climb_rviz_robot_visualizer,
        docking_server,
        lifecycle_manager,

        rviz,

        # RViz mode
        cmd_vel_rviz,

        # Gazebo mode
        gz_sim,
        spawn_robot,
        gz_bridge,
        spawner('joint_state_broadcaster', delay=8.0),
        spawner('swerve_controller', delay=10.0),
    ])
