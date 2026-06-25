#!/usr/bin/env python3
"""
flat_wall_climb.launch.py — Stage 1 of the 3D3S climbing digital twin.

ONE source of truth: config/climb_scene.yaml. This launch reads it and:
  * generates the Gazebo world (ground + vertical flat steel wall) to match,
  * injects the SAME wall geometry + adhesion model into the C++
    Kmw100AdhesionSystem plugin (so the adhesion gap is measured against the wall
    Gazebo actually simulates),
  * computes the robot spawn pose from the wall geometry so the three wheels land
    coplanar on the climbed face (no magic Euler angles),
  * publishes RViz markers for the wall AND relays the robot's live Gazebo pose to
    TF, so RViz and Gazebo agree on where everything is,
  * runs a live adhesion validation report (friction + peel margins).

The robot spawns already on the wall so we can validate that adhesion HOLDS and
that you can drive up/down/around without falling, before doing ground->wall
transitions or curved surfaces.

Drive it (separate terminal, after pressing Play / with run_sim:=true):
  ros2 run robot_3d3s teleop_keyboard_swerve.py
"main" body +X = forward = UP the wall.
"""

import math
import os
import re
import tempfile

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

import yaml


# ── scene helpers ────────────────────────────────────────────────────────────

def _wall_face_x_and_normal(wall):
    pose = [float(v) for v in wall['pose']]
    size = [float(v) for v in wall['size']]
    face = str(wall.get('climb_face', '-x'))
    if face == '+x':
        return pose[0] + 0.5 * size[0], 1.0
    return pose[0] - 0.5 * size[0], -1.0


def _slide_geometry(scene, face_x):
    """Return the ramp box pose/size and analytic top-plane endpoints.

    By default the top surface runs from (start_x, start_z) to the wall face.
    For a short transition lip, the scene may instead provide:

      slide.pose: [cx, cy, cz]   # Gazebo model center
      slide.length: L            # box length along the slide

    In both modes we return the real top-surface start/end so the adhesion plugin
    and planner use the same geometry Gazebo renders.
    """
    slide = scene.get('slide')
    if not isinstance(slide, dict):
        return None
    theta = math.radians(float(slide.get('angle_deg', 45.0)))
    w = float(slide.get('width', 1.2))
    t = float(slide.get('thickness', 0.04))
    pose = slide.get('pose')
    explicit_length = slide.get('length')

    if isinstance(pose, list) and len(pose) >= 3 and explicit_length is not None:
        cx = float(pose[0])
        cy = float(pose[1])
        cz = float(pose[2])
        length = float(explicit_length)
        top_cx = cx - 0.5 * t * math.sin(theta)
        top_cz = cz + 0.5 * t * math.cos(theta)
        sx = top_cx - 0.5 * length * math.cos(theta)
        sz = top_cz - 0.5 * length * math.sin(theta)
        end_x = top_cx + 0.5 * length * math.cos(theta)
        top_z = top_cz + 0.5 * length * math.sin(theta)
    else:
        sx = float(slide.get('start_x', -0.15))
        sz = float(slide.get('start_z', 0.0))
        end_x = float(slide.get('wall_x', face_x))
        margin = float(slide.get('length_margin', 0.12))
        top_z = sz + (end_x - sx) * math.tan(theta)
        length = math.hypot(end_x - sx, top_z - sz) + margin
        mid_x = 0.5 * (sx + end_x)
        mid_z = 0.5 * (sz + top_z)
        # Offset the box center below the top surface by t/2 along the surface
        # normal n = (-sin, 0, cos): center = mid_top - n*(t/2).
        cx = mid_x + 0.5 * t * math.sin(theta)
        cy = 0.0
        cz = mid_z - 0.5 * t * math.cos(theta)

    return {
        'angle_deg': float(slide.get('angle_deg', 45.0)),
        'theta': theta, 'start_x': sx, 'start_z': sz,
        'center': (cx, cy, cz), 'pitch': -theta,
        'size': (length, w, t), 'wall_x': end_x, 'top_z': top_z,
        'mu': float(slide.get('mu', 1.0)),
    }


def _slide_model_xml(slide):
    if slide is None:
        return ''
    cx, cy, cz = slide['center']
    sx, sy, sz = slide['size']
    mu = slide['mu']
    pitch = slide['pitch']
    return f"""
    <!-- Transition ramp: top surface runs ground -> wall at {slide['angle_deg']} deg. -->
    <model name="steel_slide">
      <static>true</static>
      <pose>{cx} {cy} {cz} 0 {pitch} 0</pose>
      <link name="slide_link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>0.22 0.03 0.04 1</ambient>
            <diffuse>0.42 0.06 0.07 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _build_world_sdf(scene):
    """Generate the world SDF from the scene YAML (ground + optional slide + wall)."""
    world_name = scene.get('world_name', 'flat_wall')
    ground = scene.get('ground', {})
    gz = float(ground.get('z', 0.0))
    gmu = float(ground.get('mu', 1.0))
    wall = scene['flat_wall']
    wp = [float(v) for v in wall['pose']]
    ws = [float(v) for v in wall['size']]
    wmu = float(wall.get('mu', 1.0))
    face_x, _n = _wall_face_x_and_normal(wall)
    slide_xml = _slide_model_xml(_slide_geometry(scene, face_x))

    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{world_name}">
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
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 {gz} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <surface><friction><ode><mu>{gmu}</mu><mu2>{gmu}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Flat vertical steel wall. Climbed face per climb_scene.yaml. -->
    <model name="flat_steel_wall">
      <static>true</static>
      <pose>{wp[0]} {wp[1]} {wp[2]} 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry><box><size>{ws[0]} {ws[1]} {ws[2]}</size></box></geometry>
          <surface><friction><ode><mu>{wmu}</mu><mu2>{wmu}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{ws[0]} {ws[1]} {ws[2]}</size></box></geometry>
          <material>
            <ambient>0.20 0.02 0.03 1</ambient>
            <diffuse>0.45 0.06 0.07 1</diffuse>
            <specular>0.30 0.10 0.10 1</specular>
          </material>
        </visual>
      </link>
    </model>
{slide_xml}
  </world>
</sdf>
"""


def _build_adhesion_plugin(scene, face_x, wall_thickness, robot_name):
    """Generate the Kmw100AdhesionSystem <plugin> block from the scene YAML."""
    a = scene.get('adhesion', {})
    slide = scene.get('slide', {}) if isinstance(scene.get('slide'), dict) else {}
    slide_geom = _slide_geometry(scene, face_x)
    slide_angle = float(slide.get('angle_deg', 45.0))
    slide_start_x = float(slide_geom['start_x']) if slide_geom else float(slide.get('start_x', -0.15))
    slide_start_z = float(slide_geom['start_z']) if slide_geom else float(slide.get('start_z', 0.0))
    slide_wall_x = float(slide_geom['wall_x']) if slide_geom else face_x

    def g(key, default):
        return a.get(key, default)

    def b(key, default):
        return 'true' if bool(a.get(key, default)) else 'false'

    return (
        f'<plugin filename="librobot_3d3s_kmw100_adhesion_system.so" '
        f'name="robot_3d3s::Kmw100AdhesionSystem">'
        f'<enabled>true</enabled>'
        f'<adhesion_model>{g("model", "gap_decay")}</adhesion_model>'
        f'<force_n>{g("force_n", 900.0)}</force_n>'
        f'<min_force_n>{g("min_force_n", 0.0)}</min_force_n>'
        f'<min_wall_force_n>{g("min_wall_force_n", 350.0)}</min_wall_force_n>'
        f'<gap_reference_mm>{g("gap_reference_mm", 3.0)}</gap_reference_mm>'
        f'<gap_bias_mm>{g("gap_bias_mm", 0.0)}</gap_bias_mm>'
        f'<wheel_radius_m>{scene.get("robot", {}).get("wheel_radius", 0.05)}</wheel_radius_m>'
        f'<max_gap_m>{g("max_gap_m", 0.06)}</max_gap_m>'
        f'<full_contact_depth_m>{g("full_contact_depth_m", 0.002)}</full_contact_depth_m>'
        f'<min_contact_fraction>{g("min_contact_fraction", 0.10)}</min_contact_fraction>'
        f'<tilt_floor>{g("tilt_floor", 0.45)}</tilt_floor>'
        f'<tilt_recovery_90>{g("tilt_recovery_90", 0.55)}</tilt_recovery_90>'
        f'<ground_z>{scene.get("ground", {}).get("z", 0.0)}</ground_z>'
        f'<slide_angle_deg>{slide_angle}</slide_angle_deg>'
        f'<slide_start_x>{slide_start_x}</slide_start_x>'
        f'<slide_start_z>{slide_start_z}</slide_start_z>'
        f'<slide_wall_x>{slide_wall_x}</slide_wall_x>'
        f'<flat_panel_thickness_m>{wall_thickness}</flat_panel_thickness_m>'
        f'<enable_ground>{b("enable_ground", False)}</enable_ground>'
        f'<enable_slide>{b("enable_slide", False)}</enable_slide>'
        f'<enable_opposite_wall>{b("enable_opposite_wall", False)}</enable_opposite_wall>'
        f'<surface_margin_m>0.08</surface_margin_m>'
        f'<debug_period_s>1.0</debug_period_s>'
        f'<status_period_s>0.05</status_period_s>'
        f'<status_topic>/model/{robot_name}/adhesion_status</status_topic>'
        f'</plugin>'
    )


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('robot_3d3s')
    # URDF variant selectable via launch arg (default = rigid). The compliant
    # variant (robot_3d3s_compliant.urdf) adds passive sprung wheel pitch joints.
    urdf_name = LaunchConfiguration('urdf').perform(context) or 'robot_3d3s.urdf'
    urdf_file = os.path.join(pkg, 'urdf', urdf_name)
    controllers_yaml = os.path.join(pkg, 'config', 'swerve_controller.yaml')
    rviz_config = os.path.join(pkg, 'config', 'flat_wall.rviz')
    scene_path = LaunchConfiguration('scene_yaml').perform(context)
    if not scene_path:
        scene_path = os.path.join(pkg, 'config', 'climb_scene.yaml')

    with open(scene_path) as f:
        scene = yaml.safe_load(f)

    world_name = scene.get('world_name', 'flat_wall')
    robot_name = scene.get('robot', {}).get('name', 'robot_3d3s')
    wall = scene['flat_wall']
    wp = [float(v) for v in wall['pose']]
    ws = [float(v) for v in wall['size']]
    face_x, _normal = _wall_face_x_and_normal(wall)

    robot = scene.get('robot', {})
    preload = float(robot.get('preload_m', 0.0))
    start_on = str(robot.get('start_on', 'wall')).lower()

    if start_on == 'ground':
        # Flat on the ground before the ramp, facing +X. base_footprint is the
        # wheel-contact plane, so z = ground height. Drive forward (+X) to reach
        # the ramp, then the slide, then the wall. The robot's body orientation
        # follows the surface, so "forward" naturally becomes "up" on the climb.
        ground_z = float(scene.get('ground', {}).get('z', 0.0))
        spawn_x = float(robot.get('ground_start_x', -1.0))
        spawn_y = wp[1]
        spawn_z = ground_z + float(robot.get('ground_clearance_m', 0.0))
        spawn_roll, spawn_pitch, spawn_yaw = 0.0, 0.0, 0.0
    else:
        # On the climbed face: body +Z -> world -X (off wall), body +X "forward"
        # -> world +Z (up). +preload presses the wheels into the -X face.
        spawn_x = face_x + preload
        spawn_y = wp[1]
        spawn_z = float(robot.get('start_height_z', wp[2]))
        spawn_roll, spawn_pitch, spawn_yaw = 0.0, -math.pi / 2.0, 0.0

    # World SDF from the YAML.
    world_sdf = _build_world_sdf(scene)
    world_path = os.path.join(tempfile.gettempdir(), f'{world_name}.sdf')
    with open(world_path, 'w') as f:
        f.write(world_sdf)

    # URDF: inject controllers path + regenerate adhesion plugin from YAML.
    with open(urdf_file) as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('CONTROLLERS_YAML_PATH', controllers_yaml)
    adhesion_block = _build_adhesion_plugin(scene, face_x, ws[0], robot_name)
    robot_desc = re.sub(
        r'<plugin\s+filename="librobot_3d3s_kmw100_adhesion_system\.so".*?</plugin>',
        lambda _m: adhesion_block,
        robot_desc,
        count=1,
        flags=re.DOTALL,
    )

    # Gazebo plugin + resource paths.
    models_path = os.path.join(pkg, 'models')
    plugins_path = os.path.abspath(os.path.join(pkg, '..', '..', 'lib'))
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(
        p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), models_path] if p)
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(
        p for p in [os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), plugins_path] if p)

    run_sim = LaunchConfiguration('run_sim').perform(context) == 'true'
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    use_rviz = LaunchConfiguration('use_rviz')

    gz_flags = '-v 4'
    if run_sim:
        gz_flags = '-r ' + gz_flags
    if headless:
        gz_flags = '-s ' + gz_flags
    gz_args = f'{gz_flags} {world_path}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-name', robot_name, '-string', robot_desc,
            '-x', f'{spawn_x}', '-y', f'{spawn_y}', '-z', f'{spawn_z}',
            '-R', f'{spawn_roll}', '-P', f'{spawn_pitch}', '-Y', f'{spawn_yaw}',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    pose_info_topic = f'/world/{world_name}/pose/info'
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'{pose_info_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/model/{robot_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/model/{robot_name}/adhesion_status@std_msgs/msg/String[gz.msgs.StringMsg',
            f'/robot_3d3s/wheel_1_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            f'/robot_3d3s/wheel_2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            f'/robot_3d3s/wheel_3_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[
            (f'/model/{robot_name}/adhesion_status', '/robot_3d3s/adhesion_status'),
        ],
    )

    scene_rviz = Node(
        package='robot_3d3s', executable='climb_scene_rviz.py',
        name='climb_scene_rviz', output='screen',
        parameters=[{
            'use_sim_time': True,
            'scene_yaml': scene_path,
            'world_frame': 'world',
            'robot_name': robot_name,
            'base_frame': 'base_footprint',
            'status_topic': '/robot_3d3s/adhesion_status',
        }],
    )

    adhesion_report = Node(
        package='robot_3d3s', executable='adhesion_report.py',
        name='adhesion_report', output='screen',
        parameters=[{'use_sim_time': True, 'scene_yaml': scene_path}],
    )

    # Autonomous "click a point -> climb to it" controller. Idle until a goal is
    # published with RViz "Publish Point", so teleop still works without a goal.
    goal_controller = Node(
        package='robot_3d3s', executable='climb_goal_controller.py',
        name='climb_goal_controller', output='screen',
        condition=IfCondition(LaunchConfiguration('start_goal_controller')),
        parameters=[{
            'use_sim_time': True,
            'scene_yaml': scene_path,
            'status_topic': '/robot_3d3s/adhesion_status',
            'clicked_point_topic': '/clicked_point',
            'cmd_out_topic': '/swerve_controller/cmd_vel',
            'base_frame': 'base_link',
            'v_climb': ParameterValue(LaunchConfiguration('v_climb'), value_type=float),
            'v_lat': ParameterValue(LaunchConfiguration('v_lat'), value_type=float),
            'accel_limit_mps2': ParameterValue(
                LaunchConfiguration('accel_limit_mps2'), value_type=float),
            'yaw_accel_limit_radps2': ParameterValue(
                LaunchConfiguration('yaw_accel_limit_radps2'), value_type=float),
            'transition_slow_band_m': ParameterValue(
                LaunchConfiguration('transition_slow_band_m'), value_type=float),
            'transition_speed_scale': ParameterValue(
                LaunchConfiguration('transition_speed_scale'), value_type=float),
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config], output='screen',
        parameters=[{'use_sim_time': True}],
    )

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager-timeout', '60'],
            output='screen')])

    step_startup = TimerAction(
        period=9.0,
        condition=IfCondition(LaunchConfiguration('auto_step_startup')),
        actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', f'/world/{world_name}/control',
                 '--reqtype', 'gz.msgs.WorldControl',
                 '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                 '--req', 'multi_step: 80'],
            output='screen')],
    )

    hint = TimerAction(period=12.0, actions=[LogInfo(msg=[
        '\n================ FLAT WALL CLIMB READY ================\n',
        f'World: {world_path}\n',
        f'Wall climbed face at x={face_x:.3f}, robot spawned at '
        f'({spawn_x:.3f}, {spawn_y:.3f}, {spawn_z:.3f}) pitch -90deg.\n',
        'If started paused, press Play. Then drive (separate terminal):\n',
        '  ros2 run robot_3d3s teleop_keyboard_swerve.py\n',
        'Forward (body +X) climbs UP. Watch /robot_3d3s/adhesion_report for '
        'friction/peel margins.\n',
        '======================================================\n',
    ])])

    return [
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        scene_rviz,
        adhesion_report,
        goal_controller,
        rviz,
        spawner('joint_state_broadcaster', 8.0),
        spawner('swerve_controller', 10.0),
        step_startup,
        hint,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene_yaml', default_value='',
                              description='Path to climb_scene.yaml (defaults to package config).'),
        DeclareLaunchArgument('run_sim', default_value='true',
                              description='Start Gazebo running (true) or paused (false).'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz server only (no GUI), for smoke tests.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 with the flat_wall config.'),
        DeclareLaunchArgument('auto_step_startup', default_value='false',
                              description='Step paused Gazebo briefly so controllers activate.'),
        DeclareLaunchArgument('start_goal_controller', default_value='true',
                              description='Start the click-a-point climb-to-goal controller.'),
        DeclareLaunchArgument('v_climb', default_value='0.18',
                              description='Autonomous climb speed along the surface, m/s.'),
        DeclareLaunchArgument('v_lat', default_value='0.08',
                              description='Autonomous lateral speed on the surface, m/s.'),
        DeclareLaunchArgument('accel_limit_mps2', default_value='0.12',
                              description='Autonomous body acceleration limit for smooth climbing.'),
        DeclareLaunchArgument('yaw_accel_limit_radps2', default_value='0.45',
                              description='Autonomous yaw acceleration limit for smooth climbing.'),
        DeclareLaunchArgument('transition_slow_band_m', default_value='0.28',
                              description='Distance around ramp/corner transitions where speed is reduced.'),
        DeclareLaunchArgument('transition_speed_scale', default_value='0.45',
                              description='Minimum speed scale near geometric transitions.'),
        DeclareLaunchArgument('urdf', default_value='robot_3d3s.urdf',
                              description='URDF filename in the package urdf/ dir. Use '
                                          'robot_3d3s_compliant.urdf for sprung wheel pitch.'),
        OpaqueFunction(function=launch_setup),
    ])
