#!/usr/bin/env python3
"""
Ground -> 45 degree steel slide -> wall adhesion test using swerve control.

Starts Gazebo paused, spawns the robot on the ground before the slide, enables
the C++ KMW100 adhesion plugin, then waits for the operator to press Play and
drive. Direct teleop is the default for this regression test so forward,
backward, lateral, and rotation commands reach the swerve controller without
surface-state remapping delays.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')
    climb_launch = os.path.join(pkg, 'launch', 'climb_test_swerve.launch.py')
    use_climb_controller = LaunchConfiguration('use_climb_controller')

    slide_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(climb_launch),
        launch_arguments={
            'run_sim': 'false',
            'start_adhesion': 'false',
            'start_controllers': 'true',
            'use_climb_controller': use_climb_controller,
            'auto_step_startup': 'true',
            'adhesion_mode': 'slide',
            'adhesion_model': 'lookup',
            'adhesion_force': '900.0',
            'min_force': '350.0',
            'gap_reference_mm': '3.0',
            'safe_tilt_deg': '20.0',
            'cutoff_tilt_deg': '60.0',
            'max_lookup_gap_mm': '1.0',
            'gap_bias_mm': '0.0',
            'wall_thickness_mm': '3.0',
            'wheel_radius_m': '0.05',
            'spawn_x': '-0.9806',
            'spawn_y': '0.0',
            'spawn_z': '0.0',
            'spawn_roll': '0.0',
            'spawn_pitch': '0.0',
            'spawn_yaw': '0.0',
            'normal_x': '0.0',
            'normal_y': '0.0',
            'normal_z': '1.0',
            'ground_z': '0.0',
            'slide_angle_deg': '45.0',
            'slide_start_x': '-0.15',
            'slide_start_z': '0.0',
            'slide_wall_x': '0.95',
            'surface_margin_m': '0.08',
            'surface_hysteresis_m': '0.03',
            'plane_point_x': '0.95',
            'plane_point_y': '0.0',
            'plane_point_z': '0.0',
        }.items(),
    )

    ready_hint = TimerAction(
        period=15.0,
        actions=[LogInfo(msg=[
            '\nSLIDE TEST: Gazebo should still be paused. The C++ Gazebo ',
            'Kmw100AdhesionSystem now applies adhesion every physics step; ',
            'the old Python wrench node is disabled for this launch. Check ',
            'Gazebo logs for Kmw100AdhesionSystem ground/slide/wall messages, ',
            'then run direct teleop as:\n'
            '  ros2 run robot_3d3s teleop_keyboard_swerve.py\n'
            'This default path publishes directly to /swerve_controller/cmd_vel, ',
            'matching the previous working slide test. For the experimental ',
            'surface-aware controller, launch with use_climb_controller:=true ',
            'and publish teleop to /climb_controller/cmd_vel_in.\n'
            'Press Play and drive forward toward +X with teleop.\n',
        ])],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_climb_controller',
            default_value='false',
            description=(
                'When true, teleop should publish to /climb_controller/cmd_vel_in. '
                'Default false restores direct teleop to /swerve_controller/cmd_vel '
                'for slide adhesion debugging.'
            ),
        ),
        slide_test,
        ready_hint,
    ])
