#!/usr/bin/env python3
"""
ground_to_wall_climb_compliant.launch.py — ground -> slide -> wall course, run
with the PASSIVE-COMPLIANT robot (sprung wheel pitch) instead of the rigid one.

Identical to ground_to_wall_climb.launch.py except it selects
urdf:=robot_3d3s_compliant.urdf, which adds a passive sprung pitch joint per wheel
so each wheel can tilt to seat flat through the slide->wall concave corner. The
rigid baseline (ground_to_wall_climb.launch.py) is untouched — run both and
compare adhesion_report ΣN / attached-wheel count through the corner.

Drive / goal (separate terminal, after Play):
  ros2 run robot_3d3s teleop_keyboard_swerve.py        # manual
  or click "Publish Point" in RViz on the ramp/wall    # autonomous
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')
    engine = os.path.join(pkg, 'launch', 'flat_wall_climb.launch.py')
    course_yaml = os.path.join(pkg, 'config', 'climb_course.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('run_sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_step_startup', default_value='false'),
        DeclareLaunchArgument('start_goal_controller', default_value='true'),
        DeclareLaunchArgument('v_climb', default_value='0.18'),
        DeclareLaunchArgument('v_lat', default_value='0.08'),
        DeclareLaunchArgument('accel_limit_mps2', default_value='0.12'),
        DeclareLaunchArgument('yaw_accel_limit_radps2', default_value='0.45'),
        DeclareLaunchArgument('transition_slow_band_m', default_value='0.28'),
        DeclareLaunchArgument('transition_speed_scale', default_value='0.45'),
        DeclareLaunchArgument('scene_yaml', default_value=course_yaml,
                              description='Course scene (ground+slide+wall).'),
        DeclareLaunchArgument('urdf', default_value='robot_3d3s_compliant.urdf',
                              description='Passive-compliant variant (sprung wheel pitch).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(engine),
            launch_arguments={
                'scene_yaml': LaunchConfiguration('scene_yaml'),
                'run_sim': LaunchConfiguration('run_sim'),
                'headless': LaunchConfiguration('headless'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'auto_step_startup': LaunchConfiguration('auto_step_startup'),
                'start_goal_controller': LaunchConfiguration('start_goal_controller'),
                'v_climb': LaunchConfiguration('v_climb'),
                'v_lat': LaunchConfiguration('v_lat'),
                'accel_limit_mps2': LaunchConfiguration('accel_limit_mps2'),
                'yaw_accel_limit_radps2': LaunchConfiguration('yaw_accel_limit_radps2'),
                'transition_slow_band_m': LaunchConfiguration('transition_slow_band_m'),
                'transition_speed_scale': LaunchConfiguration('transition_speed_scale'),
                'urdf': LaunchConfiguration('urdf'),
            }.items(),
        ),
    ])
