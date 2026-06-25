#!/usr/bin/env python3
"""
ground_to_wall_climb.launch.py — full course: GROUND -> SLIDE -> FLAT WALL.

Thin wrapper around the generalized climb engine (flat_wall_climb.launch.py),
selecting config/climb_course.yaml. The robot spawns on the floor, you drive it
forward up the transition ramp and onto the vertical wall. Edit the geometry
(especially slide.angle_deg) in climb_course.yaml.

Drive (separate terminal, after Play / with run_sim:=true):
  ros2 run robot_3d3s teleop_keyboard_swerve.py
Forward (body +X) drives toward the ramp, then up the slide, then up the wall.
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
        DeclareLaunchArgument('v_climb', default_value='0.18'),
        DeclareLaunchArgument('v_lat', default_value='0.08'),
        DeclareLaunchArgument('accel_limit_mps2', default_value='0.12'),
        DeclareLaunchArgument('yaw_accel_limit_radps2', default_value='0.45'),
        DeclareLaunchArgument('transition_slow_band_m', default_value='0.28'),
        DeclareLaunchArgument('transition_speed_scale', default_value='0.45'),
        DeclareLaunchArgument('scene_yaml', default_value=course_yaml,
                              description='Course scene (ground+slide+wall).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(engine),
            launch_arguments={
                'scene_yaml': LaunchConfiguration('scene_yaml'),
                'run_sim': LaunchConfiguration('run_sim'),
                'headless': LaunchConfiguration('headless'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'auto_step_startup': LaunchConfiguration('auto_step_startup'),
                'v_climb': LaunchConfiguration('v_climb'),
                'v_lat': LaunchConfiguration('v_lat'),
                'accel_limit_mps2': LaunchConfiguration('accel_limit_mps2'),
                'yaw_accel_limit_radps2': LaunchConfiguration('yaw_accel_limit_radps2'),
                'transition_slow_band_m': LaunchConfiguration('transition_slow_band_m'),
                'transition_speed_scale': LaunchConfiguration('transition_speed_scale'),
            }.items(),
        ),
    ])
