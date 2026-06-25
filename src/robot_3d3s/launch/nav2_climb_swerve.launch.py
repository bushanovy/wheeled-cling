#!/usr/bin/env python3
"""Nav2 climbing launch for the 3D3S Gazebo slide-to-wall world.

RViz Nav2 goals are interpreted as 2D surface-progress goals:
  x = forward distance along the approach/slide/wall path
  y = lateral offset across the surface
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('robot_3d3s')

    return LaunchDescription([
        LogInfo(msg=(
            'Starting Nav2 climbing mode. In RViz, use Nav2 Goal: '
            'goal X is surface progress, goal Y is lateral motion.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav2_swerve.launch.py')
            ),
            launch_arguments={
                'gazebo': 'true',
                'use_climb_controller': 'true',
                'spawn_x': '-0.9806',
                'spawn_y': '0.0',
                'spawn_z': '0.0',
                'spawn_roll': '0.0',
                'spawn_pitch': '0.0',
                'spawn_yaw': '0.0',
                'slide_angle_deg': '45.0',
            }.items(),
        ),
    ])
