import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_3d3s')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_3d3s.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Run teleop in a real terminal so it can capture keyboard input.
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-lc',
                'ros2 run robot_3d3s teleop_keyboard.py; exec bash'
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
