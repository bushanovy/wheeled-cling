#!/usr/bin/env python3
"""RViz wall-click helper for the 3D3S climbing Nav2 demo.

Publishes RViz markers for the Gazebo slide / wall geometry and converts
/clicked_point selections on those markers into Nav2 /goal_pose commands.
"""

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ClimbRvizGoalBridge(Node):
    def __init__(self):
        super().__init__('climb_rviz_goal_bridge')

        self.declare_parameter('marker_frame', 'odom')
        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/climb_surface_markers')
        self.declare_parameter('spawn_x', -0.9806)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('slide_start_x', -0.15)
        self.declare_parameter('slide_start_z', 0.0)
        self.declare_parameter('slide_wall_x', 0.95)
        self.declare_parameter('slide_angle_deg', 45.0)
        self.declare_parameter('slide_length', 1.60)
        self.declare_parameter('slide_width', 1.20)
        self.declare_parameter('slide_thickness', 0.04)
        self.declare_parameter('panel_center_x', 1.0)
        self.declare_parameter('panel_center_z', 1.5)
        self.declare_parameter('panel_thickness', 0.10)
        self.declare_parameter('panel_width', 3.0)
        self.declare_parameter('panel_height', 3.0)
        self.declare_parameter('wall_pick_tolerance_m', 0.35)
        self.declare_parameter('marker_period_s', 0.5)

        self.marker_frame = self.get_parameter('marker_frame').value
        self.goal_frame = self.get_parameter('goal_frame').value
        clicked_topic = self.get_parameter('clicked_point_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.slide_start_x = float(self.get_parameter('slide_start_x').value)
        self.slide_start_z = float(self.get_parameter('slide_start_z').value)
        self.slide_wall_x = float(self.get_parameter('slide_wall_x').value)
        self.slide_angle = math.radians(float(self.get_parameter('slide_angle_deg').value))
        self.slide_length = float(self.get_parameter('slide_length').value)
        self.slide_width = float(self.get_parameter('slide_width').value)
        self.slide_thickness = float(self.get_parameter('slide_thickness').value)
        self.panel_center_x = float(self.get_parameter('panel_center_x').value)
        self.panel_center_z = float(self.get_parameter('panel_center_z').value)
        self.panel_thickness = float(self.get_parameter('panel_thickness').value)
        self.panel_width = float(self.get_parameter('panel_width').value)
        self.panel_height = float(self.get_parameter('panel_height').value)
        self.wall_pick_tolerance = float(self.get_parameter('wall_pick_tolerance_m').value)
        marker_period = max(0.1, float(self.get_parameter('marker_period_s').value))

        self.wall_entry_z = (
            self.slide_start_z
            + (self.slide_wall_x - self.slide_start_x) * math.tan(self.slide_angle)
        )
        self.ground_progress = max(0.0, self.slide_start_x - self.spawn_x)
        self.slide_progress = (
            max(0.0, self.slide_wall_x - self.slide_start_x)
            / max(0.05, math.cos(self.slide_angle))
        )
        self.wall_entry_progress = self.ground_progress + self.slide_progress
        self.selected_point: Optional[Tuple[float, float, float]] = None

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, marker_qos)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.create_subscription(PointStamped, clicked_topic, self._clicked_cb, 10)
        self.create_timer(marker_period, self._publish_markers)

        self.get_logger().info(
            f'RViz climb goal bridge ready: click {clicked_topic}, publish {goal_topic}')

    def _clicked_cb(self, msg: PointStamped):
        if msg.header.frame_id and msg.header.frame_id != self.marker_frame:
            self.get_logger().warn(
                f'Clicked point is in frame {msg.header.frame_id}, expected '
                f'{self.marker_frame}. Set RViz Fixed Frame to {self.marker_frame}.',
                throttle_duration_sec=2.0)
            return

        world_x = msg.point.x + self.spawn_x
        world_y = msg.point.y + self.spawn_y
        world_z = msg.point.z + self.spawn_z
        goal_x, goal_y, surface = self._surface_goal(world_x, world_y, world_z)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.goal_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        self.selected_point = (msg.point.x, msg.point.y, msg.point.z)
        self._publish_markers()
        self.get_logger().info(
            f'Clicked {surface}: world=({world_x:.2f}, {world_y:.2f}, {world_z:.2f}) '
            f'-> Nav2 goal=({goal_x:.2f}, {goal_y:.2f})')

    def _surface_goal(self, world_x: float, world_y: float, world_z: float):
        wall_face_x = self.panel_center_x - self.panel_thickness * 0.5
        panel_top = self.panel_center_z + self.panel_height * 0.5
        panel_bottom = self.panel_center_z - self.panel_height * 0.5
        on_wall_height = world_z >= self.wall_entry_z - 0.15
        near_wall = abs(world_x - wall_face_x) <= self.wall_pick_tolerance

        if near_wall and on_wall_height:
            wall_z = _clamp(world_z, self.wall_entry_z, panel_top)
            return (
                self.wall_entry_progress + (wall_z - self.wall_entry_z),
                _clamp(world_y, -self.panel_width * 0.5, self.panel_width * 0.5),
                'wall',
            )

        slide_dir_x = math.cos(self.slide_angle)
        slide_dir_z = math.sin(self.slide_angle)
        dx = world_x - self.slide_start_x
        dz = world_z - self.slide_start_z
        slide_s = _clamp(dx * slide_dir_x + dz * slide_dir_z, 0.0, self.slide_progress)
        slide_x_at_s = self.slide_start_x + slide_s * slide_dir_x
        slide_z_at_s = self.slide_start_z + slide_s * slide_dir_z
        slide_distance = math.hypot(world_x - slide_x_at_s, world_z - slide_z_at_s)
        if slide_distance <= self.wall_pick_tolerance and world_z >= panel_bottom:
            return (
                self.ground_progress + slide_s,
                _clamp(world_y, -self.slide_width * 0.5, self.slide_width * 0.5),
                'slide',
            )

        return (
            max(0.0, world_x - self.spawn_x),
            world_y,
            'ground',
        )

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.extend([
            self._cube(
                now, 1, 'ground',
                x=0.45, y=0.0, z=-0.015,
                sx=1.8, sy=2.4, sz=0.03,
                rgba=(0.46, 0.46, 0.46, 0.35)),
            self._cube(
                now, 2, 'steel_slide',
                x=0.430 - self.spawn_x, y=0.0, z=0.552 - self.spawn_z,
                sx=self.slide_length, sy=self.slide_width, sz=self.slide_thickness,
                pitch=-self.slide_angle,
                rgba=(0.70, 0.08, 0.08, 0.78)),
            self._cube(
                now, 3, 'flat_steel_panel',
                x=self.panel_center_x - self.spawn_x, y=0.0,
                z=self.panel_center_z - self.spawn_z,
                sx=self.panel_thickness, sy=self.panel_width, sz=self.panel_height,
                rgba=(0.63, 0.07, 0.08, 0.55)),
        ])

        if self.selected_point is not None:
            markers.markers.append(self._route_marker(now))
            markers.markers.append(self._selected_marker(now))

        self.marker_pub.publish(markers)

    def _cube(self, stamp, marker_id, name, x, y, z, sx, sy, sz,
              rgba, roll=0.0, pitch=0.0, yaw=0.0):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = stamp
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        qx, qy, qz, qw = _quat_from_rpy(roll, pitch, yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        return marker

    def _route_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = stamp
        marker.ns = 'climb_route'
        marker.id = 4
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 0.1
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        for x, y, z in self._route_points_to_selected():
            marker.points.append(Point(x=x, y=y, z=z))
        return marker

    def _route_points_to_selected(self):
        if self.selected_point is None:
            return []

        sx, sy, sz = self.selected_point
        world_x = sx + self.spawn_x
        world_y = sy + self.spawn_y
        world_z = sz + self.spawn_z
        _, _, surface = self._surface_goal(world_x, world_y, world_z)
        route_z = 0.03
        panel_top = self.panel_center_z + self.panel_height * 0.5
        wall_face_x = self.panel_center_x - self.panel_thickness * 0.5
        goal_y = _clamp(world_y, -self.panel_width * 0.5, self.panel_width * 0.5)

        points = [(0.0, 0.0, route_z)]
        if surface == 'ground':
            points.append((sx, sy, route_z))
            return points

        slide_start = (
            self.slide_start_x - self.spawn_x,
            0.0,
            self.slide_start_z - self.spawn_z + route_z)
        points.append(slide_start)

        if surface == 'slide':
            slide_dir_x = math.cos(self.slide_angle)
            slide_dir_z = math.sin(self.slide_angle)
            dx = world_x - self.slide_start_x
            dz = world_z - self.slide_start_z
            slide_s = _clamp(
                dx * slide_dir_x + dz * slide_dir_z,
                0.0,
                self.slide_progress)
            points.append((
                self.slide_start_x + slide_s * slide_dir_x - self.spawn_x,
                _clamp(world_y, -self.slide_width * 0.5, self.slide_width * 0.5) - self.spawn_y,
                self.slide_start_z + slide_s * slide_dir_z - self.spawn_z + route_z))
            return points

        points.append((
            self.slide_wall_x - self.spawn_x,
            0.0,
            self.wall_entry_z - self.spawn_z + route_z))
        wall_z = _clamp(world_z, self.wall_entry_z, panel_top)
        points.append((
            wall_face_x - self.spawn_x,
            goal_y - self.spawn_y,
            wall_z - self.spawn_z))
        return points

    def _selected_marker(self, stamp):
        x, y, z = self.selected_point
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = stamp
        marker.ns = 'selected_wall_goal'
        marker.id = 5
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ClimbRvizGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
