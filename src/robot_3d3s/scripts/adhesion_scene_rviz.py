#!/usr/bin/env python3
"""RViz markers for the centered vertical pipe test scene."""

import math

import rclpy
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


class AdhesionSceneRviz(Node):
    def __init__(self):
        super().__init__('adhesion_scene_rviz')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('marker_topic', '/adhesion_model_markers')
        self.declare_parameter('vertical_pipe_radius_m', 1.20)
        self.declare_parameter('vertical_pipe_length_m', 4.0)
        self.declare_parameter('vertical_pipe_x', 0.0)
        self.declare_parameter('vertical_pipe_y', 0.0)
        self.declare_parameter('vertical_pipe_z', 2.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(
            f'adhesion_scene_rviz publishing {self.get_parameter("marker_topic").value}')

    def _delete_marker(self, stamp, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = self.get_parameter('world_frame').value
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _vertical_pipe(self, stamp):
        radius = float(self.get_parameter('vertical_pipe_radius_m').value)
        length = float(self.get_parameter('vertical_pipe_length_m').value)
        marker = Marker()
        marker.header.frame_id = self.get_parameter('world_frame').value
        marker.header.stamp = stamp
        marker.ns = 'steel_vertical_pipe'
        marker.id = 20
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.get_parameter('vertical_pipe_x').value)
        marker.pose.position.y = float(self.get_parameter('vertical_pipe_y').value)
        marker.pose.position.z = float(self.get_parameter('vertical_pipe_z').value)
        qx, qy, qz, qw = _quat_from_rpy(0.0, 0.0, 0.0)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = length
        marker.color.r = 0.24
        marker.color.g = 0.24
        marker.color.b = 0.82
        marker.color.a = 0.55
        return marker

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for ns, marker_id in (
            ('steel_flat_panel', 1),
            ('steel_internal_corner', 2),
            ('steel_internal_corner', 3),
            ('steel_point_contact_obstacle', 4),
        ):
            markers.markers.append(self._delete_marker(stamp, ns, marker_id))
        markers.markers.append(self._vertical_pipe(stamp))
        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AdhesionSceneRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
