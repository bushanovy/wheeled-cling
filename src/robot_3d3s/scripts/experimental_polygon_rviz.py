#!/usr/bin/env python3
"""RViz markers for the experimental polygon pipe scene."""

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


class ExperimentalPolygonRviz(Node):
    def __init__(self):
        super().__init__('experimental_polygon_rviz')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('marker_topic', '/experimental_polygon_markers')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(
            f'experimental_polygon_rviz publishing {self.get_parameter("marker_topic").value}')

    @property
    def frame(self):
        return str(self.get_parameter('world_frame').value)

    def _cylinder(self, stamp, marker_id, name, x, y, z, radius, length, axis):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.ns = 'experimental_polygon'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        roll, pitch, yaw = 0.0, 0.0, 0.0
        if axis == 'x':
            pitch = math.pi / 2.0
        elif axis == 'y':
            roll = math.pi / 2.0
        qx, qy, qz, qw = _quat_from_rpy(roll, pitch, yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = length
        marker.color.r = 0.34
        marker.color.g = 0.025
        marker.color.b = 0.030
        marker.color.a = 0.82
        marker.text = name
        return marker

    def _delete_marker(self, stamp, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(
            self._cylinder(stamp, 1, 'left vertical R457.2 H3000',
                           -1.5656, 0.0, 1.5, 0.4572, 3.0, 'z'))
        markers.markers.append(
            self._cylinder(stamp, 2, 'right vertical R254 H3000',
                           1.5656, 0.0, 1.5, 0.2540, 3.0, 'z'))
        markers.markers.append(
            self._cylinder(stamp, 3, 'horizontal R254 L3131.2 Z1246',
                           0.0, 0.0, 1.246, 0.2540, 3.1312, 'x'))
        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentalPolygonRviz()
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
