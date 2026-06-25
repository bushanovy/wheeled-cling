#!/usr/bin/env python3
"""Projected RViz robot visualization for the 3D3S climbing demo.

Nav2 keeps using flat surface-progress odometry. This node projects that 2D
progress onto the Gazebo ground / slide / wall geometry and publishes a
separate prefixed robot TF tree for RViz visualization.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _normalize(v):
    length = math.sqrt(sum(c * c for c in v))
    if length <= 1e-12:
        return (0.0, 0.0, 0.0)
    return tuple(c / length for c in v)


def _quat_from_matrix(m):
    """Quaternion from a 3x3 rotation matrix whose columns are body XYZ axes."""
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
            0.25 * s,
        )
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (
            0.25 * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
            (m21 - m12) / s,
        )
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return (
            (m01 + m10) / s,
            0.25 * s,
            (m12 + m21) / s,
            (m02 - m20) / s,
        )

    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return (
        (m02 + m20) / s,
        (m12 + m21) / s,
        0.25 * s,
        (m10 - m01) / s,
    )


def _quat_from_surface(forward, lateral, normal, yaw: float):
    """Rotate a surface-aligned body by yaw about the surface normal."""
    forward = _normalize(forward)
    lateral = _normalize(lateral)
    normal = _normalize(normal)

    cy = math.cos(yaw)
    sy = math.sin(yaw)
    body_x = tuple(cy * forward[i] + sy * lateral[i] for i in range(3))
    body_y = tuple(-sy * forward[i] + cy * lateral[i] for i in range(3))
    body_z = normal

    return _quat_from_matrix((
        (body_x[0], body_y[0], body_z[0]),
        (body_x[1], body_y[1], body_z[1]),
        (body_x[2], body_y[2], body_z[2]),
    ))


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ClimbRvizRobotVisualizer(Node):
    def __init__(self):
        super().__init__('climb_rviz_robot_visualizer')

        self.declare_parameter('visual_prefix', 'climb_visual_')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('robot_description_topic', '/robot_description_visual')
        self.declare_parameter('joint_states_in', '/joint_states')
        self.declare_parameter('joint_states_out', '/joint_states_visual')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('spawn_x', -0.9806)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('slide_start_x', -0.15)
        self.declare_parameter('slide_start_z', 0.0)
        self.declare_parameter('slide_wall_x', 0.95)
        self.declare_parameter('slide_angle_deg', 45.0)
        self.declare_parameter('panel_center_z', 1.5)
        self.declare_parameter('panel_height', 3.0)
        self.declare_parameter('publish_hz', 50.0)

        self.prefix = self.get_parameter('visual_prefix').value
        self.robot_description = self.get_parameter('robot_description').value
        description_topic = self.get_parameter('robot_description_topic').value
        joint_in = self.get_parameter('joint_states_in').value
        joint_out = self.get_parameter('joint_states_out').value
        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.slide_start_x = float(self.get_parameter('slide_start_x').value)
        self.slide_start_z = float(self.get_parameter('slide_start_z').value)
        self.slide_wall_x = float(self.get_parameter('slide_wall_x').value)
        self.slide_angle = math.radians(float(self.get_parameter('slide_angle_deg').value))
        self.panel_center_z = float(self.get_parameter('panel_center_z').value)
        self.panel_height = float(self.get_parameter('panel_height').value)
        publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self.ground_progress = max(0.0, self.slide_start_x - self.spawn_x)
        self.slide_progress = (
            max(0.0, self.slide_wall_x - self.slide_start_x)
            / max(0.05, math.cos(self.slide_angle))
        )
        self.wall_entry_z = (
            self.slide_start_z
            + (self.slide_wall_x - self.slide_start_x) * math.tan(self.slide_angle)
        )
        self.wall_entry_progress = self.ground_progress + self.slide_progress
        self.panel_top = self.panel_center_z + self.panel_height * 0.5

        description_qos = QoSProfile(depth=1)
        description_qos.reliability = ReliabilityPolicy.RELIABLE
        description_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.description_pub = self.create_publisher(
            String, description_topic, description_qos)
        self.joint_pub = self.create_publisher(JointState, joint_out, 10)
        self.tf_br = TransformBroadcaster(self)

        self.last_odom = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(JointState, joint_in, self._joint_cb, 10)
        self.create_timer(1.0 / publish_hz, self._timer_cb)
        self._publish_description()

        self.get_logger().info(
            f'RViz climbing robot visualizer ready: {self.parent_frame} -> '
            f'{self.prefix}base_footprint')

    def _odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def _joint_cb(self, msg: JointState):
        visual = JointState()
        visual.header = msg.header
        visual.name = [
            name if name.startswith(self.prefix) else self.prefix + name
            for name in msg.name
        ]
        visual.position = list(msg.position)
        visual.velocity = list(msg.velocity)
        visual.effort = list(msg.effort)
        self.joint_pub.publish(visual)

    def _timer_cb(self):
        self._publish_description()
        if self.last_odom is None:
            return

        stamp = self.get_clock().now().to_msg()
        progress = self.last_odom.pose.pose.position.x
        lateral = self.last_odom.pose.pose.position.y
        yaw = _yaw_from_quat(self.last_odom.pose.pose.orientation)
        x, y, z, forward, lateral_axis, normal = self._project(progress, lateral)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.prefix + 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        qx, qy, qz, qw = _quat_from_surface(
            forward, lateral_axis, normal, yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    def _project(self, progress: float, lateral: float):
        progress = max(0.0, progress)

        if progress <= self.ground_progress:
            return (
                progress, lateral, 0.0,
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
            )

        if progress <= self.wall_entry_progress:
            slide_s = progress - self.ground_progress
            x = (self.slide_start_x - self.spawn_x) + slide_s * math.cos(self.slide_angle)
            z = (self.slide_start_z - self.spawn_z) + slide_s * math.sin(self.slide_angle)
            return (
                x, lateral, z,
                (math.cos(self.slide_angle), 0.0, math.sin(self.slide_angle)),
                (0.0, 1.0, 0.0),
                (-math.sin(self.slide_angle), 0.0, math.cos(self.slide_angle)),
            )

        wall_s = progress - self.wall_entry_progress
        x = self.slide_wall_x - self.spawn_x
        z = min(self.panel_top - self.spawn_z, self.wall_entry_z - self.spawn_z + wall_s)
        return (
            x, lateral, z,
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (-1.0, 0.0, 0.0),
        )

    def _publish_description(self):
        if not self.robot_description:
            return
        msg = String()
        msg.data = self.robot_description
        self.description_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClimbRvizRobotVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
