#!/usr/bin/env python3
"""Relay a Gazebo model pose topic into the robot TF tree.

ros_gz_bridge exposes Gazebo Pose_V topics as TFMessage topics, but they are
normal ROS topics, not automatically part of /tf.  RViz and the mockup planner
need a real world -> base_footprint transform, so this node republishes the
bridged model pose as TF.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import math


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
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


class GazeboModelTfRelay(Node):
    def __init__(self):
        super().__init__('gazebo_model_tf_relay')

        self.declare_parameter('pose_topic', '/model/robot_3d3s/pose')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('model_name', 'robot_3d3s')
        self.declare_parameter('fallback_x', 0.0)
        self.declare_parameter('fallback_y', 0.0)
        self.declare_parameter('fallback_z', 0.0)
        self.declare_parameter('fallback_roll', 0.0)
        self.declare_parameter('fallback_pitch', 0.0)
        self.declare_parameter('fallback_yaw', 0.0)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('fallback_transform_index', -1)

        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.model_name = str(self.get_parameter('model_name').value)
        self.fallback_x = float(self.get_parameter('fallback_x').value)
        self.fallback_y = float(self.get_parameter('fallback_y').value)
        self.fallback_z = float(self.get_parameter('fallback_z').value)
        self.fallback_roll = float(self.get_parameter('fallback_roll').value)
        self.fallback_pitch = float(self.get_parameter('fallback_pitch').value)
        self.fallback_yaw = float(self.get_parameter('fallback_yaw').value)
        self.fallback_transform_index = int(
            self.get_parameter('fallback_transform_index').value)
        publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))
        pose_topic = str(self.get_parameter('pose_topic').value)

        self.tf_br = TransformBroadcaster(self)
        self._logged_source = False
        self._last_live_tf = None
        self._using_fallback_logged = False
        self.create_subscription(TFMessage, pose_topic, self._pose_cb, 10)
        self.create_timer(1.0 / publish_hz, self._timer_cb)
        self.get_logger().info(
            f'gazebo_model_tf_relay ready: {pose_topic} -> '
            f'{self.parent_frame}->{self.child_frame}')

    def _pose_cb(self, msg: TFMessage):
        if not msg.transforms:
            return

        source = self._select_transform(msg)
        if source is None:
            return

        self._last_live_tf = TransformStamped()
        self._last_live_tf.header.frame_id = self.parent_frame
        self._last_live_tf.child_frame_id = self.child_frame
        self._last_live_tf.transform.translation = source.transform.translation
        self._last_live_tf.transform.rotation = source.transform.rotation

        if not self._logged_source:
            self._logged_source = True
            self.get_logger().info(
                'Relaying Gazebo transform '
                f'{source.header.frame_id}->{source.child_frame_id} as '
                f'{self.parent_frame}->{self.child_frame}')

    def _timer_cb(self):
        if self._last_live_tf is not None:
            out = self._last_live_tf
            out.header.stamp = self.get_clock().now().to_msg()
            self.tf_br.sendTransform(out)
            return

        qx, qy, qz, qw = _rpy_to_quat(
            self.fallback_roll, self.fallback_pitch, self.fallback_yaw)
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.transform.translation.x = self.fallback_x
        out.transform.translation.y = self.fallback_y
        out.transform.translation.z = self.fallback_z
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        self.tf_br.sendTransform(out)

        if not self._using_fallback_logged:
            self._using_fallback_logged = True
            self.get_logger().warn(
                'Using spawn-pose TF fallback until live Gazebo model pose arrives.')

    def _select_transform(self, msg: TFMessage):
        candidates = []
        for tf in msg.transforms:
            child = tf.child_frame_id or ''
            parent = tf.header.frame_id or ''
            score = 0
            if child == self.model_name or child.endswith('/' + self.model_name):
                score += 3
            if self.model_name in child:
                score += 2
            if parent == self.parent_frame:
                score += 1
            candidates.append((score, tf))

        candidates.sort(key=lambda item: item[0], reverse=True)
        if candidates and candidates[0][0] > 0:
            return candidates[0][1]

        # Some ros_gz_bridge Pose_V -> TFMessage conversions preserve order but
        # drop entity names.  For dynamic_pose/info, transform 0 is the model
        # root pose, so the launch can opt into that stable fallback.
        index = self.fallback_transform_index
        if 0 <= index < len(msg.transforms):
            if not self._logged_source:
                self.get_logger().warn(
                    f'Pose topic has no frame names; using transform index {index}.')
            return msg.transforms[index]
        return None


def main(args=None):
    rclpy.init(args=args)
    node = GazeboModelTfRelay()
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
