#!/usr/bin/env python3
"""Check that drive-wheel spin does not translate the wheel center.

This node watches TF from each steering/pitch parent frame to its wheel link.
The translation should stay constant while wheel_*_joint rotates; only the
orientation of wheel_*_link should change.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _translation(tf_msg: TransformStamped):
    t = tf_msg.transform.translation
    return (float(t.x), float(t.y), float(t.z))


class WheelTfDebug(Node):
    def __init__(self):
        super().__init__('wheel_tf_debug')

        self.declare_parameter('publish_test_joint_states', False)
        self.declare_parameter('test_spin_rad_s', 8.0)
        self.declare_parameter('test_steering_1', 0.0)
        self.declare_parameter('test_steering_2', 0.0)
        self.declare_parameter('test_steering_3', 0.0)
        self.declare_parameter('parent_mode', 'auto')  # auto, steering, pitch
        self.declare_parameter('report_hz', 2.0)

        self.publish_test = bool(
            self.get_parameter('publish_test_joint_states').value)
        self.spin_speed = float(self.get_parameter('test_spin_rad_s').value)
        self.steering = [
            float(self.get_parameter('test_steering_1').value),
            float(self.get_parameter('test_steering_2').value),
            float(self.get_parameter('test_steering_3').value),
        ]
        self.parent_mode = str(self.get_parameter('parent_mode').value)
        report_hz = max(0.2, float(self.get_parameter('report_hz').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.js_pub = (
            self.create_publisher(JointState, '/joint_states', 10)
            if self.publish_test else None
        )

        self.initial = {}
        self.parent_frames = {}
        self.wheel_pos = [0.0, 0.0, 0.0]
        self.last_joint_positions = {}
        self.last_report = self.get_clock().now()

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_timer(0.02, self._tick)
        self.create_timer(1.0 / report_hz, self._report)

        mode = 'publishing test /joint_states' if self.publish_test else 'monitoring existing TF'
        self.get_logger().info(
            f'wheel_tf_debug ready ({mode}). Expected drift while wheels spin: '
            '< 0.5 mm. Orientation of wheel_*_link will still rotate.')

    def _joint_state_cb(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name.startswith('pitch_') or name.startswith('wheel_'):
                self.last_joint_positions[name] = float(position)

    def _tick(self):
        if not self.publish_test:
            return

        dt = 0.02
        for i in range(3):
            self.wheel_pos[i] += self.spin_speed * dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'steering_1_joint', 'steering_2_joint', 'steering_3_joint',
            'wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint',
        ]
        msg.position = self.steering + self.wheel_pos
        self.js_pub.publish(msg)

    def _candidate_parents(self, i):
        if self.parent_mode == 'pitch':
            return [f'pitch_{i}_link']
        if self.parent_mode == 'steering':
            return [f'steering_{i}_link']
        return [f'pitch_{i}_link', f'steering_{i}_link']

    def _lookup_parent_to_wheel(self, i):
        wheel = f'wheel_{i}_link'
        parents = [self.parent_frames[i]] if i in self.parent_frames else self._candidate_parents(i)
        last_exc = None
        for parent in parents:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    parent, wheel, rclpy.time.Time(), timeout=Duration(seconds=0.02))
                self.parent_frames[i] = parent
                return parent, tf_msg
            except TransformException as exc:
                last_exc = exc
        raise last_exc

    def _report(self):
        parts = []
        for i in (1, 2, 3):
            try:
                parent, tf_msg = self._lookup_parent_to_wheel(i)
            except TransformException as exc:
                parts.append(f'w{i}: TF missing ({exc})')
                continue

            t = _translation(tf_msg)
            if i not in self.initial:
                self.initial[i] = t
            drift_m = _norm(t[j] - self.initial[i][j] for j in range(3))
            radius_m = _norm(t)
            pitch = self.last_joint_positions.get(f'pitch_{i}_joint')
            pitch_text = 'pitch=n/a' if pitch is None else f'pitch={pitch:+.4f}rad'
            parts.append(
                f'w{i} parent={parent} xyz=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f}) '
                f'r={radius_m:.4f} drift={drift_m * 1000.0:.3f}mm {pitch_text}')

        self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = WheelTfDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
