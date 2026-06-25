#!/usr/bin/env python3
"""RViz click-to-climb planner for the ferrous mockup structure.

This node is intentionally small and mockup-specific.  It converts RViz
PublishPoint clicks into task-frame velocity commands for
climb_surface_controller.py:

* before magnetic contact, drive toward the front face of the mockup;
* after at least two wheels are attached to the ferrous mesh, climb toward the
  clicked height;
* slow or hold when contact quality is poor.

Gazebo remains the dynamics source of truth.  RViz is used for goal selection,
route visualization, and robot TF display.
"""

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


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


class MockupClimbPlanner(Node):
    def __init__(self):
        super().__init__('mockup_climb_planner')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('cmd_topic', '/climb_controller/cmd_vel_in')
        self.declare_parameter('marker_topic', '/mockup_climb_markers')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('min_attached_wheels', 2)
        self.declare_parameter('status_timeout_s', 0.7)
        self.declare_parameter('tf_timeout_s', 0.1)
        self.declare_parameter('mockup_front_y', 0.0)
        self.declare_parameter('mockup_contact_y', -0.03)
        self.declare_parameter('mockup_x_min', 0.0)
        self.declare_parameter('mockup_x_max', 3.7)
        self.declare_parameter('mockup_z_min', 0.0)
        self.declare_parameter('mockup_z_max', 3.11)
        self.declare_parameter('default_target_x', 1.0)
        self.declare_parameter('approach_speed_mps', 0.12)
        self.declare_parameter('climb_speed_mps', 0.11)
        self.declare_parameter('lateral_speed_mps', 0.05)
        self.declare_parameter('kp_x', 0.7)
        self.declare_parameter('kp_y', 0.9)
        self.declare_parameter('kp_z', 0.7)
        self.declare_parameter('goal_tolerance_m', 0.06)
        self.declare_parameter('gap_slow_start_mm', 15.0)
        self.declare_parameter('gap_stop_mm', 30.0)
        self.declare_parameter('low_force_slow_n', 300.0)
        self.declare_parameter('min_motion_scale', 0.25)
        self.declare_parameter('mockup_mesh_resource',
                               'package://robot_3d3s/models/mockup_structure/meshes/mockup_structure_visual.dae')

        gp = self.get_parameter
        self.world_frame = str(gp('world_frame').value)
        self.base_frame = str(gp('base_frame').value)
        self.publish_hz = max(2.0, float(gp('publish_hz').value))
        self.min_attached = int(gp('min_attached_wheels').value)
        self.status_timeout = float(gp('status_timeout_s').value)
        self.tf_timeout = float(gp('tf_timeout_s').value)
        self.front_y = float(gp('mockup_front_y').value)
        self.contact_y = float(gp('mockup_contact_y').value)
        self.x_min = float(gp('mockup_x_min').value)
        self.x_max = float(gp('mockup_x_max').value)
        self.z_min = float(gp('mockup_z_min').value)
        self.z_max = float(gp('mockup_z_max').value)
        self.default_x = float(gp('default_target_x').value)
        self.approach_speed = float(gp('approach_speed_mps').value)
        self.climb_speed = float(gp('climb_speed_mps').value)
        self.lateral_speed = float(gp('lateral_speed_mps').value)
        self.kp_x = float(gp('kp_x').value)
        self.kp_y = float(gp('kp_y').value)
        self.kp_z = float(gp('kp_z').value)
        self.goal_tol = float(gp('goal_tolerance_m').value)
        self.gap_slow_start = float(gp('gap_slow_start_mm').value)
        self.gap_stop = max(
            self.gap_slow_start + 1.0, float(gp('gap_stop_mm').value))
        self.low_force_slow = float(gp('low_force_slow_n').value)
        self.min_scale = _clamp(float(gp('min_motion_scale').value), 0.05, 1.0)
        self.mesh_resource = str(gp('mockup_mesh_resource').value)

        self.goal: Optional[Tuple[float, float, float]] = None
        self.reached = False
        self.attached = 0
        self.pipe_count = 0
        self.max_gap_mm = 0.0
        self.min_force_n = 0.0
        self.last_status_t = None

        self.cmd_pub = self.create_publisher(
            TwistStamped, str(gp('cmd_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, '/mockup_climb_planner/debug', 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray, str(gp('marker_topic').value), marker_qos)

        self.create_subscription(
            PointStamped, str(gp('clicked_point_topic').value),
            self._clicked_cb, 10)
        self.create_subscription(
            PoseStamped, str(gp('goal_pose_topic').value),
            self._pose_goal_cb, 10)
        self.create_subscription(
            String, str(gp('status_topic').value), self._status_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.publish_hz, self._tick)
        self.create_timer(0.5, self._publish_markers)

        self.get_logger().info(
            'mockup_climb_planner ready. Click the ferrous mockup in RViz; '
            f'commands go to {gp("cmd_topic").value}.')

    def _clicked_cb(self, msg: PointStamped):
        self._set_goal_from_world_point(
            msg.point.x, msg.point.y, msg.point.z, 'Publish Point')

    def _pose_goal_cb(self, msg: PoseStamped):
        # RViz SetGoal is 2D by default, so z is usually 0.  For this mockup
        # mode, interpret a 2D goal as x=mockup horizontal coordinate,
        # y=target climb height.  If a real 3D PoseStamped is sent, use its z.
        raw_x = msg.pose.position.x
        raw_y = msg.pose.position.y
        raw_z = msg.pose.position.z
        if raw_z > self.z_min + 0.05:
            self._set_goal_from_world_point(raw_x, raw_y, raw_z, 'Pose goal')
        else:
            x = _clamp(raw_x, self.x_min, self.x_max)
            z = _clamp(raw_y, self.z_min + 0.25, self.z_max - 0.25)
            self._set_goal(x, self.contact_y, z, '2D Goal Pose')

    def _set_goal_from_world_point(self, x_raw: float, y_raw: float,
                                   z_raw: float, source: str):
        x = _clamp(x_raw, self.x_min, self.x_max)
        z = _clamp(z_raw, self.z_min + 0.25, self.z_max - 0.25)
        y = self.contact_y
        if y_raw < self.front_y:
            y = _clamp(y_raw, -0.25, self.contact_y)
        self._set_goal(x, y, z, source)

    def _set_goal(self, x: float, y: float, z: float, source: str):
        self.goal = (x, y, z)
        self.reached = False
        self._publish_markers()
        self.get_logger().info(
            f'New mockup climb goal from {source}: '
            f'x={x:.2f}, y={y:.2f}, z={z:.2f}')

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

        wheels = data.get('wheels', [])
        attached = [w for w in wheels if w.get('attached')]
        self.attached = int(data.get('attached_count', len(attached)))
        self.pipe_count = int(data.get('pipe_count', 0))
        self.max_gap_mm = max(
            (float(w.get('gap_mm', 0.0)) for w in attached), default=0.0)
        self.min_force_n = min(
            (float(w.get('force_n', 0.0)) for w in attached), default=0.0)
        self.last_status_t = self.get_clock().now()

    def _robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot read {self.world_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return t.x, t.y, t.z

    def _status_stale(self):
        if self.last_status_t is None:
            return True
        age = (self.get_clock().now() - self.last_status_t).nanoseconds * 1e-9
        return age > self.status_timeout

    def _motion_scale(self):
        if self.attached < self.min_attached:
            return 1.0
        scale = 1.0
        if self.max_gap_mm > self.gap_slow_start:
            ratio = ((self.max_gap_mm - self.gap_slow_start) /
                     (self.gap_stop - self.gap_slow_start))
            scale = min(scale, 1.0 - _clamp(ratio, 0.0, 1.0) *
                        (1.0 - self.min_scale))
        if 0.0 < self.min_force_n < self.low_force_slow:
            scale = min(scale, _clamp(
                self.min_force_n / self.low_force_slow,
                self.min_scale, 1.0))
        return _clamp(scale, self.min_scale, 1.0)

    def _tick(self):
        if self.goal is None or self.reached:
            return

        pos = self._robot_position()
        if pos is None:
            self._publish_cmd(0.0, 0.0, 0.0, 'no TF')
            return

        gx, gy, gz = self.goal
        rx, ry, rz = pos
        contact_ready = (
            not self._status_stale() and
            self.attached >= self.min_attached and
            self.pipe_count >= self.min_attached)

        if contact_ready:
            z_err = gz - rz
            y_err = gy - ry
            if abs(z_err) <= self.goal_tol and abs(y_err) <= self.goal_tol:
                self.reached = True
                self._publish_cmd(0.0, 0.0, 0.0, 'goal reached')
                self.get_logger().info('Mockup climb goal reached. Holding.')
                return
            scale = self._motion_scale()
            task_x = _clamp(self.kp_z * z_err,
                            -self.climb_speed, self.climb_speed) * scale
            task_y = _clamp(self.kp_y * y_err,
                            -self.lateral_speed, self.lateral_speed) * scale
            self._publish_cmd(task_x, task_y, 0.0, 'climb')
            return

        # Approach in the ground task frame.  Keep x aligned with the selected
        # mockup member, then drive toward the front ferrous face.
        x_err = gx - rx
        y_err = self.contact_y - ry
        task_x = _clamp(self.kp_x * x_err,
                        -self.lateral_speed, self.lateral_speed)
        task_y = _clamp(self.kp_y * y_err,
                        -self.approach_speed, self.approach_speed)
        self._publish_cmd(task_x, task_y, 0.0, 'approach')

    def _publish_cmd(self, x: float, y: float, wz: float, mode: str):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

        dbg = String()
        dbg.data = json.dumps({
            'mode': mode,
            'goal': self.goal,
            'attached': self.attached,
            'pipe_count': self.pipe_count,
            'max_gap_mm': self.max_gap_mm,
            'min_force_n': self.min_force_n,
            'cmd': [x, y, wz],
        })
        self.debug_pub.publish(dbg)

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._mockup_mesh(now))
        markers.markers.extend(self._mockup_fallback_markers(now))
        if self.goal is not None:
            markers.markers.append(self._goal_marker(now))
            marker = self._route_marker(now)
            if marker is not None:
                markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _mockup_mesh(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'mockup_structure'
        marker.id = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = False
        marker.pose.position.z = 0.0
        qx, qy, qz, qw = _quat_from_rpy(1.5708, 0.0, 0.0)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.70
        marker.color.g = 0.08
        marker.color.b = 0.08
        marker.color.a = 1.0
        return marker

    def _mockup_fallback_markers(self, stamp):
        markers = []

        volume = Marker()
        volume.header.frame_id = self.world_frame
        volume.header.stamp = stamp
        volume.ns = 'mockup_structure_fallback'
        volume.id = 10
        volume.type = Marker.CUBE
        volume.action = Marker.ADD
        volume.pose.position.x = 0.5 * (self.x_min + self.x_max)
        volume.pose.position.y = 0.55
        volume.pose.position.z = 0.5 * (self.z_min + self.z_max)
        volume.pose.orientation.w = 1.0
        volume.scale.x = self.x_max - self.x_min
        volume.scale.y = 1.10
        volume.scale.z = self.z_max - self.z_min
        volume.color.r = 0.70
        volume.color.g = 0.08
        volume.color.b = 0.08
        volume.color.a = 0.22
        markers.append(volume)

        front_face = Marker()
        front_face.header.frame_id = self.world_frame
        front_face.header.stamp = stamp
        front_face.ns = 'mockup_structure_fallback'
        front_face.id = 11
        front_face.type = Marker.CUBE
        front_face.action = Marker.ADD
        front_face.pose.position.x = 0.5 * (self.x_min + self.x_max)
        front_face.pose.position.y = self.contact_y
        front_face.pose.position.z = 0.5 * (self.z_min + self.z_max)
        front_face.pose.orientation.w = 1.0
        front_face.scale.x = self.x_max - self.x_min
        front_face.scale.y = 0.025
        front_face.scale.z = self.z_max - self.z_min
        front_face.color.r = 0.85
        front_face.color.g = 0.05
        front_face.color.b = 0.06
        front_face.color.a = 0.55
        markers.append(front_face)

        return markers

    def _goal_marker(self, stamp):
        gx, gy, gz = self.goal
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'mockup_goal'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = gz
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.09
        marker.scale.y = 0.09
        marker.scale.z = 0.09
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.95
        return marker

    def _route_marker(self, stamp):
        pos = self._robot_position()
        if pos is None or self.goal is None:
            return None
        rx, ry, rz = pos
        gx, gy, gz = self.goal
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'mockup_route'
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.35
        marker.color.a = 0.95
        marker.points.append(Point(x=rx, y=ry, z=rz))
        marker.points.append(Point(x=gx, y=self.contact_y, z=max(rz, 0.15)))
        marker.points.append(Point(x=gx, y=gy, z=gz))
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MockupClimbPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0, 0.0, 'shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
