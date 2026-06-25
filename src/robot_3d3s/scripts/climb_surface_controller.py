#!/usr/bin/env python3
"""Surface-aware teleop relay for the 3D3S climbing test.

The node accepts operator TwistStamped commands in a simple task frame and
publishes body-frame TwistStamped commands for swerve_controller.
"""

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


GROUND_APPROACH = 'GROUND_APPROACH'
SLIDE_CLIMB = 'SLIDE_CLIMB'
WALL_CAPTURE = 'WALL_CAPTURE'
WALL_CLIMB = 'WALL_CLIMB'
PIPE_CAPTURE = 'PIPE_CAPTURE'
PIPE_CLIMB = 'PIPE_CLIMB'
RECOVERY_STOP = 'RECOVERY_STOP'


@dataclass
class WheelStatus:
    wheel: str
    surface: str
    attached: bool
    gap_mm: float
    force_n: float


@dataclass
class AdhesionStatus:
    dominant_surface: str
    attached_count: int
    ground_count: int
    slide_count: int
    wall_count: int
    pipe_count: int
    wheels: List[WheelStatus]


def _quat_rotate(qx: float, qy: float, qz: float, qw: float,
                 x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Rotate vector v by unit quaternion q."""
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    # v' = v + qw * t + cross(q.xyz, t)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _limit_xy(vx: float, vy: float, limit: float) -> Tuple[float, float]:
    mag = math.hypot(vx, vy)
    if limit <= 0.0 or mag <= limit:
        return vx, vy
    scale = limit / mag
    return vx * scale, vy * scale


class ClimbSurfaceController(Node):
    def __init__(self):
        super().__init__('climb_surface_controller')

        self.declare_parameter('cmd_in_topic', '/climb_controller/cmd_vel_in')
        self.declare_parameter('cmd_out_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('state_topic', '/climb_controller/state')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('slide_angle_deg', 45.0)
        self.declare_parameter('min_attached_wheels', 2)
        self.declare_parameter('max_safe_gap_mm', 30.0)
        self.declare_parameter('status_timeout_s', 1.0)
        self.declare_parameter('tf_timeout_s', 0.3)
        self.declare_parameter('cmd_timeout_s', 2.0)
        self.declare_parameter('wall_confirm_s', 0.3)
        self.declare_parameter('capture_speed_limit_mps', 0.05)
        self.declare_parameter('climb_speed_limit_mps', 0.10)
        self.declare_parameter('motion_smoothing', True)
        self.declare_parameter('accel_limit_mps2', 0.12)
        self.declare_parameter('yaw_accel_limit_radps2', 0.45)
        self.declare_parameter('gap_slow_start_mm', 15.0)
        self.declare_parameter('low_force_slow_n', 300.0)
        self.declare_parameter('min_motion_scale', 0.20)
        self.declare_parameter('partial_contact_scale', 0.55)

        self.cmd_in_topic = self.get_parameter('cmd_in_topic').value
        self.cmd_out_topic = self.get_parameter('cmd_out_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self.slide_angle = math.radians(float(self.get_parameter('slide_angle_deg').value))
        self.min_attached = int(self.get_parameter('min_attached_wheels').value)
        self.max_safe_gap_mm = float(self.get_parameter('max_safe_gap_mm').value)
        self.status_timeout_s = float(self.get_parameter('status_timeout_s').value)
        self.tf_timeout_s = float(self.get_parameter('tf_timeout_s').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.wall_confirm_s = float(self.get_parameter('wall_confirm_s').value)
        self.capture_limit = float(self.get_parameter('capture_speed_limit_mps').value)
        self.climb_limit = float(self.get_parameter('climb_speed_limit_mps').value)
        self.motion_smoothing = bool(self.get_parameter('motion_smoothing').value)
        self.accel_limit = float(self.get_parameter('accel_limit_mps2').value)
        self.yaw_accel_limit = float(self.get_parameter('yaw_accel_limit_radps2').value)
        self.gap_slow_start_mm = float(self.get_parameter('gap_slow_start_mm').value)
        self.low_force_slow_n = float(self.get_parameter('low_force_slow_n').value)
        self.min_motion_scale = float(self.get_parameter('min_motion_scale').value)
        self.partial_contact_scale = float(self.get_parameter('partial_contact_scale').value)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_out_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.cmd_sub = self.create_subscription(
            TwistStamped, self.cmd_in_topic, self._cmd_cb, 10)
        self.status_sub = self.create_subscription(
            String, self.status_topic, self._status_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_cmd: Optional[TwistStamped] = None
        self.last_cmd_time = None
        self.last_status: Optional[AdhesionStatus] = None
        self.last_status_time = None
        self.wall_ready_since = None
        self.state = GROUND_APPROACH
        self.state_reason = 'startup'
        self.last_out = (0.0, 0.0, 0.0)
        self.last_out_time = None

        self.timer = self.create_timer(1.0 / self.publish_hz, self._timer_cb)

        self.get_logger().info(
            f'climb_surface_controller ready: {self.cmd_in_topic} -> '
            f'{self.cmd_out_topic}, status={self.status_topic}')

    def _cmd_cb(self, msg: TwistStamped):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _status_cb(self, msg: String):
        try:
            raw = json.loads(msg.data)
            wheels = [
                WheelStatus(
                    wheel=str(w.get('wheel', '')),
                    surface=str(w.get('surface', 'none')),
                    attached=bool(w.get('attached', False)),
                    gap_mm=float(w.get('gap_mm', 0.0)),
                    force_n=float(w.get('force_n', 0.0)),
                )
                for w in raw.get('wheels', [])
            ]
            self.last_status = AdhesionStatus(
                dominant_surface=str(raw.get('dominant_surface', 'none')),
                attached_count=int(raw.get('attached_count', 0)),
                ground_count=int(raw.get('ground_count', 0)),
                slide_count=int(raw.get('slide_count', 0)),
                wall_count=int(raw.get('wall_count', 0)),
                pipe_count=int(raw.get('pipe_count', 0)),
                wheels=wheels,
            )
            self.last_status_time = self.get_clock().now()
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'Ignoring malformed adhesion status: {exc}',
                throttle_duration_sec=2.0)

    def _timer_cb(self):
        now = self.get_clock().now()
        cmd = self.last_cmd
        if cmd is None or self._is_stale(self.last_cmd_time, self.cmd_timeout_s, now):
            self._publish_zero(now, 'no recent command')
            return

        state, reason = self._choose_state(now)
        self._set_state(state, reason)

        if state == RECOVERY_STOP:
            self._publish_zero(now, reason)
            return

        vx, vy, vz = self._world_vector_for_state(
            state, cmd.twist.linear.x, cmd.twist.linear.y)

        if state in (WALL_CAPTURE, PIPE_CAPTURE):
            vx, vy = _limit_xy(vx, vy, self.capture_limit)
            vz = max(-self.capture_limit, min(self.capture_limit, vz))
        elif state in (WALL_CLIMB, PIPE_CLIMB):
            vx, vy = _limit_xy(vx, vy, self.climb_limit)
            vz = max(-self.climb_limit, min(self.climb_limit, vz))

        motion_scale = self._motion_scale(state)
        vx *= motion_scale
        vy *= motion_scale
        vz *= motion_scale
        yaw_cmd = cmd.twist.angular.z * motion_scale

        body = self._world_vector_to_base(vx, vy, vz, now)
        if body is None:
            if state in (SLIDE_CLIMB, WALL_CAPTURE, WALL_CLIMB, PIPE_CAPTURE, PIPE_CLIMB):
                self._set_state(RECOVERY_STOP, 'TF unavailable during climb')
                self._publish_zero(now, 'TF unavailable during climb')
                return
            body = (cmd.twist.linear.x * motion_scale,
                    cmd.twist.linear.y * motion_scale,
                    0.0)

        body_x, body_y, yaw_cmd = self._rate_limit(
            body[0], body[1], yaw_cmd, now)

        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.base_frame
        out.twist.linear.x = body_x
        out.twist.linear.y = body_y
        out.twist.linear.z = 0.0
        out.twist.angular.z = yaw_cmd
        self.cmd_pub.publish(out)

    def _choose_state(self, now):
        status = self.last_status
        if status is None:
            return GROUND_APPROACH, 'waiting for adhesion status'

        if self._is_stale(self.last_status_time, self.status_timeout_s, now):
            if self.state in (SLIDE_CLIMB, WALL_CAPTURE, WALL_CLIMB, PIPE_CAPTURE, PIPE_CLIMB):
                return RECOVERY_STOP, 'adhesion status stale'
            return GROUND_APPROACH, 'adhesion status stale before climb'

        max_gap = max((w.gap_mm for w in status.wheels if w.attached), default=0.0)
        if max_gap > self.max_safe_gap_mm:
            return RECOVERY_STOP, f'gap too large: {max_gap:.1f} mm'

        attached = status.attached_count
        slide_or_wall = status.slide_count + status.wall_count

        if status.pipe_count >= self.min_attached:
            if self.wall_ready_since is None:
                self.wall_ready_since = now
                return PIPE_CAPTURE, 'confirming pipe attachment'
            if (now - self.wall_ready_since).nanoseconds * 1e-9 >= self.wall_confirm_s:
                return PIPE_CLIMB, 'pipe attachment confirmed'
            return PIPE_CAPTURE, 'confirming pipe attachment'

        if status.pipe_count > 0:
            self.wall_ready_since = None
            if self.state in (PIPE_CAPTURE, PIPE_CLIMB):
                return RECOVERY_STOP, 'lost pipe attachment'
            return GROUND_APPROACH, 'partial pipe contact'

        if status.wall_count >= self.min_attached:
            if self.wall_ready_since is None:
                self.wall_ready_since = now
                return WALL_CAPTURE, 'confirming wall attachment'
            if (now - self.wall_ready_since).nanoseconds * 1e-9 >= self.wall_confirm_s:
                return WALL_CLIMB, 'wall attachment confirmed'
            return WALL_CAPTURE, 'confirming wall attachment'

        self.wall_ready_since = None

        if status.wall_count > 0 and slide_or_wall >= self.min_attached:
            return WALL_CAPTURE, 'mixed slide/wall capture'

        if status.slide_count > 0:
            if attached >= self.min_attached:
                return SLIDE_CLIMB, 'slide attachment'
            if self.state in (SLIDE_CLIMB, WALL_CAPTURE, WALL_CLIMB, PIPE_CAPTURE, PIPE_CLIMB):
                return RECOVERY_STOP, 'lost climb attachment'
            return GROUND_APPROACH, 'partial slide contact'

        if self.state in (SLIDE_CLIMB, WALL_CAPTURE, WALL_CLIMB, PIPE_CAPTURE, PIPE_CLIMB) and attached < self.min_attached:
            return RECOVERY_STOP, 'lost climb attachment'

        return GROUND_APPROACH, 'ground approach'

    def _world_vector_for_state(self, state: str, cmd_x: float, cmd_y: float):
        if state == SLIDE_CLIMB:
            return (
                cmd_x * math.cos(self.slide_angle),
                cmd_y,
                cmd_x * math.sin(self.slide_angle),
            )
        if state in (WALL_CAPTURE, WALL_CLIMB, PIPE_CAPTURE, PIPE_CLIMB):
            return 0.0, cmd_y, cmd_x
        return cmd_x, cmd_y, 0.0

    def _world_vector_to_base(self, x: float, y: float, z: float, now):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.world_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s))
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {self.world_frame} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None

        q = transform.transform.rotation
        return _quat_rotate(q.x, q.y, q.z, q.w, x, y, z)

    def _motion_scale(self, state: str) -> float:
        if state == GROUND_APPROACH:
            return 1.0

        status = self.last_status
        if status is None or not status.wheels:
            return self.min_motion_scale

        attached = [w for w in status.wheels if w.attached]
        if len(attached) < self.min_attached:
            return max(self.min_motion_scale, self.partial_contact_scale)

        scale = 1.0

        max_gap = max((w.gap_mm for w in attached), default=0.0)
        gap_stop_mm = max(self.gap_slow_start_mm + 1.0, self.max_safe_gap_mm)
        if max_gap > self.gap_slow_start_mm:
            ratio = ((max_gap - self.gap_slow_start_mm) /
                     (gap_stop_mm - self.gap_slow_start_mm))
            gap_scale = 1.0 - max(0.0, min(1.0, ratio))
            scale = min(scale, self.min_motion_scale +
                        (1.0 - self.min_motion_scale) * gap_scale)

        min_force = min((w.force_n for w in attached), default=self.low_force_slow_n)
        if self.low_force_slow_n > 0.0 and min_force < self.low_force_slow_n:
            force_scale = max(0.0, min(1.0, min_force / self.low_force_slow_n))
            scale = min(scale, self.min_motion_scale +
                        (1.0 - self.min_motion_scale) * force_scale)

        if state in (WALL_CAPTURE, PIPE_CAPTURE):
            scale = min(scale, 0.65)

        return max(self.min_motion_scale, min(1.0, scale))

    def _rate_limit(self, vx: float, vy: float, wz: float, now):
        if not self.motion_smoothing:
            self.last_out = (vx, vy, wz)
            self.last_out_time = now
            return vx, vy, wz

        if self.last_out_time is None:
            self.last_out = (vx, vy, wz)
            self.last_out_time = now
            return vx, vy, wz

        dt = (now - self.last_out_time).nanoseconds * 1e-9
        self.last_out_time = now
        if dt <= 0.0 or dt > 1.0:
            self.last_out = (vx, vy, wz)
            return vx, vy, wz

        last_vx, last_vy, last_wz = self.last_out
        max_dv = max(0.0, self.accel_limit) * dt
        dx = vx - last_vx
        dy = vy - last_vy
        dmag = math.hypot(dx, dy)
        if max_dv > 0.0 and dmag > max_dv:
            ratio = max_dv / dmag
            vx = last_vx + dx * ratio
            vy = last_vy + dy * ratio

        max_dw = max(0.0, self.yaw_accel_limit) * dt
        dw = wz - last_wz
        if max_dw > 0.0 and abs(dw) > max_dw:
            wz = last_wz + math.copysign(max_dw, dw)

        self.last_out = (vx, vy, wz)
        return vx, vy, wz

    def _publish_zero(self, now, reason: str):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        self.cmd_pub.publish(msg)
        self.last_out = (0.0, 0.0, 0.0)
        self.last_out_time = now
        if reason:
            self.get_logger().warn(f'climb controller stop: {reason}',
                                   throttle_duration_sec=2.0)

    def _set_state(self, state: str, reason: str):
        if state == self.state and reason == self.state_reason:
            return

        if state != self.state:
            self.last_out_time = None
        self.state = state
        self.state_reason = reason
        text = f'{state}: {reason}'
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _is_stale(stamp, timeout_s: float, now) -> bool:
        if stamp is None:
            return True
        return (now - stamp).nanoseconds * 1e-9 > timeout_s


def main(args=None):
    rclpy.init(args=args)
    node = ClimbSurfaceController()
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
