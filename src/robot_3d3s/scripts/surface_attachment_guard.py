#!/usr/bin/env python3
"""Keep the robot attached to a known surface while debugging climbing logic.

This is a simulation guard, not a physical adhesion model.  It projects the
Gazebo model pose back onto the configured pipe/cylinder whenever the adhesion
status says enough wheels are attached and the model drifts away from the
surface.  Use it to debug the planner and dashboard without Gazebo letting the
robot fall because of imperfect contact dynamics.
"""

import json
import math
import subprocess
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def _clamp(value: float, lower: float, upper: float):
    return max(lower, min(upper, value))


class SurfaceAttachmentGuard(Node):
    def __init__(self):
        super().__init__('surface_attachment_guard')

        self.declare_parameter('enabled', True)
        self.declare_parameter('world_name', 'horizontal_pipe_test')
        self.declare_parameter('model_name', 'robot_3d3s')
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('surface_mode', 'cylinder')
        self.declare_parameter('experimental_polygon_surface_selection', False)
        self.declare_parameter('cylinder_axis', 'x')
        self.declare_parameter('axis_x', 0.0)
        self.declare_parameter('axis_y', 0.0)
        self.declare_parameter('axis_z', 1.20)
        self.declare_parameter('surface_radius_m', 1.20)
        self.declare_parameter('surface_length_m', 6.0)
        self.declare_parameter('base_surface_offset_m', -0.03)
        self.declare_parameter('min_attached_wheels', 2)
        self.declare_parameter('geometric_capture', False)
        self.declare_parameter('capture_tolerance_m', 0.25)
        self.declare_parameter('latch_after_contact', True)
        self.declare_parameter('latch_timeout_s', 3.0)
        self.declare_parameter('startup_hold_s', 0.0)
        self.declare_parameter('startup_hold_theta_deg', 0.0)
        self.declare_parameter('align_orientation', True)
        self.declare_parameter('orientation_tolerance_deg', 2.0)
        self.declare_parameter('status_timeout_s', 0.7)
        self.declare_parameter('tf_timeout_s', 0.05)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('radial_tolerance_m', 0.015)
        self.declare_parameter('max_correction_m', 0.08)
        self.declare_parameter('correction_method', 'wrench')
        self.declare_parameter('position_correction_gain', 0.45)
        self.declare_parameter('orientation_correction_gain', 0.35)
        self.declare_parameter('attachment_stiffness_n_per_m', 5000.0)
        self.declare_parameter('attachment_damping_n_per_mps', 700.0)
        self.declare_parameter('max_attachment_force_n', 900.0)
        self.declare_parameter('orientation_stiffness_nm_per_rad', 70.0)
        self.declare_parameter('max_orientation_torque_nm', 35.0)
        self.declare_parameter('emergency_pose_error_m', 0.22)
        self.declare_parameter('allow_emergency_pose_correction', False)
        self.declare_parameter('service_timeout_ms', 250)
        self.declare_parameter('process_timeout_s', 0.35)

        gp = self.get_parameter
        self.enabled = bool(gp('enabled').value)
        self.world_name = str(gp('world_name').value)
        self.model_name = str(gp('model_name').value)
        self.base_link_name = str(gp('base_link_name').value)
        self.world_frame = str(gp('world_frame').value)
        self.base_frame = str(gp('base_frame').value)
        self.surface_mode = str(gp('surface_mode').value).lower()
        self.experimental_polygon_surface_selection = bool(
            gp('experimental_polygon_surface_selection').value)
        self.axis = str(gp('cylinder_axis').value).lower()
        self.axis_point = (
            float(gp('axis_x').value),
            float(gp('axis_y').value),
            float(gp('axis_z').value),
        )
        self.surface_radius = max(0.05, float(gp('surface_radius_m').value))
        self.length = max(0.1, float(gp('surface_length_m').value))
        self.active_surface_name = 'configured_surface'
        self.experimental_surfaces = self._experimental_polygon_surfaces()
        if (self.experimental_polygon_surface_selection and
                self.experimental_surfaces):
            self._set_active_surface(self.experimental_surfaces[0])
        self.base_surface_offset = float(gp('base_surface_offset_m').value)
        self.min_attached = int(gp('min_attached_wheels').value)
        self.geometric_capture = bool(gp('geometric_capture').value)
        self.capture_tolerance = max(0.0, float(gp('capture_tolerance_m').value))
        self.latch_after_contact = bool(gp('latch_after_contact').value)
        self.latch_timeout = max(0.0, float(gp('latch_timeout_s').value))
        self.startup_hold_s = max(0.0, float(gp('startup_hold_s').value))
        self.startup_hold_theta = math.radians(
            float(gp('startup_hold_theta_deg').value))
        self.align_orientation = bool(gp('align_orientation').value)
        self.orientation_tolerance = math.radians(max(
            0.0, float(gp('orientation_tolerance_deg').value)))
        self.status_timeout = float(gp('status_timeout_s').value)
        self.tf_timeout = float(gp('tf_timeout_s').value)
        self.radial_tolerance = max(0.0, float(gp('radial_tolerance_m').value))
        self.max_correction = max(0.001, float(gp('max_correction_m').value))
        self.correction_method = str(gp('correction_method').value).lower()
        if self.correction_method not in ('pose', 'wrench'):
            self.get_logger().warn(
                f'unknown correction_method={self.correction_method}; using pose')
            self.correction_method = 'pose'
        self.position_gain = _clamp(
            float(gp('position_correction_gain').value), 0.01, 1.0)
        self.orientation_gain = _clamp(
            float(gp('orientation_correction_gain').value), 0.01, 1.0)
        self.attachment_stiffness = max(
            0.0, float(gp('attachment_stiffness_n_per_m').value))
        self.attachment_damping = max(
            0.0, float(gp('attachment_damping_n_per_mps').value))
        self.max_attachment_force = max(
            0.0, float(gp('max_attachment_force_n').value))
        self.orientation_stiffness = max(
            0.0, float(gp('orientation_stiffness_nm_per_rad').value))
        self.max_orientation_torque = max(
            0.0, float(gp('max_orientation_torque_nm').value))
        self.emergency_pose_error = max(
            0.0, float(gp('emergency_pose_error_m').value))
        self.allow_emergency_pose_correction = bool(
            gp('allow_emergency_pose_correction').value)
        self.service_timeout_ms = int(gp('service_timeout_ms').value)
        self.process_timeout = float(gp('process_timeout_s').value)
        rate_hz = max(1.0, float(gp('rate_hz').value))
        self.period_s = 1.0 / rate_hz

        self.attached = 0
        self.pipe_count = 0
        self.last_status_t = None
        self.last_attached_t = None
        self.first_tf_wall_t = None
        self.last_pose_wall_t = None
        self.last_position = None
        self._last_log_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            String, str(gp('status_topic').value), self._status_cb, 10)
        self.create_timer(self.period_s, self._tick)

        self.get_logger().info(
            'surface_attachment_guard '
            f'{"enabled" if self.enabled else "disabled"}: '
            f'{self.model_name} on {self.surface_mode}/{self.axis}, '
            f'active_surface={self.active_surface_name}, '
            f'r={self.surface_radius:.3f}m, '
            f'base_offset={self.base_surface_offset:.3f}m, '
            f'startup_hold={self.startup_hold_s:.1f}s, '
            f'method={self.correction_method}')

    @staticmethod
    def _experimental_polygon_surfaces():
        return (
            {
                'name': 'left_vertical_pipe',
                'axis': 'z',
                'axis_point': (-1.5656, 0.0, 1.5),
                'radius': 0.4572,
                'length': 3.0,
            },
            {
                'name': 'right_vertical_pipe',
                'axis': 'z',
                'axis_point': (1.5656, 0.0, 1.5),
                'radius': 0.2540,
                'length': 3.0,
            },
            {
                'name': 'horizontal_pipe',
                'axis': 'x',
                'axis_point': (0.0, 0.0, 1.246),
                'radius': 0.2540,
                'length': 3.1312,
            },
        )

    def _set_active_surface(self, surface):
        self.active_surface_name = str(surface['name'])
        self.axis = str(surface['axis'])
        self.axis_point = tuple(surface['axis_point'])
        self.surface_radius = float(surface['radius'])
        self.length = float(surface['length'])

    @staticmethod
    def _surface_axis_value(surface, x: float, y: float, z: float):
        axis = surface['axis']
        if axis == 'x':
            return x
        if axis == 'y':
            return y
        return z

    @staticmethod
    def _surface_projection(surface, x: float, y: float, z: float,
                            base_surface_offset: float):
        ax, ay, az = surface['axis_point']
        axis = surface['axis']
        radius = max(0.01, float(surface['radius']) + base_surface_offset)
        half_len = 0.5 * float(surface['length'])
        axis_center = SurfaceAttachmentGuard._surface_axis_value(
            surface, ax, ay, az)

        if axis == 'x':
            axis_value = _clamp(x, axis_center - half_len, axis_center + half_len)
            ry, rz = y - ay, z - az
            radial = math.hypot(ry, rz)
            if radial < 1e-6:
                return None
            scale = radius / radial
            px, py, pz = axis_value, ay + ry * scale, az + rz * scale
        elif axis == 'y':
            axis_value = _clamp(y, axis_center - half_len, axis_center + half_len)
            rx, rz = x - ax, z - az
            radial = math.hypot(rx, rz)
            if radial < 1e-6:
                return None
            scale = radius / radial
            px, py, pz = ax + rx * scale, axis_value, az + rz * scale
        else:
            axis_value = _clamp(z, axis_center - half_len, axis_center + half_len)
            rx, ry = x - ax, y - ay
            radial = math.hypot(rx, ry)
            if radial < 1e-6:
                return None
            scale = radius / radial
            px, py, pz = ax + rx * scale, ay + ry * scale, axis_value

        return px, py, pz, math.dist((px, py, pz), (x, y, z))

    def _closest_experimental_surface(self, x: float, y: float, z: float):
        best = None
        for surface in self.experimental_surfaces:
            projection = self._surface_projection(
                surface, x, y, z, self.base_surface_offset)
            if projection is None:
                continue
            px, py, pz, distance = projection
            candidate = (distance, surface, px, py, pz)
            if best is None or candidate[0] < best[0]:
                best = candidate
        return best

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.attached = int(data.get('attached_count', 0))
        pipe_count = int(data.get('pipe_count', 0))
        wheels = data.get('wheels', [])
        wheel_pipe_count = 0
        for wheel in wheels:
            if wheel.get('attached') and str(wheel.get('surface', '')) in ('pipe', 'cylinder'):
                wheel_pipe_count += 1
        self.pipe_count = max(pipe_count, wheel_pipe_count)
        self.last_status_t = self.get_clock().now()
        if self.attached >= self.min_attached and self.pipe_count >= self.min_attached:
            self.last_attached_t = self.last_status_t

    def _status_stale(self):
        if self.last_status_t is None:
            return True
        age = (self.get_clock().now() - self.last_status_t).nanoseconds * 1e-9
        return age > self.status_timeout

    def _guard_active(self):
        if self.attached >= self.min_attached and self.pipe_count >= self.min_attached:
            return True
        if not self.latch_after_contact or self.last_attached_t is None:
            return False
        age = (self.get_clock().now() - self.last_attached_t).nanoseconds * 1e-9
        return age <= self.latch_timeout

    def _tick(self):
        if not self.enabled or self.surface_mode != 'cylinder':
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                f'attachment guard cannot read TF: {exc}',
                throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        now_wall_t = time.monotonic()
        velocity = (0.0, 0.0, 0.0)
        if self.last_pose_wall_t is not None and self.last_position is not None:
            dt = max(1e-6, now_wall_t - self.last_pose_wall_t)
            velocity = (
                (t.x - self.last_position[0]) / dt,
                (t.y - self.last_position[1]) / dt,
                (t.z - self.last_position[2]) / dt,
            )
        self.last_pose_wall_t = now_wall_t
        self.last_position = (t.x, t.y, t.z)
        if self.first_tf_wall_t is None:
            self.first_tf_wall_t = now_wall_t
        if (self.experimental_polygon_surface_selection and
                not self._startup_theta_hold_active()):
            selected = self._closest_experimental_surface(t.x, t.y, t.z)
            if selected is not None:
                _, surface, _, _, _ = selected
                self._set_active_surface(surface)
        projected = self._project_to_surface(t.x, t.y, t.z)
        if projected is None:
            return
        px, py, pz, correction, radial_error = projected
        guard_active = self._guard_active()
        captured = (
            self.geometric_capture and
            radial_error <= self.capture_tolerance)
        if self._status_stale() and not guard_active and not captured:
            return
        if not guard_active and not captured:
            return

        current_q = (q.x, q.y, q.z, q.w)
        target_q = current_q
        orientation_error = 0.0
        if self.align_orientation:
            target_q = self._surface_orientation(px, py, pz)
            if target_q is not None:
                orientation_error = self._orientation_error(
                    current_q, target_q)
            else:
                target_q = current_q

        if (correction <= self.radial_tolerance and
                orientation_error <= self.orientation_tolerance):
            return

        if (self.correction_method == 'wrench' and
                (not self.allow_emergency_pose_correction or
                 radial_error <= self.emergency_pose_error)):
            self._apply_attachment_wrench(
                t.x, t.y, t.z, current_q,
                px, py, pz, orientation_error, velocity)
            return

        px = t.x + (px - t.x) * self.position_gain
        py = t.y + (py - t.y) * self.position_gain
        pz = t.z + (pz - t.z) * self.position_gain
        target_q = self._slerp(current_q, target_q, self.orientation_gain)

        self._set_model_pose(
            px, py, pz,
            target_q[0], target_q[1], target_q[2], target_q[3],
            correction, orientation_error)

    def _axis_value(self, x: float, y: float, z: float):
        if self.axis == 'x':
            return x
        if self.axis == 'y':
            return y
        return z

    def _project_to_surface(self, x: float, y: float, z: float):
        ax, ay, az = self.axis_point
        half_len = 0.5 * self.length
        target_radius = max(0.01, self.surface_radius + self.base_surface_offset)
        hold_theta = self._startup_theta_hold_active()

        if self.axis == 'x':
            axis_value = _clamp(x, ax - half_len, ax + half_len)
            if hold_theta:
                px = axis_value
                py = ay + target_radius * math.cos(self.startup_hold_theta)
                pz = az + target_radius * math.sin(self.startup_hold_theta)
            else:
                ry, rz = y - ay, z - az
                radial = math.hypot(ry, rz)
                if radial < 1e-6:
                    return None
                scale = target_radius / radial
                px, py, pz = axis_value, ay + ry * scale, az + rz * scale
        elif self.axis == 'y':
            axis_value = _clamp(y, ay - half_len, ay + half_len)
            if hold_theta:
                px = ax + target_radius * math.sin(self.startup_hold_theta)
                py = axis_value
                pz = az + target_radius * math.cos(self.startup_hold_theta)
            else:
                rx, rz = x - ax, z - az
                radial = math.hypot(rx, rz)
                if radial < 1e-6:
                    return None
                scale = target_radius / radial
                px, py, pz = ax + rx * scale, axis_value, az + rz * scale
        else:
            axis_value = _clamp(z, az - half_len, az + half_len)
            if hold_theta:
                px = ax + target_radius * math.cos(self.startup_hold_theta)
                py = ay + target_radius * math.sin(self.startup_hold_theta)
                pz = axis_value
            else:
                rx, ry = x - ax, y - ay
                radial = math.hypot(rx, ry)
                if radial < 1e-6:
                    return None
                scale = target_radius / radial
                px, py, pz = ax + rx * scale, ay + ry * scale, axis_value

        dx, dy, dz = px - x, py - y, pz - z
        raw_correction = math.sqrt(dx * dx + dy * dy + dz * dz)
        correction = raw_correction
        if correction > self.max_correction:
            ratio = self.max_correction / correction
            px = x + dx * ratio
            py = y + dy * ratio
            pz = z + dz * ratio
            correction = self.max_correction
        return px, py, pz, correction, raw_correction

    def _startup_theta_hold_active(self):
        if self.startup_hold_s <= 0.0 or self.first_tf_wall_t is None:
            return False
        return (time.monotonic() - self.first_tf_wall_t) <= self.startup_hold_s

    def _axis_vector(self):
        if self.axis == 'x':
            return 1.0, 0.0, 0.0
        if self.axis == 'y':
            return 0.0, 1.0, 0.0
        return 0.0, 0.0, 1.0

    def _surface_normal(self, x: float, y: float, z: float):
        ax, ay, az = self.axis_point
        if self.axis == 'x':
            nx, ny, nz = 0.0, y - ay, z - az
        elif self.axis == 'y':
            nx, ny, nz = x - ax, 0.0, z - az
        else:
            nx, ny, nz = x - ax, y - ay, 0.0
        return self._normalize(nx, ny, nz)

    def _surface_orientation(self, x: float, y: float, z: float):
        radial = self._surface_normal(x, y, z)
        if radial is None:
            return None
        axis = self._axis_vector()

        # The robot's local +Z should point away from the pipe center.  Local
        # +X follows the decreasing-theta tangent, matching the default top
        # spawn where +X points along +Y for the horizontal pipe.
        x_axis = self._cross(radial, axis)
        x_axis = self._normalize(*x_axis)
        if x_axis is None:
            return None
        z_axis = radial
        y_axis = self._cross(z_axis, x_axis)
        y_axis = self._normalize(*y_axis)
        if y_axis is None:
            return None
        return self._quat_from_basis(x_axis, y_axis, z_axis)

    @staticmethod
    def _cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _normalize(x: float, y: float, z: float):
        mag = math.sqrt(x * x + y * y + z * z)
        if mag <= 1e-9:
            return None
        return x / mag, y / mag, z / mag

    @staticmethod
    def _quat_from_basis(x_axis, y_axis, z_axis):
        # Rotation matrix with basis vectors as columns.
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m21 - m12) / s
            qy = (m02 - m20) / s
            qz = (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        return qx / norm, qy / norm, qz / norm, qw / norm

    @staticmethod
    def _orientation_error(current, target):
        dot = abs(
            current[0] * target[0] +
            current[1] * target[1] +
            current[2] * target[2] +
            current[3] * target[3])
        dot = max(-1.0, min(1.0, dot))
        return 2.0 * math.acos(dot)

    @staticmethod
    def _slerp(current, target, ratio: float):
        ratio = max(0.0, min(1.0, ratio))
        dot = (
            current[0] * target[0] +
            current[1] * target[1] +
            current[2] * target[2] +
            current[3] * target[3])
        if dot < 0.0:
            target = (-target[0], -target[1], -target[2], -target[3])
            dot = -dot
        if dot > 0.9995:
            out = tuple(current[i] + ratio * (target[i] - current[i])
                        for i in range(4))
        else:
            theta_0 = math.acos(max(-1.0, min(1.0, dot)))
            sin_theta_0 = math.sin(theta_0)
            theta = theta_0 * ratio
            sin_theta = math.sin(theta)
            s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0
            out = tuple(s0 * current[i] + s1 * target[i] for i in range(4))
        norm = math.sqrt(sum(v * v for v in out))
        if norm <= 1e-9:
            return current
        return tuple(v / norm for v in out)

    @staticmethod
    def _quat_rotate(q, x: float, y: float, z: float):
        qx, qy, qz, qw = q
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    def _apply_attachment_wrench(self, x: float, y: float, z: float,
                                 current_q, target_x: float, target_y: float,
                                 target_z: float, orientation_error: float,
                                 velocity=(0.0, 0.0, 0.0)):
        dx, dy, dz = target_x - x, target_y - y, target_z - z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        fx = self.attachment_stiffness * dx - self.attachment_damping * velocity[0]
        fy = self.attachment_stiffness * dy - self.attachment_damping * velocity[1]
        fz = self.attachment_stiffness * dz - self.attachment_damping * velocity[2]
        force_mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if force_mag > self.max_attachment_force and force_mag > 1e-9:
            scale = self.max_attachment_force / force_mag
            fx *= scale
            fy *= scale
            fz *= scale

        tx, ty, tz = 0.0, 0.0, 0.0
        desired_z = self._surface_normal(target_x, target_y, target_z)
        if self.align_orientation and desired_z is not None:
            current_z = self._quat_rotate(current_q, 0.0, 0.0, 1.0)
            axis = self._cross(current_z, desired_z)
            axis_mag = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if axis_mag > 1e-9:
                dot = max(-1.0, min(1.0, (
                    current_z[0] * desired_z[0] +
                    current_z[1] * desired_z[1] +
                    current_z[2] * desired_z[2])))
                angle = math.atan2(axis_mag, dot)
                torque_mag = min(
                    self.max_orientation_torque,
                    self.orientation_stiffness * angle)
                tx, ty, tz = (
                    torque_mag * axis[0] / axis_mag,
                    torque_mag * axis[1] / axis_mag,
                    torque_mag * axis[2] / axis_mag,
                )

        self._apply_world_wrench(fx, fy, fz, tx, ty, tz, dist, orientation_error)

    def _apply_world_wrench(self, fx: float, fy: float, fz: float,
                            tx: float, ty: float, tz: float,
                            correction: float, orientation_error: float):
        duration_nsec = int(max(0.02, self.period_s * 2.0) * 1e9)
        entity = f'{self.model_name}::{self.base_link_name}'
        req = (
            f'entity: {{name: "{entity}", type: LINK}}, '
            f'wrench: {{'
            f'force: {{x: {fx:.3f}, y: {fy:.3f}, z: {fz:.3f}}}, '
            f'torque: {{x: {tx:.3f}, y: {ty:.3f}, z: {tz:.3f}}}'
            f'}}, '
            f'duration: {{sec: 0, nsec: {duration_nsec}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/apply_link_wrench',
            '--reqtype', 'gz.msgs.EntityWrench',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.service_timeout_ms),
            '--req', req,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self.process_timeout)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(
                f'attachment guard wrench failed: {exc}',
                throttle_duration_sec=2.0)
            return

        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            self.get_logger().warn(
                f'attachment guard wrench rejected: {detail[:160]}',
                throttle_duration_sec=2.0)
            return

        now = time.time()
        if now - self._last_log_t > 2.0:
            self._last_log_t = now
            force_mag = math.sqrt(fx * fx + fy * fy + fz * fz)
            torque_mag = math.sqrt(tx * tx + ty * ty + tz * tz)
            self.get_logger().info(
                'attachment guard wrench '
                f'force={force_mag:.1f} N, torque={torque_mag:.1f} Nm, '
                f'radial drift={correction:.3f} m, '
                f'orientation={math.degrees(orientation_error):.1f} deg')

    def _set_model_pose(self, x: float, y: float, z: float,
                        qx: float, qy: float, qz: float, qw: float,
                        correction: float, orientation_error: float):
        req = (
            f'name: "{self.model_name}", '
            f'position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
            f'orientation: {{x: {qx:.8f}, y: {qy:.8f}, '
            f'z: {qz:.8f}, w: {qw:.8f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.service_timeout_ms),
            '--req', req,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self.process_timeout)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(
                f'attachment guard set_pose failed: {exc}',
                throttle_duration_sec=2.0)
            return

        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            self.get_logger().warn(
                f'attachment guard set_pose rejected: {detail[:160]}',
                throttle_duration_sec=2.0)
            return

        now = time.time()
        if now - self._last_log_t > 2.0:
            self._last_log_t = now
            self.get_logger().info(
                'attachment guard corrected '
                f'radial drift {correction:.3f} m, '
                f'orientation {math.degrees(orientation_error):.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceAttachmentGuard()
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
