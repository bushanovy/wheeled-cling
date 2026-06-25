#!/usr/bin/env python3
"""
magnetic_adhesion.py — Per-wheel KMW100 magnetic adhesion for the 3D3S robot.

The default climb_test world is a flat vertical steel panel. The current launch
places the robot on the panel's -X side, so plane mode uses normal=(-1, 0).
The curved-pipe mockup can still be tested by re-enabling it in my_world.sdf and
running this node in cylinder mode with the pipe center passed through
axis_x / axis_y. A slide mode is also available for ground -> 45 degree slide
-> wall transition tests.

For each magnetic wheel (wheel_1_link, wheel_2_link, wheel_3_link) we look up
its world XY position via /tf, then apply a radial force toward the pipe axis.
In plane mode, it applies a fixed force opposite to the supplied panel normal.

Physics
-------
KMW100 spec: 900 N on a flat steel plate (zero air gap).
The lookup model follows the current literature-backed assumption for this
robot: Q235 wall thickness saturates around 5 mm, so 6-10 mm pipe walls are
near full force. Air gap and curvature dominate the force loss.

The installed seed CSV is provisional. Replace it with FEMM / Ansys Maxwell
output for the exact KMW100 magnetic circuit when those results are available.

How the force is applied
------------------------
gz-transport Python bindings are not installed system-wide on most ROS 2
Humble/Jazzy + Gazebo Sim combos, so we shell out to
`gz service apply_link_wrench`. The three wheel calls are launched in parallel
so one slow CLI request does not block the other wheels for the whole cycle.

Parameters (ros2 param)
-----------------------
  ~mode           : "cylinder" (default), "plane", or "slide"
  ~adhesion_model : "constant" (default) or "lookup"
  ~force_n        : constant/fallback adhesion force per wheel (N), default 300
  ~min_force_n    : minimum force after lookup, default 0
  ~gap_bias_mm    : added to estimated gap before lookup, default 0
  ~max_lookup_gap_mm
                  : optional cap for lookup gap, negative disables
  ~force_table_csv: lookup table path. Legacy tables use
                  gap_mm, thickness_mm, force_n columns. COMSOL mockup tables
                  add wall_thickness_mm, pipe_radius_m, contact_fraction.
  ~wall_thickness_mm
                  : target steel wall thickness for lookup, default 10
  ~wheel_radius_m : wheel collision/contact radius, default 0.05
  ~pipe_radius_m  : target pipe radius for COMSOL lookup, default 0.55
  ~full_contact_depth_m
                  : contact depth treated as full wheel contact, default 0.002
  ~full_contact_points
                  : contact point count treated as full contact, default 4
  ~min_contact_fraction
                  : minimum fraction for a fresh magnetic contact, default 0.10
  ~cylinder_axis  : cylinder mode — pipe axis, x/y/z,       default z
  ~axis_x/y/z     : cylinder mode — point on pipe axis,     default (0, 0, 0)
  ~surface_radius_m
                  : cylinder mode — pipe outer radius,      default 0
  ~plane_point_x/y/z
                  : plane mode — point on the surface,      default (0, 0, 0)
  ~normal_x       : plane mode — surface outward normal X,  default 1.0
  ~normal_y       : plane mode — surface outward normal Y,  default 0.0
  ~normal_z       : plane mode — surface outward normal Z,  default 0.0
                    (force pulls each wheel in -normal direction)
  ~ground_z       : slide mode — ground steel plate height, default 0.0
  ~slide_angle_deg: slide mode — slide angle from ground,   default 45.0
  ~slide_start_x/z: slide mode — lower point on slide,      default (0.1, 0.0)
  ~slide_wall_x   : slide mode — vertical wall face X,      default 0.95
  ~surface_margin_m
                  : slide mode — surface transition margin, default 0.08
  ~surface_hysteresis_m
                  : slide mode — keep previous surface inside this tolerance,
                    default 0.03
  ~fallback_normal_x/y
                  : cylinder mode fallback outward normal when TF is not ready,
                    default (1, 0), useful for the +X side of the big pipe
  ~world_name     : Gazebo world name,            default "default"
  ~robot_name     : Gazebo model name,            default "robot_3d3s"
  ~rate_hz        : force-update frequency,       default 10
  ~startup_delay_s: delay before first force cycle, default 0
  ~tf_timeout_s   : TF lookup timeout per wheel,   default 0.02
  ~gazebo_pose_topic
                  : live Gazebo world pose topic,  default /world/default/pose/info
  ~gazebo_pose_timeout_s
                  : max age for live Gazebo pose,  default 2.0
  ~contact_timeout_s
                  : max age for wheel contact data, default 0.25
  ~contact_only   : only apply adhesion while a fresh magnetic contact exists,
                    default false
  ~scale_constant_by_contact_fraction
                  : in constant mode, multiply force_n by estimated contact
                    fraction, default true. Set false to model a KMW100 wheel
                    as full rated force whenever it is magnetically captured.
  ~magnetic_surface_allowlist
                  : optional comma-separated collision-name tokens. When set,
                    only matching non-robot contacts can generate adhesion.
  ~analytic_capture_gap_mm
                  : in contact_only cylinder mode, allow analytical magnetic
                    capture inside this air gap when contact is briefly absent,
                    default 0 disables
  ~allow_tf_analytic_capture
                  : allow TF-derived wheel poses for analytic cylinder capture
                    when live Gazebo link poses are momentarily unavailable,
                    default false
  ~boundary_low_contact_fraction
                  : contact fraction below this is treated as edge/partial
                    contact risk, default 0.35
  ~boundary_high_risk_gap_mm
                  : air gap where a wheel is considered near loss of surface,
                    default follows analytic_capture_gap_mm or 30 mm
  ~service_timeout_ms
                  : gz service transport timeout,  default 200
  ~process_timeout_s
                  : subprocess timeout,            default 0.3
  ~use_persistent_wrench
                  : in plane mode, publish one persistent Gazebo wrench per
                    wheel instead of repeatedly calling the service, default false
  ~diagnostic_period_s
                  : status log period, default 1.0 s

Usage
-----
Launched automatically by climb_test.launch.py, or run manually:
  ros2 run robot_3d3s magnetic_adhesion.py
  ros2 run robot_3d3s magnetic_adhesion.py --ros-args -p force_n:=400.0
"""

import concurrent.futures
import csv
import json
import math
import subprocess
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)


WHEEL_LINKS = ['wheel_1_link', 'wheel_2_link', 'wheel_3_link']


@dataclass
class ContactState:
    stamp_s: float
    surface: str
    collision1: str
    collision2: str
    position: tuple | None
    normal: tuple | None
    depth_m: float
    contact_points: int
    contact_fraction: float


def _normalize_xy(x: float, y: float):
    mag = math.hypot(x, y)
    if mag <= 1e-12:
        return 1.0, 0.0
    return x / mag, y / mag


def _normalize_xyz(x: float, y: float, z: float):
    mag = math.sqrt(x * x + y * y + z * z)
    if mag <= 1e-12:
        return 1.0, 0.0, 0.0
    return x / mag, y / mag, z / mag


def _bounds(values, query: float):
    if not values:
        raise ValueError('lookup table axis is empty')
    if query <= values[0]:
        return values[0], values[0], query < values[0]
    if query >= values[-1]:
        return values[-1], values[-1], query > values[-1]
    for low, high in zip(values, values[1:]):
        if low <= query <= high:
            return low, high, False
    return values[-1], values[-1], True


def _lerp(a: float, b: float, ratio: float):
    return a + (b - a) * ratio


def estimate_contact_fraction(
        depth_m: float, contact_points: int, full_contact_depth_m: float,
        full_contact_points: int, min_contact_fraction: float):
    """Estimate how much of the wheel is magnetically engaged."""
    min_fraction = max(0.0, min(1.0, min_contact_fraction))
    if depth_m <= 0.0 and contact_points <= 0:
        return 0.0

    depth_fraction = 0.0
    if full_contact_depth_m > 0.0:
        depth_fraction = max(0.0, min(1.0, depth_m / full_contact_depth_m))

    point_fraction = 0.0
    if full_contact_points > 0:
        point_fraction = max(0.0, min(1.0, contact_points / full_contact_points))

    fraction = 0.5 * depth_fraction + 0.5 * point_fraction
    return max(min_fraction, min(1.0, fraction))


def plane_gap_m(
        wheel_x: float, wheel_y: float, plane_point_x: float, plane_point_y: float,
        normal_x: float, normal_y: float, wheel_radius_m: float):
    """Return non-negative wheel-to-plane air gap in meters."""
    nx, ny = _normalize_xy(normal_x, normal_y)
    signed_center_gap = (
        (wheel_x - plane_point_x) * nx + (wheel_y - plane_point_y) * ny)
    return max(0.0, signed_center_gap - wheel_radius_m)


def plane_gap_3d_m(
        wheel_x: float, wheel_y: float, wheel_z: float,
        plane_point_x: float, plane_point_y: float, plane_point_z: float,
        normal_x: float, normal_y: float, normal_z: float,
        wheel_radius_m: float):
    """Return non-negative wheel-to-plane air gap in meters."""
    nx, ny, nz = _normalize_xyz(normal_x, normal_y, normal_z)
    signed_center_gap = (
        (wheel_x - plane_point_x) * nx +
        (wheel_y - plane_point_y) * ny +
        (wheel_z - plane_point_z) * nz)
    return max(0.0, signed_center_gap - wheel_radius_m)


def signed_plane_distance_m(
        wheel_x: float, wheel_y: float, wheel_z: float,
        plane_point_x: float, plane_point_y: float, plane_point_z: float,
        normal_x: float, normal_y: float, normal_z: float):
    nx, ny, nz = _normalize_xyz(normal_x, normal_y, normal_z)
    return (
        (wheel_x - plane_point_x) * nx +
        (wheel_y - plane_point_y) * ny +
        (wheel_z - plane_point_z) * nz)


def cylinder_gap_m(
        wheel_x: float, wheel_y: float, axis_x: float, axis_y: float,
        surface_radius_m: float, wheel_radius_m: float):
    """Return non-negative wheel-to-cylinder air gap in meters."""
    dist = math.hypot(wheel_x - axis_x, wheel_y - axis_y)
    return max(0.0, dist - surface_radius_m - wheel_radius_m)


def cylinder_axis_direction_and_gap_m(
        wheel_x: float, wheel_y: float, wheel_z: float,
        axis: str, axis_x: float, axis_y: float, axis_z: float,
        surface_radius_m: float, wheel_radius_m: float):
    """Return force direction toward a cylinder axis and non-negative air gap."""
    axis = axis.lower()
    if axis == 'x':
        dx = 0.0
        dy = axis_y - wheel_y
        dz = axis_z - wheel_z
    elif axis == 'y':
        dx = axis_x - wheel_x
        dy = 0.0
        dz = axis_z - wheel_z
    else:
        dx = axis_x - wheel_x
        dy = axis_y - wheel_y
        dz = 0.0

    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-3:
        return None
    gap_m = max(0.0, dist - surface_radius_m - wheel_radius_m)
    return dx / dist, dy / dist, dz / dist, gap_m


def gap_decay_scale(gap_mm: float, reference_gap_mm: float):
    """HKCM-style inverse cubic air-gap attenuation."""
    if reference_gap_mm <= 0.0:
        return 1.0
    s = max(0.0, gap_mm) / reference_gap_mm
    return 1.0 / (1.0 + s * s * s)


def tilt_scale(tilt_deg: float, safe_tilt_deg: float, cutoff_tilt_deg: float):
    """Return a conservative adhesion multiplier for wheel/surface tilt."""
    if safe_tilt_deg <= 0.0 or cutoff_tilt_deg <= safe_tilt_deg:
        return 1.0
    tilt = abs(tilt_deg)
    if tilt <= safe_tilt_deg:
        return 1.0
    if tilt >= cutoff_tilt_deg:
        return 0.0
    ratio = (tilt - safe_tilt_deg) / (cutoff_tilt_deg - safe_tilt_deg)
    return max(0.0, 1.0 - ratio)


class ForceLookupTable:
    """Multilinear lookup over KMW100 COMSOL force-table axes."""

    def __init__(
            self, gaps_mm, thicknesses_mm, pipe_radii_m, contact_fractions, forces,
            has_pipe_radius_axis=True, has_contact_fraction_axis=True):
        self.gaps_mm = sorted(gaps_mm)
        self.thicknesses_mm = sorted(thicknesses_mm)
        self.pipe_radii_m = sorted(pipe_radii_m)
        self.contact_fractions = sorted(contact_fractions)
        self._forces = forces
        self._has_pipe_radius_axis = has_pipe_radius_axis
        self._has_contact_fraction_axis = has_contact_fraction_axis

    @classmethod
    def from_csv(cls, path: str):
        forces = {}
        gaps = set()
        thicknesses = set()
        pipe_radii = set()
        contact_fractions = set()
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            fieldnames = set(reader.fieldnames or [])
            has_pipe_radius_axis = 'pipe_radius_m' in fieldnames
            has_contact_fraction_axis = 'contact_fraction' in fieldnames
            has_thickness = (
                'thickness_mm' in fieldnames or
                'wall_thickness_mm' in fieldnames)
            required = {'gap_mm', 'force_n'}
            if not required.issubset(fieldnames) or not has_thickness:
                raise ValueError(
                    f'{path} must contain columns: gap_mm, '
                    f'thickness_mm or wall_thickness_mm, force_n')
            for row in reader:
                gap = float(row['gap_mm'])
                thickness_key = (
                    'wall_thickness_mm'
                    if 'wall_thickness_mm' in row and row['wall_thickness_mm'] != ''
                    else 'thickness_mm')
                thickness = float(row[thickness_key])
                pipe_radius = float(row.get('pipe_radius_m') or 0.0)
                contact_fraction = float(row.get('contact_fraction') or 1.0)
                force = float(row['force_n'])
                gaps.add(gap)
                thicknesses.add(thickness)
                pipe_radii.add(pipe_radius)
                contact_fractions.add(contact_fraction)
                forces[(gap, thickness, pipe_radius, contact_fraction)] = force

        if not gaps or not thicknesses or not pipe_radii or not contact_fractions:
            raise ValueError(f'{path} contains no force samples')

        for gap in gaps:
            for thickness in thicknesses:
                for pipe_radius in pipe_radii:
                    for contact_fraction in contact_fractions:
                        key = (gap, thickness, pipe_radius, contact_fraction)
                        if key not in forces:
                            raise ValueError(
                                f'{path} is not rectangular; missing gap={gap}, '
                                f'thickness={thickness}, pipe_radius={pipe_radius}, '
                                f'contact_fraction={contact_fraction}')

        return cls(
            gaps, thicknesses, pipe_radii, contact_fractions, forces,
            has_pipe_radius_axis, has_contact_fraction_axis)

    def lookup(
            self, gap_mm: float, thickness_mm: float,
            pipe_radius_m: float = 0.0, contact_fraction: float = 1.0):
        g0, g1, gap_clamped = _bounds(self.gaps_mm, gap_mm)
        t0, t1, thickness_clamped = _bounds(self.thicknesses_mm, thickness_mm)
        r0, r1, radius_clamped = _bounds(self.pipe_radii_m, pipe_radius_m)
        c0, c1, contact_clamped = _bounds(
            self.contact_fractions, contact_fraction)

        axes = (
            (g0, g1, 0.0 if g0 == g1 else (gap_mm - g0) / (g1 - g0)),
            (t0, t1, 0.0 if t0 == t1 else (thickness_mm - t0) / (t1 - t0)),
            (r0, r1, 0.0 if r0 == r1 else (pipe_radius_m - r0) / (r1 - r0)),
            (c0, c1, 0.0 if c0 == c1 else (contact_fraction - c0) / (c1 - c0)),
        )

        total = 0.0
        for gi, gv in enumerate((g0, g1)):
            gw = (1.0 - axes[0][2]) if gi == 0 else axes[0][2]
            for ti, tv in enumerate((t0, t1)):
                tw = (1.0 - axes[1][2]) if ti == 0 else axes[1][2]
                for ri, rv in enumerate((r0, r1)):
                    rw = (1.0 - axes[2][2]) if ri == 0 else axes[2][2]
                    for ci, cv in enumerate((c0, c1)):
                        cw = (1.0 - axes[3][2]) if ci == 0 else axes[3][2]
                        total += self._forces[(gv, tv, rv, cv)] * gw * tw * rw * cw

        return total, {
            'gap': gap_clamped,
            'thickness': thickness_clamped,
            'pipe_radius': radius_clamped and self._has_pipe_radius_axis,
            'contact_fraction': contact_clamped and self._has_contact_fraction_axis,
        }

    @property
    def has_contact_fraction_axis(self):
        return self._has_contact_fraction_axis


class MagneticAdhesionNode(Node):

    def __init__(self):
        super().__init__('magnetic_adhesion')

        # Parameters
        self.declare_parameter('mode',       'cylinder')   # cylinder, plane, slide
        self.declare_parameter('adhesion_model', 'constant')
        self.declare_parameter('force_n',    300.0)
        self.declare_parameter('min_force_n', 0.0)
        self.declare_parameter('gap_reference_mm', 1.0)
        self.declare_parameter('safe_tilt_deg', 8.0)
        self.declare_parameter('cutoff_tilt_deg', 30.0)
        self.declare_parameter('gap_bias_mm', 0.0)
        self.declare_parameter('max_lookup_gap_mm', -1.0)
        self.declare_parameter('force_table_csv', '')
        self.declare_parameter('wall_thickness_mm', 10.0)
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('pipe_radius_m', 0.55)
        self.declare_parameter('full_contact_depth_m', 0.002)
        self.declare_parameter('full_contact_points', 4)
        self.declare_parameter('min_contact_fraction', 0.10)
        self.declare_parameter('axis_x',     0.0)
        self.declare_parameter('axis_y',     0.0)
        self.declare_parameter('axis_z',     0.0)
        self.declare_parameter('cylinder_axis', 'z')
        self.declare_parameter('surface_radius_m', 0.0)
        self.declare_parameter('plane_point_x', 0.0)
        self.declare_parameter('plane_point_y', 0.0)
        self.declare_parameter('plane_point_z', 0.0)
        self.declare_parameter('normal_x',   1.0)
        self.declare_parameter('normal_y',   0.0)
        self.declare_parameter('normal_z',   0.0)
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('slide_angle_deg', 45.0)
        self.declare_parameter('slide_start_x', 0.1)
        self.declare_parameter('slide_start_z', 0.0)
        self.declare_parameter('slide_wall_x', 0.95)
        self.declare_parameter('surface_margin_m', 0.08)
        self.declare_parameter('surface_hysteresis_m', 0.03)
        self.declare_parameter('fallback_normal_x', 1.0)
        self.declare_parameter('fallback_normal_y', 0.0)
        self.declare_parameter('fallback_normal_z', 0.0)
        self.declare_parameter('world_name', 'default')
        self.declare_parameter('robot_name', 'robot_3d3s')
        self.declare_parameter('rate_hz',    10.0)
        self.declare_parameter('startup_delay_s', 0.0)
        self.declare_parameter('tf_timeout_s', 0.02)
        self.declare_parameter('gazebo_pose_topic', '/world/default/pose/info')
        self.declare_parameter('gazebo_pose_timeout_s', 2.0)
        self.declare_parameter('contact_timeout_s', 0.25)
        self.declare_parameter('contact_only', False)
        self.declare_parameter('scale_constant_by_contact_fraction', True)
        self.declare_parameter('magnetic_surface_allowlist', '')
        self.declare_parameter('analytic_capture_gap_mm', 0.0)
        self.declare_parameter('allow_tf_analytic_capture', False)
        self.declare_parameter('boundary_low_contact_fraction', 0.35)
        self.declare_parameter('boundary_high_risk_gap_mm', 0.0)
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('status_period_s', 0.1)
        self.declare_parameter('service_timeout_ms', 200)
        self.declare_parameter('process_timeout_s', 0.3)
        self.declare_parameter('use_persistent_wrench', False)
        self.declare_parameter('diagnostic_period_s', 1.0)

        self._mode       = str(self.get_parameter('mode').value).lower()
        self._adhesion_model = str(
            self.get_parameter('adhesion_model').value).lower()
        self._force_n    = float(self.get_parameter('force_n').value)
        self._min_force_n = max(0.0, float(self.get_parameter('min_force_n').value))
        self._gap_reference_mm = max(
            1e-6, float(self.get_parameter('gap_reference_mm').value))
        self._safe_tilt_deg = max(
            0.0, float(self.get_parameter('safe_tilt_deg').value))
        self._cutoff_tilt_deg = max(
            0.0, float(self.get_parameter('cutoff_tilt_deg').value))
        self._gap_bias_mm = float(self.get_parameter('gap_bias_mm').value)
        self._max_lookup_gap_mm = float(
            self.get_parameter('max_lookup_gap_mm').value)
        self._force_table_csv = str(self.get_parameter('force_table_csv').value)
        self._wall_thickness_mm = float(
            self.get_parameter('wall_thickness_mm').value)
        self._wheel_radius_m = max(
            0.0, float(self.get_parameter('wheel_radius_m').value))
        self._pipe_radius_m = max(
            0.0, float(self.get_parameter('pipe_radius_m').value))
        self._full_contact_depth_m = max(
            0.0, float(self.get_parameter('full_contact_depth_m').value))
        self._full_contact_points = max(
            0, int(self.get_parameter('full_contact_points').value))
        self._min_contact_fraction = max(
            0.0, min(1.0, float(self.get_parameter('min_contact_fraction').value)))
        self._axis_x     = float(self.get_parameter('axis_x').value)
        self._axis_y     = float(self.get_parameter('axis_y').value)
        self._axis_z     = float(self.get_parameter('axis_z').value)
        self._cylinder_axis = str(
            self.get_parameter('cylinder_axis').value).lower()
        if self._cylinder_axis not in ('x', 'y', 'z'):
            self.get_logger().warn(
                f'Unsupported cylinder_axis={self._cylinder_axis}; using z.')
            self._cylinder_axis = 'z'
        self._surface_radius_m = max(
            0.0, float(self.get_parameter('surface_radius_m').value))
        self._plane_point_x = float(self.get_parameter('plane_point_x').value)
        self._plane_point_y = float(self.get_parameter('plane_point_y').value)
        self._plane_point_z = float(self.get_parameter('plane_point_z').value)
        nx               = float(self.get_parameter('normal_x').value)
        ny               = float(self.get_parameter('normal_y').value)
        nz               = float(self.get_parameter('normal_z').value)
        self._normal_x, self._normal_y, self._normal_z = _normalize_xyz(nx, ny, nz)
        self._ground_z = float(self.get_parameter('ground_z').value)
        self._slide_angle_rad = math.radians(
            float(self.get_parameter('slide_angle_deg').value))
        self._slide_start_x = float(self.get_parameter('slide_start_x').value)
        self._slide_start_z = float(self.get_parameter('slide_start_z').value)
        self._slide_wall_x = float(self.get_parameter('slide_wall_x').value)
        self._surface_margin_m = max(
            0.0, float(self.get_parameter('surface_margin_m').value))
        self._surface_hysteresis_m = max(
            0.0, float(self.get_parameter('surface_hysteresis_m').value))
        fnx              = float(self.get_parameter('fallback_normal_x').value)
        fny              = float(self.get_parameter('fallback_normal_y').value)
        fnz              = float(self.get_parameter('fallback_normal_z').value)
        self._world_name = self.get_parameter('world_name').value
        self._robot_name = self.get_parameter('robot_name').value
        self._rate_hz    = float(self.get_parameter('rate_hz').value)
        self._period_s   = 1.0 / self._rate_hz
        self._startup_delay_s = max(0.0, float(self.get_parameter('startup_delay_s').value))
        self._tf_timeout_s = max(0.0, float(self.get_parameter('tf_timeout_s').value))
        self._gazebo_pose_topic = str(self.get_parameter('gazebo_pose_topic').value)
        self._gazebo_pose_timeout_s = max(
            0.0, float(self.get_parameter('gazebo_pose_timeout_s').value))
        self._contact_timeout_s = max(
            0.0, float(self.get_parameter('contact_timeout_s').value))
        self._contact_only = bool(self.get_parameter('contact_only').value)
        self._scale_constant_by_contact_fraction = bool(
            self.get_parameter('scale_constant_by_contact_fraction').value)
        allowlist_raw = str(
            self.get_parameter('magnetic_surface_allowlist').value)
        self._magnetic_surface_allowlist = tuple(
            token.strip().lower()
            for token in allowlist_raw.split(',')
            if token.strip())
        self._analytic_capture_gap_mm = max(
            0.0, float(self.get_parameter('analytic_capture_gap_mm').value))
        self._allow_tf_analytic_capture = bool(
            self.get_parameter('allow_tf_analytic_capture').value)
        self._boundary_low_contact_fraction = max(0.0, min(
            1.0, float(self.get_parameter('boundary_low_contact_fraction').value)))
        boundary_gap = float(self.get_parameter('boundary_high_risk_gap_mm').value)
        self._boundary_high_risk_gap_mm = (
            boundary_gap if boundary_gap > 0.0
            else max(30.0, self._analytic_capture_gap_mm))
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._status_period_s = max(
            0.0, float(self.get_parameter('status_period_s').value))
        self._next_status_time = time.time() + self._status_period_s
        self._service_timeout_ms = int(self.get_parameter('service_timeout_ms').value)
        self._process_timeout_s = max(0.05, float(self.get_parameter('process_timeout_s').value))
        self._use_persistent_wrench = bool(self.get_parameter('use_persistent_wrench').value)
        self._diagnostic_period_s = max(
            0.0, float(self.get_parameter('diagnostic_period_s').value))
        self._next_diagnostic_time = time.time() + self._diagnostic_period_s
        self._sent_count = 0
        self._ok_count = 0
        self._fail_count = 0
        self._last_forces = {}
        self._last_gaps_mm = {}
        self._last_surfaces = {}
        self._last_attached = {wheel: False for wheel in WHEEL_LINKS}
        self._last_contact_fractions = {wheel: 0.0 for wheel in WHEEL_LINKS}
        self._last_normals = {wheel: (0.0, 0.0, 0.0) for wheel in WHEEL_LINKS}
        self._last_pose_sources = {}
        self._last_depths_m = {wheel: 0.0 for wheel in WHEEL_LINKS}
        self._last_contact_points = {wheel: 0 for wheel in WHEEL_LINKS}
        self._surface_latch = {}
        self._wheel_contacts = {}
        self._contact_lock = threading.Lock()
        self._last_error = ''
        self._persistent_active = False
        self._clamp_warnings = set()

        # Pre-compute plane-mode force vector (constant, opposite to normal)
        self._plane_fx = -self._force_n * self._normal_x
        self._plane_fy = -self._force_n * self._normal_y
        self._plane_fz = -self._force_n * self._normal_z

        fnmag = math.sqrt(fnx * fnx + fny * fny + fnz * fnz)
        self._has_fallback = fnmag > 1e-9
        if self._has_fallback:
            self._fallback_fx = -self._force_n * fnx / fnmag
            self._fallback_fy = -self._force_n * fny / fnmag
            self._fallback_fz = -self._force_n * fnz / fnmag
        else:
            self._fallback_fx = 0.0
            self._fallback_fy = 0.0
            self._fallback_fz = 0.0

        self._force_table = None
        if self._adhesion_model == 'lookup':
            if not self._force_table_csv:
                self.get_logger().warn(
                    'adhesion_model=lookup requested without force_table_csv; '
                    'falling back to constant force.')
                self._adhesion_model = 'constant'
            else:
                try:
                    self._force_table = ForceLookupTable.from_csv(
                        self._force_table_csv)
                except Exception as e:                       # noqa: BLE001
                    self.get_logger().warn(
                        f'failed to load force_table_csv={self._force_table_csv}: '
                        f'{e}; falling back to constant force.')
                    self._adhesion_model = 'constant'
        elif self._adhesion_model not in ('constant', 'gap_decay'):
            self.get_logger().warn(
                f'unknown adhesion_model={self._adhesion_model}; '
                'falling back to constant force.')
            self._adhesion_model = 'constant'

        # TF buffer for wheel world poses
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._gazebo_poses = {}
        pose_topics = [
            self._gazebo_pose_topic,
            f'/model/{self._robot_name}/pose',
        ]
        self._pose_subs = []
        for topic in dict.fromkeys(pose_topics):
            self._pose_subs.append(self.create_subscription(
                TFMessage,
                topic,
                self._gazebo_pose_callback,
                10,
            ))
        self._contact_subs = []
        self._contact_msg_counts = {wheel: 0 for wheel in WHEEL_LINKS}
        self._contact_seen_counts = {wheel: 0 for wheel in WHEEL_LINKS}
        for wheel in WHEEL_LINKS:
            topic = f'/robot_3d3s/{wheel.replace("_link", "")}_contacts'
            self._contact_subs.append(self.create_subscription(
                Contacts,
                topic,
                lambda msg, wheel_link=wheel: self._contact_callback(wheel_link, msg),
                10,
            ))
        self._wrench_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=len(WHEEL_LINKS))
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._running = True

        if self._mode == 'plane':
            self.get_logger().info(
                f'Magnetic adhesion (PLANE mode, {self._adhesion_model}): '
                f'{self._force_n:.0f} N fallback per wheel '
                f'in direction ({self._plane_fx:+.1f}, {self._plane_fy:+.1f}, '
                f'{self._plane_fz:+.1f}) N, '
                f'rate {self._rate_hz:.1f} Hz')
            if self._use_persistent_wrench and self._adhesion_model == 'constant':
                self.get_logger().info(
                    'Using Gazebo persistent wrench topic for flat-wall adhesion.')
            elif self._use_persistent_wrench:
                self.get_logger().warn(
                    'Persistent wrench mode requires constant adhesion; '
                    'lookup mode will use repeated wrench updates.')
        elif self._mode == 'slide':
            self.get_logger().info(
                f'Magnetic adhesion (SLIDE mode, {self._adhesion_model}): '
                f'{math.degrees(self._slide_angle_rad):.1f} deg slide, '
                f'ground_z={self._ground_z:.2f}, wall_x={self._slide_wall_x:.2f}, '
                f'{self._force_n:.0f} N fallback per wheel, '
                f'rate {self._rate_hz:.1f} Hz')
        else:
            self.get_logger().info(
                f'Magnetic adhesion (CYLINDER mode, {self._adhesion_model}): '
                f'{self._force_n:.0f} N fallback per wheel, '
                f'axis at world ({self._axis_x:.2f}, {self._axis_y:.2f}), '
                f'rate {self._rate_hz:.1f} Hz')
            if self._has_fallback:
                self.get_logger().info(
                    'Cylinder fallback force while TF is unavailable: '
                    f'({self._fallback_fx:+.1f}, {self._fallback_fy:+.1f}) N')
        if self._adhesion_model == 'lookup':
            self.get_logger().info(
                'KMW100 lookup active: 900 N at zero gap on saturated steel; '
                '6-10 mm walls are treated near saturation. '
                f'table={self._force_table_csv}, '
                f'min_force={self._min_force_n:.0f} N, '
                f'gap_bias={self._gap_bias_mm:.1f} mm, '
                f'max_lookup_gap={self._max_lookup_gap_mm:.1f} mm, '
                f'pipe_radius={self._pipe_radius_m:.3f} m')
        elif self._adhesion_model == 'gap_decay':
            self.get_logger().info(
                'KMW100 gap-decay model active: '
                f'F={self._force_n:.0f}/(1+(gap_mm/'
                f'{self._gap_reference_mm:.2f})^3), '
                f'tilt safe/cutoff={self._safe_tilt_deg:.1f}/'
                f'{self._cutoff_tilt_deg:.1f} deg.')
        else:
            self.get_logger().info(
                'KMW100 constant-force mode active. Lookup mode is available '
                'for air-gap and wall-thickness sensitivity.')
            if not self._scale_constant_by_contact_fraction:
                self.get_logger().info(
                    'Constant-force mode is using full force_n whenever a '
                    'wheel is attached/captured; contact fraction is reported '
                    'but does not reduce magnetic force.')
        self.get_logger().info(
            f'Live Gazebo pose sources: {", ".join(dict.fromkeys(pose_topics))} '
            f'(timeout {self._gazebo_pose_timeout_s:.2f}s); TF is fallback.')
        self.get_logger().info(
            'Wheel contact topics active: /robot_3d3s/wheel_1_contacts, '
            '/robot_3d3s/wheel_2_contacts, /robot_3d3s/wheel_3_contacts '
            f'(timeout {self._contact_timeout_s:.2f}s).')
        if self._contact_only:
            capture_note = ''
            if self._analytic_capture_gap_mm > 0.0:
                source = 'Gazebo/TF' if self._allow_tf_analytic_capture else 'Gazebo'
                capture_note = (
                    f' Near-surface cylinder capture is enabled inside '
                    f'{self._analytic_capture_gap_mm:.1f} mm using {source} poses.')
            self.get_logger().info(
                'Contact-only adhesion active: forces are applied only from '
                'fresh magnetic wheel contacts or configured capture gaps.'
                f'{capture_note}')
        if self._magnetic_surface_allowlist:
            self.get_logger().info(
                'Magnetic contact allowlist active: '
                f'{", ".join(self._magnetic_surface_allowlist)}')
        self.get_logger().info(
            f'Adhesion status publishing on {self._status_topic} '
            f'at {self._status_period_s:.2f} s period.')

        self._adhesion_thread = threading.Thread(target=self._adhesion_loop)
        self._adhesion_thread.start()

    def destroy_node(self):
        self._running = False
        if self._persistent_active:
            self._clear_persistent_wrenches()
        if getattr(self, '_adhesion_thread', None) is not None:
            self._adhesion_thread.join(
                timeout=max(1.0, self._process_timeout_s + 0.5))
        self._wrench_executor.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()

    # ── Main adhesion loop ─────────────────────────────────────────────────────

    def _adhesion_loop(self):
        if self._startup_delay_s > 0.0:
            time.sleep(self._startup_delay_s)
        self.get_logger().info('Applying per-wheel adhesion forces.')

        if (self._mode == 'plane' and self._use_persistent_wrench and
                self._adhesion_model == 'constant'):
            self._publish_persistent_wrenches()
            while rclpy.ok() and self._running:
                self._publish_diagnostic()
                time.sleep(0.1)
            return

        while rclpy.ok() and self._running:
            t0 = time.time()
            futures = []
            for wheel in WHEEL_LINKS:
                try:
                    force = self._force_for_wheel(wheel)
                except Exception as e:                       # noqa: BLE001
                    self.get_logger().warn(
                        f'{wheel}: {e}', throttle_duration_sec=5.0)
                    continue

                if force is None:
                    self._last_attached[wheel] = False
                    self._last_forces[wheel] = (0.0, 0.0, 0.0)
                    self._last_gaps_mm[wheel] = None
                    self._last_surfaces[wheel] = 'none'
                    self._last_contact_fractions[wheel] = 0.0
                    self._last_normals[wheel] = (0.0, 0.0, 0.0)
                    self._last_depths_m[wheel] = 0.0
                    self._last_contact_points[wheel] = 0
                    continue

                fx, fy, fz, gap_mm, force_n, surface, contact_fraction, normal = force
                contact = self._latest_contact(wheel)
                self._last_forces[wheel] = (fx, fy, fz)
                self._last_gaps_mm[wheel] = gap_mm
                self._last_surfaces[wheel] = surface
                self._last_attached[wheel] = force_n > 0.0
                self._last_contact_fractions[wheel] = contact_fraction
                self._last_normals[wheel] = normal
                self._last_depths_m[wheel] = contact.depth_m if contact else 0.0
                self._last_contact_points[wheel] = contact.contact_points if contact else 0
                futures.append(self._wrench_executor.submit(
                    self._apply_world_wrench, wheel, fx, fy, fz))

            done, not_done = concurrent.futures.wait(
                futures, timeout=self._process_timeout_s + 0.05)
            for future in done:
                try:
                    future.result()
                except Exception as e:                       # noqa: BLE001
                    self._fail_count += 1
                    self._last_error = str(e)
                    self.get_logger().warn(
                        f'adhesion worker failed: {e}', throttle_duration_sec=5.0)
            if not_done:
                self._fail_count += len(not_done)
                self._last_error = 'adhesion service call still running'
                self.get_logger().warn(
                    f'{len(not_done)} adhesion service call(s) still running',
                    throttle_duration_sec=5.0)

            if not (rclpy.ok() and self._running):
                break
            self._publish_diagnostic()
            self._publish_status()
            elapsed = time.time() - t0
            time.sleep(max(0.0, self._period_s - elapsed))

    def _force_for_wheel(self, wheel_link: str):
        """Compute the world-frame adhesion force for one wheel.

        In CYLINDER mode the force points radially toward (axis_x, axis_y).
        In PLANE mode the force points opposite to the panel normal. If TF is
        not available yet, use the configured fallback force so adhesion starts
        before gravity pulls the robot away.
        """
        if self._adhesion_model == 'constant' and self._mode == 'plane':
            normal = (-self._normal_x, -self._normal_y, -self._normal_z)
            return (
                self._plane_fx, self._plane_fy, self._plane_fz,
                None, self._force_n, 'plane', 1.0, normal)

        contact = self._latest_contact(wheel_link)
        pose = self._wheel_world_pose(wheel_link)
        if contact is not None:
            contact_fraction = contact.contact_fraction
            if (
                    self._mode == 'cylinder' and
                    contact.surface in ('pipe', 'cylinder') and
                    pose is not None):
                wx, wy, wz, _pose_source = pose
                dir_x, dir_y, dir_z = self._cylinder_inward_direction(wx, wy, wz)
                force_n = self._force_for_gap(0.0, contact_fraction)
                self._last_pose_sources[wheel_link] = 'cylinder-contact'
                return (
                    force_n * dir_x, force_n * dir_y, force_n * dir_z,
                    0.0, force_n, f'contact:{contact.surface}',
                    contact_fraction, (dir_x, dir_y, dir_z))
            if pose is not None and contact.position is not None:
                wx, wy, wz, _pose_source = pose
                px, py, pz = contact.position
                dir_x, dir_y, dir_z = _normalize_xyz(px - wx, py - wy, pz - wz)
                force_n = self._force_for_gap(0.0, contact_fraction)
                self._last_pose_sources[wheel_link] = 'contact'
                return (
                    force_n * dir_x, force_n * dir_y, force_n * dir_z,
                    0.0, force_n, f'contact:{contact.surface}',
                    contact_fraction, (dir_x, dir_y, dir_z))
            if contact.normal is not None:
                dir_x, dir_y, dir_z = self._contact_normal_direction(
                    wheel_link, contact)
                force_n = self._force_for_gap(0.0, contact_fraction)
                self._last_pose_sources[wheel_link] = 'contact-normal'
                return (
                    force_n * dir_x, force_n * dir_y, force_n * dir_z,
                    0.0, force_n, f'contact:{contact.surface}',
                    contact_fraction, (dir_x, dir_y, dir_z))

        if self._contact_only:
            analytic = self._contact_only_analytic_capture(wheel_link, pose)
            if analytic is not None:
                return analytic
            self._last_pose_sources[wheel_link] = 'no-contact'
            self._last_surfaces[wheel_link] = 'none'
            return None

        # Lookup, slide, and cylinder modes use wheel world pose when no fresh
        # Gazebo contact is available.
        if pose is None:
            if self._has_fallback:
                self.get_logger().warn(
                    f'{wheel_link}: live Gazebo pose and TF unavailable; '
                    'using fallback adhesion direction',
                    throttle_duration_sec=5.0)
                self._last_pose_sources[wheel_link] = 'fallback'
                return (
                    self._fallback_fx, self._fallback_fy, self._fallback_fz,
                    None, max(self._force_n, self._min_force_n), 'fallback',
                    1.0, _normalize_xyz(
                        self._fallback_fx, self._fallback_fy, self._fallback_fz))
            return None

        wx, wy, wz, pose_source = pose
        self._last_pose_sources[wheel_link] = pose_source

        if self._mode == 'plane':
            dir_x = -self._normal_x
            dir_y = -self._normal_y
            dir_z = -self._normal_z
            gap_m = plane_gap_3d_m(
                wx, wy, wz,
                self._plane_point_x, self._plane_point_y, self._plane_point_z,
                self._normal_x, self._normal_y, self._normal_z,
                self._wheel_radius_m)
            surface = 'plane'
        elif self._mode == 'slide':
            normal, gap_m, surface = self._slide_surface_for_wheel(
                wheel_link, wx, wy, wz)
            dir_x = -normal[0]
            dir_y = -normal[1]
            dir_z = -normal[2]
        else:
            cylinder = cylinder_axis_direction_and_gap_m(
                wx, wy, wz,
                self._cylinder_axis,
                self._axis_x, self._axis_y, self._axis_z,
                self._surface_radius_m, self._wheel_radius_m)
            if cylinder is None:
                return None
            dir_x, dir_y, dir_z, gap_m = cylinder
            surface = 'cylinder'

        gap_mm = gap_m * 1000.0
        if self._mode == 'slide' and pose_source == 'tf' and gap_mm > 100.0:
            self.get_logger().warn(
                f'{wheel_link}: slide adhesion is using TF fallback with '
                f'gap={gap_mm:.1f} mm. This is usually the static robot_state '
                'pose, not the live Gazebo pose; check /model/robot_3d3s/pose '
                'bridge or contact topics.',
                throttle_duration_sec=5.0)
        force_n = self._force_for_gap(gap_mm)
        return (
            force_n * dir_x, force_n * dir_y, force_n * dir_z,
            gap_mm, force_n, surface, 1.0, (dir_x, dir_y, dir_z))

    def _cylinder_inward_direction(self, wx: float, wy: float, wz: float):
        """Return the world direction from a wheel center toward cylinder axis."""
        if self._cylinder_axis == 'x':
            return _normalize_xyz(0.0, self._axis_y - wy, self._axis_z - wz)
        if self._cylinder_axis == 'y':
            return _normalize_xyz(self._axis_x - wx, 0.0, self._axis_z - wz)
        return _normalize_xyz(self._axis_x - wx, self._axis_y - wy, 0.0)

    def _contact_only_analytic_capture(self, wheel_link: str, pose):
        if (pose is None or self._mode != 'cylinder' or
                self._analytic_capture_gap_mm <= 0.0):
            return None

        wx, wy, wz, pose_source = pose
        if pose_source != 'gz' and not self._allow_tf_analytic_capture:
            self._last_pose_sources[wheel_link] = f'capture-rejected:{pose_source}'
            return None

        cylinder = cylinder_axis_direction_and_gap_m(
            wx, wy, wz,
            self._cylinder_axis,
            self._axis_x, self._axis_y, self._axis_z,
            self._surface_radius_m, self._wheel_radius_m)
        if cylinder is None:
            return None

        dir_x, dir_y, dir_z, gap_m = cylinder
        gap_mm = gap_m * 1000.0
        if gap_mm > self._analytic_capture_gap_mm:
            return None

        # Permanent magnets do not switch on only after a collision event.  Use
        # a smooth capture fraction so the force grows as the wheel closes the
        # small air gap, while still leaving real contact as the strongest case.
        contact_fraction = max(
            self._min_contact_fraction,
            1.0 - gap_mm / max(1e-6, self._analytic_capture_gap_mm))
        force_n = self._force_for_gap(gap_mm, contact_fraction)
        self._last_pose_sources[wheel_link] = f'capture:{pose_source}'
        return (
            force_n * dir_x, force_n * dir_y, force_n * dir_z,
            gap_mm, force_n, 'cylinder', contact_fraction,
            (dir_x, dir_y, dir_z))

    def _wheel_world_pose(self, wheel_link: str):
        live_pose = self._live_gazebo_pose(wheel_link)
        if live_pose is not None:
            wx, wy, wz = live_pose
            return wx, wy, wz, 'gz'

        try:
            tf = self._tf_buffer.lookup_transform(
                'world', wheel_link, rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout_s))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            'tf')

    def _gazebo_pose_callback(self, msg: TFMessage):
        now = time.time()
        for transform in msg.transforms:
            name = transform.child_frame_id
            if not name:
                continue
            translation = transform.transform.translation
            self._gazebo_poses[name] = (
                translation.x, translation.y, translation.z, now)

    def _contact_callback(self, wheel_link: str, msg: Contacts):
        self._contact_msg_counts[wheel_link] = (
            self._contact_msg_counts.get(wheel_link, 0) + 1)
        best = None
        for contact in msg.contacts:
            collision1 = contact.collision1.name
            collision2 = contact.collision2.name
            if not self._contact_mentions_wheel(wheel_link, collision1, collision2):
                continue

            other_collision = self._other_contact_collision(
                wheel_link, collision1, collision2)
            if not self._is_magnetic_surface(other_collision):
                continue

            position = None
            if contact.positions:
                p = contact.positions[0]
                position = (p.x, p.y, p.z)

            normal = None
            if contact.normals:
                n = contact.normals[0]
                normal = _normalize_xyz(n.x, n.y, n.z)

            depth_m = max(contact.depths) if contact.depths else 0.0
            contact_points = max(
                len(contact.positions), len(contact.normals), len(contact.depths))
            contact_fraction = estimate_contact_fraction(
                depth_m,
                contact_points,
                self._full_contact_depth_m,
                self._full_contact_points,
                self._min_contact_fraction)
            candidate = ContactState(
                stamp_s=time.time(),
                surface=self._surface_name_from_collision(other_collision),
                collision1=collision1,
                collision2=collision2,
                position=position,
                normal=normal,
                depth_m=depth_m,
                contact_points=contact_points,
                contact_fraction=contact_fraction,
            )
            if best is None or candidate.depth_m > best.depth_m:
                best = candidate

        with self._contact_lock:
            if best is None:
                self._wheel_contacts.pop(wheel_link, None)
            else:
                self._wheel_contacts[wheel_link] = best
                self._contact_seen_counts[wheel_link] = (
                    self._contact_seen_counts.get(wheel_link, 0) + 1)

    def _latest_contact(self, wheel_link: str):
        with self._contact_lock:
            contact = self._wheel_contacts.get(wheel_link)
        if contact is None:
            return None
        if time.time() - contact.stamp_s > self._contact_timeout_s:
            with self._contact_lock:
                self._wheel_contacts.pop(wheel_link, None)
            return None
        return contact

    def _contact_mentions_wheel(
            self, wheel_link: str, collision1: str, collision2: str):
        return (
            self._name_mentions_link(collision1, wheel_link) or
            self._name_mentions_link(collision2, wheel_link)
        )

    def _other_contact_collision(
            self, wheel_link: str, collision1: str, collision2: str):
        if self._name_mentions_link(collision1, wheel_link):
            return collision2
        return collision1

    def _name_mentions_link(self, name: str, link_name: str):
        lowered = name.lower()
        return link_name.lower() in lowered

    def _is_magnetic_surface(self, collision_name: str):
        lowered = collision_name.lower()
        if self._robot_name.lower() in lowered:
            return False
        if self._magnetic_surface_allowlist:
            return any(
                token in lowered for token in self._magnetic_surface_allowlist)
        magnetic_keywords = (
            'steel', 'panel', 'slide', 'wall', 'ground_plane',
            'mockup_structure', 'pipe', 'tank', 'ferromagnetic')
        return any(keyword in lowered for keyword in magnetic_keywords)

    def _surface_name_from_collision(self, collision_name: str):
        lowered = collision_name.lower()
        if 'ground_plane' in lowered:
            return 'ground'
        if 'slide' in lowered:
            return 'slide'
        if 'panel' in lowered or 'wall' in lowered:
            return 'wall'
        if 'pipe' in lowered or 'mockup_structure' in lowered:
            return 'pipe'
        if 'steel' in lowered:
            return 'steel'
        return 'surface'

    def _contact_normal_direction(self, wheel_link: str, contact: ContactState):
        if contact.normal is None:
            return 1.0, 0.0, 0.0

        nx, ny, nz = contact.normal
        # Gazebo contact normals are reported relative to collision1/collision2.
        # If the wheel is collision2, flip so the force points from wheel to
        # the surface instead of away from it.
        if self._name_mentions_link(contact.collision2, wheel_link):
            nx, ny, nz = -nx, -ny, -nz
        return _normalize_xyz(nx, ny, nz)

    def _live_gazebo_pose(self, wheel_link: str):
        candidates = (
            f'{self._robot_name}::{wheel_link}',
            f'{self._robot_name}/{wheel_link}',
            wheel_link,
        )
        now = time.time()
        for name, pose in self._gazebo_poses.items():
            matches = (
                name in candidates or
                name.endswith(f'::{wheel_link}') or
                name.endswith(f'/{wheel_link}')
            )
            if matches:
                wx, wy, wz, stamp = pose
                if now - stamp <= self._gazebo_pose_timeout_s:
                    return wx, wy, wz
        return None

    def _slide_surface_for_wheel(self, wheel_link: str, wx: float, wy: float, wz: float):
        angle = self._slide_angle_rad
        margin = self._surface_margin_m
        slide_normal = (-math.sin(angle), 0.0, math.cos(angle))
        slide_dir = (math.cos(angle), 0.0, math.sin(angle))
        slide_len = max(
            0.0, (self._slide_wall_x - self._slide_start_x) / max(1e-6, slide_dir[0]))
        slide_top_z = self._slide_start_z + slide_len * slide_dir[2]

        ground = self._surface_candidate(
            'ground',
            (0.0, 0.0, 1.0),
            (0.0, 0.0, self._ground_z),
            wx, wy, wz,
            valid=(
                wx <= self._slide_start_x + margin and
                wz <= self._slide_start_z + self._wheel_radius_m + 2.0 * margin))

        slide_progress = (
            (wx - self._slide_start_x) * slide_dir[0] +
            (wz - self._slide_start_z) * slide_dir[2])
        slide = self._surface_candidate(
            'slide',
            slide_normal,
            (self._slide_start_x, 0.0, self._slide_start_z),
            wx, wy, wz,
            valid=(-margin <= slide_progress <= slide_len + margin))

        wall = self._surface_candidate(
            'wall',
            (-1.0, 0.0, 0.0),
            (self._slide_wall_x, 0.0, 0.0),
            wx, wy, wz,
            valid=(
                wx >= self._slide_wall_x - self._wheel_radius_m - margin and
                wz >= slide_top_z - 2.0 * margin))

        candidates = [c for c in (ground, slide, wall) if c is not None]
        if not candidates:
            # Fall back to the ramp near the transition area because it is the
            # only surface that can carry the robot toward the wall.
            candidates = [
                self._surface_candidate(
                    'slide', slide_normal,
                    (self._slide_start_x, 0.0, self._slide_start_z),
                    wx, wy, wz, valid=True)
            ]

        best = min(candidates, key=lambda item: item['score'])
        previous_name = self._surface_latch.get(wheel_link)
        previous = next(
            (c for c in candidates if c['name'] == previous_name), None)
        if (previous is not None and
                previous['score'] <= best['score'] + self._surface_hysteresis_m):
            best = previous

        self._surface_latch[wheel_link] = best['name']
        return best['normal'], best['gap_m'], best['name']

    def _surface_candidate(
            self, name: str, normal, point, wx: float, wy: float, wz: float,
            valid: bool):
        if not valid:
            return None

        signed_center_gap = signed_plane_distance_m(
            wx, wy, wz,
            point[0], point[1], point[2],
            normal[0], normal[1], normal[2])
        gap_m = max(0.0, signed_center_gap - self._wheel_radius_m)
        contact_error = abs(signed_center_gap - self._wheel_radius_m)
        return {
            'name': name,
            'normal': normal,
            'gap_m': gap_m,
            'score': contact_error,
        }

    def _force_for_gap(
            self, gap_mm: float, contact_fraction: float = 1.0,
            tilt_deg: float = 0.0):
        contact_fraction = max(0.0, min(1.0, contact_fraction))
        if (self._adhesion_model == 'constant' and
                not self._scale_constant_by_contact_fraction and
                contact_fraction > 0.0):
            contact_fraction = 1.0
        tilt_factor = tilt_scale(
            tilt_deg, self._safe_tilt_deg, self._cutoff_tilt_deg)

        if self._adhesion_model == 'constant' or self._force_table is None:
            if self._adhesion_model == 'gap_decay':
                force_n = self._force_n * gap_decay_scale(
                    gap_mm + self._gap_bias_mm, self._gap_reference_mm)
            else:
                force_n = self._force_n
            return max(force_n * contact_fraction * tilt_factor, self._min_force_n)

        lookup_gap_mm = max(0.0, gap_mm + self._gap_bias_mm)
        if self._max_lookup_gap_mm >= 0.0:
            lookup_gap_mm = min(lookup_gap_mm, self._max_lookup_gap_mm)
        force_n, clamped = self._force_table.lookup(
            lookup_gap_mm,
            self._wall_thickness_mm,
            self._pipe_radius_m,
            max(0.0, min(1.0, contact_fraction)))
        for key, was_clamped in clamped.items():
            if was_clamped and key not in self._clamp_warnings:
                self._clamp_warnings.add(key)
                self.get_logger().warn(
                    f'adhesion lookup {key} outside CSV range; clamping.',
                    throttle_duration_sec=10.0)
        if not self._force_table.has_contact_fraction_axis:
            force_n *= contact_fraction
        force_n *= tilt_factor
        return max(force_n, self._min_force_n)

    # ── gz service call ────────────────────────────────────────────────────────

    def _apply_world_wrench(self, link: str, fx: float, fy: float, fz: float):
        """Apply a world-frame force at the given robot link for ~2 periods."""
        self._sent_count += 1
        duration_nsec = int(self._period_s * 2 * 1e9)
        entity = f'{self._robot_name}::{link}'
        req = (
            f'entity: {{name: "{entity}", type: LINK}}, '
            f'wrench: {{force: {{x: {fx:.2f}, y: {fy:.2f}, z: {fz:.2f}}}}}, '
            f'duration: {{sec: 0, nsec: {duration_nsec}}}'
        )
        try:
            result = subprocess.run(
                ['gz', 'service',
                 '-s', f'/world/{self._world_name}/apply_link_wrench',
                 '--reqtype', 'gz.msgs.EntityWrench',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', str(self._service_timeout_ms),
                 '--req', req],
                capture_output=True,
                text=True,
                timeout=self._process_timeout_s,
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or '').strip()
                self._fail_count += 1
                self._last_error = detail
                self.get_logger().warn(
                    f'gz service failed for {link}: {detail}',
                    throttle_duration_sec=5.0)
                return
            output = (result.stdout or result.stderr or '').strip().lower()
            if 'false' in output and 'true' not in output:
                self._fail_count += 1
                self._last_error = output
                self.get_logger().warn(
                    f'Gazebo rejected adhesion wrench for {link}: {output}',
                    throttle_duration_sec=5.0)
                return
            self._ok_count += 1
        except subprocess.TimeoutExpired:
            self._fail_count += 1
            self._last_error = f'gz service timeout for {link}'
            self.get_logger().warn(
                f'gz service timeout for {link}',
                throttle_duration_sec=5.0)
        except FileNotFoundError:
            self._fail_count += 1
            self._last_error = 'gz CLI not found'
            self.get_logger().error(
                'gz CLI not found — install gz-tools or source the Gazebo setup.',
                once=True)

    def _publish_persistent_wrenches(self):
        fx, fy, fz = self._plane_fx, self._plane_fy, self._plane_fz
        for wheel in WHEEL_LINKS:
            self._last_forces[wheel] = (fx, fy, fz)
            self._last_gaps_mm[wheel] = None
            self._last_surfaces[wheel] = 'plane'
            self._publish_persistent_world_wrench(wheel, fx, fy, fz)
        self._persistent_active = self._ok_count > 0
        if self._ok_count == len(WHEEL_LINKS):
            self.get_logger().info(
                'Persistent adhesion active on all wheel links.')
        else:
            self.get_logger().warn(
                'Persistent adhesion did not activate cleanly on all wheels. '
                'Check Gazebo /world/default/wrench/persistent topic and link names.')

    def _publish_persistent_world_wrench(self, link: str, fx: float, fy: float, fz: float):
        self._sent_count += 1
        entity = f'{self._robot_name}::{link}'
        msg = (
            f'entity: {{name: "{entity}", type: LINK}}, '
            f'wrench: {{force: {{x: {fx:.2f}, y: {fy:.2f}, z: {fz:.2f}}}}}'
        )
        try:
            result = subprocess.run(
                ['gz', 'topic',
                 '-t', f'/world/{self._world_name}/wrench/persistent',
                 '-m', 'gz.msgs.EntityWrench',
                 '-p', msg],
                capture_output=True,
                text=True,
                timeout=self._process_timeout_s,
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or '').strip()
                self._fail_count += 1
                self._last_error = detail
                self.get_logger().warn(
                    f'gz persistent wrench failed for {link}: {detail}',
                    throttle_duration_sec=5.0)
                return
            self._ok_count += 1
        except subprocess.TimeoutExpired:
            self._fail_count += 1
            self._last_error = f'gz topic timeout for {link}'
            self.get_logger().warn(
                f'gz topic timeout for {link}', throttle_duration_sec=5.0)
        except FileNotFoundError:
            self._fail_count += 1
            self._last_error = 'gz CLI not found'
            self.get_logger().error(
                'gz CLI not found — install gz-tools or source the Gazebo setup.',
                once=True)

    def _clear_persistent_wrenches(self):
        for wheel in WHEEL_LINKS:
            entity = f'name: "{self._robot_name}::{wheel}", type: LINK'
            try:
                subprocess.run(
                    ['gz', 'topic',
                     '-t', f'/world/{self._world_name}/wrench/clear',
                     '-m', 'gz.msgs.Entity',
                     '-p', entity],
                    capture_output=True,
                    text=True,
                    timeout=self._process_timeout_s,
                )
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

    def _wheel_boundary_risk(self, attached: bool, gap_mm, contact_fraction: float,
                             pose_source: str, contact_points: int):
        if not attached:
            return 1.0

        risk = 0.0
        if contact_fraction < self._boundary_low_contact_fraction:
            denom = max(1e-6, self._boundary_low_contact_fraction)
            risk = max(risk, 1.0 - contact_fraction / denom)

        if gap_mm is not None:
            risk = max(risk, max(0.0, min(
                1.0, gap_mm / max(1e-6, self._boundary_high_risk_gap_mm))))

        if contact_points <= 1 and pose_source.startswith('contact'):
            risk = max(risk, 0.55)
        if pose_source.startswith('capture'):
            risk = max(risk, 0.45)
        if pose_source in ('tf', 'fallback'):
            risk = max(risk, 0.35)
        return max(0.0, min(1.0, risk))

    @staticmethod
    def _boundary_state(risk: float):
        if risk >= 0.75:
            return 'hold'
        if risk >= 0.45:
            return 'caution'
        return 'ok'

    def _publish_status(self):
        if self._status_period_s <= 0.0:
            return
        now = time.time()
        if now < self._next_status_time:
            return
        self._next_status_time = now + self._status_period_s

        counts = {
            'ground': 0,
            'slide': 0,
            'wall': 0,
            'pipe': 0,
        }
        attached_count = 0
        wheels = []
        risks = []
        active_contact_fractions = []
        for wheel in WHEEL_LINKS:
            attached = bool(self._last_attached.get(wheel, False))
            surface = str(self._last_surfaces.get(wheel, 'none'))
            clean_surface = surface.split(':', 1)[-1] if surface.startswith('contact:') else surface
            if clean_surface == 'cylinder':
                clean_surface = 'pipe'
            gap = self._last_gaps_mm.get(wheel)
            force = self._last_forces.get(wheel, (0.0, 0.0, 0.0))
            force_n = math.sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2])
            contact_fraction = self._last_contact_fractions.get(wheel, 0.0)
            contact_points = self._last_contact_points.get(wheel, 0)
            pose_source = self._last_pose_sources.get(wheel, 'none')
            risk = self._wheel_boundary_risk(
                attached, gap, contact_fraction, pose_source, contact_points)
            risks.append(risk)
            if attached:
                attached_count += 1
                active_contact_fractions.append(contact_fraction)
                if clean_surface in counts:
                    counts[clean_surface] += 1
            wheels.append({
                'wheel': wheel,
                'surface': clean_surface,
                'attached': attached,
                'gap_mm': 0.0 if gap is None else gap,
                'force_n': force_n,
                'force_vector_n': [force[0], force[1], force[2]],
                'holding_torque_nm': force_n * self._wheel_radius_m,
                'contact_fraction': contact_fraction,
                'contact_points': contact_points,
                'depth_m': self._last_depths_m.get(wheel, 0.0),
                'normal': list(self._last_normals.get(wheel, (0.0, 0.0, 0.0))),
                'pose_source': pose_source,
                'boundary_risk': risk,
                'boundary_state': self._boundary_state(risk),
            })

        dominant_surface = 'none'
        if attached_count > 0:
            dominant_surface = max(counts.items(), key=lambda item: item[1])[0]
        boundary_risk = max(risks, default=1.0)
        avg_contact_fraction = (
            sum(active_contact_fractions) / len(active_contact_fractions)
            if active_contact_fractions else 0.0)

        msg = String()
        msg.data = json.dumps({
            'dominant_surface': dominant_surface,
            'attached_count': attached_count,
            'ground_count': counts['ground'],
            'slide_count': counts['slide'],
            'wall_count': counts['wall'],
            'pipe_count': counts['pipe'],
            'mode': self._mode,
            'adhesion_model': self._adhesion_model,
            'wall_thickness_mm': self._wall_thickness_mm,
            'pipe_radius_m': self._pipe_radius_m,
            'surface_radius_m': self._surface_radius_m,
            'cylinder_axis': self._cylinder_axis,
            'wheel_radius_m': self._wheel_radius_m,
            'avg_contact_fraction': avg_contact_fraction,
            'boundary_risk': boundary_risk,
            'boundary_state': self._boundary_state(boundary_risk),
            'boundary_low_contact_fraction': self._boundary_low_contact_fraction,
            'boundary_high_risk_gap_mm': self._boundary_high_risk_gap_mm,
            'wheels': wheels,
        }, separators=(',', ':'))
        self._status_pub.publish(msg)

    def _publish_diagnostic(self):
        if self._diagnostic_period_s <= 0.0:
            return
        now = time.time()
        if now < self._next_diagnostic_time:
            return
        self._next_diagnostic_time = now + self._diagnostic_period_s

        force_parts = []
        for wheel, (fx, fy, fz) in self._last_forces.items():
            gap = self._last_gaps_mm.get(wheel)
            gap_text = 'gap=n/a' if gap is None else f'gap={gap:.1f}mm'
            surface = self._last_surfaces.get(wheel, 'n/a')
            pose_source = self._last_pose_sources.get(wheel, 'n/a')
            contact_msgs = self._contact_msg_counts.get(wheel, 0)
            contact_hits = self._contact_seen_counts.get(wheel, 0)
            contact_fraction = self._last_contact_fractions.get(wheel, 0.0)
            force_parts.append(
                f'{wheel}=({fx:+.0f},{fy:+.0f},{fz:+.0f})N,'
                f'{gap_text},surface={surface},pose={pose_source},'
                f'contact_fraction={contact_fraction:.2f},'
                f'contact_msgs={contact_msgs},contact_hits={contact_hits}')
        forces = ', '.join(force_parts)
        status = (
            f'adhesion status: sent={self._sent_count}, ok={self._ok_count}, '
            f'failed={self._fail_count}; model={self._adhesion_model}, '
            f'wall={self._wall_thickness_mm:.1f}mm; {forces}')
        if self._last_error:
            status += f'; last_error="{self._last_error}"'
        self.get_logger().info(status)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MagneticAdhesionNode()
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
