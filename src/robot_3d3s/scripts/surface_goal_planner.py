#!/usr/bin/env python3
"""Unified RViz goal planner for magnetic climbing surfaces.

The planner is **surface-agnostic**: it drives motion using a
``SurfaceGraph`` of any combination of ``CylinderSurface``,
``PolygonSurface``, and future surface types.  The user can swap the
underlying surface at runtime in two ways:

* Change the ``surface_graph_yaml`` parameter — the planner reloads
  the graph and re-routes the active goal onto the new geometry.
* Call the ``~/reload_surface_graph`` service with a new YAML path
  (or with the empty string to use the legacy single-cylinder
  fallback).

When no graph is configured the planner still works for the original
single-cylinder use case so existing launches keep running.  The old
hard-coded ``surface_mode`` flag is preserved as an informational
parameter; the controller no longer branches on it.
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
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from surface_geometry import (
    CylinderSurface,
    PolygonSurface,
    RoutePoint,
    Surface,
    SurfaceGraph,
    SurfaceProjection,
    default_cylinder_graph,
    edge_motion_scale,
    footprint_edge_clearance,
    graph_from_yaml,
)


def _clamp(value: float, lower: float, upper: float):
    return max(lower, min(upper, value))


def _wrap_pi(angle: float):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _quat_rotate(qx: float, qy: float, qz: float, qw: float,
                 x: float, y: float, z: float) -> Tuple[float, float, float]:
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * z)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


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


class SurfaceGoalPlanner(Node):
    def __init__(self):
        super().__init__('surface_goal_planner')

        # ---- World / IO parameters ---------------------------------------
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/surface_goal_markers')
        self.declare_parameter('debug_topic', '/surface_goal_planner/debug')
        self.declare_parameter('legacy_debug_topic', '')
        self.declare_parameter('hold_steering_topic',
                               '/swerve_controller/hold_steering')

        # ---- Surface graph (live configurable) ---------------------------
        self.declare_parameter('surface_graph_yaml', '')
        # ``surface_mode`` is now informational only; the controller drives
        # any surface via SurfaceGraph.  Kept for backward-compatible launch
        # files.
        self.declare_parameter('surface_mode', 'auto')
        self.declare_parameter('show_experimental_polygon', False)
        self.declare_parameter('experimental_polygon_goal_selection', False)
        self.declare_parameter('route_samples_per_surface', 24)

        # Legacy single-cylinder fallback parameters.  Used when no graph
        # YAML is provided.
        self.declare_parameter('cylinder_axis', 'x')
        self.declare_parameter('axis_x', 0.0)
        self.declare_parameter('axis_y', 0.0)
        self.declare_parameter('axis_z', 1.20)
        self.declare_parameter('surface_radius_m', 1.20)
        self.declare_parameter('surface_length_m', 6.00)

        # ---- Safety / control parameters ---------------------------------
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('min_attached_wheels', 2)
        self.declare_parameter('status_timeout_s', 0.7)
        self.declare_parameter('tf_timeout_s', 0.1)
        self.declare_parameter('boundary_hold_risk', 0.78)
        self.declare_parameter('attachment_settle_s', 0.0)
        self.declare_parameter('robot_mass_kg', 8.715)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('friction_mu', 1.0)
        self.declare_parameter('safety_factor', 1.5)
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('wheel_goal_projection', False)
        self.declare_parameter('wheel_steering_radius_m', 0.38905)
        self.declare_parameter('wheel_torque_limit_nm', 40.0)
        self.declare_parameter('min_side_contact_fraction', 0.45)
        self.declare_parameter('steep_load_fraction', 0.35)
        self.declare_parameter('allow_guarded_low_contact', False)
        self.declare_parameter('edge_margin_m', 0.10)
        self.declare_parameter('edge_slow_min_scale', 0.20)

        # ---- Target / approach parameters --------------------------------
        self.declare_parameter('target_axis_m', -2.0)
        self.declare_parameter('target_angle_deg', 90.0)
        self.declare_parameter('wait_for_goal', True)
        self.declare_parameter('approach_axis_m', -2.0)
        self.declare_parameter('approach_theta_deg', -90.0)
        self.declare_parameter('approach_speed_mps', 0.12)
        self.declare_parameter('axis_speed_mps', 0.08)
        self.declare_parameter('curve_speed_mps', 0.14)
        self.declare_parameter('kp_axis', 0.6)
        self.declare_parameter('kp_theta', 0.9)
        self.declare_parameter('angle_tolerance_deg', 4.0)
        self.declare_parameter('axis_tolerance_m', 0.06)

        gp = self.get_parameter
        self.world_frame = str(gp('world_frame').value)
        self.base_frame = str(gp('base_frame').value)
        self.publish_hz = max(2.0, float(gp('publish_hz').value))
        self.min_attached = int(gp('min_attached_wheels').value)
        self.status_timeout = float(gp('status_timeout_s').value)
        self.tf_timeout = float(gp('tf_timeout_s').value)
        self.boundary_hold_risk = max(
            0.0, min(1.0, float(gp('boundary_hold_risk').value)))
        self.attachment_settle_s = max(
            0.0, float(gp('attachment_settle_s').value))
        self.robot_mass = max(0.1, float(gp('robot_mass_kg').value))
        self.gravity = max(0.1, float(gp('gravity').value))
        self.friction_mu = max(0.0, float(gp('friction_mu').value))
        self.safety_factor = max(1.0, float(gp('safety_factor').value))
        self.wheel_radius = max(1e-3, float(gp('wheel_radius_m').value))
        self.wheel_goal_projection = bool(gp('wheel_goal_projection').value)
        self.wheel_steering_radius = max(
            0.0, float(gp('wheel_steering_radius_m').value))
        wheel_half_radius = 0.5 * self.wheel_steering_radius
        wheel_side_offset = 0.5 * math.sqrt(3.0) * self.wheel_steering_radius
        self.wheel_offsets = (
            ('wheel_1', self.wheel_steering_radius, 0.0),
            ('wheel_2', -wheel_half_radius, wheel_side_offset),
            ('wheel_3', -wheel_half_radius, -wheel_side_offset),
        )
        self.wheel_torque_limit = max(
            0.0, float(gp('wheel_torque_limit_nm').value))
        self.min_side_contact_fraction = max(
            0.0, min(1.0, float(gp('min_side_contact_fraction').value)))
        self.steep_load_fraction = max(
            0.0, min(1.0, float(gp('steep_load_fraction').value)))
        self.allow_guarded_low_contact = bool(
            gp('allow_guarded_low_contact').value)
        self.edge_margin = max(0.0, float(gp('edge_margin_m').value))
        self.edge_slow_min_scale = _clamp(
            float(gp('edge_slow_min_scale').value), 0.01, 1.0)

        self.surface_graph_yaml = str(gp('surface_graph_yaml').value)
        self.surface_mode = str(gp('surface_mode').value).lower()
        self.show_experimental_polygon = bool(
            gp('show_experimental_polygon').value)
        self.experimental_polygon_goal_selection = bool(
            gp('experimental_polygon_goal_selection').value)
        self.route_samples_per_surface = max(
            2, int(gp('route_samples_per_surface').value))

        # Legacy single-cylinder parameters (used only when no graph YAML
        # is available).
        self.axis = str(gp('cylinder_axis').value).lower()
        if self.axis not in ('x', 'y', 'z'):
            self.get_logger().warn(f'Invalid cylinder_axis={self.axis}; using x.')
            self.axis = 'x'
        self.axis_point = (
            float(gp('axis_x').value),
            float(gp('axis_y').value),
            float(gp('axis_z').value),
        )
        self.radius = max(0.05, float(gp('surface_radius_m').value))
        self.length = max(0.1, float(gp('surface_length_m').value))
        self.target_axis = float(gp('target_axis_m').value)
        self.target_theta = math.radians(float(gp('target_angle_deg').value))
        self.wait_for_goal = bool(gp('wait_for_goal').value)
        self.approach_axis = float(gp('approach_axis_m').value)
        self.approach_theta = math.radians(
            float(gp('approach_theta_deg').value))
        self.approach_speed = float(gp('approach_speed_mps').value)
        self.axis_speed = float(gp('axis_speed_mps').value)
        self.curve_speed = float(gp('curve_speed_mps').value)
        self.kp_axis = float(gp('kp_axis').value)
        self.kp_theta = float(gp('kp_theta').value)
        self.angle_tol = math.radians(float(gp('angle_tolerance_deg').value))
        self.axis_tol = float(gp('axis_tolerance_m').value)

        # ---- State -------------------------------------------------------
        self.active_surface_name = 'configured_surface'
        self.target_projection: Optional[SurfaceProjection] = None
        self.goal_projection: Optional[SurfaceProjection] = None
        self.current_projection: Optional[SurfaceProjection] = None
        self.route_points: list = []
        self.route_index = 0
        self.edge_clearance_m = float('inf')
        self.edge_motion_scale = 1.0

        self.attached = 0
        self.sum_force_n = 0.0
        self.avg_contact_fraction = 0.0
        self.pipe_count = 0
        self.dominant_surface = 'none'
        self.surface_counts: dict = {}
        self.boundary_risk = 0.0
        self.boundary_state = 'ok'
        self.last_status_t = None
        self.contact_ready_since = None
        self.contact_settled = False
        self.reached = False
        self.goal_active = not self.wait_for_goal
        self.goal_surface_axis = self.target_axis
        self.goal_surface_theta = self.target_theta
        self.goal_wheel_name = 'base_center'
        self.goal_wheel_error_m = 0.0
        self.last_safety: dict = {}

        # ---- IO setup ----------------------------------------------------
        self.cmd_pub = self.create_publisher(
            TwistStamped, str(gp('cmd_topic').value), 10)
        self.hold_steering_pub = self.create_publisher(
            Float64MultiArray, str(gp('hold_steering_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, str(gp('debug_topic').value), 10)
        legacy_debug_topic = str(gp('legacy_debug_topic').value)
        self.legacy_debug_pub = (
            self.create_publisher(String, legacy_debug_topic, 10)
            if legacy_debug_topic else None)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray, str(gp('marker_topic').value), marker_qos)

        self.create_subscription(
            String, str(gp('status_topic').value), self._status_cb, 10)
        self.create_subscription(
            PointStamped, str(gp('clicked_point_topic').value),
            self._clicked_cb, 10)
        self.create_subscription(
            PoseStamped, str(gp('goal_pose_topic').value),
            self._pose_goal_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Load initial surface graph ----------------------------------
        self.surface_graph = self._load_surface_graph()
        if self.surface_graph.surfaces:
            # ``self.surface_graph.surfaces`` is a Dict[name, Surface],
            # so iter() yields the *name* strings, not Surface objects.
            self.active_surface_name = next(iter(self.surface_graph.surfaces))
        self._install_parameter_callback()
        self._install_reload_service()

        # Re-project the current goal onto the active surface.
        if not self.wait_for_goal:
            self._rebuild_default_goal()

        self.create_timer(1.0 / self.publish_hz, self._tick)
        self.create_timer(0.5, self._publish_markers)

        self.get_logger().info(
            f'surface_goal_planner ready: graph surfaces='
            f'{len(self.surface_graph.surfaces)} '
            f'(mode={self.surface_mode}, yaml='
            f'"{self.surface_graph_yaml or "<single-cylinder-fallback>"}"), '
            f'active_surface={self.active_surface_name}. '
            + ('Waiting for RViz Publish Point / 2D Goal Pose.'
               if self.wait_for_goal else
               f'target axis={self.target_axis:.2f}, '
               f'theta={math.degrees(self.target_theta):.1f} deg'))

    # ------------------------------------------------------------------
    # Surface graph management
    # ------------------------------------------------------------------
    def _install_parameter_callback(self):
        """Reload the graph whenever the YAML parameter changes."""

        from rcl_interfaces.msg import SetParametersResult

        def _on_change(params):
            for p in params:
                if p.name == 'surface_graph_yaml' and p.type_ != p.NOT_SET:
                    new_value = (
                        p.value.string_value
                        if hasattr(p.value, 'string_value') else '')
                    if new_value != self.surface_graph_yaml:
                        self.surface_graph_yaml = new_value
                        self._reload_surface_graph(reason='parameter change')
            # Always return a valid SetParametersResult so rclpy does not
            # reject unrelated parameter declarations (e.g. goal_x) that
            # flow through this callback.
            return SetParametersResult(successful=True)

        try:
            self.add_on_set_parameters_callback(_on_change)
        except Exception as exc:  # noqa: BLE001 - keep planner alive
            self.get_logger().warn(
                f'Cannot install parameter callback: {exc}')

    def _install_reload_service(self):
        from std_srvs.srv import SetBool, Trigger
        # Trigger variant: reload the currently configured YAML.
        self.create_service(Trigger, '~/reload_surface_graph',
                            self._reload_trigger_cb)
        # SetBool variant: enable/disable the experimental polygon graph at
        # runtime (True == enable, False == disable).
        self.create_service(SetBool, '~/use_experimental_polygon_graph',
                            self._use_experimental_polygon_cb)
        # Trigger variant: set the goal from ``goal_x``/``goal_y``/``goal_z``
        # parameters.  This is the CLI fallback for when RViz Publish Point
        # cannot reliably hit the pipe surface in the running RViz version.
        # Usage:
        #   ros2 param set /surface_goal_planner goal_x 0.5
        #   ros2 param set /surface_goal_planner goal_y 1.18
        #   ros2 param set /surface_goal_planner goal_z 2.0
        #   ros2 service call /surface_goal_planner/set_goal_from_params \
        #       std_srvs/srv/Trigger
        # Alternative: publish a one-shot ``PointStamped`` directly on
        # ``/clicked_point`` (the same topic RViz Publish Point uses).
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.0)
        self.create_service(Trigger, '~/set_goal_from_params',
                            self._set_goal_from_params_srv_cb)

    def _reload_trigger_cb(self, request, response):
        ok = self._reload_surface_graph(reason='service call')
        response.success = ok
        response.message = (
            f'reloaded {len(self.surface_graph.surfaces)} surfaces'
            if ok else 'reload failed; see node logs')
        return response

    def _use_experimental_polygon_cb(self, request, response):
        enable = bool(request.data)
        if enable:
            self.surface_graph_yaml = '__experimental_polygon__'
        else:
            self.surface_graph_yaml = ''
        ok = self._reload_surface_graph(
            reason='service use_experimental_polygon_graph')
        response.success = ok
        response.message = (
            f'experimental polygon graph active: '
            f'{len(self.surface_graph.surfaces)} surfaces'
            if ok else 'reload failed; see node logs')
        return response

    def _reload_surface_graph(self, reason: str) -> bool:
        try:
            new_graph = self._build_surface_graph()
        except Exception as exc:  # noqa: BLE001 - keep old graph alive
            self.get_logger().error(
                f'surface graph reload failed ({reason}): {exc}; '
                'keeping previous graph.')
            return False
        self.surface_graph = new_graph
        if new_graph.surfaces:
            # ``new_graph.surfaces`` is a Dict[name, Surface]; the keys
            # are the surface names.
            self.active_surface_name = next(iter(new_graph.surfaces))
        self._clear_route_and_goal('graph reloaded')
        self._publish_markers()
        self.get_logger().info(
            f'surface graph reloaded ({reason}): '
            f'{len(new_graph.surfaces)} surfaces, '
            f'active={self.active_surface_name}.')
        return True

    def _build_surface_graph(self) -> SurfaceGraph:
        if self.surface_graph_yaml == '__experimental_polygon__':
            return self._experimental_polygon_graph()
        if self.surface_graph_yaml:
            return graph_from_yaml(self.surface_graph_yaml)
        if (self.experimental_polygon_goal_selection and
                self.show_experimental_polygon):
            return self._experimental_polygon_graph()
        return default_cylinder_graph(
            self.axis, self.axis_point, self.radius, self.length)

    def _load_surface_graph(self) -> SurfaceGraph:
        try:
            graph = self._build_surface_graph()
            if graph.surfaces:
                return graph
        except Exception as exc:  # noqa: BLE001 - keep planner alive
            self.get_logger().warn(
                f'Cannot build initial surface graph: {exc}; '
                'falling back to configured cylinder.')
        return default_cylinder_graph(
            self.axis, self.axis_point, self.radius, self.length)

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

    def _experimental_polygon_graph(self) -> SurfaceGraph:
        surfaces = []
        for item in self._experimental_polygon_surfaces():
            surfaces.append(CylinderSurface(
                item['name'], item['axis'], tuple(item['axis_point']),
                float(item['radius']), float(item['length'])))
        # Portal points are at the geometric intersections of the three
        # pipes.  These let the planner transition through a T-junction
        # when it has to move from one pipe onto another.
        transitions = [
            {
                'from': 'left_vertical_pipe',
                'to': 'horizontal_pipe',
                'cost': 2.0,
                'risk': 0.4,
                'point': [-1.1084, 0.0, 1.246],
            },
            {
                'from': 'horizontal_pipe',
                'to': 'right_vertical_pipe',
                'cost': 2.0,
                'risk': 0.4,
                'point': [1.3116, 0.0, 1.246],
            },
        ]
        return SurfaceGraph(surfaces, transitions)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        wheels = data.get('wheels', [])
        attached = [w for w in wheels if w.get('attached')]
        self.attached = int(data.get('attached_count', len(attached)))
        self.sum_force_n = sum(
            float(w.get('force_n', 0.0)) for w in attached)
        if attached:
            self.avg_contact_fraction = sum(
                float(w.get('contact_fraction', 0.0))
                for w in attached) / len(attached)
        else:
            self.avg_contact_fraction = 0.0
        self.pipe_count = int(data.get('pipe_count', 0))
        self.dominant_surface = str(data.get('dominant_surface', 'none'))
        self.boundary_risk = float(data.get('boundary_risk', 0.0))
        self.boundary_state = str(data.get('boundary_state', 'ok'))
        counts = {}
        for wheel in wheels:
            surface = str(wheel.get('surface', 'none'))
            if surface == 'cylinder':
                surface = 'pipe'
            if wheel.get('attached'):
                counts[surface] = counts.get(surface, 0) + 1
        self.surface_counts = counts
        self.pipe_count = max(self.pipe_count, counts.get('pipe', 0))
        self.last_status_t = self.get_clock().now()

    def _clicked_cb(self, msg: PointStamped):
        self.get_logger().info(
            f'Received /clicked_point in frame '
            f'"{msg.header.frame_id or "<empty>"}": '
            f'({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})')
        self._set_goal_from_world_point(
            msg.point.x, msg.point.y, msg.point.z, 'Publish Point')

    def _set_goal_from_params_srv_cb(self, request, response):
        """Service wrapper for ``~/set_goal_from_params``.

        Lets users set the goal from the CLI when RViz Publish Point
        cannot reliably hit the pipe surface.  The world-space point
        is read from the ``goal_x``/``goal_y``/``goal_z`` parameters,
        which the caller sets with ``ros2 param set`` before invoking
        the service.  This bypasses RViz Publish Point entirely.
        """
        try:
            x = float(self.get_parameter('goal_x').value)
            y = float(self.get_parameter('goal_y').value)
            z = float(self.get_parameter('goal_z').value)
        except (TypeError, ValueError) as exc:
            response.success = False
            response.message = f'invalid goal_x/goal_y/goal_z: {exc}'
            return response
        self.get_logger().info(
            f'set_goal_from_params service: ({x:.3f}, {y:.3f}, {z:.3f})')
        goal_projection = self.surface_graph.closest_projection(
            (x, y, z))
        if goal_projection is None:
            response.success = False
            response.message = 'no surfaces in graph; nothing to project onto'
            return response
        if goal_projection.distance > 1.5:
            response.success = False
            response.message = (
                f'click is {goal_projection.distance:.2f} m from the '
                f'closest surface ({goal_projection.surface.name}); '
                f'move closer to the pipe before setting the goal')
            self.get_logger().warn(response.message)
            return response
        before_active = self.goal_active
        self._set_goal_from_world_point(
            x, y, z,
            f'set_goal_from_params service (was {"active" if before_active else "waiting"})')
        response.success = True
        response.message = (
            f'goal set on {self.active_surface_name} '
            f'uv=({self.goal_surface_axis:.2f}, '
            f'{math.degrees(self.goal_surface_theta):.1f} deg)')
        return response

    def _pose_goal_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._set_goal_from_world_point(p.x, p.y, p.z, '2D Goal Pose')

    # ------------------------------------------------------------------
    # Surface helpers (polygon-aware)
    # ------------------------------------------------------------------
    def _active_surface(self) -> Surface:
        return self.surface_graph.surfaces.get(
            self.active_surface_name,
            next(iter(self.surface_graph.surfaces.values())))

    def _legacy_axis_value(self, x, y, z):
        if self.axis == 'x':
            return x
        if self.axis == 'y':
            return y
        return z

    def _legacy_axis_vector(self):
        if self.axis == 'x':
            return 1.0, 0.0, 0.0
        if self.axis == 'y':
            return 0.0, 1.0, 0.0
        return 0.0, 0.0, 1.0

    def _legacy_point_for(self, axis_value, theta):
        ax, ay, az = self.axis_point
        if self.axis == 'x':
            return (axis_value,
                    ay + self.radius * math.cos(theta),
                    az + self.radius * math.sin(theta))
        if self.axis == 'y':
            return (ax + self.radius * math.sin(theta),
                    axis_value,
                    az + self.radius * math.cos(theta))
        return (ax + self.radius * math.cos(theta),
                ay + self.radius * math.sin(theta),
                axis_value)

    def _legacy_theta_for(self, x, y, z):
        ax, ay, az = self.axis_point
        if self.axis == 'x':
            return math.atan2(z - az, y - ay)
        if self.axis == 'y':
            return math.atan2(x - ax, z - az)
        return math.atan2(y - ay, x - ax)

    def _rebuild_default_goal(self):
        surface = self._active_surface()
        if isinstance(surface, CylinderSurface):
            self.target_projection = surface.project(
                surface.point_at(self.target_axis, self.target_theta))
        else:
            # Use the surface centroid as the default goal for polygon
            # surfaces when no specific (u, v) is configured.
            self.target_projection = surface.project(
                surface.point_at(
                    0.5 * (surface.clamp_uv(0.0, 0.0)[0] +
                           surface.clamp_uv(1.0, 0.0)[0]),
                    0.5 * (surface.clamp_uv(0.0, 0.0)[1] +
                           surface.clamp_uv(0.0, 1.0)[1])))
        self.goal_projection = self.target_projection
        self.goal_surface_axis = self.target_projection.u
        self.goal_surface_theta = self.target_projection.v
        self.route_points = [
            RoutePoint(
                self.target_projection.surface,
                self.target_projection.u,
                self.target_projection.v,
                self.target_projection.point)
        ]

    def _clear_route_and_goal(self, reason: str):
        self.route_points = []
        self.route_index = 0
        self.target_projection = None
        self.goal_projection = None
        self.reached = False
        self.goal_active = not self.wait_for_goal
        if self.goal_active:
            self._rebuild_default_goal()
        self.get_logger().info(
            f'route and goal reset: {reason}; '
            f'goal_active={self.goal_active}')

    def _set_goal_from_world_point(self, x, y, z, source):
        goal_projection = self.surface_graph.closest_projection((x, y, z))
        if goal_projection is None:
            return

        previous_surface = self.active_surface_name
        self.goal_projection = goal_projection
        self.active_surface_name = goal_projection.surface.name
        if previous_surface != self.active_surface_name:
            self.contact_ready_since = None
            self.contact_settled = False

        if goal_projection.distance > 0.30:
            self.get_logger().warn(
                f'Goal point is {goal_projection.distance:.2f} m from the '
                f'closest surface ({goal_projection.surface.name}); '
                f'projecting to surface.',
                throttle_duration_sec=1.0)

        surface_axis = goal_projection.u
        surface_theta = goal_projection.v
        self.goal_surface_axis = surface_axis
        self.goal_surface_theta = surface_theta
        self.goal_wheel_name = 'base_center'
        self.goal_wheel_error_m = 0.0
        if (self.wheel_goal_projection and
                isinstance(goal_projection.surface, CylinderSurface)):
            (self.target_axis, self.target_theta, self.goal_wheel_name,
             self.goal_wheel_error_m) = self._base_goal_for_wheel_goal(
                goal_projection.surface, surface_axis, surface_theta)
            tx, ty, tz = goal_projection.surface.point_at(
                self.target_axis, self.target_theta)
            self.target_projection = goal_projection.surface.project(
                (tx, ty, tz))
        else:
            self.target_projection = goal_projection
        self._build_route()
        self.reached = False
        self.goal_active = True
        self._publish_markers()
        self.get_logger().info(
            f'New surface goal from {source}: surface='
            f'{self.active_surface_name}, '
            f'clicked uv=({surface_axis:.2f}, '
            f'{math.degrees(surface_theta):.1f} deg), '
            f'{self.goal_wheel_name} target gives base uv='
            f'({self.target_projection.u:.2f}, '
            f'{math.degrees(self.target_projection.v):.1f} deg), '
            f'wheel reach error={self.goal_wheel_error_m:.3f} m')

    def _build_route(self):
        self.route_points = []
        self.route_index = 0
        if self.target_projection is None:
            return
        pos = self._robot_position()
        if pos is None:
            self.route_points = [
                RoutePoint(
                    self.target_projection.surface,
                    self.target_projection.u,
                    self.target_projection.v,
                    self.target_projection.point)
            ]
            return
        start = self.surface_graph.closest_projection(pos)
        if start is None:
            return
        self.route_points = self.surface_graph.route_points(
            start, self.target_projection, self.route_samples_per_surface)

    def _base_goal_for_wheel_goal(
            self, surface: CylinderSurface, surface_axis, surface_theta):
        axis_center = surface.axis_value(surface.axis_point)
        half_len = 0.5 * surface.length
        arc_radius = max(surface.radius + self.wheel_radius, 1e-3)
        current_axis = None
        current_theta = None
        pos = self._robot_position()
        if pos is not None:
            current_axis = surface.axis_value(pos)
            current_theta = surface.theta_for(pos)

        best = None
        for name, tangent_offset, axis_offset in self.wheel_offsets:
            base_axis = _clamp(
                surface_axis + axis_offset,
                axis_center - half_len,
                axis_center + half_len)
            base_theta = _wrap_pi(
                surface_theta + tangent_offset / arc_radius)
            achieved_axis = base_axis - axis_offset
            achieved_theta = _wrap_pi(
                base_theta - tangent_offset / arc_radius)
            wheel_error = math.hypot(
                achieved_axis - surface_axis,
                arc_radius * _wrap_pi(achieved_theta - surface_theta))
            if current_axis is None or current_theta is None:
                score = math.hypot(tangent_offset, axis_offset)
            else:
                score = (
                    abs(base_axis - current_axis) +
                    arc_radius * abs(
                        _wrap_pi(base_theta - current_theta)))
            score += 10.0 * wheel_error
            candidate = (score, base_axis, base_theta, name, wheel_error)
            if best is None or candidate[0] < best[0]:
                best = candidate

        if best is None:
            return surface_axis, surface_theta, 'base_center', 0.0
        _, base_axis, base_theta, name, wheel_error = best
        return base_axis, base_theta, name, wheel_error

    # ------------------------------------------------------------------
    # ROS infrastructure
    # ------------------------------------------------------------------
    def _robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot read {self.world_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return t.x, t.y, t.z

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot read {self.world_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def _route_target(self, current: SurfaceProjection):
        if not self.route_points:
            if self.target_projection is None:
                return None
            return RoutePoint(
                self.target_projection.surface,
                self.target_projection.u,
                self.target_projection.v,
                self.target_projection.point)

        self.route_index = min(self.route_index, len(self.route_points) - 1)
        for index in range(self.route_index, len(self.route_points)):
            point = self.route_points[index]
            if point.surface.name == current.surface.name:
                self.route_index = index
                return point
        return self.route_points[self.route_index]

    def _edge_motion_scale(self, base_position):
        if self.edge_margin <= 1e-9 or self.current_projection is None:
            self.edge_clearance_m = float('inf')
            self.edge_motion_scale = 1.0
            return 1.0

        pose = self._robot_pose()
        if pose is None:
            self.edge_clearance_m = self.current_projection.edge_clearance
        else:
            (bx, by, bz), quat = pose
            self.edge_clearance_m = footprint_edge_clearance(
                self.current_projection.surface, (bx, by, bz),
                quat, self.wheel_offsets)

        self.edge_motion_scale = edge_motion_scale(
            self.edge_clearance_m, self.edge_margin, self.edge_slow_min_scale)
        return self.edge_motion_scale

    def _world_to_base(self, vx, vy, vz):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.world_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {self.world_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        return _quat_rotate(q.x, q.y, q.z, q.w, vx, vy, vz)

    def _status_stale(self):
        if self.last_status_t is None:
            return True
        age = (self.get_clock().now() - self.last_status_t).nanoseconds * 1e-9
        return age > self.status_timeout

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def _tick(self):
        if not self.goal_active:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'waiting for goal')
            return
        if self.reached:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'goal reached')
            return

        pos = self._robot_position()
        if pos is None:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'no TF')
            return
        rx, ry, rz = pos

        contact_ready = (
            not self._status_stale() and
            self.attached >= self.min_attached)
        now = self.get_clock().now()
        if contact_ready:
            if self.contact_ready_since is None:
                self.contact_ready_since = now
            settled_s = (
                now - self.contact_ready_since).nanoseconds * 1e-9
            self.contact_settled = settled_s >= self.attachment_settle_s
        else:
            self.contact_ready_since = None
            self.contact_settled = False
        boundary_hold = (
            contact_ready and self.boundary_risk >= self.boundary_hold_risk)

        current_projection = self.surface_graph.closest_projection(
            (rx, ry, rz))
        if current_projection is None:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'no surface projection')
            return
        self.active_surface_name = current_projection.surface.name
        self.current_projection = current_projection

        if not contact_ready:
            self._approach_surface(rx, ry, rz, current_projection)
            return
        if not self.contact_settled:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'settling adhesion')
            return
        if boundary_hold and not self.allow_guarded_low_contact:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'boundary hold')
            return

        current_safety = self._traction_safety(
            projection=current_projection)
        target_safety = self._traction_safety(
            projection=self.target_projection)
        self.last_safety = {
            'current': current_safety,
            'target': target_safety,
        }
        if not current_safety['safe']:
            self._publish_body_cmd(
                0.0, 0.0, 0.0,
                f'unsafe current: {current_safety["reason"]}')
            return
        if not target_safety['safe']:
            self._publish_body_cmd(
                0.0, 0.0, 0.0,
                f'unsafe target: {target_safety["reason"]}')
            return

        route_target = self._route_target(current_projection)
        if route_target is None:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'no route target')
            return

        goal_reached = self._goal_reached(
            current_projection, self.target_projection)
        if goal_reached:
            self.reached = True
            self._publish_body_cmd(0.0, 0.0, 0.0, 'goal reached')
            self.get_logger().info('Surface goal reached. Holding.')
            return

        tangent_u, tangent_v, _ = current_projection.surface.tangents_at(
            current_projection.u, current_projection.v)
        target_projection = current_projection.surface.project(
            route_target.point)
        u_err, v_err = current_projection.surface.motion_vector(
            (current_projection.u, current_projection.v),
            (target_projection.u, target_projection.v))
        v_scale = (
            current_projection.surface.radius
            if isinstance(current_projection.surface, CylinderSurface)
            else 1.0)
        lateral_tolerance = (
            self.radius * self.angle_tol
            if isinstance(current_projection.surface, CylinderSurface)
            else self.axis_tol)
        if (current_projection.surface.distance_between(
                (current_projection.u, current_projection.v),
                (target_projection.u, target_projection.v)) <=
                max(self.axis_tol, lateral_tolerance) and
                self.route_index < len(self.route_points) - 1):
            self.route_index += 1

        axis_speed = _clamp(
            self.kp_axis * u_err, -self.axis_speed, self.axis_speed)
        curve_speed = _clamp(
            self.kp_theta * v_err * v_scale,
            -self.curve_speed, self.curve_speed)
        edge_scale = self._edge_motion_scale((rx, ry, rz))
        wx = edge_scale * (
            axis_speed * tangent_u[0] + curve_speed * tangent_v[0])
        wy = edge_scale * (
            axis_speed * tangent_u[1] + curve_speed * tangent_v[1])
        wz = edge_scale * (
            axis_speed * tangent_u[2] + curve_speed * tangent_v[2])
        self._publish_world_cmd(wx, wy, wz, 'surface_graph')

    def _goal_reached(self, current_projection, target_projection):
        if target_projection is None:
            return False
        if (current_projection.surface.name !=
                target_projection.surface.name):
            return False
        if isinstance(target_projection.surface, CylinderSurface):
            tolerance = max(self.axis_tol, self.radius * self.angle_tol)
        else:
            tolerance = self.axis_tol
        return current_projection.surface.distance_between(
            (current_projection.u, current_projection.v),
            (target_projection.u, target_projection.v)) <= tolerance

    def _approach_surface(self, rx, ry, rz, current_projection):
        surface = current_projection.surface
        if not isinstance(surface, CylinderSurface):
            target = self.target_projection or current_projection
            sx = target.point[0] + 0.10 * target.normal[0]
            sy = target.point[1] + 0.10 * target.normal[1]
            sz = target.point[2] + 0.10 * target.normal[2]
            dx, dy, dz = sx - rx, sy - ry, sz - rz
            mag = math.sqrt(dx * dx + dy * dy + dz * dz)
            if mag > 1e-6:
                dx, dy, dz = dx / mag, dy / mag, dz / mag
            self._publish_world_cmd(
                self.approach_speed * dx,
                self.approach_speed * dy,
                self.approach_speed * dz,
                'approach_graph')
            return

        target_axis = _clamp(
            self.target_axis,
            self.approach_axis - 0.5 * self.length,
            self.approach_axis + 0.5 * self.length)
        sx, sy, sz = self._legacy_point_for(
            target_axis, self.approach_theta)
        ax, ay, az = self._legacy_axis_vector()
        axis_err = self.target_axis - self._legacy_axis_value(rx, ry, rz)
        axis_speed = _clamp(
            self.kp_axis * axis_err, -self.axis_speed, self.axis_speed)
        dx, dy, dz = sx - rx, sy - ry, sz - rz
        axial_component = dx * ax + dy * ay + dz * az
        dx -= axial_component * ax
        dy -= axial_component * ay
        dz -= axial_component * az
        mag = math.sqrt(dx * dx + dy * dy + dz * dz)
        if mag > 1e-6:
            dx, dy, dz = dx / mag, dy / mag, dz / mag
        wx = axis_speed * ax + self.approach_speed * dx
        wy = axis_speed * ay + self.approach_speed * dy
        wz = axis_speed * az + self.approach_speed * dz
        self._publish_world_cmd(wx, wy, wz, 'approach')

    def _publish_world_cmd(self, wx, wy, wz, mode):
        body = self._world_to_base(wx, wy, wz)
        if body is None:
            self._publish_body_cmd(0.0, 0.0, 0.0, 'no transform')
            return
        self._publish_body_cmd(body[0], body[1], 0.0, mode)

    # ------------------------------------------------------------------
    # Safety / adhesion
    # ------------------------------------------------------------------
    def _projection_tangential_load(self, projection):
        if projection is None:
            return 0.0, None
        nz = projection.normal[2]
        load_fraction = math.sqrt(max(0.0, 1.0 - nz * nz))
        return (self.robot_mass * self.gravity * load_fraction,
                load_fraction)

    def _traction_safety(self, projection=None, theta=None):
        weight = self.robot_mass * self.gravity
        load_fraction_override = None
        if projection is not None:
            required, load_fraction_override = (
                self._projection_tangential_load(projection))
        elif theta is not None:
            axis = self._legacy_axis_vector()
            ax_z = axis[2]
            theta_tan = (math.cos(theta), math.sin(theta), 0.0)
            if self.axis == 'x':
                tan_z = 0.0
            else:
                tan_z = theta_tan[2]
            load_fraction = math.sqrt(
                max(0.0, 1.0 - ax_z * ax_z - tan_z * tan_z))
            required = self.robot_mass * self.gravity * load_fraction
        else:
            required = 0.0
        traction_capacity = self.friction_mu * self.sum_force_n
        torque_capacity = (
            self.attached * self.wheel_torque_limit / self.wheel_radius
            if self.wheel_torque_limit > 0.0 else float('inf'))
        capacity = min(traction_capacity, torque_capacity)
        required_with_sf = self.safety_factor * required
        if required_with_sf <= 1e-6:
            margin = float('inf')
        else:
            margin = capacity / required_with_sf
        load_fraction = (
            load_fraction_override if load_fraction_override is not None
            else required / weight if weight > 1e-9 else 0.0)
        steep = load_fraction >= self.steep_load_fraction
        contact_ok = (
            not steep or
            self.avg_contact_fraction >= self.min_side_contact_fraction)
        force_ok = (
            required_with_sf <= 1e-6 or
            margin >= 1.0)
        attached_ok = self.attached >= self.min_attached
        guarded_low_contact = (
            self.allow_guarded_low_contact and steep and attached_ok and
            not contact_ok)
        if not attached_ok:
            reason = 'not enough attached wheels'
        elif guarded_low_contact and force_ok:
            reason = 'guarded low-contact motion'
        elif not contact_ok:
            reason = 'partial wheel contact on steep surface'
        elif not force_ok:
            reason = 'low traction/torque margin'
        else:
            reason = 'ok'
        return {
            'safe': bool(attached_ok and (contact_ok or guarded_low_contact) and force_ok),
            'reason': reason,
            'guarded_low_contact': guarded_low_contact,
            'allow_guarded_low_contact': self.allow_guarded_low_contact,
            'surface': projection.surface.name if projection is not None else None,
            'required_tangential_n': required,
            'required_with_safety_n': required_with_sf,
            'normal_force_sum_n': self.sum_force_n,
            'traction_capacity_n': traction_capacity,
            'torque_capacity_n': torque_capacity,
            'margin': margin,
            'load_fraction': load_fraction,
            'avg_contact_fraction': self.avg_contact_fraction,
            'min_side_contact_fraction': self.min_side_contact_fraction,
        }

    def _steering_for_body_direction(self, vx, vy):
        if math.hypot(vx, vy) <= 1e-9:
            return [0.0, 0.0, 0.0]
        rolling_dir = math.atan2(vy, vx)
        out = []
        for alpha in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
            angle = _wrap_pi(alpha - rolling_dir + math.pi / 2.0)
            if abs(angle) > math.pi / 2.0:
                angle = _wrap_pi(angle + math.pi)
            out.append(angle)
        return out

    def _publish_hold_steering(self, surface_hold):
        msg = Float64MultiArray()
        if surface_hold:
            body = self._world_to_base(*self._parking_direction())
            if body is not None:
                msg.data = self._steering_for_body_direction(
                    body[0], body[1])
            else:
                msg.data = [0.0, 0.0, 0.0]
        else:
            msg.data = [0.0, 0.0, 0.0]
        self.hold_steering_pub.publish(msg)

    def _parking_direction(self):
        if self.current_projection is not None:
            tangent_u = self.current_projection.tangent_u
            tangent_v = self.current_projection.tangent_v
            if abs(tangent_v[2]) < abs(tangent_u[2]):
                return tangent_v
            return tangent_u
        axis_dir = self._legacy_axis_vector()
        pos = self._robot_position()
        if pos is None:
            return axis_dir
        # Park in the tangent direction with the smallest vertical component.
        surface = self._active_surface()
        if isinstance(surface, CylinderSurface):
            theta = self._legacy_theta_for(*pos)
            _, tangent, _ = surface.tangents_at(
                self._legacy_axis_value(*self.axis_point), theta)
        else:
            projection = surface.project(pos)
            _, tangent, _ = surface.tangents_at(projection.u, projection.v)
        if abs(tangent[2]) < abs(axis_dir[2]):
            return tangent
        return axis_dir

    def _publish_body_cmd(self, x, y, wz, mode):
        zero_cmd = (
            abs(x) <= 1e-9 and abs(y) <= 1e-9 and abs(wz) <= 1e-9)
        if zero_cmd:
            surface_hold = (
                self.attached >= self.min_attached and
                mode not in ('waiting for goal', 'no TF', 'no transform'))
            self._publish_hold_steering(surface_hold)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

        dbg = String()
        dbg.data = json.dumps({
            'planner': 'surface_goal_planner',
            'mode': mode,
            'surface_mode': self.surface_mode,
            'active_surface': self.active_surface_name,
            'surface_count': len(self.surface_graph.surfaces),
            'surface_graph_yaml': self.surface_graph_yaml,
            'goal_active': self.goal_active,
            'attached': self.attached,
            'pipe_count': self.pipe_count,
            'dominant_surface': self.dominant_surface,
            'surface_counts': self.surface_counts,
            'boundary_risk': self.boundary_risk,
            'boundary_state': self.boundary_state,
            'sum_force_n': self.sum_force_n,
            'avg_contact_fraction': self.avg_contact_fraction,
            'safety': self.last_safety,
            'contact_settled': self.contact_settled,
            'attachment_settle_s': self.attachment_settle_s,
            'target_uv': [
                self.target_projection.u if self.target_projection else None,
                self.target_projection.v if self.target_projection else None,
            ],
            'published_goal_uv': [
                self.goal_projection.u if self.goal_projection else None,
                self.goal_projection.v if self.goal_projection else None,
            ],
            'goal_wheel': self.goal_wheel_name,
            'goal_wheel_reach_error_m': self.goal_wheel_error_m,
            'wheel_goal_projection': self.wheel_goal_projection,
            'wheel_steering_radius_m': self.wheel_steering_radius,
            'wheel_radius_m': self.wheel_radius,
            'route_index': self.route_index,
            'route_point_count': len(self.route_points),
            'route_algorithm': self.surface_graph.last_route.algorithm,
            'route_path': self.surface_graph.last_route.path,
            'route_cost': self.surface_graph.last_route.total_cost,
            'route_cost_components': self.surface_graph.last_route.components,
            'route_expanded_surfaces': self.surface_graph.last_route.expanded,
            'edge_clearance_m': self.edge_clearance_m,
            'edge_motion_scale': self.edge_motion_scale,
            'edge_margin_m': self.edge_margin,
            'cmd_body': [x, y, wz],
        })
        self.debug_pub.publish(dbg)
        if self.legacy_debug_pub is not None:
            self.legacy_debug_pub.publish(dbg)

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------
    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        if self.surface_graph_yaml or self.show_experimental_polygon:
            markers.markers.append(
                self._delete_marker(stamp, 'surface_goal_surface', 1))
            markers.markers.extend(self._surface_graph_markers(stamp))
        else:
            markers.markers.append(self._surface_marker(stamp))
            for marker_id in (101, 102, 103):
                markers.markers.append(
                    self._delete_marker(stamp, 'surface_goal_surface',
                                        marker_id))
        if self.goal_active:
            markers.markers.append(self._goal_marker(stamp))
            if self.wheel_goal_projection:
                markers.markers.append(self._base_goal_marker(stamp))
            else:
                markers.markers.append(
                    self._delete_marker(stamp, 'surface_goal', 4))
            route = self._route_marker(stamp)
            if route is not None:
                markers.markers.append(route)
            else:
                markers.markers.append(
                    self._delete_marker(stamp, 'surface_goal_route', 3))
        else:
            markers.markers.append(
                self._delete_marker(stamp, 'surface_goal', 2))
            markers.markers.append(
                self._delete_marker(stamp, 'surface_goal', 4))
            markers.markers.append(
                self._delete_marker(stamp, 'surface_goal_route', 3))
        self.marker_pub.publish(markers)

    def _delete_marker(self, stamp, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _surface_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'surface_goal_surface'
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.012
        marker.color.r = 0.52
        marker.color.g = 0.62
        marker.color.b = 0.66
        marker.color.a = 0.75
        axis_center = self._legacy_axis_value(*self.axis_point)
        half_len = 0.5 * self.length
        ring_count = 9
        segment_count = 48
        for ring_i in range(ring_count):
            axis_value = (
                axis_center - half_len +
                self.length * ring_i / max(1, ring_count - 1))
            for seg_i in range(segment_count):
                t0 = 2.0 * math.pi * seg_i / segment_count
                t1 = 2.0 * math.pi * (seg_i + 1) / segment_count
                p0 = self._legacy_point_for(axis_value, t0)
                p1 = self._legacy_point_for(axis_value, t1)
                marker.points.append(Point(x=p0[0], y=p0[1], z=p0[2]))
                marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
        for seg_i in range(0, segment_count, 4):
            theta = 2.0 * math.pi * seg_i / segment_count
            p0 = self._legacy_point_for(axis_center - half_len, theta)
            p1 = self._legacy_point_for(axis_center + half_len, theta)
            marker.points.append(Point(x=p0[0], y=p0[1], z=p0[2]))
            marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
        return marker

    def _polygon_cylinder_marker(
            self, stamp, marker_id, x, y, z, radius, length, axis):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'surface_goal_surface'
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
        marker.color.a = 0.86
        return marker

    def _polygon_face_marker(self, stamp, marker_id, surface):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'surface_goal_surface'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.018
        if surface.name == self.active_surface_name:
            marker.color.r = 0.0
            marker.color.g = 0.85
            marker.color.b = 1.0
            marker.color.a = 0.95
        else:
            marker.color.r = 0.72
            marker.color.g = 0.52
            marker.color.b = 0.22
            marker.color.a = 0.85
        for i, p0 in enumerate(surface.vertices3):
            p1 = surface.vertices3[(i + 1) % len(surface.vertices3)]
            marker.points.append(Point(x=p0[0], y=p0[1], z=p0[2]))
            marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
        return marker

    def _polygon_cylinder_grid_marker(self, stamp, marker_id, surface):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'surface_goal_surface'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.014
        if surface.name == self.active_surface_name:
            marker.color.r = 0.0
            marker.color.g = 0.85
            marker.color.b = 1.0
            marker.color.a = 0.95
        else:
            marker.color.r = 0.90
            marker.color.g = 0.76
            marker.color.b = 0.16
            marker.color.a = 0.85

        radius = surface.radius + 0.01
        axis_center = self._cylinder_axis_value(
            surface.axis, *surface.axis_point)
        half_len = 0.5 * surface.length
        ring_count = 9
        segment_count = 48

        for ring_i in range(ring_count):
            axis_value = (
                axis_center - half_len +
                surface.length * ring_i / max(1, ring_count - 1))
            for seg_i in range(segment_count):
                t0 = 2.0 * math.pi * seg_i / segment_count
                t1 = 2.0 * math.pi * (seg_i + 1) / segment_count
                p0 = self._cylinder_point(
                    surface.axis, surface.axis_point, radius, axis_value, t0)
                p1 = self._cylinder_point(
                    surface.axis, surface.axis_point, radius, axis_value, t1)
                marker.points.append(Point(x=p0[0], y=p0[1], z=p0[2]))
                marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))

        for seg_i in range(0, segment_count, 4):
            theta = 2.0 * math.pi * seg_i / segment_count
            p0 = self._cylinder_point(
                surface.axis, surface.axis_point, radius,
                axis_center - half_len, theta)
            p1 = self._cylinder_point(
                surface.axis, surface.axis_point, radius,
                axis_center + half_len, theta)
            marker.points.append(Point(x=p0[0], y=p0[1], z=p0[2]))
            marker.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
        return marker

    @staticmethod
    def _cylinder_point(axis, axis_point, radius, axis_value, theta):
        ax, ay, az = axis_point
        if axis == 'x':
            return (
                axis_value,
                ay + radius * math.cos(theta),
                az + radius * math.sin(theta))
        if axis == 'y':
            return (
                ax + radius * math.sin(theta),
                axis_value,
                az + radius * math.cos(theta))
        return (
            ax + radius * math.cos(theta),
            ay + radius * math.sin(theta),
            axis_value)

    @staticmethod
    def _cylinder_axis_value(axis, x, y, z):
        if axis == 'x':
            return x
        if axis == 'y':
            return y
        return z

    def _surface_graph_markers(self, stamp):
        markers = []
        marker_id = 101
        for surface in self.surface_graph.surfaces.values():
            if isinstance(surface, CylinderSurface):
                x, y, z = surface.axis_point
                markers.append(self._polygon_cylinder_marker(
                    stamp, marker_id, x, y, z,
                    surface.radius, surface.length, surface.axis))
                markers.append(self._polygon_cylinder_grid_marker(
                    stamp, marker_id + 100, surface))
            elif isinstance(surface, PolygonSurface):
                markers.append(
                    self._polygon_face_marker(stamp, marker_id, surface))
            marker_id += 1
        for old_id in range(marker_id, 116):
            markers.append(self._delete_marker(
                stamp, 'surface_goal_surface', old_id))
        for old_id in range(marker_id + 100, 216):
            markers.append(self._delete_marker(
                stamp, 'surface_goal_surface', old_id))
        return markers

    def _goal_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'surface_goal'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        if self.goal_projection is not None:
            x, y, z = self.goal_projection.point
        elif isinstance(self._active_surface(), CylinderSurface):
            x, y, z = self._legacy_point_for(
                self.goal_surface_axis, self.goal_surface_theta)
        else:
            x, y, z = (0.0, 0.0, 0.0)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        return marker

    def _base_goal_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = 'surface_goal'
        marker.id = 4
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        if self.target_projection is not None:
            x, y, z = self.target_projection.point
        else:
            x, y, z = (0.0, 0.0, 0.0)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.82
        marker.color.b = 0.12
        marker.color.a = 0.95
        return marker

    def _route_marker(self, stamp):
        if self.route_points:
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns = 'surface_goal_route'
            marker.id = 3
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.35
            marker.color.a = 0.95
            for route_point in self.route_points:
                x, y, z = route_point.point
                marker.points.append(Point(x=x, y=y, z=z))
            return marker

        if isinstance(self._active_surface(), CylinderSurface):
            pos = self._robot_position()
            if pos is None:
                return None
            rx, ry, rz = pos
            start_axis = self._legacy_axis_value(rx, ry, rz)
            start_theta = self._legacy_theta_for(rx, ry, rz)
            theta_err = _wrap_pi(self.target_theta - start_theta)
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns = 'surface_goal_route'
            marker.id = 3
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.35
            marker.color.a = 0.95
            for i in range(25):
                ratio = i / 24.0
                theta = start_theta + theta_err * ratio
                axis_value = (start_axis +
                              (self.target_axis - start_axis) * ratio)
                x, y, z = self._legacy_point_for(axis_value, theta)
                marker.points.append(Point(x=x, y=y, z=z))
            return marker
        return None


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceGoalPlanner()
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
