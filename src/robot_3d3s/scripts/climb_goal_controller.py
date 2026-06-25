#!/usr/bin/env python3
"""climb_goal_controller.py — autonomous "click a point, climb to it" controller,
FOOTPRINT-AWARE / edge-safe.

Pick a goal with RViz "Publish Point" on the wall (or ramp), or publish a
PoseStamped surface goal on /goal_pose where x=surface progress, y=lateral
offset, theta=surface yaw. The robot plans staged surface-pose waypoints, drives
floor -> ramp -> wall, and STOPS there without falling and WITHOUT driving any
wheel off the finite surface.

Why not Nav2: the climb path is a folded 3-D surface (ground -> slide -> wall),
not a flat 2-D costmap. We exploit that the whole course lies in the X-Z plane
(every surface is a rotation about world Y), so:

  * body +X is always "forward = up the surface",
  * body +Y is always world +Y (lateral),

and "navigate to a point" becomes 1-D surface-progress control + lateral control.

Edge safety (the point of this controller)
------------------------------------------
The robot is NOT a point: its three wheels sit on a ring of radius ~circumradius.
The slide is only 1.2 m wide while the wall is 3.0 m wide, so a base path that is
"on the wall" can still push a rear wheel off the *narrow slide*. We therefore:

  1. compute the 3 wheel contact points from the base pose every tick,
  2. map each to its surface facet and that facet's real lateral half-width,
  3. constrain the commanded velocity with a per-edge "virtual wall" so NO wheel
     is ever driven past a slide/wall edge (with a margin), while still allowing
     motion AWAY from a violated edge (recovery),
  4. clamp the goal so the whole footprint fits at the target.

No-fall margins (from Jia et al. 2025, Applied Ocean Research, Sec. 3)
---------------------------------------------------------------------
Using the per-wheel magnetic force_n + attached flags in the adhesion status, we
also enforce the static failure conditions and HOLD if a margin is low:
  * friction / longitudinal slip :  mu * sum(N) >= sf * m g
  * peel / longitudinal capsize  :  sum(N)      >= sf * m g * com_standoff/contact_span
Keeping all 3 wheels on-surface (above) preserves the full support triangle, which
is what prevents the lateral-capsize trigger geometrically.

Inputs: scene YAML (geometry, mass, mu), /robot_3d3s/adhesion_status (base_pose +
per-wheel attached/force_n), /clicked_point (goal, world frame).
Output: TwistStamped on /swerve_controller/cmd_vel.
"""

import json
import math

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

# Body-frame wheel ring: 3 legs at 120 deg. Radius = contact circumradius
# (overridden from scene 'robot.circumradius' when available).
_LEG_ANGLES = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _quat_rotate(q, v):
    """Rotate vector v by unit quaternion q=(x,y,z,w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _quat_inv_rotate(q, v):
    """Rotate v by the inverse (conjugate) of unit quaternion q."""
    x, y, z, w = q
    return _quat_rotate((-x, -y, -z, w), v)


def _yaw_from_quat(q):
    """Planar yaw from quaternion q=(x,y,z,w)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class ClimbGoalController(Node):
    def __init__(self):
        super().__init__('climb_goal_controller')

        self.declare_parameter('scene_yaml', '')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('cmd_out_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('v_climb', 0.12)        # m/s along the surface
        self.declare_parameter('v_lat', 0.06)          # m/s lateral
        self.declare_parameter('kp_s', 0.6)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('kp_yaw', 1.2)          # heading-align gain
        self.declare_parameter('wz_max', 0.5)          # rad/s yaw rate cap
        self.declare_parameter('tol_s', 0.05)          # m, surface-progress
        self.declare_parameter('tol_y', 0.04)          # m, lateral
        self.declare_parameter('tol_theta', 0.10)      # rad, surface yaw
        self.declare_parameter('min_attached_wheels', 2)
        self.declare_parameter('status_timeout_s', 0.5)
        self.declare_parameter('wall_top_margin_m', 0.30)   # keep below top edge
        self.declare_parameter('region_margin_m', 0.15)
        # --- edge safety ---
        self.declare_parameter('edge_margin_m', 0.12)       # keep wheels this far in
        self.declare_parameter('edge_slow_gain', 1.5)       # 1/s barrier slowdown
        self.declare_parameter('footprint_radius_m', 0.0)   # 0 => read scene/0.414
        # --- surface planner ---
        self.declare_parameter('wall_settle_margin_m', 0.08)
        self.declare_parameter('goal_pose_is_surface_pose', True)
        self.declare_parameter('clicked_point_theta', 0.0)
        self.declare_parameter('straight_transition_drive', True)
        # --- motion quality / transition tracking ---
        self.declare_parameter('motion_smoothing', True)
        self.declare_parameter('accel_limit_mps2', 0.12)
        self.declare_parameter('yaw_accel_limit_radps2', 0.45)
        self.declare_parameter('transition_slow_band_m', 0.28)
        self.declare_parameter('transition_speed_scale', 0.45)
        self.declare_parameter('gap_slow_start_mm', 25.0)
        self.declare_parameter('gap_stop_mm', 55.0)
        self.declare_parameter('low_force_slow_n', 300.0)
        self.declare_parameter('min_motion_scale', 0.20)
        # --- no-fall (slip/peel) margins ---
        self.declare_parameter('mass_kg', 8.715)
        self.declare_parameter('mu', 1.0)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('com_standoff_m', 0.18)
        self.declare_parameter('contact_span_m', 0.62)
        self.declare_parameter('safety_factor', 1.5)
        self.declare_parameter('force_guard', True)         # enable slip/peel hold

        gp = self.get_parameter
        scene_path = str(gp('scene_yaml').value)
        self.base_frame = str(gp('base_frame').value)
        self.publish_hz = max(2.0, float(gp('publish_hz').value))
        self.v_climb = float(gp('v_climb').value)
        self.v_lat = float(gp('v_lat').value)
        self.kp_s = float(gp('kp_s').value)
        self.kp_y = float(gp('kp_y').value)
        self.kp_yaw = float(gp('kp_yaw').value)
        self.wz_max = float(gp('wz_max').value)
        self.tol_s = float(gp('tol_s').value)
        self.tol_y = float(gp('tol_y').value)
        self.tol_theta = float(gp('tol_theta').value)
        self.min_attached = int(gp('min_attached_wheels').value)
        self.status_timeout = float(gp('status_timeout_s').value)
        self.wall_top_margin = float(gp('wall_top_margin_m').value)
        self.region_margin = float(gp('region_margin_m').value)
        self.edge_margin = float(gp('edge_margin_m').value)
        self.edge_gain = max(0.1, float(gp('edge_slow_gain').value))
        self.mass = float(gp('mass_kg').value)
        self.mu = float(gp('mu').value)
        self.g = float(gp('gravity').value)
        self.com_standoff = float(gp('com_standoff_m').value)
        self.contact_span = max(1e-3, float(gp('contact_span_m').value))
        self.sf = max(1.0, float(gp('safety_factor').value))
        self.force_guard = bool(gp('force_guard').value)
        self._fp_radius_param = float(gp('footprint_radius_m').value)
        self.wall_settle_margin = float(gp('wall_settle_margin_m').value)
        self.goal_pose_is_surface_pose = bool(gp('goal_pose_is_surface_pose').value)
        self.clicked_point_theta = float(gp('clicked_point_theta').value)
        self.straight_transition_drive = bool(gp('straight_transition_drive').value)
        self.motion_smoothing = bool(gp('motion_smoothing').value)
        self.accel_limit = max(0.01, float(gp('accel_limit_mps2').value))
        self.yaw_accel_limit = max(0.01, float(gp('yaw_accel_limit_radps2').value))
        self.transition_slow_band = max(
            0.01, float(gp('transition_slow_band_m').value))
        self.transition_speed_scale = _clamp(
            float(gp('transition_speed_scale').value), 0.05, 1.0)
        self.gap_slow_start_mm = max(0.0, float(gp('gap_slow_start_mm').value))
        self.gap_stop_mm = max(
            self.gap_slow_start_mm + 1.0, float(gp('gap_stop_mm').value))
        self.low_force_slow_n = max(0.0, float(gp('low_force_slow_n').value))
        self.min_motion_scale = _clamp(
            float(gp('min_motion_scale').value), 0.05, 1.0)

        self._load_geometry(scene_path)

        # Slip/peel: minimum total normal force the magnets must supply.
        self.weight = self.mass * self.g
        self.req_friction = self.weight / max(1e-6, self.mu)
        self.req_peel = self.weight * self.com_standoff / self.contact_span

        self.base_pose = None        # (x, y, z) world
        self.base_quat = None        # (x, y, z, w) world
        self.attached = 0
        self.sum_n = 0.0             # total attached magnetic normal force
        self.max_gap_mm = 0.0
        self.min_force_n = 0.0
        self.surface_counts = {'ground': 0, 'slide': 0, 'wall': 0}
        self.dominant_surface = 'none'
        self.last_status_t = None
        self.goal_pose = None        # (s_goal, y_goal, theta_goal)
        self.route = []              # staged surface-pose waypoints
        self.route_index = 0
        self.route_start = None
        self.route_wall_contact = None
        self.reached = False
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_t = None

        self.cmd_pub = self.create_publisher(
            TwistStamped, str(gp('cmd_out_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, '/climb_controller/debug', 10)
        self.create_subscription(
            String, str(gp('status_topic').value), self._status_cb, 10)
        self.create_subscription(
            PointStamped, str(gp('clicked_point_topic').value), self._goal_cb, 10)
        self.create_subscription(
            PoseStamped, str(gp('goal_pose_topic').value), self._pose_goal_cb, 10)
        self.create_timer(1.0 / self.publish_hz, self._tick)

        self.get_logger().info(
            'climb_goal_controller (edge-safe) ready. footprint r='
            f'{self.fp_radius:.3f} m, slide half-width={self.slide_y_half:.2f} m, '
            f'wall half-width={self.wall_y_half:.2f} m, edge margin='
            f'{self.edge_margin:.2f} m, transition slow band='
            f'{self.transition_slow_band:.2f} m. Pick a goal with RViz "Publish Point".')

    # ── surface geometry from the scene YAML ───────────────────────────────────
    def _load_geometry(self, scene_path):
        scene = {}
        if scene_path:
            try:
                with open(scene_path) as f:
                    scene = yaml.safe_load(f) or {}
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'scene_yaml read failed: {exc}')

        wall = scene.get('flat_wall', {})
        wp = [float(v) for v in wall.get('pose', [1.0, 0.0, 1.5])]
        ws = [float(v) for v in wall.get('size', [0.10, 3.0, 3.0])]
        face = str(wall.get('climb_face', '-x'))
        self.face_x = wp[0] - 0.5 * ws[0] if face != '+x' else wp[0] + 0.5 * ws[0]
        self.wall_y = wp[1]
        self.wall_y_half = 0.5 * ws[1]
        self.wall_top = wp[2] + 0.5 * ws[2]
        self.wall_bottom = wp[2] - 0.5 * ws[2]

        self.ground_z = float(scene.get('ground', {}).get('z', 0.0))

        robot = scene.get('robot', {})
        self.mass = float(robot.get('mass', self.mass))
        self.mu = float(wall.get('mu', self.mu))
        scene_r = float(robot.get('circumradius', 0.41405))
        self.fp_radius = self._fp_radius_param if self._fp_radius_param > 0 else scene_r

        slide = scene.get('slide')
        if isinstance(slide, dict):
            self.theta = math.radians(float(slide.get('angle_deg', 45.0)))
            self.slide_y_half = 0.5 * float(slide.get('width', 1.2))
            t = float(slide.get('thickness', 0.04))
            pose = slide.get('pose')
            explicit_length = slide.get('length')
            if isinstance(pose, list) and len(pose) >= 3 and explicit_length is not None:
                length = float(explicit_length)
                top_cx = float(pose[0]) - 0.5 * t * math.sin(self.theta)
                top_cz = float(pose[2]) + 0.5 * t * math.cos(self.theta)
                self.slide_start_x = top_cx - 0.5 * length * math.cos(self.theta)
                self.slide_start_z = top_cz - 0.5 * length * math.sin(self.theta)
                self.slide_wall_x = top_cx + 0.5 * length * math.cos(self.theta)
                self.top_z = top_cz + 0.5 * length * math.sin(self.theta)
                self.slide_len = length
            else:
                self.slide_start_x = float(slide.get('start_x', -0.15))
                self.slide_start_z = float(slide.get('start_z', 0.0))
                self.slide_wall_x = float(slide.get('wall_x', self.face_x))
                self.top_z = self.slide_start_z + \
                    (self.slide_wall_x - self.slide_start_x) * math.tan(self.theta)
                self.slide_len = math.hypot(
                    self.slide_wall_x - self.slide_start_x,
                    self.top_z - self.slide_start_z)
        else:
            # No ramp: wall starts at the ground.
            self.theta = 0.0
            self.slide_start_x = self.face_x
            self.slide_start_z = self.ground_z
            self.slide_wall_x = self.face_x
            self.slide_y_half = self.wall_y_half
            self.top_z = self.wall_bottom
            self.slide_len = 0.0

        # Wheel top edge = physical top of the wall in surface-progress.
        self.s_wall_top = self.slide_len + (self.wall_top - self.top_z)
        # Goal cap = top minus a safety margin (the BASE target, not the wheels).
        self.s_wall_max = self.s_wall_top - self.wall_top_margin
        # Base progress where a theta=0 footprint has fully entered the wall.
        self.s_wall_settle = min(
            self.s_wall_max,
            self.slide_len + self.fp_radius + self.edge_margin + self.wall_settle_margin)
        # The largest lateral half-width any wheel could need (ground is open).
        self.ground_y_half = 1.0e3

    def _facet_halfwidth_for_s(self, s):
        if s < 0.0:
            return self.ground_y_half
        if s <= self.slide_len + self.region_margin:
            return self.slide_y_half
        return self.wall_y_half

    def _safe_y_limit_for_s(self, s):
        half = self._facet_halfwidth_for_s(s)
        return max(0.0, half - self.fp_radius - self.edge_margin)

    def _clamp_surface_pose(self, s, y, theta):
        s = _clamp(s, -1.0e3, self.s_wall_max)
        y_lim = self._safe_y_limit_for_s(s)
        return s, _clamp(y, self.wall_y - y_lim, self.wall_y + y_lim), _wrap_pi(theta)

    def _world_to_progress(self, x, y, z):
        """Map a world point to (s, lateral_y, up_dir, lateral_halfwidth).

        up_dir is the unit "up the surface" tangent in WORLD coords (the +s
        direction). lateral_halfwidth is the |y| limit of the facet the point is
        on (slide is narrow, wall is wide, ground is open).
        """
        # Wall: above the ramp top.
        if z >= self.top_z - self.region_margin and \
                x >= self.face_x - (0.5 + self.region_margin):
            return (self.slide_len + (z - self.top_z), y, (0.0, 0.0, 1.0),
                    self.wall_y_half)
        # Slide: between ramp foot and wall, below the top.
        if x >= self.slide_start_x - self.region_margin and self.slide_len > 0.0:
            d = (x - self.slide_start_x) * math.cos(self.theta) + \
                (z - self.slide_start_z) * math.sin(self.theta)
            up = (math.cos(self.theta), 0.0, math.sin(self.theta))
            return _clamp(d, 0.0, self.slide_len), y, up, self.slide_y_half
        # Ground: before the ramp foot.
        return (x - self.slide_start_x), y, (1.0, 0.0, 0.0), self.ground_y_half

    def _looks_like_wall_point(self, x, y, z):
        return (
            abs(x - self.face_x) <= 0.35 and
            abs(y - self.wall_y) <= self.wall_y_half + self.region_margin and
            self.wall_bottom - self.region_margin <= z <=
            self.wall_top + self.region_margin)

    def _wall_progress_from_z(self, z):
        # Keep wall clicks close to the ramp corner on the wall branch of the
        # folded surface; otherwise the scalar progress overlaps with the slide.
        return max(
            self.slide_len + self.region_margin + self.tol_s,
            self.slide_len + (z - self.top_z))

    def _wheel_world_points(self):
        """World positions of the 3 wheel contact points from the base pose."""
        pts = []
        for a in _LEG_ANGLES:
            body = (self.fp_radius * math.cos(a), self.fp_radius * math.sin(a), 0.0)
            wx, wy, wz = _quat_rotate(self.base_quat, body)
            pts.append((self.base_pose[0] + wx,
                        self.base_pose[1] + wy,
                        self.base_pose[2] + wz))
        return pts

    # ── callbacks ──────────────────────────────────────────────────────────────
    def _status_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        wheels = d.get('wheels', [])
        if isinstance(wheels, list) and wheels:
            attached_wheels = [w for w in wheels if w.get('attached')]
            self.attached = len(attached_wheels)
            self.sum_n = sum(float(w.get('force_n', 0.0))
                             for w in attached_wheels)
            gaps = [float(w.get('gap_mm', 0.0)) for w in attached_wheels]
            forces = [float(w.get('force_n', 0.0)) for w in attached_wheels]
            self.max_gap_mm = max(gaps, default=0.0)
            self.min_force_n = min(forces, default=0.0)
        else:
            self.attached = int(d.get('attached_count', 0))
            self.sum_n = 0.0
            self.max_gap_mm = 0.0
            self.min_force_n = 0.0
        self.surface_counts = {
            'ground': int(d.get('ground_count', 0)),
            'slide': int(d.get('slide_count', 0)),
            'wall': int(d.get('wall_count', 0)),
        }
        self.dominant_surface = str(d.get('dominant_surface', 'none'))
        bp = d.get('base_pose', {})
        p = bp.get('p') if isinstance(bp, dict) else None
        q = bp.get('q') if isinstance(bp, dict) else None
        if p and len(p) == 3 and q and len(q) == 4:
            self.base_pose = (float(p[0]), float(p[1]), float(p[2]))
            self.base_quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            self.last_status_t = self.get_clock().now()

    def _set_goal(self, s_goal, y_goal, theta_goal, label):
        s_goal, y_goal, theta_goal = self._clamp_surface_pose(
            s_goal, y_goal, theta_goal)
        self.goal_pose = (s_goal, y_goal, theta_goal)
        self.route = []
        self.route_index = 0
        self.route_start = None
        self.route_wall_contact = None
        self.reached = False
        self.get_logger().info(
            f'New {label}: surface pose x={s_goal:.2f}, y={y_goal:.2f}, '
            f'theta={theta_goal:+.2f} rad (footprint-feasible).')

    def _goal_cb(self, msg: PointStamped):
        gx, gy, gz = msg.point.x, msg.point.y, msg.point.z
        if self._looks_like_wall_point(gx, gy, gz):
            s_goal = self._wall_progress_from_z(gz)
        else:
            s_goal, _, _, _ = self._world_to_progress(gx, gy, gz)
        self._set_goal(s_goal, gy, self.clicked_point_theta,
                       f'point goal from world ({gx:.2f},{gy:.2f},{gz:.2f})')

    def _pose_goal_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        theta = _yaw_from_quat((q.x, q.y, q.z, q.w))
        if self.goal_pose_is_surface_pose:
            s_goal = msg.pose.position.x
            y_goal = msg.pose.position.y
        else:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            if self._looks_like_wall_point(x, y, z):
                s_goal = self._wall_progress_from_z(z)
                y_goal = y
            else:
                s_goal, y_goal, _, _ = self._world_to_progress(x, y, z)
        self._set_goal(s_goal, y_goal, theta, 'pose goal')

    def _current_surface_theta(self, q, up_dir):
        """Yaw around the current surface normal, relative to +surface progress."""
        bx = _quat_rotate(q, (1.0, 0.0, 0.0))
        bz = _quat_rotate(q, (0.0, 0.0, 1.0))
        ddotn = sum(up_dir[i] * bz[i] for i in range(3))
        dproj = tuple(up_dir[i] - ddotn * bz[i] for i in range(3))
        norm = math.sqrt(sum(v * v for v in dproj))
        if norm < 1.0e-6:
            return 0.0
        dproj = tuple(v / norm for v in dproj)
        cross = (dproj[1] * bx[2] - dproj[2] * bx[1],
                 dproj[2] * bx[0] - dproj[0] * bx[2],
                 dproj[0] * bx[1] - dproj[1] * bx[0])
        sin_t = sum(cross[i] * bz[i] for i in range(3))
        cos_t = sum(dproj[i] * bx[i] for i in range(3))
        return math.atan2(sin_t, cos_t)

    def _wall_contact_established(self):
        """True when contact sensing says the robot has transferred onto the wall.

        Position alone is ambiguous near the slide/wall corner: a base pose low on
        the wall can have surface progress below the conservative settle point.
        Once enough wheels report wall contact, new goals should be planned as
        wall-to-wall moves instead of re-centering through the slide.
        """
        wall_count = self.surface_counts.get('wall', 0)
        return (
            wall_count >= self.min_attached or
            (self.dominant_surface == 'wall' and wall_count > 0))

    def _plan_route(self, s_robot, y_robot):
        """Build a finite-surface route in (x=s, y, theta).

        The slide is the narrow choke point. If the robot is not fully settled on
        the wide wall yet, keep it centered and theta=0 through the slide, then
        allow lateral/yaw motion after the whole footprint is on the wall.
        """
        if self.goal_pose is None:
            return []

        s_goal, y_goal, theta_goal = self.goal_pose
        route = []
        wall_contact = self._wall_contact_established()

        if wall_contact:
            route.append((s_goal, y_goal, theta_goal, 'wall direct'))
            return route

        slide_y_lim = self._safe_y_limit_for_s(0.5 * self.slide_len)
        slide_y = _clamp(self.wall_y, self.wall_y - slide_y_lim,
                         self.wall_y + slide_y_lim)

        crosses_narrow_slide = (
            self.slide_len > 0.0 and
            s_robot < self.s_wall_settle and
            s_goal > self.slide_len + self.tol_s)

        if crosses_narrow_slide:
            if abs(y_robot - slide_y) > self.tol_y:
                route.append((s_robot, slide_y, 0.0, 'center before slide'))
            route.append((self.s_wall_settle, slide_y, 0.0, 'climb slide centerline'))
            if abs(y_goal - slide_y) > self.tol_y:
                route.append((self.s_wall_settle, y_goal, 0.0, 'fan out on wall'))

        route.append((s_goal, y_goal, theta_goal, 'final pose'))
        return route

    def _reset_motion_filter(self):
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_t = None

    def _rate_limit(self, vx, vy, wz):
        if not self.motion_smoothing:
            self.last_cmd = (vx, vy, wz)
            self.last_cmd_t = self.get_clock().now()
            return vx, vy, wz

        now = self.get_clock().now()
        if self.last_cmd_t is None:
            self.last_cmd = (vx, vy, wz)
            self.last_cmd_t = now
            return vx, vy, wz

        dt = max(1.0 / self.publish_hz,
                 (now - self.last_cmd_t).nanoseconds * 1e-9)
        px, py, pw = self.last_cmd

        def step(prev, target, limit):
            delta = _clamp(target - prev, -limit * dt, limit * dt)
            return prev + delta

        vx = step(px, vx, self.accel_limit)
        vy = step(py, vy, self.accel_limit)
        wz = step(pw, wz, self.yaw_accel_limit)
        self.last_cmd = (vx, vy, wz)
        self.last_cmd_t = now
        return vx, vy, wz

    def _motion_scale(self, s_robot):
        """Slow down near geometric transitions and weak contact.

        This is the software substitute for the report's adaptive/full-contact
        wheel suggestion: instead of changing wheel mechanics, reduce commanded
        motion when transition geometry or adhesion status says contact is less
        trustworthy.
        """
        scale = 1.0

        transition_points = [0.0]
        if self.slide_len > 0.0:
            transition_points.append(self.slide_len)
        for s_transition in transition_points:
            distance = abs(s_robot - s_transition)
            if distance < self.transition_slow_band:
                blend = distance / self.transition_slow_band
                local = self.transition_speed_scale + \
                    (1.0 - self.transition_speed_scale) * blend
                scale = min(scale, local)

        if self.attached <= self.min_attached:
            scale = min(scale, 0.55)

        if self.max_gap_mm >= self.gap_stop_mm:
            scale = self.min_motion_scale
        elif self.max_gap_mm > self.gap_slow_start_mm:
            span = self.gap_stop_mm - self.gap_slow_start_mm
            bad = (self.max_gap_mm - self.gap_slow_start_mm) / span
            scale = min(scale, 1.0 - bad * (1.0 - self.min_motion_scale))

        if self.low_force_slow_n > 0.0 and \
                0.0 < self.min_force_n < self.low_force_slow_n:
            force_scale = _clamp(
                self.min_force_n / self.low_force_slow_n,
                self.min_motion_scale, 1.0)
            scale = min(scale, force_scale)

        mixed_transition_contact = (
            self.surface_counts.get('slide', 0) > 0 and
            self.surface_counts.get('wall', 0) > 0)
        if mixed_transition_contact:
            scale = min(scale, 0.65)

        return _clamp(scale, self.min_motion_scale, 1.0)

    # ── edge-safety velocity filter ────────────────────────────────────────────
    def _edge_limit(self, vs, vl):
        """Clamp surface-progress (vs) and lateral (vl) velocities so no wheel is
        driven past a surface edge. Motion AWAY from a violated edge is allowed."""
        plus = []     # distance from each wheel to the +Y edge of its facet
        minus = []    # distance to the -Y edge
        top = []      # distance from each wheel to the wall top edge (in s)
        for (x, y, z) in self._wheel_world_points():
            s_i, y_i, _, hw_i = self._world_to_progress(x, y, z)
            if self._wall_contact_established() and \
                    s_i >= self.slide_len - self.region_margin:
                hw_i = self.wall_y_half
            plus.append(hw_i - y_i)
            minus.append(hw_i + y_i)
            top.append(self.s_wall_top - s_i)
        d_plus, d_minus, d_top = min(plus), min(minus), min(top)

        # Lateral: barrier that ramps the allowed speed to 0 at the margin.
        allow_plus = _clamp((d_plus - self.edge_margin) * self.edge_gain, 0.0, self.v_lat)
        allow_minus = _clamp((d_minus - self.edge_margin) * self.edge_gain, 0.0, self.v_lat)
        vl = _clamp(vl, -allow_minus, allow_plus)

        # Progress: do not climb a wheel off the top edge.
        allow_up = _clamp((d_top - self.edge_margin) * self.edge_gain, 0.0, self.v_climb)
        if vs > 0.0:
            vs = min(vs, allow_up)
        return vs, vl, (d_plus, d_minus, d_top)

    # ── control loop ───────────────────────────────────────────────────────────
    def _tick(self):
        if self.goal_pose is None:
            return  # idle: let teleop drive

        now = self.get_clock().now()
        stale = (self.last_status_t is None or
                 (now - self.last_status_t).nanoseconds * 1e-9 > self.status_timeout)
        if self.base_pose is None or self.base_quat is None or stale:
            self._publish(0.0, 0.0, 0.0, 'no fresh robot pose/adhesion status')
            return
        if self.attached < self.min_attached:
            self._publish(0.0, 0.0, 0.0,
                          f'only {self.attached} wheel(s) attached — holding')
            return
        # No-fall (slip / peel) guard from the magnetic force margins.
        if self.force_guard and self.sum_n > 0.0:
            if self.mu * self.sum_n < self.sf * self.weight:
                self._publish(0.0, 0.0, 0.0,
                              f'friction/slip margin LOW (muN={self.mu*self.sum_n:.0f} '
                              f'< {self.sf*self.weight:.0f} N) — holding')
                return
            if self.sum_n < self.sf * self.req_peel:
                self._publish(0.0, 0.0, 0.0,
                              f'peel/capsize margin LOW (sumN={self.sum_n:.0f} '
                              f'< {self.sf*self.req_peel:.0f} N) — holding')
                return

        q = self.base_quat
        s_robot, y_robot, up_dir, _ = self._world_to_progress(*self.base_pose)
        wall_contact = self._wall_contact_established()
        if wall_contact and s_robot >= self.slide_len - self.region_margin:
            up_dir = (0.0, 0.0, 1.0)
        theta_robot = self._current_surface_theta(q, up_dir)

        if not self.route or self.route_start != self.goal_pose or \
                self.route_wall_contact != wall_contact:
            self.route = self._plan_route(s_robot, y_robot)
            self.route_index = 0
            self.route_start = self.goal_pose
            self.route_wall_contact = wall_contact
            self._reset_motion_filter()
            route_text = ' -> '.join(
                f'{label}(x={s:.2f}, y={y:.2f}, th={th:+.2f})'
                for s, y, th, label in self.route)
            self.get_logger().info(f'Surface route: {route_text}')

        while self.route_index < len(self.route):
            s_goal, y_goal, theta_goal, label = self.route[self.route_index]
            es = s_goal - s_robot
            ey = y_goal - y_robot
            yaw_err = _wrap_pi(theta_goal - theta_robot)
            reached_step = (
                abs(es) <= self.tol_s and
                abs(ey) <= self.tol_y and
                abs(yaw_err) <= self.tol_theta)
            if not reached_step:
                break
            if self.route_index < len(self.route) - 1:
                self.get_logger().info(f'Route step complete: {label}')
                self.route_index += 1
                self._reset_motion_filter()
                continue
            if not self.reached:
                self.reached = True
                self.get_logger().info(
                    f'GOAL REACHED (x err {es:+.3f} m, y err {ey:+.3f} m, '
                    f'theta err {yaw_err:+.2f} rad). Holding.')
            self._publish(0.0, 0.0, 0.0, '')
            return

        if self.route_index >= len(self.route):
            if not self.reached:
                self.reached = True
            self._publish(0.0, 0.0, 0.0, '')
            return
        self.reached = False

        s_goal, y_goal, theta_goal, label = self.route[self.route_index]
        es = s_goal - s_robot
        ey = y_goal - y_robot
        yaw_err = _wrap_pi(theta_goal - theta_robot)
        wz = _clamp(self.kp_yaw * yaw_err, -self.wz_max, self.wz_max)

        transition_drive = (
            self.straight_transition_drive and
            not wall_contact and
            label == 'climb slide centerline' and
            s_robot < self.s_wall_settle - self.tol_s and
            es > 0.0)

        # Nominal command (surface progress + lateral), then EDGE-LIMIT it so no
        # wheel leaves the surface.
        yaw_scale = 1.0 if transition_drive else _clamp(1.0 - abs(yaw_err), 0.25, 1.0)
        vs = yaw_scale * _clamp(self.kp_s * es, -self.v_climb, self.v_climb)
        vl = yaw_scale * _clamp(self.kp_y * ey, -self.v_lat, self.v_lat)
        if transition_drive:
            vl = 0.0
            wz = 0.0
        motion_scale = self._motion_scale(s_robot)
        vs *= motion_scale
        vl *= motion_scale
        wz *= motion_scale
        vs, vl, (d_plus, d_minus, d_top) = self._edge_limit(vs, vl)

        # Desired velocity. During the mixed slide/wall transition, direct body
        # +X keeps all three swerve wheels driving together even if surface
        # classification flips a little early near the corner.
        if transition_drive:
            vbx, vby = vs, 0.0
        else:
            v_world = (up_dir[0] * vs, vl, up_dir[2] * vs)
            vbx, vby, _vbz = _quat_inv_rotate(q, v_world)
        vbx, vby, wz = self._rate_limit(vbx, vby, wz)
        self.get_logger().debug(
            f'ctrl[{self.route_index+1}/{len(self.route)} {label}]: '
            f'x={s_robot:+.2f} ex={es:+.2f} ey={ey:+.2f} eth={yaw_err:+.2f} '
            f'edge(+Y/-Y/top)='
            f'({d_plus:+.2f},{d_minus:+.2f},{d_top:+.2f}) v_body=({vbx:+.3f},'
            f'{vby:+.3f}) wz={wz:+.2f} scale={motion_scale:.2f} '
            f'att={self.attached} sumN={self.sum_n:.0f} gap={self.max_gap_mm:.1f}mm',
            throttle_duration_sec=2.0)
        dbg = String()
        dbg.data = json.dumps({
            'route_index': self.route_index,
            'route_len': len(self.route),
            'step': label,
            'transition_drive': transition_drive,
            'surface_x': s_robot,
            'surface_y': y_robot,
            'surface_theta': theta_robot,
            'target_x': s_goal,
            'target_y': y_goal,
            'target_theta': theta_goal,
            'error_x': es,
            'error_y': ey,
            'error_theta': yaw_err,
            'cmd_surface_x': vs,
            'cmd_surface_y': vl,
            'cmd_body_x': vbx,
            'cmd_body_y': vby,
            'cmd_yaw': wz,
            'motion_scale': motion_scale,
            'max_gap_mm': self.max_gap_mm,
            'min_force_n': self.min_force_n,
            'surface_counts': self.surface_counts,
            'dominant_surface': self.dominant_surface,
            'wall_contact_established': wall_contact,
            'attached': self.attached,
            'sum_n': self.sum_n,
            'edge_plus_y': d_plus,
            'edge_minus_y': d_minus,
            'edge_top': d_top,
        })
        self.debug_pub.publish(dbg)
        self._publish(vbx, vby, wz, '')

    def _publish(self, vx, vy, wz, reason):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(msg)
        if reason:
            self.get_logger().warn(reason, throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClimbGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
