#!/usr/bin/env python3
"""RViz grid markers and pick points for pipe surfaces.

Publishes a translucent wireframe grid mapped onto the surface of each
pipe so the operator can see where Publish Point is hitting and pick
goals on the pipe surface, not just on the world XY plane.  The grid is
purely visual: it does not change the surface geometry used by the
planner.

Three pipes are visualised:
  * left_vertical_pipe:  z-axis, R 0.4572 m, length 3.0 m
  * right_vertical_pipe: z-axis, R 0.2540 m, length 3.0 m
  * horizontal_pipe:     x-axis, R 0.2540 m, length 3.1312 m

Each pipe gets:
  * a SOLID OGRE cylinder hit-target (alpha 1.0 by default) so RViz Publish
    Point has a reliable, visible 3D surface to ray-cast against.  This is the
    key fix: transparent TRIANGLE_LIST/POINT_CLOUD markers are not
    reliably hit by RViz Publish Point across versions; a built-in
    ``Marker.CYLINDER`` always registers in the OGRE scene query.
  * a faint filled cylinder skin (TRIANGLE_LIST) so the user can see
    the surface boundaries between grid lines.
  * a dense PointCloud2 shell so RViz Publish Point can select the
    pipe surface as a fallback.
  * 16 axial grid lines (parallel to the pipe axis) at evenly spaced
    angles, drawn as LINE_LIST marker segments.
  * 24 circumferential grid rings drawn at evenly spaced axial
    positions, again as LINE_LIST segments.
"""

import math
import struct
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


Vec3 = Tuple[float, float, float]


def _clamp(value: float, lower: float, upper: float):
    return max(lower, min(upper, value))


def _cylinder_surface_point(axis: str, axis_point, radius: float,
                            u: float, v: float):
    """Return (x, y, z) on a cylinder. u is along the axis, v is angle."""
    ax, ay, az = axis_point
    if axis == 'x':
        return (u, ay + radius * math.cos(v), az + radius * math.sin(v))
    if axis == 'y':
        return (ax + radius * math.sin(v), u, az + radius * math.cos(v))
    return (ax + radius * math.cos(v), ay + radius * math.sin(v), u)


def _axis_center(axis: str, axis_point: Vec3) -> float:
    if axis == 'x':
        return axis_point[0]
    if axis == 'y':
        return axis_point[1]
    return axis_point[2]


def _cylinder_axis_orientation(axis: str):
    """Return (roll, pitch, yaw) so an OGRE CYLINDER marker (default
    aligned with +Z) is rotated onto the requested axis.

    x-axis pipe:  rotate +Z -> +X  (pitch = +pi/2)
    y-axis pipe:  rotate +Z -> +Y  (roll  = +pi/2)
    z-axis pipe:  identity.
    """
    if axis == 'x':
        return 0.0, math.pi / 2.0, 0.0
    if axis == 'y':
        return math.pi / 2.0, 0.0, 0.0
    return 0.0, 0.0, 0.0


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


def _hit_target_marker(stamp, frame_id: str, namespace: str, marker_id: int,
                       axis: str, axis_point: Vec3, radius: float,
                       length: float, color_rgba):
    """Solid OGRE cylinder marker that Publish Point can always hit.

    This is the geometry that Publish Point ray-casts against when the user
    clicks on the pipe surface, so it stays fully opaque and easy to see in
    every configured map.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = axis_point[0]
    marker.pose.position.y = axis_point[1]
    marker.pose.position.z = axis_point[2]
    roll, pitch, yaw = _cylinder_axis_orientation(axis)
    qx, qy, qz, qw = _quat_from_rpy(roll, pitch, yaw)
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw
    marker.scale.x = 2.0 * radius
    marker.scale.y = 2.0 * radius
    marker.scale.z = length
    marker.color.r = color_rgba[0]
    marker.color.g = color_rgba[1]
    marker.color.b = color_rgba[2]
    marker.color.a = color_rgba[3]
    return marker


def _grid_marker(stamp, frame_id: str, namespace: str, marker_id: int,
                 axis: str, axis_point: Vec3, radius: float, length: float,
                 color_rgba, axial_lines: int, rings: int,
                 ring_segments: int, line_width: float):
    """Return a LINE_LIST marker that draws a wireframe grid on the
    cylinder surface."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = line_width
    marker.color.r = color_rgba[0]
    marker.color.g = color_rgba[1]
    marker.color.b = color_rgba[2]
    marker.color.a = color_rgba[3]

    half_len = 0.5 * length
    axis_center = _axis_center(axis, axis_point)
    for i in range(axial_lines):
        v = 2.0 * math.pi * i / axial_lines
        a = _cylinder_surface_point(
            axis, axis_point, radius, axis_center - half_len, v)
        b = _cylinder_surface_point(
            axis, axis_point, radius, axis_center + half_len, v)
        marker.points.append(_point(a))
        marker.points.append(_point(b))

    for j in range(rings):
        u = axis_center - half_len + length * j / max(1, rings - 1)
        for k in range(ring_segments):
            v0 = 2.0 * math.pi * k / ring_segments
            v1 = 2.0 * math.pi * (k + 1) / ring_segments
            p0 = _cylinder_surface_point(axis, axis_point, radius, u, v0)
            p1 = _cylinder_surface_point(axis, axis_point, radius, u, v1)
            marker.points.append(_point(p0))
            marker.points.append(_point(p1))

    return marker


def _surface_marker(stamp, frame_id: str, namespace: str, marker_id: int,
                    axis: str, axis_point: Vec3, radius: float, length: float,
                    color_rgba, axis_segments: int, ring_segments: int):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = color_rgba[0]
    marker.color.g = color_rgba[1]
    marker.color.b = color_rgba[2]
    marker.color.a = color_rgba[3]

    half_len = 0.5 * length
    axis_center = _axis_center(axis, axis_point)
    for i in range(axis_segments):
        u0 = axis_center - half_len + length * i / axis_segments
        u1 = axis_center - half_len + length * (i + 1) / axis_segments
        for j in range(ring_segments):
            v0 = 2.0 * math.pi * j / ring_segments
            v1 = 2.0 * math.pi * (j + 1) / ring_segments
            p00 = _cylinder_surface_point(axis, axis_point, radius, u0, v0)
            p10 = _cylinder_surface_point(axis, axis_point, radius, u1, v0)
            p11 = _cylinder_surface_point(axis, axis_point, radius, u1, v1)
            p01 = _cylinder_surface_point(axis, axis_point, radius, u0, v1)
            marker.points.extend((
                _point(p00), _point(p10), _point(p11),
                _point(p00), _point(p11), _point(p01),
            ))

    return marker


def _point(p):
    from geometry_msgs.msg import Point
    return Point(x=p[0], y=p[1], z=p[2])


def _point_cloud(stamp, frame_id: str, points):
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
    return cloud


def _surface_points(axis: str, axis_point: Vec3, radius: float, length: float,
                    axis_samples: int, ring_samples: int):
    half_len = 0.5 * length
    axis_center = _axis_center(axis, axis_point)
    points = []
    for i in range(axis_samples):
        u = axis_center - half_len + length * i / max(1, axis_samples - 1)
        for j in range(ring_samples):
            v = 2.0 * math.pi * j / ring_samples
            points.append(_cylinder_surface_point(axis, axis_point, radius, u, v))
    return points


class ExperimentalPolygonGrid(Node):
    def __init__(self):
        super().__init__('experimental_polygon_grid')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('marker_topic',
                               '/experimental_polygon_markers')
        self.declare_parameter('point_cloud_topic', '')
        self.declare_parameter('hit_target_enabled', True)
        self.declare_parameter('hit_target_alpha', 1.0)
        self.declare_parameter('grid_profile', 'experimental_polygon')
        self.declare_parameter('grid_namespace', '')
        self.declare_parameter('publish_hz', 1.0)
        # The default grid densities below are deliberately high so
        # that RViz Publish Point reliably hits the pipe surface and
        # operators get fine-grained visual feedback.  The
        # ``horizontal_pipe_test.launch.py`` overrides these to even
        # higher values.
        self.declare_parameter('axial_lines', 32)
        self.declare_parameter('rings', 48)
        self.declare_parameter('ring_segments', 128)
        self.declare_parameter('line_width_m', 0.008)
        self.declare_parameter('surface_alpha', 0.0)
        self.declare_parameter('surface_axis_segments', 64)
        self.declare_parameter('surface_ring_segments', 160)
        self.declare_parameter('point_cloud_axis_samples', 80)
        self.declare_parameter('point_cloud_ring_samples', 160)
        self.declare_parameter('cylinder_axis', 'x')
        self.declare_parameter('axis_x', 0.0)
        self.declare_parameter('axis_y', 0.0)
        self.declare_parameter('axis_z', 2.0)
        self.declare_parameter('surface_radius_m', 1.2)
        self.declare_parameter('surface_length_m', 6.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('marker_topic').value,
            qos)
        self.point_pub = None
        point_cloud_topic = str(self.get_parameter('point_cloud_topic').value)
        if point_cloud_topic:
            self.point_pub = self.create_publisher(
                PointCloud2, point_cloud_topic, qos)
        hz = max(0.1, float(self.get_parameter('publish_hz').value))
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(
            f'experimental_polygon_grid publishing surface grids on '
            f'{self.get_parameter("marker_topic").value}')
        if self.point_pub is not None:
            self.get_logger().info(
                f'experimental_polygon_grid publishing RViz pick points on '
                f'{point_cloud_topic}')

    @property
    def frame(self):
        return str(self.get_parameter('world_frame').value)

    @property
    def namespace(self):
        configured = str(self.get_parameter('grid_namespace').value)
        if configured:
            return configured
        profile = str(self.get_parameter('grid_profile').value)
        if profile == 'configured_pipe':
            return 'pipe_surface_grid'
        return 'experimental_polygon_grid'

    def _configured_pipe_surfaces(self) -> Iterable[Tuple[str, Vec3, float, float]]:
        axis = str(self.get_parameter('cylinder_axis').value).lower()
        if axis not in ('x', 'y', 'z'):
            self.get_logger().warn(
                f'Invalid cylinder_axis={axis}; using x.',
                throttle_duration_sec=2.0)
            axis = 'x'
        return ((
            axis,
            (
                float(self.get_parameter('axis_x').value),
                float(self.get_parameter('axis_y').value),
                float(self.get_parameter('axis_z').value),
            ),
            max(0.01, float(self.get_parameter('surface_radius_m').value)),
            max(0.01, float(self.get_parameter('surface_length_m').value)),
        ),)

    def _experimental_polygon_surfaces(
            self) -> Iterable[Tuple[str, Vec3, float, float]]:
        return (
            ('z', (-1.5656, 0.0, 1.5), 0.4572, 3.0),
            ('z', (1.5656, 0.0, 1.5), 0.2540, 3.0),
            ('x', (0.0, 0.0, 1.246), 0.2540, 3.1312),
        )

    def _surfaces(self) -> Iterable[Tuple[str, Vec3, float, float]]:
        profile = str(self.get_parameter('grid_profile').value)
        if profile == 'configured_pipe':
            return self._configured_pipe_surfaces()
        return self._experimental_polygon_surfaces()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        surface_alpha = _clamp(
            float(self.get_parameter('surface_alpha').value), 0.0, 1.0)
        surface_axis_segments = max(
            1, int(self.get_parameter('surface_axis_segments').value))
        surface_ring_segments = max(
            8, int(self.get_parameter('surface_ring_segments').value))
        axial_lines = max(
            1, int(self.get_parameter('axial_lines').value))
        rings = max(
            1, int(self.get_parameter('rings').value))
        ring_segments = max(
            8, int(self.get_parameter('ring_segments').value))
        line_width = max(
            0.001, float(self.get_parameter('line_width_m').value))
        hit_target_enabled = bool(
            self.get_parameter('hit_target_enabled').value)
        hit_target_alpha = _clamp(
            float(self.get_parameter('hit_target_alpha').value), 0.0, 1.0)
        surfaces = list(self._surfaces())
        # Publish three layers per surface so RViz Publish Point can
        # always hit the pipe AND the operator gets a clearly visible
        # grid on the pipe:
        #   * 9000+: solid OGRE CYLINDER hit target (opaque by default)
        #   * 1000+: filled TRIANGLE_LIST surface mesh (alpha = surface_alpha)
        #   * 2000+: LINE_LIST wireframe (alpha 0.75, very visible)
        for marker_id, (axis, axis_point, radius, length) in enumerate(
                surfaces, start=1):
            if hit_target_enabled:
                markers.markers.append(_hit_target_marker(
                    stamp, self.frame, self.namespace, 9000 + marker_id,
                    axis, axis_point, radius, length,
                    (0.90, 0.62, 0.18, hit_target_alpha)))
            if surface_alpha > 0.0:
                markers.markers.append(_surface_marker(
                    stamp, self.frame, self.namespace, 1000 + marker_id,
                    axis, axis_point, radius, length,
                    (0.70, 0.70, 0.70, surface_alpha),
                    surface_axis_segments, surface_ring_segments))
            # Always publish the wireframe so the operator can see the
            # grid.  The line color matches the planner's active-surface
            # cyan so it reads as part of the same surface overlay.
            markers.markers.append(_grid_marker(
                stamp, self.frame, self.namespace, 2000 + marker_id,
                axis, axis_point, radius, length,
                (0.62, 0.62, 0.62, 0.85),
                axial_lines, rings, ring_segments, line_width))
        self.pub.publish(markers)
        # PointCloud2 publish is a dense hit-test fallback for RViz Publish
        # Point.  The PointCloud2 display must have ``Selectable = true`` for
        # this fallback to work.
        if self.point_pub is not None:
            pc_axis_samples = max(
                2, int(self.get_parameter(
                    'point_cloud_axis_samples').value))
            pc_ring_samples = max(
                8, int(self.get_parameter(
                    'point_cloud_ring_samples').value))
            pc_points = []
            for (axis, axis_point, radius, length) in surfaces:
                pc_points.extend(_surface_points(
                    axis, axis_point, radius, length,
                    pc_axis_samples, pc_ring_samples))
            self.point_pub.publish(
                _point_cloud(stamp, self.frame, pc_points))


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentalPolygonGrid()
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
