#!/usr/bin/env python3
"""climb_scene_rviz.py — make RViz show *exactly* what Gazebo simulates.

Two jobs, both driven by the single-source-of-truth scene YAML
(config/climb_scene.yaml):

1. Publish RViz markers for the scene geometry (ground + flat wall, and the
   slide/pipe if present) in the `world` frame, at the same poses/sizes Gazebo
   loads. So the wall is in the same place in RViz and Gazebo.

2. Relay the robot's LIVE Gazebo world pose to TF as `world -> base_footprint`,
   so the RViz RobotModel tracks the real simulated robot as it climbs. (The old
   visualizer faked this by projecting flat odometry onto the wall; this uses the
   real pose.) robot_state_publisher then provides base_footprint -> links from
   /joint_states.

   The base pose is read from the C++ adhesion plugin's status JSON
   ("base_pose"), NOT from the bridged Gazebo Pose_V topics: in ros_gz those
   arrive with EMPTY frame names, so the robot's transform can't be identified.
   The status topic is known to flow (adhesion_report consumes it).

Inputs
------
  scene_yaml        : path to climb_scene.yaml (required)
  world_frame       : RViz fixed frame, default 'world'
  status_topic      : adhesion status JSON carrying base_pose, default
                      /robot_3d3s/adhesion_status
  clicked_point_topic / goal_point_topic : RViz "Publish Point" -> stored marker
                      and a PointStamped echo for the future click-to-climb step.
"""

import json
import math

import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _rgba(r, g, b, a):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class ClimbSceneRviz(Node):
    def __init__(self):
        super().__init__('climb_scene_rviz')

        self.declare_parameter('scene_yaml', '')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_name', 'robot_3d3s')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('goal_point_topic', '/climb_goal_point')
        self.declare_parameter('marker_topic', '/climb_scene_markers')
        self.declare_parameter('marker_period_s', 0.5)

        scene_path = str(self.get_parameter('scene_yaml').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_name = str(self.get_parameter('robot_name').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        status_topic = str(self.get_parameter('status_topic').value)
        clicked_topic = str(self.get_parameter('clicked_point_topic').value)
        goal_topic = str(self.get_parameter('goal_point_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        marker_period = max(0.1, float(self.get_parameter('marker_period_s').value))

        self.scene = {}
        if scene_path:
            try:
                with open(scene_path) as f:
                    self.scene = yaml.safe_load(f) or {}
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Failed to read scene_yaml {scene_path}: {exc}')
        else:
            self.get_logger().warn('No scene_yaml given; markers disabled.')

        self.selected_point = None

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, latched)
        self.goal_pub = self.create_publisher(PointStamped, goal_topic, 10)
        self.tf_br = TransformBroadcaster(self)
        self._warned_no_base = False

        self.create_subscription(String, status_topic, self._status_cb, 10)
        self.create_subscription(PointStamped, clicked_topic, self._clicked_cb, 10)
        self.create_timer(marker_period, self._publish_markers)
        self._publish_markers()

        self.get_logger().info(
            f'climb_scene_rviz ready: markers on {marker_topic} in '
            f'{self.world_frame}, broadcasting {self.world_frame}->{self.base_frame} '
            f'from base_pose in {status_topic}.')

    # ── Live Gazebo pose -> TF (world -> base_footprint) via status JSON ────────
    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        base = data.get('base_pose')
        if not isinstance(base, dict):
            if not self._warned_no_base:
                self._warned_no_base = True
                self.get_logger().warn(
                    'adhesion status has no base_pose; rebuild the C++ plugin so '
                    'RViz can track the robot.')
            return
        p = base.get('p', [0.0, 0.0, 0.0])
        q = base.get('q', [0.0, 0.0, 0.0, 1.0])
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.world_frame
        # Always publish the configured RViz root frame. Gazebo may report the
        # pose of base_link when the fixed base_footprint link is lumped away;
        # publishing world->base_link would compete with robot_state_publisher's
        # base_footprint->base_link edge and can make RViz choose an unstable TF
        # path for the wheels.
        out.child_frame_id = self.base_frame
        out.transform.translation.x = float(p[0])
        out.transform.translation.y = float(p[1])
        out.transform.translation.z = float(p[2])
        out.transform.rotation.x = float(q[0])
        out.transform.rotation.y = float(q[1])
        out.transform.rotation.z = float(q[2])
        out.transform.rotation.w = float(q[3])
        self.tf_br.sendTransform(out)

    # ── Click on the wall -> remember + echo (used later for click-to-climb) ───
    def _clicked_cb(self, msg: PointStamped):
        self.selected_point = (msg.point.x, msg.point.y, msg.point.z)
        echo = PointStamped()
        echo.header.stamp = self.get_clock().now().to_msg()
        echo.header.frame_id = self.world_frame
        echo.point = msg.point
        self.goal_pub.publish(echo)
        self.get_logger().info(
            f'Wall goal picked at world ({msg.point.x:.2f}, {msg.point.y:.2f}, '
            f'{msg.point.z:.2f}).')
        self._publish_markers()

    # ── Scene markers from the YAML (same numbers Gazebo loads) ────────────────
    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        mid = 0

        ground = self.scene.get('ground')
        if isinstance(ground, dict):
            markers.markers.append(self._box(
                stamp, mid, 'ground',
                (0.0, 0.0, float(ground.get('z', 0.0)) - 0.005),
                (6.0, 6.0, 0.01),
                _rgba(0.55, 0.55, 0.55, 0.35)))
            mid += 1

        wall = self.scene.get('flat_wall')
        face_x = None
        if isinstance(wall, dict):
            pose = [float(v) for v in wall.get('pose', [1.0, 0.0, 1.5])]
            size = [float(v) for v in wall.get('size', [0.10, 3.0, 3.0])]
            markers.markers.append(self._box(
                stamp, mid, 'flat_wall', pose, size,
                _rgba(0.55, 0.07, 0.08, 0.55)))
            mid += 1
            # Arrow showing the climbed-face outward normal.
            face_x, nx = self._wall_face(wall, pose, size)
            markers.markers.append(self._arrow(
                stamp, mid, 'wall_normal',
                (face_x, pose[1], pose[2]),
                (face_x + 0.5 * nx, pose[1], pose[2]),
                _rgba(0.1, 0.9, 1.0, 0.9)))
            mid += 1

        slide = self.scene.get('slide')
        if isinstance(slide, dict) and face_x is not None:
            center, pitch, size = self._slide_geometry(slide, face_x)
            markers.markers.append(self._box(
                stamp, mid, 'slide', center, size,
                _rgba(0.50, 0.06, 0.07, 0.70), pitch=pitch))
            mid += 1

        if self.selected_point is not None:
            route = self._route_marker(stamp, mid, wall, face_x, slide)
            if route is not None:
                markers.markers.append(route)
                mid += 1
            markers.markers.append(self._sphere(
                stamp, mid, 'wall_goal', self.selected_point,
                _rgba(0.1, 1.0, 0.3, 1.0)))
            mid += 1

        self.marker_pub.publish(markers)

    @staticmethod
    def _wall_face(wall, pose, size):
        face = str(wall.get('climb_face', '-x'))
        if face == '-x':
            return pose[0] - 0.5 * size[0], -1.0
        if face == '+x':
            return pose[0] + 0.5 * size[0], 1.0
        return pose[0] - 0.5 * size[0], -1.0

    @staticmethod
    def _slide_geometry(slide, face_x):
        """Same derivation as the launch: either a derived full ramp or an
        explicit short transition lip from slide.pose + slide.length."""
        theta = math.radians(float(slide.get('angle_deg', 45.0)))
        w = float(slide.get('width', 1.2))
        t = float(slide.get('thickness', 0.04))
        pose = slide.get('pose')
        explicit_length = slide.get('length')

        if isinstance(pose, list) and len(pose) >= 3 and explicit_length is not None:
            return (
                (float(pose[0]), float(pose[1]), float(pose[2])),
                -theta,
                (float(explicit_length), w, t),
            )

        sx = float(slide.get('start_x', -0.15))
        sz = float(slide.get('start_z', 0.0))
        end_x = float(slide.get('wall_x', face_x))
        margin = float(slide.get('length_margin', 0.12))
        top_z = sz + (end_x - sx) * math.tan(theta)
        length = math.hypot(end_x - sx, top_z - sz) + margin
        cx = 0.5 * (sx + end_x) + 0.5 * t * math.sin(theta)
        cz = 0.5 * (sz + top_z) - 0.5 * t * math.cos(theta)
        return (cx, 0.0, cz), -theta, (length, w, t)

    def _box(self, stamp, mid, ns, pos, size, color, pitch=0.0):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        # Quaternion for a pure pitch (rotation about world Y).
        m.pose.orientation.y = math.sin(0.5 * pitch)
        m.pose.orientation.w = math.cos(0.5 * pitch)
        m.scale.x, m.scale.y, m.scale.z = size
        m.color = color
        return m

    def _sphere(self, stamp, mid, ns, pos, color):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color = color
        return m

    def _arrow(self, stamp, mid, ns, p0, p1, color):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.scale.y = 0.06
        m.scale.z = 0.06
        m.points = [Point(x=p0[0], y=p0[1], z=p0[2]),
                    Point(x=p1[0], y=p1[1], z=p1[2])]
        m.color = color
        return m

    def _route_marker(self, stamp, mid, wall, face_x, slide):
        if self.selected_point is None or not isinstance(wall, dict):
            return None

        pose = [float(v) for v in wall.get('pose', [1.0, 0.0, 1.5])]
        size = [float(v) for v in wall.get('size', [0.10, 3.0, 3.0])]
        if face_x is None:
            face_x, _ = self._wall_face(wall, pose, size)
        target_y = _clamp(
            self.selected_point[1],
            pose[1] - 0.5 * size[1],
            pose[1] + 0.5 * size[1])
        target_z = _clamp(
            self.selected_point[2],
            pose[2] - 0.5 * size[2],
            pose[2] + 0.5 * size[2])

        points = []
        ground = self.scene.get('ground') if isinstance(self.scene, dict) else {}
        ground_z = float(ground.get('z', 0.0)) if isinstance(ground, dict) else 0.0
        if isinstance(slide, dict):
            theta = math.radians(float(slide.get('angle_deg', 45.0)))
            sx = float(slide.get('start_x', -0.15))
            sz = float(slide.get('start_z', ground_z))
            end_x = float(slide.get('wall_x', face_x))
            top_z = sz + (end_x - sx) * math.tan(theta)
            points.extend((
                (sx, pose[1], ground_z + 0.03),
                (end_x, pose[1], top_z + 0.03),
            ))
        else:
            robot = self.scene.get('robot') if isinstance(self.scene, dict) else {}
            start_z = (
                float(robot.get('start_height_z', pose[2]))
                if isinstance(robot, dict) else pose[2])
            points.append((face_x, pose[1], start_z))

        points.append((face_x, target_y, target_z))
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = stamp
        m.ns = 'climb_route'
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.035
        m.color = _rgba(0.1, 0.85, 1.0, 0.95)
        for x, y, z in points:
            m.points.append(Point(x=x, y=y, z=z))
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ClimbSceneRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
