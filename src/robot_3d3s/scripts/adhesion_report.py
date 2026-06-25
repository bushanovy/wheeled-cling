#!/usr/bin/env python3
"""adhesion_report.py — live magnetic-adhesion validation against the literature.

Subscribes to the adhesion status JSON published by the C++ Kmw100AdhesionSystem
plugin and continuously checks the TWO stay-attached conditions from
OmniClimbers (Tavakoli et al., RAS 2013, Fig. 5 / Section 3.2):

  (1) FRICTION condition:  mu * sum(N_i)  >=  m*g
      The tangential friction from the magnetically-loaded wheels must carry the
      robot's weight without sliding down the wall.

  (2) PEEL / MOMENT condition:  sum(N_i)  >=  m*g * c / L
      The magnetic normal forces must resist the overturning moment created by
      gravity acting at the centre of mass, which stands off the wall by `c`
      over a contact span `L`. This is the condition that actually peels wheeled
      climbers off; it is usually the binding one and is why OmniClimbers sized
      their magnet with a safety factor of ~5.

It also classifies each wheel's contact as FULL vs POINT contact from the
plugin's contact_fraction, which is the "full vs point contact" validation your
advisor asked about (the passive-compliant / shape-adaptive papers are all about
keeping contact_fraction high on curved surfaces).

Publishes a one-line String summary on ~report_topic and logs it periodically.
Reads m, mu, and geometry defaults from climb_scene.yaml when given.
"""

import json
import math

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class AdhesionReport(Node):
    def __init__(self):
        super().__init__('adhesion_report')

        self.declare_parameter('scene_yaml', '')
        self.declare_parameter('status_topic', '/robot_3d3s/adhesion_status')
        self.declare_parameter('report_topic', '/robot_3d3s/adhesion_report')
        self.declare_parameter('mass_kg', 8.715)
        self.declare_parameter('mu', 1.0)
        self.declare_parameter('gravity', 9.81)
        # CoM perpendicular stand-off from the wall face (m) and the vertical
        # span between the top and bottom wheel contact lines (m).
        self.declare_parameter('com_standoff_m', 0.18)
        self.declare_parameter('contact_span_m', 0.62)
        self.declare_parameter('full_contact_fraction', 0.85)
        self.declare_parameter('point_contact_fraction', 0.40)
        self.declare_parameter('safety_factor', 1.5)
        self.declare_parameter('log_period_s', 1.0)

        scene_path = str(self.get_parameter('scene_yaml').value)
        self.mass = float(self.get_parameter('mass_kg').value)
        self.mu = float(self.get_parameter('mu').value)
        self.g = float(self.get_parameter('gravity').value)
        if scene_path:
            try:
                with open(scene_path) as f:
                    scene = yaml.safe_load(f) or {}
                self.mass = float(scene.get('robot', {}).get('mass', self.mass))
                self.mu = float(scene.get('flat_wall', {}).get('mu', self.mu))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Could not read scene_yaml: {exc}')

        self.com_standoff = float(self.get_parameter('com_standoff_m').value)
        self.contact_span = max(1e-3, float(self.get_parameter('contact_span_m').value))
        self.full_cf = float(self.get_parameter('full_contact_fraction').value)
        self.point_cf = float(self.get_parameter('point_contact_fraction').value)
        self.sf = max(1.0, float(self.get_parameter('safety_factor').value))
        self.log_period = max(0.1, float(self.get_parameter('log_period_s').value))

        status_topic = str(self.get_parameter('status_topic').value)
        report_topic = str(self.get_parameter('report_topic').value)
        self.report_pub = self.create_publisher(String, report_topic, 10)
        self.create_subscription(String, status_topic, self._status_cb, 10)
        self.create_timer(self.log_period, self._tick)

        self.weight = self.mass * self.g
        self.req_friction = self.weight / max(1e-6, self.mu)        # min sum(N)
        self.req_peel = self.weight * self.com_standoff / self.contact_span
        self.last = None

        self.get_logger().info(
            f'adhesion_report ready. m={self.mass:.2f} kg, weight={self.weight:.1f} N, '
            f'mu={self.mu:.2f}. Min sum(N): friction={self.req_friction:.0f} N, '
            f'peel={self.req_peel:.0f} N (safety x{self.sf:.1f}). Listening on '
            f'{status_topic}.')

    def _status_cb(self, msg: String):
        try:
            self.last = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.last = None

    def _tick(self):
        if not self.last:
            self.get_logger().warn('No adhesion status yet (plugin running?).',
                                   throttle_duration_sec=5.0)
            return

        wheels = self.last.get('wheels', [])
        attached = [w for w in wheels if w.get('attached')]
        sum_n = sum(float(w.get('force_n', 0.0)) for w in attached)

        friction_cap = self.mu * sum_n
        m_friction = friction_cap / self.weight if self.weight > 0 else 0.0
        m_peel = sum_n / self.req_peel if self.req_peel > 0 else float('inf')

        # full vs point contact per wheel
        full = sum(1 for w in wheels
                   if float(w.get('contact_fraction', 0.0)) >= self.full_cf)
        point = sum(1 for w in wheels
                    if 0.0 < float(w.get('contact_fraction', 0.0)) < self.point_cf)

        ok_friction = friction_cap >= self.sf * self.weight
        ok_peel = sum_n >= self.sf * self.req_peel
        will_hold = ok_friction and ok_peel and len(attached) >= 2

        verdict = 'HOLD' if will_hold else 'AT RISK'
        if len(attached) == 0:
            verdict = 'DETACHED -> WILL FALL'

        text = (
            f'[{verdict}] attached={len(attached)}/{len(wheels)} '
            f'sumN={sum_n:.0f}N | friction: cap={friction_cap:.0f}N vs W={self.weight:.0f}N '
            f'(margin x{m_friction:.1f}{"" if ok_friction else " LOW"}) | '
            f'peel: need>={self.req_peel:.0f}N (margin x{m_peel:.1f}{"" if ok_peel else " LOW"}) | '
            f'contact: full={full} point={point}')

        out = String()
        out.data = text
        self.report_pub.publish(out)
        if will_hold:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = AdhesionReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
