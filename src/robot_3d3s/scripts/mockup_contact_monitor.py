#!/usr/bin/env python3
"""Print compact wheel contact summaries for the mockup collision test."""

import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts


WHEEL_LINKS = ['wheel_1_link', 'wheel_2_link', 'wheel_3_link']


def estimate_contact_fraction(depth_m: float, contact_points: int):
    if depth_m <= 0.0 and contact_points <= 0:
        return 0.0
    depth_fraction = max(0.0, min(1.0, depth_m / 0.002))
    point_fraction = max(0.0, min(1.0, contact_points / 4.0))
    return max(0.10, min(1.0, 0.5 * depth_fraction + 0.5 * point_fraction))


@dataclass
class ContactSummary:
    stamp_s: float
    other_collision: str
    depth_m: float
    contact_points: int
    contact_fraction: float
    position: Optional[tuple]
    normal: Optional[tuple]


class MockupContactMonitor(Node):
    def __init__(self):
        super().__init__('mockup_contact_monitor')
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('report_hz', 2.0)

        self.timeout_s = float(self.get_parameter('timeout_s').value)
        report_hz = max(0.1, float(self.get_parameter('report_hz').value))

        self.contacts: Dict[str, ContactSummary] = {}
        self.msg_counts = {wheel: 0 for wheel in WHEEL_LINKS}
        self.seen_counts = {wheel: 0 for wheel in WHEEL_LINKS}

        for wheel in WHEEL_LINKS:
            topic = f'/robot_3d3s/{wheel.replace("_link", "")}_contacts'
            self.create_subscription(
                Contacts,
                topic,
                lambda msg, wheel_link=wheel: self._contact_cb(wheel_link, msg),
                10)

        self.create_timer(1.0 / report_hz, self._report_cb)
        self.get_logger().info(
            'mockup_contact_monitor ready. Drive the robot into the mockup and '
            'watch for other_collision containing mockup_structure.')

    def _contact_cb(self, wheel_link: str, msg: Contacts):
        self.msg_counts[wheel_link] += 1
        best = None
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if not self._mentions(c1, wheel_link) and not self._mentions(c2, wheel_link):
                continue

            other = c2 if self._mentions(c1, wheel_link) else c1
            depth = max(contact.depths) if contact.depths else 0.0
            contact_points = max(
                len(contact.positions), len(contact.normals), len(contact.depths))
            contact_fraction = estimate_contact_fraction(depth, contact_points)

            position = None
            if contact.positions:
                p = contact.positions[0]
                position = (p.x, p.y, p.z)

            normal = None
            if contact.normals:
                n = contact.normals[0]
                normal = (n.x, n.y, n.z)

            candidate = ContactSummary(
                stamp_s=time.time(),
                other_collision=other,
                depth_m=depth,
                contact_points=contact_points,
                contact_fraction=contact_fraction,
                position=position,
                normal=normal)
            if best is None or candidate.depth_m > best.depth_m:
                best = candidate

        if best is None:
            return

        self.contacts[wheel_link] = best
        self.seen_counts[wheel_link] += 1

    def _report_cb(self):
        now = time.time()
        parts = []
        for wheel in WHEEL_LINKS:
            contact = self.contacts.get(wheel)
            if contact is None or now - contact.stamp_s > self.timeout_s:
                parts.append(
                    f'{wheel}: none(msg={self.msg_counts[wheel]}, seen={self.seen_counts[wheel]})')
                continue

            pos = ''
            if contact.position is not None:
                pos = (
                    f',pos=({contact.position[0]:+.2f},'
                    f'{contact.position[1]:+.2f},{contact.position[2]:+.2f})')
            parts.append(
                f'{wheel}: {contact.other_collision},'
                f'depth={contact.depth_m * 1000.0:.2f}mm,'
                f'points={contact.contact_points},'
                f'fraction={contact.contact_fraction:.2f}{pos}')

        self.get_logger().info(' | '.join(parts))

    @staticmethod
    def _mentions(name: str, token: str) -> bool:
        return token in name or token.replace('_link', '') in name


def main(args=None):
    rclpy.init(args=args)
    node = MockupContactMonitor()
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
