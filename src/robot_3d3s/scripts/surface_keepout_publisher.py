#!/usr/bin/env python3
"""
surface_keepout_publisher.py - publish the traversable-surface keepout map.

Turns the Stage-1 flat-ground geometry (flat_ground_nav.yaml) into:
  * /keepout_map      nav_msgs/OccupancyGrid  (latched, transient_local)
                      free (0) inside the plate, lethal (100) beyond the edges.
  * /keepout_outline  visualization_msgs/Marker (LINE_STRIP) for RViz.

A Nav2 costmap StaticLayer consumes /keepout_map, so the planner cannot route a
path off the surface. This is the deterministic "Layer 1" edge guarantee: the
edges come from KNOWN geometry, not from a sensor.

Parameters (set from flat_ground_nav.yaml by the launch):
  frame_id       (str)    costmap global frame for the map (default: odom)
  center_x/y     (float)  plate center (m)
  size_x/y       (float)  plate extent (m)
  resolution     (float)  grid cell size (m)
  margin         (float)  lethal border width around the plate (m)
  edge_standoff  (float)  inset of the free region from the edge (m)
  publish_period (float)  0 -> publish once latched; >0 -> republish every N s
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


FREE = 0
LETHAL = 100


class SurfaceKeepoutPublisher(Node):
    def __init__(self):
        super().__init__('surface_keepout_publisher')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('size_x', 4.0)
        self.declare_parameter('size_y', 4.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('margin', 1.0)
        self.declare_parameter('edge_standoff', 0.0)
        self.declare_parameter('publish_period', 0.0)

        g = self.get_parameter
        self.frame_id = g('frame_id').value
        self.cx = float(g('center_x').value)
        self.cy = float(g('center_y').value)
        self.sx = float(g('size_x').value)
        self.sy = float(g('size_y').value)
        self.res = float(g('resolution').value)
        self.margin = float(g('margin').value)
        self.standoff = float(g('edge_standoff').value)
        period = float(g('publish_period').value)

        if self.res <= 0.0:
            raise ValueError('resolution must be > 0')
        if self.sx <= 0.0 or self.sy <= 0.0:
            raise ValueError('surface size must be > 0')

        # Latched, transient_local QoS so a StaticLayer that starts late still
        # receives the (single) map. This matches Nav2 StaticLayer defaults.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'keepout_map', latched)
        self.marker_pub = self.create_publisher(Marker, 'keepout_outline', latched)

        self.grid = self._build_grid()
        self.marker = self._build_marker()

        self._publish_once()
        n_lethal = sum(1 for c in self.grid.data if c == LETHAL)
        self.get_logger().info(
            f'Keepout map: {self.grid.info.width}x{self.grid.info.height} cells @ '
            f'{self.res:.3f} m, origin ({self.grid.info.origin.position.x:.2f}, '
            f'{self.grid.info.origin.position.y:.2f}) in "{self.frame_id}". '
            f'Plate {self.sx:.2f}x{self.sy:.2f} m at ({self.cx:.2f}, {self.cy:.2f}); '
            f'{n_lethal}/{len(self.grid.data)} cells lethal (edges).')

        if period > 0.0:
            self.create_timer(period, self._publish_once)

    def _build_grid(self) -> OccupancyGrid:
        # Map spans the plate plus a lethal border (margin) on every side.
        width_m = self.sx + 2.0 * self.margin
        height_m = self.sy + 2.0 * self.margin
        ncols = int(round(width_m / self.res))
        nrows = int(round(height_m / self.res))

        origin_x = self.cx - self.sx / 2.0 - self.margin
        origin_y = self.cy - self.sy / 2.0 - self.margin

        # Free region (optionally inset from the physical edge by edge_standoff).
        half_x = self.sx / 2.0 - self.standoff
        half_y = self.sy / 2.0 - self.standoff

        data = [LETHAL] * (ncols * nrows)
        for j in range(nrows):
            wy = origin_y + (j + 0.5) * self.res
            if abs(wy - self.cy) > half_y:
                continue
            row = j * ncols
            for i in range(ncols):
                wx = origin_x + (i + 0.5) * self.res
                if abs(wx - self.cx) <= half_x:
                    data[row + i] = FREE

        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.res
        grid.info.width = ncols
        grid.info.height = nrows
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        return grid

    def _build_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = 'surface_edge'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        hx, hy = self.sx / 2.0, self.sy / 2.0
        corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy), (hx, hy)]
        for dx, dy in corners:
            m.points.append(Point(x=self.cx + dx, y=self.cy + dy, z=0.01))
        return m

    def _publish_once(self):
        now = self.get_clock().now().to_msg()
        self.grid.header.stamp = now
        self.marker.header.stamp = now
        self.map_pub.publish(self.grid)
        self.marker_pub.publish(self.marker)


def main():
    rclpy.init()
    node = SurfaceKeepoutPublisher()
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
