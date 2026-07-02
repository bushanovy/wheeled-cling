#!/usr/bin/env python3
"""Reusable surface geometry and graph routing for 3D3S climbing planners."""

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import yaml


Vec3 = Tuple[float, float, float]
Vec2 = Tuple[float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rate_limited_heading(
        previous: Optional[float],
        desired: float,
        dt: float,
        max_rate: float,
        deadband: float = 0.0) -> Tuple[float, float]:
    """Return a heading limited toward desired plus the original error.

    Small heading errors inside ``deadband`` keep the previous heading.  This
    avoids steering chatter when the path target jitters by only a few degrees.
    """
    desired = wrap_pi(float(desired))
    if previous is None:
        return desired, 0.0
    previous = wrap_pi(float(previous))
    error = wrap_pi(desired - previous)
    if abs(error) <= max(0.0, deadband):
        return previous, error
    max_delta = max(0.0, float(max_rate)) * max(0.0, float(dt))
    if max_delta <= 1e-12:
        return previous, error
    delta = clamp(error, -max_delta, max_delta)
    return wrap_pi(previous + delta), error


def heading_alignment_scale(error: float, min_scale: float = 0.20) -> float:
    """Slow translation when held heading is far from desired heading."""
    return clamp(math.cos(abs(float(error))), min_scale, 1.0)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def add(a: Vec3, b: Vec3) -> Vec3:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def sub(a: Vec3, b: Vec3) -> Vec3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def mul(a: Vec3, scale: float) -> Vec3:
    return a[0] * scale, a[1] * scale, a[2] * scale


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Vec3, fallback: Vec3 = (1.0, 0.0, 0.0)) -> Vec3:
    length = norm(a)
    if length <= 1e-9:
        return fallback
    return a[0] / length, a[1] / length, a[2] / length


def distance(a: Vec3, b: Vec3) -> float:
    return norm(sub(a, b))


def quat_rotate(q: Tuple[float, float, float, float], v: Vec3) -> Vec3:
    qx, qy, qz, qw = q
    x, y, z = v
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _vec3(values: Sequence[float]) -> Vec3:
    return float(values[0]), float(values[1]), float(values[2])


def _point_segment_distance_2d(
        p: Vec2, a: Vec2, b: Vec2) -> Tuple[float, Vec2, float]:
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 1e-12:
        return math.hypot(px - ax, py - ay), a, 0.0
    t = clamp(((px - ax) * dx + (py - ay) * dy) / length2, 0.0, 1.0)
    q = ax + t * dx, ay + t * dy
    return math.hypot(px - q[0], py - q[1]), q, t


def _point_in_polygon_2d(p: Vec2, vertices: Sequence[Vec2]) -> bool:
    inside = False
    x, y = p
    count = len(vertices)
    for i in range(count):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % count]
        if ((y1 > y) != (y2 > y)):
            x_at_y = (x2 - x1) * (y - y1) / max(y2 - y1, 1e-12) + x1
            if x < x_at_y:
                inside = not inside
    return inside


@dataclass
class SurfaceProjection:
    surface: "Surface"
    u: float
    v: float
    point: Vec3
    normal: Vec3
    tangent_u: Vec3
    tangent_v: Vec3
    distance: float
    edge_clearance: float = float("inf")
    inside: bool = True


@dataclass
class RoutePoint:
    surface: "Surface"
    u: float
    v: float
    point: Vec3


@dataclass
class RouteDiagnostics:
    algorithm: str
    path: List[str]
    total_cost: float
    components: Dict[str, float]
    expanded: int


class Surface:
    def __init__(self, name: str):
        self.name = name
        self.neighbors: List[str] = []
        self.adhesion_quality = 1.0
        self.contact_quality = 1.0
        self.traversal_cost = 1.0
        self.edge_risk = 0.0
        self.transition_risk = 0.0
        self.slip_risk = 0.0

    def project(self, point: Vec3) -> SurfaceProjection:
        raise NotImplementedError

    def point_at(self, u: float, v: float) -> Vec3:
        raise NotImplementedError

    def tangents_at(self, u: float, v: float) -> Tuple[Vec3, Vec3, Vec3]:
        raise NotImplementedError

    def clamp_uv(self, u: float, v: float) -> Tuple[float, float]:
        return u, v

    def distance_between(self, a: Vec2, b: Vec2) -> float:
        pa = self.point_at(a[0], a[1])
        pb = self.point_at(b[0], b[1])
        return distance(pa, pb)

    def interpolate(self, start: Vec2, goal: Vec2, count: int) -> List[RoutePoint]:
        points = []
        for i in range(max(2, count)):
            ratio = i / max(1, count - 1)
            u = start[0] + (goal[0] - start[0]) * ratio
            v = start[1] + (goal[1] - start[1]) * ratio
            u, v = self.clamp_uv(u, v)
            points.append(RoutePoint(self, u, v, self.point_at(u, v)))
        return points

    def motion_vector(self, current: Vec2, goal: Vec2) -> Tuple[float, float]:
        return goal[0] - current[0], goal[1] - current[1]


class CylinderSurface(Surface):
    def __init__(self, name: str, axis: str, axis_point: Vec3,
                 radius: float, length: float):
        super().__init__(name)
        self.axis = axis.lower()
        if self.axis not in ("x", "y", "z"):
            raise ValueError(f"invalid cylinder axis {axis!r}")
        self.axis_point = axis_point
        self.radius = max(1e-6, float(radius))
        self.length = max(1e-6, float(length))

    def _axis_value(self, point: Vec3) -> float:
        if self.axis == "x":
            return point[0]
        if self.axis == "y":
            return point[1]
        return point[2]

    def axis_value(self, point: Vec3) -> float:
        return self._axis_value(point)

    def _axis_center(self) -> float:
        return self._axis_value(self.axis_point)

    def _axis_vector(self) -> Vec3:
        if self.axis == "x":
            return 1.0, 0.0, 0.0
        if self.axis == "y":
            return 0.0, 1.0, 0.0
        return 0.0, 0.0, 1.0

    def axis_vector(self) -> Vec3:
        return self._axis_vector()

    def _theta_for(self, point: Vec3) -> float:
        ax, ay, az = self.axis_point
        x, y, z = point
        if self.axis == "x":
            return math.atan2(z - az, y - ay)
        if self.axis == "y":
            return math.atan2(x - ax, z - az)
        return math.atan2(y - ay, x - ax)

    def theta_for(self, point: Vec3) -> float:
        return self._theta_for(point)

    def clamp_uv(self, u: float, v: float) -> Tuple[float, float]:
        center = self._axis_center()
        return clamp(u, center - 0.5 * self.length, center + 0.5 * self.length), wrap_pi(v)

    def point_at(self, u: float, v: float) -> Vec3:
        ax, ay, az = self.axis_point
        if self.axis == "x":
            return u, ay + self.radius * math.cos(v), az + self.radius * math.sin(v)
        if self.axis == "y":
            return ax + self.radius * math.sin(v), u, az + self.radius * math.cos(v)
        return ax + self.radius * math.cos(v), ay + self.radius * math.sin(v), u

    def tangents_at(self, u: float, v: float) -> Tuple[Vec3, Vec3, Vec3]:
        tangent_u = self._axis_vector()
        if self.axis == "x":
            tangent_v = (0.0, -math.sin(v), math.cos(v))
            normal = (0.0, math.cos(v), math.sin(v))
        elif self.axis == "y":
            tangent_v = (math.cos(v), 0.0, -math.sin(v))
            normal = (math.sin(v), 0.0, math.cos(v))
        else:
            tangent_v = (-math.sin(v), math.cos(v), 0.0)
            normal = (math.cos(v), math.sin(v), 0.0)
        return tangent_u, tangent_v, normal

    def project(self, point: Vec3) -> SurfaceProjection:
        raw_u = self._axis_value(point)
        theta = self._theta_for(point)
        u, v = self.clamp_uv(raw_u, theta)
        projected = self.point_at(u, v)
        tangent_u, tangent_v, normal = self.tangents_at(u, v)
        axis_edge = min(
            u - (self._axis_center() - 0.5 * self.length),
            (self._axis_center() + 0.5 * self.length) - u)
        inside = abs(raw_u - u) <= 1e-9
        return SurfaceProjection(
            self, u, v, projected, normal, tangent_u, tangent_v,
            distance(projected, point), axis_edge, inside)

    def distance_between(self, a: Vec2, b: Vec2) -> float:
        du = b[0] - a[0]
        dv = wrap_pi(b[1] - a[1]) * self.radius
        return math.hypot(du, dv)

    def interpolate(self, start: Vec2, goal: Vec2, count: int) -> List[RoutePoint]:
        points = []
        theta_delta = wrap_pi(goal[1] - start[1])
        for i in range(max(2, count)):
            ratio = i / max(1, count - 1)
            u = start[0] + (goal[0] - start[0]) * ratio
            v = wrap_pi(start[1] + theta_delta * ratio)
            u, v = self.clamp_uv(u, v)
            points.append(RoutePoint(self, u, v, self.point_at(u, v)))
        return points

    def motion_vector(self, current: Vec2, goal: Vec2) -> Tuple[float, float]:
        return goal[0] - current[0], wrap_pi(goal[1] - current[1])


class PolygonSurface(Surface):
    def __init__(self, name: str, vertices: Sequence[Sequence[float]]):
        super().__init__(name)
        if len(vertices) < 3:
            raise ValueError("polygon surface requires at least 3 vertices")
        self.vertices3 = [_vec3(v) for v in vertices]
        self.origin = self.vertices3[0]
        first_edge = sub(self.vertices3[1], self.origin)
        normal = (0.0, 0.0, 0.0)
        for vertex in self.vertices3[2:]:
            normal = cross(first_edge, sub(vertex, self.origin))
            if norm(normal) > 1e-9:
                break
        self.normal = normalize(normal, (0.0, 0.0, 1.0))
        self.u_axis = normalize(first_edge)
        self.v_axis = normalize(cross(self.normal, self.u_axis))
        self.vertices2 = [self._to_uv(v) for v in self.vertices3]

    def _to_uv(self, point: Vec3) -> Vec2:
        rel = sub(point, self.origin)
        return dot(rel, self.u_axis), dot(rel, self.v_axis)

    def _from_uv(self, u: float, v: float) -> Vec3:
        return add(self.origin, add(mul(self.u_axis, u), mul(self.v_axis, v)))

    def clamp_uv(self, u: float, v: float) -> Tuple[float, float]:
        if _point_in_polygon_2d((u, v), self.vertices2):
            return u, v
        best = None
        for i, a in enumerate(self.vertices2):
            b = self.vertices2[(i + 1) % len(self.vertices2)]
            d, q, _ = _point_segment_distance_2d((u, v), a, b)
            if best is None or d < best[0]:
                best = (d, q)
        return best[1] if best is not None else (u, v)

    def _edge_clearance(self, uv: Vec2, inside: bool) -> float:
        distances = []
        for i, a in enumerate(self.vertices2):
            b = self.vertices2[(i + 1) % len(self.vertices2)]
            d, _, _ = _point_segment_distance_2d(uv, a, b)
            distances.append(d)
        clearance = min(distances) if distances else float("inf")
        return clearance if inside else -clearance

    def point_at(self, u: float, v: float) -> Vec3:
        return self._from_uv(u, v)

    def tangents_at(self, u: float, v: float) -> Tuple[Vec3, Vec3, Vec3]:
        return self.u_axis, self.v_axis, self.normal

    def project(self, point: Vec3) -> SurfaceProjection:
        plane_distance = dot(sub(point, self.origin), self.normal)
        plane_point = sub(point, mul(self.normal, plane_distance))
        raw_uv = self._to_uv(plane_point)
        inside = _point_in_polygon_2d(raw_uv, self.vertices2)
        u, v = self.clamp_uv(*raw_uv)
        projected = self.point_at(u, v)
        return SurfaceProjection(
            self, u, v, projected, self.normal, self.u_axis, self.v_axis,
            distance(projected, point), self._edge_clearance((u, v), inside),
            inside)


class SurfaceGraph:
    def __init__(self, surfaces: Iterable[Surface],
                 transitions: Optional[Iterable[Dict]] = None,
                 cost_weights: Optional[Dict[str, float]] = None):
        self.surfaces: Dict[str, Surface] = {surface.name: surface for surface in surfaces}
        self.edges: Dict[str, Dict[str, float]] = {name: {} for name in self.surfaces}
        self.portals: Dict[Tuple[str, str], Tuple[Vec2, Vec2]] = {}
        self.transition_risks: Dict[Tuple[str, str], float] = {}
        self.cost_weights = {
            "distance": 1.0,
            "transition": 1.0,
            "adhesion": 2.0,
            "contact": 2.5,
            "edge": 1.5,
            "gravity": 1.0,
            "slip": 2.0,
            "turn": 0.75,
            "heuristic": 1.0,
            "edge_margin_m": 0.10,
        }
        if cost_weights:
            for key, value in cost_weights.items():
                if key in self.cost_weights:
                    self.cost_weights[key] = float(value)
        self.last_route = RouteDiagnostics("none", [], 0.0, {}, 0)
        for surface in self.surfaces.values():
            for neighbor in surface.neighbors:
                self.connect(surface.name, neighbor)
        for transition in transitions or []:
            a = str(transition.get("from") or transition.get("a") or "")
            b = str(transition.get("to") or transition.get("b") or "")
            if a and b:
                self.connect(a, b, float(transition.get("cost", 1.0)))
                risk = float(transition.get("risk", transition.get("transition_risk", 0.0)))
                self.transition_risks[(a, b)] = risk
                self.transition_risks[(b, a)] = risk
                self._set_portal_from_transition(a, b, transition)

    def connect(self, a: str, b: str, cost: float = 1.0):
        if a not in self.surfaces or b not in self.surfaces:
            return
        self.edges.setdefault(a, {})[b] = max(0.0, cost)
        self.edges.setdefault(b, {})[a] = max(0.0, cost)

    def _set_portal_from_transition(self, a: str, b: str, transition: Dict):
        if a not in self.surfaces or b not in self.surfaces:
            return
        if "point" in transition:
            point = _vec3(transition["point"])
            a_proj = self.surfaces[a].project(point)
            b_proj = self.surfaces[b].project(point)
            a_uv = (a_proj.u, a_proj.v)
            b_uv = (b_proj.u, b_proj.v)
        elif "from_uv" in transition and "to_uv" in transition:
            a_uv = (float(transition["from_uv"][0]), float(transition["from_uv"][1]))
            b_uv = (float(transition["to_uv"][0]), float(transition["to_uv"][1]))
        else:
            return
        self.portals[(a, b)] = (a_uv, b_uv)
        self.portals[(b, a)] = (b_uv, a_uv)

    def _portal(self, a: str, b: str) -> Optional[Tuple[Vec2, Vec2]]:
        return self.portals.get((a, b))

    def _transition_risk(self, a: str, b: str) -> float:
        return self.transition_risks.get((a, b), 0.0)

    def closest_projection(self, point: Vec3) -> Optional[SurfaceProjection]:
        best = None
        for surface in self.surfaces.values():
            projection = surface.project(point)
            if best is None or projection.distance < best.distance:
                best = projection
        return best

    def shortest_surface_path(self, start: str, goal: str) -> List[str]:
        if start not in self.surfaces or goal not in self.surfaces:
            return []
        start_projection = self.surfaces[start].project(self._surface_center(start))
        goal_projection = self.surfaces[goal].project(self._surface_center(goal))
        return self.astar_surface_path(start_projection, goal_projection)

    def _surface_center(self, name: str) -> Vec3:
        surface = self.surfaces[name]
        if isinstance(surface, CylinderSurface):
            return surface.point_at(surface._axis_center(), 0.0)
        if isinstance(surface, PolygonSurface):
            count = max(1, len(surface.vertices3))
            return (
                sum(p[0] for p in surface.vertices3) / count,
                sum(p[1] for p in surface.vertices3) / count,
                sum(p[2] for p in surface.vertices3) / count,
            )
        return (0.0, 0.0, 0.0)

    def _surface_load_fraction(self, surface: Surface, uv: Vec2) -> float:
        _, _, normal = surface.tangents_at(uv[0], uv[1])
        nz = normal[2]
        return math.sqrt(max(0.0, 1.0 - nz * nz))

    def _edge_penalty(self, projection: SurfaceProjection) -> float:
        margin = max(1e-6, self.cost_weights["edge_margin_m"])
        clearance = projection.edge_clearance
        if not math.isfinite(clearance):
            return 0.0
        if clearance <= 0.0:
            return 10.0
        if clearance >= margin:
            return 0.0
        return (margin - clearance) / margin

    def _turn_penalty(
            self, previous: Optional[str], current: str, neighbor: str) -> float:
        if previous is None:
            return 0.0
        entry = self._portal(previous, current)
        exit_ = self._portal(current, neighbor)
        if entry is None or exit_ is None:
            return 0.0
        in_uv = entry[1]
        out_uv = exit_[0]
        surface = self.surfaces[current]
        center = surface.point_at(in_uv[0], in_uv[1])
        vin = normalize(sub(center, self.surfaces[previous].point_at(
            entry[0][0], entry[0][1])))
        vout = normalize(sub(surface.point_at(out_uv[0], out_uv[1]), center))
        alignment = clamp(dot(vin, vout), -1.0, 1.0)
        return math.acos(alignment) / math.pi

    def _edge_cost(
            self, current: str, neighbor: str, current_uv: Vec2,
            goal: SurfaceProjection, previous: Optional[str]) -> Tuple[float, Dict[str, float], Vec2]:
        surface = self.surfaces[current]
        portal = self._portal(current, neighbor)
        if portal is not None:
            exit_uv, neighbor_entry_uv = portal
        else:
            exit_projection = surface.project(goal.point)
            exit_uv = (exit_projection.u, exit_projection.v)
            neighbor_projection = self.surfaces[neighbor].project(
                surface.point_at(exit_uv[0], exit_uv[1]))
            neighbor_entry_uv = (neighbor_projection.u, neighbor_projection.v)

        exit_projection = surface.project(surface.point_at(exit_uv[0], exit_uv[1]))
        distance_cost = surface.distance_between(current_uv, exit_uv)
        gravity_cost = self._surface_load_fraction(surface, current_uv) * distance_cost
        adhesion_quality = clamp(surface.adhesion_quality, 1e-3, 1.0)
        adhesion_cost = (1.0 / adhesion_quality - 1.0) * distance_cost
        contact_quality = clamp(surface.contact_quality, 1e-3, 1.0)
        contact_cost = (1.0 / contact_quality - 1.0) * distance_cost
        edge_cost = self._edge_penalty(exit_projection) + surface.edge_risk
        transition_cost = self.edges.get(current, {}).get(neighbor, 1.0)
        transition_cost += surface.transition_risk + self._transition_risk(current, neighbor)
        slip_cost = surface.slip_risk * distance_cost
        turn_cost = self._turn_penalty(previous, current, neighbor)

        components = {
            "distance": distance_cost * surface.traversal_cost,
            "transition": transition_cost,
            "adhesion": adhesion_cost,
            "contact": contact_cost,
            "edge": edge_cost,
            "gravity": gravity_cost,
            "slip": slip_cost,
            "turn": turn_cost,
        }
        total = (
            self.cost_weights["distance"] * components["distance"] +
            self.cost_weights["transition"] * components["transition"] +
            self.cost_weights["adhesion"] * components["adhesion"] +
            self.cost_weights["contact"] * components["contact"] +
            self.cost_weights["edge"] * components["edge"] +
            self.cost_weights["gravity"] * components["gravity"] +
            self.cost_weights["turn"] * components["turn"] +
            self.cost_weights["slip"] * components["slip"])
        return total, components, neighbor_entry_uv

    def _heuristic(self, name: str, uv: Vec2, goal: SurfaceProjection) -> float:
        point = self.surfaces[name].point_at(uv[0], uv[1])
        return self.cost_weights["heuristic"] * distance(point, goal.point)

    def astar_surface_path(
            self, start: SurfaceProjection, goal: SurfaceProjection) -> List[str]:
        if start.surface.name == goal.surface.name:
            self.last_route = RouteDiagnostics(
                "astar", [start.surface.name], 0.0, {
                    "distance": start.surface.distance_between(
                        (start.u, start.v), (goal.u, goal.v)),
                    "transition": 0.0,
                    "adhesion": 0.0,
                    "contact": 0.0,
                    "edge": self._edge_penalty(goal),
                    "gravity": 0.0,
                    "slip": 0.0,
                    "turn": 0.0,
                }, 0)
            return [start.surface.name]

        start_name = start.surface.name
        goal_name = goal.surface.name
        start_uv = (start.u, start.v)
        push_count = 0
        queue = [(
            self._heuristic(start_name, start_uv, goal),
            push_count,
            0.0,
            start_name,
            start_uv,
            None,
            [start_name],
            {"distance": 0.0, "transition": 0.0, "adhesion": 0.0,
             "contact": 0.0,
             "edge": self._edge_penalty(start), "gravity": 0.0,
             "slip": 0.0, "turn": 0.0},
        )]
        best_cost: Dict[str, float] = {start_name: 0.0}
        expanded = 0

        while queue:
            _, _, cost_so_far, current, current_uv, previous, path, components = heapq.heappop(queue)
            if cost_so_far > best_cost.get(current, float("inf")) + 1e-9:
                continue
            expanded += 1
            if current == goal_name:
                surface = self.surfaces[current]
                final_distance = surface.distance_between(
                    current_uv, (goal.u, goal.v))
                final_gravity = self._surface_load_fraction(surface, current_uv) * final_distance
                adhesion_quality = clamp(surface.adhesion_quality, 1e-3, 1.0)
                final_adhesion = (1.0 / adhesion_quality - 1.0) * final_distance
                contact_quality = clamp(surface.contact_quality, 1e-3, 1.0)
                final_contact = (1.0 / contact_quality - 1.0) * final_distance
                final_edge = self._edge_penalty(goal) + surface.edge_risk
                final_slip = surface.slip_risk * final_distance
                final_cost = (
                    self.cost_weights["distance"] * final_distance * surface.traversal_cost +
                    self.cost_weights["gravity"] * final_gravity +
                    self.cost_weights["adhesion"] * final_adhesion +
                    self.cost_weights["contact"] * final_contact +
                    self.cost_weights["edge"] * final_edge +
                    self.cost_weights["slip"] * final_slip)
                final_components = dict(components)
                final_components["distance"] += final_distance
                final_components["gravity"] += final_gravity
                final_components["adhesion"] += final_adhesion
                final_components["contact"] += final_contact
                final_components["edge"] += final_edge
                final_components["slip"] += final_slip
                self.last_route = RouteDiagnostics(
                    "astar", path, cost_so_far + final_cost,
                    final_components, expanded)
                return path

            for neighbor in self.edges.get(current, {}):
                edge_cost, edge_components, neighbor_uv = self._edge_cost(
                    current, neighbor, current_uv, goal, previous)
                new_cost = cost_so_far + edge_cost
                if new_cost >= best_cost.get(neighbor, float("inf")):
                    continue
                best_cost[neighbor] = new_cost
                next_components = dict(components)
                for key, value in edge_components.items():
                    next_components[key] = next_components.get(key, 0.0) + value
                priority = new_cost + self._heuristic(neighbor, neighbor_uv, goal)
                push_count += 1
                heapq.heappush(queue, (
                    priority, push_count, new_cost, neighbor, neighbor_uv, current,
                    path + [neighbor], next_components))

        fallback = [start_name, goal_name]
        self.last_route = RouteDiagnostics(
            "astar", fallback, float("inf"), {}, expanded)
        return fallback

    def route_points(self, start: SurfaceProjection, goal: SurfaceProjection,
                     samples_per_surface: int = 18) -> List[RoutePoint]:
        path = self.astar_surface_path(start, goal)
        if len(path) <= 1:
            return start.surface.interpolate(
                (start.u, start.v), (goal.u, goal.v), samples_per_surface)

        points: List[RoutePoint] = []
        for index, surface_name in enumerate(path):
            surface = self.surfaces[surface_name]
            if index == 0:
                local_start = (start.u, start.v)
                portal = self._portal(surface_name, path[index + 1])
                if portal is not None:
                    local_goal = portal[0]
                else:
                    projected = surface.project(goal.point)
                    local_goal = projected.u, projected.v
            elif index == len(path) - 1:
                portal = self._portal(path[index - 1], surface_name)
                if portal is not None:
                    local_start = portal[1]
                else:
                    projected = surface.project(start.point)
                    local_start = projected.u, projected.v
                local_goal = (goal.u, goal.v)
            else:
                entry = self._portal(path[index - 1], surface_name)
                exit_ = self._portal(surface_name, path[index + 1])
                if entry is not None:
                    local_start = entry[1]
                else:
                    projected = surface.project(start.point)
                    local_start = projected.u, projected.v
                if exit_ is not None:
                    local_goal = exit_[0]
                else:
                    projected = surface.project(goal.point)
                    local_goal = projected.u, projected.v
            segment = surface.interpolate(local_start, local_goal, samples_per_surface)
            if points:
                segment = segment[1:]
            points.extend(segment)
        return points


def footprint_edge_clearance(
        surface: Surface,
        base_position: Vec3,
        base_quat_xyzw: Tuple[float, float, float, float],
        wheel_offsets: Iterable[Tuple[str, float, float]]) -> float:
    clearances = [surface.project(base_position).edge_clearance]
    for _, wheel_x, wheel_y in wheel_offsets:
        rotated = quat_rotate(base_quat_xyzw, (wheel_x, wheel_y, 0.0))
        wheel_point = add(base_position, rotated)
        clearances.append(surface.project(wheel_point).edge_clearance)
    return min(clearances)


def edge_motion_scale(
        edge_clearance: float, edge_margin: float,
        min_scale: float = 0.20) -> float:
    if edge_margin <= 1e-9 or not math.isfinite(edge_clearance):
        return 1.0
    if edge_clearance < 0.0:
        return 0.0
    if edge_clearance < edge_margin:
        return clamp(edge_clearance / edge_margin, min_scale, 1.0)
    return 1.0


def surface_from_dict(data: Dict) -> Surface:
    stype = str(data.get("type", data.get("surface_type", "polygon"))).lower()
    name = str(data.get("name", data.get("id", f"surface_{id(data)}")))
    if stype in ("cylinder", "pipe"):
        surface = CylinderSurface(
            name=name,
            axis=str(data.get("axis", data.get("cylinder_axis", "x"))),
            axis_point=_vec3(data.get("axis_point", [
                data.get("axis_x", 0.0),
                data.get("axis_y", 0.0),
                data.get("axis_z", 0.0),
            ])),
            radius=float(data.get("radius", data.get("surface_radius_m", 1.0))),
            length=float(data.get("length", data.get("surface_length_m", 1.0))),
        )
    elif stype in ("polygon", "plane", "plane_polygon", "face"):
        surface = PolygonSurface(name=name, vertices=data["vertices"])
    else:
        raise ValueError(f"unsupported surface type {stype!r}")
    surface.neighbors = [str(n) for n in data.get("neighbors", [])]
    surface.adhesion_quality = clamp(float(data.get("adhesion_quality", 1.0)), 1e-3, 1.0)
    surface.contact_quality = clamp(float(data.get(
        "contact_quality", data.get("wheel_contact_quality",
                                    surface.adhesion_quality))), 1e-3, 1.0)
    surface.traversal_cost = max(0.0, float(data.get("traversal_cost", 1.0)))
    surface.edge_risk = max(0.0, float(data.get("edge_risk", 0.0)))
    surface.transition_risk = max(0.0, float(data.get("transition_risk", 0.0)))
    surface.slip_risk = max(0.0, float(data.get("slip_risk", 0.0)))
    return surface


def graph_from_yaml(path: str) -> SurfaceGraph:
    with open(path, "r") as stream:
        data = yaml.safe_load(stream) or {}
    graph_data = data.get("surface_graph", {})
    surfaces_data = data.get("surfaces", graph_data.get("surfaces", []))
    transitions = data.get("transitions", graph_data.get("transitions", []))
    cost_weights = data.get("cost_weights", graph_data.get("cost_weights", {}))
    return SurfaceGraph(
        [surface_from_dict(item) for item in surfaces_data],
        transitions,
        cost_weights=cost_weights)


def default_cylinder_graph(axis: str, axis_point: Vec3,
                           radius: float, length: float) -> SurfaceGraph:
    return SurfaceGraph([
        CylinderSurface("configured_surface", axis, axis_point, radius, length)
    ])
