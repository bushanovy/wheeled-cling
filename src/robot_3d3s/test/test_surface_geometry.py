import math
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from surface_geometry import (  # noqa: E402
    CylinderSurface,
    PolygonSurface,
    SurfaceGraph,
    edge_motion_scale,
    footprint_edge_clearance,
    graph_from_yaml,
)


def test_cylinder_projection_clamps_axis_and_wraps_angle():
    surface = CylinderSurface(
        "pipe", "x", (0.0, 0.0, 1.0), radius=0.5, length=2.0)

    projection = surface.project((2.0, 0.0, 1.5))

    assert projection.u == 1.0
    assert math.isclose(projection.v, math.pi / 2.0)
    assert math.isclose(projection.point[0], 1.0)
    assert math.isclose(projection.point[1], 0.0, abs_tol=1e-12)
    assert math.isclose(projection.point[2], 1.5)
    assert projection.edge_clearance == 0.0
    assert not projection.inside


def test_polygon_projection_clamps_to_nearest_edge():
    surface = PolygonSurface("panel", [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    ])

    inside = surface.project((0.5, 0.5, 0.3))
    outside = surface.project((1.2, 0.5, 0.0))

    assert inside.inside
    assert inside.edge_clearance > 0.49
    assert math.isclose(inside.point[2], 0.0)
    assert not outside.inside
    assert math.isclose(outside.point[0], 1.0)
    assert math.isclose(outside.point[1], 0.5)
    assert outside.edge_clearance <= 0.0


def test_surface_graph_loads_yaml_and_routes(tmp_path):
    graph_yaml = tmp_path / "graph.yaml"
    graph_yaml.write_text(
        """
surfaces:
  - name: wall
    type: polygon
    vertices:
      - [0.0, 0.0, 0.0]
      - [1.0, 0.0, 0.0]
      - [1.0, 1.0, 0.0]
      - [0.0, 1.0, 0.0]
    neighbors: [pipe]
  - name: pipe
    type: cylinder
    axis: z
    axis_point: [2.0, 0.0, 0.5]
    radius: 0.5
    length: 1.0
transitions:
  - from: wall
    to: pipe
    cost: 2.0
    from_uv: [1.0, 0.5]
    to_uv: [0.5, 0.0]
""",
        encoding="utf-8",
    )

    graph = graph_from_yaml(str(graph_yaml))
    start = graph.surfaces["wall"].project((0.2, 0.2, 0.1))
    goal = graph.surfaces["pipe"].project((2.5, 0.0, 0.8))
    route = graph.route_points(start, goal, samples_per_surface=4)

    assert graph.shortest_surface_path("wall", "pipe") == ["wall", "pipe"]
    assert route[0].surface.name == "wall"
    assert route[3].surface.name == "wall"
    assert route[3].point == graph.surfaces["wall"].point_at(1.0, 0.5)
    assert route[-1].surface.name == "pipe"
    assert route[-1].point == goal.point


def test_surface_graph_picks_nearest_surface():
    wall = PolygonSurface("wall", [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    ])
    pipe = CylinderSurface(
        "pipe", "z", (4.0, 0.0, 0.5), radius=0.5, length=1.0)
    graph = SurfaceGraph([wall, pipe])

    projection = graph.closest_projection((4.5, 0.0, 0.6))

    assert projection.surface.name == "pipe"


def test_astar_chooses_lower_risk_transition_path():
    start_surface = PolygonSurface("start", [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
    ])
    risky = PolygonSurface("risky", [
        (1.0, 0.0, 0.0), (2.0, 0.0, 0.0),
        (2.0, 1.0, 0.0), (1.0, 1.0, 0.0),
    ])
    safe = PolygonSurface("safe", [
        (1.0, -1.5, 0.0), (2.0, -1.5, 0.0),
        (2.0, -0.5, 0.0), (1.0, -0.5, 0.0),
    ])
    goal_surface = PolygonSurface("goal", [
        (2.0, 0.0, 0.0), (3.0, 0.0, 0.0),
        (3.0, 1.0, 0.0), (2.0, 1.0, 0.0),
    ])
    graph = SurfaceGraph(
        [start_surface, risky, safe, goal_surface],
        transitions=[
            {"from": "start", "to": "risky", "cost": 1.0, "risk": 8.0,
             "point": [1.0, 0.5, 0.0]},
            {"from": "risky", "to": "goal", "cost": 1.0, "risk": 8.0,
             "point": [2.0, 0.5, 0.0]},
            {"from": "start", "to": "safe", "cost": 1.0, "risk": 0.0,
             "point": [1.0, 0.0, 0.0]},
            {"from": "safe", "to": "goal", "cost": 1.0, "risk": 0.0,
             "point": [2.0, 0.0, 0.0]},
        ],
        cost_weights={"transition": 3.0, "heuristic": 0.5},
    )

    start = start_surface.project((0.2, 0.5, 0.0))
    goal = goal_surface.project((2.8, 0.5, 0.0))

    path = graph.astar_surface_path(start, goal)

    assert path == ["start", "safe", "goal"]
    assert graph.last_route.algorithm == "astar"
    assert graph.last_route.components["transition"] < 10.0


def test_footprint_edge_clearance_uses_all_three_wheels():
    surface = PolygonSurface("panel", [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
    ])
    wheel_offsets = [
        ("front", 0.2, 0.0),
        ("left", -0.1, 0.17),
        ("right", -0.1, -0.17),
    ]
    identity = (0.0, 0.0, 0.0, 1.0)

    centered = footprint_edge_clearance(
        surface, (0.5, 0.5, 0.0), identity, wheel_offsets)
    near_edge = footprint_edge_clearance(
        surface, (0.9, 0.5, 0.0), identity, wheel_offsets)

    assert centered > 0.29
    assert near_edge <= 0.01


def test_edge_motion_scale_slows_and_holds_near_edges():
    assert edge_motion_scale(0.20, 0.10, 0.25) == 1.0
    assert edge_motion_scale(0.05, 0.10, 0.25) == 0.5
    assert edge_motion_scale(0.01, 0.10, 0.25) == 0.25
    assert edge_motion_scale(-0.01, 0.10, 0.25) == 0.0
