import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

from adhesion_gap_sweep import ForceTable, curvature_gap_m  # noqa: E402


def test_curvature_gap_increases_on_smaller_pipe():
    wheel_width = 0.05

    large_pipe = curvature_gap_m(1.0, wheel_width, axis_alignment=0.0)
    small_pipe = curvature_gap_m(0.30, wheel_width, axis_alignment=0.0)

    assert large_pipe > 0.0
    assert small_pipe > large_pipe
    assert math.isclose(
        curvature_gap_m(0.30, wheel_width, axis_alignment=1.0),
        0.0,
        abs_tol=1.0e-12,
    )


def test_lookup_uses_radius_axis_for_pipe_force():
    table = ForceTable(ROOT / "config" / "kmw100_comsol_seed_table.csv")

    small_force, _ = table.lookup(
        gap_mm=0.0,
        thickness_mm=10.0,
        pipe_radius_m=0.30,
        contact_fraction=1.0,
    )
    large_force, _ = table.lookup(
        gap_mm=0.0,
        thickness_mm=10.0,
        pipe_radius_m=1.00,
        contact_fraction=1.0,
    )

    assert small_force < large_force
