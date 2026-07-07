#!/usr/bin/env python3
"""Estimate radius-dependent magnetic adhesion from air gap and lookup data.

This is the lightweight engineering version of the paper workflow for pipe
studies: compute the effective gap seen by the magnet, including pipe curvature,
then evaluate the force table over gap, wall thickness, pipe radius, and contact
fraction. Replace the CSV with measured/FEM/MoI values for the final hardware.
"""

import argparse
import csv
import math
from pathlib import Path


def _bounds(values, query):
    if query <= values[0]:
        return values[0], values[0], query < values[0]
    if query >= values[-1]:
        return values[-1], values[-1], query > values[-1]
    for low, high in zip(values, values[1:]):
        if low <= query <= high:
            return low, high, False
    return values[-1], values[-1], True


class ForceTable:
    def __init__(self, path):
        self.path = Path(path)
        self.forces = {}
        self.gaps = set()
        self.thicknesses = set()
        self.radii = set()
        self.contacts = set()
        with self.path.open(newline="") as f:
            reader = csv.DictReader(f)
            fields = set(reader.fieldnames or [])
            if "gap_mm" not in fields or "force_n" not in fields:
                raise ValueError("force table needs gap_mm and force_n columns")
            if "wall_thickness_mm" not in fields and "thickness_mm" not in fields:
                raise ValueError("force table needs wall_thickness_mm or thickness_mm")
            self.has_radius = "pipe_radius_m" in fields
            self.has_contact = "contact_fraction" in fields
            for row in reader:
                gap = float(row["gap_mm"])
                thickness_key = (
                    "wall_thickness_mm"
                    if row.get("wall_thickness_mm", "") != ""
                    else "thickness_mm"
                )
                thickness = float(row[thickness_key])
                radius = float(row.get("pipe_radius_m") or 0.0)
                contact = float(row.get("contact_fraction") or 1.0)
                force = float(row["force_n"])
                self.gaps.add(gap)
                self.thicknesses.add(thickness)
                self.radii.add(radius)
                self.contacts.add(contact)
                self.forces[(gap, thickness, radius, contact)] = force

        self.gaps = sorted(self.gaps)
        self.thicknesses = sorted(self.thicknesses)
        self.radii = sorted(self.radii)
        self.contacts = sorted(self.contacts)
        if not self.forces:
            raise ValueError(f"{self.path} contains no force samples")

    def lookup(self, gap_mm, thickness_mm, pipe_radius_m, contact_fraction):
        g0, g1, gap_clamped = _bounds(self.gaps, gap_mm)
        t0, t1, thickness_clamped = _bounds(self.thicknesses, thickness_mm)
        r0, r1, radius_clamped = _bounds(self.radii, pipe_radius_m)
        c0, c1, contact_clamped = _bounds(self.contacts, contact_fraction)
        axes = (
            (g0, g1, 0.0 if g0 == g1 else (gap_mm - g0) / (g1 - g0)),
            (t0, t1, 0.0 if t0 == t1 else (thickness_mm - t0) / (t1 - t0)),
            (r0, r1, 0.0 if r0 == r1 else (pipe_radius_m - r0) / (r1 - r0)),
            (c0, c1, 0.0 if c0 == c1 else (contact_fraction - c0) / (c1 - c0)),
        )
        total = 0.0
        for gi, gv in enumerate((g0, g1)):
            gw = (1.0 - axes[0][2]) if gi == 0 else axes[0][2]
            for ti, tv in enumerate((t0, t1)):
                tw = (1.0 - axes[1][2]) if ti == 0 else axes[1][2]
                for ri, rv in enumerate((r0, r1)):
                    rw = (1.0 - axes[2][2]) if ri == 0 else axes[2][2]
                    for ci, cv in enumerate((c0, c1)):
                        cw = (1.0 - axes[3][2]) if ci == 0 else axes[3][2]
                        total += self.forces[(gv, tv, rv, cv)] * gw * tw * rw * cw
        if not self.has_contact:
            total *= contact_fraction
        return total, {
            "gap": gap_clamped,
            "thickness": thickness_clamped,
            "pipe_radius": radius_clamped and self.has_radius,
            "contact_fraction": contact_clamped and self.has_contact,
        }


def curvature_gap_m(pipe_radius_m, wheel_width_m, axis_alignment):
    """Sagitta gap from a flat magnet/wheel face over a cylindrical surface."""
    axis_alignment = max(0.0, min(1.0, abs(axis_alignment)))
    span = 0.5 * wheel_width_m * math.sqrt(max(0.0, 1.0 - axis_alignment ** 2))
    if span <= 0.0:
        return 0.0
    if span >= pipe_radius_m:
        return float("inf")
    return pipe_radius_m - math.sqrt(pipe_radius_m ** 2 - span ** 2)


def contact_fraction_from_gap(effective_gap_mm, max_gap_mm, min_fraction):
    if effective_gap_mm > max_gap_mm:
        return 0.0
    span = max(1.0e-9, max_gap_mm)
    return max(min_fraction, min(1.0, 1.0 - effective_gap_mm / span))


def parse_float_list(value):
    return [float(item) for item in value.split(",") if item.strip()]


def main():
    parser = argparse.ArgumentParser(
        description="Sweep magnetic adhesion force over pipe radii and gaps.")
    parser.add_argument(
        "--force-table",
        default="src/robot_3d3s/config/kmw100_comsol_seed_table.csv")
    parser.add_argument("--radii-m", default="0.20,0.30,0.45,0.55,1.00,1.20")
    parser.add_argument("--radial-gaps-mm", default="0,1,3")
    parser.add_argument("--wall-thickness-mm", type=float, default=9.0)
    parser.add_argument("--wheel-width-m", type=float, default=0.05)
    parser.add_argument(
        "--axis-alignment", type=float, default=0.0,
        help="abs(dot(wheel spin axis, pipe axis)); 0 is worst curvature case.")
    parser.add_argument("--contact-fraction", type=float, default=-1.0)
    parser.add_argument("--min-contact-fraction", type=float, default=0.10)
    parser.add_argument("--max-gap-mm", type=float, default=6.0)
    parser.add_argument("--wheels", type=int, default=3)
    parser.add_argument("--mass-kg", type=float, default=25.0)
    parser.add_argument("--mu", type=float, default=1.0)
    parser.add_argument("--safety-factor", type=float, default=1.5)
    parser.add_argument("--output-csv", default="")
    args = parser.parse_args()

    table = ForceTable(args.force_table)
    radii = parse_float_list(args.radii_m)
    radial_gaps = parse_float_list(args.radial_gaps_mm)
    rows = []
    weight = args.mass_kg * 9.81

    for radius in radii:
        curv_gap = curvature_gap_m(radius, args.wheel_width_m, args.axis_alignment)
        for radial_gap_mm in radial_gaps:
            if math.isinf(curv_gap):
                effective_gap_mm = float("inf")
                contact_fraction = 0.0
                force_per_wheel = 0.0
                clamped = {}
            else:
                effective_gap_mm = radial_gap_mm + curv_gap * 1000.0
                if args.contact_fraction >= 0.0:
                    contact_fraction = max(0.0, min(1.0, args.contact_fraction))
                else:
                    contact_fraction = contact_fraction_from_gap(
                        effective_gap_mm,
                        args.max_gap_mm,
                        args.min_contact_fraction)
                force_per_wheel, clamped = table.lookup(
                    effective_gap_mm,
                    args.wall_thickness_mm,
                    radius,
                    contact_fraction)
            sum_n = args.wheels * force_per_wheel
            friction_margin = args.mu * sum_n / max(1.0e-9, args.safety_factor * weight)
            rows.append({
                "pipe_radius_m": radius,
                "radial_gap_mm": radial_gap_mm,
                "curvature_gap_mm": curv_gap * 1000.0 if not math.isinf(curv_gap) else "inf",
                "effective_gap_mm": effective_gap_mm if not math.isinf(effective_gap_mm) else "inf",
                "wall_thickness_mm": args.wall_thickness_mm,
                "contact_fraction": contact_fraction,
                "force_per_wheel_n": force_per_wheel,
                "sum_force_n": sum_n,
                "friction_margin_x": friction_margin,
                "lookup_clamped": ",".join(k for k, v in clamped.items() if v),
            })

    fields = list(rows[0].keys()) if rows else []
    print(",".join(fields))
    for row in rows:
        print(",".join(str(row[field]) for field in fields))

    if args.output_csv:
        with Path(args.output_csv).open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    main()
