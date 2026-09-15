"""Audit the single-aircraft motion model against a coupled acceleration envelope.

Reports, for one corridor and one lateral limit:

* the normal acceleration the reference route alone produces at each candidate
  offset, and how much of the route that already consumes;
* whether a lane change of a given size is admissible when launched at points
  along the route, and the duration required where the closed-form
  ``d_dot``/``d_ddot`` bounds are not sufficient.

This is a kinematic audit of research assumptions, not a flight-performance or
separation certification. Existing archived runs are unaffected: nothing here
changes the simulator defaults.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from capacity_policy import load_scenario  # noqa: E402
from uam_simulator.lateral_study import (  # noqa: E402
    LateralConfig, SmoothCorridorFrame, maneuver_duration,
)
from uam_simulator.motion_envelope import (  # noqa: E402
    AccelerationEnvelope, corridor_turning_report, feasible_duration,
    offset_acceleration_budget, turning_load,
)


def audit(frame, cfg, envelope, *, offsets, displacement, launch_points,
          margin_m=6000.0):
    turning = corridor_turning_report(frame, cfg, envelope, offsets)
    baseline = maneuver_duration(displacement, cfg)
    starts = np.linspace(2000.0, max(frame.length_m - margin_m, 2000.0),
                         int(launch_points))
    rows, counts = [], {}
    for q0 in starts:
        point = np.array([float(q0)])
        row = {
            "q_m": float(q0),
            "route_turning_normal_mps2": float(np.ravel(np.abs(
                turning_load(frame, point, 0.0, cfg.cruise_mps)))[0]),
            "lateral_budget_at_launch_mps2": float(np.ravel(
                offset_acceleration_budget(frame, point, 0.0, cfg.cruise_mps,
                                           envelope))[0]),
        }
        report = feasible_duration(frame, cfg, envelope, q0=float(q0), t0=0.0,
                                   source=0.0, target=float(displacement))
        row.update({key: report[key] for key in
                    ("duration_s", "status", "max_utilisation", "duration_cap_s")})
        counts[report["status"]] = counts.get(report["status"], 0) + 1
        rows.append(row)
    admissible = [row for row in rows if row["duration_s"]]
    return {
        "cruise_mps": cfg.cruise_mps,
        "displacement_m": float(displacement),
        "baseline_duration_s": baseline,
        "lateral_accel_max_mps2": envelope.lateral_accel_max_mps2,
        "longitudinal_accel_max_mps2": envelope.longitudinal_accel_max_mps2,
        "longitudinal_decel_max_mps2": envelope.longitudinal_decel_max_mps2,
        "envelope_provenance": envelope.provenance,
        "coupling_mode": envelope.coupling_mode,
        "route_normal_accel_max_mps2": envelope.route_normal_accel_max_mps2,
        "route_bank_angle_max_deg": envelope.route_bank_angle_max_deg,
        "route_turning": turning,
        "launch_points": rows,
        "status_counts": counts,
        "admissible_fraction": len(admissible) / len(rows) if rows else 0.0,
        "max_admissible_duration_s": max((row["duration_s"] for row in admissible),
                                         default=None),
        "coupling_is_an_assumption": (
            "the ellipse is a coupling hypothesis; the automotive friction circle "
            "arises from tyre-road contact and does not by itself establish that an "
            "eVTOL shares one budget across axes"),
        "certified_flight_performance": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario",
                        default=str(ROOT / "scenarios/airport_to_airport/scenario.json"))
    parser.add_argument("--lateral-accel-mps2", type=float, default=0.981,
                        help="0.981 is 0.1 g; 1.962 is 0.2 g. Bounds the lane-change component")
    parser.add_argument("--coupling-mode", choices=("maneuver_only", "total"),
                        default="maneuver_only",
                        help="whether the lateral limit binds the maneuver only "
                             "or the total normal acceleration")
    parser.add_argument("--route-normal-accel-mps2", type=float, default=4.575,
                        help="steady coordinated-turn allowance; 4.575 is a 25 deg bank")
    parser.add_argument("--displacement-m", type=float, default=100.0)
    parser.add_argument("--offsets-m", type=float, nargs="+",
                        default=[-300.0, 0.0, 300.0])
    parser.add_argument("--launch-points", type=int, default=16)
    parser.add_argument("--output", default=None)
    args = parser.parse_args(argv)

    scenario = load_scenario(args.scenario)
    cfg = LateralConfig()
    frame = SmoothCorridorFrame(scenario.corridor, cfg.center_control_step_m)
    envelope = AccelerationEnvelope(lateral_accel_max_mps2=args.lateral_accel_mps2,
                                    route_normal_accel_max_mps2=args.route_normal_accel_mps2,
                                    coupling_mode=args.coupling_mode)
    report = audit(frame, cfg, envelope, offsets=args.offsets_m,
                   displacement=args.displacement_m,
                   launch_points=args.launch_points)

    turning = report["route_turning"]
    print(f"corridor minimum turn radius : {turning['minimum_turn_radius_m']:.1f} m")
    print(f"coupling mode                : {envelope.coupling_mode}")
    print(f"lane-change lateral limit    : {envelope.lateral_accel_max_mps2:.3f} m/s^2")
    print(f"route steady-turn allowance  : {envelope.route_normal_accel_max_mps2:.3f} m/s^2 "
          f"({envelope.route_bank_angle_max_deg:.1f} deg bank)")
    print(f"baseline {args.displacement_m:.0f} m duration     : "
          f"{report['baseline_duration_s']:.3f} s")
    print("\nroute turning load by candidate offset")
    print(f"{'offset':>8} {'max a_N':>9} {'p95':>8} {'max bank':>9} {'over route':>11}")
    for row in turning["rows"]:
        print(f"{row['offset_m']:+8.0f} {row['max_turning_normal_mps2']:9.3f} "
              f"{row['p95_turning_normal_mps2']:8.3f} "
              f"{row['max_bank_angle_deg']:8.2f}° "
              f"{100 * row['fraction_over_route_allowance']:10.2f}%")
    print("\nlane-change admissibility by launch point")
    print(f"{'q (km)':>8} {'a_N turn':>9} {'budget':>8} {'T (s)':>9}  status")
    for row in report["launch_points"]:
        duration = f"{row['duration_s']:9.2f}" if row["duration_s"] else f"{'none':>9}"
        print(f"{row['q_m'] / 1000:8.1f} {row['route_turning_normal_mps2']:9.3f} "
              f"{row['lateral_budget_at_launch_mps2']:8.3f} {duration}  {row['status']}")
    print(f"\nstatus counts: {report['status_counts']}")
    print(f"admissible fraction: {report['admissible_fraction']:.2%}")

    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2))
        print(f"\nwrote {path}")
    return report


if __name__ == "__main__":
    main()
