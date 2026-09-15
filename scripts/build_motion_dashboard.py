"""Build the motion-control dashboard bundle from a single-stream corridor run.

The bundle holds both cases of the study (stay at the centre with longitudinal
control, and moving within the 3 x 3 lateral/altitude grid), sampled every
``--frame-s`` seconds, with the capacity traces, lane-change events and the
calibrated settings. It is read by ``dashboard/motion.html``.

    python scripts/build_motion_dashboard.py \
        --run research/dynamic-transitions/runs/R0062 \
        --scenario scenarios/airport_to_airport/scenario.json \
        --output dashboard/data/airport_to_airport_motion.js
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
from pathlib import Path

import numpy as np
from pyproj import Transformer

POLICIES = "CRF"
CASE_LABELS = {
    "fixed_centerline": "Stay at centre · longitudinal control",
    "spatial_grid": "Move within 3 × 3 · longitudinal control + lane changes",
}


def read_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def lower_tail(values, rho=0.95):
    ordered = np.sort(np.asarray(values, float))
    return float(ordered[int(np.floor((1 - rho) * (len(ordered) - 1)))])


def load_events(case_dir):
    events = json.loads((case_dir / "events.json").read_text(encoding="utf-8"))
    return events if isinstance(events, list) else events.get("events", [])


def scenario_geometry(scenario_path):
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    site_rows = read_csv((scenario_path.parent / scenario["base_stations"]["path"]).resolve())
    active = scenario["base_stations"].get("active_site_ids")
    if active:
        by_id = {row["public_site_id"]: row for row in site_rows}
        site_rows = [by_id[site_id] for site_id in active]
    route = json.loads((scenario_path.parent / scenario["corridor"]["path"]).read_text(encoding="utf-8"))
    route = route["features"][0]["geometry"]["coordinates"]
    to_metric = Transformer.from_crs("EPSG:4326", "EPSG:26910", always_xy=True)
    array = np.asarray(route, dtype=float)
    x, y = to_metric.transform(array[:, 0], array[:, 1])
    s = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
    stations = []
    for row in site_rows:
        sx, sy = to_metric.transform(float(row["longitude"]), float(row["latitude"]))
        stations.append({"id": row["public_site_id"], "lat": float(row["latitude"]), "lon": float(row["longitude"]),
                         "x_m": float(sx), "y_m": float(sy), "height_m": float(row["antenna_height_m"]),
                         "site_class": row["fcc_site_type"], "physical_form": row["physical_form"]})
    route_metric = [{"s_m": float(s[i]), "x_m": float(x[i]), "y_m": float(y[i]), "lon": float(array[i, 0]),
                     "lat": float(array[i, 1])} for i in range(len(array))]
    return scenario, route, route_metric, stations


def build_case(run_dir, case, frame_s, to_geo, controllers):
    case_dir = run_dir / case
    summary = json.loads((case_dir / "summary.json").read_text(encoding="utf-8"))
    summary = summary[0] if isinstance(summary, list) else summary
    events = load_events(case_dir)
    entries = sorted((e for e in events if e.get("status") == "corridor_entry"), key=lambda e: (e["t_s"], e["aircraft_id"]))
    exits = {e["aircraft_id"]: e["t_s"] for e in events if e.get("status") == "exit"}
    index = {e["aircraft_id"]: i for i, e in enumerate(entries)}
    aircraft = [{"id": e["aircraft_id"], "entry": e["t_s"], "exit": exits.get(e["aircraft_id"]),
                 "entry_delay_s": round(float(summary["entry_delays_s"].get(e["aircraft_id"], 0.0)), 2)} for e in entries]
    trace = json.load(gzip.open(case_dir / "trace.json.gz", "rt", encoding="utf-8"))
    frames = {}
    for r in trace:
        t = float(r["t_s"])
        k = round(t / frame_s)
        if abs(t - k * frame_s) > 1e-6:
            continue
        mode = r["controller"]
        if mode not in controllers:
            controllers.append(mode)
        lon, lat = to_geo.transform(r["xyz"][0], r["xyz"][1])
        gaps = [float(leader[1]) for leader in r.get("leaders") or []]
        frames.setdefault(k, []).append([
            index[r["aircraft_id"]], round(lat * 1e5), round(lon * 1e5), round(r["xyz"][0]), round(r["xyz"][1]),
            round(r["xyz"][2]), round(r["offset_m"]), round(r["v_mps"] * 10), POLICIES.index(r["policy"]),
            controllers.index(mode), round(min(gaps)) if gaps else -1, 0 if r.get("target_lane") is None else 1,
            int(r["lane"])])
    capacity = [[float(row["timestamp_s"]), round(float(row["q_mix_uam_h"]), 3), int(row["n_C"]), int(row["n_R"]), int(row["n_F"])]
                for row in read_csv(case_dir / "capacity_trace.csv")]
    changes = [[e["t_s"], index[e["aircraft_id"]], int(e["source_lane"]), int(e["target_lane"]), float(e["duration_s"])]
               for e in events if e.get("status") == "change_started"]
    metrics = summary["policy_and_longitudinal_metrics"]
    return {
        "label": CASE_LABELS.get(case, case),
        "lane_change_allowed": bool(summary["lane_change_allowed"]),
        "aircraft": aircraft,
        "frames": [{"t": k * frame_s, "rows": sorted(rows)} for k, rows in sorted(frames.items())],
        "capacity": capacity,
        "changes": changes,
        "stats": {
            "completed": summary["completed_requests"], "scheduled": summary["scheduled_requests"],
            "end_s": summary["end_s"], "policy_shares": metrics["policy_shares"],
            "speed_reversals_per_flight": metrics.get("mean_speed_direction_reversals_per_flight"),
            "sampled_nmac": metrics["nmac_sampled"],
            "held_at_entry": sum(1 for v in summary["entry_delays_s"].values() if v),
            "longest_entry_hold_s": max(summary["entry_delays_s"].values()),
            "completed_lane_changes": summary.get("lane_change_metrics", {}).get("completed_lane_changes", 0),
        },
        "_events": events,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-s", type=float, default=5.0)
    args = parser.parse_args()
    run_dir = args.run.resolve()
    validation = json.loads((run_dir / "validation.json").read_text(encoding="utf-8"))
    if validation.get("status") not in ("pass", "passed"):
        raise ValueError(f"refusing to build from a run that did not pass validation: {run_dir}")
    cfg = json.loads((run_dir / "resolved_config.json").read_text(encoding="utf-8"))
    scenario, route, route_metric, stations = scenario_geometry(args.scenario.resolve())
    to_geo = Transformer.from_crs("EPSG:26910", "EPSG:4326", always_xy=True)
    controllers: list[str] = []
    cases = {c["id"]: build_case(run_dir, c["id"], args.frame_s, to_geo, controllers) for c in cfg["cases"]}

    # compared window: corridor full in every case and still admitting aircraft
    first_exit = max(min(e["t_s"] for e in c["_events"] if e.get("status") == "exit") for c in cases.values())
    last_entry = min(max(e["t_s"] for e in c["_events"] if e.get("status") == "corridor_entry") for c in cases.values())
    for c in cases.values():
        q = [row[1] for row in c["capacity"] if first_exit <= row[0] <= last_entry]
        c["stats"]["window_mean_uam_h"] = float(np.mean(q))
        c["stats"]["window_q95_uam_h"] = lower_tail(q, cfg["capacity"]["reliability_rho"])
        del c["_events"]

    p = cfg["parameters"]
    spacing = {policy: p["d0_m"] + p[f"tau_{policy.lower()}_s"] * p["cruise_mps"] + p["buffer_s2_per_m"] * p["cruise_mps"] ** 2
               for policy in POLICIES}
    bundle = {
        "summary": {
            "schema_version": 1, "run_id": run_dir.name, "study": cfg["study"], "scenario_id": scenario["scenario_id"],
            "display": scenario.get("display", {}), "radio": scenario["radio"], "frame_s": args.frame_s,
            "parameters": {key: p[key] for key in ("threshold_db", "radio_s", "policy_s", "window_s", "persistence_k",
                                                  "exposure_c", "exposure_r", "group_mode", "cruise_mps", "speed_min_mps",
                                                  "speed_max_mps", "acceleration_limit_mps2", "deceleration_limit_mps2",
                                                  "d0_m", "tau_c_s", "tau_r_s", "tau_f_s", "buffer_s2_per_m")},
            "spacing_m": spacing, "envelope": cfg["envelope"],
            "grid": cfg["traffic"]["flow_points"], "entry_flow_index": cfg["traffic"]["entry_flow_index"],
            "global_headway_s": cfg["traffic"]["global_headway_s"], "requests": cfg["traffic"]["global_entry_count"],
            "compared_window_s": [first_exit, last_entry], "controllers": controllers,
            "row_fields": ["aircraft", "lat_e5", "lon_e5", "x_m", "y_m", "altitude_m", "offset_m", "speed_dmps",
                           "policy", "controller", "gap_m", "moving", "cell"],
            "validation": validation.get("status"),
        },
        "route": route, "route_metric": route_metric, "stations": stations, "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("window.UAM_MOTION_DATA = " + json.dumps(bundle, separators=(",", ":")) + ";\n", encoding="utf-8")
    print(f"motion bundle: {args.output} ({args.output.stat().st_size / 1e6:.2f} MB)")
    for case, c in cases.items():
        print(case, len(c["frames"]), "frames", c["stats"])


if __name__ == "__main__":
    main()
