"""Independently verify a saved Bay Area longitudinal-control run."""
from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path


def load_json(path: Path):
    with (gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz"
          else path.open(encoding="utf-8")) as stream:
        return json.load(stream)


def verify(run_dir: Path) -> dict:
    run_dir = run_dir.resolve()
    config = load_json(run_dir / "resolved_config.json")
    summary = load_json(run_dir / "summary.json")
    trace = load_json(run_dir / "trace.json.gz")
    observations = load_json(run_dir / "observations.json.gz")
    events = load_json(run_dir / "events.json")
    failures = []
    updates = [row for row in events if row["status"] == "policy_update"]

    parameters = config["parameters"]
    expected = {
        "threshold_db": -2.0, "radio_s": 2.0, "policy_s": 5.0,
        "exposure_c": 0.05, "exposure_r": 0.10,
        "persistence_k": 3, "d0_m": 152.4,
        "target_c_m": 1319.9, "target_r_m": 2069.9, "target_f_m": 3569.9,
    }
    for key, value in expected.items():
        if parameters.get(key) != value:
            failures.append(f"config:{key}")
    if config["study"] == "bay_area_sparse_window_sweep":
        if parameters.get("window_s") not in (30.0, 60.0, 90.0, 120.0):
            failures.append("window_sweep:window_s")
        if config["traffic"].get("entry_headway_s") not in (32.0, 90.0):
            failures.append("window_sweep:entry_headway_s")
        if config["traffic"].get("aircraft_count") != 5:
            failures.append("window_sweep:request_count")
        if parameters.get("group_radius_m") != 3569.9:
            failures.append("window_sweep:group_radius_m")
        if config["case"].get("lane_change_allowed") is not False:
            failures.append("window_sweep:lane_change_allowed")
    elif config["study"] == "bay_area_longitudinal_control_90s":
        if parameters.get("window_s") != 90.0:
            failures.append("config:window_s")
    elif parameters.get("window_s") != 30.0:
        failures.append("config:window_s")
    if summary["scheduled_aircraft"] != config["traffic"]["aircraft_count"]:
        failures.append("scheduled_aircraft_config")
    if config["study"] in ("bay_area_longitudinal_control",
                            "bay_area_longitudinal_control_90s"):
        large_expected = {
            "aircraft_count": 93,
            "entry_headway_s": 32.0,
            "comparison_horizon_s": 2972.56,
        }
        for key, value in large_expected.items():
            if config["traffic"].get(key) != value:
                failures.append(f"large_stream:{key}")
        if parameters.get("group_radius_m") is not None:
            failures.append("large_stream:group_radius_m")
        if config["case"].get("lane_change_allowed") is not False:
            failures.append("large_stream:lane_change_allowed")
        common = summary["metrics"].get("common_horizon_comparison")
        if not common or common.get("horizon_s") != 2972.56:
            failures.append("large_stream:common_horizon")
        else:
            selected = [row for row in updates if float(row["t_s"]) <= 2972.56 + 1e-9]
            counts = {key: sum(row["policy"] == key for row in selected) for key in "CRF"}
            if common.get("policy_update_counts") != counts:
                failures.append("large_stream:common_horizon_counts")
    if any(abs(float(row["t_s"]) / 2 - round(float(row["t_s"]) / 2)) > 1e-8
           for row in observations):
        failures.append("radio_clock")
    if any(abs(float(row["t_s"]) / 5 - round(float(row["t_s"]) / 5)) > 1e-8
           for row in updates):
        failures.append("policy_clock")

    by_aircraft = {}
    for row in updates:
        by_aircraft.setdefault(row["aircraft_id"], []).append(row)
    for identifier, rows in by_aircraft.items():
        state = rows[0]["previous_policy"]
        candidate, count = state, 0
        for row in rows:
            raw = row["raw_policy"]
            if raw == state:
                candidate, count = state, 0
            else:
                if raw == candidate:
                    count += 1
                else:
                    candidate, count = raw, 1
                if count >= 3:
                    state, candidate, count = raw, raw, 0
            if row["policy"] != state:
                failures.append(f"persistence:{identifier}:{row['t_s']}")
                break

    if summary["completed_aircraft"] != summary["scheduled_aircraft"]:
        failures.append("mission_completion")
    if summary["status"] not in ("all_exited", "completed_horizon"):
        failures.append("run_status")
    if any(abs(float(row["offset_m"])) > 1e-7 or row["target_lane"] is not None
           for row in trace):
        failures.append("lateral_motion_disabled")
    if any(float(row["total_speed_mps"]) < 30 - 1e-7
           or float(row["total_speed_mps"]) > 80 + 1e-7 for row in trace):
        failures.append("speed_bounds")
    if any(float(row["peak_eta_step"]) > 1 + 1e-6 for row in trace):
        failures.append("joint_envelope")
    if any(bool(row["nmac_sampled"]) for row in trace):
        failures.append("nmac")
    if any(not math.isfinite(float(row["q_m"])) for row in trace):
        failures.append("nonfinite_state")
    if any(event["status"] in ("execution_rejected", "no_verified_action") for event in events):
        failures.append("execution_rejection")

    result = {
        "run_id": run_dir.name,
        "passed": not failures,
        "failures": failures,
        "checks": {
            "trace_rows": len(trace),
            "radio_observations": len(observations),
            "policy_updates": len(updates),
            "completed_aircraft": summary["completed_aircraft"],
        },
    }
    (run_dir / "validation.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    result = verify(args.run_dir)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
