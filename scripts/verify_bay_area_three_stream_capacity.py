#!/usr/bin/env python3
"""Independently audit a same-altitude three-origin-stream capacity archive."""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from capacity_policy import load_scenario  # noqa: E402
from uam_simulator.geographic_traffic import SmoothCorridorFrame  # noqa: E402
from uam_simulator.policy_motion import BSRadio  # noqa: E402


def read(path: Path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def reliability_floor(values, rho):
    ordered = np.sort(np.asarray(values, float))
    index = int(np.floor((1.0 - rho) * len(ordered) + 1e-12))
    return float(ordered[min(max(index, 0), len(ordered) - 1)])


def audit_case(run: Path, spec: dict, case: dict, scenario, frame) -> dict:
    folder = run / case["id"]
    trace = read(folder / "trace.json.gz")
    observations = read(folder / "observations.json.gz")
    events = read(folder / "events.json")
    summary = read(folder / "summary.json")
    cfg = spec["parameters"]
    traffic = spec["traffic"]
    offsets = np.asarray(traffic["lateral_offsets_m"], float)
    entries = spec["entries"]
    origin = {row["aircraft_id"]: int(row["lane"]) for row in entries}
    failures = []

    if summary["status"] != "all_exited":
        failures.append("not_all_exited")
    if summary["completed_requests"] != summary["scheduled_requests"]:
        failures.append("incomplete_requests")
    if summary["active_aircraft"] or summary["pending_aircraft"]:
        failures.append("remaining_aircraft")

    q = np.asarray([row["q_m"] for row in trace], float)
    d = np.asarray([row["offset_m"] for row in trace], float)
    center, _, normal, _, _ = frame.frame(q)
    archived_xyz = np.asarray([row["xyz"] for row in trace], float)
    expected_xy = center + d[:, None] * normal
    position_error = float(np.max(np.linalg.norm(
        expected_xy - archived_xyz[:, :2], axis=1)))
    altitude_error = float(np.max(abs(archived_xyz[:, 2] - traffic["altitude_m"])))
    if position_error > 1e-6 or altitude_error > 1e-8:
        failures.append("geographic_position")

    trace_index = {(row["aircraft_id"], round(float(row["t_s"]), 8)): row
                   for row in trace}
    obs_xyz = np.asarray([
        trace_index[(row["aircraft_id"], round(float(row["t_s"]), 8))]["xyz"]
        for row in observations], float)
    radio = BSRadio(scenario.base_stations, scenario.radio)
    recomputed = radio(obs_xyz)
    sinr_error = float(np.max(abs(
        recomputed["sinr_db"] - np.asarray([row["sinr_db"]
                                             for row in observations]))))
    serving_match = bool(np.array_equal(
        recomputed["serving_bs"], [row["serving_bs"] for row in observations]))
    if sinr_error > 1e-8 or not serving_match:
        failures.append("radio_reconstruction")

    metrics = summary["policy_and_longitudinal_metrics"]
    if metrics["nmac_sampled"] or any(row["nmac_sampled"] for row in trace):
        failures.append("sampled_nmac")
    if metrics["minimum_sampled_horizontal_separation_m"] < cfg["horizontal_separation_m"] - 1e-7:
        failures.append("sampled_horizontal_separation")
    if (metrics["minimum_total_speed_mps"] < cfg["speed_min_mps"] - 1e-7
            or metrics["maximum_total_speed_mps"] > cfg["speed_max_mps"] + 1e-7):
        failures.append("speed_envelope")
    if metrics["maximum_joint_envelope_utilisation"] > 1 + 1e-6:
        failures.append("acceleration_envelope")

    starts = [event for event in events if event["status"] == "change_started"]
    completed = [event for event in events
                 if event["status"] == "transition_completed"]
    if case["lane_change_allowed"]:
        if any(abs(int(event["source_lane"]) - int(event["target_lane"])) != 1
               for event in starts):
            failures.append("nonadjacent_lane_change")
        if len(starts) != len(completed):
            failures.append("incomplete_lane_change")
        if any(event.get("worsened_aircraft") for event in starts):
            failures.append("accepted_worsening")
    else:
        if starts or completed or any(row["target_lane"] is not None for row in trace):
            failures.append("motion_in_no_change_case")
        if any(abs(float(row["offset_m"]) - offsets[origin[row["aircraft_id"]]]) > 1e-7
               for row in trace):
            failures.append("origin_lane_drift")

    # Recompute every complete-origin snapshot without importing the production
    # capacity routine.
    period = float(spec["capacity"]["snapshot_interval_s"])
    rho = float(spec["capacity"]["reliability_rho"])
    spacing = {policy: float(cfg[f"target_{policy.lower()}_m"])
               for policy in "CRF"}
    grouped = {}
    for row in trace:
        timestamp = float(row["t_s"])
        if abs(timestamp / period - round(timestamp / period)) > 1e-8:
            continue
        grouped.setdefault(timestamp, {}).setdefault(
            origin[row["aircraft_id"]], []).append(row)
    reconstructed = []
    for timestamp in sorted(grouped):
        if any(not grouped[timestamp].get(lane) for lane in range(3)):
            continue
        total = 0.0
        origins = []
        for lane in range(3):
            policies = [row["policy"] for row in grouped[timestamp][lane]]
            mean_spacing = np.mean([spacing[policy] for policy in policies])
            rate = 3600.0 * cfg["cruise_mps"] / mean_spacing
            origins.append(float(rate))
            total += rate
        reconstructed.append((timestamp, origins, total))

    with (folder / "origin_capacity_trace.csv").open(
            encoding="utf-8-sig", newline="") as stream:
        archived = list(csv.DictReader(stream))
    if len(archived) != len(reconstructed):
        failures.append("origin_capacity_row_count")
    maximum_capacity_error = 0.0
    for archived_row, (timestamp, rates, total) in zip(archived, reconstructed):
        values = [abs(float(archived_row["timestamp_s"]) - timestamp),
                  abs(float(archived_row["corridor_capacity_uam_h"]) - total)]
        values.extend(abs(float(archived_row[f"origin_{lane}_capacity_uam_h"])
                          - rates[lane]) for lane in range(3))
        maximum_capacity_error = max(maximum_capacity_error, *values)
    if maximum_capacity_error > 1e-8:
        failures.append("origin_capacity_formula")
    expected_rho = reliability_floor([row[2] for row in reconstructed], rho)
    archived_rho = summary["origin_stream_capacity"]["corridor_capacity_rho_uam_h"]
    if abs(expected_rho - archived_rho) > 1e-8:
        failures.append("origin_capacity_reliability")

    return {
        "case_id": case["id"],
        "passed": not failures,
        "failures": failures,
        "trace_rows": len(trace),
        "observation_rows": len(observations),
        "started_lane_changes": len(starts),
        "completed_lane_changes": len(completed),
        "complete_origin_snapshots": len(reconstructed),
        "maximum_position_error_m": position_error,
        "maximum_altitude_error_m": altitude_error,
        "maximum_sinr_error_db": sinr_error,
        "serving_base_station_match": serving_match,
        "maximum_origin_capacity_error_uam_h": maximum_capacity_error,
        "minimum_sampled_horizontal_separation_m":
            metrics["minimum_sampled_horizontal_separation_m"],
    }


def audit(run: Path) -> dict:
    run = run.resolve()
    manifest = read(run / "manifest.json")
    spec = read(run / "resolved_config.json")
    failures = []
    if manifest["status"] != "completed":
        failures.append("manifest_not_completed")
    for item in manifest.get("outputs", []):
        if sha256(run / item["path"]) != item["sha256"]:
            failures.append(f"output_hash:{item['path']}")
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        for item in manifest["sources"]:
            if sha256_bytes(archive.read(item["path"])) != item["sha256"]:
                failures.append(f"source_snapshot:{item['path']}")

    cfg = spec["parameters"]
    traffic = spec["traffic"]
    if (cfg["window_s"] != 90.0 or cfg["radio_s"] != 2.0
            or cfg["policy_s"] != 5.0 or cfg["persistence_k"] != 3
            or cfg["threshold_db"] != -2.0):
        failures.append("fixed_policy_configuration")
    if (traffic["lateral_offsets_m"] != [-300.0, 0.0, 300.0]
            or traffic["altitude_m"] != 300.0):
        failures.append("three_lane_geometry")
    if (traffic["assignment"] != "independent_synchronous_stream_per_lane"
            or traffic["per_lane_headway_s"] != 32.0):
        failures.append("three_stream_schedule_definition")
    by_lane = {lane: [] for lane in range(3)}
    for entry in spec["entries"]:
        by_lane[int(entry["lane"])].append(float(entry["requested_time_s"]))
    if any(times != by_lane[0] for times in by_lane.values()):
        failures.append("nonmatched_origin_schedules")
    if any(abs(value - index * traffic["per_lane_headway_s"]) > 1e-8
           for index, value in enumerate(by_lane[0])):
        failures.append("per_lane_headway")

    config_path = ROOT / "research/dynamic-transitions/configs/bay_area_three_stream_capacity_3x1.json"
    raw = read(config_path)
    scenario_path = (config_path.parent / raw["scenario"]).resolve()
    scenario = load_scenario(scenario_path)
    frame = SmoothCorridorFrame(scenario.corridor, cfg["center_control_step_m"])
    selected_case = spec.get("execution_overrides", {}).get("selected_case")
    configured = [case for case in spec["cases"]
                  if selected_case is None or case["id"] == selected_case]
    cases = [audit_case(run, spec, case, scenario, frame) for case in configured]
    failures.extend(f"{case['case_id']}:{failure}"
                    for case in cases for failure in case["failures"])
    result = {"run_id": run.name, "passed": not failures,
              "failures": failures, "cases": cases}
    (run / "validation.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = audit(args.run)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
