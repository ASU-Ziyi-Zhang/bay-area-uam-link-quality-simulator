"""Independently audit the 90 s Bay Area same-altitude three-lane archive."""
from __future__ import annotations

import argparse
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


def on_clock(value: float, period: float) -> bool:
    return abs(value / period - round(value / period)) < 1e-8


def audit_case(run: Path, spec: dict, case: dict, scenario, frame) -> dict:
    folder = run / case["id"]
    trace = read(folder / "trace.json.gz")
    observations = read(folder / "observations.json.gz")
    events = read(folder / "events.json")
    decisions = read(folder / "decisions.json.gz")
    summary = read(folder / "summary.json")
    cfg = spec["parameters"]
    offsets = np.asarray(spec["traffic"]["lateral_offsets_m"], float)

    failures = []
    if summary["status"] != "all_exited":
        failures.append("not_all_exited")
    if summary["completed_requests"] != summary["scheduled_requests"]:
        failures.append("incomplete_requests")
    if summary["active_aircraft"] or summary["pending_aircraft"]:
        failures.append("remaining_aircraft")

    # Independent geometric reconstruction of every archived position.
    q = np.asarray([row["q_m"] for row in trace], float)
    d = np.asarray([row["offset_m"] for row in trace], float)
    center, _, normal, _, _ = frame.frame(q)
    expected_xy = center + d[:, None] * normal
    archived_xyz = np.asarray([row["xyz"] for row in trace], float)
    position_error = float(np.max(np.linalg.norm(expected_xy - archived_xyz[:, :2], axis=1)))
    altitude_error = float(np.max(abs(archived_xyz[:, 2] - spec["traffic"]["altitude_m"])))
    if position_error > 1e-6 or altitude_error > 1e-8:
        failures.append("geographic_position")

    # Recompute radio observations from archived positions without using the
    # simulator's stored SINR or serving-cell result.
    trace_index = {(row["aircraft_id"], round(float(row["t_s"]), 8)): row for row in trace}
    obs_xyz = np.asarray([
        trace_index[(row["aircraft_id"], round(float(row["t_s"]), 8))]["xyz"]
        for row in observations
    ], float)
    radio = BSRadio(scenario.base_stations, scenario.radio)
    recomputed = radio(obs_xyz)
    sinr_error = float(np.max(abs(recomputed["sinr_db"]
                                  - np.asarray([row["sinr_db"] for row in observations]))))
    serving_match = bool(np.array_equal(recomputed["serving_bs"],
                                         [row["serving_bs"] for row in observations]))
    if sinr_error > 1e-8 or not serving_match:
        failures.append("radio_reconstruction")

    if any(not on_clock(float(row["t_s"]), cfg["radio_s"]) for row in observations):
        failures.append("radio_clock")
    updates = [event for event in events if event["status"] == "policy_update"]
    if any(not on_clock(float(event["t_s"]), cfg["policy_s"]) for event in updates):
        failures.append("policy_clock")

    # Exposure/raw-policy audit using only archived group fractions.
    histories: dict[str, list[tuple[float, float]]] = {}
    maximum_exposure_error = 0.0
    raw_errors = 0
    for row in sorted(observations, key=lambda item: (float(item["t_s"]), item["aircraft_id"])):
        identifier = row["aircraft_id"]
        now = float(row["t_s"])
        history = histories.setdefault(identifier, [])
        history[:] = [item for item in history if item[0] >= now - cfg["window_s"] - 1e-8]
        history.append((now, float(row["group_bad_fraction"])))
        exposure = float(np.mean([item[1] for item in history]))
        maximum_exposure_error = max(maximum_exposure_error,
                                     abs(exposure - float(row["exposure"])))
        raw = ("C" if exposure <= cfg["exposure_c"] + 1e-12 else
               "R" if exposure <= cfg["exposure_r"] + 1e-12 else "F")
        raw_errors += raw != row["raw_policy"]
    if maximum_exposure_error > 1e-12 or raw_errors:
        failures.append("exposure_or_raw_policy")

    # Replay the k-update persistence state from the archived update events.
    filter_state: dict[str, dict] = {}
    persistence_errors = 0
    for event in sorted(updates, key=lambda item: (float(item["t_s"]), item["aircraft_id"])):
        identifier = event["aircraft_id"]
        record = filter_state.setdefault(identifier, {
            "state": event["previous_policy"],
            "candidate": event["previous_policy"], "count": 0})
        if record["state"] != event["previous_policy"]:
            persistence_errors += 1
        raw = event["raw_policy"]
        if raw == record["state"]:
            record["candidate"], record["count"] = record["state"], 0
        else:
            if raw == record["candidate"]:
                record["count"] += 1
            else:
                record["candidate"], record["count"] = raw, 1
            if record["count"] >= cfg["persistence_k"]:
                record["state"], record["candidate"], record["count"] = raw, raw, 0
        if (record["state"] != event["policy"]
                or record["candidate"] != event["candidate_policy"]
                or record["count"] != event["candidate_count"]):
            persistence_errors += 1
    if persistence_errors:
        failures.append("persistence_replay")

    metrics = summary["policy_and_longitudinal_metrics"]
    policy_time = {policy: sum(float(row["dt_s"]) for row in trace
                               if row["policy"] == policy) for policy in "CRF"}
    total = sum(policy_time.values())
    if any(abs(policy_time[policy] - metrics["policy_aircraft_time_s"][policy]) > 1e-7
           for policy in "CRF"):
        failures.append("policy_time")
    if any(abs(policy_time[policy] / total - metrics["policy_shares"][policy]) > 1e-12
           for policy in "CRF"):
        failures.append("policy_share")
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
    completed = [event for event in events if event["status"] == "transition_completed"]
    if case["lane_change_allowed"]:
        selected = [row for row in decisions if row.get("take_change")]
        if len(selected) != len(starts):
            failures.append("selected_start_count")
        for event in starts:
            if abs(event["source_lane"] - event["target_lane"]) != 1:
                failures.append("nonadjacent_lane_change")
            if event.get("worsened_aircraft"):
                failures.append("accepted_worsening")
        for row in selected:
            candidate = row.get("selected_candidate", {})
            if row["reason"] == "beneficial_change":
                if (candidate.get("ego_policy_cost_improvement_s", -np.inf)
                        <= spec["lane_change"]["ego_policy_gain_margin_s"]):
                    failures.append("insufficient_ego_benefit")
                if candidate.get("worsened_aircraft"):
                    failures.append("selected_worsening")
            elif row["reason"] != "recovery_change":
                failures.append("undeclared_change_reason")
        if len(completed) != len(starts):
            failures.append("incomplete_lane_change")
    elif starts or completed or any(row["target_lane"] is not None for row in trace):
        failures.append("motion_in_no_change_case")
    elif any(abs(float(row["offset_m"]) - offsets[row["lane"]]) > 1e-7 for row in trace):
        failures.append("lane_offset_in_no_change_case")

    archived_moves = summary["lane_change_metrics"]
    if (archived_moves["started_lane_changes"] != len(starts)
            or archived_moves["completed_lane_changes"] != len(completed)):
        failures.append("movement_summary")

    return {
        "case_id": case["id"],
        "passed": not failures,
        "failures": failures,
        "trace_rows": len(trace),
        "observation_rows": len(observations),
        "policy_updates": len(updates),
        "started_lane_changes": len(starts),
        "completed_lane_changes": len(completed),
        "maximum_position_error_m": position_error,
        "maximum_altitude_error_m": altitude_error,
        "maximum_sinr_error_db": sinr_error,
        "serving_base_station_match": serving_match,
        "maximum_exposure_error": maximum_exposure_error,
        "persistence_errors": persistence_errors,
        "minimum_sampled_horizontal_separation_m":
            metrics["minimum_sampled_horizontal_separation_m"],
    }


def audit(run: Path) -> dict:
    run = run.resolve()
    manifest = read(run / "manifest.json")
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

    spec = read(run / "resolved_config.json")
    cfg = spec["parameters"]
    traffic = spec["traffic"]
    if (cfg["window_s"] != 90.0 or cfg["radio_s"] != 2.0
            or cfg["policy_s"] != 5.0 or cfg["persistence_k"] != 3
            or cfg["threshold_db"] != -2.0):
        failures.append("fixed_policy_configuration")
    if (traffic["lateral_offsets_m"] != [-300.0, 0.0, 300.0]
            or traffic["altitude_m"] != 300.0):
        failures.append("three_lane_geometry")
    entries = spec["entries"]
    for index, entry in enumerate(entries):
        if (abs(float(entry["requested_time_s"]) - index * traffic["global_headway_s"]) > 1e-8
                or entry["lane"] != index % 3):
            failures.append("global_round_robin_schedule")
            break

    config_path = ROOT / "research/dynamic-transitions/configs/bay_area_three_lane_90s.json"
    raw = read(config_path)
    scenario_path = (config_path.parent / raw["scenario"]).resolve()
    scenario = load_scenario(scenario_path)
    frame = SmoothCorridorFrame(scenario.corridor, cfg["center_control_step_m"])
    selected_case = spec.get("execution_overrides", {}).get("selected_case")
    configured_cases = [case for case in spec["cases"]
                        if selected_case is None or case["id"] == selected_case]
    cases = [audit_case(run, spec, case, scenario, frame) for case in configured_cases]
    failures.extend(f"{case['case_id']}:{failure}"
                    for case in cases for failure in case["failures"])
    result = {
        "run_id": run.name,
        "passed": not failures,
        "failures": failures,
        "cases": cases,
    }
    (run / "validation.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = audit(args.run)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
