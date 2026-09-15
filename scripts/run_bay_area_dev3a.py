"""Run calibrated-policy Bay Area longitudinal-control experiments."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from capacity_policy import load_scenario  # noqa: E402
from uam_simulator.bay_area_dispatch import DispatchConfig, DispatchExperiment  # noqa: E402
from uam_simulator.geographic_traffic import EntryRequest  # noqa: E402
from uam_simulator.motion_envelope import AccelerationEnvelope  # noqa: E402
from uam_simulator.research_provenance import execution_metadata  # noqa: E402


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def policy_metrics(result: dict, cfg: DispatchConfig,
                   comparison_horizon_s: float | None = None) -> dict:
    trace = result["trace"]
    aircraft_ids = sorted({row["aircraft_id"] for row in trace})
    trace_by_aircraft = {identifier: [] for identifier in aircraft_ids}
    policy_time = {key: 0.0 for key in "CRF"}
    controller_time: dict[str, float] = {}
    switches_by_aircraft = {}
    speed_direction_reversals_by_aircraft = {}
    for row in trace:
        trace_by_aircraft[row["aircraft_id"]].append(row)
        dt = float(row["dt_s"])
        policy_time[row["policy"]] += dt
        mode = row["controller"]
        controller_time[mode] = controller_time.get(mode, 0.0) + dt
    for identifier in aircraft_ids:
        rows = trace_by_aircraft[identifier]
        policies = [row["policy"] for row in rows]
        switches_by_aircraft[identifier] = sum(a != b for a, b in zip(policies, policies[1:]))
        nonzero_directions = []
        for left, right in zip(rows, rows[1:]):
            delta = float(right["v_mps"]) - float(left["v_mps"])
            if abs(delta) > 1e-7:
                nonzero_directions.append(1 if delta > 0 else -1)
        speed_direction_reversals_by_aircraft[identifier] = sum(
            left != right for left, right in zip(nonzero_directions, nonzero_directions[1:]))
    total = sum(policy_time.values())

    updates = [row for row in result["events"] if row["status"] == "policy_update"]
    changes = [row for row in updates if row["previous_policy"] != row["policy"]]
    changes_by_aircraft = {identifier: [] for identifier in aircraft_ids}
    for row in changes:
        changes_by_aircraft[row["aircraft_id"]].append(row)
    nominal_settling_s = {
        ("C", "R"): 73.8, ("R", "C"): 50.7,
        ("R", "F"): 147.5, ("F", "R"): 99.0,
        ("C", "F"): 221.3, ("F", "C"): 148.5,
    }
    action_rows = []
    for change in changes:
        identifier = change["aircraft_id"]
        start = float(change["t_s"])
        later_changes = [float(row["t_s"]) for row in changes_by_aircraft[identifier]
                         if float(row["t_s"]) > start]
        stop = min(later_changes, default=math.inf)
        rows = [row for row in trace_by_aircraft[identifier]
                if start - 1e-9 <= float(row["t_s"]) < stop - 1e-9]
        if not rows or not rows[0]["leaders"]:
            continue
        leader_id = rows[0]["leaders"][0][0]
        relation_changed = any(not row["leaders"] or row["leaders"][0][0] != leader_id
                               for row in rows)
        comparable = [row for row in rows if row["leaders"]
                      and row["leaders"][0][0] == leader_id]
        errors = [float(row["leaders"][0][1]) - float(row["target_gap_m"])
                  for row in comparable]
        completed = None
        for row in comparable:
            gap = float(row["leaders"][0][1])
            relative = float(row["leaders"][0][2])
            if abs(gap - float(row["target_gap_m"])) <= 1.0 and abs(relative) <= 0.1:
                completed = float(row["t_s"])
                break
        dwell = ((stop - start) if math.isfinite(stop)
                 else sum(float(row["dt_s"]) for row in rows))
        reference = nominal_settling_s[(change["previous_policy"], change["policy"])]
        action_rows.append({
            "aircraft_id": identifier,
            "leader_id": leader_id,
            "start_s": start,
            "from_policy": change["previous_policy"],
            "to_policy": change["policy"],
            "next_change_s": None if not later_changes else stop,
            "dwell_s": dwell,
            "nominal_isolated_settling_s": reference,
            "timing_compatible_with_isolated_reference": dwell >= reference - 1e-9,
            "relationship_changed": relation_changed,
            "initial_gap_error_m": errors[0],
            "final_gap_error_m": errors[-1],
            "best_absolute_gap_error_m": min(abs(value) for value in errors),
            "gap_error_improved": abs(errors[-1]) < abs(errors[0]) - 1e-6,
            "completed_before_next_change": completed is not None,
            "completion_s": completed,
            "duration_s": None if completed is None else completed - start,
        })
    applicable = len(action_rows)
    completed = sum(row["completed_before_next_change"] for row in action_rows)
    improved = sum(row["gap_error_improved"] for row in action_rows)
    timing_compatible = sum(row["timing_compatible_with_isolated_reference"]
                            for row in action_rows)
    finite_separations = [float(row["minimum_separation_step_m"]) for row in trace
                          if row["minimum_separation_step_m"] is not None
                          and math.isfinite(float(row["minimum_separation_step_m"]))]
    common = None
    if comparison_horizon_s is not None:
        common_time = {key: 0.0 for key in "CRF"}
        for row in trace:
            start = float(row["t_s"])
            duration = max(0.0, min(float(row["dt_s"]), comparison_horizon_s - start))
            if duration:
                common_time[row["policy"]] += duration
        common_total = sum(common_time.values())
        common_updates = [row for row in updates
                          if float(row["t_s"]) <= comparison_horizon_s + 1e-9]
        common_counts = {key: sum(row["policy"] == key for row in common_updates)
                         for key in "CRF"}
        common_count = sum(common_counts.values())
        common = {
            "horizon_s": comparison_horizon_s,
            "policy_aircraft_time_s": common_time,
            "policy_time_shares": {key: common_time[key] / common_total
                                    for key in "CRF"},
            "policy_update_counts": common_counts,
            "policy_update_shares": {key: common_counts[key] / common_count
                                      for key in "CRF"},
            "policy_update_observations": common_count,
        }
    group_size_counts = {}
    for row in result["observations"]:
        size = str(row["group_size"])
        group_size_counts[size] = group_size_counts.get(size, 0) + 1
    raw_switches_by_aircraft = {}
    observations_by_aircraft = {identifier: [] for identifier in aircraft_ids}
    for row in result["observations"]:
        observations_by_aircraft.setdefault(row["aircraft_id"], []).append(row)
    for identifier, rows in observations_by_aircraft.items():
        rows.sort(key=lambda row: float(row["t_s"]))
        raw = [row.get("raw_policy", row["policy"]) for row in rows]
        raw_switches_by_aircraft[identifier] = sum(
            left != right for left, right in zip(raw, raw[1:]))
    return {
        "aircraft_count": len(aircraft_ids),
        "aircraft_time_s": total,
        "policy_aircraft_time_s": policy_time,
        "policy_shares": {key: (policy_time[key] / total if total else 0.0) for key in "CRF"},
        "switches_by_aircraft": switches_by_aircraft,
        "mean_switches_per_flight": (float(np.mean(list(switches_by_aircraft.values())))
                                      if switches_by_aircraft else 0.0),
        "raw_switches_by_aircraft": raw_switches_by_aircraft,
        "mean_raw_switches_per_flight": (
            float(np.mean(list(raw_switches_by_aircraft.values())))
            if raw_switches_by_aircraft else 0.0),
        "speed_direction_reversals_by_aircraft": speed_direction_reversals_by_aircraft,
        "mean_speed_direction_reversals_per_flight": (
            float(np.mean(list(speed_direction_reversals_by_aircraft.values())))
            if speed_direction_reversals_by_aircraft else 0.0),
        "controller_aircraft_time_s": controller_time,
        "policy_transition_count": len(changes),
        "longitudinal_tasks": action_rows,
        "longitudinal_task_count": applicable,
        "completed_before_next_change_count": completed,
        "completion_before_next_change_fraction": completed / applicable if applicable else None,
        "gap_error_improved_count": improved,
        "gap_error_improved_fraction": improved / applicable if applicable else None,
        "timing_compatible_count": timing_compatible,
        "timing_compatible_fraction": timing_compatible / applicable if applicable else None,
        "minimum_total_speed_mps": min(float(row["total_speed_mps"]) for row in trace),
        "maximum_total_speed_mps": max(float(row["total_speed_mps"]) for row in trace),
        "maximum_joint_envelope_utilisation": max(float(row["peak_eta_step"]) for row in trace),
        "minimum_sampled_horizontal_separation_m": min(finite_separations, default=None),
        "nmac_sampled": any(bool(row["nmac_sampled"]) for row in trace),
        "maximum_absolute_lateral_offset_m": max(abs(float(row["offset_m"])) for row in trace),
        "radio_sampling_s": cfg.radio_period_s,
        "policy_update_s": cfg.policy_s,
        "persistence_k": cfg.persistence_k,
        "group_size_observation_counts": group_size_counts,
        "common_horizon_comparison": common,
    }


def run(config_path: Path, output: Path, *, window_s: float | None = None,
        entry_headway_s: float | None = None) -> dict:
    config_path, output = config_path.resolve(), output.resolve()
    if output.exists():
        raise FileExistsError(f"research run directories are immutable: {output}")
    spec = json.loads(config_path.read_text(encoding="utf-8"))
    if window_s is not None:
        spec["parameters"]["window_s"] = float(window_s)
    if entry_headway_s is not None:
        spec["traffic"]["entry_headway_s"] = float(entry_headway_s)
    if window_s is not None or entry_headway_s is not None:
        spec["case"]["id"] = (
            f"window_{spec['parameters']['window_s']:g}s_"
            f"headway_{spec['traffic']['entry_headway_s']:g}s")
    supported = {"bay_area_dev3a_longitudinal", "bay_area_longitudinal_control",
                 "bay_area_longitudinal_control_90s",
                 "bay_area_sparse_window_sweep"}
    if spec.get("schema_version") != 1 or spec.get("study") not in supported:
        raise ValueError("unsupported Bay Area longitudinal configuration")
    protocol = ROOT / spec["protocol"]
    scenario_path = (config_path.parent / spec["scenario"]).resolve()
    scenario = load_scenario(scenario_path)
    cfg = DispatchConfig(**spec["parameters"])
    envelope = AccelerationEnvelope(**spec["envelope"])
    traffic = spec["traffic"]
    count = int(traffic["aircraft_count"])
    headway = float(traffic["entry_headway_s"])
    entries = [EntryRequest(f"UAM-{index + 1:02d}", index * headway, 0)
               for index in range(count)]
    experiment = DispatchExperiment(
        scenario, cfg, envelope, offsets=(float(traffic["lateral_offset_m"]),))

    source_files = [
        Path(__file__).resolve(),
        ROOT / "src/uam_simulator/bay_area_dispatch.py",
        ROOT / "src/uam_simulator/geographic_traffic.py",
        ROOT / "src/uam_simulator/two_uam_longitudinal.py",
        ROOT / "src/uam_simulator/aks.py",
        ROOT / "src/uam_simulator/motion_envelope.py",
        ROOT / "src/uam_simulator/policy_motion.py",
        protocol,
        config_path,
        scenario_path,
        scenario_path.parent / "data/corridor.geojson",
        scenario_path.parent / "data/base_stations.csv",
    ]
    output.mkdir(parents=True)
    command = [str(Path(__file__).resolve()), "--config", str(config_path),
               "--output", str(output)]
    if window_s is not None:
        command.extend(["--window-s", str(float(window_s))])
    if entry_headway_s is not None:
        command.extend(["--entry-headway-s", str(float(entry_headway_s))])
    metadata = execution_metadata(ROOT, command)
    manifest = {
        "status": "running",
        "study": spec["study"],
        "run_id": output.name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "execution": metadata,
        "sources": [{"path": str(path.relative_to(ROOT)), "sha256": sha256(path)}
                    for path in source_files],
    }
    resolved = {**spec, "parameters": asdict(cfg), "envelope": asdict(envelope),
                "entries": [asdict(row) for row in entries],
                "scenario_id": scenario.scenario_id}
    write_json(output / "resolved_config.json", resolved)
    write_json(output / "manifest.json", manifest)
    with zipfile.ZipFile(output / "source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for path in source_files:
            archive.write(path, path.relative_to(ROOT))
    started = time.monotonic()
    try:
        result = experiment.run(entries, False, horizon_s=float(traffic["horizon_s"]))
        metrics = policy_metrics(result, cfg, traffic.get("comparison_horizon_s"))
        summary = {
            "run_id": output.name,
            "case_id": spec["case"]["id"],
            "status": result["status"],
            "end_s": result["end_s"],
            "scheduled_aircraft": count,
            "completed_aircraft": len(result["completed"]),
            "active_aircraft": result["active"],
            "pending_aircraft": result["pending"],
            "entry_delays_s": {event["aircraft_id"]: event["entry_delay_s"]
                               for event in result["events"]
                               if event["status"] == "corridor_entry"},
            "metrics": metrics,
            "limitations": result["limitations"],
            "wall_time_s": time.monotonic() - started,
        }
        with gzip.open(output / "trace.json.gz", "wt", encoding="utf-8") as stream:
            json.dump(result["trace"], stream, allow_nan=False)
        with gzip.open(output / "observations.json.gz", "wt", encoding="utf-8") as stream:
            json.dump(result["observations"], stream, allow_nan=False)
        write_json(output / "events.json", result["events"])
        write_json(output / "summary.json", summary)
        manifest["status"] = "completed"
        return summary
    except Exception as exc:
        manifest["status"] = "failed"
        manifest["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        manifest["completed_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["outputs"] = [
            {"path": str(path.relative_to(output)), "sha256": sha256(path)}
            for path in sorted(output.rglob("*"))
            if path.is_file() and path.name != "manifest.json"
        ]
        write_json(output / "manifest.json", manifest)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
        default=ROOT / "research/dynamic-transitions/configs/bay_area_dev3a_longitudinal.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--window-s", type=float,
                        help="override the assessment window in the resolved archive")
    parser.add_argument("--entry-headway-s", type=float,
                        help="override the requested arrival interval in the resolved archive")
    args = parser.parse_args()
    summary = run(args.config, args.output, window_s=args.window_s,
                  entry_headway_s=args.entry_headway_s)
    metrics = summary["metrics"]
    print(f"{summary['run_id']}: {summary['status']}; "
          f"completed {summary['completed_aircraft']}/{summary['scheduled_aircraft']}")
    print("C/R/F = " + "/".join(f"{100 * metrics['policy_shares'][key]:.1f}%" for key in "CRF"))
    print(f"policy transitions={metrics['policy_transition_count']}; "
          f"longitudinal tasks={metrics['longitudinal_task_count']}; "
          f"completed before next change={metrics['completion_before_next_change_fraction']}")


if __name__ == "__main__":
    main()
