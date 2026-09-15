"""Run the matched 90 s Bay Area three-lane longitudinal/lane-change study."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import zipfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from capacity_policy import load_scenario  # noqa: E402
from run_bay_area_dev3a import policy_metrics  # noqa: E402
from uam_simulator.bay_area_dispatch import DispatchConfig, DispatchExperiment  # noqa: E402
from uam_simulator.capacity_analysis import (  # noqa: E402
    observed_exit_rate, planning_capacity_by_origin_stream,
    planning_capacity_from_trace,
)
from uam_simulator.geographic_traffic import EntryRequest  # noqa: E402
from uam_simulator.motion_envelope import AccelerationEnvelope  # noqa: E402
from uam_simulator.research_provenance import execution_metadata  # noqa: E402


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_json_atomic(path: Path, value) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError(f"cannot write empty CSV: {path}")
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_entries(traffic: dict, *, limit_requests: int | None = None) -> list[EntryRequest]:
    if "per_lane_headway_s" in traffic:
        offsets = traffic["lateral_offsets_m"]
        headway = float(traffic["per_lane_headway_s"])
        duration = float(traffic["release_duration_s"])
        if headway <= 0 or duration <= 0:
            raise ValueError("per-lane headway and release duration must be positive")
        phases = traffic.get("lane_phase_offsets_s", [0.0] * len(offsets))
        if len(phases) != len(offsets):
            raise ValueError("lane phase offsets must align with lateral lanes")
        entries = []
        for lane, phase in enumerate(phases):
            sequence = 0
            requested = float(phase)
            while requested < duration - 1e-8:
                sequence += 1
                entries.append(EntryRequest(
                    f"L{lane + 1}-UAM{sequence:02d}", requested, lane))
                requested = float(phase) + sequence * headway
        entries.sort(key=lambda row: (row.requested_time_s, row.lane,
                                      row.aircraft_id))
        if limit_requests is not None:
            entries = entries[:int(limit_requests)]
        return entries
    count = int(traffic["global_entry_count"])
    if limit_requests is not None:
        count = min(count, int(limit_requests))
    headway = float(traffic["global_headway_s"])
    if "entry_flow_index" in traffic:
        lane = int(traffic["entry_flow_index"])
        return [EntryRequest(f"UAM-{sequence + 1:02d}", sequence * headway, lane)
                for sequence in range(count)]
    offsets = traffic["lateral_offsets_m"]
    per_lane = [0] * len(offsets)
    entries = []
    for sequence in range(count):
        lane = sequence % len(offsets)
        per_lane[lane] += 1
        entries.append(EntryRequest(
            f"L{lane + 1}-UAM{per_lane[lane]:02d}", sequence * headway, lane))
    return entries


def movement_metrics(result: dict, offsets: tuple[float, ...],
                     heights: tuple[float, ...] | None = None) -> dict:
    if heights is None:
        heights = tuple(0. for _ in offsets)
    starts = [event for event in result["events"] if event["status"] == "change_started"]
    completed = [event for event in result["events"]
                 if event["status"] == "transition_completed"]
    changed_ids = sorted({event["aircraft_id"] for event in starts})
    directions: dict[str, int] = {}
    for event in starts:
        key = f"{event['source_lane']}->{event['target_lane']}"
        directions[key] = directions.get(key, 0) + 1
    selected = [row for row in result["decisions"] if row.get("take_change")]
    predicted_ego_gain = [
        float(row["selected_candidate"]["ego_policy_cost_improvement_s"])
        for row in selected
        if "ego_policy_cost_improvement_s" in row.get("selected_candidate", {})
    ]
    return {
        "started_lane_changes": len(starts),
        "completed_lane_changes": len(completed),
        "aircraft_with_lane_change": len(changed_ids),
        "lane_change_directions": directions,
        "total_commanded_lateral_distance_m": sum(
            abs(offsets[event["target_lane"]] - offsets[event["source_lane"]])
            for event in starts),
        "total_commanded_vertical_distance_m": sum(
            abs(heights[event["target_lane"]] - heights[event["source_lane"]])
            for event in starts),
        "predicted_ego_policy_gain_s": predicted_ego_gain,
        "mean_predicted_ego_policy_gain_s": (
            sum(predicted_ego_gain) / len(predicted_ego_gain)
            if predicted_ego_gain else None),
    }


def run(config_path: Path, output: Path, *, limit_requests: int | None = None,
        horizon_s: float | None = None, selected_case: str | None = None,
        progress_wall_s: float = 30.0) -> list[dict]:
    config_path, output = config_path.resolve(), output.resolve()
    if output.exists():
        raise FileExistsError(f"research run directories are immutable: {output}")
    spec = json.loads(config_path.read_text(encoding="utf-8"))
    supported = {"bay_area_three_lane_90s", "bay_area_single_stream_capacity_3x3",
                 "bay_area_three_stream_capacity_3x1"}
    if spec.get("schema_version") != 1 or spec.get("study") not in supported:
        raise ValueError("unsupported three-lane configuration")
    protocol = ROOT / spec["protocol"]
    scenario_path = (config_path.parent / spec["scenario"]).resolve()
    scenario = load_scenario(scenario_path)
    cfg = DispatchConfig(**spec["parameters"])
    envelope = AccelerationEnvelope(**spec["envelope"])
    traffic = spec["traffic"]
    if "flow_points" in traffic:
        points = [(float(row["offset_m"]), float(row["altitude_m"]))
                  for row in traffic["flow_points"]]
        offsets = tuple(point[0] for point in points)
        heights = tuple(point[1] for point in points)
        entry_flow = int(traffic["entry_flow_index"])
        if not 0 <= entry_flow < len(points):
            raise ValueError("entry flow index is outside the configured grid")
        candidate_topology = traffic.get("candidate_topology", "grid_8_connected")
    else:
        offsets = tuple(float(value) for value in traffic["lateral_offsets_m"])
        heights = tuple(float(traffic["altitude_m"]) for _ in offsets)
        if len(offsets) != 3 or len(set(offsets)) != 3:
            raise ValueError("exactly three distinct lateral lanes are required")
        candidate_topology = "linear_adjacent"
    entries = build_entries(traffic, limit_requests=limit_requests)
    experiment = DispatchExperiment(
        scenario, cfg, envelope, offsets=offsets, heights=heights,
        candidate_topology=candidate_topology)
    lane_cfg = spec["lane_change"]
    run_horizon = float(traffic["horizon_s"] if horizon_s is None else horizon_s)
    cases = [case for case in spec["cases"]
             if selected_case is None or case["id"] == selected_case]
    if not cases:
        raise ValueError(f"unknown case: {selected_case}")

    verifier_by_study = {
        "bay_area_single_stream_capacity_3x3":
            "scripts/verify_bay_area_single_stream_capacity.py",
        "bay_area_three_stream_capacity_3x1":
            "scripts/verify_bay_area_three_stream_capacity.py",
        "bay_area_three_lane_90s":
            "scripts/verify_bay_area_three_lane_90s.py",
    }
    verifier = ROOT / verifier_by_study[spec["study"]]
    source_files = [
        Path(__file__).resolve(),
        ROOT / "scripts/run_bay_area_dev3a.py",
        verifier,
        ROOT / "src/uam_simulator/bay_area_dispatch.py",
        ROOT / "src/uam_simulator/capacity_analysis.py",
        ROOT / "src/uam_simulator/geographic_traffic.py",
        ROOT / "src/uam_simulator/lateral_study.py",
        ROOT / "src/uam_simulator/two_uam_longitudinal.py",
        ROOT / "src/uam_simulator/aks.py",
        ROOT / "src/uam_simulator/motion_envelope.py",
        ROOT / "src/uam_simulator/policy_motion.py",
        ROOT / "tests/test_bay_area_dispatch.py",
        protocol,
        config_path,
        scenario_path,
        scenario_path.parent / "data/corridor.geojson",
        scenario_path.parent / "data/base_stations.csv",
    ]
    output.mkdir(parents=True)
    command = [str(Path(__file__).resolve()), "--config", str(config_path),
               "--output", str(output)]
    if limit_requests is not None:
        command.extend(["--limit-requests", str(limit_requests)])
    if horizon_s is not None:
        command.extend(["--horizon-s", str(horizon_s)])
    if selected_case is not None:
        command.extend(["--case", selected_case])
    if progress_wall_s != 30.0:
        command.extend(["--progress-wall-s", str(progress_wall_s)])
    manifest = {
        "status": "running",
        "study": spec["study"],
        "run_id": output.name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "execution": execution_metadata(ROOT, command),
        "sources": [{"path": str(path.relative_to(ROOT)), "sha256": sha256(path)}
                    for path in source_files],
    }
    resolved = {
        **spec,
        "parameters": asdict(cfg),
        "envelope": asdict(envelope),
        "entries": [asdict(entry) for entry in entries],
        "scenario_id": scenario.scenario_id,
        "execution_overrides": {
            "limit_requests": limit_requests,
            "horizon_s": horizon_s,
            "selected_case": selected_case,
            "progress_wall_s": progress_wall_s,
        },
    }
    write_json(output / "resolved_config.json", resolved)
    write_json(output / "manifest.json", manifest)
    with zipfile.ZipFile(output / "source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for path in source_files:
            archive.write(path, path.relative_to(ROOT))

    summaries = []
    try:
        for case in cases:
            case_dir = output / case["id"]
            case_dir.mkdir()
            print(f"Running {case['id']}", flush=True)
            started = time.monotonic()

            def record_progress(payload: dict) -> None:
                row = {
                    "recorded_utc": datetime.now(timezone.utc).isoformat(),
                    "run_id": output.name,
                    "case_id": case["id"],
                    **payload,
                }
                write_json_atomic(case_dir / "progress.json", row)
                with (case_dir / "progress.jsonl").open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                print(
                    "  progress "
                    f"sim={row['sim_time_s']:.1f}/{row['sim_horizon_s']:.1f}s "
                    f"entered={row['entered_requests']}/{row['scheduled_requests']} "
                    f"completed={row['completed_requests']} "
                    f"active={row['active_requests']} pending={row['pending_requests']} "
                    f"moves={row['lane_changes_completed']}/{row['lane_changes_started']} "
                    f"phase={row['phase']} wall={row['wall_elapsed_s']:.1f}s",
                    flush=True,
                )

            result = experiment.run(
                entries,
                bool(case["lane_change_allowed"]),
                horizon_s=run_horizon,
                prediction_s=float(lane_cfg["prediction_s"]),
                duration_factors=tuple(float(value) for value in lane_cfg["duration_factors"]),
                gain_margin=float(lane_cfg["ego_policy_gain_margin_s"]),
                candidate_recheck_s=float(lane_cfg["candidate_recheck_s"]),
                progress_callback=record_progress,
                progress_wall_s=progress_wall_s,
            )
            metrics = policy_metrics(result, cfg)
            moves = movement_metrics(result, offsets, heights)
            capacity_rows, capacity = planning_capacity_from_trace(
                result["trace"], cfg,
                rho=float(spec.get("capacity", {}).get("reliability_rho", 0.95)),
                snapshot_s=float(spec.get("capacity", {}).get(
                    "snapshot_interval_s", cfg.policy_s)))
            origin_capacity_rows = None
            origin_capacity = None
            if spec.get("capacity", {}).get("aggregation") == "origin_stream_then_sum":
                origin_capacity_rows, origin_capacity = planning_capacity_by_origin_stream(
                    result["trace"], cfg,
                    {entry.aircraft_id: entry.lane for entry in entries},
                    origin_streams=range(len(offsets)),
                    rho=float(spec["capacity"].get("reliability_rho", 0.95)),
                    snapshot_s=float(spec["capacity"].get(
                        "snapshot_interval_s", cfg.policy_s)))
            throughput = observed_exit_rate(result["events"])
            delays = {event["aircraft_id"]: event["entry_delay_s"]
                      for event in result["events"] if event["status"] == "corridor_entry"}
            summary = {
                "run_id": output.name,
                "case_id": case["id"],
                "lane_change_allowed": bool(case["lane_change_allowed"]),
                "status": result["status"],
                "end_s": result["end_s"],
                "scheduled_requests": len(entries),
                "completed_requests": len(result["completed"]),
                "active_aircraft": result["active"],
                "pending_aircraft": result["pending"],
                "entry_delays_s": delays,
                "policy_and_longitudinal_metrics": metrics,
                "lane_change_metrics": moves,
                "conditional_planning_capacity": capacity,
                "origin_stream_capacity": origin_capacity,
                "observed_throughput": throughput,
                "limitations": result["limitations"],
                "wall_time_s": time.monotonic() - started,
            }
            with gzip.open(case_dir / "trace.json.gz", "wt", encoding="utf-8") as stream:
                json.dump(result["trace"], stream, allow_nan=False)
            with gzip.open(case_dir / "observations.json.gz", "wt", encoding="utf-8") as stream:
                json.dump(result["observations"], stream, allow_nan=False)
            with gzip.open(case_dir / "decisions.json.gz", "wt", encoding="utf-8") as stream:
                json.dump(result["decisions"], stream, allow_nan=False)
            write_json(case_dir / "events.json", result["events"])
            write_csv(case_dir / "capacity_trace.csv", capacity_rows)
            if origin_capacity_rows is not None:
                write_csv(case_dir / "origin_capacity_trace.csv",
                          origin_capacity_rows)
            write_json(case_dir / "summary.json", summary)
            summaries.append(summary)
            write_json(output / "summary.json", summaries)
            shares = metrics["policy_shares"]
            corridor_text = (
                f"corridor_Q95={origin_capacity['corridor_capacity_rho_uam_h']:.1f} UAM/h; "
                if origin_capacity is not None else "")
            print(
                f"  completed={len(result['completed'])}/{len(entries)}; "
                f"moves={moves['completed_lane_changes']}; "
                f"C/R/F={100*shares['C']:.1f}/{100*shares['R']:.1f}/{100*shares['F']:.1f}%; "
                f"Q95={capacity['q_mix_rho_uam_h']:.1f} UAM/h; "
                f"{corridor_text}wall={summary['wall_time_s']:.1f}s",
                flush=True)
        manifest["status"] = "completed"
    except BaseException as exc:
        manifest["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        manifest["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        drift = [str(path.relative_to(ROOT)) for path in source_files
                 if sha256(path) != next(item["sha256"] for item in manifest["sources"]
                                         if item["path"] == str(path.relative_to(ROOT)))]
        if drift and manifest["status"] == "completed":
            manifest["status"] = "requires_review"
        manifest["completed_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["completed_cases"] = len(summaries)
        manifest["changed_during_run"] = drift
        manifest["outputs"] = [
            {"path": str(path.relative_to(output)), "sha256": sha256(path)}
            for path in sorted(output.rglob("*"))
            if path.is_file() and path.name != "manifest.json"
        ]
        write_json(output / "manifest.json", manifest)
    return summaries


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
        default=ROOT / "research/dynamic-transitions/configs/bay_area_three_lane_90s.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit-requests", type=int,
                        help="development-only request-count cap recorded in resolved_config")
    parser.add_argument("--horizon-s", type=float,
                        help="development-only horizon override recorded in resolved_config")
    parser.add_argument("--case",
                        help="run one member of the matched pair in its own archive")
    parser.add_argument("--progress-wall-s", type=float, default=30.0,
                        help="wall-clock interval between durable progress records")
    args = parser.parse_args()
    run(args.config, args.output, limit_requests=args.limit_requests,
        horizon_s=args.horizon_s, selected_case=args.case,
        progress_wall_s=args.progress_wall_s)


if __name__ == "__main__":
    main()
