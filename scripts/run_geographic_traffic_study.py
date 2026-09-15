"""Run and archive the Step-3 Bay Area multi-UAM ACC experiment."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import json
from pathlib import Path
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from capacity_policy import load_scenario  # noqa: E402
from uam_simulator.geographic_traffic import (  # noqa: E402
    EntryRequest,
    GeographicTrafficConfig,
    simulate_geographic_traffic,
)
from uam_simulator.lateral_study import SmoothCorridorFrame  # noqa: E402
from uam_simulator.research_provenance import execution_metadata  # noqa: E402
from run_lateral_study import sha, write_json  # noqa: E402


def build_entries(traffic):
    offsets = (traffic["offsets_m"] if "offsets_m" in traffic else
               [row[0] for row in traffic["flow_coordinates_m"]])
    if "global_entry_count" in traffic:
        count = int(traffic["global_entry_count"])
        headway = float(traffic["global_headway_s"])
        if count <= 0 or headway <= 0 or not offsets:
            raise ValueError("invalid global traffic schedule")
        per_flow = [0] * len(offsets)
        entries = []
        for sequence in range(count):
            lane = sequence % len(offsets)
            per_flow[lane] += 1
            entries.append(EntryRequest(
                f"F{lane + 1}-UAM{per_flow[lane]:03d}",
                float(sequence * headway), lane))
        return entries
    phases = traffic["lane_phase_s"]
    count = int(traffic["aircraft_per_lane"])
    headway = float(traffic["per_lane_headway_s"])
    if len(phases) != len(offsets) or count <= 0 or headway <= 0:
        raise ValueError("invalid traffic schedule")
    entries = []
    for lane, phase in enumerate(phases):
        for sequence in range(count):
            entries.append(EntryRequest(
                f"L{lane + 1}-UAM{sequence + 1:02d}",
                float(phase + sequence * headway), lane))
    return sorted(entries, key=lambda row: (row.requested_time_s, row.aircraft_id))


def run(config_path, output):
    config_path, output = Path(config_path).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError("research run directories are immutable")
    spec = json.loads(config_path.read_text())
    if spec.get("schema_version") != 1 or spec.get("study") != "bay_area_multi_uam_acc_lane_change":
        raise ValueError("unsupported Step-3 protocol")
    cfg = GeographicTrafficConfig(**spec["parameters"])
    scenario_path = (config_path.parent / spec["scenario"]).resolve()
    scenario = load_scenario(scenario_path)
    raw = json.loads(scenario_path.read_text())
    if "flow_coordinates_m" in spec["traffic"]:
        flow_coordinates = tuple(tuple(float(v) for v in row)
                                 for row in spec["traffic"]["flow_coordinates_m"])
        if any(len(row) != 2 for row in flow_coordinates):
            raise ValueError("each flow coordinate must contain offset and altitude")
        offsets = tuple(row[0] for row in flow_coordinates)
        altitudes = tuple(row[1] for row in flow_coordinates)
    else:
        offsets = tuple(float(value) for value in spec["traffic"]["offsets_m"])
        altitudes = tuple(cfg.altitude_m for _ in offsets)
    entries = build_entries(spec["traffic"])
    frame = SmoothCorridorFrame(scenario.corridor, cfg.center_control_step_m)
    geometry = frame.report(max(abs(value) for value in offsets))
    sources = sorted((ROOT / "src").rglob("*.py")) + [
        Path(__file__).resolve(), ROOT / "scripts/run_lateral_study.py",
        ROOT / "tests/test_geographic_traffic.py", ROOT / "pyproject.toml",
        ROOT / "research/dynamic-transitions/step-03-multi-uam-acc-protocol.md",
        ROOT / "research/dynamic-transitions/step-03-nominal-recovery-protocol.md",
        ROOT / "research/dynamic-transitions/step-03-trb-traffic-scale-protocol.md",
    ]
    inputs = [config_path, scenario_path, scenario_path.parent / raw["corridor"]["path"],
              scenario_path.parent / raw["base_stations"]["path"]]
    records = lambda files: [{"path": path.relative_to(ROOT).as_posix(),
                              "sha256": sha(path)} for path in files]
    code, input_records = records(sources), records(inputs)
    metadata = execution_metadata(ROOT, [str(Path(__file__).resolve()),
        "--config", str(config_path), "--output", str(output)])
    output.mkdir(parents=True)
    resolved = {**spec, "parameters": asdict(cfg), "scenario_id": scenario.scenario_id,
                "entries": [asdict(row) for row in entries],
                "radio": asdict(scenario.radio),
                "base_stations": [asdict(site) for site in scenario.base_stations.stations]}
    write_json(output / "resolved_config.json", resolved)
    write_json(output / "geometry.json", geometry)
    with zipfile.ZipFile(output / "source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources + inputs:
            archive.write(source, source.relative_to(ROOT))
    manifest = {"status": "running", "study": spec["study"], "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(), "execution": metadata,
        "code": code, "inputs": input_records}
    write_json(output / "manifest.json", manifest)
    summaries = []
    try:
        for case in spec["cases"]:
            print(f"Running {case['id']}", flush=True)
            started = time.monotonic()
            result = simulate_geographic_traffic(frame, scenario, cfg, offsets,
                entries, transitions=bool(case["transitions"]),
                flow_altitudes=altitudes)
            folder = output / case["id"]
            folder.mkdir()
            for key in ("trace", "observations"):
                with gzip.open(folder / f"{key}.json.gz", "wt") as stream:
                    json.dump(result[key], stream, allow_nan=False)
            write_json(folder / "events.json", result["events"])
            summary = {"case_id": case["id"], **result["summary"],
                       "wall_time_s": time.monotonic() - started}
            write_json(folder / "summary.json", summary)
            summaries.append(summary)
            write_json(output / "summary.json", summaries)
            print(f"  entries={summary['realized_entries']}; moves={summary['completed_transitions']}; "
                  f"F={summary['policy_aircraft_time_s']['F']:.1f} aircraft-s; "
                  f"min_v={summary['minimum_longitudinal_speed_mps']:.3f} m/s; "
                  f"wall={summary['wall_time_s']:.1f}s", flush=True)
    except Exception as exc:
        manifest["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        drift = [item["path"] for item in code + input_records
                 if sha(ROOT / item["path"]) != item["sha256"]]
        manifest.update(
            status="completed" if len(summaries) == len(spec["cases"]) and not drift else "requires_review",
            completed_cases=len(summaries), changed_during_run=drift,
            outputs=[{"path": path.relative_to(output).as_posix(), "sha256": sha(path)}
                     for path in sorted(output.rglob("*"))
                     if path.is_file() and path.name != "manifest.json"])
        write_json(output / "manifest.json", manifest)
    return summaries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
        default=ROOT / "research/dynamic-transitions/configs/step03_multi_uam_acc.json")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    run(arguments.config, arguments.output)
