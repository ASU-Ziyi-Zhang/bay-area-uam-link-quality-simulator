"""Run and archive matched Stage-2 singleton directional-control conditions."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import gzip
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capacity_policy import load_scenario
from uam_simulator.lateral_study import SmoothCorridorFrame
from uam_simulator.minimum_change import MinimumChangeConfig
from uam_simulator.motion_envelope import AccelerationEnvelope
from uam_simulator.spatial_study import SpatialConfig, field_samples_spatial, simulate_spatial
from uam_simulator.research_provenance import execution_metadata
from run_lateral_study import sha, write_json


def run(config_path, output):
    config_path, output = Path(config_path).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError("research run directories are immutable")
    spec = json.loads(config_path.read_text())
    if spec["study"] not in {"bay_area_singleton_spatial_response", "bay_area_predictive_spatial_response",
                             "bay_area_motion_envelope_single_uam"} or spec["schema_version"] != 1:
        raise ValueError("unsupported spatial protocol")
    cfg = SpatialConfig(**spec["parameters"]); planning = MinimumChangeConfig(**spec["planning"])
    scenario_path = (config_path.parent / spec["scenario"]).resolve()
    scenario = load_scenario(scenario_path); raw = json.loads(scenario_path.read_text())
    offsets, heights = np.array(spec["offsets_m"], float), np.array(spec["heights_m"], float)
    for values in [offsets, heights]:
        if not len(values) or not np.isfinite(values).all() or np.any(np.diff(values) <= 0):
            raise ValueError("invalid candidate coordinates")
        if np.any(abs(np.diff(values)/planning.candidate_spacing_m-np.round(np.diff(values)/planning.candidate_spacing_m)) > 1e-8):
            raise ValueError("candidate coordinates must follow planning resolution")
    cases = [{**c, "parameters": asdict(replace(cfg, **c.get("parameters", {})))} for c in spec["cases"]]
    ids = [c["id"] for c in cases]
    if len(set(ids)) != len(ids) or any(not x.replace("_", "").isalnum() for x in ids):
        raise ValueError("unique simple case IDs required")
    if any(c["mode"] not in {"none", "lateral", "vertical", "joint"} for c in cases):
        raise ValueError("unknown spatial mode")
    frame = SmoothCorridorFrame(scenario.corridor, cfg.center_control_step_m)
    geometry = frame.report(float(np.max(abs(offsets))))
    sources = sorted((ROOT/"src").rglob("*.py")) + [Path(__file__).resolve(), ROOT/"scripts/run_lateral_study.py",
        ROOT/"scripts/verify_spatial_study.py", ROOT/"scripts/verify_lateral_study.py", ROOT/"scripts/plot_wide_radio_map.py",
        ROOT/"tests/test_spatial_study.py", ROOT/"pyproject.toml", ROOT/"research/dynamic-transitions/spatial-transition-protocol.md"]
    inputs = [config_path, scenario_path, scenario_path.parent/raw["corridor"]["path"],
              scenario_path.parent/raw["base_stations"]["path"]]
    records = lambda files: [{"path": p.relative_to(ROOT).as_posix(), "sha256": sha(p)} for p in files]
    code, input_records = records(sources), records(inputs)
    metadata = execution_metadata(ROOT, [str(Path(__file__).resolve()), "--config", str(config_path), "--output", str(output)])
    metadata["source_snapshot_included"] = True
    output.mkdir(parents=True)
    resolved = {**spec, "parameters": asdict(cfg), "cases": cases, "scenario_id": scenario.scenario_id,
                "radio": asdict(scenario.radio), "base_stations": [asdict(s) for s in scenario.base_stations.stations]}
    write_json(output/"resolved_config.json", resolved); write_json(output/"geometry.json", geometry)
    with zipfile.ZipFile(output/"source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources+inputs:
            archive.write(source, source.relative_to(ROOT))
    manifest = {"status": "running", "study": spec["study"], "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(), "execution": metadata, "code": code, "inputs": input_records}
    write_json(output/"manifest.json", manifest); summaries = []
    try:
        np.savez_compressed(output/"field.npz", **field_samples_spatial(frame, scenario, cfg, offsets, heights))
        for case in cases:
            print(f"Running {case['id']}", flush=True); clock = time.monotonic()
            local = SpatialConfig(**case["parameters"])
            if local.center_control_step_m != cfg.center_control_step_m:
                raise ValueError("this matched experiment cannot change reference geometry")
            envelope = None
            if case.get("envelope") is not None:
                fields = {k: v for k, v in {**spec.get("envelope_defaults", {}),
                                            **case["envelope"]}.items()
                          if k in AccelerationEnvelope.__dataclass_fields__}
                envelope = AccelerationEnvelope(**fields)
            result = simulate_spatial(frame, scenario, local, offsets, heights, case["mode"],
                                      planning, envelope=envelope)
            folder = output/case["id"]; folder.mkdir()
            for key in ["trace", "observations", "events"]:
                with gzip.open(folder/f"{key}.json.gz", "wt") as stream:
                    json.dump(result[key], stream, allow_nan=False)
            # Keep events JSON for convenient decision inspection and independent audits.
            write_json(folder/"events.json", result["events"])
            write_json(folder/"terminal.json", result["terminal"])
            summary = {"case_id": case["id"], "parameters": asdict(local),
                       "acceleration_envelope": asdict(envelope) if envelope else None,
                       **result["summary"], "wall_time_s": time.monotonic()-clock}
            write_json(folder/"summary.json", summary); summaries.append(summary)
            write_json(output/"summary.json", summaries)
            print(f"  moves={summary['completed_moves']}; F={summary['policy_time_s']['F']:.1f}s; "
                  f"sequence={summary['offset_height_sequence_m']}; wall={summary['wall_time_s']:.1f}s", flush=True)
    except Exception as exc:
        manifest["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        drift = [e["path"] for e in code+input_records if sha(ROOT/e["path"]) != e["sha256"]]
        manifest.update(status="completed" if len(summaries) == len(cases) and not drift else "requires_review",
            completed_cases=len(summaries), changed_during_run=drift,
            outputs=[{"path": p.relative_to(output).as_posix(), "sha256": sha(p)} for p in sorted(output.rglob("*"))
                     if p.is_file() and p.name != "manifest.json"])
        write_json(output/"manifest.json", manifest)
    return summaries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"research/dynamic-transitions/configs/spatial_transitions_100m.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); run(args.config, args.output)
