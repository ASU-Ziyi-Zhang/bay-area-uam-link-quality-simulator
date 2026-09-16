"""Resume only the missing final case of an interrupted spatial run."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path
import gzip
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capacity_policy import load_scenario
from uam_simulator.lateral_study import SmoothCorridorFrame
from uam_simulator.minimum_change import MinimumChangeConfig
from uam_simulator.spatial_study import SpatialConfig, simulate_spatial
from run_lateral_study import sha, write_json


def resume(run_path: Path) -> None:
    run = run_path.resolve()
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest.get("status") != "running":
        raise ValueError("resume is only allowed for an interrupted running archive")
    spec = json.loads((run / "resolved_config.json").read_text())
    case = next(c for c in spec["cases"] if c["id"] == "joint_forecast_refined")
    folder = run / case["id"]
    if folder.exists():
        raise FileExistsError("missing final case directory is not missing")
    scenario = load_scenario(ROOT / "scenarios/airport_to_airport/scenario.json")
    cfg = SpatialConfig(**case["parameters"])
    planning = MinimumChangeConfig(**spec["planning"])
    offsets = __import__("numpy").array(spec["offsets_m"], float)
    heights = __import__("numpy").array(spec["heights_m"], float)
    frame = SmoothCorridorFrame(scenario.corridor, cfg.center_control_step_m)
    print("Resuming joint_forecast_refined", flush=True)
    started = time.monotonic()
    result = simulate_spatial(frame, scenario, cfg, offsets, heights, "joint", planning)
    folder.mkdir()
    for key in ["trace", "observations", "events"]:
        with gzip.open(folder / f"{key}.json.gz", "wt") as stream:
            json.dump(result[key], stream, allow_nan=False)
    write_json(folder / "events.json", result["events"])
    write_json(folder / "terminal.json", result["terminal"])
    summary = {"case_id": case["id"], "parameters": asdict(cfg), **result["summary"],
               "wall_time_s": time.monotonic() - started}
    summaries = json.loads((run / "summary.json").read_text())
    if any(row["case_id"] == case["id"] for row in summaries):
        raise ValueError("final case already appears in summary")
    summaries.append(summary)
    order = {c["id"]: i for i, c in enumerate(spec["cases"])}
    summaries.sort(key=lambda row: order[row["case_id"]])
    write_json(run / "summary.json", summaries)
    code_inputs = manifest.get("code", []) + manifest.get("inputs", [])
    drift = [entry["path"] for entry in code_inputs if sha(ROOT / entry["path"]) != entry["sha256"]]
    manifest.update(status="completed" if len(summaries) == len(spec["cases"]) and not drift else "requires_review",
                    completed_cases=len(summaries), changed_during_run=drift,
                    outputs=[{"path": p.relative_to(run).as_posix(), "sha256": sha(p)}
                             for p in sorted(run.rglob("*")) if p.is_file() and p.name != "manifest.json"])
    write_json(run / "manifest.json", manifest)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    resume(parser.parse_args().run)
