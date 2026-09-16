"""Evaluate a reusable offset-height response table without choosing lane count."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capacity_policy import load_scenario
from uam_simulator.baseline_studies import write_csv, write_json
from uam_simulator.geometry_sweep import geometry_grid
from uam_simulator.research_provenance import execution_metadata


def run(config_path, output):
    config_path, output = Path(config_path).resolve(), Path(output).resolve()
    config = json.loads(config_path.read_text())
    if config.get("schema_version") != 1:
        raise ValueError("unsupported sweep schema")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to replace existing results: {output}")
    execution = execution_metadata(ROOT, [str(Path(__file__).resolve()), "--config", str(config_path),
                                         "--output", str(output)])
    slices = config["plot_slices"]
    if slices["height_at_offset_m"] not in config["offsets_m"] or slices["offset_at_altitude_m"] not in config["altitudes_m"]:
        raise ValueError("plot slices must exist in the geometry grid")
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "resolved_config.json", config)
    inputs = [config_path]
    totals = {}
    for relative in config["scenarios"]:
        path = (config_path.parent / relative).resolve()
        scenario = load_scenario(path)
        source = json.loads(path.read_text())
        inputs.extend([path, path.parent / source["corridor"]["path"], path.parent / source["base_stations"]["path"]])
        print(f"{scenario.scenario_id}: offset-height response grid", flush=True)
        evaluations = geometry_grid(scenario, config["offsets_m"], config["altitudes_m"], **config["evaluation"])
        rows = [e.summary for e in evaluations]
        directory = output / scenario.scenario_id
        directory.mkdir()
        write_csv(directory / "locations.csv", rows)
        write_json(directory / "locations.json", rows)
        write_json(directory / "plot_slices.json", {
            "scenario_id": scenario.scenario_id,
            "height": [r for r in rows if r["offset_m"] == slices["height_at_offset_m"]],
            "offset": [r for r in rows if r["altitude_m"] == slices["offset_at_altitude_m"]],
        })
        np.savez_compressed(directory / "traces.npz",
            offset_m=np.array([r["offset_m"] for r in rows]), altitude_m=np.array([r["altitude_m"] for r in rows]),
            profile_along_m=evaluations[0].profile["along_m"],
            profile_sinr_db=np.array([e.profile["sinr_db"] for e in evaluations]),
            profile_rsrp_dbm=np.array([e.profile["rsrp_dbm"] for e in evaluations]),
            time_s=evaluations[0].stream.time_s,
            q_uam_h=np.array([e.stream.capacity_uam_h for e in evaluations]),
            counts_crf=np.array([e.stream.counts for e in evaluations]))
        totals[scenario.scenario_id] = len(rows)

    def record(path, base):
        try:
            name = path.relative_to(base).as_posix()
        except ValueError:
            name = str(path)
        return {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    write_json(output / "manifest.json", {
        "execution": execution,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "locations_per_scenario": totals,
        "inputs": [record(p, ROOT) for p in inputs],
        "code": [record(p, ROOT) for p in sorted((ROOT / "src").rglob("*.py")) + [Path(__file__).resolve()]],
        "outputs": [record(p, output) for p in sorted(output.rglob("*")) if p.is_file()],
        "notes": config["notes"],
    })
    return totals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "research/geometry-response/configs/geometry_sweep.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(run(args.config, args.output))
    print(f"Research artifacts: {args.output.resolve()}")


if __name__ == "__main__":
    main()
