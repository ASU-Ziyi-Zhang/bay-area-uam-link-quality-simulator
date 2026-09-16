"""Archive reviewed closed-loop mechanism pilots; NOT Bay Area capacity experiments."""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.radio import RadioConfig
from uam_simulator.dynamic_transitions import Layout
from uam_simulator.policy_motion import BSRadio, Config, Request, Simulator, Vehicle
from uam_simulator.research_provenance import execution_metadata


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(sim, case_id, transitions):
    measured = [r for r in sim.observations if r["phase"] == "CORRIDOR"]
    counts = {p: sum(r["policy"] == p for r in measured) for p in "CRF"}
    corridor = [r for r in sim.trace if r["phase"] == "CORRIDOR"]
    deficits = [r["spacing_deficit_m"] for r in corridor if r["spacing_deficit_m"] is not None]
    policy_changes = 0
    previous = {}
    for row in measured:
        aid = row["aircraft_id"]
        if aid in previous and previous[aid] != row["policy"]:
            policy_changes += 1
        previous[aid] = row["policy"]
    return {
        "case_id": case_id, "status": "failed" if sim.issue else "completed_mechanism_check",
        "issue": sim.issue, "mode": sim.mode, "transitions_enabled": transitions,
        "elapsed_s": sim.t, "aircraft_requested_or_initial": sim.total_count,
        "released_or_initial": len(sim.fleet), "unreleased": len(sim.pending),
        "corridor_entries": sum(a.entry_s is not None for a in sim.fleet),
        "corridor_exits": sum(a.exit_s is not None for a in sim.fleet),
        "completed_missions": sum(a.phase == "COMPLETE" for a in sim.fleet),
        "airborne_at_end": sum(a.phase != "COMPLETE" for a in sim.fleet),
        "policy_observations": len(measured), "policy_counts": counts,
        "policy_fractions": {p: counts[p]/len(measured) if measured else None for p in "CRF"},
        "policy_changes": policy_changes,
        "mean_group_exposure": sum(r["exposure"] for r in measured)/len(measured) if measured else None,
        "minimum_group_size": min((r["group_size"] for r in measured), default=None),
        "maximum_group_size": max((r["group_size"] for r in measured), default=None),
        "accepted_transitions": sim.accepted_moves, "completed_transitions": sim.completed_moves,
        "minimum_longitudinal_speed_mps": min((r["velocity_mps"][0] for r in corridor), default=None),
        "minimum_raw_acceleration_mps2": min((r["raw_acceleration"] for r in corridor), default=None),
        "minimum_command_acceleration_mps2": min((r["command_acceleration"] for r in corridor), default=None),
        "max_policy_spacing_deficit_m": max(deficits, default=None),
        "minimum_protection_score_bound": sim.minimum_separation if sim.minimum_separation != float("inf") else None,
        "boundary_deferred_attempts": sim.deferrals,
        "observed_exit_rate_uam_h": 3600*sum(a.exit_s is not None for a in sim.fleet)/sim.t if sim.t else None,
        "capacity_uam_h": None,
        "limitations": [
            "Local straight geometry and illustrative BS inventory, NOT actual Bay Area traffic.",
            "Protection ellipsoid values are exploratory mechanism assumptions, NOT aviation minima.",
            "Numerical polynomial interval screening is not certified continuous-time safety.",
            "Finite-window exit rate is not capacity; unfinished missions are right-censored.",
            "No load-dependent radio, wake, wind, actuator lag, or emergency controller.",
        ],
    }


def prepare(spec):
    if spec["schema_version"] != 1 or spec["study"] != "policy_spacing_closed_loop_mechanisms":
        raise ValueError("unsupported study schema")
    if spec["geometry"] != "local_straight" or spec["protection_status"] != "exploratory_not_operational":
        raise ValueError("explicit mechanism geometry/protection scope required")
    cfg = Config(**spec["parameters"])
    layout = Layout(tuple(spec["offsets_m"]), tuple(spec["heights_m"]))
    bs = BaseStationSet(tuple(BaseStation(**x) for x in spec["base_stations"]), "LOCAL_METRIC")
    radio = BSRadio(bs, RadioConfig(**spec["radio"]))
    cases = []
    ids = []
    for case in spec["cases"]:
        name = case["id"]
        if not name.replace("_", "").isalnum() or name in ids:
            raise ValueError("unique simple case IDs required")
        ids.append(name)
        local = replace(cfg, **case.get("parameters", {}))
        initial = [Vehicle(**{**a, "cell": tuple(a["cell"])}) for a in case.get("initial", [])]
        requests = [Request(**{**r, "cell": tuple(r["cell"])}) for r in case.get("requests", [])]
        for mode in case.get("modes", ["fixed_cruise", "acc"]):
            for enabled in case.get("transitions", [False, True]):
                sim = Simulator(layout, local, radio, mode, initial=initial, requests=requests)
                cases.append((f"{name}_{mode}_{'on' if enabled else 'off'}", enabled, sim))
    if not cases:
        raise ValueError("empty pilot")
    return cfg, cases


def run(config_path, output):
    config_path, output = Path(config_path).resolve(), Path(output).resolve()
    spec = json.loads(config_path.read_text())
    cfg, cases = prepare(spec)  # validate before creating a run directory
    if output.exists():
        raise FileExistsError(f"refusing to reuse any existing run directory: {output}")
    execution = execution_metadata(ROOT, [str(Path(__file__).resolve()), "--config", str(config_path),
                                         "--output", str(output)])
    sources = sorted((ROOT/"src").rglob("*.py")) + sorted((ROOT/"tests").glob("test_policy_motion.py"))
    sources += [Path(__file__).resolve(), ROOT/"pyproject.toml",
                ROOT/"research/dynamic-transitions/full-mission-model-review.md"]
    code = [{"path": p.relative_to(ROOT).as_posix(), "sha256": digest(p)} for p in sources]
    input_hash = digest(config_path)
    output.mkdir(parents=True)
    write_json(output/"resolved_config.json", {**spec, "parameters": asdict(cfg)})
    with zipfile.ZipFile(output/"source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(p, p.relative_to(ROOT).as_posix())
        archive.write(config_path, "pilot_input.json")
    execution["source_snapshot_included"] = True
    execution["reproducibility_note"] = "Actual dirty source snapshot included; dependency versions recorded, not a bundled runtime."
    summaries = []
    log = []
    manifest = {"schema_version": 1, "study": spec["study"], "status": "running",
                "created_utc": datetime.now(timezone.utc).isoformat(), "execution": execution,
                "code": code, "input_sha256": input_hash, "input_path": str(config_path)}
    write_json(output/"manifest.json", manifest)
    try:
        for name, enabled, sim in cases:
            started = time.monotonic()
            print(f"Running {name}", flush=True)
            directory = output/name
            directory.mkdir()
            sim.run(enabled)
            summary = summarize(sim, name, enabled)
            summary["wall_time_s"] = time.monotonic()-started
            summary["resolved_parameters"] = asdict(sim.cfg)
            write_json(directory/"summary.json", summary)
            write_json(directory/"events.json", sim.events)
            write_json(directory/"missions.json", [
                {"aircraft_id": a.aircraft_id, "phase": a.phase, "request_s": a.request_s,
                 "release_s": a.release_s, "entry_s": a.entry_s, "exit_s": a.exit_s,
                 "completion_s": a.completion_s, "final_cell": list(a.cell)} for a in sim.fleet])
            for filename, data in [("trace", sim.trace), ("observations", sim.observations)]:
                with gzip.open(directory/f"{filename}.json.gz", "wt", encoding="utf-8") as stream:
                    json.dump(data, stream, allow_nan=False)
            summaries.append(summary)
            log.append({"case": name, "status": summary["status"], "wall_time_s": summary["wall_time_s"]})
            write_json(output/"summary.json", summaries)
            write_json(output/"run_log.json", log)
            print(f"  {summary['status']}; policies={summary['policy_counts']}; moves={sim.completed_moves}; "
                  f"completed={summary['completed_missions']}; {summary['wall_time_s']:.2f}s", flush=True)
    except Exception as exc:
        manifest["exception"] = {"type": type(exc).__name__, "message": str(exc)}
        raise
    finally:
        drift = [row["path"] for row in code if digest(ROOT/row["path"]) != row["sha256"]]
        input_changed = digest(config_path) != input_hash
        manifest.update(source_changed_during_run=drift, input_changed_during_run=input_changed,
            status="completed" if len(summaries)==len(cases) and not drift and not input_changed
            and all(s["status"]=="completed_mechanism_check" for s in summaries) else "requires_review",
            outputs=[{"path": p.relative_to(output).as_posix(), "sha256": digest(p)}
                     for p in sorted(output.rglob("*")) if p.is_file() and p.name != "manifest.json"])
        write_json(output/"manifest.json", manifest)
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"research/dynamic-transitions/configs/policy_motion_pilot.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.config, args.output)
    if json.loads((args.output/"manifest.json").read_text())["status"] != "completed":
        raise SystemExit("Run archived; requires review. Inspect manifest and case events.")


if __name__ == "__main__":
    main()
