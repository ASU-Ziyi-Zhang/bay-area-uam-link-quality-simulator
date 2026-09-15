"""Run local synthetic transition mechanism checks, not Bay Area capacity studies."""
from dataclasses import asdict, replace
from datetime import datetime, timezone
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from uam_simulator.baseline_studies import write_json
from uam_simulator.dynamic_transitions import Aircraft, Layout, TransitionConfig, simulate, synthetic_radio
from uam_simulator.research_provenance import execution_metadata


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(config_path, output):
    config_path, output = Path(config_path).resolve(), Path(output).resolve()
    spec = json.loads(config_path.read_text())
    if spec.get("schema_version") != 1 or spec.get("radio_fixture") != "synthetic_radio_v1":
        raise ValueError("unknown mechanism-check schema or radio fixture")
    if spec.get("speed_definition") != "local_straight_axis_ds_dt_not_total_3d_speed":
        raise ValueError("unsupported speed convention")
    config = TransitionConfig(**spec["parameters"])
    ids = [case["id"] for case in spec["layouts"]]
    if not ids or len(set(ids)) != len(ids) or any(not i.replace("_", "").isalnum() for i in ids):
        raise ValueError("unique simple layout IDs required")
    refinement = spec["refinement"]
    if refinement["layout_id"] not in ids or refinement["factor"] < 2:
        raise ValueError("invalid refinement specification")
    prepared = []
    for case in spec["layouts"]:
        layout = Layout(tuple(case["offsets_m"]), tuple(case["heights_m"]))
        fleet = [Aircraft(**{**a, "cell": tuple(a["cell"])}) for a in case["initial_aircraft"]]
        for a in fleet:
            layout.position(a.cell)
        prepared.append((case["id"], layout, fleet))
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to replace existing results: {output}")
    execution = execution_metadata(ROOT, [str(Path(__file__).resolve()), "--config", str(config_path),
                                         "--output", str(output)])
    input_digest = digest(config_path)
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "resolved_config.json", {**spec, "parameters": asdict(config)})
    # Freeze actual source BEFORE running. A dirty HEAD alone is not reproducible.
    code_paths = sorted((ROOT / "src").rglob("*.py")) + [Path(__file__).resolve(), ROOT / "pyproject.toml"]
    code = [{"path": p.relative_to(ROOT).as_posix(), "sha256": digest(p)} for p in code_paths]
    with zipfile.ZipFile(output / "source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for p in code_paths:
            archive.write(p, p.relative_to(ROOT).as_posix())
        archive.write(config_path, "research/dynamic-transitions/configs/mechanism_checks.json")
    execution["source_snapshot_included"] = True
    execution["reproducibility_note"] = (
        "Actual source snapshot is included; dirty Git remains declared. "
        "Environment is described, not a bit-for-bit bundled runtime.")
    summaries, log = [], []
    for layout_id, layout, fleet in prepared:
        variants = [(mode, enabled, 1) for mode in ("fixed_cruise", "acc") for enabled in (False, True)]
        if layout_id == refinement["layout_id"]:
            variants += [(mode, True, refinement["factor"]) for mode in ("fixed_cruise", "acc")]
        for mode, enabled, factor in variants:
            name = f'{layout_id}_{mode}_{"on" if enabled else "off"}_refine{factor}'
            print(name, flush=True)
            local_config = replace(config, dt_s=config.dt_s/factor, forecast_dt_s=config.forecast_dt_s/factor)
            result = simulate(fleet, layout, local_config, synthetic_radio, mode=mode, transitions=enabled)
            result["summary"].update(case_id=name, layout=layout_id, refinement_factor=factor,
                dt_s=local_config.dt_s, forecast_dt_s=local_config.forecast_dt_s)
            directory = output / name
            directory.mkdir()
            write_json(directory / "summary.json", result["summary"])
            write_json(directory / "events.json", result["events"])
            with gzip.open(directory / "trace.json.gz", "wt", encoding="utf-8") as stream:
                json.dump(result["trace"], stream, allow_nan=False)
            summaries.append(result["summary"])
            log.append({"case": name, "status": result["summary"]["status"]})
    write_json(output / "summary.json", summaries)
    write_json(output / "run_log.json", log)
    drift = [row["path"] for row in code if digest(ROOT / row["path"]) != row["sha256"]]
    input_changed = digest(config_path) != input_digest
    artifacts = [{"path": p.relative_to(output).as_posix(), "sha256": digest(p)}
                 for p in sorted(output.rglob("*")) if p.is_file()]
    write_json(output / "manifest.json", {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "study": spec["study"], "execution": execution, "code": code,
        "input": {"original_path": str(config_path), "sha256": input_digest},
        "outputs": artifacts, "source_changed_during_run": drift,
        "input_changed_during_run": input_changed,
        "status": "completed" if not drift and not input_changed and all(s["status"] == "completed_sampled_checks" for s in summaries)
                  else "requires_review", "notes": spec["notes"],
    })
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "research/dynamic-transitions/configs/mechanism_checks.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.config, args.output)
    print(f"Mechanism checks: {args.output.resolve()}")
    if json.loads((args.output / "manifest.json").read_text())["status"] != "completed":
        raise SystemExit("Run preserved but requires review; inspect manifest and events.")


if __name__ == "__main__":
    main()
