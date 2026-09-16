"""Reproducible experiment orchestration for the two ITL baseline extensions."""

from __future__ import annotations

from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import platform

import numpy as np
from pyproj import Transformer

from capacity_policy import load_scenario
from .baseline_core import FixedStream, aggregate_streams, evaluate_stream
from .research_provenance import execution_metadata
from .path_design import offset_path, path_constraints, polyline_curvature_diagnostic, search_offsets


def write_json(path: Path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v) if isinstance(v, (list, dict)) else v for k, v in row.items()})


def layout_streams(layout: dict, mode: str, demand: float) -> list[FixedStream]:
    if mode not in ("fixed_total", "fixed_per_stream"):
        raise ValueError("unknown demand allocation mode")
    offsets, altitudes = layout["offsets_m"], layout["altitudes_m"]
    if not offsets or not altitudes or len(set(offsets)) != len(offsets) or len(set(altitudes)) != len(altitudes):
        raise ValueError("each layout requires distinct offsets and levels")
    per_stream = demand / (len(offsets) * len(altitudes)) if mode == "fixed_total" else demand
    return [FixedStream(f"lane_{i}", f"level_{j}", offset, altitude, per_stream)
            for j, altitude in enumerate(altitudes) for i, offset in enumerate(offsets)]


def lane_study(scenario, config: dict, output: Path, rho: float) -> dict:
    output.mkdir()
    conditions, stream_rows = [], []
    cache = {}
    for estimator in config["estimators"]:
        for mode in config["demand_modes"]:
            for demand in config["demand_values_uam_h"]:
                for layout in config["layouts"]:
                    case = f'{estimator["id"]}_{mode}_{demand:g}_{layout["id"]}'
                    specs = layout_streams(layout, mode, demand)
                    evaluations = []
                    for spec in specs:
                        key = (estimator["id"], spec.offset_m, spec.altitude_m, spec.demand_uam_h)
                        if key not in cache:
                            cache[key] = evaluate_stream(scenario, spec, dt_s=estimator["dt_s"],
                                policy_definition=estimator["policy_definition"], rho=rho)
                        evaluation = cache[key]
                        evaluations.append(evaluation)
                        # IDs belong to this layout; numeric evaluations may be reused.
                        stream_rows.append({"case": case, **evaluation.summary,
                                            "lane_id": spec.lane_id, "level_id": spec.level_id})
                    aggregated = aggregate_streams(evaluations, rho)
                    conditions.append({"case": case, "layout": layout["id"], "estimator": estimator["id"],
                        "demand_mode": mode, "demand_parameter_uam_h": demand, **aggregated["summary"]})
                    np.savez_compressed(output / f"{case}.npz", time_s=aggregated["time_s"],
                        q_total_uam_h=aggregated["capacity_uam_h"],
                        q_stream_uam_h=np.array([e.capacity_uam_h for e in evaluations]),
                        counts_crf=np.array([e.counts for e in evaluations]),
                        active_count=np.array([e.active_count for e in evaluations]),
                        stream_ids=np.array([f"{s.lane_id}:{s.level_id}" for s in specs]))
    write_csv(output / "conditions.csv", conditions)
    write_csv(output / "streams.csv", stream_rows)
    result = {"scenario_id": scenario.scenario_id, "conditions": conditions, "streams": stream_rows}
    write_json(output / "summary.json", result)
    return result


def offset_study(scenario, config: dict, output: Path, rho: float) -> dict:
    output.mkdir()
    constraints = {k: config[k] for k in ("minimum_radius_m", "maximum_deviation_m", "maximum_length_ratio")}
    # One shared horizon/warmup across raw, smoothed and optimized geometries.
    warmup = scenario.transit_time_s * config["maximum_length_ratio"] + scenario.policy.window_s
    if warmup >= scenario.simulation_duration_s:
        raise ValueError("duration is too short for a common post-transit scoring window")
    spec = FixedStream("lane_0", "level_0", 0.0, config["altitude_m"], config["demand_uam_h"])
    dt = config["objective_dt_s"]
    definition = config["objective_policy_definition"]

    def evaluate(path=None, offset=0.0):
        local_spec = FixedStream(spec.lane_id, spec.level_id, offset, spec.altitude_m, spec.demand_uam_h)
        return evaluate_stream(scenario, local_spec, route=path, dt_s=dt,
                               policy_definition=definition, warmup_s=warmup, rho=rho)

    def objective(path):
        value = evaluate(path).summary["q_rho_uam_h"]
        if value is None:
            raise ValueError("no valid group observations for optimization objective")
        return value

    raw_scan = []
    for offset in config["fixed_offset_values_m"]:
        raw_scan.append({"comparison": "legacy_constant_offset_diagnostic", **evaluate(offset=offset).summary,
                         "curvature_constrained": False, "endpoints_fixed": offset == 0})
    write_csv(output / "legacy_fixed_offset_scan.csv", raw_scan)
    base = offset_path(scenario.corridor, np.zeros(config["control_count"]), arc_step_m=config["arc_step_m"])
    base_constraints = path_constraints(base, scenario.corridor, **constraints)
    smoothed = {"constraints": base_constraints, "metrics": evaluate(base).summary}
    search_keys = ("control_count", "max_offset_m", "seed_offsets_m", "steps_m",
                   "passes_per_step", "max_evaluations", "arc_step_m")
    search = search_offsets(scenario.corridor, objective,
        **{k: config[k] for k in search_keys}, **constraints)
    write_csv(output / "candidates.csv", search["records"])
    best_path = search.pop("best_path")
    validation = None
    if best_path is not None:
        best = evaluate(best_path)
        fine_path = offset_path(scenario.corridor, search["best"]["offset_controls_m"],
                                arc_step_m=config["validation_arc_step_m"])
        fine_constraints = path_constraints(fine_path, scenario.corridor, **constraints)
        fine = evaluate(fine_path)
        # All-active crosscheck is separate from the full-group optimization objective.
        display = evaluate_stream(scenario, spec, route=fine_path, dt_s=1.0,
                                 policy_definition="all_active", rho=rho)
        validation = {
            "selected_metrics": best.summary, "fine_arc_metrics": fine.summary,
            "fine_arc_constraints": fine_constraints,
            "length_difference_m": fine_path.length_m - best_path.length_m,
            "objective_difference_uam_h": fine.summary["q_rho_uam_h"] - best.summary["q_rho_uam_h"],
            "dashboard_style_all_active_1s": display.summary,
        }
        if not fine_constraints["feasible"]:
            raise RuntimeError("selected candidate failed finer arc-table validation")
        reference = evaluate()
        smooth_eval = evaluate(base)
        np.savez_compressed(output / "comparison_capacity.npz", time_s=best.time_s,
            reference_q=reference.capacity_uam_h, smoothed_q=smooth_eval.capacity_uam_h,
            selected_q=best.capacity_uam_h, fine_q=fine.capacity_uam_h)
        fraction = np.linspace(0, 1, 2001)
        xy = fine_path.at_parameter(fraction * (len(fine_path.control_xy_m) - 1))
        inverse = Transformer.from_crs(scenario.corridor.crs, "EPSG:4326", always_xy=True)
        lon, lat = inverse.transform(xy[:, 0], xy[:, 1])
        write_json(output / "selected_path.geojson", {"type": "FeatureCollection", "features": [{
            "type": "Feature", "properties": {"scenario": scenario.scenario_id,
                "altitude_m": spec.altitude_m, "status": "research_candidate_not_operational_route"},
            "geometry": {"type": "LineString", "coordinates": np.column_stack([lon, lat]).tolist()},
        }]})
        write_csv(output / "selected_path_controls.csv", [{"control": i, "x_m": float(x), "y_m": float(y),
            "offset_m": search["best"]["offset_controls_m"][i]} for i, (x, y) in enumerate(fine_path.control_xy_m)])
    result = {"scenario_id": scenario.scenario_id,
        "raw_geometry": polyline_curvature_diagnostic(scenario.corridor),
        "scoring": {"policy_definition": definition, "dt_s": dt, "warmup_s": warmup,
                    "duration_s": scenario.simulation_duration_s, "objective": "Q_rho", "rho": rho},
        "legacy_fixed_offset_scan": raw_scan, "zero_control_smoothed_baseline": smoothed,
        "search": search, "validation": validation}
    write_json(output / "summary.json", result)
    return result


def run_baseline_studies(config_path: str | Path, output_dir: str | Path, study="all") -> dict:
    if study not in ("all", "lanes", "offsets"):
        raise ValueError("unknown study")
    config_path, output = Path(config_path).resolve(), Path(output_dir).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported research config schema")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to replace existing results: {output}")
    root = Path(__file__).resolve().parents[2]
    execution = execution_metadata(root, [str(root / "scripts/run_baseline_studies.py"),
        "--config", str(config_path), "--output", str(output), "--study", study])
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "resolved_config.json", config)
    results = {}
    inputs = [config_path]
    for relative in config["scenarios"]:
        path = (config_path.parent / relative).resolve()
        scenario = load_scenario(path)
        scenario_config = json.loads(path.read_text(encoding="utf-8"))
        inputs.extend([path, path.parent / scenario_config["corridor"]["path"],
                       path.parent / scenario_config["base_stations"]["path"]])
        directory = output / scenario.scenario_id
        directory.mkdir()
        results[scenario.scenario_id] = {}
        if study in ("all", "lanes"):
            print(f"{scenario.scenario_id}: fixed-stream comparisons", flush=True)
            results[scenario.scenario_id]["lanes"] = lane_study(scenario, config["lanes"], directory / "lanes", config["reliability_rho"])
        if study in ("all", "offsets"):
            print(f"{scenario.scenario_id}: constrained offset search", flush=True)
            results[scenario.scenario_id]["offsets"] = offset_study(scenario, config["offsets"], directory / "offsets", config["reliability_rho"])
    root = Path(__file__).resolve().parents[2]

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def portable(path):
        try:
            return path.resolve().relative_to(root).as_posix()
        except ValueError:
            return str(path.resolve())

    code = sorted((root / "src").rglob("*.py")) + [root / "scripts/run_baseline_studies.py"]
    artifacts = sorted(p for p in output.rglob("*") if p.is_file())
    manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "execution": execution,
        "study": study, "command": f"python scripts/run_baseline_studies.py --config {portable(config_path)} --output {portable(output)} --study {study}",
        "python": platform.python_version(), "numpy": np.__version__,
        "inputs": [{"path": portable(p), "sha256": digest(p)} for p in inputs],
        "code": [{"path": portable(p), "sha256": digest(p)} for p in code],
        "outputs": [{"path": p.relative_to(output).as_posix(), "sha256": digest(p)} for p in artifacts],
        "assumptions": config.get("assumptions", []),
        "scientific_status": "exploratory conditional planning estimates; not validated operational capacity"}
    write_json(output / "manifest.json", manifest)
    return results
