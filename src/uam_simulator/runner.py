"""Executable single-UAM, fixed-speed corridor simulator.

This first runner is deliberately conservative: it advances one UAM on the
declared corridor, evaluates the existing radio kernel at its own clock, and
writes trace files.  Policy and capacity implementations can consume these
traces later without changing the clock or state schema.
"""

from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path
import sys
from typing import Iterable

import numpy as np

from capacity_policy.radio import compute_link_state
from capacity_policy.scenario import load_scenario
from capacity_policy.trajectory import TrajectoryState

from .config import SimulatorConfig, load_simulator_config


def make_time_grid(stop_s: float, dt_s: float) -> np.ndarray:
    """Return regular samples plus one exact terminal sample if needed."""
    if stop_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("stop_s and dt_s must be positive")
    count = int(np.floor(stop_s / dt_s))
    grid = np.arange(count + 1, dtype=float) * dt_s
    if grid[-1] < stop_s - 1e-9:
        grid = np.append(grid, stop_s)
    else:
        grid[-1] = stop_s
    return grid


def _is_tick(timestamp_s: float, dt_s: float) -> bool:
    return bool(np.isclose(timestamp_s / dt_s, round(timestamp_s / dt_s), atol=1e-9, rtol=0.0))


def _write_csv(path: Path, rows: Iterable[dict[str, object]]) -> int:
    materialized = list(rows)
    if not materialized:
        raise ValueError(f"cannot write empty CSV: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(materialized[0]))
        writer.writeheader()
        writer.writerows(materialized)
    return len(materialized)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _portable_path(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError:
        return resolved.name


def _scenario_input_paths(scenario_path: Path) -> tuple[Path, Path]:
    """Resolve route and site inputs from a scenario pack."""
    payload = json.loads(scenario_path.read_text(encoding="utf-8"))
    corridor_path = (scenario_path.parent / payload["corridor"]["path"]).resolve()
    site_path = (scenario_path.parent / payload["base_stations"]["path"]).resolve()
    return corridor_path, site_path


def run_simulation(
    config_path: str | Path,
    output_dir: str | Path,
    *,
    overwrite: bool = False,
) -> dict[str, object]:
    """Run the fixed-speed baseline and return a reproducibility summary."""
    cfg: SimulatorConfig = load_simulator_config(config_path)
    project_root = Path(__file__).resolve().parents[2]
    scenario = load_scenario(cfg.simulation.scenario_path)
    if not np.isclose(cfg.speed_mps, scenario.speed_mps, atol=1e-9, rtol=0.0):
        raise ValueError(
            "simulator speed differs from scenario speed; create an explicit "
            "scenario variant instead of silently changing the baseline"
        )
    stop_s = cfg.duration_s if cfg.duration_s is not None else scenario.transit_time_s
    clock = cfg.simulation.clock
    motion_times = make_time_grid(stop_s, clock.dt_motion_s)
    motion_along_m = np.clip(cfg.speed_mps * motion_times, 0.0, scenario.corridor.length_m)
    motion_xy = scenario.corridor.interpolate(motion_along_m, cfg.lateral_offset_m)
    motion_position = np.column_stack(
        [motion_xy, np.full(motion_times.shape, cfg.altitude_m)]
    )
    radio_times = make_time_grid(stop_s, clock.dt_radio_s)
    along_m = np.clip(cfg.speed_mps * radio_times, 0.0, scenario.corridor.length_m)
    xy = scenario.corridor.interpolate(along_m, cfg.lateral_offset_m)
    position = np.column_stack([xy, np.full(radio_times.shape, cfg.altitude_m)])
    active = (along_m >= 0.0) & (along_m <= scenario.corridor.length_m + 1e-9)
    trajectory = TrajectoryState(
        time_s=radio_times,
        position_m=position[None, :, :],
        along_m=along_m[None, :],
        active=active[None, :],
    )
    radio = compute_link_state(trajectory, scenario.base_stations, scenario.radio)
    site_ids = np.asarray(scenario.base_stations.site_ids)
    serving_index = radio["serving_bs"][0].astype(int)
    serving_ids = site_ids[serving_index]
    serving_rsrp = radio["serving_rsrp_dbm"][0]
    sinr = radio["sinr_db"][0]
    handoff = np.zeros(radio_times.shape, dtype=bool)
    handoff[1:] = serving_index[1:] != serving_index[:-1]
    control_tick = np.asarray([_is_tick(t, clock.dt_control_s) for t in radio_times])
    resource_offset_db = 10.0 * np.log10(scenario.radio.resource_elements)
    all_rsrp = radio["received_power_dbm"][0] - resource_offset_db
    stations_xyz = scenario.base_stations.positions(scenario.radio.assumed_bs_height_m)
    distances = np.linalg.norm(position[:, None, :] - stations_xyz[None, :, :], axis=2)

    state_rows = []
    for i, timestamp_s in enumerate(radio_times):
        state_rows.append(
            {
                "timestamp_s": float(timestamp_s),
                "uam_id": cfg.uam_id,
                "corridor_id": scenario.scenario_id,
                "s_m": float(along_m[i]),
                "level_id": cfg.level_id,
                "lane_id": cfg.lane_id,
                "x_m": float(position[i, 0]),
                "y_m": float(position[i, 1]),
                "z_m": float(position[i, 2]),
                "speed_mps": cfg.speed_mps,
                "active": bool(active[i]),
                "serving_bs_id": str(serving_ids[i]),
                "serving_rsrp_dbm_per_re": float(serving_rsrp[i]),
                "sinr_db": float(sinr[i]),
                "handoff": bool(handoff[i]),
                "control_tick": bool(control_tick[i]),
            }
        )

    measurement_rows = []
    for i, timestamp_s in enumerate(radio_times):
        for station_index, site_id in enumerate(site_ids):
            measurement_rows.append(
                {
                    "timestamp_s": float(timestamp_s),
                    "uam_id": cfg.uam_id,
                    "bs_id": str(site_id),
                    "distance_m": float(distances[i, station_index]),
                    "received_power_dbm_full_carrier": float(radio["received_power_dbm"][0, i, station_index]),
                    "rsrp_dbm_per_re": float(all_rsrp[i, station_index]),
                    "is_serving": bool(serving_index[i] == station_index),
                }
            )

    control_times = make_time_grid(stop_s, clock.dt_control_s)
    control_rows = []
    for timestamp_s in control_times:
        s_m = float(np.clip(cfg.speed_mps * timestamp_s, 0.0, scenario.corridor.length_m))
        control_rows.append(
            {
                "timestamp_s": float(timestamp_s),
                "uam_id": cfg.uam_id,
                "action_type": "hold_baseline",
                "target_speed_mps": cfg.speed_mps,
                "target_level_id": cfg.level_id,
                "target_lane_id": cfg.lane_id,
                "s_m": s_m,
                "policy_implemented": False,
                "controller_implemented": False,
            }
        )

    motion_rows = []
    for i, timestamp_s in enumerate(motion_times):
        motion_rows.append(
            {
                "timestamp_s": float(timestamp_s),
                "uam_id": cfg.uam_id,
                "corridor_id": scenario.scenario_id,
                "s_m": float(motion_along_m[i]),
                "level_id": cfg.level_id,
                "lane_id": cfg.lane_id,
                "x_m": float(motion_position[i, 0]),
                "y_m": float(motion_position[i, 1]),
                "z_m": float(motion_position[i, 2]),
                "speed_mps": cfg.speed_mps,
                "active": bool(motion_along_m[i] <= scenario.corridor.length_m + 1e-9),
            }
        )

    output = Path(output_dir).resolve()
    if output.exists() and any(output.iterdir()) and not overwrite:
        raise FileExistsError(f"output directory is non-empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    state_path = output / "uam_state_trace.csv"
    motion_path = output / "motion_state_trace.csv"
    measurement_path = output / "radio_measurements.csv"
    control_path = output / "control_ticks.csv"
    summary_path = output / "summary.json"
    n_state = _write_csv(state_path, state_rows)
    n_motion = _write_csv(motion_path, motion_rows)
    n_measurements = _write_csv(measurement_path, measurement_rows)
    n_controls = _write_csv(control_path, control_rows)
    summary = {
        "schema_version": 1,
        "status": "pass",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "config_path": _portable_path(cfg.source_path, project_root),
        "scenario_path": _portable_path(Path(cfg.simulation.scenario_path), project_root),
        "scenario_id": scenario.scenario_id,
        "corridor_length_km": scenario.corridor.length_m / 1000.0,
        "site_count": len(site_ids),
        "site_ids": [str(site_id) for site_id in site_ids],
        "uam_id": cfg.uam_id,
        "trajectory": {
            "speed_mps": cfg.speed_mps,
            "altitude_m": cfg.altitude_m,
            "lateral_offset_m": cfg.lateral_offset_m,
            "level_id": cfg.level_id,
            "lane_id": cfg.lane_id,
            "stop_s": float(stop_s),
        },
        "clock": asdict(clock),
        "samples": {
            "radio_state_rows": n_state,
            "motion_state_rows": n_motion,
            "radio_measurement_rows": n_measurements,
            "control_tick_rows": n_controls,
            "handoff_count": int(handoff.sum()),
        },
        "outputs": [path.name for path in (motion_path, state_path, measurement_path, control_path)],
        "scientific_boundary": "Deterministic single-UAM centerline RSRP/SINR trace; policy, controller, multi-UAM conflict, and capacity models are not yet implemented.",
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    scenario_path = Path(cfg.simulation.scenario_path).resolve()
    corridor_path, site_path = _scenario_input_paths(scenario_path)
    input_paths = [
        cfg.source_path,
        scenario_path,
        corridor_path,
        site_path,
    ]
    code_paths = [
        Path(__file__).resolve(),
        Path(__file__).with_name("config.py").resolve(),
        Path(__file__).with_name("interfaces.py").resolve(),
        project_root / "src" / "capacity_policy" / "radio.py",
        project_root / "src" / "capacity_policy" / "scenario.py",
        project_root / "src" / "capacity_policy" / "geometry.py",
        project_root / "src" / "capacity_policy" / "base_stations.py",
        project_root / "src" / "capacity_policy" / "trajectory.py",
    ]
    manifest = {
        "schema_version": 1,
        "run_id": output.name,
        "stage": "simulator",
        "standalone": True,
        "status": "awaiting_review",
        "created_utc": summary["created_utc"],
        "objective": f"Run one UAM through scenario {scenario.scenario_id} and record multi-rate motion, radio, and control-clock traces.",
        "command": (
            "python scripts/run_simulator.py "
            f"--config {_portable_path(cfg.source_path, project_root)} --output runs/{output.name}"
        ),
        "inputs": [
            {"path": _portable_path(path, project_root), "sha256": _sha256(path)}
            for path in input_paths
            if path.exists()
        ],
        "code_snapshot": [
            {"path": _portable_path(path, project_root), "sha256": _sha256(path)}
            for path in code_paths
            if path.exists()
        ],
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "outputs": [
            {"path": path.name, "bytes": path.stat().st_size, "sha256": _sha256(path)}
            for path in (motion_path, state_path, measurement_path, control_path, summary_path)
        ],
        "qa": {
            "status": "pass",
            "all_values_finite": bool(
                np.all(np.isfinite(position))
                and np.all(np.isfinite(radio["serving_rsrp_dbm"]))
                and np.all(np.isfinite(radio["sinr_db"]))
            ),
            "corridor_endpoint_reached": bool(
                np.isclose(along_m[-1], scenario.corridor.length_m, atol=1e-6)
            ),
            "serving_site_count": int(np.unique(serving_index).size),
            "handoff_count": int(handoff.sum()),
        },
        "evidence_grade": "traceable and rerunnable deterministic planning-model baseline",
        "limitations": [
            "No controller, policy threshold, TTT, multi-UAM conflict, or capacity model is active.",
            "Radio uses the existing deterministic LOS/co-channel planning kernel.",
            "The run is not measured coverage or operator-network validation.",
        ],
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    review = f"""# Simulator review packet - {output.name}

## Objective

Run one UAM along the declared scenario centerline and record motion, radio, and
control-clock traces using the multi-rate simulator.

## Acceptance checks

- Scenario: `{scenario.scenario_id}`; {len(site_ids)} retained base stations.
- Corridor length: {scenario.corridor.length_m / 1000.0:.6f} km.
- Fixed trajectory: {cfg.altitude_m} m altitude, {cfg.lateral_offset_m} m lateral offset, {cfg.speed_mps} m/s.
- Motion/radio/control clocks: {clock.dt_motion_s}/{clock.dt_radio_s}/{clock.dt_control_s} s.
- Endpoint reached: {manifest['qa']['corridor_endpoint_reached']}.
- Finite position and radio values: {manifest['qa']['all_values_finite']}.
- Handoffs: {manifest['qa']['handoff_count']}.

## Artifacts

See `manifest.json` for input/code/output SHA-256 records. The CSV traces
are the editable numerical outputs for inspection.

## Boundary

This is a deterministic single-UAM link-quality baseline. Policy, controller,
multi-UAM conflicts, and capacity are not active.

## Decision requested

Approve this baseline, request a correction, or branch to the next policy/
controller implementation.
"""
    (output / "REVIEW.md").write_text(review, encoding="utf-8")
    return summary
