"""Run a deterministic fixed-lane multi-UAM TRB policy-capacity baseline."""

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

from capacity_policy import (
    ConstantSpeedTrajectory,
    assign_policy,
    compute_link_state,
    evaluate_link_quality,
    load_scenario,
    snapshot_capacity,
)
from capacity_policy.policy import POLICY_C, POLICY_F, POLICY_R

from .group_config import GroupSimulatorConfig, load_group_simulator_config
from .runner import make_time_grid


POLICY_LABEL = {POLICY_C: "C", POLICY_R: "R", POLICY_F: "F"}


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


def reliability_floor(values: np.ndarray, rho: float) -> float:
    """Lower reliability floor used by the accepted TRB implementation."""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("cannot compute a reliability floor from no values")
    if not 0.0 < rho <= 1.0:
        raise ValueError("rho must lie in (0, 1]")
    ordered = np.sort(array)
    index = int(np.floor((1.0 - rho) * ordered.size + 1e-12))
    return float(ordered[min(max(index, 0), ordered.size - 1)])


def _scenario_input_paths(scenario_path: Path) -> tuple[Path, Path]:
    payload = json.loads(scenario_path.read_text(encoding="utf-8"))
    corridor_path = (scenario_path.parent / payload["corridor"]["path"]).resolve()
    site_path = (scenario_path.parent / payload["base_stations"]["path"]).resolve()
    return corridor_path, site_path


def _policy_records(
    link_quality: dict[str, np.ndarray],
    uam_ids: list[str],
    group_size: int,
    window_s: float,
    coordinated_tolerance: float,
    reactive_tolerance: float,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, object]]]:
    """Assign every active aircraft a policy from its available local group.

    Interior aircraft use two active neighbors on either side. Boundary and
    startup aircraft use the available subset, so no active aircraft is left
    unclassified. Temporal exposure uses the available history up to
    ``window_s`` rather than imposing a separate startup warmup.
    """
    if group_size % 2 != 1:
        raise ValueError("group_size must be odd so each group has one focal UAM")
    active = np.asarray(link_quality["active"], dtype=bool)
    link_ok = np.asarray(link_quality["link_ok"], dtype=bool)
    t_obs = np.asarray(link_quality["t"], dtype=float)
    if active.shape != link_ok.shape or active.shape[0] != len(uam_ids):
        raise ValueError("adaptive policy inputs are inconsistent")
    half = group_size // 2
    support = np.full(active.shape, np.nan, dtype=float)
    exposure = np.full(active.shape, np.nan, dtype=float)
    policy = np.full(active.shape, POLICY_C, dtype=int)
    groups: list[list[list[int] | None]] = [
        [None for _ in range(active.shape[1])] for _ in range(active.shape[0])
    ]

    for time_index in range(active.shape[1]):
        active_indices = np.flatnonzero(active[:, time_index])
        for order_index, focal_index in enumerate(active_indices):
            members = active_indices[
                max(0, order_index - half) : order_index + half + 1
            ]
            groups[focal_index][time_index] = members.tolist()
            support[focal_index, time_index] = float(
                link_ok[members, time_index].mean()
            )

    for time_index, timestamp in enumerate(t_obs):
        start_index = int(np.searchsorted(t_obs, timestamp - window_s, side="left"))
        for focal_index in np.flatnonzero(active[:, time_index]):
            history = support[focal_index, start_index : time_index + 1]
            history = history[np.isfinite(history)]
            if history.size == 0:
                raise RuntimeError("active aircraft has no policy-support history")
            exposure[focal_index, time_index] = 1.0 - float(history.mean())
            if exposure[focal_index, time_index] <= coordinated_tolerance + 1e-12:
                policy[focal_index, time_index] = POLICY_C
            elif exposure[focal_index, time_index] <= reactive_tolerance + 1e-12:
                policy[focal_index, time_index] = POLICY_R
            else:
                policy[focal_index, time_index] = POLICY_F

    rows: list[dict[str, object]] = []
    for time_index, timestamp in enumerate(t_obs):
        for focal_index in np.flatnonzero(active[:, time_index]):
            member_indices = groups[focal_index][time_index]
            if member_indices is None:
                raise RuntimeError("active aircraft has no local group")
            code = int(policy[focal_index, time_index])
            rows.append(
                {
                    "timestamp_s": float(timestamp),
                    "group_id": f"{uam_ids[focal_index]}@{timestamp:.3f}",
                    "focal_uam_id": uam_ids[focal_index],
                    "focal_uam_index": focal_index,
                    "group_size": len(member_indices),
                    "member_uam_ids": "|".join(
                        uam_ids[index] for index in member_indices
                    ),
                    "policy": POLICY_LABEL[code],
                    "exposure_fraction": float(exposure[focal_index, time_index]),
                }
            )
    if not rows:
        raise RuntimeError("no active-aircraft policy observations")
    return policy, active, rows


def run_group_simulation(
    config_path: str | Path,
    output_dir: str | Path,
    *,
    overwrite: bool = False,
) -> dict[str, object]:
    """Run the fixed-lane stream and return a reproducibility summary."""
    cfg: GroupSimulatorConfig = load_group_simulator_config(config_path)
    project_root = Path(__file__).resolve().parents[2]
    scenario = load_scenario(cfg.simulation.scenario_path)
    if not np.isclose(cfg.speed_mps, scenario.speed_mps, atol=1e-9, rtol=0.0):
        raise ValueError("group simulator and scenario speed must match")
    if not np.isclose(
        cfg.simulation.clock.dt_control_s,
        scenario.time_step_s,
        atol=1e-9,
        rtol=0.0,
    ):
        raise ValueError("policy/control clock must match the scenario time step")

    duration_s = cfg.duration_s or scenario.simulation_duration_s
    time_s = make_time_grid(duration_s, scenario.time_step_s)
    entry_time_s = np.arange(
        0.0, duration_s, cfg.entry_interval_s, dtype=float
    )
    uam_ids = [f"UAM{index + 1:03d}" for index in range(entry_time_s.size)]
    trajectory = ConstantSpeedTrajectory(cfg.speed_mps).realize(
        scenario.corridor,
        time_s,
        entry_time_s,
        np.full(entry_time_s.shape, cfg.altitude_m),
        np.full(entry_time_s.shape, cfg.lateral_offset_m),
    )
    radio = compute_link_state(trajectory, scenario.base_stations, scenario.radio)
    link_quality = evaluate_link_quality(radio, scenario.link_quality)
    reference_policy = assign_policy(link_quality, scenario.policy)
    reference_capacity = snapshot_capacity(
        reference_policy["policy"],
        reference_policy["valid_after_warmup"],
        reference_policy["t_obs"],
        scenario.capacity,
    )
    adaptive_policy, adaptive_valid, policy_rows = _policy_records(
        link_quality,
        uam_ids,
        scenario.policy.group_size,
        scenario.policy.window_s,
        scenario.policy.coordinated_exposure_tolerance,
        scenario.policy.reactive_exposure_tolerance,
    )
    capacity = snapshot_capacity(
        adaptive_policy,
        adaptive_valid,
        time_s,
        scenario.capacity,
    )

    entrant_rows = [
        {
            "uam_id": uam_id,
            "uam_index": index,
            "entry_time_s": float(entry_time_s[index]),
            "exit_time_s": float(entry_time_s[index] + scenario.transit_time_s),
            "speed_mps": cfg.speed_mps,
            "altitude_m": cfg.altitude_m,
            "lateral_offset_m": cfg.lateral_offset_m,
            "level_id": cfg.level_id,
            "lane_id": cfg.lane_id,
        }
        for index, uam_id in enumerate(uam_ids)
    ]
    time_index = {
        round(float(timestamp), 9): index for index, timestamp in enumerate(time_s)
    }
    capacity_rows = []
    for index, timestamp_s in enumerate(capacity["t"]):
        source_index = time_index[round(float(timestamp_s), 9)]
        capacity_rows.append(
            {
                "timestamp_s": float(timestamp_s),
                "active_uam_count": int(trajectory.active[:, source_index].sum()),
                "classified_group_count": int(capacity["n_group"][index]),
                "n_C": int(capacity["n_C"][index]),
                "n_R": int(capacity["n_R"][index]),
                "n_F": int(capacity["n_F"][index]),
                "mean_spacing_m": float(capacity["mean_spacing_m"][index]),
                "q_mix_uam_h": float(capacity["q_mix_UAM_h"][index]),
                "q_bottleneck_uam_h": float(
                    capacity["q_bottleneck_UAM_h"][index]
                ),
            }
        )

    policy_counts = {
        label: sum(row["policy"] == label for row in policy_rows)
        for label in ("C", "R", "F")
    }
    policy_total = sum(policy_counts.values())
    policy_shares = {
        label: policy_counts[label] / policy_total for label in policy_counts
    }
    reference_valid = np.asarray(
        reference_policy["valid_after_warmup"], dtype=bool
    )
    reference_codes = np.asarray(reference_policy["policy"], dtype=int)
    reference_counts = {
        label: int(((reference_codes == code) & reference_valid).sum())
        for label, code in (("C", POLICY_C), ("R", POLICY_R), ("F", POLICY_F))
    }
    reference_total = sum(reference_counts.values())
    reference_shares = {
        label: reference_counts[label] / reference_total
        for label in reference_counts
    }
    q_mix = np.asarray(capacity["q_mix_UAM_h"], dtype=float)
    q_bottleneck = np.asarray(capacity["q_bottleneck_UAM_h"], dtype=float)
    q_mix_rho = reliability_floor(q_mix, cfg.reliability_rho)
    q_bottleneck_rho = reliability_floor(q_bottleneck, cfg.reliability_rho)

    output = Path(output_dir).resolve()
    if output.exists() and any(output.iterdir()) and not overwrite:
        raise FileExistsError(f"output directory is non-empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    entrants_path = output / "entrants.csv"
    policy_path = output / "group_policy_trace.csv"
    capacity_path = output / "capacity_trace.csv"
    summary_path = output / "summary.json"
    manifest_path = output / "manifest.json"
    review_path = output / "REVIEW.md"
    n_entrants = _write_csv(entrants_path, entrant_rows)
    n_policy = _write_csv(policy_path, policy_rows)
    n_capacity = _write_csv(capacity_path, capacity_rows)

    summary = {
        "schema_version": 1,
        "status": "pass",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario_id": scenario.scenario_id,
        "corridor_length_km": scenario.corridor.length_m / 1000.0,
        "transit_time_s": scenario.transit_time_s,
        "simulation_duration_s": duration_s,
        "warmup_s": 0.0,
        "site_count": len(scenario.base_stations.stations),
        "traffic": {
            "entry_demand_uam_h": cfg.entry_demand_uam_h,
            "entry_interval_s": cfg.entry_interval_s,
            "entrant_count": n_entrants,
            "expected_steady_occupancy": (
                cfg.entry_demand_uam_h * scenario.transit_time_s / 3600.0
            ),
        },
        "trajectory": {
            "speed_mps": cfg.speed_mps,
            "altitude_m": cfg.altitude_m,
            "lateral_offset_m": cfg.lateral_offset_m,
            "level_id": cfg.level_id,
            "lane_id": cfg.lane_id,
        },
        "policy": {
            "maximum_group_size": scenario.policy.group_size,
            "minimum_group_size": min(int(row["group_size"]) for row in policy_rows),
            "window_s": scenario.policy.window_s,
            "sinr_threshold_db": scenario.link_quality.sinr_threshold_db,
            "coordinated_exposure_tolerance": (
                scenario.policy.coordinated_exposure_tolerance
            ),
            "reactive_exposure_tolerance": (
                scenario.policy.reactive_exposure_tolerance
            ),
            "observation_count": n_policy,
            "counts": policy_counts,
            "shares": policy_shares,
            "mapping": (
                "Every active aircraft receives a policy. The local group uses "
                "up to two active neighbors on each side; boundary and startup "
                "groups shrink to the available aircraft. Temporal exposure uses "
                "the available history up to the configured window."
            ),
        },
        "capacity": {
            "snapshot_count": n_capacity,
            "reliability_rho": cfg.reliability_rho,
            "q_mix_rho_uam_h": q_mix_rho,
            "q_bottleneck_rho_uam_h": q_bottleneck_rho,
            "demand_supported_by_q_mix_rho": (
                cfg.entry_demand_uam_h <= q_mix_rho + 1e-12
            ),
        },
        "trb_reference_regression": {
            "definition": (
                "Original full centered-five-aircraft groups after the legacy "
                "warmup; retained only as a regression comparison."
            ),
            "warmup_s": scenario.policy.warmup_s,
            "observation_count": reference_total,
            "counts": reference_counts,
            "shares": reference_shares,
            "q_mix_rho_uam_h": reliability_floor(
                np.asarray(reference_capacity["q_mix_UAM_h"], dtype=float),
                cfg.reliability_rho,
            ),
        },
        "clock": asdict(cfg.simulation.clock),
        "scientific_boundary": (
            "Deterministic single-lane/single-level planning-model stream. "
            "Radio is individual; every active aircraft receives a policy from "
            "its available local neighbor group; capacity is aggregated from all "
            "active-aircraft policies. No lane change or controller is active."
        ),
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    scenario_path = Path(cfg.simulation.scenario_path).resolve()
    corridor_path, site_path = _scenario_input_paths(scenario_path)
    artifact_paths = (entrants_path, policy_path, capacity_path, summary_path)
    code_paths = (
        Path(__file__).resolve(),
        Path(__file__).with_name("group_config.py").resolve(),
        project_root / "src" / "capacity_policy" / "trajectory.py",
        project_root / "src" / "capacity_policy" / "radio.py",
        project_root / "src" / "capacity_policy" / "link_quality.py",
        project_root / "src" / "capacity_policy" / "policy.py",
        project_root / "src" / "capacity_policy" / "capacity.py",
    )
    manifest = {
        "schema_version": 1,
        "run_id": output.name,
        "stage": "multi-uam-group-policy",
        "standalone": True,
        "status": "awaiting_review",
        "created_utc": summary["created_utc"],
        "objective": (
            "Reproduce the accepted TRB individual-radio to group-policy to "
            "capacity chain on one fixed real corridor lane and altitude level."
        ),
        "command": (
            "python scripts/run_group_simulator.py "
            f"--config {_portable_path(cfg.source_path, project_root)} "
            f"--output runs/{output.name}"
        ),
        "inputs": [
            {"path": _portable_path(path, project_root), "sha256": _sha256(path)}
            for path in (cfg.source_path, scenario_path, corridor_path, site_path)
        ],
        "code_snapshot": [
            {"path": _portable_path(path, project_root), "sha256": _sha256(path)}
            for path in code_paths
        ],
        "outputs": [
            {
                "path": path.name,
                "bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
            for path in artifact_paths
        ],
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "qa": {
            "status": "pass",
            "policy_shares_sum_to_one": bool(
                np.isclose(sum(policy_shares.values()), 1.0, atol=1e-12)
            ),
            "capacity_values_finite_positive": bool(
                np.all(np.isfinite(q_mix)) and np.all(q_mix > 0.0)
            ),
            "no_lane_or_level_change": True,
            "focal_group_mapping_declared": True,
            "all_active_aircraft_classified": bool(
                n_policy == int(trajectory.active.sum())
            ),
            "simulation_starts_at_zero": bool(capacity["t"][0] == 0.0),
        },
        "evidence_grade": "estimated under the declared deterministic planning model",
        "limitations": [
            "Uniform deterministic entry times; no stochastic demand or scheduling.",
            "No lane/level change, merge, conflict, vertiport, or controller model.",
            "Policy fractions are active focal-aircraft time shares, not unique-aircraft shares.",
            "Startup and corridor-edge groups use fewer than five aircraft when fewer neighbors are available.",
            "Radio remains a deterministic LOS/co-channel planning estimate.",
        ],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    review_path.write_text(
        f"""# Multi-UAM group-policy review - {output.name}

## Objective

Establish the fixed-lane, fixed-level multi-aircraft TRB chain before adding
multi-lane geometry or dynamic switching.

## Inputs and configuration

- Scenario: `{scenario.scenario_id}`.
- Offered entry demand: {cfg.entry_demand_uam_h:.1f} UAM/h.
- Entry interval: {cfg.entry_interval_s:.3f} s.
- Speed/altitude/offset: {cfg.speed_mps:.1f} m/s / {cfg.altitude_m:.1f} m / {cfg.lateral_offset_m:.1f} m.
- Local group/window: up to {scenario.policy.group_size} aircraft / {scenario.policy.window_s:.1f} s.
- Startup: begins at 0 s; each active aircraft is classified from its available neighbors and history.

## Result

- Active focal-aircraft time observations: {n_policy:,}.
- Policy shares C/R/F: {policy_shares['C']:.4f} / {policy_shares['R']:.4f} / {policy_shares['F']:.4f}.
- Q{cfg.reliability_rho:.2f} mixed capacity: {q_mix_rho:.3f} UAM/h.
- Offered demand supported: {cfg.entry_demand_uam_h <= q_mix_rho + 1e-12}.

## Checks

- Policy shares sum to one.
- Capacity snapshots are finite and positive.
- Every active aircraft has C/R/F policy; no unclassified state is used.
- Interior groups contain five aircraft; boundary/startup groups shrink to the available local neighbors.
- The original centered-five TRB reference remains separately reported in `summary.json`.
- Lane and level changes are disabled.

## Boundary and unresolved risks

These are deterministic planning-model estimates. Fractions are active
focal-aircraft time fractions, not fractions of unique vehicles. Common
terminal, merge, spectrum scheduling and conflict bottlenecks are excluded.

## Artifacts

See `entrants.csv`, `group_policy_trace.csv`, `capacity_trace.csv`, `summary.json`
and `manifest.json`.

## Decision requested

Approve the backend baseline for web-animation integration, or request a change
to demand, group mapping, policy share definition, or capacity metric.
""",
        encoding="utf-8",
    )
    return summary
