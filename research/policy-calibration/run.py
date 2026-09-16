"""Screen Bay Area policy-classification parameters on fixed trajectories.

This is an observation/classification experiment.  Aircraft remain on one
fixed stream at constant speed; policy labels never feed back into motion.
The deterministic sweep varies sampling, window, complete group size, SINR
threshold and persistence.  A separate local sweep exposes the integer nature
of the nominal 35-member-time window.  The noise sweep is an explicitly
illustrative iid-Gaussian stress test, not receiver calibration.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from capacity_policy import ConstantSpeedTrajectory, compute_link_state, load_scenario  # noqa: E402
from uam_simulator.runner import make_time_grid  # noqa: E402


POLICIES = ("C", "R", "F")
RADIO_STEPS_S = (1.0, 2.0, 5.0)
SETTLING_S = {
    (0, 1): 73.8,
    (1, 0): 50.7,
    (1, 2): 147.5,
    (2, 1): 99.0,
    (0, 2): 221.3,
    (2, 0): 148.5,
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError(f"no rows for {path}")
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def moving_sum(values: np.ndarray, width: int, axis: int) -> np.ndarray:
    zeros_shape = list(values.shape)
    zeros_shape[axis] = 1
    cumulative = np.concatenate(
        [np.zeros(zeros_shape, dtype=float), np.cumsum(values, axis=axis)], axis=axis
    )
    hi = [slice(None)] * values.ndim
    lo = [slice(None)] * values.ndim
    hi[axis] = slice(width, None)
    lo[axis] = slice(None, -width)
    return cumulative[tuple(hi)] - cumulative[tuple(lo)]


def fixed_trace(scenario, dt_s: float) -> dict[str, np.ndarray]:
    times = make_time_grid(scenario.simulation_duration_s, dt_s)
    entries = np.arange(0.0, scenario.simulation_duration_s, 3600.0 / 112.5)
    trajectory = ConstantSpeedTrajectory(scenario.speed_mps).realize(
        scenario.corridor,
        times,
        entries,
        np.full(entries.shape, 300.0),
        np.zeros(entries.shape),
    )
    radio = compute_link_state(trajectory, scenario.base_stations, scenario.radio)
    return {"t": times, "active": trajectory.active, "sinr_db": radio["sinr_db"]}


def exposure_for_threshold(
    trace: dict[str, np.ndarray], threshold_db: float, group_size: int, window_s: float, dt_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    active = np.asarray(trace["active"], bool)
    bad = active & (np.asarray(trace["sinr_db"], float) < threshold_db)
    group_active = moving_sum(active.astype(float), group_size, axis=0)
    group_bad = moving_sum(bad.astype(float), group_size, axis=0)
    group_full = group_active >= group_size - 0.5
    window_n = int(round(window_s / dt_s)) + 1
    exposure = moving_sum(group_bad / group_size, window_n, axis=1) / window_n
    valid = moving_sum(group_full.astype(float), window_n, axis=1) >= window_n - 0.5
    times = np.asarray(trace["t"], float)[window_n - 1 :]
    warmup_s = float(trace["warmup_s"])
    valid &= times[None, :] >= warmup_s
    return exposure, valid, times


def raw_policy(exposure: np.ndarray, c_tolerance: float, r_tolerance: float) -> np.ndarray:
    if not 0 <= c_tolerance < r_tolerance <= 1:
        raise ValueError("expected 0 <= C exposure < R exposure <= 1")
    return np.where(exposure <= c_tolerance + 1e-12, 0,
                    np.where(exposure <= r_tolerance + 1e-12, 1, 2)).astype(np.int8)


def policy_tick_samples(times: np.ndarray, policy_update_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Map each policy tick to the newest radio observation available at that tick."""
    times = np.asarray(times, dtype=float)
    first_tick = np.ceil((times[0] - 1e-12) / policy_update_s) * policy_update_s
    last_tick = np.floor((times[-1] + 1e-12) / policy_update_s) * policy_update_s
    ticks = np.arange(first_tick, last_tick + policy_update_s / 2.0, policy_update_s)
    indices = np.searchsorted(times, ticks + 1e-12, side="right") - 1
    valid = indices >= 0
    return indices[valid], ticks[valid]


def persistent_sequence(raw: np.ndarray, k: int) -> np.ndarray:
    if k < 1:
        raise ValueError("persistence must be positive")
    out = np.empty_like(raw)
    state = int(raw[0])
    candidate = state
    count = 0
    for index, value_raw in enumerate(raw):
        value = int(value_raw)
        if value == state:
            candidate, count = state, 0
        else:
            if value == candidate:
                count += 1
            else:
                candidate, count = value, 1
            if count >= k:
                state, candidate, count = candidate, candidate, 0
        out[index] = state
    return out


def metrics(policy: np.ndarray, valid: np.ndarray, times: np.ndarray, dt_s: float, k: int) -> dict:
    counts = np.zeros(3, dtype=int)
    switches = 0
    sequence_count = 0
    action_total = 0
    action_complete = 0
    action_right_censored = 0
    dwell_s: list[float] = []
    valid_observations = 0
    group_time_s = 0.0
    for group_index in range(policy.shape[0]):
        indices = np.flatnonzero(valid[group_index])
        if not len(indices):
            continue
        cuts = np.flatnonzero(np.diff(indices) > 1) + 1
        for block in np.split(indices, cuts):
            if not len(block):
                continue
            sequence_count += 1
            stable = persistent_sequence(policy[group_index, block], k)
            counts += np.bincount(stable, minlength=3)
            valid_observations += len(block)
            group_time_s += len(block) * dt_s
            changes = np.flatnonzero(stable[1:] != stable[:-1]) + 1
            switches += len(changes)
            run_starts = np.r_[0, changes]
            run_ends = np.r_[changes, len(stable)]
            for run_number in range(1, len(run_starts)):
                start = int(run_starts[run_number])
                end = int(run_ends[run_number])
                duration = float(times[block[end - 1]] - times[block[start]] + dt_s)
                dwell_s.append(duration)
                old = int(stable[start - 1])
                new = int(stable[start])
                required = SETTLING_S[(old, new)]
                has_observed_next_switch = run_number < len(run_starts) - 1
                if not has_observed_next_switch and duration + 1e-9 < required:
                    action_right_censored += 1
                    continue
                action_total += 1
                if duration + 1e-9 >= required:
                    action_complete += 1
    shares = counts / counts.sum() if counts.sum() else np.full(3, np.nan)
    return {
        "valid_observations": valid_observations,
        "sequence_count": sequence_count,
        "c_share": float(shares[0]),
        "r_share": float(shares[1]),
        "f_share": float(shares[2]),
        "switches": switches,
        "mean_switches_per_sequence": float(switches / sequence_count) if sequence_count else None,
        "switches_per_100_group_h": float(switches / group_time_s * 360000.0) if group_time_s else None,
        "median_post_switch_dwell_s": float(np.median(dwell_s)) if dwell_s else None,
        "aks_actionability_evaluable_transitions": action_total,
        "aks_actionability_right_censored_transitions": action_right_censored,
        "aks_completion_before_next_switch_fraction": (
            float(action_complete / action_total) if action_total else None
        ),
    }


def evaluate_curve(
    trace: dict[str, np.ndarray], thresholds: np.ndarray, *, group_size: int,
    window_s: float, dt_s: float, policy_update_s: float,
    c_tolerance: float, r_tolerance: float, k: int,
) -> list[dict]:
    rows = []
    for theta in thresholds:
        exposure, valid, times = exposure_for_threshold(trace, float(theta), group_size, window_s, dt_s)
        update_indices, policy_times = policy_tick_samples(times, policy_update_s)
        policy = raw_policy(exposure[:, update_indices], c_tolerance, r_tolerance)
        result = metrics(policy, valid[:, update_indices], policy_times, policy_update_s, k)
        rows.append({"threshold_db": float(theta), **result})
    return rows


def adaptive_exposure_for_threshold(
    trace: dict[str, np.ndarray], threshold_db: float, group_size: int, window_s: float, dt_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Match the current geographic runner: focal groups and available history."""
    active = np.asarray(trace["active"], bool)
    bad = active & (np.asarray(trace["sinr_db"], float) < threshold_db)
    support_bad = np.full(active.shape, np.nan, dtype=float)
    half = group_size // 2
    for time_index in range(active.shape[1]):
        indices = np.flatnonzero(active[:, time_index])
        if not len(indices):
            continue
        positions = np.arange(len(indices))
        lo = np.maximum(0, positions - half)
        hi = np.minimum(len(indices), positions + half + 1)
        cumulative = np.r_[0, np.cumsum(bad[indices, time_index])]
        support_bad[indices, time_index] = (cumulative[hi] - cumulative[lo]) / (hi - lo)
    finite = np.isfinite(support_bad)
    values = np.where(finite, support_bad, 0.0)
    cumulative = np.concatenate([np.zeros((values.shape[0], 1)), np.cumsum(values, axis=1)], axis=1)
    counts = np.concatenate([np.zeros((values.shape[0], 1)), np.cumsum(finite, axis=1)], axis=1)
    window_n = int(round(window_s / dt_s)) + 1
    end = np.arange(values.shape[1]) + 1
    start = np.maximum(0, end - window_n)
    total = cumulative[:, end] - cumulative[:, start]
    count = counts[:, end] - counts[:, start]
    exposure = np.divide(total, count, out=np.zeros_like(total), where=count > 0)
    return exposure, active, np.asarray(trace["t"], float)


def adaptive_policy_cache(
    trace: dict[str, np.ndarray], thresholds: np.ndarray, *, group_size: int,
    window_s: float, dt_s: float, policy_update_s: float,
    c_tolerance: float, r_tolerance: float,
) -> list[tuple[float, np.ndarray, np.ndarray, np.ndarray]]:
    cached = []
    for theta in thresholds:
        exposure, valid, times = adaptive_exposure_for_threshold(
            trace, float(theta), group_size, window_s, dt_s
        )
        update_indices, policy_times = policy_tick_samples(times, policy_update_s)
        cached.append((float(theta), raw_policy(exposure[:, update_indices], c_tolerance, r_tolerance),
                       valid[:, update_indices], policy_times))
    return cached


def evaluate_adaptive_cache(
    cached: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]], *,
    policy_update_s: float, k: int,
) -> list[dict]:
    return [
        {"threshold_db": theta, **metrics(policy, valid, times, policy_update_s, k)}
        for theta, policy, valid, times in cached
    ]


def capability_boundary(rows: list[dict], key: str, thresholds: np.ndarray) -> tuple[float | None, str]:
    accepted = [row["threshold_db"] for row in rows if row[key] >= 0.95 - 1e-12]
    if not accepted:
        return None, "below_scan"
    boundary = max(accepted)
    return boundary, "at_upper_edge" if np.isclose(boundary, thresholds[-1]) else "within_scan"


def summarize_curve(rows: list[dict], thresholds: np.ndarray) -> dict:
    tc, tc_status = capability_boundary(rows, "c_share", thresholds)
    for row in rows:
        row["cr_share"] = row["c_share"] + row["r_share"]
    tr, tr_status = capability_boundary(rows, "cr_share", thresholds)
    nominal = next(row for row in rows if np.isclose(row["threshold_db"], -1.5))
    return {
        "t_c_db": tc, "t_c_status": tc_status, "t_r_db": tr, "t_r_status": tr_status,
        **{f"nominal_{key}": value for key, value in nominal.items() if key != "threshold_db"},
    }


def main(output: Path) -> None:
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    output.mkdir(parents=True)
    scenario_path = ROOT / "scenarios/airport_to_airport/scenario.json"
    scenario = load_scenario(scenario_path)
    thresholds = np.round(np.arange(-4.0, 1.0001, 0.1), 1)
    policy_update_s = 5.0
    traces = {dt: fixed_trace(scenario, dt) for dt in RADIO_STEPS_S}
    warmup_s = scenario.corridor.length_m / scenario.speed_mps
    for trace in traces.values():
        trace["warmup_s"] = warmup_s

    deterministic_rows: list[dict] = []
    deterministic_summary: list[dict] = []
    adaptive_rows: list[dict] = []
    adaptive_summary: list[dict] = []
    for dt_s in RADIO_STEPS_S:
        for group_size in (3, 5, 7):
            for window_s in (15.0, 30.0, 60.0, 120.0):
                adaptive_cached = adaptive_policy_cache(
                    traces[dt_s], thresholds, group_size=group_size, window_s=window_s,
                    dt_s=dt_s, policy_update_s=policy_update_s,
                    c_tolerance=0.05, r_tolerance=0.10,
                )
                for k in (1, 2, 3):
                    curve = evaluate_curve(
                        traces[dt_s], thresholds, group_size=group_size, window_s=window_s,
                        dt_s=dt_s, policy_update_s=policy_update_s,
                        c_tolerance=0.05, r_tolerance=0.10, k=k,
                    )
                    prefix = {"dt_radio_s": dt_s, "group_size": group_size,
                              "window_s": window_s, "persistence_k": k,
                              "policy_update_s": policy_update_s,
                              "minimum_confirmation_delay_s": (k - 1) * policy_update_s}
                    deterministic_rows.extend([{**prefix, **row} for row in curve])
                    deterministic_summary.append({**prefix, **summarize_curve(curve, thresholds)})
                    adaptive_curve = evaluate_adaptive_cache(
                        adaptive_cached, policy_update_s=policy_update_s, k=k
                    )
                    adaptive_rows.extend([{**prefix, **row} for row in adaptive_curve])
                    adaptive_summary.append({**prefix, **summarize_curve(adaptive_curve, thresholds)})

    integer_rows: list[dict] = []
    nominal_n = 5 * (int(round(30.0 / 5.0)) + 1)
    for b_c in range(0, 6):
        for b_r in range(b_c + 1, 6):
            for k in (1, 2, 3):
                curve = evaluate_curve(
                    traces[5.0], thresholds, group_size=5, window_s=30.0, dt_s=5.0,
                    policy_update_s=policy_update_s, c_tolerance=b_c / nominal_n,
                    r_tolerance=b_r / nominal_n, k=k,
                )
                integer_rows.append({
                    "member_time_observations": nominal_n,
                    "allowed_bad_c": b_c, "allowed_bad_r": b_r,
                    "exposure_c": b_c / nominal_n, "exposure_r": b_r / nominal_n,
                    "persistence_k": k, "policy_update_s": policy_update_s,
                    "minimum_confirmation_delay_s": (k - 1) * policy_update_s,
                    **summarize_curve(curve, thresholds),
                })

    noise_rows: list[dict] = []
    for dt_s in RADIO_STEPS_S:
        base = traces[dt_s]
        for sigma_db in (0.0, 0.5, 1.0):
            seeds = (0,) if sigma_db == 0 else tuple(range(20))
            for seed in seeds:
                rng = np.random.default_rng(seed)
                noisy = {**base, "sinr_db": base["sinr_db"] + rng.normal(0.0, sigma_db, base["sinr_db"].shape)}
                for k in (1, 2, 3):
                    curve = evaluate_curve(
                        noisy, thresholds, group_size=5, window_s=30.0, dt_s=dt_s,
                        policy_update_s=policy_update_s,
                        c_tolerance=0.05, r_tolerance=0.10, k=k,
                    )
                    noise_rows.append({
                        "dt_radio_s": dt_s, "sigma_db": sigma_db, "seed": seed,
                        "persistence_k": k, "policy_update_s": policy_update_s,
                        "minimum_confirmation_delay_s": (k - 1) * policy_update_s,
                        **summarize_curve(curve, thresholds),
                    })

    write_csv(output / "deterministic_curves.csv", deterministic_rows)
    write_csv(output / "deterministic_summary.csv", deterministic_summary)
    write_csv(output / "adaptive_focal_curves.csv", adaptive_rows)
    write_csv(output / "adaptive_focal_summary.csv", adaptive_summary)
    write_csv(output / "integer_budget_summary.csv", integer_rows)
    write_csv(output / "iid_noise_seed_summary.csv", noise_rows)
    config = {
        "study": "bay_area_policy_calibration_screen",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario": "scenarios/airport_to_airport/scenario.json",
        "trajectory": {"offset_m": 0.0, "altitude_m": 300.0, "speed_mps": 50.0,
                       "demand_uam_h": 112.5, "motion_feedback": False, "lane_change": False},
        "threshold_db": thresholds.tolist(), "dt_radio_s": list(RADIO_STEPS_S),
        "policy_update_s": policy_update_s,
        "window_s": [15.0, 30.0, 60.0, 120.0], "group_size": [3, 5, 7],
        "exposure_nominal": {"C": 0.05, "R": 0.10}, "persistence_k": [1, 2, 3],
        "integer_budget_screen": {"fixed_n": nominal_n, "bad_c": [0, 1, 2, 3, 4, 5],
                                  "bad_r_rule": "b_C < b_R <= 5"},
        "noise_stress": {"model": "iid zero-mean Gaussian added to modeled SINR",
                         "sigma_db": [0.0, 0.5, 1.0], "replicates_nonzero": 20,
                         "calibration_status": "illustrative_not_receiver_calibrated"},
        "policy_semantics": {
            "complete_group": "complete consecutive groups and complete trailing windows; route-transit warmup",
            "adaptive_focal": "current geographic runner focal groups (smaller at stream edges) and available history",
        },
        "aks_settling_s_from_R0036": {f"{POLICIES[a]}->{POLICIES[b]}": value
                                      for (a, b), value in SETTLING_S.items()},
        "sources": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in [scenario_path, Path(__file__).resolve(), ROOT / "src/capacity_policy/policy.py",
                         ROOT / "src/uam_simulator/geographic_traffic.py"]
        },
    }
    write_json(output / "resolved_config.json", config)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    main(args.output.resolve())
