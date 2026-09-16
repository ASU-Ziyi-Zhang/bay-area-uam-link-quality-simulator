"""Research adapters around the accepted radio -> policy -> capacity kernels.

Fixed independent streams are a planning assumption, not a conflict controller.
The legacy runner and dashboard bundles deliberately remain unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from capacity_policy import (
    ConstantSpeedTrajectory, assign_policy, compute_link_state,
    evaluate_link_quality, snapshot_capacity,
)
from capacity_policy.scenario import Scenario
from .group_runner import _policy_records, reliability_floor
from .runner import make_time_grid


@dataclass(frozen=True)
class FixedStream:
    lane_id: str
    level_id: str
    offset_m: float
    altitude_m: float
    demand_uam_h: float

    def __post_init__(self):
        if not self.lane_id or not self.level_id:
            raise ValueError("stream IDs must be nonempty")
        if not np.all(np.isfinite([self.offset_m, self.altitude_m, self.demand_uam_h])):
            raise ValueError("stream parameters must be finite")
        if self.altitude_m <= 0 or self.demand_uam_h <= 0:
            raise ValueError("altitude and demand must be positive")


@dataclass
class StreamEvaluation:
    time_s: np.ndarray
    capacity_uam_h: np.ndarray  # NaN means unestimated, never zero capacity.
    counts: np.ndarray         # time x C/R/F
    active_count: np.ndarray
    summary: dict


def evaluate_stream(
    scenario: Scenario,
    stream: FixedStream,
    *,
    dt_s: float = 1.0,
    policy_definition: str = "all_active",
    duration_s: float | None = None,
    warmup_s: float | None = None,
    rho: float = 0.95,
    route=None,
) -> StreamEvaluation:
    """Evaluate one fixed lane/level with either explicit policy convention.

``full_group`` uses the paper's complete centered groups and post-warmup
estimator. ``all_active`` exactly reuses the dashboard's shrinking-neighborhood
and available-history estimator. Never silently substitute one for the other.
"""
    if policy_definition not in ("all_active", "full_group"):
        raise ValueError("unknown policy_definition")
    if not np.isfinite(dt_s) or dt_s <= 0 or not 0 < rho <= 1:
        raise ValueError("invalid sampling interval or reliability")
    if policy_definition == "full_group" and not np.isclose(
        scenario.policy.window_s / dt_s, round(scenario.policy.window_s / dt_s), rtol=0, atol=1e-9
    ):
        raise ValueError("full-group window must be an integer multiple of dt_s")
    duration = scenario.simulation_duration_s if duration_s is None else duration_s
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    path = scenario.corridor if route is None else route
    times = make_time_grid(duration, dt_s)
    entries = np.arange(0, duration, 3600 / stream.demand_uam_h)
    trajectory = ConstantSpeedTrajectory(scenario.speed_mps).realize(
        path, times, entries, np.full(entries.shape, stream.altitude_m),
        np.full(entries.shape, stream.offset_m),
    )
    radio = compute_link_state(trajectory, scenario.base_stations, scenario.radio)
    links = evaluate_link_quality(radio, scenario.link_quality)
    ids = [f"{stream.lane_id}:{stream.level_id}:UAM{i + 1:03d}" for i in range(len(entries))]
    warmup = path.length_m / scenario.speed_mps if warmup_s is None else warmup_s
    if not np.isfinite(warmup) or warmup < 0:
        raise ValueError("warmup must be finite and nonnegative")
    if policy_definition == "all_active":
        codes, valid, _ = _policy_records(
            links, ids, scenario.policy.group_size, scenario.policy.window_s,
            scenario.policy.coordinated_exposure_tolerance,
            scenario.policy.reactive_exposure_tolerance,
        )
        obs_times = times
        if warmup_s is not None:
            valid = valid & (times[None, :] >= warmup)
    elif len(entries) < scenario.policy.group_size or len(times) < round(scenario.policy.window_s / dt_s) + 1:
        codes = np.empty((0, len(times)), dtype=int)
        valid = np.empty((0, len(times)), dtype=bool)
        obs_times = times
    else:
        policy = assign_policy(links, replace(scenario.policy, time_step_s=dt_s, warmup_s=warmup))
        codes, valid = policy["policy"], policy["valid_after_warmup"]
        obs_times = policy["t_obs"]
    q = np.full(len(times), np.nan)
    counts = np.zeros((len(times), 3), dtype=int)
    if valid.any():
        capacity = snapshot_capacity(codes, valid, obs_times, scenario.capacity)
        indices = np.searchsorted(times, capacity["t"])
        q[indices] = capacity["q_mix_UAM_h"]
        counts[indices] = np.column_stack([capacity[f"n_{p}"] for p in "CRF"])
    totals = counts.sum(axis=0)
    n = int(totals.sum())
    finite = np.isfinite(q)
    active = trajectory.active
    summary = {
        "lane_id": stream.lane_id, "level_id": stream.level_id,
        "altitude_m": stream.altitude_m, "offset_m": stream.offset_m,
        "demand_uam_h": stream.demand_uam_h, "entry_interval_s": 3600 / stream.demand_uam_h,
        "entrant_count": len(entries), "route_length_m": path.length_m,
        "position_convention": (
            "legacy reference-arclength, segment-normal lateral offset; not flight-dynamics validated"
            if route is None else "smooth path with numerically inverted physical arc length"
        ),
        "transit_time_s": path.length_m / scenario.speed_mps,
        "policy_definition": policy_definition, "dt_s": dt_s,
        "warmup_s": warmup if policy_definition == "full_group" or warmup_s is not None else 0.0,
        "status": "estimated" if n else "no_valid_groups",
        "observation_count": n, "counts": dict(zip("CRF", map(int, totals))),
        "shares": {p: int(v) / n if n else None for p, v in zip("CRF", totals)},
        "capacity_snapshot_count": int(finite.sum()),
        "reliability_rho": rho,
        "q_rho_uam_h": reliability_floor(q[finite], rho) if finite.any() else None,
        "active_aircraft_time_observations": int(active.sum()),
        "poor_link_fraction": float((active & ~links["link_ok"]).sum() / active.sum()),
        "mean_rsrp_dbm": float(radio["serving_rsrp_dbm"][active].mean()),
        "mean_sinr_db": float(radio["sinr_db"][active].mean()),
    }
    return StreamEvaluation(times, q, counts, active.sum(axis=0), summary)


def aggregate_streams(streams: list[StreamEvaluation], rho: float = 0.95) -> dict:
    """Sum simultaneous stream capacities BEFORE taking the reliability floor.

All streams must be estimated at a snapshot. Fixed-allocation support is a
joint per-stream test, stricter than pooled capacity (no traffic reallocation).
"""
    if not streams or not 0 < rho <= 1:
        raise ValueError("streams and valid reliability required")
    times = streams[0].time_s
    if any(not np.array_equal(s.time_s, times) for s in streams):
        raise ValueError("stream time grids must align")
    matrix = np.asarray([s.capacity_uam_h for s in streams])
    common = np.isfinite(matrix).all(axis=0)
    demand = np.asarray([s.summary["demand_uam_h"] for s in streams])
    counts = sum(s.counts for s in streams)
    n = int(counts.sum())
    totals = np.full(times.shape, np.nan)
    totals[common] = matrix[:, common].sum(axis=0)
    joint = (matrix[:, common] + 1e-12 >= demand[:, None]).all(axis=0)
    pooled = totals[common] + 1e-12 >= demand.sum()
    return {
        "summary": {
            "status": "estimated" if common.any() else "no_common_valid_snapshots",
            "stream_count": len(streams), "offered_demand_uam_h": float(demand.sum()),
            "common_snapshot_count": int(common.sum()), "observation_count": n,
            "common_fraction_of_full_run_samples": float(common.mean()),
            "support_is_conditional_on_common_valid_samples": True,
            "policy_shares": {p: int(v) / n if n else None for p, v in zip("CRF", counts.sum(axis=0))},
            "reliability_rho": rho,
            "q_total_rho_uam_h": reliability_floor(totals[common], rho) if common.any() else None,
            "pooled_support_fraction": float(pooled.mean()) if common.any() else None,
            "fixed_allocation_support_fraction": float(joint.mean()) if common.any() else None,
            "fixed_allocation_supported": bool(joint.mean() + 1e-12 >= rho) if common.any() else None,
            "aggregation": "simultaneous independent-stream sum, then reliability floor; common valid times only",
        },
        "time_s": times, "capacity_uam_h": totals, "common_valid": common,
    }
