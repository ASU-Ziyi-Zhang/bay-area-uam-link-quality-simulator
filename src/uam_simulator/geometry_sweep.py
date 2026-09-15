"""Geometry-response functions for independent, fixed-offset flight streams.

The sampled radio profile depends on route, stations and position, not on how
many other streams are present. Group policy/capacity also depends on the
within-stream entry interval and estimator. This module changes neither kernel.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np

from capacity_policy import compute_link_state
from capacity_policy.trajectory import TrajectoryState
from .baseline_core import FixedStream, StreamEvaluation, evaluate_stream


@dataclass
class LocationEvaluation:
    summary: dict
    profile: dict[str, np.ndarray]
    stream: StreamEvaluation


def radio_profile(scenario, offset_m: float, altitude_m: float, *, profile_step_m=50.0):
    """One uniformly sampled full-route profile, independent of traffic demand.

The proportion below threshold is a sample fraction, not the fleet-time
weighted fraction in evaluate_stream. Endpoint and curvature conventions are
the inherited raw GIS offsets; this is not a terminal-connected route design.
"""
    FixedStream("profile", "profile", offset_m, altitude_m, 1.0)
    if not np.isfinite(profile_step_m) or profile_step_m <= 0:
        raise ValueError("profile_step_m must be finite and positive")
    along = np.linspace(0, scenario.corridor.length_m,
                        int(np.ceil(scenario.corridor.length_m / profile_step_m)) + 1)
    xy = scenario.corridor.interpolate(along, offset_m)
    position = np.column_stack([xy, np.full(len(along), altitude_m)])
    trajectory = TrajectoryState(along / scenario.speed_mps, position[None, :, :],
                                 along[None, :], np.ones((1, len(along)), dtype=bool))
    radio = compute_link_state(trajectory, scenario.base_stations, scenario.radio)
    sinr, rsrp = radio["sinr_db"][0], radio["serving_rsrp_dbm"][0]
    stations = scenario.base_stations.positions(scenario.radio.assumed_bs_height_m)
    distance_2d = np.linalg.norm(xy[:, None, :] - stations[None, :, :2], axis=-1)
    selected = radio["served_mask"][0]
    # Domain cautions are inherited from docs/assumptions.md, not certification.
    summary = {
        "profile_sample_count": len(along),
        "profile_spacing_m": float(along[1] - along[0]),
        "sinr_p05_db": float(np.quantile(sinr, 0.05)),
        "sinr_median_db": float(np.median(sinr)),
        "rsrp_median_dbm": float(np.median(rsrp)),
        "profile_below_threshold_fraction": float(np.mean(sinr < scenario.link_quality.sinr_threshold_db)),
        "sinr_threshold_db": scenario.link_quality.sinr_threshold_db,
        "height_extrapolated": not (22.5 < altitude_m <= 300),
        "profile_any_served_link_beyond_4km_fraction": float(np.mean(np.any(selected & (distance_2d > 4000), axis=1))),
        "profile_statistic": "uniform reference-distance samples over one full route, including endpoints",
    }
    return summary, {"along_m": along, "sinr_db": sinr, "rsrp_dbm": rsrp,
                     "serving_bs": radio["serving_bs"][0]}


def evaluate_location(scenario, offset_m: float, altitude_m: float, *,
                      demand_uam_h=112.5, dt_s=5.0, policy_definition="full_group",
                      rho=0.95, profile_step_m=50.0) -> LocationEvaluation:
    """Evaluate any (offset, height), with explicitly fixed per-stream demand."""
    stream = evaluate_stream(scenario, FixedStream("location", "fixed", offset_m, altitude_m, demand_uam_h),
                             dt_s=dt_s, policy_definition=policy_definition, rho=rho)
    summary, profile = radio_profile(scenario, offset_m, altitude_m, profile_step_m=profile_step_m)
    return LocationEvaluation({"scenario_id": scenario.scenario_id, **stream.summary, **summary}, profile, stream)


def evaluate_locations(scenario, positions, **kwargs) -> list[LocationEvaluation]:
    """Arbitrary coordinate pairs, not necessarily a rectangular lane/level grid."""
    positions = list(positions)
    if not positions:
        raise ValueError("at least one position is required")
    if len({tuple(p) for p in positions}) != len(positions):
        raise ValueError("positions must be distinct")
    return [evaluate_location(scenario, offset, altitude, **kwargs) for offset, altitude in positions]


def geometry_grid(scenario, offsets_m, altitudes_m, **kwargs) -> list[LocationEvaluation]:
    """Convenience Cartesian sweep; coordinates and grid size are unrestricted."""
    return evaluate_locations(scenario, product(offsets_m, altitudes_m), **kwargs)
