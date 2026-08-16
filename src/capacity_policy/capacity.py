"""Operating-policy to spacing and capacity translation."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .policy import POLICY_C, POLICY_F, POLICY_R


@dataclass(frozen=True)
class CapacityConfig:
    speed_mps: float = 50.0
    standstill_distance_m: float = 200.0
    response_time_s: dict[str, float] = field(
        default_factory=lambda: {"C": 15.0, "R": 30.0, "F": 60.0}
    )
    braking_buffer_s2_per_m: float = 0.167

    def spacing_m(self, policy: str) -> float:
        return (
            self.standstill_distance_m
            + self.response_time_s[policy] * self.speed_mps
            + self.braking_buffer_s2_per_m * self.speed_mps**2
        )


def snapshot_capacity(
    policy: np.ndarray,
    valid: np.ndarray,
    t_obs: np.ndarray,
    config: CapacityConfig,
) -> dict[str, np.ndarray]:
    """Compute mixed and bottleneck capacity for every valid snapshot."""
    policy = np.asarray(policy, dtype=int)
    valid = np.asarray(valid, dtype=bool)
    t_obs = np.asarray(t_obs, dtype=float)
    spacing_by_code = np.asarray(
        [config.spacing_m("F"), config.spacing_m("R"), config.spacing_m("C")]
    )
    if policy.shape != valid.shape or policy.shape[1] != t_obs.size:
        raise ValueError("policy, validity, and observation times are inconsistent")
    n_group = valid.sum(axis=0)
    keep = n_group > 0
    spacing = spacing_by_code[policy]
    spacing_sum = np.where(valid, spacing, 0.0).sum(axis=0)
    mean_spacing = spacing_sum[keep] / n_group[keep]
    local_flow = 3600.0 * config.speed_mps / spacing
    bottleneck = np.where(valid, local_flow, np.inf).min(axis=0)[keep]
    return {
        "t": t_obs[keep],
        "n_group": n_group[keep],
        "n_C": ((policy == POLICY_C) & valid).sum(axis=0)[keep],
        "n_R": ((policy == POLICY_R) & valid).sum(axis=0)[keep],
        "n_F": ((policy == POLICY_F) & valid).sum(axis=0)[keep],
        "mean_spacing_m": mean_spacing,
        "q_mix_UAM_h": 3600.0 * config.speed_mps / mean_spacing,
        "q_bottleneck_UAM_h": bottleneck,
    }
