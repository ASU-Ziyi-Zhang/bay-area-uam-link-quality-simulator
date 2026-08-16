"""Link-quality to operating-policy translation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


POLICY_F, POLICY_R, POLICY_C = 0, 1, 2


@dataclass(frozen=True)
class PolicyConfig:
    reactive_exposure_tolerance: float = 0.10
    coordinated_exposure_tolerance: float = 0.05
    group_size: int = 5
    window_s: float = 30.0
    time_step_s: float = 5.0
    warmup_s: float = 1600.0


def _window_sum_rows(values: np.ndarray, width: int) -> np.ndarray:
    cumulative = np.vstack(
        [np.zeros((1, values.shape[1])), np.cumsum(values, axis=0)]
    )
    return cumulative[width:] - cumulative[:-width]


def _rolling_time(values: np.ndarray, width: int) -> np.ndarray:
    cumulative = np.hstack(
        [np.zeros((values.shape[0], 1)), np.cumsum(values, axis=1)]
    )
    return cumulative[:, width:] - cumulative[:, :-width]


def assign_policy(
    link_quality: dict[str, np.ndarray],
    config: PolicyConfig,
) -> dict[str, np.ndarray]:
    """Assign C/R/F from already evaluated per-UAM link quality."""
    if not (
        0.0
        <= config.coordinated_exposure_tolerance
        <= config.reactive_exposure_tolerance
        <= 1.0
    ):
        raise ValueError("expected 0 <= coordinated tolerance <= reactive <= 1")
    active = np.asarray(link_quality["active"], dtype=bool)
    if active.shape[0] < config.group_size:
        raise ValueError("fewer UAMs than group size")
    link_ok = np.asarray(link_quality["link_ok"], dtype=bool)
    if link_ok.shape != active.shape:
        raise ValueError("link-quality and active arrays must have equal shape")
    group_count = _window_sum_rows(active.astype(float), config.group_size)
    group_full = group_count >= config.group_size - 0.5
    group_support = (
        _window_sum_rows(link_ok.astype(float), config.group_size)
        / config.group_size
    )
    window_n = int(round(config.window_s / config.time_step_s)) + 1
    exposure = 1.0 - _rolling_time(group_support, window_n) / window_n
    valid = _rolling_time(group_full.astype(float), window_n) >= window_n - 0.5
    support_c = exposure <= config.coordinated_exposure_tolerance + 1e-12
    support_r = exposure <= config.reactive_exposure_tolerance + 1e-12
    policy = np.where(
        support_c,
        POLICY_C,
        np.where(support_r, POLICY_R, POLICY_F),
    )
    t_obs = np.asarray(link_quality["t"])[window_n - 1 :]
    valid_after_warmup = valid & (t_obs[None, :] >= config.warmup_s)
    return {
        "policy": policy,
        "valid": valid,
        "valid_after_warmup": valid_after_warmup,
        "t_obs": t_obs,
        "exposure": exposure,
        "support_c": support_c,
        "support_r": support_r,
    }
