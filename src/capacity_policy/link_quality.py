"""Per-UAM radio metrics to explicit link-quality state."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LinkQualityConfig:
    sinr_threshold_db: float = -1.5


def evaluate_link_quality(
    radio_state: dict[str, np.ndarray],
    config: LinkQualityConfig,
) -> dict[str, np.ndarray]:
    """Evaluate the accepted common-SINR per-UAM link requirement."""
    active = np.asarray(radio_state["active"], dtype=bool)
    sinr_db = np.asarray(radio_state["sinr_db"], dtype=float)
    if active.shape != sinr_db.shape:
        raise ValueError("active mask and SINR array must have equal shape")
    return {
        "t": np.asarray(radio_state["t"], dtype=float),
        "active": active,
        "link_ok": active & (sinr_db >= config.sinr_threshold_db),
        "sinr_db": sinr_db,
        "sinr_threshold_db": float(config.sinr_threshold_db),
    }
