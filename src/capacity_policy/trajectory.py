"""Replaceable trajectory models."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import Corridor


@dataclass(frozen=True)
class TrajectoryState:
    time_s: np.ndarray
    position_m: np.ndarray
    along_m: np.ndarray
    active: np.ndarray


@dataclass(frozen=True)
class ConstantSpeedTrajectory:
    speed_mps: float

    def realize(
        self,
        corridor: Corridor,
        time_s: np.ndarray,
        entry_time_s: np.ndarray,
        altitude_m: np.ndarray,
        lateral_m: np.ndarray,
    ) -> TrajectoryState:
        if self.speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
        time = np.asarray(time_s, dtype=float)
        entry = np.asarray(entry_time_s, dtype=float)
        altitude = np.asarray(altitude_m, dtype=float)
        lateral = np.asarray(lateral_m, dtype=float)
        if not (entry.shape == altitude.shape == lateral.shape):
            raise ValueError("entry, altitude, and lateral arrays must have equal shape")
        along = self.speed_mps * (time[None, :] - entry[:, None])
        active = (along >= 0.0) & (along <= corridor.length_m)
        # Preserve the accepted TRB convention for inactive records: their
        # distance-evaluation point is the route origin, while the active mask
        # excludes them from policy/capacity observations.
        along_for_position = np.where(active, along, 0.0)
        xy = corridor.interpolate(along_for_position, lateral[:, None])
        z = np.broadcast_to(altitude[:, None], along.shape)
        return TrajectoryState(time, np.dstack([xy[..., 0], xy[..., 1], z]), along, active)
