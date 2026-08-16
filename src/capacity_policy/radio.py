"""Per-UAM received-power, association, interference, and SINR kernel."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .base_stations import BaseStationSet
from .trajectory import TrajectoryState


@dataclass(frozen=True)
class RadioConfig:
    frequency_ghz: float = 5.0
    eirp_dbm: float = 46.0
    receiver_gain_db: float = 0.0
    noise_dbm: float = -99.0
    resource_elements: int = 300
    assumed_bs_height_m: float | None = None
    #: Number of nearest sites the UAM can associate with and receive from. The
    #: strongest of that set serves; the remainder are the interference set.
    #: ``None`` keeps every site co-channel, which is the worst-case bound.
    served_set_size: int | None = None

    def __post_init__(self) -> None:
        if self.served_set_size is not None and self.served_set_size < 1:
            raise ValueError("served_set_size must be at least 1")


def served_set_mask(d2: np.ndarray, served_set_size: int | None) -> np.ndarray:
    """Mark the nearest ``served_set_size`` sites along the station axis.

    Selection is by 3D distance. Because every site radiates a common EIRP and
    the path loss is monotonic in distance, the nearest set is also the
    strongest set, so restricting it never changes which site serves.
    """
    mask = np.ones(d2.shape, dtype=bool)
    n_sites = d2.shape[-1]
    if served_set_size is None or served_set_size >= n_sites:
        return mask
    nearest = np.argpartition(d2, served_set_size - 1, axis=-1)[..., :served_set_size]
    mask = np.zeros(d2.shape, dtype=bool)
    np.put_along_axis(mask, nearest, True, axis=-1)
    return mask


def compute_link_state(
    trajectory: TrajectoryState,
    base_stations: BaseStationSet,
    config: RadioConfig,
) -> dict[str, np.ndarray]:
    """Compute the full pairwise radio state without geometry assumptions."""
    stations = base_stations.positions(config.assumed_bs_height_m)
    position = np.asarray(trajectory.position_m, dtype=float)
    delta = position[:, :, None, :] - stations[None, None, :, :]
    d2 = np.sum(delta * delta, axis=3)
    d2 = np.maximum(d2, np.finfo(float).tiny)
    received_dbm = (
        config.eirp_dbm
        + config.receiver_gain_db
        - 28.0
        - 11.0 * np.log10(d2)
        - 20.0 * np.log10(config.frequency_ghz)
    )
    # received_power_dbm stays a full per-site measurement report; only the
    # association and the interference sum are limited to the served set.
    served = served_set_mask(d2, config.served_set_size)
    serving = np.argmax(np.where(served, received_dbm, -np.inf), axis=2)
    row = np.arange(received_dbm.shape[0])[:, None]
    col = np.arange(received_dbm.shape[1])[None, :]
    desired_dbm = received_dbm[row, col, serving]
    received_mw = np.power(10.0, received_dbm / 10.0)
    desired_mw = received_mw[row, col, serving]
    interference_mw = np.maximum(
        np.sum(np.where(served, received_mw, 0.0), axis=2) - desired_mw,
        np.finfo(float).tiny,
    )
    noise_mw = 10.0 ** (config.noise_dbm / 10.0)
    sinr_linear = desired_mw / (interference_mw + noise_mw)
    per_re_offset_db = 10.0 * np.log10(config.resource_elements)
    return {
        "t": trajectory.time_s,
        "active": trajectory.active,
        "serving_bs": serving,
        "received_power_dbm": received_dbm,
        "served_mask": served,
        "serving_rx_dbm": desired_dbm,
        "desired_dbm": desired_dbm,
        "serving_rsrp_dbm": desired_dbm - per_re_offset_db,
        "interference_dbm": 10.0 * np.log10(interference_mw),
        "sinr_db": 10.0 * np.log10(sinr_linear),
        "noise_fraction": noise_mw / (interference_mw + noise_mw),
    }
