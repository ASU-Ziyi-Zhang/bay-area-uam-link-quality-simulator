"""Corridor geometry and route-relative coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from pyproj import Transformer


@dataclass(frozen=True)
class Corridor:
    """A metric centerline parameterized by cumulative arc length."""

    xy_m: np.ndarray
    crs: str
    source: str = ""

    def __post_init__(self) -> None:
        xy = np.asarray(self.xy_m, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] < 2:
            raise ValueError("corridor must contain at least two XY vertices")
        if not np.all(np.isfinite(xy)):
            raise ValueError("corridor coordinates must be finite")
        segment_lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        if np.any(segment_lengths <= 0.0):
            raise ValueError("corridor contains a zero-length segment")
        object.__setattr__(self, "xy_m", xy)

    @property
    def cumulative_m(self) -> np.ndarray:
        lengths = np.linalg.norm(np.diff(self.xy_m, axis=0), axis=1)
        return np.concatenate(([0.0], np.cumsum(lengths)))

    @property
    def length_m(self) -> float:
        return float(self.cumulative_m[-1])

    @classmethod
    def straight(cls, length_m: float) -> "Corridor":
        if length_m <= 0.0:
            raise ValueError("length_m must be positive")
        return cls(np.asarray([[0.0, 0.0], [length_m, 0.0]]), "LOCAL_METRIC")

    @classmethod
    def from_geojson(
        cls,
        path: str | Path,
        source_crs: str = "EPSG:4326",
        target_crs: str = "EPSG:26910",
    ) -> "Corridor":
        path = Path(path)
        payload = json.loads(path.read_text(encoding="utf-8"))
        feature = payload["features"][0]
        if feature["geometry"]["type"] != "LineString":
            raise ValueError("corridor GeoJSON must contain a LineString")
        lon_lat = np.asarray(feature["geometry"]["coordinates"], dtype=float)
        transformer = Transformer.from_crs(source_crs, target_crs, always_xy=True)
        x, y = transformer.transform(lon_lat[:, 0], lon_lat[:, 1])
        return cls(np.column_stack([x, y]), target_crs, str(path))

    def interpolate(
        self,
        s_m: np.ndarray,
        lateral_m: np.ndarray | float = 0.0,
    ) -> np.ndarray:
        """Return XY positions using positive-left local lateral offsets."""
        s = np.asarray(s_m, dtype=float)
        lateral = np.broadcast_to(np.asarray(lateral_m, dtype=float), s.shape)
        clipped = np.clip(s, 0.0, self.length_m)
        cumulative = self.cumulative_m
        index = np.searchsorted(cumulative, clipped, side="right") - 1
        index = np.clip(index, 0, len(cumulative) - 2)
        segment = self.xy_m[index + 1] - self.xy_m[index]
        segment_length = cumulative[index + 1] - cumulative[index]
        fraction = (clipped - cumulative[index]) / segment_length
        center = self.xy_m[index] + fraction[..., None] * segment
        tangent = segment / segment_length[..., None]
        normal = np.stack([-tangent[..., 1], tangent[..., 0]], axis=-1)
        return center + lateral[..., None] * normal
