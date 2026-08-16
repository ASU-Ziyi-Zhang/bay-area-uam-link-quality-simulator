"""Stable-ID base-station inventory and scenario selection."""

from __future__ import annotations

from dataclasses import dataclass, field
import csv
from pathlib import Path
from typing import Iterable

import numpy as np
from pyproj import Transformer


@dataclass(frozen=True)
class BaseStation:
    site_id: str
    x_m: float
    y_m: float
    height_m: float | None = None
    attributes: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class BaseStationSet:
    stations: tuple[BaseStation, ...]
    crs: str

    def __post_init__(self) -> None:
        if not self.stations:
            raise ValueError("base-station set must not be empty")
        ids = [station.site_id for station in self.stations]
        if len(ids) != len(set(ids)):
            raise ValueError("base-station site_id values must be unique")

    @property
    def site_ids(self) -> tuple[str, ...]:
        return tuple(station.site_id for station in self.stations)

    def select(self, site_ids: Iterable[str]) -> "BaseStationSet":
        requested = tuple(site_ids)
        lookup = {station.site_id: station for station in self.stations}
        missing = sorted(set(requested) - set(lookup))
        if missing:
            raise KeyError(f"unknown site IDs: {missing}")
        return BaseStationSet(tuple(lookup[site_id] for site_id in requested), self.crs)

    def without(self, site_ids: Iterable[str]) -> "BaseStationSet":
        excluded = set(site_ids)
        return BaseStationSet(
            tuple(station for station in self.stations if station.site_id not in excluded),
            self.crs,
        )

    def positions(self, assumed_height_m: float | None = None) -> np.ndarray:
        heights = []
        missing = []
        for station in self.stations:
            if station.height_m is None and assumed_height_m is None:
                missing.append(station.site_id)
            heights.append(
                station.height_m if station.height_m is not None else assumed_height_m
            )
        if missing:
            raise ValueError(
                "antenna height is unresolved for: " + ", ".join(missing)
            )
        return np.asarray(
            [
                [station.x_m, station.y_m, float(height)]
                for station, height in zip(self.stations, heights)
            ],
            dtype=float,
        )

    @classmethod
    def symmetric_staggered(
        cls,
        count: int,
        corridor_length_m: float,
        lateral_offset_m: float,
        height_m: float = 0.0,
    ) -> "BaseStationSet":
        if count <= 0 or count % 2:
            raise ValueError("legacy staggered layout requires a positive even count")
        n_side = count // 2
        dx = corridor_length_m / n_side
        x = np.concatenate(
            [(np.arange(n_side) + 0.25) * dx, (np.arange(n_side) + 0.75) * dx]
        )
        y = np.concatenate(
            [
                np.full(n_side, lateral_offset_m),
                np.full(n_side, -lateral_offset_m),
            ]
        )
        order = np.argsort(x)
        stations = tuple(
            BaseStation(f"BS{rank + 1:02d}", float(x[i]), float(y[i]), height_m)
            for rank, i in enumerate(order)
        )
        return cls(stations, "LOCAL_METRIC")

    @classmethod
    def from_wgs84_csv(
        cls,
        path: str | Path,
        target_crs: str = "EPSG:26910",
        id_column: str = "public_site_id",
        longitude_column: str = "longitude",
        latitude_column: str = "latitude",
        height_column: str | None = None,
    ) -> "BaseStationSet":
        path = Path(path)
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        transformer = Transformer.from_crs("EPSG:4326", target_crs, always_xy=True)
        stations = []
        for row in rows:
            x, y = transformer.transform(
                float(row[longitude_column]), float(row[latitude_column])
            )
            raw_height = row.get(height_column, "") if height_column else ""
            height = float(raw_height) if raw_height not in (None, "") else None
            stations.append(
                BaseStation(
                    site_id=row[id_column],
                    x_m=float(x),
                    y_m=float(y),
                    height_m=height,
                    attributes=dict(row),
                )
            )
        return cls(tuple(stations), target_crs)
