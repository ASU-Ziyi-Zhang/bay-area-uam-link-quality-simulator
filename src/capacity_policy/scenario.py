"""Scenario composition without model-specific hard coding."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from .base_stations import BaseStationSet
from .capacity import CapacityConfig
from .geometry import Corridor
from .link_quality import LinkQualityConfig
from .policy import PolicyConfig
from .radio import RadioConfig


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    corridor: Corridor
    base_stations: BaseStationSet
    speed_mps: float
    simulation_duration_s: float
    time_step_s: float
    radio: RadioConfig
    link_quality: LinkQualityConfig
    policy: PolicyConfig
    capacity: CapacityConfig
    source_path: Path

    @property
    def transit_time_s(self) -> float:
        return self.corridor.length_m / self.speed_mps

    @property
    def radio_ready(self) -> bool:
        try:
            self.base_stations.positions(self.radio.assumed_bs_height_m)
        except ValueError:
            return False
        return True


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path).resolve()
    config = json.loads(path.read_text(encoding="utf-8"))
    root = path.parent

    corridor_config = config["corridor"]
    if corridor_config["kind"] == "straight":
        corridor = Corridor.straight(float(corridor_config["length_m"]))
    elif corridor_config["kind"] == "geojson":
        corridor = Corridor.from_geojson(
            root / corridor_config["path"],
            corridor_config.get("source_crs", "EPSG:4326"),
            corridor_config.get("target_crs", "EPSG:26910"),
        )
    else:
        raise ValueError(f"unknown corridor kind: {corridor_config['kind']}")

    station_config = config["base_stations"]
    if station_config["kind"] == "symmetric_staggered":
        stations = BaseStationSet.symmetric_staggered(
            int(station_config["count"]),
            corridor.length_m,
            float(station_config["lateral_offset_m"]),
            float(station_config.get("height_m", 0.0)),
        )
    elif station_config["kind"] == "wgs84_csv":
        stations = BaseStationSet.from_wgs84_csv(
            root / station_config["path"],
            target_crs=corridor.crs,
            height_column=station_config.get("height_column"),
        )
    else:
        raise ValueError(f"unknown base-station kind: {station_config['kind']}")
    active_ids = station_config.get("active_site_ids")
    if active_ids:
        stations = stations.select(active_ids)

    speed = float(config["trajectory"]["speed_mps"])
    dt = float(config["simulation"]["time_step_s"])
    duration = float(config["simulation"]["duration_s"])
    radio_config = RadioConfig(**config["radio"])
    link_quality_config = LinkQualityConfig(**config["link_quality"])
    policy_values = dict(config["policy"])
    policy_values.setdefault("time_step_s", dt)
    policy_values.setdefault("warmup_s", corridor.length_m / speed)
    capacity_values = dict(config["capacity"])
    capacity_values.setdefault("speed_mps", speed)
    return Scenario(
        scenario_id=config["scenario_id"],
        corridor=corridor,
        base_stations=stations,
        speed_mps=speed,
        simulation_duration_s=duration,
        time_step_s=dt,
        radio=radio_config,
        link_quality=link_quality_config,
        policy=PolicyConfig(**policy_values),
        capacity=CapacityConfig(**capacity_values),
        source_path=path,
    )
