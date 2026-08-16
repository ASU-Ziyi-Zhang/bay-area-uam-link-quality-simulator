"""Configuration loading for the modular simulator."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from .interfaces import ClockConfig, SimulationConfig


@dataclass(frozen=True)
class SimulatorConfig:
    """Validated simulator settings plus the referenced capacity-policy case."""

    source_path: Path
    simulation: SimulationConfig
    speed_mps: float
    altitude_m: float
    lateral_offset_m: float
    level_id: str
    lane_id: str
    uam_id: str
    duration_s: float | None

    @property
    def stop_at_arrival(self) -> bool:
        return self.duration_s is None


def load_simulator_config(path: str | Path) -> SimulatorConfig:
    path = Path(path).resolve()
    payload: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    clock = ClockConfig(**payload.get("clock", {}))
    trajectory = payload.get("trajectory", {})
    speed_mps = float(trajectory.get("speed_mps", 50.0))
    altitude_m = float(trajectory.get("altitude_m", 300.0))
    lateral_offset_m = float(trajectory.get("lateral_offset_m", 0.0))
    if speed_mps <= 0.0:
        raise ValueError("trajectory.speed_mps must be positive")
    if altitude_m < 0.0:
        raise ValueError("trajectory.altitude_m must be non-negative")
    duration_raw = payload.get("duration_s")
    duration_s = None if duration_raw in (None, "") else float(duration_raw)
    if duration_s is not None and duration_s <= 0.0:
        raise ValueError("duration_s must be positive when supplied")
    scenario_path = (path.parent / payload["scenario"]).resolve()
    if not scenario_path.exists():
        raise FileNotFoundError(f"scenario does not exist: {scenario_path}")
    simulation = SimulationConfig(
        name=str(payload.get("name", path.stem)),
        scenario_path=str(scenario_path),
        clock=clock,
        metadata={"config_path": str(path)},
    )
    return SimulatorConfig(
        source_path=path,
        simulation=simulation,
        speed_mps=speed_mps,
        altitude_m=altitude_m,
        lateral_offset_m=lateral_offset_m,
        level_id=str(trajectory.get("level_id", "L300")),
        lane_id=str(trajectory.get("lane_id", "lane_0")),
        uam_id=str(payload.get("uam_id", "UAM01")),
        duration_s=duration_s,
    )
