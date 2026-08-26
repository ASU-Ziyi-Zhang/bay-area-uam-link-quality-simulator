"""Configuration for the fixed-lane multi-UAM group-policy simulator."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from .interfaces import ClockConfig, SimulationConfig


@dataclass(frozen=True)
class GroupSimulatorConfig:
    """Validated traffic-stream settings plus the referenced scenario."""

    simulation: SimulationConfig
    source_path: Path
    entry_demand_uam_h: float
    reliability_rho: float
    speed_mps: float
    altitude_m: float
    lateral_offset_m: float
    level_id: str
    lane_id: str
    duration_s: float | None

    @property
    def entry_interval_s(self) -> float:
        return 3600.0 / self.entry_demand_uam_h


def load_group_simulator_config(path: str | Path) -> GroupSimulatorConfig:
    path = Path(path).resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    trajectory = payload["trajectory"]
    traffic = payload["traffic"]
    clock = ClockConfig(**payload.get("clock", {}))
    demand = float(traffic["entry_demand_uam_h"])
    rho = float(traffic.get("reliability_rho", 0.95))
    if demand <= 0.0:
        raise ValueError("entry_demand_uam_h must be positive")
    if not 0.0 < rho <= 1.0:
        raise ValueError("reliability_rho must lie in (0, 1]")
    duration = traffic.get("duration_s")
    scenario_path = (path.parent / payload["scenario"]).resolve()
    if not scenario_path.exists():
        raise FileNotFoundError(f"scenario does not exist: {scenario_path}")
    return GroupSimulatorConfig(
        simulation=SimulationConfig(
            name=payload["name"],
            scenario_path=str(scenario_path),
            clock=clock,
            metadata=payload.get("metadata", {}),
        ),
        source_path=path,
        entry_demand_uam_h=demand,
        reliability_rho=rho,
        speed_mps=float(trajectory["speed_mps"]),
        altitude_m=float(trajectory["altitude_m"]),
        lateral_offset_m=float(trajectory.get("lateral_offset_m", 0.0)),
        level_id=str(trajectory.get("level_id", "L300")),
        lane_id=str(trajectory.get("lane_id", "lane_0")),
        duration_s=None if duration is None else float(duration),
    )

