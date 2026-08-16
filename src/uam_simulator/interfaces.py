"""Stable contracts shared by the future simulator components.

The module intentionally contains no channel, policy, or capacity formula.
Those models can evolve behind these contracts without changing the simulator
clock or trace schema.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, Sequence


@dataclass(frozen=True)
class ClockConfig:
    """Independent clocks for motion, radio, control, policy, and capacity."""

    dt_motion_s: float = 1.0
    dt_radio_s: float = 1.0
    dt_control_s: float = 5.0
    policy_window_s: float = 30.0
    capacity_aggregation_s: float = 60.0

    def __post_init__(self) -> None:
        values = {
            "dt_motion_s": self.dt_motion_s,
            "dt_radio_s": self.dt_radio_s,
            "dt_control_s": self.dt_control_s,
            "policy_window_s": self.policy_window_s,
            "capacity_aggregation_s": self.capacity_aggregation_s,
        }
        if any(value <= 0 for value in values.values()):
            raise ValueError("all clock intervals must be positive")
        if self.policy_window_s < self.dt_control_s:
            raise ValueError("policy_window_s must be >= dt_control_s")
        if self.capacity_aggregation_s < self.dt_control_s:
            raise ValueError("capacity_aggregation_s must be >= dt_control_s")


@dataclass(frozen=True)
class UAMState:
    """One time-stamped vehicle state in a corridor/lane/level graph."""

    timestamp_s: float
    uam_id: str
    corridor_id: str
    s_m: float
    level_id: str
    lane_id: str
    x_m: float
    y_m: float
    z_m: float
    speed_mps: float
    heading_rad: float = 0.0
    active: bool = True


@dataclass(frozen=True)
class RadioObservation:
    timestamp_s: float
    uam_id: str
    serving_bs_id: str | None
    rsrp_dbm_per_re: float | None
    sinr_db: float | None
    handoff: bool = False
    measurements: Mapping[str, Mapping[str, float]] = field(default_factory=dict)


@dataclass(frozen=True)
class PolicyAction:
    """Bounded intent; feasibility remains the controller/integrator's job."""

    timestamp_s: float
    uam_id: str
    target_speed_mps: float | None = None
    acceleration_mps2: float | None = None
    target_level_id: str | None = None
    target_lane_id: str | None = None
    lateral_offset_m: float | None = None


@dataclass(frozen=True)
class CapacitySnapshot:
    window_start_s: float
    window_end_s: float
    corridor_id: str
    level_id: str
    lane_id: str
    flow_vph: float
    occupancy: float
    bottleneck: str | None = None


@dataclass(frozen=True)
class SimulationConfig:
    name: str
    scenario_path: str
    clock: ClockConfig = field(default_factory=ClockConfig)
    metadata: Mapping[str, Any] = field(default_factory=dict)


class CorridorProvider(Protocol):
    def position_at(self, s_m: float, level_id: str, lane_id: str) -> tuple[float, float, float]: ...

    def heading_at(self, s_m: float, level_id: str, lane_id: str) -> float: ...


class TrajectoryIntegrator(Protocol):
    def advance(self, state: UAMState, dt_s: float, action: PolicyAction | None = None) -> UAMState: ...


class RadioModel(Protocol):
    def observe(self, state: UAMState) -> RadioObservation: ...


class PolicyModel(Protocol):
    def decide(self, state: UAMState, observation: RadioObservation) -> PolicyAction: ...


class Controller(Protocol):
    def constrain(self, action: PolicyAction, state: UAMState) -> PolicyAction: ...


class CapacityModel(Protocol):
    def aggregate(self, states: Sequence[UAMState], window_start_s: float, window_end_s: float) -> Sequence[CapacitySnapshot]: ...


class TraceSink(Protocol):
    def write(self, record: Mapping[str, Any]) -> None: ...
