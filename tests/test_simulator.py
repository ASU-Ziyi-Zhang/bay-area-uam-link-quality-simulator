import numpy as np
import pytest
from pathlib import Path

from uam_simulator.config import load_simulator_config
from uam_simulator.interfaces import ClockConfig
from uam_simulator.runner import make_time_grid


ROOT = Path(__file__).resolve().parents[1]


def test_default_clock_preserves_separate_rates() -> None:
    clock = ClockConfig()
    assert clock.dt_motion_s == 1.0
    assert clock.dt_radio_s == 1.0
    assert clock.dt_control_s == 5.0
    assert clock.policy_window_s == 30.0
    assert clock.capacity_aggregation_s == 60.0


def test_clock_rejects_nonpositive_intervals() -> None:
    with pytest.raises(ValueError):
        ClockConfig(dt_radio_s=0.0)


def test_time_grid_includes_exact_noninteger_terminal_time() -> None:
    grid = make_time_grid(10.4, 1.0)
    assert np.allclose(grid[:-1], np.arange(0.0, 11.0, 1.0))
    assert np.isclose(grid[-1], 10.4)


def test_time_grid_does_not_duplicate_exact_terminal_time() -> None:
    assert np.allclose(make_time_grid(10.0, 5.0), [0.0, 5.0, 10.0])


@pytest.mark.parametrize("scenario_name", ["sf_sj_full", "airport_to_airport"])
def test_scenario_pack_simulator_config_resolves_locally(scenario_name: str) -> None:
    config = load_simulator_config(ROOT / "scenarios" / scenario_name / "simulator.json")
    assert Path(config.simulation.scenario_path) == (
        ROOT / "scenarios" / scenario_name / "scenario.json"
    ).resolve()
    assert config.speed_mps == 50.0
    assert config.altitude_m == 300.0
    assert config.lateral_offset_m == 0.0
