import numpy as np
import pytest

from uam_simulator.interfaces import ClockConfig
from uam_simulator.runner import make_time_grid


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
