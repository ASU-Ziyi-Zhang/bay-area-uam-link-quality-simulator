from pathlib import Path

import numpy as np

from capacity_policy import ConstantSpeedTrajectory, load_scenario


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "configs" / "scenario.json"


def test_real_scenario_loads_with_retained_sites() -> None:
    scenario = load_scenario(SCENARIO)
    assert scenario.scenario_id == "sf_sj_real"
    assert len(scenario.base_stations.stations) == 18
    assert scenario.base_stations.site_ids == tuple(f"BS{i:02d}" for i in range(1, 19))
    assert scenario.radio_ready
    assert np.isclose(scenario.corridor.length_m / 1000.0, 75.423116, atol=1e-5)


def test_centerline_trajectory_starts_and_ends_on_route() -> None:
    scenario = load_scenario(SCENARIO)
    time_s = np.asarray([0.0, scenario.transit_time_s])
    state = ConstantSpeedTrajectory(scenario.speed_mps).realize(
        scenario.corridor,
        time_s=time_s,
        entry_time_s=np.asarray([0.0]),
        altitude_m=np.asarray([300.0]),
        lateral_m=np.asarray([0.0]),
    )
    assert state.active.all()
    assert np.isclose(state.along_m[0, -1], scenario.corridor.length_m)
    assert np.allclose(state.position_m[0, 0, :2], scenario.corridor.xy_m[0])
    assert np.allclose(state.position_m[0, -1, :2], scenario.corridor.xy_m[-1])
    assert np.allclose(state.position_m[0, :, 2], 300.0)
