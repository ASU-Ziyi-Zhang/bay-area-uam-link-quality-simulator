import csv
import json
from pathlib import Path

import numpy as np

from capacity_policy import ConstantSpeedTrajectory, load_scenario


ROOT = Path(__file__).resolve().parents[1]
LEGACY_SCENARIO = ROOT / "configs" / "scenario.json"
FULL_SCENARIO = ROOT / "scenarios" / "sf_sj_full" / "scenario.json"
AIRPORT_SCENARIO = ROOT / "scenarios" / "airport_to_airport" / "scenario.json"


def test_real_scenario_loads_with_retained_sites() -> None:
    scenario = load_scenario(LEGACY_SCENARIO)
    assert scenario.scenario_id == "sf_sj_real"
    assert len(scenario.base_stations.stations) == 18
    assert scenario.base_stations.site_ids == tuple(f"BS{i:02d}" for i in range(1, 19))
    assert scenario.radio_ready
    assert np.isclose(scenario.corridor.length_m / 1000.0, 75.423116, atol=1e-5)


def test_centerline_trajectory_starts_and_ends_on_route() -> None:
    scenario = load_scenario(FULL_SCENARIO)
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


def test_full_scenario_pack_preserves_frozen_inputs() -> None:
    scenario = load_scenario(FULL_SCENARIO)
    assert scenario.scenario_id == "sf_sj_full"
    assert scenario.base_stations.site_ids == tuple(f"BS{i:02d}" for i in range(1, 19))
    assert (ROOT / "data/corridor.geojson").read_bytes() == (
        ROOT / "scenarios/sf_sj_full/data/corridor.geojson"
    ).read_bytes()
    assert (ROOT / "data/base_stations.csv").read_bytes() == (
        ROOT / "scenarios/sf_sj_full/data/base_stations.csv"
    ).read_bytes()


def test_airport_scenario_uses_station_proxies_and_retains_only_nearby_sites() -> None:
    scenario = load_scenario(AIRPORT_SCENARIO)
    expected_ids = tuple(f"BS{i:02d}" for i in range(5, 17))
    assert scenario.scenario_id == "airport_to_airport"
    assert scenario.base_stations.site_ids == expected_ids
    assert scenario.radio_ready
    assert np.isclose(scenario.corridor.length_m / 1000.0, 49.542589, atol=1e-6)

    report = json.loads(
        (ROOT / "scenarios/airport_to_airport/build_report.json").read_text(encoding="utf-8")
    )
    assert tuple(report["retained_site_ids"]) == expected_ids
    assert report["site_inclusion_distance_km"] == 5.0
    assert all(endpoint["projection_error_m"] < 1e-6 for endpoint in report["endpoint_results"])

    with (ROOT / "scenarios/airport_to_airport/data/base_stations.csv").open(
        "r", encoding="utf-8-sig", newline=""
    ) as stream:
        rows = list(csv.DictReader(stream))
    assert all(float(row["lateral_offset_km"]) <= 5.0 for row in rows)
    assert all(0.0 <= float(row["chainage_km"]) <= 49.543 for row in rows)
    assert all(row["source_chainage_km"] for row in rows)
