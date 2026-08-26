import json
from pathlib import Path
import csv

import numpy as np

from uam_simulator.group_config import load_group_simulator_config
from uam_simulator.group_runner import reliability_floor, run_group_simulation


ROOT = Path(__file__).resolve().parents[1]


def test_group_config_declares_fixed_lane_level_baseline():
    config = load_group_simulator_config(
        ROOT / "scenarios" / "airport_to_airport" / "group_simulator.json"
    )
    assert config.entry_demand_uam_h == 112.5
    assert config.entry_interval_s == 32.0
    assert config.altitude_m == 300.0
    assert config.lateral_offset_m == 0.0
    assert config.simulation.metadata["lane_change_allowed"] is False
    assert config.simulation.metadata["level_change_allowed"] is False


def test_reliability_floor_matches_lower_five_percent_order_statistic():
    values = np.arange(1.0, 101.0)
    assert reliability_floor(values, 0.95) == 6.0


def test_real_airport_group_run_emits_policy_shares_and_capacity(tmp_path):
    summary = run_group_simulation(
        ROOT / "scenarios" / "airport_to_airport" / "group_simulator.json",
        tmp_path / "run",
    )
    assert summary["policy"]["maximum_group_size"] == 5
    assert summary["policy"]["minimum_group_size"] == 1
    assert np.isclose(sum(summary["policy"]["shares"].values()), 1.0)
    assert summary["policy"]["observation_count"] > 0
    assert summary["capacity"]["q_mix_rho_uam_h"] > 0.0
    assert summary["warmup_s"] == 0.0
    assert (tmp_path / "run" / "entrants.csv").exists()
    assert (tmp_path / "run" / "group_policy_trace.csv").exists()
    assert (tmp_path / "run" / "capacity_trace.csv").exists()
    manifest = json.loads((tmp_path / "run" / "manifest.json").read_text())
    assert manifest["qa"]["policy_shares_sum_to_one"] is True
    assert manifest["qa"]["no_lane_or_level_change"] is True
    assert manifest["qa"]["all_active_aircraft_classified"] is True
    assert manifest["qa"]["simulation_starts_at_zero"] is True

    with (tmp_path / "run" / "group_policy_trace.csv").open(
        encoding="utf-8-sig", newline=""
    ) as stream:
        policy_rows = list(csv.DictReader(stream))
    at_zero = [row for row in policy_rows if float(row["timestamp_s"]) == 0.0]
    assert len(at_zero) == 1
    assert at_zero[0]["focal_uam_id"] == "UAM001"
    assert int(at_zero[0]["group_size"]) == 1
    at_160 = [row for row in policy_rows if float(row["timestamp_s"]) == 160.0]
    assert [int(row["group_size"]) for row in at_160] == [3, 4, 5, 5, 4, 3]
    assert {row["policy"] for row in policy_rows} <= {"C", "R", "F"}

    reference = summary["trb_reference_regression"]
    assert np.isclose(sum(reference["shares"].values()), 1.0)
    assert reference["observation_count"] > 0
