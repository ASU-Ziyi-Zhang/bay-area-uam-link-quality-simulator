from dataclasses import replace
from pathlib import Path
import importlib.util
import json
import hashlib

import numpy as np
import pytest

from capacity_policy import load_scenario, compute_link_state, ConstantSpeedTrajectory
from capacity_policy.trajectory import TrajectoryState
from uam_simulator.baseline_core import evaluate_stream
from uam_simulator.baseline_studies import layout_streams
from uam_simulator.geometry_sweep import evaluate_location, evaluate_locations, geometry_grid, radio_profile

ROOT = Path(__file__).resolve().parents[1]


def scenario():
    return load_scenario(ROOT / "scenarios/sf_sj_full/scenario.json")


def test_radio_has_no_cross_aircraft_or_cross_stream_interference():
    s = scenario()
    a = ConstantSpeedTrajectory(s.speed_mps).realize(s.corridor, np.arange(0, 1000, 5),
        np.array([0, 32]), np.array([300, 300]), np.array([-250, -250]))
    b = ConstantSpeedTrajectory(s.speed_mps).realize(s.corridor, a.time_s,
        np.array([0, 8, 16]), np.array([600, 300, 450]), np.array([250, -250, 0]))
    joined = TrajectoryState(a.time_s, np.concatenate([a.position_m, b.position_m]),
        np.concatenate([a.along_m, b.along_m]), np.concatenate([a.active, b.active]))
    single, multiple = compute_link_state(a, s.base_stations, s.radio), compute_link_state(joined, s.base_stations, s.radio)
    for key in single.keys() - {"t"}:
        np.testing.assert_array_equal(single[key], multiple[key][:2])


@pytest.mark.parametrize("mode,per_stream", [("fixed_total", 28.125), ("fixed_per_stream", 112.5)])
@pytest.mark.parametrize("definition,dt", [("full_group", 5.0), ("all_active", 1.0)])
def test_single_location_equals_corresponding_multilane_stream(mode, per_stream, definition, dt):
    s = scenario()
    spec = layout_streams({"offsets_m": [-250, 250], "altitudes_m": [300, 600]}, mode, 112.5)[0]
    member = evaluate_stream(s, spec, dt_s=dt, policy_definition=definition)
    alone = evaluate_location(s, -250, 300, demand_uam_h=per_stream, dt_s=dt, policy_definition=definition)
    np.testing.assert_array_equal(alone.stream.capacity_uam_h, member.capacity_uam_h)
    np.testing.assert_array_equal(alone.stream.counts, member.counts)
    assert alone.summary["q_rho_uam_h"] == member.summary["q_rho_uam_h"]


def test_radio_profile_is_demand_independent_but_group_capacity_is_not():
    a = evaluate_location(scenario(), -250, 300, demand_uam_h=112.5)
    b = evaluate_location(scenario(), -250, 300, demand_uam_h=28.125)
    np.testing.assert_array_equal(a.profile["sinr_db"], b.profile["sinr_db"])
    assert a.summary["profile_below_threshold_fraction"] == b.summary["profile_below_threshold_fraction"]
    assert a.summary["q_rho_uam_h"] != b.summary["q_rho_uam_h"]


def test_arbitrary_positions_not_limited_to_two_lanes_or_levels():
    s = replace(scenario(), simulation_duration_s=60)
    results = geometry_grid(s, [-700, -100, 300], [150, 250, 450, 750], policy_definition="all_active")
    assert len(results) == 12
    assert {r.summary["demand_uam_h"] for r in results} == {112.5}
    assert len(evaluate_locations(s, [(123, 300), (-456, 230), (10, 170)], policy_definition="all_active")) == 3
    with pytest.raises(ValueError, match="distinct"):
        evaluate_locations(s, [(0, 300), (0, 300)])
    with pytest.raises(ValueError, match="at least"):
        evaluate_locations(s, [])


def test_profile_domain_cautions_and_uniform_route_sampling():
    s = scenario()
    for height in [150, 300, 600]:
        summary, profile = radio_profile(s, -250, height)
        assert summary["height_extrapolated"] == (height > 300)
        assert profile["along_m"][0] == 0
        assert profile["along_m"][-1] == s.corridor.length_m
        np.testing.assert_allclose(np.diff(profile["along_m"]), summary["profile_spacing_m"])
        assert summary["profile_spacing_m"] <= 50
        assert 0 <= summary["profile_any_served_link_beyond_4km_fraction"] <= 1
    with pytest.raises(ValueError, match="positive"):
        radio_profile(s, 0, 300, profile_step_m=0)


def test_sweep_runner_preserves_results_and_records_provenance(tmp_path):
    spec = importlib.util.spec_from_file_location("sweep_cli", ROOT / "scripts/run_geometry_sweep.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    config = json.loads((ROOT / "research/geometry-response/configs/geometry_sweep.json").read_text())
    config.update(scenarios=[str(ROOT / "scenarios/sf_sj_full/scenario.json")], offsets_m=[0], altitudes_m=[300])
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    output = tmp_path / "results"
    assert cli.run(path, output) == {"sf_sj_full": 1}
    manifest = json.loads((output / "manifest.json").read_text())
    assert "git_commit" in manifest["execution"]
    assert "run_geometry_sweep.py" in manifest["execution"]["command"]
    for row in manifest["outputs"]:
        assert hashlib.sha256((output / row["path"]).read_bytes()).hexdigest() == row["sha256"]
    slices = json.loads((output / "sf_sj_full/plot_slices.json").read_text())
    assert slices["height"] == slices["offset"]
    assert slices["height"][0]["q_rho_uam_h"] == 72.47021371222584
    with pytest.raises(FileExistsError, match="refusing"):
        cli.run(path, output)
