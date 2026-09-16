import json
from pathlib import Path

import numpy as np
import pytest

from capacity_policy import load_scenario
from capacity_policy.geometry import Corridor
from uam_simulator.baseline_core import FixedStream, StreamEvaluation, aggregate_streams, evaluate_stream
from uam_simulator.baseline_studies import layout_streams, run_baseline_studies
from uam_simulator.path_design import SmoothPath, offset_path, path_constraints, search_offsets

ROOT = Path(__file__).resolve().parents[1]
# 95%-reliable all-active planning rate of the frozen 0.1.0 sf_sj_full bundle (scenario settings, 1 s clock, k = 1)
FROZEN_SF_SJ_ALL_ACTIVE_Q = 82.41129339946586


@pytest.mark.parametrize("name,definition,dt", [
    ("sf_sj_full", "all_active", 1.0),
    ("airport_to_airport", "full_group", 5.0),
    ("sf_sj_full", "full_group", 5.0),
])
def test_single_stream_exactly_reproduces_committed_baselines(name, definition, dt, tmp_path):
    text = (ROOT / f"dashboard/data/{name}_traffic.js").read_text()
    bundle = json.loads(text.removeprefix("window.UAM_TRAFFIC_DATA = ").strip().removesuffix(";"))
    scenario = load_scenario(ROOT / f"scenarios/{name}/scenario.json")
    result = evaluate_stream(scenario, FixedStream("lane_0", "L300", 0, 300, 112.5),
                             dt_s=dt, policy_definition=definition)
    if definition == "all_active":
        # The published sf_sj_full bundle now uses the calibrated stream settings. This case keeps
        # the frozen all-active reference of the 0.1.0 bundle (scenario settings, 1 s clock, k = 1),
        # which evaluate_stream reproduces exactly. The current group runner at those settings
        # differs by 47 of 178652 observations; that pre-existing difference is tracked separately.
        reference = {"policy": {"shares": {"C": 0.4581476837650852, "R": 0.3290363388039317,
                                           "F": 0.21281597743098313},
                                "observation_count": 178652},
                     "capacity": {"q_mix_rho_uam_h": FROZEN_SF_SJ_ALL_ACTIVE_Q}}
        assert result.summary["shares"] == reference["policy"]["shares"]
        assert result.summary["observation_count"] == reference["policy"]["observation_count"]
        q = reference["capacity"]["q_mix_rho_uam_h"]
    else:
        reference = bundle["summary"]["trb_reference_regression"]
        assert result.summary["shares"] == reference["shares"]
        assert result.summary["observation_count"] == reference["observation_count"]
        q = reference["q_mix_rho_uam_h"]
    assert result.summary["q_rho_uam_h"] == q
    finite = result.capacity_uam_h[np.isfinite(result.capacity_uam_h)]
    lower = 3600 * scenario.speed_mps / scenario.capacity.spacing_m("F")
    upper = 3600 * scenario.speed_mps / scenario.capacity.spacing_m("C")
    assert np.all((finite >= lower - 1e-9) & (finite <= upper + 1e-9))
    assert np.all(result.counts.sum(axis=1) <= result.active_count)


def test_low_demand_paper_estimator_is_unestimated_not_zero_capacity():
    scenario = load_scenario(ROOT / "scenarios/airport_to_airport/scenario.json")
    stream = evaluate_stream(scenario, FixedStream("lane_0", "L300", 0, 300, 1), policy_definition="full_group")
    assert stream.summary["status"] == "no_valid_groups"
    assert stream.summary["q_rho_uam_h"] is None
    assert aggregate_streams([stream])["summary"]["status"] == "no_common_valid_snapshots"


def fake_stream(q, demand=50):
    q = np.asarray(q, dtype=float)
    return StreamEvaluation(np.arange(len(q)), q, np.ones((len(q), 3), dtype=int),
                            np.ones(len(q)), {"demand_uam_h": demand})


def test_capacity_quantile_is_taken_after_simultaneous_sum():
    a, b = fake_stream([10, 100, 100]), fake_stream([100, 10, 100])
    result = aggregate_streams([a, b])
    assert result["summary"]["q_total_rho_uam_h"] == 110  # Not 10 + 10.
    assert result["summary"]["pooled_support_fraction"] == 1
    assert result["summary"]["fixed_allocation_support_fraction"] == 1 / 3
    assert result["summary"]["fixed_allocation_supported"] is False
    np.testing.assert_equal(aggregate_streams([a, a])["capacity_uam_h"], 2 * a.capacity_uam_h)


def test_missing_stream_snapshots_are_not_imputed_as_capacity():
    result = aggregate_streams([fake_stream([np.nan, 100]), fake_stream([100, 100])])
    assert np.isnan(result["capacity_uam_h"][0])
    assert result["summary"]["common_snapshot_count"] == 1
    with pytest.raises(ValueError, match="align"):
        aggregate_streams([fake_stream([10]), fake_stream([10, 20])])


def test_layout_demand_conservation_and_isolation():
    layout = {"offsets_m": [-250, 250], "altitudes_m": [300, 600]}
    for mode, total in [("fixed_total", 112.5), ("fixed_per_stream", 450)]:
        streams = layout_streams(layout, mode, 112.5)
        assert len({(s.lane_id, s.level_id) for s in streams}) == 4
        assert sum(s.demand_uam_h for s in streams) == total
    with pytest.raises(ValueError):
        FixedStream("lane_0", "L300", float("nan"), 300, 100)
    with pytest.raises(ValueError):
        layout_streams(layout, "typo", 100)


def test_straight_spline_constant_speed_endpoints_and_radius():
    corridor = Corridor.straight(50000)
    path = offset_path(corridor, np.zeros(6))
    np.testing.assert_allclose(path.interpolate([0, 500, 25000, 50000]), [[0, 0], [500, 0], [25000, 0], [50000, 0]], atol=1e-8)
    report = path_constraints(path, corridor)
    assert report["feasible"] and report["endpoint_error_m"] == 0
    assert report["max_curvature_per_m"] == 0
    assert report["min_radius_m"] is None  # Straight line has no finite radius.
    with pytest.raises(ValueError, match="endpoints"):
        offset_path(corridor, [10, 0, 0, 0])
    with pytest.raises(ValueError, match="second lateral"):
        path.interpolate([0], lateral_m=10)


def test_curvature_extrema_bound_dense_independent_samples():
    path = SmoothPath(np.array([[0, 0], [3000, 1000], [7000, -700], [10000, 0]]), "LOCAL_METRIC")
    dense = []
    for segment in path.coefficients:
        u = np.linspace(0, 1, 20001)[:, None]
        d = segment[1] + 2 * segment[2] * u + 3 * segment[3] * u*u
        dd = 2 * segment[2] + 6 * segment[3] * u
        dense.extend(abs(d[:, 0] * dd[:, 1] - d[:, 1] * dd[:, 0]) / np.linalg.norm(d, axis=1)**3)
    maximum = path.curvature_report()["max_curvature_per_m"]
    assert maximum >= max(dense) - 1e-12
    assert np.isclose(maximum, max(dense), rtol=1e-5)
    assert not path_constraints(path, Corridor.straight(10000))["feasible"]


def test_degenerate_cusp_is_not_a_feasible_smooth_route():
    path = SmoothPath(np.array([[0, 0], [1000, 0], [0, 0], [1000, 0]]), "LOCAL_METRIC")
    assert not path.curvature_report()["regular"]


def test_changed_endpoints_and_rounded_windows_are_rejected():
    path = SmoothPath(np.array([[0, 10], [20000, 10], [50000, 10]]), "LOCAL_METRIC")
    assert not path_constraints(path, Corridor.straight(50000))["feasible"]
    scenario = load_scenario(ROOT / "scenarios/airport_to_airport/scenario.json")
    with pytest.raises(ValueError, match="integer multiple"):
        evaluate_stream(scenario, FixedStream("a", "b", 0, 300, 100), dt_s=7, policy_definition="full_group")


def test_search_is_deterministic_and_never_selects_infeasible_candidate():
    corridor = Corridor.straight(50000)
    objective = lambda path: float(path.control_xy_m[1:-1, 1].sum())
    kwargs = dict(control_count=4, seed_offsets_m=[0], steps_m=[500], max_evaluations=20)
    a, b = search_offsets(corridor, objective, **kwargs), search_offsets(corridor, objective, **kwargs)
    assert a["records"] == b["records"]
    assert a["best"]["feasible"]
    assert a["best"]["objective_q_rho_uam_h"] >= 0
    impossible = search_offsets(corridor, objective, control_count=4, seed_offsets_m=[1000],
                                max_evaluations=1, maximum_deviation_m=1)
    assert impossible["best_path"] is None
    assert impossible["status"] == "no_feasible_candidate_found"


def test_small_experiment_writes_artifacts_and_refuses_overwrite(tmp_path):
    config = json.loads((ROOT / "research/_archive/itl-baselines/configs/research_baselines.json").read_text())
    config["scenarios"] = [str(ROOT / "scenarios/airport_to_airport/scenario.json")]
    config["lanes"]["estimators"] = [config["lanes"]["estimators"][1]]
    config["lanes"]["layouts"] = [config["lanes"]["layouts"][0]]
    config["lanes"]["demand_modes"] = ["fixed_total"]
    config["lanes"]["demand_values_uam_h"] = [112.5]
    config["offsets"]["max_evaluations"] = 3
    config["offsets"]["fixed_offset_values_m"] = [0]
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    result = run_baseline_studies(path, tmp_path / "run")
    assert result["airport_to_airport"]["offsets"]["search"]["best"]["feasible"]
    assert (tmp_path / "run/airport_to_airport/offsets/selected_path.geojson").is_file()
    manifest = json.loads((tmp_path / "run/manifest.json").read_text())
    assert manifest["inputs"] and manifest["code"] and manifest["outputs"]
    assert "git_dirty" in manifest["execution"]
    assert "--study all" in manifest["execution"]["command"]
    with pytest.raises(FileExistsError):
        run_baseline_studies(path, tmp_path / "run")
