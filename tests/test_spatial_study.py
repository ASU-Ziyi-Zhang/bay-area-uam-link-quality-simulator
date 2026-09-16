from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.geometry import Corridor
from capacity_policy.radio import RadioConfig
from uam_simulator.lateral_study import LateralConfig, SmoothCorridorFrame, rk4_progress, simulate_isolated
from uam_simulator.minimum_change import MinimumChangeConfig, MinimumChangePlanner
from uam_simulator.spatial_study import (SpatialConfig, SpatialMove, SpatialPlanner, candidate_pairs,
    direction, duration_min, rolling_endpoint_candidates, simulate_spatial, xyz_at)


@pytest.mark.parametrize("target", [(100, 300), (0, 400), (0, 200), (100, 400), (-100, 200)])
def test_spatial_quintic_endpoints_and_component_limits(target):
    cfg = SpatialConfig(); source = (0, 300)
    duration = duration_min(source, target, cfg)
    move = SpatialMove(5, source, target, duration)
    for t, expected in [(5, source), (5 + duration, target)]:
        p, v, a = move.sample(t)
        np.testing.assert_allclose(p, expected)
        np.testing.assert_allclose(v, 0, atol=1e-12)
        np.testing.assert_allclose(a, 0, atol=1e-12)
    p, v, a = move.sample(np.linspace(5, 5 + duration, 10001))
    assert np.all(np.max(abs(v), axis=0) <= np.array([8, 3]) + 1e-8)
    assert np.all(np.max(abs(a), axis=0) <= np.array([.3, .2]) + 1e-8)
    assert duration == duration_min(source, target, replace(cfg, dt_s=.25))


def test_candidate_sets_and_diagonal_are_not_forced():
    d, h = [-100, 0, 100], [200, 300, 400]
    assert candidate_pairs(d, h, (0, 300), "none") == []
    assert len(candidate_pairs(d, h, (0, 300), "lateral")) == 2
    assert len(candidate_pairs(d, h, (0, 300), "vertical")) == 2
    assert len(candidate_pairs(d, h, (0, 300), "joint")) == 8
    assert direction((0, 300), (100, 200)) == "diagonal_descent"
    assert direction((0, 300), (-100, 400)) == "diagonal_climb"
    assert duration_min((0, 300), (0, 400), SpatialConfig()) == 62.5


def test_curved_diagonal_cartesian_derivatives_and_bound():
    x = np.linspace(0, 10000, 21)
    frame = SmoothCorridorFrame(Corridor(np.c_[x, 200 * np.sin(x / 2000)], "LOCAL_METRIC"), 500)
    move = SpatialMove(0, (0, 300), (100, 200), 62.5)
    q, t, eps = 2100., 25., .001
    dh, vd, ad = move.sample(t)
    vxy, axy = frame.kinematics(q, dh[0], vd[0], ad[0], 50)
    v, a = np.r_[vxy, vd[1]], np.r_[axy, ad[1]]
    qm = rk4_progress(frame, q, t, -eps, lambda s: move.sample(s)[0][0], 50)
    qp = rk4_progress(frame, q, t, eps, lambda s: move.sample(s)[0][0], 50)
    pm, p0, pp = xyz_at(frame, qm, *move.sample(t - eps)[0]), xyz_at(frame, q, *dh), xyz_at(frame, qp, *move.sample(t + eps)[0])
    np.testing.assert_allclose((pp - pm) / (2 * eps), v, atol=1e-6)
    np.testing.assert_allclose((pp - 2 * p0 + pm) / eps**2, a, atol=2e-5)
    assert np.linalg.norm(v) == pytest.approx(np.sqrt(50**2 + np.sum(vd**2)))
    planner = SpatialPlanner(frame)
    bound = planner.curvature.offset_bound(0, frame.length_m, [0, 100]) + 10 / np.sqrt(3) * np.sqrt(20000) / 62.5**2 / 50**2
    for tt in np.linspace(0, 62.5, 101):
        dh, vd, ad = move.sample(tt)
        vxy, axy = frame.kinematics(50 * tt, dh[0], vd[0], ad[0], 50)
        vv, aa = np.r_[vxy, vd[1]], np.r_[axy, ad[1]]
        assert np.linalg.norm(np.cross(vv, aa)) / np.linalg.norm(vv)**3 <= bound + 1e-12


def test_spatial_planner_selects_diagonal_only_when_needed():
    frame = SmoothCorridorFrame(Corridor.straight(5000), 100)
    cfg = replace(SpatialConfig(), cruise_mps=10, policy_s=1, decision_s=2, window_s=3,
        horizon_s=40, min_maneuver_s=2, lateral_speed_limit_mps=10, lateral_accel_limit_mps2=10,
        vertical_speed_limit_mps=10, vertical_accel_limit_mps2=10)
    planner = SpatialPlanner(frame, MinimumChangeConfig(minimum_radius_m=1, candidate_spacing_m=10))
    radio = lambda p: {"sinr_db": np.where((p[:, 1] >= 5) & (p[:, 2] <= 295), 10., -10.)}
    rows = planner.candidates(frame, radio, cfg, q=0, t=0, source=(0, 300),
        targets=candidate_pairs([-10, 0, 10], [290, 300, 310], (0, 300), "joint"), history=[(0, 1)], policy="F")
    chosen = planner.choose(rows)
    assert chosen["target"] == [10, 290]
    assert chosen["direction"] == "diagonal_descent"
    assert chosen["displacement_m"] == pytest.approx(np.sqrt(200))


def test_lateral_forecasts_reproduce_old_gate():
    frame = SmoothCorridorFrame(Corridor.straight(10000), 500)
    cfg = SpatialConfig()
    radio = lambda p: {"sinr_db": np.where(p[:, 1] > 50, 10., -10.)}
    old = MinimumChangePlanner(frame).evaluate(frame, radio, cfg, q=0, t=0, source=0,
        target=100, durations=[65, 70], history=[(0, 1)])
    new = SpatialPlanner(frame).evaluate(frame, radio, cfg, q=0, t=0, source=(0, 300),
        alternatives=[((100, 300), 65), ((100, 300), 70)], history=[(0, 1)])
    for a, b in zip(old, new):
        for key in ["status", "recovery_time_s", "stay_recovery_time_s", "duration_s", "maneuver_curvature_bound_per_m"]:
            assert a.get(key) == b.get(key)


def test_disabled_spatial_motion_matches_lateral_baseline():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    scenario = SimpleNamespace(base_stations=BaseStationSet((BaseStation("B", 500, 0, 10),), "LOCAL_METRIC"),
                               radio=RadioConfig(served_set_size=1))
    old = simulate_isolated(frame, scenario, LateralConfig(), [0], 0, False)
    new = simulate_spatial(frame, scenario, SpatialConfig(), [-100, 0, 100], [200, 300, 400], "none")
    for a, b in zip(old["trace"], new["trace"]):
        for key in ["t_s", "end_s", "q_m", "position_m", "velocity_mps", "acceleration_mps2", "policy", "exposure"]:
            assert a[key] == b[key]
    assert old["summary"]["path_length_m"] == new["summary"]["path_length_m"]


def test_c_policy_does_not_invoke_spatial_forecast():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    def radio(_):
        raise AssertionError("unexpected prediction while C")
    rows = SpatialPlanner(frame).candidates(frame, radio, SpatialConfig(), q=0, t=0,
        source=(0, 300), targets=[(100, 400)], history=[(0, 0)], policy="C")
    assert rows[0]["status"] == "policy_satisfied_hold"


def test_rolling_endpoint_requires_strict_policy_upgrade():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    cfg = replace(SpatialConfig(), cruise_mps=10, policy_s=1, decision_s=1,
        window_s=3, horizon_s=20, forecast_dt_s=1, min_maneuver_s=2,
        lateral_speed_limit_mps=10, lateral_accel_limit_mps2=10,
        vertical_speed_limit_mps=10, vertical_accel_limit_mps2=10,
        predictive_enabled=True, predictive_rollout_s=10,
        predictive_endpoint_upgrade=True)
    planner = SpatialPlanner(frame, MinimumChangeConfig(minimum_radius_m=1,
                                                        candidate_spacing_m=10))
    radio = lambda p: {"sinr_db": np.where(p[:, 1] < -5, 10., -10.)}
    rows = rolling_endpoint_candidates(frame, radio, cfg, planner, q=0, t=0,
        source=(0, 300), targets=[(-10, 300), (10, 300)], history=[(0, 0)])
    assert rows[0]["endpoint_policy"] == "F"
    assert rows[1]["endpoint_policy"] == "C" and rows[1]["status"] == "eligible"
    assert rows[2]["endpoint_policy"] == "F" and rows[2]["status"] == "endpoint_policy_not_improved"


def test_dual_horizon_accepts_maneuver_longer_than_warning_when_cost_improves():
    frame = SmoothCorridorFrame(Corridor.straight(2000), 100)
    cfg = replace(SpatialConfig(), cruise_mps=10, policy_s=1, decision_s=1,
        window_s=3, horizon_s=60, forecast_dt_s=1, min_maneuver_s=30,
        lateral_speed_limit_mps=10, lateral_accel_limit_mps2=10,
        vertical_speed_limit_mps=10, vertical_accel_limit_mps2=10,
        predictive_enabled=True, predictive_rollout_s=20,
        predictive_evaluation_s=20, predictive_endpoint_upgrade=True)
    planner = SpatialPlanner(frame, MinimumChangeConfig(minimum_radius_m=1,
                                                        candidate_spacing_m=10))
    radio = lambda p: {"sinr_db": np.where(p[:, 1] < -5, 10., -10.)}
    rows = rolling_endpoint_candidates(frame, radio, cfg, planner, q=0, t=0,
        source=(0, 300), targets=[(-10, 300)], history=[(0, 0)])
    move = rows[1]
    assert move["duration_s"] == 30
    assert move["duration_s"] > move["warning_horizon_s"]
    assert move["planning_horizon_s"] == 40
    assert move["policy_cost_s"] < move["stay_policy_cost_s"]
    assert move["status"] == "eligible"


def test_rolling_endpoint_requires_positive_aligned_horizon():
    with pytest.raises(ValueError):
        SpatialConfig(predictive_endpoint_upgrade=True, predictive_rollout_s=0)
    with pytest.raises(ValueError):
        SpatialConfig(predictive_endpoint_upgrade=True, predictive_rollout_s=61)
    with pytest.raises(ValueError):
        SpatialConfig(predictive_endpoint_upgrade=True, predictive_rollout_s=60,
                      predictive_evaluation_s=61)


@pytest.mark.parametrize("key", ["vertical_speed_limit_mps", "vertical_accel_limit_mps2"])
def test_invalid_vertical_limits_rejected(key):
    with pytest.raises(ValueError):
        SpatialConfig(**{key: 0})
