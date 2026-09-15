import json
from pathlib import Path

import numpy as np
import pytest

from uam_simulator.aks import minimum_duration, plan_equal_speed_transition
from uam_simulator.two_uam_longitudinal import resolve, plan_request, simulate


@pytest.fixture
def spec():
    return json.loads((Path(__file__).parents[1] / "research/dynamic-transitions/configs/two_uam_longitudinal.json").read_text())


def test_actual_gap_and_objective_determine_request(spec):
    cfg = resolve(spec["parameters"])
    assert plan_request(cfg, 4500, "F")["status"] == "hold"
    p = plan_request(cfg, 4500, "F", "exact_target")
    assert p["target_reduction_m"] == pytest.approx(882.5)
    assert p["transition"].junction > 50
    assert plan_request(cfg, 4500, "F", leader_speed=30)["status"] == "observed_state_acc"


def test_recovery_closes_established_pair_but_does_not_pursue_large_gap(spec):
    spec["parameters"]["controller"]["acc_policy_recovery"] = True
    cfg = resolve(spec["parameters"])
    case = {"id": "recovery", "initial_gap_policy": "F", "target_policy": "C",
            "objective": "exact_target", "established_following": True}
    result = simulate(case, cfg, "acc", .125)
    assert result["acc_policy_recovery_active"]
    assert result["trace"][0]["acceleration_mps2"] == 1.5
    assert max(r["follower_v_mps"] for r in result["trace"]) > 60
    assert result["trace"][-1]["gap_m"] < result["trace"][0]["gap_m"]-1500
    hold = {"id": "hold", "initial_gap_m": 4500, "target_policy": "F",
            "objective": "minimum_spacing", "established_following": False}
    result = simulate(hold, cfg, "acc", .125)
    assert not result["acc_policy_recovery_active"]
    assert all(r["follower_v_mps"] == 50 for r in result["trace"])


def test_policy_parameter_changes_propagate_without_duration_input(spec):
    cfg = resolve(spec["parameters"])
    base = plan_request(cfg, 1367.5, "F")
    # With the wing-borne floor at 30 m/s the junction, not the acceleration
    # limit, sets the duration: T >= 2|dS| / (cruise - speed_min).
    assert base["duration_s"] == pytest.approx(2 * 2250 / (50 - 30))
    assert base["transition"].junction == pytest.approx(cfg.vehicle.speed_min_mps)
    spec["parameters"]["spacing"]["tau_f_s"] = 45
    changed = plan_request(resolve(spec["parameters"]), 1367.5, "F")
    assert changed["target_reduction_m"] == -1500
    # A smaller displacement moves the binding constraint back to acceleration.
    assert changed["duration_s"] == pytest.approx(
        max(np.sqrt(3 * 1500 * (1 / 1.5 + 1 / 2)), 2 * 1500 / (50 - 30)))


def test_a_zero_floor_recovers_the_retired_mathematical_bound(spec):
    """The old floor of zero let the aircraft hover, which shortened everything."""
    legacy = resolve(spec["parameters"], {"speed_min_mps": 0.0})
    assert plan_request(legacy, 1367.5, "F")["duration_s"] == pytest.approx(90)
    wing_borne = resolve(spec["parameters"])
    assert (plan_request(wing_borne, 1367.5, "F")["duration_s"]
            > plan_request(legacy, 1367.5, "F")["duration_s"])


def test_bounds_propagate_to_duration_and_refusal(spec):
    cfg = resolve(spec["parameters"], {"speed_min_mps": 30})
    p = plan_request(cfg, 1367.5, "F")
    assert p["duration_s"] == pytest.approx(225)
    cfg = resolve(spec["parameters"], {"speed_max_mps": 50})
    assert plan_request(cfg, 3617.5, "C", "exact_target")["status"] == "infeasible"


@pytest.mark.parametrize("jerk", [.05, .18])
def test_joint_phase_feasibility_and_minimality(jerk):
    r = minimum_duration(-2250, 50, 1.5, 2, jerk_limit=jerk, speed_max=60)
    t = r["duration_s"]
    p = plan_equal_speed_transition(50, -2250, t, accel_limit=1.5,
        decel_limit=2, jerk_limit=jerk, speed_max=60)
    assert p["feasible"]
    tr = p["transition"]
    # Independent endpoint formulas, not the implementation peak helpers.
    durations = (tr.alpha*t, (1-tr.alpha)*t)
    dv = abs(tr.junction-50)
    assert 1.5*dv/durations[0] <= 2+1e-10
    assert 1.5*dv/durations[1] <= 1.5+1e-10
    assert max(6*dv/d**2 for d in durations) <= jerk+1e-10
    smaller = t*(1-1e-5)
    lower1 = max(3*2250/(2*smaller**2), np.sqrt(12*2250/(jerk*smaller**3)))
    lower2 = max(3*2250/(1.5*smaller**2), np.sqrt(12*2250/(jerk*smaller**3)))
    assert lower1+lower2 > 1


def test_hold_and_leader_change_exercise_different_paths(spec):
    cfg = resolve(spec["parameters"])
    hold = simulate(spec["cases"][0], cfg, "aks", .5)
    assert all(row["follower_v_mps"] == 50 for row in hold["trace"])
    dynamic = simulate(spec["cases"][4], cfg, "aks", .5)
    assert dynamic["events"][1] == {"t_s": 20, "event": "reference_invalidated_leader_changed"}
    assert min(row["follower_v_mps"] for row in dynamic["trace"]) < 49


def test_resolver_rejects_conflicting_or_unknown_inputs(spec):
    with pytest.raises((ValueError, TypeError)):
        resolve(spec["parameters"], {"speed_max_mps": 40})
    with pytest.raises((ValueError, TypeError)):
        resolve({**spec["parameters"], "duration_s": 90})
