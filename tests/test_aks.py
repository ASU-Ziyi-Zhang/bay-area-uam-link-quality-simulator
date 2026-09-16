import numpy as np
import pytest

from uam_simulator.aks import (
    KS2Transition, equal_speed_junction, hermite, junction_speed,
    minimum_duration, optimal_split, plan_equal_speed_transition,
    reference_gap, tracking_command,
)

# The policy spacings the traffic model uses at 50 m/s cruise.
SPACING = {"C": 1367.5, "R": 2117.5, "F": 3617.5}
CRUISE = 50.0


def test_hermite_endpoint_conditions():
    h0, dh0, _ = hermite(0.0)
    h1, dh1, _ = hermite(1.0)
    assert (h0, dh0) == pytest.approx((0.0, 0.0))
    assert (h1, dh1) == pytest.approx((1.0, 0.0))
    # Peak slope at the midpoint is 3/2, so a cubic phase peaks at 3|dv|/(2T).
    assert hermite(0.5)[1] == pytest.approx(1.5)


def test_equal_speed_junction_is_independent_of_the_split():
    reduction = SPACING["C"] - SPACING["F"]      # negative: the gap opens
    direct = equal_speed_junction(CRUISE, reduction, 300.0)
    required = CRUISE * 300.0 + reduction
    for alpha in (0.2, 0.5, 0.8):
        assert junction_speed(CRUISE, CRUISE, 300.0, required, alpha) == pytest.approx(direct)
    assert direct == pytest.approx(CRUISE + 2 * reduction / 300.0)
    assert direct < CRUISE      # opening a gap dips below cruise


def test_closing_a_gap_needs_an_overshoot():
    reduction = SPACING["F"] - SPACING["C"]      # positive: the gap closes
    assert equal_speed_junction(CRUISE, reduction, 300.0) > CRUISE


def test_ks2_closes_the_required_displacement_and_stays_continuous():
    reduction = SPACING["C"] - SPACING["F"]
    duration = 300.0
    plan = plan_equal_speed_transition(CRUISE, reduction, duration,
                                       accel_limit=1.5, decel_limit=2.0)
    assert plan["feasible"]
    transition = plan["transition"]
    assert transition.displacement() == pytest.approx(CRUISE * duration + reduction)
    assert plan["displacement_m"] == pytest.approx(plan["required_displacement_m"])

    t = np.linspace(0.0, duration, 30001)
    v, a, l = transition.sample(t)
    assert v[0] == pytest.approx(CRUISE)
    assert v[-1] == pytest.approx(CRUISE)
    assert a[0] == pytest.approx(0.0, abs=1e-9)
    assert a[-1] == pytest.approx(0.0, abs=1e-9)
    # Speed and acceleration are continuous across the junction.
    assert np.max(np.abs(np.diff(v))) < 0.05
    assert np.max(np.abs(np.diff(a))) < 1e-3
    # The integrated displacement matches the sampled speed integral.
    assert np.trapezoid(v, t) == pytest.approx(l[-1], rel=1e-6)
    assert l[-1] == pytest.approx(transition.displacement(), rel=1e-9)
    # Opening a gap dips: the minimum speed is the junction.
    assert v.min() == pytest.approx(plan["junction_speed_mps"], abs=1e-6)


def test_unequal_limits_move_the_optimal_split_off_a_half():
    reduction = SPACING["C"] - SPACING["F"]
    alpha, ok = optimal_split(reduction, 300.0, accel_limit=1.5, decel_limit=2.0)
    assert ok
    assert alpha != pytest.approx(0.5)
    # Braking (2.0) is the larger limit, so the decelerating first phase can be
    # the shorter one: alpha = a2/(a1+a2) = 1.5/3.5, which is below one half.
    assert alpha == pytest.approx(1.5 / 3.5)
    assert alpha < 0.5
    equal, ok_equal = optimal_split(reduction, 300.0, accel_limit=2.0, decel_limit=2.0)
    assert ok_equal and equal == pytest.approx(0.5)


def test_balanced_split_equalises_the_two_normalised_demands():
    """Both phases should sit at the same fraction of their own limit."""
    reduction = SPACING["C"] - SPACING["F"]
    duration = 300.0
    plan = plan_equal_speed_transition(CRUISE, reduction, duration,
                                       accel_limit=1.5, decel_limit=2.0)
    decelerating, accelerating = plan["peak_accelerations_mps2"]
    assert decelerating / 2.0 == pytest.approx(accelerating / 1.5)


def test_split_respects_each_phase_limit():
    reduction = SPACING["C"] - SPACING["F"]
    duration = 120.0
    plan = plan_equal_speed_transition(CRUISE, reduction, duration,
                                       accel_limit=1.5, decel_limit=2.0)
    assert plan["feasible"]
    decelerating, accelerating = plan["peak_accelerations_mps2"]
    assert decelerating <= 2.0 + 1e-9
    assert accelerating <= 1.5 + 1e-9


def test_minimum_duration_for_the_policy_change_is_set_by_the_junction():
    reduction = SPACING["C"] - SPACING["F"]      # -2250 m
    report = minimum_duration(reduction, CRUISE, accel_limit=1.5, decel_limit=2.0)
    assert report["binding"] == "junction_floor"
    assert report["duration_s"] == pytest.approx(2 * 2250.0 / CRUISE)
    # The unequal-limit acceleration bound is longer than the equal-limit shortcut.
    assert report["bounds"]["acceleration"] == pytest.approx(
        np.sqrt(3 * 2250.0 * (1 / 1.5 + 1 / 2.0)))
    assert report["bounds"]["acceleration"] > np.sqrt(6 * 2250.0 / 2.0)


def test_equal_limits_reduce_to_the_manuscript_bound():
    report = minimum_duration(-2250.0, CRUISE, accel_limit=2.0, decel_limit=2.0)
    assert report["bounds"]["acceleration"] == pytest.approx(np.sqrt(6 * 2250.0 / 2.0))


def test_jerk_bound_enters_when_specified():
    without = minimum_duration(-2250.0, CRUISE, accel_limit=1.5, decel_limit=2.0)
    with_jerk = minimum_duration(-2250.0, CRUISE, accel_limit=1.5, decel_limit=2.0,
                                 jerk_limit=0.05)
    assert "jerk" not in without["bounds"]
    assert with_jerk["bounds"]["jerk"] == pytest.approx((48 * 2250.0 / 0.05) ** (1 / 3))


def test_a_speed_cap_at_cruise_makes_gap_closing_infeasible():
    """The traffic model clips commands at cruise, so no overshoot is available."""
    reduction = SPACING["F"] - SPACING["C"]      # closing needs an overshoot
    report = minimum_duration(reduction, CRUISE, accel_limit=1.5, decel_limit=2.0,
                              speed_max=CRUISE)
    assert report["binding"] == "junction_ceiling"
    assert not np.isfinite(report["duration_s"])
    plan = plan_equal_speed_transition(CRUISE, reduction, 300.0, accel_limit=1.5,
                                       decel_limit=2.0, speed_max=CRUISE)
    assert not plan["feasible"]
    assert "junction speed exceeds the maximum speed" in plan["reasons"]


def test_too_short_a_duration_is_refused_with_a_reason():
    reduction = SPACING["C"] - SPACING["F"]
    plan = plan_equal_speed_transition(CRUISE, reduction, 60.0,
                                       accel_limit=1.5, decel_limit=2.0)
    assert not plan["feasible"]
    assert "junction speed is negative" in plan["reasons"]


def test_reference_gap_tracks_the_leader():
    reduction = SPACING["C"] - SPACING["F"]
    duration = 300.0
    plan = plan_equal_speed_transition(CRUISE, reduction, duration,
                                       accel_limit=1.5, decel_limit=2.0)
    t = np.linspace(0.0, duration, 601)
    _, _, l = plan["transition"].sample(t)
    leader = CRUISE * t
    s_ref = reference_gap(l, SPACING["C"], leader)
    assert s_ref[0] == pytest.approx(SPACING["C"])
    # The plan ends exactly on the new target: opening by |dS_real|.
    assert s_ref[-1] == pytest.approx(SPACING["F"])
    assert np.all(np.diff(s_ref) >= -1e-9)      # the gap only opens


def test_tracking_command_reduces_to_feedforward_with_no_error():
    command = tracking_command(planned_accel=-0.25, gap=1500.0,
                               reference_gap_value=1500.0, planned_speed=48.0,
                               follower_speed=48.0, gain_gap=0.005, gain_speed=0.2)
    assert command == pytest.approx(-0.25)
    # A gap larger than planned asks for more acceleration.
    assert tracking_command(-0.25, 1600.0, 1500.0, 48.0, 48.0, 0.005, 0.2) > -0.25
    # A follower slower than planned also asks for more acceleration.
    assert tracking_command(-0.25, 1500.0, 1500.0, 48.0, 47.0, 0.005, 0.2) > -0.25


def test_validation_of_inputs():
    with pytest.raises(ValueError):
        KS2Transition(50.0, 50.0, 40.0, -1.0, 0.5)
    with pytest.raises(ValueError):
        KS2Transition(50.0, 50.0, 40.0, 100.0, 0.0)
    with pytest.raises(ValueError):
        equal_speed_junction(50.0, -2250.0, 0.0)
    with pytest.raises(ValueError):
        minimum_duration(-2250.0, CRUISE, 1.5, 2.0, jerk_limit=0.0)
