from dataclasses import replace

import numpy as np
import pytest

from capacity_policy.geometry import Corridor
from uam_simulator.lateral_study import LateralConfig, SmoothCorridorFrame, maneuver_duration
from uam_simulator.motion_envelope import (
    AccelerationEnvelope, corridor_turning_report, envelope_profile,
    feasible_duration, frame_accelerations, offset_acceleration_budget,
    quintic_duration_bounds, turning_load,
)


def curved_frame(amplitude=100., span=4000., points=17, control=250.):
    x = np.linspace(0, span, points)
    return SmoothCorridorFrame(
        Corridor(np.c_[x, amplitude * np.sin(x / 1000)], "LOCAL_METRIC"), control)


def test_envelope_is_asymmetric_in_the_longitudinal_axis():
    env = AccelerationEnvelope(longitudinal_accel_max_mps2=1.5,
                               longitudinal_decel_max_mps2=2.0,
                               lateral_accel_max_mps2=0.981)
    assert env.utilisation(1.5, 0.0) == pytest.approx(1.0)
    # The same magnitude of braking is admissible because the decel limit is larger.
    assert env.utilisation(-1.5, 0.0) == pytest.approx((1.5 / 2.0) ** 2)
    assert env.admissible(-2.0, 0.0)
    assert not env.admissible(-2.0 - 1e-3, 0.0)
    assert env.utilisation(0.0, 0.981) == pytest.approx(1.0)
    # Axes combine, so full use of one leaves nothing for the other.
    assert not env.admissible(1.5, 0.981)


def test_envelope_utilisation_accepts_a_vertical_component():
    env = AccelerationEnvelope(vertical_accel_max_mps2=0.2)
    assert env.utilisation(0.0, 0.0, 0.2) == pytest.approx(1.0)
    assert not env.admissible(0.0, 0.981, 0.2)


def test_frame_accelerations_match_the_existing_kinematics():
    frame = curved_frame()
    for q in (555., 1234., 2600.):
        _, _, _, _, curvature = frame.frame(np.array([q]))
        expected = frame.kinematics(q, 30., 2., .25, 50.)[1]
        tangential, normal = frame_accelerations(curvature, 30., 2., .25, 50.)
        _, tangent, normal_vector, _, _ = frame.frame(np.array([q]))
        rebuilt = tangential[..., None] * tangent + normal[..., None] * normal_vector
        np.testing.assert_allclose(np.ravel(rebuilt), np.ravel(expected), atol=1e-9)


def test_frame_accelerations_add_the_longitudinal_command():
    tangential, normal = frame_accelerations(0.0, 0.0, 0.0, 0.0, 50.0,
                                             longitudinal_command=-1.25)
    assert tangential == pytest.approx(-1.25)
    assert normal == pytest.approx(0.0)


def test_straight_corridor_has_no_turning_load_and_keeps_the_closed_form():
    frame = SmoothCorridorFrame(Corridor.straight(20000), 500)
    cfg = LateralConfig()
    env = AccelerationEnvelope(lateral_accel_max_mps2=cfg.lateral_accel_limit_mps2)
    assert float(np.ravel(turning_load(frame, np.array([1000.]), 0., cfg.cruise_mps))[0]) == pytest.approx(0.)
    report = feasible_duration(frame, cfg, env, q0=1000., t0=0., source=0., target=100.)
    assert report["status"] == "baseline_admissible"
    # Mirroring the config limit reproduces the archived closed-form duration.
    assert report["duration_s"] == pytest.approx(maneuver_duration(100., cfg))
    # Raising the envelope limit shortens the maneuver, which is the point of
    # making the envelope the authority on the lateral limits.
    faster = feasible_duration(frame, cfg, AccelerationEnvelope(lateral_accel_max_mps2=0.981),
                               q0=1000., t0=0., source=0., target=100.)
    assert faster["duration_s"] < report["duration_s"]
    assert faster["duration_s"] == pytest.approx(cfg.min_maneuver_s)


def test_quintic_bounds_reproduce_the_archived_hundred_metre_duration():
    assert quintic_duration_bounds(100, 8., .3, 30.) == pytest.approx(43.869133, abs=1e-5)
    assert quintic_duration_bounds(0, 8., .3, 30.) == 0.0
    # The floor binds only for very small displacements.
    assert quintic_duration_bounds(10, 8., .3, 30.) == pytest.approx(30.)


def test_turning_load_consumes_the_lateral_budget_only_in_total_mode():
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    env = AccelerationEnvelope(lateral_accel_max_mps2=0.981, coupling_mode="total")
    q = np.linspace(200., 3800., 400)
    load = np.abs(turning_load(frame, q, 0., cfg.cruise_mps))
    budget = offset_acceleration_budget(frame, q, 0., cfg.cruise_mps, env)
    assert load.max() > env.lateral_accel_max_mps2
    # Where the route alone exceeds the limit no lateral maneuver is admissible.
    assert np.all(budget[load >= env.lateral_accel_max_mps2] == 0.0)
    inside = load < env.lateral_accel_max_mps2
    np.testing.assert_allclose(budget[inside],
                               env.lateral_accel_max_mps2 - load[inside], atol=1e-9)


def test_maneuver_only_mode_leaves_the_full_lateral_budget_in_a_curve():
    """The steady turn is flown by banking, so it does not draw on d_ddot."""
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    env = AccelerationEnvelope(lateral_accel_max_mps2=0.981)
    assert env.coupling_mode == "maneuver_only"
    q = np.linspace(200., 3800., 400)
    load = np.abs(turning_load(frame, q, 0., cfg.cruise_mps))
    budget = offset_acceleration_budget(frame, q, 0., cfg.cruise_mps, env)
    assert load.max() > env.lateral_accel_max_mps2
    np.testing.assert_allclose(budget, env.lateral_accel_max_mps2, atol=1e-12)


def test_binding_normal_selects_the_quantity_per_mode():
    maneuver = AccelerationEnvelope()
    total = AccelerationEnvelope(coupling_mode="total")
    assert float(maneuver.binding_normal(0.2, 2.5)) == pytest.approx(0.2)
    assert float(total.binding_normal(0.2, 2.5)) == pytest.approx(2.7)


def test_route_allowance_is_reported_as_a_bank_angle():
    env = AccelerationEnvelope(route_normal_accel_max_mps2=4.575)
    assert env.route_bank_angle_max_deg == pytest.approx(25.0, abs=0.02)
    assert bool(env.route_admissible(2.561))
    assert not bool(env.route_admissible(5.0))


def test_unknown_coupling_mode_is_rejected():
    with pytest.raises(ValueError):
        AccelerationEnvelope(coupling_mode="whatever")


def test_a_longitudinal_command_reduces_the_lateral_budget():
    frame = SmoothCorridorFrame(Corridor.straight(20000), 500)
    cfg = LateralConfig()
    env = AccelerationEnvelope(longitudinal_decel_max_mps2=2.0, lateral_accel_max_mps2=0.981)
    q = np.array([1000.])
    free = offset_acceleration_budget(frame, q, 0., cfg.cruise_mps, env)
    braking = offset_acceleration_budget(frame, q, 0., cfg.cruise_mps, env,
                                         longitudinal_command=-1.0)
    assert float(free[0]) == pytest.approx(env.lateral_accel_max_mps2)
    assert float(braking[0]) == pytest.approx(0.981 * np.sqrt(1 - (1.0 / 2.0) ** 2))
    assert float(braking[0]) < float(free[0])
    # Saturated braking leaves no lateral authority at all.
    none_left = offset_acceleration_budget(frame, q, 0., cfg.cruise_mps, env,
                                           longitudinal_command=-2.0)
    assert float(none_left[0]) == pytest.approx(0.0)


def test_envelope_profile_separates_the_route_and_maneuver_normal_terms():
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    duration = maneuver_duration(100., cfg)
    kwargs = dict(q0=600., t0=0., source=0., target=100., duration=duration,
                  min_samples=101, max_sample_step_s=1.0)
    total = envelope_profile(frame, cfg,
                             AccelerationEnvelope(lateral_accel_max_mps2=0.981,
                                                  coupling_mode="total"), **kwargs)
    maneuver = envelope_profile(frame, cfg,
                                AccelerationEnvelope(lateral_accel_max_mps2=0.981),
                                **kwargs)
    assert total["samples"] >= 101 and len(total["profile"]) == 101
    assert total["sampled_check_not_continuous_proof"] is True
    # Reading the limit as a total makes this maneuver inadmissible; reading it
    # as a maneuver-only limit does not.
    assert total["max_utilisation"] > 1.0 and total["envelope_admissible"] is False
    assert maneuver["envelope_admissible"] is True
    assert maneuver["max_utilisation"] < total["max_utilisation"]
    for row in maneuver["profile"]:
        assert row["total_normal_mps2"] == pytest.approx(
            row["route_normal_mps2"] + row["maneuver_normal_mps2"])
    assert maneuver["max_route_bank_angle_deg"] > 0


def test_feasible_duration_depends_on_the_coupling_mode():
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    env = AccelerationEnvelope(lateral_accel_max_mps2=cfg.lateral_accel_limit_mps2)
    baseline = maneuver_duration(100., cfg)
    kwargs = dict(q0=600., t0=0., source=0., target=100., min_samples=61)

    # Maneuver-only: the steady turn is banked, so the closed-form bound stands.
    maneuver = feasible_duration(frame, cfg, env, **kwargs)
    assert maneuver["baseline_duration_s"] == pytest.approx(baseline)
    assert maneuver["status"] == "baseline_admissible"
    assert maneuver["duration_s"] == pytest.approx(baseline)

    # Total: the same maneuver must be extended or is blocked by the route.
    total = feasible_duration(
        frame, cfg, AccelerationEnvelope(lateral_accel_max_mps2=cfg.lateral_accel_limit_mps2,
                                         coupling_mode="total"), **kwargs)
    assert total["minimality_proven"] is False
    if total["status"] == "extended_for_envelope":
        assert total["duration_s"] > baseline
        assert total["max_utilisation"] <= 1.0 + 1e-6
    else:
        assert total["status"] in {"route_turning_exceeds_limit",
                                   "route_bank_exceeds_allowance",
                                   "no_admissible_duration_within_cap",
                                   "insufficient_remaining_corridor"}
        assert total["duration_s"] is None


def test_a_route_bank_beyond_the_allowance_blocks_the_maneuver_in_either_mode():
    """A tight route is rejected even when the limit binds only the maneuver."""
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    env = AccelerationEnvelope(lateral_accel_max_mps2=0.981,
                               route_normal_accel_max_mps2=0.05)
    report = feasible_duration(frame, cfg, env, q0=600., t0=0., source=0.,
                               target=100., min_samples=61)
    assert report["duration_s"] is None
    assert report["status"] == "route_bank_exceeds_allowance"
    assert report["max_route_bank_angle_deg"] > np.degrees(np.arctan(0.05 / 9.81))


def test_feasible_duration_returns_zero_for_a_stay_candidate():
    frame = SmoothCorridorFrame(Corridor.straight(5000), 500)
    cfg = LateralConfig()
    report = feasible_duration(frame, cfg, AccelerationEnvelope(),
                               q0=100., t0=0., source=50., target=50.)
    assert report["status"] == "stay" and report["duration_s"] == 0.0


def test_corridor_turning_report_scales_with_the_offset():
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    env = AccelerationEnvelope(lateral_accel_max_mps2=0.981)
    report = corridor_turning_report(frame, cfg, env, [-200., 0., 200.], samples=2001)
    assert report["minimum_turn_radius_m"] > 0
    assert report["route_bank_angle_max_deg"] == pytest.approx(25.0, abs=0.02)
    rows = {row["offset_m"]: row for row in report["rows"]}
    # 1 - kappa d scales the turning term, so the two signed offsets differ.
    assert rows[200.]["max_turning_normal_mps2"] != pytest.approx(
        rows[-200.]["max_turning_normal_mps2"])
    assert all(0.0 <= row["fraction_over_lateral_limit"] <= 1.0 for row in report["rows"])


def test_envelope_rejects_nonpositive_limits():
    with pytest.raises(ValueError):
        AccelerationEnvelope(lateral_accel_max_mps2=0.0)
    with pytest.raises(ValueError):
        AccelerationEnvelope(longitudinal_decel_max_mps2=-1.0)


def test_provenance_marks_every_default():
    env = AccelerationEnvelope()
    for name in ("longitudinal_accel_max_mps2", "longitudinal_decel_max_mps2",
                 "lateral_accel_max_mps2", "route_normal_accel_max_mps2",
                 "lateral_speed_max_mps", "min_maneuver_s",
                 "vertical_accel_max_mps2"):
        assert name in env.provenance
        assert env.provenance[name].startswith(("placeholder", "advised", "unset"))
    # The lateral entry records why it is read as a maneuver limit.
    assert "lane change" in env.provenance["lateral_accel_max_mps2"]


def test_quintic_peak_factors_and_their_locations():
    from uam_simulator.motion_envelope import (
        QUINTIC_PEAK_ACCEL_FACTOR, QUINTIC_PEAK_JERK_FACTOR,
        QUINTIC_PEAK_SPEED_FACTOR, quintic_peak_demands,
    )
    u = np.linspace(0., 1., 400001)
    speed = 30 * u ** 2 * (1 - u) ** 2
    accel = 60 * u - 180 * u ** 2 + 120 * u ** 3
    jerk = 60 - 360 * u + 360 * u ** 2
    assert np.max(np.abs(speed)) == pytest.approx(QUINTIC_PEAK_SPEED_FACTOR, abs=1e-6)
    assert np.max(np.abs(accel)) == pytest.approx(QUINTIC_PEAK_ACCEL_FACTOR, abs=1e-5)
    assert np.max(np.abs(jerk)) == pytest.approx(QUINTIC_PEAK_JERK_FACTOR, abs=1e-9)
    # The acceleration peak is at the roots of u^2 - u + 1/6, not the quarters.
    assert float(u[np.argmax(np.abs(accel))]) == pytest.approx(0.788675, abs=1e-4)
    # The jerk peak is at an endpoint, so it steps from zero.
    assert abs(jerk[0]) == pytest.approx(QUINTIC_PEAK_JERK_FACTOR)
    demands = quintic_peak_demands(100., 43.869133)
    assert demands["peak_lateral_speed_mps"] == pytest.approx(4.274076, abs=1e-5)
    assert demands["peak_lateral_accel_mps2"] == pytest.approx(0.3, abs=1e-6)
    assert demands["peak_lateral_jerk_mps3"] == pytest.approx(0.071068, abs=1e-5)
    assert demands["endpoint_jerk_step_mps3"] == demands["peak_lateral_jerk_mps3"]
    assert demands["peak_jerk_at_u"] == (0.0, 1.0)
    assert demands["profile_is_c2_not_c3"] is True


def test_jerk_bound_extends_the_duration_only_when_it_binds():
    unbounded = quintic_duration_bounds(100, 8., .3, 30.)
    assert quintic_duration_bounds(100, 8., .3, 30., 0.1) == pytest.approx(unbounded)
    tight = quintic_duration_bounds(100, 8., .3, 30., 0.05)
    assert tight == pytest.approx((60 * 100 / 0.05) ** (1 / 3))
    assert tight > unbounded
    with pytest.raises(ValueError):
        quintic_duration_bounds(100, 8., .3, 30., 0.0)


def test_envelope_jerk_limit_is_optional_and_validated():
    assert AccelerationEnvelope().lateral_jerk_max_mps3 is None
    assert AccelerationEnvelope(lateral_jerk_max_mps3=0.05).lateral_jerk_max_mps3 == 0.05
    with pytest.raises(ValueError):
        AccelerationEnvelope(lateral_jerk_max_mps3=-1.0)
    assert "lateral_jerk_max_mps3" in AccelerationEnvelope().provenance


def test_feasible_duration_reports_when_jerk_sets_the_baseline():
    frame = SmoothCorridorFrame(Corridor.straight(30000), 500)
    cfg = LateralConfig()
    pinned = dict(lateral_accel_max_mps2=cfg.lateral_accel_limit_mps2)
    loose = feasible_duration(frame, cfg,
                              AccelerationEnvelope(lateral_jerk_max_mps3=0.1, **pinned),
                              q0=1000., t0=0., source=0., target=100.)
    tight = feasible_duration(frame, cfg,
                              AccelerationEnvelope(lateral_jerk_max_mps3=0.05, **pinned),
                              q0=1000., t0=0., source=0., target=100.)
    assert loose["jerk_limit_binds_baseline"] is False
    assert loose["duration_s"] == pytest.approx(maneuver_duration(100., cfg))
    assert tight["jerk_limit_binds_baseline"] is True
    assert tight["duration_s"] == pytest.approx((60 * 100 / 0.05) ** (1 / 3))
    assert tight["peak_demands"]["peak_lateral_jerk_mps3"] == pytest.approx(0.05, rel=1e-9)


def test_forecast_candidates_without_an_envelope_is_byte_for_byte_unchanged():
    """The opt-in must not perturb the archived path."""
    from types import SimpleNamespace
    from capacity_policy.base_stations import BaseStation, BaseStationSet
    from capacity_policy.radio import RadioConfig
    from uam_simulator.lateral_study import forecast_candidates
    from uam_simulator.policy_motion import BSRadio

    frame = SmoothCorridorFrame(Corridor.straight(6000), 200)
    cfg = replace(LateralConfig(cruise_mps=10, altitude_m=30, dt_s=.5, policy_s=1,
                                decision_s=2, forecast_dt_s=.5, horizon_s=20,
                                window_s=3, cooldown_s=2, min_maneuver_s=2,
                                lateral_speed_limit_mps=10, lateral_accel_limit_mps2=10,
                                center_control_step_m=200, spatial_sample_m=10))
    scenario = SimpleNamespace(
        base_stations=BaseStationSet((BaseStation("b", 3000, 200, 10),), "LOCAL_METRIC"),
        radio=RadioConfig(served_set_size=1))
    radio = BSRadio(scenario.base_stations, scenario.radio)
    kwargs = dict(q=100., t=0., source=0., targets=[100., -100.], history=[(0., 0)])
    assert (forecast_candidates(frame, radio, cfg, **kwargs)
            == forecast_candidates(frame, radio, cfg, envelope=None, **kwargs))


def test_envelope_durations_reject_what_the_route_cannot_accommodate():
    from uam_simulator.lateral_study import envelope_durations
    frame = curved_frame(amplitude=400.)
    cfg = LateralConfig()
    # A route allowance this tight cannot be met anywhere on a curved corridor.
    env = AccelerationEnvelope(route_normal_accel_max_mps2=0.05,
                               lateral_accel_max_mps2=cfg.lateral_accel_limit_mps2)
    durations, rejected = envelope_durations(frame, cfg, env, q=600., t=0.,
                                             source=0., targets=[0., 100.])
    assert rejected[0] is None            # staying is never rejected
    assert durations[0] == 0.0
    assert rejected[1] == "route_bank_exceeds_allowance"
    # A rejected candidate keeps a usable duration so the forecast stays sane.
    assert durations[1] == pytest.approx(maneuver_duration(100., cfg))


def test_envelope_durations_pass_through_a_straight_corridor():
    from uam_simulator.lateral_study import envelope_durations
    frame = SmoothCorridorFrame(Corridor.straight(30000), 500)
    cfg = LateralConfig()
    durations, rejected = envelope_durations(
        frame, cfg, AccelerationEnvelope(lateral_accel_max_mps2=cfg.lateral_accel_limit_mps2),
        q=1000., t=0., source=0., targets=[0., 100., 200.])
    assert rejected == [None, None, None]
    assert durations[1] == pytest.approx(maneuver_duration(100., cfg))
    assert durations[2] == pytest.approx(maneuver_duration(200., cfg))
