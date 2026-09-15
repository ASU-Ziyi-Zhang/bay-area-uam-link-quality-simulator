import math

import numpy as np
import pytest

from uam_simulator.five_uam_lateral import (
    Action, AdmissibleSet, LateralProfile, Proposal, admissible,
    envelope_utilisation, minimum_duration, quintic, select,
)
from uam_simulator.motion_envelope import AccelerationEnvelope

ENVELOPE = AccelerationEnvelope(lateral_accel_max_mps2=0.981,
                                lateral_speed_max_mps=8.0)


def test_quintic_endpoint_conditions():
    """Zero transverse speed and acceleration at both ends; s(1) is one, not zero."""
    s0, d1_0, d2_0, d3_0 = (float(x) for x in quintic(0.0))
    s1, d1_1, d2_1, d3_1 = (float(x) for x in quintic(1.0))
    assert (s0, s1) == pytest.approx((0.0, 1.0))
    assert (d1_0, d1_1) == pytest.approx((0.0, 0.0))
    assert (d2_0, d2_1) == pytest.approx((0.0, 0.0))
    # C2 but not C3: the jerk is bounded and equal at both ends, not zero.
    assert (d3_0, d3_1) == pytest.approx((60.0, 60.0))


def test_sampled_peaks_match_the_closed_form():
    profile = LateralProfile(start_s=10.0, duration_s=40.0, distance_m=100.0)
    t = np.linspace(0.0, 60.0, 600001)
    _, speed, accel, _ = profile.sample(t)
    peaks = profile.peaks()
    assert np.abs(speed).max() == pytest.approx(peaks["speed_mps"], rel=1e-9)
    assert np.abs(accel).max() == pytest.approx(peaks["accel_mps2"], rel=1e-6)


def test_profile_is_flat_outside_the_maneuver():
    profile = LateralProfile(start_s=10.0, duration_s=20.0, distance_m=-100.0)
    offset, speed, accel, _ = profile.sample(np.array([0.0, 5.0, 10.0, 30.0, 45.0]))
    assert offset[:3] == pytest.approx(0.0)
    assert offset[3:] == pytest.approx(-100.0)
    assert speed == pytest.approx(0.0)
    assert accel == pytest.approx(0.0)


def test_minimum_duration_reports_which_bound_binds():
    report = minimum_duration(100.0, ENVELOPE)
    # 1.875 * 100 / 8 = 23.4375 s from the speed bound;
    # sqrt(5.7735 * 100 / 0.981) = 24.26 s from the acceleration bound.
    assert report["binding"] == "lateral_accel"
    assert report["duration_s"] == pytest.approx(
        math.sqrt(10 / math.sqrt(3) * 100.0 / 0.981))
    assert report["bounds"]["lateral_speed"] == pytest.approx(1.875 * 100.0 / 8.0)
    # A profile at exactly the minimum sits on its binding limit.
    profile = LateralProfile(0.0, report["duration_s"], 100.0)
    assert profile.peaks()["accel_mps2"] == pytest.approx(0.981)


def test_envelope_utilisation_uses_the_signed_longitudinal_limit():
    # Braking is bounded at 2.0 and acceleration at 1.5, so the same magnitude
    # of longitudinal demand costs more of the budget when accelerating.
    accelerating = envelope_utilisation(1.5, 0.0, ENVELOPE)
    braking = envelope_utilisation(-1.5, 0.0, ENVELOPE)
    assert accelerating == pytest.approx(1.0)
    assert braking == pytest.approx((1.5 / 2.0) ** 2)


def _accept_all(proposal):
    return True, (), 0.5, 900.0


def test_admissible_set_cannot_be_built_directly_or_hold_a_rejection():
    with pytest.raises(TypeError):
        AdmissibleSet(object(), [])
    stay = Proposal(Action.STAY)
    admitted, log = admissible([stay], _accept_all, envelope=ENVELOPE,
                               volume=None, horizon_s=300.0)
    assert len(admitted) == 1 and len(log) == 1


def test_select_refuses_anything_that_did_not_come_from_the_filter():
    """The defect this guards against is ranking before filtering."""
    with pytest.raises(TypeError):
        select([Proposal(Action.STAY)], 100.0, lambda p: 0.0,
               now_s=0.0, delta_num=0.0, delta_pred=0.0)


def test_a_cheaper_inadmissible_candidate_is_never_selected():
    """The mixed-candidate case: both set-level predicates read 'yes'.

    Stay costs 100, an admissible change costs 90, and an inadmissible change
    costs 0. A scenario with no admissible candidate at all does not expose an
    implementation that ranks before filtering, because there the gain
    predicate is false. Here it is true, so only the ordering of filter and
    rank decides the outcome.
    """
    good = Proposal(Action.CHANGE, LateralProfile(0.0, 40.0, 100.0), label="A")
    bad = Proposal(Action.CHANGE, LateralProfile(0.0, 5.0, 100.0), label="C")
    cost = {"": 100.0, "A": 90.0, "C": 0.0}

    def check(proposal):
        if proposal.label == "C":
            return False, ("gap_insufficient",), 3.2, 40.0
        return True, (), 0.4, 900.0

    admitted, log = admissible([good, bad], check, envelope=ENVELOPE,
                               volume=None, horizon_s=300.0)
    assert len(admitted) == 1
    assert [v.admitted for v in log] == [True, False]

    decision = select(admitted, cost[""], lambda p: cost[p.label],
                      now_s=0.0, delta_num=1.0, delta_pred=1.0)
    assert decision["action"] is Action.CHANGE
    assert decision["chosen"].label == "A"      # not B, whose cost is lower
    assert decision["best_gain"] == pytest.approx(10.0)


def test_no_admissible_candidate_yields_stay_with_a_distinct_reason():
    bad = Proposal(Action.CHANGE, LateralProfile(0.0, 5.0, 100.0), label="C")
    admitted, log = admissible(
        [bad], lambda p: (False, ("envelope_exceeded",), 3.2, 900.0),
        envelope=ENVELOPE, volume=None, horizon_s=300.0)
    assert len(admitted) == 0 and log[0].reasons == ("envelope_exceeded",)
    decision = select(admitted, 100.0, lambda p: 0.0,
                      now_s=0.0, delta_num=1.0, delta_pred=1.0)
    assert decision["action"] is Action.STAY
    assert decision["reason"] == "no_admissible_candidate"


def test_gain_gate_uses_the_sum_of_both_error_bands():
    """0.8 of gain must fail against 0.5 + 0.5, not pass against each of them."""
    good = Proposal(Action.CHANGE, LateralProfile(0.0, 40.0, 100.0), label="A")
    admitted, _ = admissible([good], _accept_all, envelope=ENVELOPE,
                             volume=None, horizon_s=300.0)
    decision = select(admitted, 100.0, lambda p: 99.2,
                      now_s=0.0, delta_num=0.5, delta_pred=0.5)
    assert decision["action"] is Action.STAY
    assert decision["threshold"] == pytest.approx(1.0)
    # The same gain passes once the bands are small enough.
    assert select(admitted, 100.0, lambda p: 99.2, now_s=0.0,
                  delta_num=0.3, delta_pred=0.3)["action"] is Action.CHANGE


def test_a_future_start_yields_stay_now_and_commits_to_nothing():
    later = Proposal(Action.CHANGE, LateralProfile(120.0, 40.0, 100.0), label="A")
    admitted, _ = admissible([later], _accept_all, envelope=ENVELOPE,
                             volume=None, horizon_s=300.0)
    decision = select(admitted, 100.0, lambda p: 50.0,
                      now_s=0.0, delta_num=1.0, delta_pred=1.0)
    assert decision["action"] is Action.STAY
    assert decision["reason"] == "future_start"
    assert decision["committed"] is False
    # The same proposal is a lane change once its start time has arrived.
    assert select(admitted, 100.0, lambda p: 50.0, now_s=120.0,
                  delta_num=1.0, delta_pred=1.0)["action"] is Action.CHANGE


def test_the_two_stay_reasons_are_distinguishable():
    """S1 'cannot' and S2 'not worth it' must never be recorded identically."""
    empty, _ = admissible([], _accept_all, envelope=ENVELOPE,
                          volume=None, horizon_s=300.0)
    cannot = select(empty, 100.0, lambda p: 0.0, now_s=0.0,
                    delta_num=0.0, delta_pred=0.0)
    good = Proposal(Action.CHANGE, LateralProfile(0.0, 40.0, 100.0), label="A")
    admitted, _ = admissible([good], _accept_all, envelope=ENVELOPE,
                             volume=None, horizon_s=300.0)
    not_worth = select(admitted, 100.0, lambda p: 100.0, now_s=0.0,
                       delta_num=0.0, delta_pred=0.0)
    assert cannot["action"] is not_worth["action"] is Action.STAY
    assert cannot["reason"] == "no_admissible_candidate"
    assert not_worth["reason"] == "gain_below_threshold"

# Real trajectories, real admission and real selection. These volumes are
# synthetic test fixtures, not a proposed operational separation standard.
from dataclasses import replace
from copy import deepcopy
from uam_simulator.five_uam_lateral import (
    Scenario, WeakZone, rollout, admissibility_check, policy_cost,
    run_closed_loop, initial_state, AdmissionCheck,
)
from uam_simulator.geographic_traffic import GeographicTrafficConfig
from uam_simulator.safety import SeparationVolume

CFG = GeographicTrafficConfig(cruise_mps=50., speed_min_mps=30., speed_max_mps=80.,
                              policy_s=1., decision_s=5.)
VOLUME = SeparationVolume(horizontal_m=50., vertical_m=30.,
                          label="test_admission", provenance="synthetic regression fixture")


def scene(**changes):
    values = dict(ego_q_m=0., gap_to_leader_m=1367.5, gap_from_follower_m=None,
                  target_lane_front_offset_m=None, target_lane_rear_offset_m=None)
    return Scenario(**(values | changes))


def check_run(s, p=None, *, horizon=45., dt=.5, envelope=ENVELOPE, **options):
    r = rollout(s, CFG, p, horizon_s=horizon, dt=dt, envelope=envelope, **options)
    return r, admissibility_check(r, s, envelope, VOLUME)


def test_actual_admission_connects_to_filter_and_selection():
    s = scene()
    p = Proposal(Action.CHANGE, LateralProfile(0, 40, 100))
    result, checked = check_run(s, p.profile)
    assert isinstance(checked, AdmissionCheck)
    assert checked.admitted, checked.reasons
    accepted, _ = admissible([p], lambda _: checked, envelope=ENVELOPE,
                            volume=VOLUME, horizon_s=45)
    decision = select(accepted, policy_cost(result)["total"],
                      lambda _: policy_cost(result)["total"],
                      now_s=0, delta_num=0, delta_pred=0)
    assert decision["reason"] == "gain_below_threshold"


def test_nmac_is_reported_separately_and_cannot_be_the_default_gate():
    s = scene(target_lane_front_offset_m=10000., target_lane_rear_offset_m=1367.5,
              gap_from_follower_m=1367.5)
    r = rollout(s, CFG, None, horizon_s=1., dt=.25)
    with pytest.raises(ValueError, match="admission volume"):
        admissibility_check(r, s, ENVELOPE, SeparationVolume())
    checked = admissibility_check(r, s, ENVELOPE, VOLUME)
    assert checked.admitted, checked.reasons
    assert checked.screen["nmac_diagnostic"]["violation_count"] > 0
    assert checked.screen["violation_count"] == 0


def test_total_speed_and_declared_jerk_are_actually_rejected():
    s = scene(gap_to_leader_m=10000., neighbour_speeds={"A": 80.})
    r, checked = check_run(s, LateralProfile(0, 40, 100), control_mode="feedback")
    assert np.hypot(r["trace"]["A"]["v_mps"], r["lateral"]["speed_mps"]).max() > 80
    assert not checked.admitted
    assert "A_total_speed_exceeded" in checked.reasons
    env = replace(ENVELOPE, lateral_jerk_max_mps3=.01)
    _, checked = check_run(scene(), LateralProfile(0, 40, 100), envelope=env)
    assert not checked.admitted
    assert "lateral_jerk_exceeded" in checked.reasons


def test_unseen_and_unfinished_maneuvers_cannot_pass():
    _, checked = check_run(scene(), LateralProfile(100, 1, 100), horizon=10)
    assert not checked.admitted
    assert "prediction_does_not_cover_maneuver_and_post_state" in checked.reasons
    _, checked = check_run(scene(), LateralProfile(0, 40, 100), horizon=40)
    assert not checked.admitted
    assert "prediction_does_not_cover_maneuver_and_post_state" in checked.reasons


def test_new_insertion_shortfall_is_rejected_but_existing_deficit_is_reported():
    s = scene(gap_to_leader_m=10000., target_lane_front_offset_m=500.)
    _, checked = check_run(s, LateralProfile(0, 40, 100))
    assert not checked.admitted
    assert any("insertion_gap" in r for r in checked.reasons)
    s = scene(gap_to_leader_m=600., neighbour_speeds={"A": 30., "B": 30.})
    _, checked = check_run(s, horizon=5.)
    assert checked.admitted, checked.reasons
    assert checked.screen["policy_deficits"]
    assert not any(x["new_relationship"] for x in checked.screen["policy_deficits"])


def test_speed_floor_contact_preserves_position_integral_and_endpoint():
    s = scene(gap_to_leader_m=100., neighbour_speeds={"A":30.1, "B":30.})
    r = rollout(s, CFG, None, horizon_s=1., dt=1., control_mode="feedback")
    assert r["trace"]["A"]["q_m"][-1] == pytest.approx(30.0025)
    assert r["trace"]["A"]["v_mps"][-1] == pytest.approx(30.)
    assert .05 == pytest.approx(r["times_s"][1])
    r = rollout(scene(), CFG, None, horizon_s=1.03, dt=.3)
    assert r["times_s"][-1] == 1.03


def test_aks_is_executed_and_invalidated_when_leader_changes():
    s = scene(zones=(WeakZone(0, -1., 10., "F"),))
    r = rollout(s, CFG, None, horizon_s=1.5, dt=.05)
    assert "ks2_tracker" in r["trace"]["A"]["controller"]
    assert r["trace"]["A"]["v_mps"].min() < 50
    s = scene(leader_braking={"start_s":1., "end_speed_mps":30., "deceleration_mps2":2.})
    r = rollout(s, CFG, None, horizon_s=3., dt=.1)
    assert "hold_tracker" in r["trace"]["A"]["controller"]
    assert "acc_fallback" in r["trace"]["A"]["controller"]
    assert any(e["event"] == "reference_invalidated" for e in r["events"])


def test_relation_switch_does_not_keep_the_old_source_pair():
    s = scene(target_lane_front_offset_m=1500., gap_from_follower_m=1367.5,
              target_lane_rear_offset_m=1500.)
    r = rollout(s, CFG, LateralProfile(0, 40, 100), horizon_s=45, dt=.5)
    assert r["state"].leaders["A"] == ("D",)
    assert ("A", "B") not in r["state"].recovery_pairs
    assert r["state"].leaders["C"] == ("B",)
    assert r["state"].leaders["E"] == ("A",)


def test_no_policy_lane_override_and_no_premature_target_policy():
    s = scene(zones=(WeakZone(0, -1000, 100000),))
    with pytest.raises(ValueError, match="override removed"):
        rollout(s, CFG, None, horizon_s=1, dt=.1, ego_policy_lane=1)
    r = rollout(s, CFG, LateralProfile(0, 40, 100), horizon_s=41, dt=.25)
    assert r["trace"]["A"]["policy"][0] == "F"
    assert r["trace"]["A"]["policy"][-1] == "C"
    assert r["policy_mode"] == "prescribed_spatial_policy"


def test_split_execution_preserves_lateral_state_and_controller_history():
    s = scene()
    p = LateralProfile(0, 40, 100)
    full = rollout(s, CFG, p, horizon_s=45, dt=.25)
    first = rollout(s, CFG, p, horizon_s=17, dt=.25)
    saved = deepcopy(first["state"])
    rest = rollout(s, CFG, None, horizon_s=45, dt=.25, state=first["state"])
    for key in ("offset_m", "speed_mps", "accel_mps2"):
        assert rest["lateral"][key][0] == pytest.approx(first["lateral"][key][-1])
        assert rest["lateral"][key][-1] == pytest.approx(full["lateral"][key][-1], abs=1e-9)
    assert first["state"].t_s == saved.t_s
    assert first["state"].q == saved.q
    assert full["lateral"]["offset_m"][-1] == pytest.approx(100, abs=1e-9)
    assert full["lateral"]["speed_mps"][-1] == pytest.approx(0, abs=1e-9)
    assert full["trace"]["A"]["q_m"][-1] == pytest.approx(rest["trace"]["A"]["q_m"][-1])


def test_radio_uses_actual_positions_and_persistent_exposure_history():
    # The measured field improves at y=50. The logical source group/history
    # survives until completion, so a lane flag cannot reset the exposure.
    def radio(xyz):
        return {"sinr_db": np.where(xyz[:, 1] < 50, -20., 20.)}
    cfg = replace(CFG, neighbors_each_side=1, window_s=10.)
    s = scene()
    p = LateralProfile(0, 40, 100)
    full = rollout(s, cfg, p, horizon_s=55, dt=.5, radio=radio)
    first = rollout(s, cfg, p, horizon_s=25, dt=.5, radio=radio)
    second = rollout(s, cfg, None, horizon_s=55, dt=.5, state=first["state"], radio=radio)
    rows = [r for r in full["observations"] if r["role"] == "A"]
    assert rows[0]["policy"] == "F"
    improved_signal = next(row for row in rows if row["sinr_db"] > 0)
    assert improved_signal["exposure"] > 0
    assert rows[-1]["policy"] == "C"
    assert full["state"].history == second["state"].history
    assert full["policy_mode"] == "radio_exposure"
    assert first["state"].t_s == 25


def test_polynomial_interval_audit_catches_between_sample_conflict():
    # Two same-lane aircraft exchange positions between the only two samples.
    # Saved endpoints are both 200 m apart; the actual polynomial crosses zero.
    s = scene(gap_to_leader_m=200.)
    r = rollout(s, CFG, None, horizon_s=1, dt=1)
    r["segments"][0]["xy"]["A"] = ([0., 400.], [0.])
    r["segments"][0]["xy"]["B"] = ([200.], [0.])
    checked = admissibility_check(r, s, ENVELOPE, VOLUME)
    assert not checked.admitted
    assert checked.screen["interval_events"]


def test_nonfinite_and_sample_only_results_cannot_pass():
    s = scene()
    r = rollout(s, CFG, None, horizon_s=1, dt=.5)
    del r["segments"]
    assert "continuous_trajectory_unavailable" in admissibility_check(r, s, ENVELOPE, VOLUME).reasons
    r["trace"]["A"]["v_mps"][0] = np.nan
    assert not admissibility_check(r, s, ENVELOPE, VOLUME).admitted


def test_closed_loop_runs_real_filter_and_commits_a_continuous_maneuver():
    cfg = CFG
    s = scene(gap_from_follower_m=6000., target_lane_front_offset_m=5000.,
              target_lane_rear_offset_m=5000., zones=(WeakZone(0, 100, 10000),))
    run = run_closed_loop(s, cfg, ENVELOPE, VOLUME, horizon_s=65,
                         prediction_s=50, dt=1., duration_factors=(2.,),
                         delta_num=0., delta_pred=0.)
    assert run["status"] == "completed", run["decisions"]
    assert any(d["action"] == "change" for d in run["decisions"])
    assert any(d["reason"] == "continue_committed_maneuver" for d in run["decisions"])
    assert run["decisions"][-1]["reason"] == "target_lane_following"
    assert len(run["state"].q) == 5
    assert run["state"].lane["A"] == 1
    assert run["state"].d_m == pytest.approx(100, abs=1e-8)
    for prev, nxt in zip(run["pieces"], run["pieces"][1:]):
        assert prev["state"].d_m == pytest.approx(nxt["lateral"]["offset_m"][0])
        assert prev["state"].v == {r: float(nxt["trace"][r]["v_mps"][0]) for r in nxt["roles"]}


def test_departure_never_refuses_on_the_pair_left_behind():
    """Only relationships the maneuvering aircraft enters can refuse it.

    C follows A at 1367.5 m inside the weak zone, so under F its shortfall is
    2250 m. When A leaves, C inherits A's leader B at the sum of both gaps and
    the shortfall falls to 723.5 m. Refusing the change on that pair would
    forbid a maneuver that reduces the very shortfall being measured, so it is
    recorded and flown, never refused.
    """
    s = scene(gap_from_follower_m=1367.5, target_lane_front_offset_m=5000.,
              target_lane_rear_offset_m=5000., zones=(WeakZone(0, 100, 10000),))
    run = run_closed_loop(s, CFG, ENVELOPE, VOLUME, horizon_s=65,
                         prediction_s=50, dt=1., duration_factors=(2.,),
                         delta_num=0., delta_pred=0.)
    assert run["decisions"][0]["action"] == "change"
    assert run["status"] == "completed"
    assert not run["safety_events"]
    assert run["state"].lane["A"] == 1
    worst = {}
    for piece in run["pieces"]:
        for row in piece["admission"].screen["policy_deficits"]:
            pair = tuple(row["pair"])
            worst[pair] = max(worst.get(pair, 0.), row["maximum_deficit_m"])
    assert worst[("C", "A")] == pytest.approx(2250., abs=1.)
    assert worst[("C", "B")] == pytest.approx(723.5, abs=1.)
    assert worst[("C", "B")] < worst[("C", "A")]

    # The relationships A itself enters still refuse: a target lane with only
    # 1000 m front and rear cannot be entered under C spacing.
    blocked = run_closed_loop(
        scene(gap_from_follower_m=1367.5, target_lane_front_offset_m=1000.,
              target_lane_rear_offset_m=1000., zones=(WeakZone(0, 100, 10000),)),
        CFG, ENVELOPE, VOLUME, horizon_s=20, prediction_s=50, dt=1.,
        duration_factors=(2.,), delta_num=0., delta_pred=0.)
    first = blocked["decisions"][0]
    assert first["reason"] == "no_admissible_candidate"
    assert any(any(r.startswith("insertion_gap_insufficient") and "A" in r.split(":")
                   for r in c["reasons"]) for c in first["candidates"])


def test_change_holds_speed_and_counts_the_new_lane_from_the_midline():
    """A leaving a lane does not brake for the leader it is leaving.

    The policy lane switches when half the lateral distance is covered, which
    for an asymmetric shape is not half the duration, so the switch is checked
    against the offset itself rather than against a time.
    """
    s = scene(zones=(WeakZone(0, 100, 10000),))
    for shape in (QUINTIC_SHAPE, beta_shape(3, 2)):
        r = rollout(s, CFG, LateralProfile(0, 40, 100, shape), horizon_s=45, dt=.5)
        t = np.asarray(r["times_s"])
        a, d = r["trace"]["A"], np.asarray(r["lateral"]["offset_m"])
        during = (t > 0) & (t < 40)
        assert np.allclose(np.asarray(a["a_mps2"])[during], 0.)
        assert set(np.asarray(a["controller"])[during]) == {"lane_change_hold"}
        assert np.allclose(np.asarray(a["v_mps"])[during], a["v_mps"][0])
        lane = np.asarray(a["lane"], float)
        assert lane[d < 50 - 1e-9].max() == 0
        assert lane[d >= 50 - 1e-9].min() == 1
        # The lane flips at the crossing itself, but the policy is refreshed on
        # the observation clock, so it follows within one observation period.
        cross = 40 * shape.midpoint_u
        refreshed = t >= math.ceil(cross / CFG.policy_s) * CFG.policy_s
        assert set(np.asarray(a["policy"])[refreshed]) == {"C"}


def test_fixed_control_period_separates_the_controller_from_the_step():
    # With a declared period the command is recomputed on the same instants
    # whatever the integration step, so refining the step cannot change the
    # controller; without one, the step size is also the sample period.
    s = scene(gap_to_leader_m=1000., zones=(WeakZone(0, 100, 10000),))
    held = replace(CFG, control_period_s=1.)
    coarse = rollout(s, held, None, horizon_s=30, dt=.5)
    fine = rollout(s, held, None, horizon_s=30, dt=.25)
    assert coarse["trace"]["A"]["q_m"][-1] == pytest.approx(fine["trace"]["A"]["q_m"][-1], abs=1e-9)
    free_coarse = rollout(s, CFG, None, horizon_s=30, dt=.5)
    free_fine = rollout(s, CFG, None, horizon_s=30, dt=.25)
    assert abs(free_coarse["trace"]["A"]["q_m"][-1] - free_fine["trace"]["A"]["q_m"][-1]) > 1e-6


def test_failed_audit_is_recorded_and_the_flight_continues():
    # B sits 40 m ahead, inside this fixture's 50 m protection volume, so
    # staying is itself not admissible. The run must report it and still fly to
    # the horizon: stopping would hide what follows and would force the
    # recorded NMAC count to zero.
    run = run_closed_loop(scene(gap_to_leader_m=40.), CFG, ENVELOPE, VOLUME,
                          horizon_s=20, prediction_s=30, dt=1., duration_factors=(2.,),
                          delta_num=0., delta_pred=0.)
    assert run["status"] == "completed_with_safety_events"
    assert run["safety_events"]
    assert {e["stage"] for e in run["safety_events"]} <= {
        "stay_baseline", "committed_continuation", "executed_step"}
    assert run["state"].t_s == 20
    assert run["pieces"]


def test_declared_duration_floor_and_disconnected_segments_are_rejected():
    s = scene()
    r = rollout(s, CFG, LateralProfile(0, 40, 100), horizon_s=45, dt=1.)
    env = replace(ENVELOPE, min_maneuver_s=50.)
    assert "declared_minimum_duration_not_met" in admissibility_check(r, s, env, VOLUME).reasons
    r["segments"][0]["xy"]["A"][0][0] += 1.
    assert "continuous_trajectory_sample_mismatch" in admissibility_check(r, s, ENVELOPE, VOLUME).reasons


# --- lane-change shapes and the policy-group boundary (step 2 restoration) ---
from uam_simulator.five_uam_lateral import (
    LateralShape, QUINTIC_SHAPE, beta_shape, bezier3_shape, three_clothoid_shape,
)

SHAPES = [QUINTIC_SHAPE, beta_shape(3, 2), beta_shape(2, 3), beta_shape(3, 3),
          three_clothoid_shape(.25), three_clothoid_shape(.15)]


def test_beta_2_2_is_exactly_the_quintic_and_beta_3_3_the_septic():
    """The quintic baseline is a member of the AKS beta-kernel family."""
    assert np.allclose(beta_shape(2, 2).coefficients[0], (0, 0, 0, 10, -15, 6), atol=1e-12)
    assert np.allclose(beta_shape(3, 3).coefficients[0], (0, 0, 0, 0, 35, -84, 70, -20), atol=1e-12)


@pytest.mark.parametrize("shape", SHAPES, ids=lambda s: s.name)
def test_every_shape_joins_straight_flight_with_c2_continuity(shape):
    s, d1, d2, _ = shape.evaluate(np.array([0., 1.]))
    assert s == pytest.approx([0., 1.], abs=1e-12)
    assert d1 == pytest.approx([0., 0.], abs=1e-12)
    assert d2 == pytest.approx([0., 0.], abs=1e-12)
    for b in shape.junctions:
        left, right = shape.evaluate(np.array([b-1e-9])), shape.evaluate(np.array([b+1e-9]))
        for order in range(3):
            assert left[order] == pytest.approx(right[order], abs=1e-6)


@pytest.mark.parametrize("shape", SHAPES, ids=lambda s: s.name)
def test_peak_factors_match_dense_sampling(shape):
    u = np.linspace(0., 1., 400001)
    _, d1, d2, d3 = shape.evaluate(u)
    f = shape.peak_factors
    assert f["speed"] == pytest.approx(np.abs(d1).max(), rel=1e-6)
    assert f["accel"] == pytest.approx(np.abs(d2).max(), rel=1e-6)
    assert f["jerk"] == pytest.approx(np.abs(d3).max(), rel=1e-4)


def test_quintic_factors_are_unchanged():
    f = QUINTIC_SHAPE.peak_factors
    assert (f["speed"], f["accel"], f["jerk"]) == pytest.approx((1.875, 10/np.sqrt(3), 60.))


def test_asymmetric_beta_kernels_shift_the_lateral_motion():
    """p > q moves the motion later, p < q earlier; the quintic is centred."""
    half = lambda shape: float(shape.evaluate(np.array([.5]))[0][0])
    assert half(QUINTIC_SHAPE) == pytest.approx(.5)
    assert half(beta_shape(3, 2)) < .5 < half(beta_shape(2, 3))


def test_clothoid_quarter_split_has_equal_jerk_on_every_piece():
    shape = three_clothoid_shape(.25)
    jerks = [abs(float(shape.polynomial(k).deriv(3)(0.))) for k in range(3)]
    assert jerks[0] == pytest.approx(jerks[1]) == pytest.approx(jerks[2])


def test_minimum_duration_uses_each_shape_factor():
    quintic = minimum_duration(100., ENVELOPE)
    assert minimum_duration(100., ENVELOPE, QUINTIC_SHAPE) == quintic
    septic = minimum_duration(100., ENVELOPE, beta_shape(3, 3))
    # The septic's larger acceleration factor needs a longer manoeuvre.
    assert septic["duration_s"] > quintic["duration_s"]


def test_cubic_bezier_has_zero_speed_but_an_acceleration_step_at_both_ends():
    """The cubic Bezier is the one C1 shape: speed is zero at the ends, acceleration is not."""
    shape = bezier3_shape()
    assert shape.coefficients[0] == pytest.approx((0., 0., 3., -2.))
    s, d1, d2, _ = shape.evaluate(np.array([0., 1.]))
    assert s == pytest.approx([0., 1.], abs=1e-12)
    assert d1 == pytest.approx([0., 0.], abs=1e-12)
    assert d2 == pytest.approx([6., -6.], abs=1e-12)
    f = shape.peak_factors
    assert (f["speed"], f["accel"], f["jerk"]) == pytest.approx((1.5, 6., 12.))


def test_cubic_bezier_executes_to_the_target_lane_and_passes_the_audit():
    test_every_shape_executes_to_the_target_lane_and_passes_the_audit(bezier3_shape())


def test_invalid_shape_parameters_are_refused():
    for bad in ((1, 2), (2, 1), (2.5, 2)):
        with pytest.raises(ValueError):
            beta_shape(*bad)
    for bad in (0., .5, .7):
        with pytest.raises(ValueError):
            three_clothoid_shape(bad)


@pytest.mark.parametrize("shape", SHAPES, ids=lambda s: s.name)
def test_every_shape_executes_to_the_target_lane_and_passes_the_audit(shape):
    profile = LateralProfile(0., 60., 100., shape)
    result, checked = check_run(scene(), profile, horizon=70., dt=.5)
    assert checked.admitted, checked.reasons
    assert result["state"].d_m == pytest.approx(100., abs=1e-8)
    assert result["lateral_execution"] == f"integrated_{shape.name}_jerk"
    executed = np.abs(result["lateral"]["accel_mps2"]).max()
    assert executed == pytest.approx(profile.peaks()["accel_mps2"], rel=2e-2)


def test_closed_loop_compares_shapes_and_records_which_one_it_chose():
    s = scene(gap_from_follower_m=6000., target_lane_front_offset_m=5000.,
              target_lane_rear_offset_m=5000., zones=(WeakZone(0, 100, 10000),))
    run = run_closed_loop(s, CFG, ENVELOPE, VOLUME, horizon_s=10, prediction_s=50,
                          dt=1., duration_factors=(2.,), delta_num=0., delta_pred=0.,
                          shapes=(QUINTIC_SHAPE, beta_shape(2, 3), three_clothoid_shape(.25)))
    labels = [c["label"] for c in run["decisions"][0]["candidates"]]
    assert {l.split(",")[0] for l in labels} == {"shape=quintic", "shape=beta_2_3", "shape=clothoid3_0.25"}


def test_group_radius_leaves_a_far_neighbour_out_of_the_exposure_group():
    """The E/D case from the Bay Area runs: a bad-signal aircraft 6 km away."""
    s = scene(gap_from_follower_m=6000.)

    def radio(xyz):
        return {"sinr_db": np.where(xyz[:, 0] < -5000., -20., 20.)}

    first = lambda r: next(o for o in r["observations"] if o["role"] == "A")
    unbounded = rollout(s, CFG, None, horizon_s=2., dt=1., radio=radio)
    bounded = rollout(s, replace(CFG, group_radius_m=3617.5), None, horizon_s=2., dt=1., radio=radio)
    assert set(first(unbounded)["members"]) == {"C", "A", "B"}
    assert set(first(bounded)["members"]) == {"A", "B"}
    assert first(unbounded)["policy"] != "C"
    assert first(bounded)["policy"] == "C"


def test_group_radius_must_be_positive_or_absent():
    for bad in (0., -1., float("inf")):
        with pytest.raises(ValueError):
            replace(CFG, group_radius_m=bad)
    assert replace(CFG, group_radius_m=None).group_radius_m is None


def test_decision_phase_shifts_the_clock_without_changing_the_physics():
    s = scene(gap_from_follower_m=6000., target_lane_front_offset_m=5000.,
              target_lane_rear_offset_m=5000., zones=(WeakZone(0, 100, 10000),))
    common = dict(horizon_s=12, prediction_s=50, dt=1., duration_factors=(2.,),
                  delta_num=0., delta_pred=0.)
    plain = run_closed_loop(s, CFG, ENVELOPE, VOLUME, **common)
    zero = run_closed_loop(s, CFG, ENVELOPE, VOLUME, decision_phase_s=0., **common)
    assert [d["t_s"] for d in plain["decisions"]] == [d["t_s"] for d in zero["decisions"]]
    shifted = run_closed_loop(s, CFG, ENVELOPE, VOLUME, decision_phase_s=2., **common)
    assert shifted["decisions"][0]["reason"] == "before_first_decision_phase"
    assert [d["t_s"] for d in shifted["decisions"][1:3]] == [2., 7.]
    with pytest.raises(ValueError):
        run_closed_loop(s, CFG, ENVELOPE, VOLUME, decision_phase_s=5., **common)


def test_scenario_declares_established_pairs_and_a_distant_leader_is_not_chased():
    """A leader beyond the fallback spacing is not one E is following."""
    far = scene(gap_to_leader_m=4000.)
    legacy = rollout(far, CFG, None, horizon_s=60., dt=1.)
    declared = rollout(replace(far, established_pairs=()), CFG, None, horizon_s=60., dt=1.)
    assert max(legacy["trace"]["A"]["v_mps"]) > 50. + 1e-6
    assert max(declared["trace"]["A"]["v_mps"]) == pytest.approx(50.)
    assert initial_state(scene()).recovery_pairs == {("A", "B")}
    assert initial_state(scene(established_pairs=())).recovery_pairs == set()
    with pytest.raises(ValueError):
        scene(established_pairs=(("A", "D"),))
