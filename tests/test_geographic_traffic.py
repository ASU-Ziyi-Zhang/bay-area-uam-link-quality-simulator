from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.geometry import Corridor
from capacity_policy.radio import RadioConfig
from uam_simulator.geographic_traffic import (
    EntryRequest,
    GeographicTrafficConfig,
    OffsetArcCoordinates,
    TrafficAircraft,
    TrafficState,
    acc_controls,
    insertion_deficits,
    observe_groups,
    recovery_neighbors,
    rolling_decision,
    separation_score,
    simulate_geographic_traffic,
)
from uam_simulator.lateral_study import SmoothCorridorFrame
from uam_simulator.minimum_change import CurvatureEnvelope


def config(**changes):
    base = GeographicTrafficConfig(
        altitude_m=300,
        cruise_mps=10,
        dt_s=.5,
        forecast_dt_s=1,
        policy_s=1,
        decision_s=1,
        horizon_s=20,
        window_s=2,
        cooldown_s=1,
        center_control_step_m=100,
        predictive_enabled=True,
        predictive_rollout_s=4,
        predictive_evaluation_s=4,
        predictive_endpoint_upgrade=True,
        d0_m=2,
        tau_c_s=1,
        tau_r_s=2,
        tau_f_s=3,
        buffer_s2_per_m=.01,
        horizontal_separation_m=5,
        vertical_separation_m=5,
        maximum_time_s=70,
    )
    return replace(base, **changes)


def radio(values):
    def evaluate(positions):
        result = np.resize(np.asarray(values, float), len(positions))
        return {"sinr_db": result, "serving_bs": np.zeros(len(positions), int),
                "serving_rsrp_dbm": np.full(len(positions), -80.0)}
    return evaluate


def test_offset_arc_coordinates_and_geographic_separation():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    offsets = (-10, 0, 10)
    arc = OffsetArcCoordinates(frame, offsets)
    assert arc.gap(100, 350, 1) == pytest.approx(250, abs=1e-8)
    assert arc.remaining(900, 1) == pytest.approx(100, abs=1e-8)
    a = TrafficAircraft("a", 100, 10, 0)
    b = TrafficAircraft("b", 100, 10, 2)
    score, issue = separation_score([a, b], 0, frame, offsets, 300,
                                    config(horizontal_separation_m=15))
    assert issue is None
    assert score == pytest.approx((20 / 15) ** 2)


def test_policy_groups_remain_in_logical_flow():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    offsets = (-10, 10)
    state = TrafficState(0, [
        TrafficAircraft("a", 100, 10, 0),
        TrafficAircraft("b", 200, 10, 0),
        TrafficAircraft("c", 150, 10, 1),
    ], [])
    rows = observe_groups(state, frame, offsets, 300, config(), radio([5, -5, -5]))
    by_id = {row["aircraft_id"]: row for row in rows}
    assert by_id["a"]["members"] == ["a", "b"]
    assert by_id["b"]["members"] == ["a", "b"]
    assert by_id["c"]["members"] == ["c"]
    assert by_id["a"]["group_bad_fraction"] == pytest.approx(.5)
    assert by_id["c"]["policy"] == "F"


def _members(rows):
    return {row["aircraft_id"]: row["members"] for row in rows}


def test_neighbourhood_group_matches_lane_order_inside_one_stream():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    # 15 m apart, inside the 66 m cap (two fallback spacings in config()).
    aircraft = lambda: [TrafficAircraft(f"a{i}", 100 + 15 * i, 10, 0) for i in range(7)]
    old = _members(observe_groups(TrafficState(0, aircraft(), []), frame, (0,), 300,
                                  config(), radio([5])))
    new = _members(observe_groups(TrafficState(0, aircraft(), []), frame, (0,), 300,
                                  config(group_mode="neighbourhood"), radio([5])))
    for identifier in ("a2", "a3", "a4"):
        assert new[identifier] == old[identifier]
    # At the head of the stream the four nearest all lie behind.
    assert old["a6"] == ["a4", "a5", "a6"]
    assert new["a6"] == ["a2", "a3", "a4", "a5", "a6"]


def test_neighbourhood_group_spans_adjacent_cells_only():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    offsets = (-10, 0, 10)
    state = TrafficState(0, [
        TrafficAircraft("a", 100, 10, 0), TrafficAircraft("b", 110, 10, 1),
        TrafficAircraft("c", 120, 10, 0), TrafficAircraft("d", 130, 10, 1),
        TrafficAircraft("e", 140, 10, 0), TrafficAircraft("far_cell", 120, 10, 2),
    ], [])
    rows = observe_groups(state, frame, offsets, 300, config(group_mode="neighbourhood"),
                          radio([5]))
    groups = _members(rows)
    # Splitting one stream over two adjacent cells keeps the five-aircraft group.
    assert groups["c"] == ["a", "b", "c", "d", "e"]
    # Two lateral steps away is outside the neighbourhood of lane 0.
    assert "far_cell" not in groups["a"]
    assert groups["far_cell"] == ["b", "far_cell", "d"]


def test_neighbourhood_cap_follows_the_fallback_headway():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    make = lambda: TrafficState(0, [TrafficAircraft("a", 100, 10, 0),
                                    TrafficAircraft("b", 200, 10, 0)], [])
    # config(): d0 2 m, tau_F 3 s, b 0.01 at 10 m/s gives S_F = 33 m, cap 66 m.
    short = _members(observe_groups(make(), frame, (0,), 300,
                                    config(group_mode="neighbourhood"), radio([5])))
    assert short["a"] == ["a"]
    # tau_F = 6 s gives S_F = 63 m and a 126 m cap; no radius is restated.
    longer = _members(observe_groups(make(), frame, (0,), 300,
                                     config(group_mode="neighbourhood", tau_f_s=6), radio([5])))
    assert longer["a"] == ["a", "b"]


def test_group_mode_is_validated():
    with pytest.raises(ValueError, match="group_mode"):
        config(group_mode="nearest")


def test_fallback_on_an_aircraft_with_nothing_ahead_does_not_slow_it():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    arc = OffsetArcCoordinates(frame, (0,))
    cfg = config()
    follower = TrafficAircraft("f", 100, 10, 0)
    leader = TrafficAircraft("l", 150, 10, 0)
    state = TrafficState(0, [follower, leader], [])
    before = acc_controls(state, arc, cfg)
    leader.policy = "F"
    after = acc_controls(state, arc, cfg)
    assert after["l"]["leaders"] == [] and after["l"]["command"] == 0
    # The leader's policy is not part of the follower's spacing requirement.
    assert after["f"]["command"] == before["f"]["command"]


def test_policy_spacing_acc_is_bounded_and_uses_physical_lane_gap():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    arc = OffsetArcCoordinates(frame, (0,))
    cfg = config()
    follower = TrafficAircraft("f", 100, 10, 0)
    leader = TrafficAircraft("l", 113, 10, 0)
    state = TrafficState(0, [follower, leader], [])
    controls = acc_controls(state, arc, cfg)
    assert controls["f"]["leaders"][0][1] == pytest.approx(13)
    assert controls["f"]["command"] == pytest.approx(0)
    follower.policy = "F"
    controls = acc_controls(state, arc, cfg)
    assert controls["f"]["raw"] < 0
    assert controls["f"]["command"] >= -cfg.deceleration_limit_mps2


def test_recovery_requires_the_existing_leader_and_retains_other_lane_constraints():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    arc = OffsetArcCoordinates(frame, (0, 10))
    cfg = config(speed_max_mps=20)
    follower = TrafficAircraft("f", 100, 10, 0)
    leader = TrafficAircraft("l", 300, 10, 0)
    state = TrafficState(0, [follower, leader], [])
    pairs = frozenset({("f", "l")})
    assert acc_controls(state, arc, cfg)["f"]["command"] == 0
    assert acc_controls(state, arc, cfg, recovery_pairs=pairs)["f"]["command"] > 0
    # A different predecessor does not inherit an unrelated recovery request.
    assert acc_controls(state, arc, cfg, recovery_pairs={("f", "other")})["f"]["command"] == 0
    follower.target_lane = 1
    state.active.append(TrafficAircraft("close", 105, 5, 1))
    assert acc_controls(state, arc, cfg, recovery_pairs=pairs)["f"]["command"] < 0
    state.active = [follower]
    follower.speed_mps = 12
    assert acc_controls(state, arc, cfg, recovery_pairs=pairs)["f"]["command"] < 0


def test_recovery_respects_physical_speed_ceiling():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    arc = OffsetArcCoordinates(frame, (0,))
    cfg = config(speed_max_mps=20)
    state = TrafficState(0, [TrafficAircraft("f", 0, 20, 0),
                            TrafficAircraft("l", 500, 20, 0)], [])
    assert acc_controls(state, arc, cfg, recovery_pairs={("f", "l")})["f"]["command"] == 0


def test_new_target_predecessor_and_follower_are_checked():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    offsets = (-10, 10)
    arc = OffsetArcCoordinates(frame, offsets)
    cfg = config()
    own = TrafficAircraft("own", 100, 10, 0)
    behind = TrafficAircraft("behind", 95, 10, 1)
    ahead = TrafficAircraft("ahead", 105, 10, 1)
    deficits = insertion_deficits(own, 1,
        TrafficState(0, [own, behind, ahead], []), arc, cfg)
    assert {row["relationship"] for row in deficits} == {
        "target_predecessor", "target_follower"}


def test_nominal_lane_is_immutable_and_equal_cost_safe_return_is_prioritized():
    frame = SmoothCorridorFrame(Corridor.straight(1000), 100)
    offsets = (0, 10)
    cfg = config(horizon_s=50, predictive_rollout_s=20,
                 predictive_evaluation_s=20)
    own = TrafficAircraft("own", 100, 10, lane=1, nominal_lane=0)
    state = TrafficState(0, [own], [])
    arc = OffsetArcCoordinates(frame, offsets)
    events = []
    rolling_decision(state, frame, offsets, 300, cfg, arc, radio([5]),
                     CurvatureEnvelope(frame), events)
    accepted = [row for row in events if row["status"] == "transition_accepted"]
    assert len(accepted) == 1
    assert accepted[0]["decision_type"] == "nominal_return"
    assert accepted[0]["source_lane"] == 1
    assert accepted[0]["target_lane"] == 0
    assert accepted[0]["move_policy_cost_s"] == pytest.approx(
        accepted[0]["stay_policy_cost_s"])
    assert own.nominal_lane == 0
    assert own.lane == 1
    assert own.target_lane == 0


def test_three_by_three_recovery_uses_adjacent_lateral_vertical_grid():
    coordinates = [
        (-500, 240), (-500, 300), (-500, 360),
        (0, 240), (0, 300), (0, 360),
        (500, 240), (500, 300), (500, 360),
    ]
    offsets = [row[0] for row in coordinates]
    altitudes = [row[1] for row in coordinates]
    # From the upper-left corner to the lower-right nominal flow, only the
    # three adjacent moves that reduce Euclidean distance are returned; the
    # diagonal center move is preferred first.
    assert recovery_neighbors(2, 6, offsets, altitudes) == [4, 5, 1]


def test_multilevel_flows_use_distinct_altitudes_and_same_flow_groups():
    frame = SmoothCorridorFrame(Corridor.straight(500), 100)
    scenario = SimpleNamespace(
        base_stations=BaseStationSet((BaseStation("B", 250, 0, 30),), "LOCAL_METRIC"),
        radio=RadioConfig(served_set_size=1),
    )
    result = simulate_geographic_traffic(
        frame, scenario, config(horizontal_separation_m=5,
                                vertical_separation_m=20),
        (0, 0), [EntryRequest("low", 0, 0), EntryRequest("high", 5, 1)],
        transitions=False, flow_altitudes=(240, 360))
    assert result["summary"]["flow_coordinates_m"] == [[0.0, 240.0], [0.0, 360.0]]
    altitude_by_id = {}
    for row in result["trace"]:
        altitude_by_id.setdefault(row["aircraft_id"], row["altitude_m"])
    assert altitude_by_id == {"low": 240.0, "high": 360.0}
    assert all(row["group_size"] == 1 for row in result["observations"])


def test_fixed_entry_multi_uam_run_has_no_capacity_output():
    frame = SmoothCorridorFrame(Corridor.straight(500), 100)
    scenario = SimpleNamespace(
        base_stations=BaseStationSet((BaseStation("B", 250, 0, 30),), "LOCAL_METRIC"),
        radio=RadioConfig(served_set_size=1),
    )
    result = simulate_geographic_traffic(frame, scenario, config(), (0,), [
        EntryRequest("a", 0, 0), EntryRequest("b", 5, 0)], transitions=False)
    assert result["summary"]["status"] == "completed"
    assert result["summary"]["realized_entries"] == 2
    assert result["summary"]["completed_missions"] == 2
    assert result["summary"]["accepted_transitions"] == 0
    assert result["summary"]["capacity_estimated"] is False
    assert result["summary"]["capacity_status"] == "capacity_not_estimated"
    assert all(1 <= row["group_size"] <= 2 for row in result["observations"])


def test_speed_bounds_default_to_the_previous_cruise_cap():
    """Adding wing-borne bounds must not disturb archived behaviour."""
    from uam_simulator.geographic_traffic import GeographicTrafficConfig
    cfg = GeographicTrafficConfig()
    assert cfg.speed_min_mps == 0.0
    assert cfg.speed_max_mps is None
    assert cfg.speed_ceiling_mps == cfg.cruise_mps


def test_declared_speed_bounds_are_used_and_validated():
    from dataclasses import replace
    import pytest
    from uam_simulator.geographic_traffic import GeographicTrafficConfig
    cfg = GeographicTrafficConfig(speed_min_mps=30.0, speed_max_mps=80.0)
    assert cfg.speed_ceiling_mps == 80.0
    # A ceiling above cruise is what makes gap closing possible at all.
    assert cfg.speed_ceiling_mps > cfg.cruise_mps
    with pytest.raises(ValueError):
        replace(cfg, speed_max_mps=40.0)          # below cruise
    with pytest.raises(ValueError):
        replace(cfg, speed_min_mps=60.0)          # at or above cruise
