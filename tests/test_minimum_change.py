from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from capacity_policy.geometry import Corridor
from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.radio import RadioConfig
from uam_simulator.lateral_study import LateralConfig, SmoothCorridorFrame, simulate_isolated
from uam_simulator.minimum_change import (CurvatureEnvelope, MinimumChangeConfig,
    MinimumChangePlanner, completed_recovery, first_stable_c)


def test_continuous_recovery_and_post_completion_window():
    times = np.arange(0., 105., 5.)
    good = times >= 20
    assert first_stable_c(times, good, 30) == 20
    assert completed_recovery(times, good, 43.87, 30) == 20
    interrupted = good.copy(); interrupted[times == 60] = False
    assert completed_recovery(times, interrupted, 43.87, 30) is None
    assert completed_recovery(times[:12], good[:12], 43.87, 30) is None


def test_choice_is_displacement_then_duration_not_largest_communication_gain():
    options = [dict(status="eligible",target_m=d,duration_s=t,recovery_time_s=r)
               for d,t,r in [(400,80,10),(100,50,25),(-100,45,30),(100,45,20)]]
    assert MinimumChangePlanner.choose(options,0)["target_m"] == 100
    assert MinimumChangePlanner.choose(options,0)["duration_s"] == 45


def test_c_policy_holds_without_radio_prediction():
    frame = SmoothCorridorFrame(Corridor.straight(1000),100)
    planner = MinimumChangePlanner(frame)
    def radio(_):
        raise AssertionError("C should not trigger a move or forecast")
    rows = planner.candidates(frame,radio,LateralConfig(),q=0,t=0,source=0,
                               targets=[100,200],history=[(0,0)],policy="C")
    assert rows[0]["status"] == "policy_satisfied_hold"
    assert planner.choose(rows,0) is None


def test_reference_curvature_bound_dominates_dense_samples():
    x = np.linspace(0,10000,21)
    frame = SmoothCorridorFrame(Corridor(np.c_[x,300*np.sin(x/2000)],"LOCAL_METRIC"),500)
    envelope = CurvatureEnvelope(frame)
    for lo,hi in [(0,frame.length_m),(120,2600),(2785,6731)]:
        q = np.linspace(lo,hi,10001)
        k = frame.frame(q)[-1]
        bounds = envelope.bounds(lo,hi)
        assert bounds[0] <= k.min()+1e-12 and bounds[1] >= k.max()-1e-12
        for d in [-500,0,500]:
            assert envelope.offset_bound(lo,hi,[d]) >= np.max(abs(k/(1-k*d)))-1e-12


def test_straight_maneuver_curvature_bound_is_conservative():
    frame = SmoothCorridorFrame(Corridor.straight(10000),500)
    envelope = CurvatureEnvelope(frame)
    T=44.;u=np.linspace(0,1,10001);delta=100.;V=50.
    vd=delta*30*u*u*(1-u)**2/T
    ad=delta*60*u*(1-u)*(1-2*u)/T**2
    exact=np.abs(V*ad/(V*V+vd*vd)**1.5)
    bound=envelope.maneuver_bound(0,2200,0,100,T,V)
    assert bound >= exact.max()


def test_natural_recovery_does_not_provoke_gratuitous_move():
    frame = SmoothCorridorFrame(Corridor.straight(2000),100)
    cfg=replace(LateralConfig(),cruise_mps=10,policy_s=1,decision_s=2,window_s=3,
                horizon_s=20,min_maneuver_s=2,lateral_speed_limit_mps=10,
                lateral_accel_limit_mps2=10)
    planner=MinimumChangePlanner(frame,MinimumChangeConfig(minimum_radius_m=1,candidate_spacing_m=10))
    radio=lambda p:{"sinr_db":np.full(len(p),10.)}
    rows=planner.evaluate(frame,radio,cfg,q=0,t=0,source=0,target=10,durations=[5.],history=[(0,1)])
    assert rows[0]["status"] == "staying_recovers_no_later"
    assert rows[0]["stay_recovery_time_s"] == rows[0]["recovery_time_s"]


def test_smallest_recovery_move_selected_in_synthetic_radio_field():
    frame = SmoothCorridorFrame(Corridor.straight(5000),100)
    cfg=replace(LateralConfig(),cruise_mps=10,policy_s=1,decision_s=2,window_s=3,
                horizon_s=40,min_maneuver_s=2,lateral_speed_limit_mps=10,
                lateral_accel_limit_mps2=10)
    planner=MinimumChangePlanner(frame,MinimumChangeConfig(minimum_radius_m=1,candidate_spacing_m=10))
    radio=lambda p:{"sinr_db":np.where(p[:,1]>=5,10.,-10.)}
    rows=planner.candidates(frame,radio,cfg,q=0,t=0,source=0,targets=[-20,-10,10,20],history=[(0,1)],policy="F")
    chosen=planner.choose(rows,0)
    assert chosen is not None and chosen["target_m"] == 10
    assert all(abs(r["target_m"]) == 10 for r in rows)


def test_disabled_transitions_preserve_original_trace():
    frame=SmoothCorridorFrame(Corridor.straight(500),100)
    scenario=SimpleNamespace(base_stations=BaseStationSet((BaseStation("b",250,0,10),),"LOCAL_METRIC"),
                             radio=RadioConfig(served_set_size=1))
    cfg=replace(LateralConfig(),cruise_mps=10,altitude_m=30)
    old=simulate_isolated(frame,scenario,cfg,[-100,0,100],0,False)
    new=simulate_isolated(frame,scenario,cfg,[-100,0,100],0,False,MinimumChangePlanner(frame))
    assert new == old


def test_invalid_planning_parameters_are_rejected():
    with pytest.raises(ValueError):
        MinimumChangeConfig(minimum_radius_m=0)


def test_fixed_tangential_speed_acceleration_is_continuous_at_curve_knots():
    x=np.linspace(0,5000,21)
    frame=SmoothCorridorFrame(Corridor(np.c_[x,150*np.sin(x/1000)],"LOCAL_METRIC"),500)
    for q in frame.controls_q[1:-1]:
        left=frame.kinematics(q-1e-6,100,3,.1,50)[1]
        right=frame.kinematics(q+1e-6,100,3,.1,50)[1]
        assert np.linalg.norm(left-right)<1e-7
