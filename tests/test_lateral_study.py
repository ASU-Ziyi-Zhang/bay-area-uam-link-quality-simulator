from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from capacity_policy.geometry import Corridor
from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.radio import RadioConfig
from uam_simulator.group_runner import _policy_records
from uam_simulator.lateral_study import (
    LateralConfig, LateralMove, SmoothCorridorFrame, field_samples,
    forecast_candidates, maneuver_duration, rk4_progress, simulate_isolated,
)


def small_config(**changes):
    return replace(LateralConfig(cruise_mps=10, altitude_m=30, dt_s=.5,
        policy_s=1, decision_s=2, forecast_dt_s=.5, horizon_s=20,
        window_s=3, cooldown_s=2, min_maneuver_s=2,
        lateral_speed_limit_mps=10, lateral_accel_limit_mps2=10,
        center_control_step_m=100, spatial_sample_m=10), **changes)


def simple_scenario():
    return SimpleNamespace(base_stations=BaseStationSet((BaseStation("b",500,100,10),),"LOCAL_METRIC"),
                           radio=RadioConfig(served_set_size=1))


def test_straight_frame_kinematics_and_exact_progress():
    frame=SmoothCorridorFrame(Corridor.straight(1000),100)
    np.testing.assert_allclose(frame.xyz(50,100,300),[50,100,300])
    np.testing.assert_allclose(frame.kinematics(50,100,2,.3,50),[[50,2],[0,.3]])
    assert rk4_progress(frame,0.,0.,2.,lambda t:100.,50)==pytest.approx(100)
    assert frame.report(500)["minimum_coordinate_factor_bound"]==1


def test_quintic_endpoints_and_component_bounds():
    cfg=LateralConfig()
    for distance in [100,200,500,1000,-100]:
        duration=maneuver_duration(distance,cfg)
        move=LateralMove(0,0,distance,duration)
        assert move.sample(0)==pytest.approx((0,0,0))
        assert move.sample(duration)==pytest.approx((distance,0,0))
        d,v,a=move.sample(np.linspace(0,duration,1001))
        assert np.max(abs(v))<=cfg.lateral_speed_limit_mps+1e-8
        assert np.max(abs(a))<=cfg.lateral_accel_limit_mps2+1e-8
        assert duration==maneuver_duration(distance,replace(cfg,dt_s=.25))


def test_curved_frame_velocity_matches_cartesian_finite_difference():
    x=np.linspace(0,4000,17)
    frame=SmoothCorridorFrame(Corridor(np.c_[x,100*np.sin(x/1000)],"LOCAL_METRIC"),250)
    move=LateralMove(0,0,100,60)
    for q in [555.,1234.,2600.]:
        t=25.; eps=1e-3
        values=move.sample(t)
        v,a=frame.kinematics(q,*values,50)
        qm=rk4_progress(frame,q,t,-eps,lambda t:move.sample(t)[0],50)
        qp=rk4_progress(frame,q,t,eps,lambda t:move.sample(t)[0],50)
        pm=frame.xyz(qm,move.sample(t-eps)[0],300)
        p0=frame.xyz(q,values[0],300)
        pp=frame.xyz(qp,move.sample(t+eps)[0],300)
        np.testing.assert_allclose((pp-pm)[:2]/(2*eps),v,atol=1e-6)
        np.testing.assert_allclose((pp-2*p0+pm)[:2]/eps**2,a,atol=1e-5)
        assert np.linalg.norm(v)==pytest.approx(np.hypot(50,values[1]))


def test_offset_position_and_velocity_are_continuous_across_cubic_knots():
    x=np.linspace(0,3000,13)
    frame=SmoothCorridorFrame(Corridor(np.c_[x,80*np.sin(x/500)],"LOCAL_METRIC"),250)
    for q in frame.controls_q[1:-1]:
        p0=frame.xyz(q-1e-5,100,300);p1=frame.xyz(q+1e-5,100,300)
        v0=frame.kinematics(q-1e-5,100,3,.1,50)[0]
        v1=frame.kinematics(q+1e-5,100,3,.1,50)[0]
        assert np.linalg.norm(p1-p0)<3e-5
        assert np.linalg.norm(v1-v0)<1e-5


def test_singleton_policy_matches_original_available_history_kernel():
    cfg=small_config(threshold_db=30)
    frame=SmoothCorridorFrame(Corridor.straight(500),100)
    result=simulate_isolated(frame,simple_scenario(),cfg,[0],0,False)
    rows=result["observations"]
    signals=np.array([r["sinr_db"] for r in rows])
    times=np.array([r["t_s"] for r in rows])
    codes,valid,_=_policy_records({"t":times,"active":np.ones((1,len(rows)),bool),
        "link_ok":(signals>=cfg.threshold_db)[None,:]},["a"],5,cfg.window_s,cfg.exposure_c,cfg.exposure_r)
    assert valid.all()
    assert [{"F":0,"R":1,"C":2}[r["policy"]] for r in rows]==codes[0].tolist()
    assert sum(result["summary"]["policy_time_shares"].values())==pytest.approx(1)
    assert result["terminal"]["t_s"]==pytest.approx(50)
    assert result["summary"]["path_length_m"]==pytest.approx(500)


def test_predictor_continuous_radio_history_and_end_boundary():
    cfg=small_config()
    frame=SmoothCorridorFrame(Corridor.straight(1000),100)
    radio=lambda p:{"sinr_db":p[:,1]-5}
    rows=forecast_candidates(frame,radio,cfg,q=0,t=0,source=0,targets=[10,-10],history=[(0,1)])
    assert rows[0]["status"]=="eligible" and rows[0]["exposure_benefit"]>0
    assert rows[1]["status"]=="communication_gate_rejected"
    near=forecast_candidates(frame,radio,cfg,q=990,t=0,source=0,targets=[10],history=[(0,1)])
    assert near[0]["status"]=="insufficient_post_move_horizon"


def test_all_good_radio_does_not_generate_spurious_changes_and_static_repeatability():
    cfg=small_config(threshold_db=-100)
    frame=SmoothCorridorFrame(Corridor.straight(500),100)
    off=simulate_isolated(frame,simple_scenario(),cfg,[-10,0,10],0,False)
    on=simulate_isolated(frame,simple_scenario(),cfg,[-10,0,10],0,True)
    assert on["summary"]["completed_moves"]==0
    assert off["trace"]==on["trace"] and off["observations"]==on["observations"]


def test_invalid_inputs_and_folded_envelope_are_rejected():
    with pytest.raises(ValueError):
        small_config(dt_s=.3)
    x=np.linspace(0,1000,9)
    frame=SmoothCorridorFrame(Corridor(np.c_[x,200*np.sin(x/300)],"LOCAL_METRIC"),125)
    with pytest.raises(ValueError,match="regular corridor"):
        frame.report(10000)
    with pytest.raises(ValueError,match="initial offset"):
        simulate_isolated(SmoothCorridorFrame(Corridor.straight(1000)),simple_scenario(),small_config(),[-10,10],0,True)


def test_field_grid_uses_same_radio_as_executed_singleton():
    cfg=small_config()
    frame=SmoothCorridorFrame(Corridor.straight(500),100)
    field=field_samples(frame,simple_scenario(),cfg,[0,10])
    result=simulate_isolated(frame,simple_scenario(),cfg,[0],0,False)
    rows=result["observations"]
    np.testing.assert_allclose(field["sinr_db"][0,:len(rows)],[r["sinr_db"] for r in rows],atol=1e-10)


def test_stage_one_case_matrix_keeps_candidate_resolution_and_initial_state_separate():
    import importlib.util,json
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    module=importlib.util.spec_from_file_location("lateral_runner",root/"scripts/run_lateral_study.py")
    runner=importlib.util.module_from_spec(module);module.loader.exec_module(runner)
    spec=json.loads((root/"research/dynamic-transitions/configs/singleton_lateral_100m.json").read_text())
    static,cases=runner.make_cases(spec,LateralConfig(**spec["parameters"]))
    assert len(static)==11 and len(cases)==18
    adaptive=[c for c in cases if c[-1]=="adaptive"]
    assert len(adaptive)==5
    for _,offsets,start,enabled,config,_ in adaptive:
        assert start==0 and enabled and np.all(np.diff(offsets)==100)
    for bad in [[100,100],[100,250],[]]:
        with pytest.raises(ValueError):
            runner.make_cases({**spec,"envelopes_m":bad},LateralConfig())


def test_reference_integrator_preserves_offset_arc_length_across_a_sharp_knot():
    x=np.linspace(0,4000,17)
    frame=SmoothCorridorFrame(Corridor(np.c_[x,100*np.sin(x/500)],"LOCAL_METRIC"),500)
    q=frame.controls_q[3]-10
    d=200.;dt=2.;V=50.
    end=rk4_progress(frame,q,0,dt,lambda t:d,V)
    # Independent Gaussian quadrature of offset-curve arc length, split at knots.
    cuts=np.r_[q,frame.controls_q[(frame.controls_q>q)&(frame.controls_q<end)],end]
    nodes,weights=np.polynomial.legendre.leggauss(32)
    length=0.
    for lo,hi in zip(cuts[:-1],cuts[1:]):
        sample=(lo+hi)/2+(hi-lo)/2*nodes
        _,_,_,norm,kappa=frame.frame(sample)
        length+=(hi-lo)/2*np.dot(weights,norm*(1-kappa*d))
    assert length==pytest.approx(V*dt,abs=2e-5)


def test_subdivision_does_not_demand_sub_ulp_accuracy_at_large_progress():
    frame=SmoothCorridorFrame(Corridor.straight(60000),2000)
    for q,dt in [(49235.15,.123),(33333.333,.071),(55000.15,.019)]:
        result=rk4_progress(frame,q,0,dt,lambda t:100.,50.,tolerance_m=1e-30)
        assert result==pytest.approx(q+50*dt,abs=1e-8)
