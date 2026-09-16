from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.radio import RadioConfig, compute_link_state
from capacity_policy.trajectory import TrajectoryState
from uam_simulator.dynamic_transitions import Layout, make_maneuver
from uam_simulator.policy_motion import (
    BSRadio, Config, Quintic, Request, Simulator, Vehicle, commands, compare_predictions,
    connector_for, interval_separation, new_insertion_deficits, observe, policy_from_exposure,
)


def cfg(**kwargs):
    return replace(Config(dt_s=.25, policy_s=1, decision_s=2, horizon_s=10,
        duration_s=40, window_s=3, cruise_mps=10, d0_m=2,
        tau_c_s=.5, tau_r_s=1, tau_f_s=2, buffer_s2_per_m=.01,
        horizontal_separation_m=2, vertical_separation_m=2,
        min_maneuver_s=2, lateral_speed_limit_mps=10, vertical_speed_limit_mps=10,
        lateral_accel_limit_mps2=20, vertical_accel_limit_mps2=20,
        accel_limit_mps2=10, decel_limit_mps2=10, accept_braking_mps2=10,
        connector_s=10, entry_m=50, exit_m=300, destination_m=350,
        cooldown_s=2), **kwargs)


def good(p):
    return np.full(p.shape[0], 5.)


def test_quintic_general_boundary_and_interval_polynomial():
    rng = np.random.default_rng(8)
    for _ in range(100):
        p0,p1,v0,v1,a0,a1 = rng.normal(size=(6,3))
        q = Quintic.boundary(3, 12, p0,p1,v0,v1,a0,a1)
        for t, expected in [(3,(p0,v0,a0)), (15,(p1,v1,a1))]:
            np.testing.assert_allclose(q.sample(t), expected, atol=1e-10)
        c = q.interval(7,.2)
        for u in [0,.25,1]:
            np.testing.assert_allclose(np.polynomial.polynomial.polyval(u,c),q.sample(7+.2*u)[0],atol=1e-11)


@pytest.mark.parametrize("value,policy",[(0,"C"),(.05,"C"),(.050001,"R"),(.1,"R"),(.100001,"F")])
def test_policy_boundaries(value, policy):
    assert policy_from_exposure(value, Config()) == policy


def test_available_neighbors_position_sorted_no_padding_and_first_sample():
    config, layout = cfg(), Layout((0,), (10,))
    fleet = [Vehicle(str(k),50+(5-k)*10,10,(0,0)) for k in range(6)]
    rows = observe(fleet,0,layout,config,good)
    groups={r["aircraft_id"]:r["members"] for r in rows}
    assert groups["0"] == ["2","1","0"]
    assert groups["1"] == ["3","2","1","0"]
    assert groups["2"] == ["4","3","2","1","0"]
    assert all(a.exposure == 0 and a.policy=="C" for a in fleet)
    lone = [Vehicle("x",50,10,(0,0))]
    observe(lone,0,layout,config,lambda p:np.array([-5.]))
    assert lone[0].policy=="F" and lone[0].exposure==1


def test_available_history_time_mean_not_pooled_member_times():
    config, layout=cfg(window_s=30),Layout((0,), (10,))
    a=Vehicle("a",70,10,(0,0),history=[(0.,1/3,("a","b","c"))])
    fleet=[a]+[Vehicle(str(k),50+k*10,10,(0,0)) for k in [0,1,3,4]]
    rows=observe(fleet,5,layout,config,good)
    assert a.exposure==pytest.approx(1/6)
    assert next(r for r in rows if r["aircraft_id"]=="a")["group_size"]==5
    assert a.history[0][2]==("a","b","c")


def test_groups_do_not_include_other_flow_but_safety_does():
    layout=Layout((0,1),(10,))
    fleet=[Vehicle("a",50,10,(0,0)),Vehicle("b",50,10,(1,0))]
    rows=observe(fleet,0,layout,cfg(),lambda p:np.array([4.,-4.]))
    assert [r["policy"] for r in rows]==["C","F"]
    sim=Simulator(layout,cfg(),good,initial=fleet).run()
    assert sim.issue["reason"]=="three_dimensional_interval_conflict"


def test_policy_spacing_and_downgrade_raw_vs_clipped():
    config=Config()
    a=Vehicle("a",10000,50,(0,0))
    b=Vehicle("b",11367.5,50,(0,0))
    layout=Layout((0,),(300,))
    assert commands([a,b],0,layout,config,"acc")["a"]["cmd"]==0
    a.policy="R"
    c=commands([a,b],0,layout,config,"acc")["a"]
    assert c["raw"]==-3.75 and c["cmd"]==-2
    assert config.spacing("C",50)==1367.5
    assert config.spacing("R",50)==2117.5
    assert config.spacing("F",50)==3617.5


def test_sparse_acc_does_not_exceed_cruise_and_no_extra_300m_floor():
    config=cfg()
    a,b=Vehicle("a",50,10,(0,0)),Vehicle("b",100,10,(0,0))
    assert commands([a,b],0,Layout((0,),(10,)),config,"acc")["a"]["cmd"]==0
    assert config.spacing("C",0)==2


def test_continuous_interval_detects_between_sample_collision():
    # Endpoints are each safely separated; crossing occurs at u=0.5.
    a,b=np.zeros((6,3)),np.zeros((6,3))
    a[0]=[-5,0,10];a[1]=[10,0,0];b[0]=[0,0,10]
    score,issue=interval_separation({"a":a,"b":b},cfg())
    assert issue and issue["interval_fraction"]==pytest.approx(.5)
    assert score==pytest.approx(0)


def test_interval_safe_constant_pair_does_not_broadcast_polynomial_terms():
    a,b=np.zeros((6,3)),np.zeros((6,3))
    a[0]=[0,0,0];b[0]=[2,0,0]
    score,issue=interval_separation({"a":a,"b":b},cfg())
    assert issue is None and score==pytest.approx(1)


def test_speed_limit_crossing_uses_exact_substep_displacement():
    config=cfg(k_speed=10,duration_s=1)
    a=Vehicle("a",50,9.9,(0,0))
    sim=Simulator(Layout((0,),(10,)),config,good,initial=[a])
    sim.advance(.25)  # command=1; reaches 10 in 0.1 seconds.
    assert sim.t==pytest.approx(.1)
    assert sim.fleet[0].s_m==pytest.approx(50.995)
    sim.advance(.25)
    assert sim.fleet[0].s_m==pytest.approx(52.495)
    assert sim.fleet[0].v_mps==10


def test_transition_policy_ownership_and_history_retained():
    config,layout=cfg(),Layout((0,10),(10,))
    a=Vehicle("a",50,10,(0,0),history=[(0,1.,("a",))],exposure=1,policy="F")
    sim=Simulator(layout,config,good,initial=[a])
    own=sim.fleet[0]
    own.maneuver=make_maneuver((0,0),(1,0),0,layout,config)
    sim.t=own.maneuver.duration_s/2
    sim.settle()
    assert own.cell==(0,0)
    sim.t=own.maneuver.start_s+own.maneuver.duration_s
    sim.settle()
    assert own.cell==(1,0) and own.history==[(0,1.,("a",))]
    assert own.cooldown_until==sim.t+config.cooldown_s


def test_single_virtual_full_mission_and_initial_c_no_fake_history():
    layout,config=Layout((0,10),(10,)),cfg()
    sim=Simulator(layout,config,good,requests=[Request("a",0,(1,0))]).run()
    assert sim.issue is None
    a=sim.fleet[0]
    assert a.entry_s==pytest.approx(10)
    assert a.exit_s==pytest.approx(35)
    assert a.completion_s is None  # 40s observation right-censors 45s mission.
    full=Simulator(layout,replace(config,duration_s=50),good,requests=[Request("a",0,(1,0))]).run()
    assert full.fleet[0].completion_s==pytest.approx(45)
    assert full.fleet[0].phase=="COMPLETE"
    corridor=[r for r in sim.observations if r["phase"]=="CORRIDOR"]
    assert corridor[0]["t_s"]==10 and corridor[0]["history_count"]==1
    assert all(r["exposure"] is None for r in sim.observations if r["phase"]!="CORRIDOR")


def test_connector_component_limits_are_checked():
    layout=Layout((0,),(300,))
    with pytest.raises(ValueError,match="connector"):
        Simulator(layout,cfg(),good,requests=[Request("a",0,(0,0))])


def test_shared_origin_defers_simultaneous_requests_and_conserves_counts():
    config,layout=cfg(duration_s=3),Layout((0,),(10,))
    sim=Simulator(layout,config,good,requests=[Request("a",0,(0,0)),Request("b",0,(0,0))]).run()
    assert sim.issue is None
    releases=[r for r in sim.events if r["status"]=="released"]
    assert releases[0]["t_s"]==0
    assert all(r["t_s"]>0 for r in releases[1:])
    assert len(sim.pending)+len(sim.fleet)==2
    assert sim.deferrals["b"]>0


def test_preview_preserves_actual_state_history_and_no_future_discretion():
    config,layout=cfg(),Layout((0,10),(10,))
    sim=Simulator(layout,config,good,initial=[Vehicle("a",50,10,(0,0))])
    sim.policy_tick()
    before=deepcopy(sim.fleet)
    trial=sim.preview(5)
    assert sim.fleet==before and sim.t==0
    assert trial.t==5 and not trial.accepted_moves
    assert trial.fleet[0].s_m==100


def test_transition_uses_group_exposure_and_completes():
    config,layout=cfg(duration_s=12),Layout((0,10),(10,))
    radio=lambda p:p[:,1]-5
    sim=Simulator(layout,config,radio,"fixed_cruise",initial=[Vehicle("a",50,10,(0,0))]).run(True)
    assert sim.issue is None
    assert sim.accepted_moves==1 and sim.completed_moves==1
    accepted=[e for e in sim.events if e["status"]=="transition_accepted"][0]
    assert accepted["score"]["exposure_benefit"]>0
    assert accepted["score"]["spacing_benefit_m"]>0
    assert sim.fleet[0].cell==(1,0)


def test_bs_adapter_exactly_reuses_radio_kernel():
    bs=BaseStationSet(tuple(BaseStation(f"b{i}",x,y,30) for i,(x,y) in enumerate(
        [(0,500),(1000,500),(2000,-500),(3000,-500)])),"LOCAL_METRIC")
    rc=RadioConfig(served_set_size=3)
    positions=np.array([[500,0,300],[1400,250,600]],float)
    adapter=BSRadio(bs,rc)(positions)
    state=TrajectoryState(np.array([0.]),positions[:,None,:],positions[:,0,None],np.ones((2,1),bool))
    direct=compute_link_state(state,bs,rc)
    for key in adapter:
        np.testing.assert_array_equal(adapter[key],direct[key][:,0])


def test_bad_radio_and_invalid_config_rejected():
    with pytest.raises(ValueError,match="radio"):
        observe([Vehicle("a",50,10,(0,0))],0,Layout((0,),(10,)),cfg(),lambda p:np.array([np.nan]))
    for changes in [{"dt_s":.3},{"neighbors":1.5},{"accept_braking_mps2":100},{"policy_s":0}]:
        with pytest.raises(ValueError):
            cfg(**changes)


def test_policy_downgrade_saturates_and_integrates_instead_of_failing():
    config, layout = Config(), Layout((0,), (300,))
    sim = Simulator(layout, config, lambda p: np.full(len(p), -5.), initial=[
        Vehicle("follower", 12000, 50, (0, 0)), Vehicle("leader", 13500, 50, (0, 0))])
    sim.policy_tick()
    sim.advance(.5)
    assert sim.issue is None
    assert sim.trace[0]["raw_acceleration"] == pytest.approx(-10.5875)
    assert sim.trace[0]["command_acceleration"] == -2
    assert sim.fleet[0].v_mps == 49
    assert sim.fleet[0].s_m == pytest.approx(12024.75)
    trial = sim.preview(10)
    assert trial.issue is None
    assert all(r[3] >= -2 for r in trial.control_rows)


@pytest.mark.parametrize("mode", ["acc", "fixed_cruise"])
def test_existing_source_deficit_does_not_block_beneficial_empty_lane_move(mode):
    config = replace(Config(), duration_s=130, horizon_s=150)
    layout = Layout((-250, 250), (300,))
    radio = lambda p: np.where(p[:, 1] < 0, -5., 5.)
    sim = Simulator(layout, config, radio, mode, initial=[
        Vehicle("a", 12000, 50, (0, 0)), Vehicle("z", 13500, 50, (0, 0))]).run(True)
    accepted = [e for e in sim.events if e["status"] == "transition_accepted"]
    assert sim.issue is None
    assert any(e["aircraft_id"] == "a" and e["t_s"] == 0 for e in accepted)
    assert sim.completed_moves > 0
    assert sim.minimum_separation >= 1-1e-7
    assert all(r["command_acceleration"] >= -2 for r in sim.trace)
    assert min(r["raw_acceleration"] for r in sim.trace) < -2


@pytest.mark.parametrize("target_s,follower,leader", [
    (105, "a", "target"), (95, "target", "a")])
def test_new_target_predecessor_and_follower_deficits_still_reject(target_s, follower, leader):
    config, layout = cfg(), Layout((0, 10), (10,))
    sim = Simulator(layout, config, lambda p: p[:, 1]-5, initial=[
        Vehicle("a", 100, 10, (0, 0)), Vehicle("target", target_s, 10, (1, 0))])
    sim.policy_tick()
    sim.choose()
    event = next(e for e in sim.events if e["aircraft_id"] == "a")
    assert event["status"] == "insertion_target_gap_rejected"
    deficit = event["new_relationship_deficits"][0]
    assert (deficit["follower"], deficit["leader"]) == (follower, leader)
    assert deficit["gap_m"] == 5 < deficit["desired_gap_m"]
    assert sim.fleet[0].maneuver is None


def test_existing_source_pair_does_not_mask_a_new_target_deficit():
    config, layout = cfg(), Layout((0, 10), (10,))
    fleet = [Vehicle("a", 100, 10, (0, 0)), Vehicle("source", 103, 10, (0, 0)),
             Vehicle("target", 104, 10, (1, 0))]
    before = commands(fleet, 0, layout, config, "acc")
    fleet[0].maneuver = make_maneuver((0, 0), (1, 0), 0, layout, config)
    after = commands(fleet, 0, layout, config, "acc")
    deficits = new_insertion_deficits(fleet, before, after, config)
    assert [(d["follower"], d["leader"]) for d in deficits] == [("a", "target")]


def test_acc_saturation_does_not_hide_a_physical_conflict():
    config = cfg(duration_s=1, decel_limit_mps2=2, accept_braking_mps2=2)
    sim = Simulator(Layout((0,), (10,)), config, good, initial=[
        Vehicle("a", 100, 10, (0, 0)), Vehicle("b", 103, 0, (0, 0))])
    trial = sim.preview(1)
    assert trial.issue["reason"] == "three_dimensional_interval_conflict"
    assert trial.control_rows[0][3] >= -2
    assert sim.t == 0 and sim.issue is None


@pytest.mark.parametrize("mode", ["acc", "fixed_cruise"])
def test_candidate_with_admissible_initial_gap_but_future_conflict_is_rejected(mode):
    # Strong acceleration saturates in ACC. Both modes must reject this future
    # conflict even though the initial target spacing is met, and raw is not used.
    config = cfg(d0_m=0, tau_c_s=0, tau_r_s=0, tau_f_s=0, buffer_s2_per_m=0,
                 k_gap=0, k_relative=0, horizontal_separation_m=5)
    layout = Layout((0, 10), (10,))
    own = Vehicle("a", 100, 10, (0, 0))
    target = Vehicle("z", 107, 10 if mode == "fixed_cruise" else 0, (1, 0))
    if mode == "fixed_cruise":
        # Side-by-side target: no longitudinal leader at equality, so only the
        # continuous 3D check catches the merging conflict.
        target.s_m = 100
    sim = Simulator(layout, config, lambda p:p[:, 1]-5, mode, initial=[own, target])
    sim.policy_tick()
    sim.choose()
    event = next(e for e in sim.events if e["aircraft_id"] == "a")
    assert event["status"] == "candidate_infeasible"
    assert event["detail"]["reason"] == "three_dimensional_interval_conflict"
    assert sim.fleet[0].maneuver is None


@pytest.mark.parametrize("stay,change,rejected", [
    (-2., -2., False), (-2., -1.8, False), (-1., -2., True), (0., -1.5, False)])
def test_optional_stricter_braking_gate_uses_paired_executed_commands(stay, change, rejected):
    from types import SimpleNamespace
    config = cfg(accept_braking_mps2=1.5)
    observations = [{"aircraft_id":"a", "t_s":0, "phase":"CORRIDOR",
                     "members":["a"], "policy":"F", "exposure":1.}]
    # Different event partitions; raw feedback is deliberately extreme in both.
    before = SimpleNamespace(observations=observations, leader_history={},
                             control_rows=[(0., 1., "a", stay, -50.)])
    after = SimpleNamespace(observations=observations, leader_history={},
                            control_rows=[(0., .3, "a", change, -60.), (.3, 1., "a", change, -60.)])
    score = compare_predictions(before, after, "a", config)
    assert (score["braking_violation"] is not None) == rejected
    if rejected:
        assert score["braking_violation"]["start_s"] == 0
        assert score["braking_violation"]["end_s"] == .3


def test_full_static_groups_match_dashboard_available_history_kernel():
    from uam_simulator.group_runner import _policy_records
    config,layout=cfg(window_s=3),Layout((0,),(10,))
    fleet=[Vehicle(str(k),50+k*10,10,(0,0)) for k in range(7)]
    signals=np.array([[5 if (k+t)%5 else -5 for t in range(6)] for k in range(7)])
    expected,valid,_=_policy_records({"active":np.ones((7,6),bool),
        "link_ok":signals>=config.threshold_db,"t":np.arange(6.)},
        [str(k) for k in range(7)],5,3,config.exposure_c,config.exposure_r)
    for t in range(6):
        rows=observe(fleet,t,layout,config,lambda p:signals[:,t])
        for row in rows:
            code={"F":0,"R":1,"C":2}[row["policy"]]
            assert code==expected[int(row["aircraft_id"]),t]


def test_serial_competing_candidates_and_vertical_transition():
    config,layout=cfg(duration_s=10),Layout((-10,0,10),(10,))
    sim=Simulator(layout,config,lambda p:5-np.abs(p[:,1]),"fixed_cruise",
                  initial=[Vehicle("a",50,10,(0,0)),Vehicle("b",50,10,(2,0))]).run(True)
    accepted=[e for e in sim.events if e["status"]=="transition_accepted"]
    assert len(accepted)==1 and sim.issue is None
    vertical=Simulator(Layout((0,),(10,20)),config,lambda p:p[:,2]-15,"fixed_cruise",
                       initial=[Vehicle("a",50,10,(0,0))]).run(True)
    assert vertical.completed_moves==1 and vertical.fleet[0].cell==(0,1)


def test_runner_snapshot_hashes_and_no_run_overwrite(tmp_path):
    import gzip, hashlib, importlib.util, json, zipfile
    from dataclasses import asdict
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    module_spec=importlib.util.spec_from_file_location("policy_motion_runner",root/"scripts/run_policy_motion_checks.py")
    cli=importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(cli)
    spec=json.loads((root/"research/dynamic-transitions/configs/policy_motion_pilot.json").read_text())
    spec["parameters"]=asdict(cfg(duration_s=3))
    spec["offsets_m"]=[0]
    spec["heights_m"]=[10]
    spec["cases"]=[{"id":"tiny","modes":["fixed_cruise"],"transitions":[False],
                   "initial":[{"aircraft_id":"a","s_m":50,"v_mps":10,"cell":[0,0]}]}]
    config=tmp_path/"input.json"
    config.write_text(json.dumps(spec))
    output=tmp_path/"run"
    cli.run(config,output)
    manifest=json.loads((output/"manifest.json").read_text())
    assert manifest["status"]=="completed"
    with zipfile.ZipFile(output/"source_snapshot.zip") as archive:
        for entry in manifest["code"]:
            assert hashlib.sha256(archive.read(entry["path"])).hexdigest()==entry["sha256"]
    for entry in manifest["outputs"]:
        assert hashlib.sha256((output/entry["path"]).read_bytes()).hexdigest()==entry["sha256"]
    with gzip.open(output/"tiny_fixed_cruise_off/observations.json.gz","rt") as stream:
        assert json.load(stream)[0]["history_count"]==1
    with pytest.raises(FileExistsError):
        cli.run(config,output)
