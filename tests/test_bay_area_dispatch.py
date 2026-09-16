from pathlib import Path
from copy import deepcopy
import numpy as np
import pytest
from capacity_policy import load_scenario
from uam_simulator.bay_area_dispatch import DispatchConfig,DispatchExperiment,World
from uam_simulator.geographic_traffic import GeographicTrafficConfig,TrafficState,TrafficAircraft,EntryRequest
from uam_simulator.motion_envelope import AccelerationEnvelope
from uam_simulator.capacity_analysis import (
    planning_capacity_by_origin_stream, planning_capacity_from_trace,
)

@pytest.fixture(scope='module')
def rig():
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,speed_min_mps=30.,speed_max_mps=80.)
    return DispatchExperiment(scenario,cfg,AccelerationEnvelope(lateral_speed_max_mps=8.))

def test_fixed_target_is_independent_of_executed_speed_and_legacy_unchanged():
    cfg=DispatchConfig()
    assert cfg.spacing('F',30)==cfg.spacing('F',80)==3617.5
    assert cfg.spacing('C',50)==1367.5
    assert GeographicTrafficConfig().spacing('F',30)!=GeographicTrafficConfig().spacing('F',50)

def test_policy_targets_follow_the_spacing_law():
    cfg=DispatchConfig(cruise_mps=50.,d0_m=152.4)
    assert [cfg.spacing(p) for p in 'CRF']==pytest.approx([1319.9,2069.9,3569.9])
    # A longer fallback headway reaches the targets without restating them.
    longer=DispatchConfig(cruise_mps=50.,d0_m=152.4,tau_f_s=90.)
    assert longer.spacing('F')-cfg.spacing('F')==pytest.approx(30*50)
    assert longer.spacing('C')==cfg.spacing('C')
    # Archived configs list the targets; they load when they agree with the law.
    DispatchConfig(cruise_mps=50.,d0_m=152.4,target_c_m=1319.9,target_r_m=2069.9,target_f_m=3569.9)
    with pytest.raises(ValueError,match='spacing law'):
        DispatchConfig(cruise_mps=50.,d0_m=152.4,tau_f_s=90.,target_f_m=3569.9)

def test_departure_schedule_records_deferral_instead_of_teleporting(rig):
    result=rig.run([EntryRequest('L',0.,0),EntryRequest('NF',0.,1)],False,horizon_s=5.)
    events=[e for e in result['events'] if e['status']=='corridor_entry']
    assert events[0]['t_s']==0
    assert events[1]['requested_time_s']==0
    assert events[1]['entry_delay_s']>0
    for role in ('L','NF'):
        rows=[r for r in result['trace'] if r['aircraft_id']==role]
        assert rows[0]['q_m']==0
        assert all(r['total_speed_mps']<=80+1e-7 for r in rows)

def test_forecast_clones_schedule_controller_and_exposure_history(rig):
    w=World(TrafficState(0.,[],[EntryRequest('L',0.,0),EntryRequest('E',30.,0)]))
    rig.prepare(w,[],[]);snapshot=deepcopy(w)
    p=rig.predict(w,10.)
    assert p['admitted'],p
    assert w.traffic.t_s==snapshot.traffic.t_s
    assert w.traffic.active[0].history==snapshot.traffic.active[0].history
    assert w.traffic.pending==snapshot.traffic.pending
    assert w.plans==snapshot.plans


def test_lightweight_forecast_and_audit_copies_are_isolated(rig):
    aircraft=TrafficAircraft('A',1000.,50.,0,history=[(0.,0.,('A',))])
    world=World(TrafficState(0.,[aircraft],[EntryRequest('B',30.,1)]),
                pairs={('A','B')},policy_filters={'A':{'state':'C','count':0}})
    audit=rig.audit_state(world.traffic)
    audit.active[0].q_m=2000.
    assert world.traffic.active[0].q_m==1000.
    assert audit.active[0].history is world.traffic.active[0].history
    cloned=rig.clone_world(world)
    cloned.traffic.active[0].history.append((2.,1.,('A',)))
    cloned.policy_filters['A']['count']=2
    cloned.pairs.clear()
    assert len(world.traffic.active[0].history)==1
    assert world.policy_filters['A']['count']==0
    assert world.pairs=={('A','B')}

def test_ks2_receives_the_explicit_policy_distance(rig):
    q=float(np.interp(1367.5,rig.arc.arc[0],rig.arc.q))
    w=World(TrafficState(0.,[TrafficAircraft('E',0.,50.,0,policy='F'),TrafficAircraft('L',q,50.,0)],[]))
    controls=rig.controls(w)
    assert controls['E']['mode']=='ks2_tracker'
    assert w.plans['E'][1]['target_gap_m']==3617.5

def test_geographic_position_and_curvature_are_used(rig):
    result=rig.run([EntryRequest('L',0.,0)],False,horizon_s=10.)
    rows=result['trace']
    assert len(result['observations'])>=2
    assert all(np.linalg.norm(r['xyz'][:2])>1e6 for r in rows)
    assert any(np.linalg.norm(r['a_xyz'])>.01 for r in rows)
    assert all(r['target_gap_m']==rig.cfg.spacing(r['policy']) for r in rows)


def test_route_speed_governor_anticipates_downstream_curvature():
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,
                       speed_min_mps=30.,speed_max_mps=80.)
    experiment=DispatchExperiment(
        scenario,cfg,AccelerationEnvelope(route_normal_accel_max_mps2=4.575),
        offsets=(-300.,0.,300.))
    upstream,upstream_accel=experiment.route_constraints(45000.,-300.,80.)
    at_curve,_=experiment.route_constraints(45500.,-300.,80.)
    assert cfg.speed_min_mps < upstream < cfg.speed_ceiling_mps
    assert cfg.speed_min_mps < at_curve < upstream
    aircraft=TrafficAircraft('A',45000.,80.,0,policy='C')
    controls=experiment.controls(World(TrafficState(0.,[aircraft],[])))
    assert controls['A']['command']<0
    assert upstream_accel<0
    assert controls['A']['route_speed_ceiling_mps']==pytest.approx(upstream)

def test_route_governor_holds_the_curve_limit_through_a_control_period():
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,d0_m=152.4,
                       speed_min_mps=30.,speed_max_mps=80.)
    envelope=AccelerationEnvelope(route_normal_accel_max_mps2=4.575)
    experiment=DispatchExperiment(scenario,cfg,envelope,offsets=(0.,))
    limit=envelope.route_normal_accel_max_mps2
    kappa=lambda q:abs(float(experiment.frame.frame(q)[4]))
    # R0060: an aircraft closing a gap at 66.8 m/s, 68 m before an apex at
    # q = 8422 m whose curve speed is 66.85 m/s, was still allowed to
    # accelerate and crossed the apex at 67.2 m/s. Every start below is
    # braking-feasible, so no step may break the limit.
    for q0,v0 in ((8354.47,66.8),(8200.,70.),(7900.,75.)):
        q,v=q0,v0
        assert v<=experiment.route_constraints(q,0.,v)[0]
        while q<8700.:
            _,ceiling=experiment.route_constraints(q,0.,v)
            a=max(-cfg.deceleration_limit_mps2,min(cfg.acceleration_limit_mps2,ceiling))
            for f in (.25,.5,.75,1.):
                t=cfg.dt_s*f
                assert kappa(q+v*t+.5*a*t*t)*(v+a*t)**2<=limit+1e-6
            q,v=q+v*cfg.dt_s+.5*a*cfg.dt_s**2,v+a*cfg.dt_s

def test_entry_uses_observed_policy_not_default_c(rig):
    previous=rig.radio
    rig.radio=lambda xyz:{'sinr_db':np.full(len(xyz),-20.),'serving_bs':np.zeros(len(xyz),int),'serving_rsrp_dbm':np.full(len(xyz),-80.)}
    try:
        w=World(TrafficState(0.,[TrafficAircraft('L',2000.,50.,0)],[EntryRequest('E',0.,0)]))
        rig.prepare(w,[],[])
        assert [p.aircraft_id for p in w.traffic.pending]==['E']
        assert w.traffic.active[0].policy=='F'
    finally:rig.radio=previous

def test_entry_clearance_buffer_only_applies_to_entering_aircraft(rig):
    w=World(TrafficState(0.,[TrafficAircraft('L',6000.,50.,0),TrafficAircraft('NF',5800.,50.,1)],[EntryRequest('E',0.,0)]))
    rig.prepare(w,[],[])
    assert 'E' in [a.aircraft_id for a in w.traffic.active]


def test_radio_sampling_policy_update_and_persistence_use_separate_clocks():
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,radio_s=2.,policy_s=5.,decision_s=5.,
        persistence_k=3,speed_min_mps=30.,speed_max_mps=80.)
    experiment=DispatchExperiment(scenario,cfg,
        AccelerationEnvelope(lateral_speed_max_mps=8.),offsets=(0.,))
    experiment.radio=lambda xyz:{'sinr_db':np.full(len(xyz),-20.),
        'serving_bs':np.zeros(len(xyz),int),
        'serving_rsrp_dbm':np.full(len(xyz),-80.)}
    world=World(TrafficState(0.,[TrafficAircraft('A',1000.,50.,0)],[]))
    events=[];observations=[]
    for t_s in (0.,2.,4.,5.,6.,8.,10.):
        world.traffic.t_s=t_s
        experiment.prepare(world,events,observations)
    assert sorted({row['t_s'] for row in observations})==[0.,2.,4.,6.,8.,10.]
    updates=[row for row in events if row['status']=='policy_update']
    assert [row['t_s'] for row in updates]==[0.,5.,10.]
    assert [row['policy'] for row in updates]==['C','C','F']
    assert all(row['raw_policy']=='F' for row in updates)

def test_every_aircraft_evaluated_and_only_one_new_start(monkeypatch,rig):
    calls=[]
    def prepare(w,events,observations):
        if not w.traffic.active:
            w.traffic.active=[TrafficAircraft(r,i*3000.,50.,i%2)for i,r in enumerate('ABCDE')]
    def predict(w,end,candidate=None):
        calls.append(candidate)
        return {'admitted':True,'reasons':[],'cost':100. if candidate is None else 80.,'new_pairs':[]}
    monkeypatch.setattr(rig,'prepare',prepare)
    monkeypatch.setattr(rig,'predict',predict)
    r=rig.run([],True,horizon_s=.5,duration_factors=(1.5,2.))
    assert {c[0]for c in calls if c}==set('ABCDE')
    assert len([c for c in calls if c])==10
    starts=[e for e in r['events']if e['status']=='change_started']
    assert len(starts)==1
    assert starts[0]['aircraft_id']=='A'
    assert sum(d['reason']=='deferred_by_one_start_arbitration'for d in r['decisions'])==4


def test_recovery_search_does_not_stop_after_first_aircraft_fails(monkeypatch,rig):
    def prepare(w,events,observations):
        if not w.traffic.active:
            w.traffic.active=[TrafficAircraft(r,i*3000.,50.,i%2)for i,r in enumerate('ABCDE')]
    def predict(w,end,candidate=None):
        good=candidate is not None and candidate[0]=='E'
        return {'admitted':good,'reasons':[] if good else ['test_rejection'],
                'cost':80. if good else None,'new_pairs':[]}
    monkeypatch.setattr(rig,'prepare',prepare);monkeypatch.setattr(rig,'predict',predict)
    r=rig.run([],True,horizon_s=.5)
    selected=[d for d in r['decisions']if d['take_change']]
    assert len(selected)==1 and selected[0]['aircraft_id']=='E'
    assert selected[0]['reason']=='recovery_change'


def test_three_lane_middle_aircraft_checks_both_adjacent_lanes_and_protects_others(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,speed_min_mps=30.,speed_max_mps=80.)
    experiment=DispatchExperiment(scenario,cfg,
        AccelerationEnvelope(lateral_speed_max_mps=8.),offsets=(-300.,0.,300.))
    calls=[]
    def prepare(w,events,observations):
        if not w.traffic.active:
            w.traffic.active=[TrafficAircraft('E',1000.,50.,1,policy='R'),
                              TrafficAircraft('N',7000.,50.,0,policy='C')]
    def predict(w,end,candidate=None):
        calls.append(candidate)
        if candidate is None:
            return {'admitted':True,'reasons':[],'cost':20.,
                    'cost_by_aircraft':{'E':10.,'N':10.},'new_pairs':[]}
        target=candidate[1]
        costs=({'E':5.,'N':10.} if target==0 else {'E':4.,'N':11.})
        return {'admitted':True,'reasons':[],'cost':sum(costs.values()),
                'cost_by_aircraft':costs,'new_pairs':[]}
    monkeypatch.setattr(experiment,'prepare',prepare)
    monkeypatch.setattr(experiment,'predict',predict)
    result=experiment.run([],True,horizon_s=.5,duration_factors=(1.5,))
    targets={candidate[1] for candidate in calls if candidate and candidate[0]=='E'}
    assert targets=={0,2}
    starts=[event for event in result['events'] if event['status']=='change_started']
    assert len(starts)==1
    assert starts[0]['source_lane']==1 and starts[0]['target_lane']==0
    selected=next(row for row in result['decisions'] if row['aircraft_id']=='E')
    rejected=next(row for row in selected['candidates'] if row['target_lane']==2)
    assert rejected['worsened_aircraft']==['N']
    assert rejected['screening_reason']=='affected_aircraft_worsened'


def test_unchanged_state_waits_before_repeating_full_candidate_rollout(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,speed_min_mps=30.,speed_max_mps=80.)
    experiment=DispatchExperiment(scenario,cfg,
        AccelerationEnvelope(lateral_speed_max_mps=8.),offsets=(-300.,0.,300.))
    world_aircraft=TrafficAircraft('E',1000.,50.,1,policy='R')
    calls=[]
    def prepare(w,events,observations):
        if not w.traffic.active:
            w.traffic.active=[deepcopy(world_aircraft)]
    def predict(w,end,candidate=None):
        calls.append((w.traffic.t_s,candidate))
        return {'admitted':True,'reasons':[],'cost':10.,
                'cost_by_aircraft':{'E':10.},'new_pairs':[]}
    def step(w,end,events,observations,trace=None,new_pairs=None,
             cost_by_aircraft=None,maximum_step_s=None):
        w.traffic.t_s=min(end,w.traffic.t_s+.5)
        return [],0.
    monkeypatch.setattr(experiment,'prepare',prepare)
    monkeypatch.setattr(experiment,'predict',predict)
    monkeypatch.setattr(experiment,'step',step)
    experiment.run([],True,horizon_s=10.,duration_factors=(1.5,),
                   candidate_recheck_s=30.)
    assert {time for time,_ in calls}=={0.}


def test_no_loss_result_is_cached_for_candidate_recheck_period(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,
                       speed_min_mps=30.,speed_max_mps=80.)
    experiment=DispatchExperiment(scenario,cfg,
        AccelerationEnvelope(lateral_speed_max_mps=8.),offsets=(-300.,0.,300.))
    calls=[]
    def prepare(w,events,observations):
        if not w.traffic.active:
            w.traffic.active=[TrafficAircraft('E',1000.,50.,1,policy='C')]
        # Force irrelevant signature churn from a second aircraft. The no-loss
        # result for E must still honor the 30 s review cache.
        if w.traffic.t_s>=5. and len(w.traffic.active)==1:
            w.traffic.active.append(TrafficAircraft('N',7000.,50.,0,policy='C'))
    def predict(w,end,candidate=None):
        calls.append((w.traffic.t_s,candidate))
        costs={a.aircraft_id:0. for a in w.traffic.active}
        return {'admitted':True,'reasons':[],'cost':0.,
                'cost_by_aircraft':costs,'new_pairs':[]}
    def step(w,end,events,observations,trace=None,new_pairs=None,
             cost_by_aircraft=None,maximum_step_s=None):
        w.traffic.t_s=min(end,w.traffic.t_s+.5)
        return [],0.
    monkeypatch.setattr(experiment,'prepare',prepare)
    monkeypatch.setattr(experiment,'predict',predict)
    monkeypatch.setattr(experiment,'step',step)
    experiment.run([],True,horizon_s=10.,duration_factors=(1.5,),
                   candidate_recheck_s=30.)
    assert [time for time,candidate in calls if candidate is None]==[0.,5.]


def test_new_pair_gap_gate_expires_after_lane_change_settles(rig):
    world=World(TrafficState(100.,[
        TrafficAircraft('F',1000.,50.,0),
        TrafficAircraft('L',4000.,50.,0),
    ],[]),new_pairs={('F','L')})
    rig.prepare(world,[],[])
    assert world.new_pairs==set()


def test_progress_callback_reports_traffic_partition_and_final_state(rig):
    snapshots=[]
    result=rig.run([EntryRequest('A',0.,0),EntryRequest('B',30.,1)],False,
        horizon_s=2.,progress_callback=snapshots.append,progress_wall_s=60.)
    assert len(snapshots)>=2
    assert snapshots[0]['scheduled_requests']==2
    assert snapshots[0]['entered_requests']==0
    final=snapshots[-1]
    assert final['phase']=='finished'
    assert final['status']==result['status']=='completed_horizon'
    assert final['sim_time_s']==result['end_s']
    assert (final['completed_requests']+final['active_requests']
            +final['pending_requests']==2)
    assert final['entered_requests']==(
        final['completed_requests']+final['active_requests'])
    assert final['trace_rows']==len(result['trace'])
    assert final['observation_rows']==len(result['observations'])
    assert final['decision_rows']==len(result['decisions'])


def test_grid_topology_has_eight_center_neighbors_and_adjacent_edge_moves_only():
    root=Path(__file__).resolve().parents[1]
    scenario=load_scenario(root/'scenarios/airport_to_airport/scenario.json')
    cfg=DispatchConfig(dt_s=.5,policy_s=5.,decision_s=5.,speed_min_mps=30.,speed_max_mps=80.)
    points=[(d,h) for d in (-300.,0.,300.) for h in (200.,300.,400.)]
    experiment=DispatchExperiment(scenario,cfg,AccelerationEnvelope(),
        offsets=[p[0] for p in points],heights=[p[1] for p in points],
        candidate_topology='grid_8_connected')
    center=points.index((0.,300.))
    assert len(experiment.candidate_targets(center))==8
    corner=points.index((-300.,200.))
    assert len(experiment.candidate_targets(corner))==3
    assert center in experiment.candidate_targets(corner)
    assert experiment.candidate_duration(center,points.index((300.,300.)))>0
    assert experiment.candidate_duration(center,points.index((0.,400.)))>0


def test_capacity_trace_counts_one_origin_stream_without_adding_empty_grid_cells():
    # 100 + tau*50 + 0.04*50^2 gives 1000, 2000 and 4000 m.
    cfg=DispatchConfig(cruise_mps=50.,policy_s=5.,d0_m=100.,buffer_s2_per_m=.04,
                       tau_c_s=16.,tau_r_s=36.,tau_f_s=76.)
    trace=[]
    for timestamp,policies in [(0.,'CR'),(5.,'CC')]:
        for index,policy in enumerate(policies):
            trace.append({'t_s':timestamp,'aircraft_id':f'A{index}','policy':policy})
    rows,summary=planning_capacity_from_trace(trace,cfg,rho=.5)
    assert len(rows)==2
    assert rows[0]['q_mix_uam_h']==pytest.approx(120.)
    assert rows[1]['q_mix_uam_h']==pytest.approx(180.)
    assert summary['aircraft_snapshot_observations']==4
    assert summary['policy_counts']=={'C':3,'R':1,'F':0}


def test_capacity_sums_real_origin_streams_and_keeps_identity_after_lane_change():
    # 100 + tau*50 + 0.04*50^2 gives 1000, 2000 and 4000 m.
    cfg=DispatchConfig(cruise_mps=50.,policy_s=5.,d0_m=100.,buffer_s2_per_m=.04,
                       tau_c_s=16.,tau_r_s=36.,tau_f_s=76.)
    trace=[
        {'t_s':0.,'aircraft_id':'L1-A','lane':0,'policy':'C'},
        {'t_s':0.,'aircraft_id':'L2-A','lane':1,'policy':'R'},
        {'t_s':5.,'aircraft_id':'L1-A','lane':1,'policy':'R'},
        {'t_s':5.,'aircraft_id':'L2-A','lane':0,'policy':'C'},
        # This tail snapshot lacks origin stream 1 and must be excluded.
        {'t_s':10.,'aircraft_id':'L1-A','lane':0,'policy':'C'},
    ]
    origins={'L1-A':0,'L2-A':1}
    rows,summary=planning_capacity_by_origin_stream(
        trace,cfg,origins,origin_streams=[0,1],rho=.95)
    assert len(rows)==2
    assert rows[0]['corridor_capacity_uam_h']==pytest.approx(270.)
    assert rows[1]['corridor_capacity_uam_h']==pytest.approx(270.)
    assert summary['corridor_capacity_rho_uam_h']==pytest.approx(270.)
    assert summary['per_origin_stream']['0']['rho_uam_h']==pytest.approx(90.)
    assert summary['per_origin_stream']['1']['rho_uam_h']==pytest.approx(90.)
