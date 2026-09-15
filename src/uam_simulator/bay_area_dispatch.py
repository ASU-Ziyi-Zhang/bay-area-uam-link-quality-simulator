"""Scheduled five-aircraft geographic experiment, with explicit fixed policy targets.

Uses the existing geographic frame and radio model. Admission is a numerical
midpoint/endpoint audit plus conservative sampled separation clearance, NOT a
continuous flight certification. KS2-valid intervals and feedback fallback are
reported separately. No synthetic weak zone or initial placement is used.
"""
from dataclasses import dataclass, field, replace
from copy import copy, deepcopy
import math
import time
import numpy as np
from . import geographic_traffic as g
from .policy_motion import BSRadio
from .safety import NMAC_HORIZONTAL_M, NMAC_VERTICAL_M
from .spatial_study import SpatialMove, duration_min
from .five_uam_lateral import minimum_duration, envelope_utilisation
from .two_uam_longitudinal import resolve, plan_request
from .aks import tracking_command

@dataclass(frozen=True)
class DispatchConfig(g.GeographicTrafficConfig):
    # Fixed policy targets, evaluated once from the spacing law at cruise speed
    # (d0 + tau*v + b*v^2). Leave them unset so a change to d0, a headway or the
    # cruise speed reaches every consumer; a value given explicitly must agree
    # with the law and is kept only so archived configs still load.
    target_c_m: float | None = None
    target_r_m: float | None = None
    target_f_m: float | None = None
    # ``None`` preserves the archived implementation, where radio sampling and
    # policy updates shared ``policy_s``.  New Bay Area runs set this explicitly.
    radio_s: float | None = None
    persistence_k: int = 1
    def __post_init__(self):
        super().__post_init__()
        for policy in 'CRF':
            name=f'target_{policy.lower()}_m'
            law=g.GeographicTrafficConfig.spacing(self,policy,self.cruise_mps)
            given=getattr(self,name)
            if given is None:
                object.__setattr__(self,name,law)
            elif abs(given-law)>1e-6:
                raise ValueError(f'{name}={given} disagrees with the spacing law ({law})')
        if not 0 < self.target_c_m <= self.target_r_m <= self.target_f_m:
            raise ValueError('ordered positive policy distances required')
        if self.radio_s is not None and (not np.isfinite(self.radio_s) or self.radio_s <= 0):
            raise ValueError('radio_s must be positive and finite, or None')
        if not isinstance(self.persistence_k,int) or self.persistence_k < 1:
            raise ValueError('persistence_k must be a positive integer')
        radio_s=self.radio_period_s
        if abs(self.window_s/radio_s-round(self.window_s/radio_s))>1e-8:
            raise ValueError('policy window must align with radio samples')
    @property
    def radio_period_s(self):
        return self.policy_s if self.radio_s is None else self.radio_s
    def spacing(self, policy, speed=None):
        return {'C':self.target_c_m,'R':self.target_r_m,'F':self.target_f_m}[policy]

@dataclass
class World:
    traffic: g.TrafficState
    pairs: set = field(default_factory=set)
    plans: dict = field(default_factory=dict)
    keys: dict = field(default_factory=dict)
    last_observed: float = -math.inf
    last_policy_update: float = -math.inf
    latest_raw_policy: dict = field(default_factory=dict)
    policy_filters: dict = field(default_factory=dict)
    new_pairs: set = field(default_factory=set)
    candidate_reviews: dict = field(default_factory=dict)
    last_candidate_rollout_s: float = -math.inf
    last_candidate_rollout_signature: tuple = field(default_factory=tuple)

class DispatchExperiment:
    def __init__(self, scenario, cfg, envelope, offsets=(0.,100.), *,
                 heights=None, candidate_topology="linear_adjacent"):
        self.scenario,self.cfg,self.envelope=scenario,cfg,envelope
        self.offsets=np.asarray(offsets,float)
        if heights is None:
            self.heights=np.full(len(self.offsets),cfg.altitude_m)
        else:
            self.heights=np.asarray(heights,float)
        if (self.offsets.ndim != 1 or self.heights.ndim != 1
                or len(self.offsets) != len(self.heights) or not len(self.offsets)):
            raise ValueError('offsets and heights must be nonempty aligned vectors')
        if not np.isfinite(self.offsets).all() or not np.isfinite(self.heights).all():
            raise ValueError('flow-point coordinates must be finite')
        points=list(zip(self.offsets.tolist(),self.heights.tolist()))
        if len(set(points)) != len(points):
            raise ValueError('flow-point coordinates must be unique')
        if candidate_topology not in ('linear_adjacent','grid_8_connected'):
            raise ValueError('unsupported candidate topology')
        self.candidate_topology=candidate_topology
        self.frame=g.SmoothCorridorFrame(scenario.corridor,cfg.center_control_step_m)
        self.geometry=self.frame.report(max(abs(self.offsets)))
        self.arc=g.OffsetArcCoordinates(self.frame,self.offsets)
        self.radio=BSRadio(scenario.base_stations,scenario.radio)
        # Encode the explicit metre table for the existing equal-speed solver.
        self.pair_cfg=resolve({'controller':{},'vehicle':{'cruise_mps':cfg.cruise_mps,
            'speed_min_mps':cfg.speed_min_mps,'speed_max_mps':cfg.speed_ceiling_mps,
            'acceleration_limit_mps2':cfg.acceleration_limit_mps2,
            'deceleration_limit_mps2':cfg.deceleration_limit_mps2},
            'spacing':{}})
        self.pair_cfg=replace(self.pair_cfg,legacy=cfg)

    def candidate_targets(self, lane):
        """Return one-step neighbors without conflating grid resolution and range."""
        if not 0 <= lane < len(self.offsets):
            raise ValueError('lane index is outside the configured flow points')
        if self.candidate_topology == 'linear_adjacent':
            return [target for target in (lane-1,lane+1)
                    if 0 <= target < len(self.offsets)]
        lateral=sorted(set(self.offsets.tolist()))
        vertical=sorted(set(self.heights.tolist()))
        li={value:index for index,value in enumerate(lateral)}
        vi={value:index for index,value in enumerate(vertical)}
        source=(li[float(self.offsets[lane])],vi[float(self.heights[lane])])
        targets=[]
        for target,(offset,height) in enumerate(zip(self.offsets,self.heights)):
            if target == lane:
                continue
            cell=(li[float(offset)],vi[float(height)])
            if max(abs(cell[0]-source[0]),abs(cell[1]-source[1])) == 1:
                targets.append(target)
        return sorted(targets)

    def candidate_duration(self, source_lane, target_lane):
        source=g.flow_point(source_lane,self.offsets,self.heights)
        target=g.flow_point(target_lane,self.offsets,self.heights)
        if self.candidate_topology == 'linear_adjacent':
            return minimum_duration(abs(target[0]-source[0]),self.envelope)['duration_s']
        return duration_min(source,target,self.cfg)

    ROUTE_GRID_M=.5

    def route_curvature(self):
        """Centerline curvature on a fixed 0.5 m chainage grid, built once."""
        if getattr(self,'_route_grid',None) is None:
            q=np.arange(0.,self.frame.length_m,self.ROUTE_GRID_M)
            q=np.r_[q,self.frame.length_m]
            self._route_grid=(q,np.asarray(self.frame.frame(q)[4],float))
        return self._route_grid

    def route_constraints(self, q_m, offset_m, speed_mps):
        """Return braking-feasible speed and acceleration constraints.

        The route-normal acceleration limit is a speed constraint as well as
        an audit condition. ``V(x)`` is the highest speed at path distance
        ``x`` ahead from which braking at the deceleration limit keeps every
        later point at or below its curve speed. The speed ceiling is ``V(0)``.
        The acceleration command is held for a control period, so it must keep
        ``v^2 + 2 a x <= V(x)^2`` at every point the aircraft can reach before
        the next command, not only at the next curvature sample. Curvature is
        read on a 0.5 m grid and each node takes the larger of its neighbours,
        so an apex between nodes is not under-read.
        """
        c=self.cfg;e=self.envelope;D=c.deceleration_limit_mps2
        grid_q,grid_k=self.route_curvature()
        q0=float(q_m);v=float(speed_mps)
        remaining=max(0.,self.frame.length_m-q0)
        braking=(c.speed_ceiling_mps**2-c.speed_min_mps**2)/(2*D)
        lookahead=min(remaining,max(100.,braking+100.))
        first=max(0,int(math.floor(q0/self.ROUTE_GRID_M)))
        last=min(len(grid_q)-1,int(math.ceil((q0+lookahead)/self.ROUTE_GRID_M))+1)
        q=grid_q[first:last+1];curvature=grid_k[first:last+1]
        if len(q)<2:
            return float(c.speed_ceiling_mps),float(c.acceleration_limit_mps2)
        denominator=1-curvature*float(offset_m)
        effective=np.abs(curvature/denominator)
        bound=effective.copy()
        bound[1:]=np.maximum(bound[1:],effective[:-1])
        bound[:-1]=np.maximum(bound[:-1],effective[1:])
        curve_speed_sq=np.full(bound.shape,math.inf,float)
        mask=bound>1e-14
        curve_speed_sq[mask]=e.route_normal_accel_max_mps2/bound[mask]
        # ``q`` is centerline chainage; distance must follow the current offset
        # curve, whose local metric is |1-kappa*d| dq.
        metric=np.abs(denominator)
        path=np.zeros_like(q)
        path[1:]=np.cumsum(.5*(metric[:-1]+metric[1:])*np.diff(q))
        ahead=np.maximum(0.,path-np.interp(q0,q,path))
        # V(x)^2 = min over later points j of (curve_speed_j^2 + 2 D x_j) - 2 D x.
        later=np.minimum.accumulate((curve_speed_sq+2*D*ahead)[::-1])[::-1]
        admissible_sq=later-2*D*ahead
        speed_ceiling=float(min(c.speed_ceiling_mps,math.sqrt(max(0.,admissible_sq[0]))))
        period=max(c.dt_s,c.control_period_s or 0.)
        reach=v*period+.5*c.acceleration_limit_mps2*period**2
        # Points reached before the next command, plus the first one beyond so
        # the last partial cell is covered (``ahead`` is nondecreasing).
        future=np.flatnonzero(ahead>1e-9)
        within=future[:np.searchsorted(ahead[future],reach,side='right')+1]
        acceleration_ceiling=float(c.acceleration_limit_mps2)
        if len(within):
            acceleration_ceiling=float(min(acceleration_ceiling,np.min(
                (admissible_sq[within]-v*v)/(2*ahead[within]))))
        if v*v>admissible_sq[0]:
            acceleration_ceiling=min(acceleration_ceiling,-D)
        return speed_ceiling,acceleration_ceiling

    @staticmethod
    def on_clock(t_s,period_s):
        return abs(t_s/period_s-round(t_s/period_s))<1e-8

    @staticmethod
    def audit_state(s):
        """Copy only state mutated by a sub-step physical audit.

        The audit advances ``q_m``, speed, and last acceleration on active
        aircraft. It does not sample radio, mutate history, admit requests, or
        touch completed aircraft. Sharing those immutable/read-only objects
        preserves the calculation while avoiding three full traffic-state
        deep copies per forecast step.
        """
        return g.TrafficState(
            s.t_s,[copy(a) for a in s.active],list(s.pending),
            list(s.completed),dict(s.deferrals))

    @staticmethod
    def clone_world(w):
        """Copy the mutable forecast state without recursively copying constants."""
        active=[]
        for aircraft in w.traffic.active:
            cloned=copy(aircraft)
            cloned.history=list(aircraft.history)
            active.append(cloned)
        traffic=g.TrafficState(
            w.traffic.t_s,active,list(w.traffic.pending),
            list(w.traffic.completed),dict(w.traffic.deferrals))
        return World(
            traffic=traffic,pairs=set(w.pairs),plans=dict(w.plans),
            keys=dict(w.keys),last_observed=w.last_observed,
            last_policy_update=w.last_policy_update,
            latest_raw_policy=dict(w.latest_raw_policy),
            policy_filters={key:dict(value)
                            for key,value in w.policy_filters.items()},
            new_pairs=set(w.new_pairs),
            candidate_reviews=dict(w.candidate_reviews),
            last_candidate_rollout_s=w.last_candidate_rollout_s,
            last_candidate_rollout_signature=w.last_candidate_rollout_signature)

    @staticmethod
    def next_step(s,c,maximum_step_s):
        """Return the next event-aligned step up to the requested resolution."""
        end=s.t_s+maximum_step_s
        for period in (c.policy_s,c.decision_s):
            next_clock=(math.floor((s.t_s+1e-8)/period)+1)*period
            end=min(end,next_clock)
        for a in s.active:
            if a.move is not None:
                end=min(end,a.move.start_s+a.move.duration_s)
        future=[row.requested_time_s for row in s.pending
                if row.requested_time_s>s.t_s+1e-8]
        if future:end=min(end,min(future))
        return end-s.t_s

    def sample_policy(self,w):
        """Record radio/group exposure without changing the operational policy."""
        s=w.traffic;c=self.cfg
        previous={a.aircraft_id:a.policy for a in s.active}
        rows=g.observe_groups(s,self.frame,self.offsets,self.heights,c,self.radio)
        for row in rows:
            raw=row['policy']
            row['raw_policy']=raw
            w.latest_raw_policy[row['aircraft_id']]=raw
        for a in s.active:
            a.policy=previous[a.aircraft_id]
        return rows

    def update_policy(self,w,events):
        """Apply k-update persistence using the newest available radio sample."""
        s=w.traffic;c=self.cfg
        for a in s.active:
            raw=w.latest_raw_policy.get(a.aircraft_id)
            if raw is None:
                continue
            record=w.policy_filters.setdefault(a.aircraft_id,{
                'state':a.policy,'candidate':a.policy,'count':0})
            previous=record['state']
            if raw==record['state']:
                record['candidate'],record['count']=record['state'],0
            else:
                if raw==record['candidate']:
                    record['count']+=1
                else:
                    record['candidate'],record['count']=raw,1
                if record['count']>=c.persistence_k:
                    record['state'],record['candidate'],record['count']=raw,raw,0
            a.policy=record['state']
            events.append({'t_s':s.t_s,'status':'policy_update',
                'aircraft_id':a.aircraft_id,'raw_policy':raw,
                'previous_policy':previous,'policy':a.policy,
                'candidate_policy':record['candidate'],
                'candidate_count':record['count'],
                'persistence_k':c.persistence_k})

    def prepare(self,w,events,observations):
        s=w.traffic;c=self.cfg
        g._settle(s,c,events)
        # ``new_pairs`` is an admission gate for relationships being created
        # by an active lane change.  Once every aircraft in a pair has settled,
        # later policy-target changes belong to the longitudinal controller;
        # retaining the pair forever would falsely turn a normal C->R spacing
        # response into a forecast infeasibility.
        moving={a.aircraft_id for a in s.active if a.move is not None}
        w.new_pairs.intersection_update(
            pair for pair in w.new_pairs if pair[0] in moving or pair[1] in moving)
        for request in list(s.pending):
            if request.requested_time_s>s.t_s+1e-8:continue
            candidate=g.TrafficAircraft(request.aircraft_id,0.,c.cruise_mps,request.lane,
                requested_time_s=request.requested_time_s,entry_time_s=s.t_s)
            # Evaluate the new aircraft's actual radio/group policy at entry;
            # do not grant it an artificial C policy to pass admission.
            trial=deepcopy(s);trial.active.append(candidate)
            g.observe_groups(trial,self.frame,self.offsets,self.heights,c,self.radio)
            candidate=trial.active[-1]
            deficits=g.insertion_deficits(candidate,candidate.lane,trial,self.arc,c)
            own_position=g._physical_state(candidate,s.t_s,self.frame,self.offsets,self.heights)[0]
            conflict=False
            for other in s.active:
                position=g._physical_state(other,s.t_s,self.frame,self.offsets,self.heights)[0]
                delta=own_position-position
                conflict |= abs(delta[2])<=c.vertical_separation_m and np.linalg.norm(delta[:2])<=c.horizontal_separation_m+c.speed_ceiling_mps*c.dt_s
            if deficits or conflict:
                s.deferrals[request.aircraft_id]=s.deferrals.get(request.aircraft_id,0)+1
                continue
            candidate.history=[]  # the accepted observation is recorded below
            s.active.append(candidate);s.pending.remove(request)
            w.latest_raw_policy[candidate.aircraft_id]=candidate.policy
            w.policy_filters[candidate.aircraft_id]={
                'state':candidate.policy,'candidate':candidate.policy,'count':0}
            events.append({'t_s':s.t_s,'status':'corridor_entry','aircraft_id':request.aircraft_id,
                'requested_time_s':request.requested_time_s,'entry_delay_s':s.t_s-request.requested_time_s,
                'lane':request.lane,'entry_policy':candidate.policy})
        sampled=[]
        if s.active and s.t_s>w.last_observed+1e-8 and self.on_clock(s.t_s,c.radio_period_s):
            sampled=self.sample_policy(w)
            observations.extend(sampled)
            w.last_observed=s.t_s
        if s.active and s.t_s>w.last_policy_update+1e-8 and self.on_clock(s.t_s,c.policy_s):
            self.update_policy(w,events)
            current={a.aircraft_id:a.policy for a in s.active}
            for row in sampled:
                row['policy']=current[row['aircraft_id']]
            w.last_policy_update=s.t_s

    def controls(self,w):
        s=w.traffic;c=self.cfg;e=self.envelope
        initial=g.acc_controls(s,self.arc,c)
        current={(a.aircraft_id,row[0]) for a in s.active for row in initial[a.aircraft_id]['leaders']}
        w.pairs.intersection_update(current)
        for a in s.active:
            for lead,gap,_,_ in initial[a.aircraft_id]['leaders']:
                if gap <= c.spacing(a.policy)+1e-8:w.pairs.add((a.aircraft_id,lead))
        controls=g.acc_controls(s,self.arc,c,recovery_pairs=frozenset(w.pairs))
        lookup={a.aircraft_id:a for a in s.active}
        for a in s.active:
            row=controls[a.aircraft_id];ids=tuple(x[0] for x in row['leaders'])
            key=(ids,a.policy,tuple(x for x in ids if (a.aircraft_id,x) in w.pairs))
            mode='feedback_fallback' if ids else 'cruise'
            if w.keys.get(a.aircraft_id)!=key:
                w.keys[a.aircraft_id]=key;w.plans.pop(a.aircraft_id,None)
                if len(ids)==1 and key[2] and a.move is None:
                    lead=lookup[ids[0]]
                    request=plan_request(self.pair_cfg,row['leaders'][0][1],a.policy,'exact_target',a.speed_mps,lead.speed_mps)
                    if request['status'] in ('planned','hold'):
                        w.plans[a.aircraft_id]=(s.t_s,request,ids[0],self.arc.position(lead.q_m,a.lane))
            plan=w.plans.get(a.aircraft_id)
            if plan:
                start,request,lid,lq=plan;lead=lookup.get(lid);u=s.t_s-start
                if lead is None or a.move is not None or abs(lead.speed_mps-c.cruise_mps)>1e-6 or abs(self.arc.position(lead.q_m,a.lane)-lq-c.cruise_mps*u)>1e-4:
                    w.plans.pop(a.aircraft_id,None)
                else:
                    tr=request['transition']
                    if tr is None: vr,ar,x=c.cruise_mps,0.,c.cruise_mps*u
                    elif u>=tr.duration:vr,ar,x=c.cruise_mps,0.,tr.displacement()+c.cruise_mps*(u-tr.duration)
                    else:vr,ar,x=map(float,tr.sample(u))
                    ref=request['initial_gap_m']+c.cruise_mps*u-x
                    row['command']=float(tracking_command(ar,row['leaders'][0][1],ref,vr,a.speed_mps,c.k_gap_per_s2,c.k_relative_per_s))
                    mode='ks2_tracker' if tr else 'hold_tracker'
            dh,vd,ad=a.transverse(s.t_s,self.offsets,self.heights)
            curvature=float(self.frame.frame(a.q_m)[4]);cross=curvature*a.speed_mps*vd[0]/(1-curvature*dh[0])
            normal=ad[0] if e.coupling_mode=='maneuver_only' else ad[0]+curvature*a.speed_mps**2/(1-curvature*dh[0])
            if a.move is not None:
                # Reserve the full maneuver peak, not only its current value.
                peak_normal=10/math.sqrt(3)*abs(a.move.target[0]-a.move.source[0])/a.move.duration_s**2
                normal=math.copysign(max(abs(normal),peak_normal),normal)
            reserve=math.sqrt(max(0.,1-(normal/e.lateral_accel_max_mps2)**2))
            lower=max(-c.deceleration_limit_mps2,
                      cross-e.longitudinal_decel_max_mps2*reserve)
            upper=min(c.acceleration_limit_mps2,
                      cross+e.longitudinal_accel_max_mps2*reserve)
            route_speed,route_command=self.route_constraints(
                a.q_m,dh[0],a.speed_mps)
            route_command=max(-c.deceleration_limit_mps2,route_command)
            upper=min(upper,route_command)
            row['command']=float(np.clip(row['command'],lower,max(lower,upper)))
            row['route_speed_ceiling_mps']=route_speed
            if (a.speed_mps<=c.speed_min_mps+1e-8 and row['command']<0) or (a.speed_mps>=c.speed_ceiling_mps-1e-8 and row['command']>0):row['command']=0.
            row['mode']=mode
        return controls

    def inspect(self,s,controls,new_pairs=(),clearance=0.):
        c=self.cfg;e=self.envelope;reasons=[];peak=0.;positions=[]
        for a in s.active:
            xyz,v,acc,dh,vd,ad=g._physical_state(a,s.t_s,self.frame,self.offsets,self.heights,controls[a.aircraft_id]['command'])
            positions.append(xyz)
            _,tangent,_,_,k=self.frame.frame(a.q_m)
            at=float(acc[:2]@tangent);route=float(k*a.speed_mps**2/(1-k*dh[0]))
            normal=ad[0] if e.coupling_mode=='maneuver_only' else route+ad[0]
            eta=float(envelope_utilisation(at,normal,e));peak=max(peak,eta)
            if eta>1+1e-6:reasons.append('joint_envelope:'+a.aircraft_id)
            if abs(route)>e.route_normal_accel_max_mps2+1e-6:reasons.append('route_acceleration:'+a.aircraft_id)
            if not c.speed_min_mps-1e-7<=np.linalg.norm(v)<=c.speed_ceiling_mps+1e-7:reasons.append('total_speed:'+a.aircraft_id)
            if abs(vd[0])>e.lateral_speed_max_mps+1e-7:reasons.append('lateral_speed:'+a.aircraft_id)
            if abs(vd[1])>c.vertical_speed_limit_mps+1e-7:reasons.append('vertical_speed:'+a.aircraft_id)
            if abs(ad[1])>e.vertical_accel_max_mps2+1e-7:reasons.append('vertical_acceleration:'+a.aircraft_id)
        minimum=math.inf;nmac=False
        for i in range(len(positions)):
            for j in range(i):
                delta=positions[i]-positions[j];dist=float(np.linalg.norm(delta[:2]));minimum=min(minimum,dist)
                if abs(delta[2])<=c.vertical_separation_m and dist<=c.horizontal_separation_m+clearance:reasons.append('separation_or_unresolved_interval:'+s.active[i].aircraft_id+':'+s.active[j].aircraft_id)
                nmac|=dist<=NMAC_HORIZONTAL_M and abs(delta[2])<=NMAC_VERTICAL_M
        for a in s.active:
            for lead,gap,_,_ in controls[a.aircraft_id]['leaders']:
                if (a.aircraft_id,lead) in new_pairs and gap<c.spacing(a.policy)-1e-7:reasons.append('new_following_gap:'+a.aircraft_id+':'+lead)
        return list(dict.fromkeys(reasons)),peak,minimum,nmac

    def step(self,w,end,events,observations,trace=None,new_pairs=None,
             cost_by_aircraft=None,maximum_step_s=None):
        if new_pairs is None:new_pairs=w.new_pairs
        self.prepare(w,events,observations);s=w.traffic;c=self.cfg
        controls=self.controls(w)
        step_limit=c.dt_s if maximum_step_s is None else maximum_step_s
        h=min(self.next_step(s,c,step_limit),end-s.t_s)
        next_radio=(math.floor((s.t_s+1e-8)/c.radio_period_s)+1)*c.radio_period_s
        h=min(h,next_radio-s.t_s)
        for a in s.active:
            ac=controls[a.aircraft_id]['command']
            if ac:
                hit=((c.speed_ceiling_mps if ac>0 else c.speed_min_mps)-a.speed_mps)/ac
                if 1e-8<hit<h:h=hit
        # Audit current, midpoint and endpoint. A speed-based clearance guard
        # prevents accepting a near-threshold interval just from its samples.
        checks=[]
        for refinement in range(10):
            checks=[]
            for f in (0.,.5,1.):
                trial=self.audit_state(s)
                if f:g._progress_collection(self.frame,trial.active,s.t_s,h*f,controls,self.offsets,self.heights,c)
                trial.t_s+=h*f
                tc=g.acc_controls(trial,self.arc,c)
                for a in trial.active:tc[a.aircraft_id]['command']=controls[a.aircraft_id]['command']
                checks.append(self.inspect(trial,tc,new_pairs,clearance=c.speed_ceiling_mps*h/2))
            reasons=list(dict.fromkeys(x for ck in checks for x in ck[0]))
            if not reasons:break
            if all(x.startswith('separation_or_unresolved_interval:') for x in reasons) and min(ck[2] for ck in checks)>c.horizontal_separation_m:
                h/=2
            else:break
        if reasons:return reasons,0.
        increments={a.aircraft_id:c.policy_loss(a.policy)*h for a in s.active}
        # Unserved due requests cannot gain a lower objective by staying outside.
        for request in s.pending:
            if request.requested_time_s<=s.t_s:
                increments[request.aircraft_id]=increments.get(request.aircraft_id,0.)+2*h
        if cost_by_aircraft is not None:
            for identifier,value in increments.items():
                cost_by_aircraft[identifier]=cost_by_aircraft.get(identifier,0.)+value
        cost=sum(increments.values())
        if trace is not None:
            for a in s.active:
                xyz,v,acc,dh,vd,ad=g._physical_state(a,s.t_s,self.frame,self.offsets,self.heights,controls[a.aircraft_id]['command'])
                minimum_step=min(x[2] for x in checks)
                trace.append({'t_s':s.t_s,'dt_s':h,'aircraft_id':a.aircraft_id,'q_m':a.q_m,'lane':a.lane,'target_lane':a.target_lane,'offset_m':float(dh[0]),'xyz':xyz.tolist(),'v_mps':a.speed_mps,'total_speed_mps':float(np.linalg.norm(v)),'a_xyz':acc.tolist(),'policy':a.policy,'raw_policy':w.latest_raw_policy.get(a.aircraft_id),'exposure':a.exposure,'target_gap_m':c.spacing(a.policy),'leaders':controls[a.aircraft_id]['leaders'],'controller':controls[a.aircraft_id]['mode'],'route_speed_ceiling_mps':controls[a.aircraft_id]['route_speed_ceiling_mps'],'peak_eta_step':max(x[1] for x in checks),'minimum_separation_step_m':None if not math.isfinite(minimum_step) else minimum_step,'nmac_sampled':any(x[3] for x in checks)})
        g._progress_collection(self.frame,s.active,s.t_s,h,controls,self.offsets,self.heights,c)
        s.t_s+=h;g._settle(s,c,events)
        for a in list(s.active):
            if a.q_m>=self.frame.length_m:
                a.q_m=self.frame.length_m;a.completion_time_s=s.t_s;s.active.remove(a);s.completed.append(a)
                events.append({'t_s':s.t_s,'status':'exit','aircraft_id':a.aircraft_id})
        return [],cost

    def predict(self,w,end,candidate=None,heartbeat=None):
        trial=self.clone_world(w);s=trial.traffic;new=set()
        if candidate:
            ident,target,duration=candidate;a=next(a for a in s.active if a.aircraft_id==ident)
            old={(x.aircraft_id,row[0])for x in s.active for row in g.acc_controls(s,self.arc,self.cfg)[x.aircraft_id]['leaders']}
            a.move=SpatialMove(s.t_s,g.flow_point(a.lane,self.offsets,self.heights),g.flow_point(target,self.offsets,self.heights),duration);a.target_lane=target
            # Track newly created relationships during crossing and after exit.
            during={(x.aircraft_id,row[0])for x in s.active for row in g.acc_controls(s,self.arc,self.cfg)[x.aircraft_id]['leaders']}
            settled=self.audit_state(s);aa=next(x for x in settled.active if x.aircraft_id==ident);aa.lane=target;aa.target_lane=None
            after={(x.aircraft_id,row[0])for x in settled.active for row in g.acc_controls(settled,self.arc,self.cfg)[x.aircraft_id]['leaders']}
            new=(during|after)-old
            trial.new_pairs.update(new)
        cost=0.;events=[];obs=[]
        cost_by_aircraft={a.aircraft_id:0. for a in s.active}
        cost_by_aircraft.update({a.aircraft_id:0. for a in s.pending})
        while s.t_s<end-1e-8:
            issue,part=self.step(trial,end,events,obs,new_pairs=trial.new_pairs,
                                 cost_by_aircraft=cost_by_aircraft,
                                 maximum_step_s=self.cfg.forecast_dt_s);cost+=part
            if heartbeat is not None:
                heartbeat(s.t_s,end)
            if issue:return {'admitted':False,'reasons':issue,'cost':None}
        return {'admitted':True,'reasons':[],'cost':cost,
                'cost_by_aircraft':cost_by_aircraft,
                'new_pairs':sorted(trial.new_pairs)}

    def run(self,entries,allow_change,*,horizon_s,prediction_s=90.,
            duration_factors=(1.5,2.),gain_margin=1.,candidate_recheck_s=0.,
            progress_callback=None,progress_wall_s=30.):
        if not np.isfinite(candidate_recheck_s) or candidate_recheck_s<0:
            raise ValueError('candidate_recheck_s must be finite and nonnegative')
        if not np.isfinite(progress_wall_s) or progress_wall_s<=0:
            raise ValueError('progress_wall_s must be positive and finite')
        w=World(g.TrafficState(0.,[],sorted(entries,key=lambda r:(r.requested_time_s,r.aircraft_id))))
        events=[];obs=[];trace=[];decisions=[];last=-math.inf;status='completed_horizon'
        wall_started=time.monotonic();last_progress_wall=-math.inf
        progress_context={'phase':'main_simulation'}

        def emit_progress(*,force=False,**updates):
            nonlocal last_progress_wall
            progress_context.update(updates)
            if progress_callback is None:
                return
            wall_now=time.monotonic()
            if not force and wall_now-last_progress_wall<progress_wall_s:
                return
            s=w.traffic
            policy_counts={policy:sum(a.policy==policy for a in s.active)
                           for policy in ('C','R','F')}
            payload={
                'status':(
                    progress_context.get('final_status')
                    or ('running' if status=='completed_horizon' else status)),
                'wall_elapsed_s':wall_now-wall_started,
                'sim_time_s':s.t_s,
                'sim_horizon_s':horizon_s,
                'sim_progress_fraction':min(1.,s.t_s/horizon_s) if horizon_s else 1.,
                'scheduled_requests':len(entries),
                'entered_requests':len(s.active)+len(s.completed),
                'completed_requests':len(s.completed),
                'active_requests':len(s.active),
                'pending_requests':len(s.pending),
                'lane_changes_started':sum(e['status']=='change_started' for e in events),
                'lane_changes_completed':sum(e['status']=='transition_completed' for e in events),
                'policy_counts_active':policy_counts,
                'trace_rows':len(trace),
                'observation_rows':len(obs),
                'decision_rows':len(decisions),
                'active_aircraft':[
                    {'aircraft_id':a.aircraft_id,'q_m':a.q_m,'lane':a.lane,
                     'target_lane':a.target_lane,'speed_mps':a.speed_mps,
                     'policy':a.policy,'exposure':a.exposure}
                    for a in s.active],
                'next_pending':(
                    {'aircraft_id':s.pending[0].aircraft_id,
                     'requested_time_s':s.pending[0].requested_time_s,
                     'lane':s.pending[0].lane}
                    if s.pending else None),
                'recent_events':events[-25:],
                'recent_decisions':[
                    {'t_s':row['t_s'],'aircraft_id':row['aircraft_id'],
                     'reason':row.get('reason'),'take_change':row.get('take_change',False)}
                    for row in decisions[-25:]],
                **{key:value for key,value in progress_context.items()
                   if key!='final_status'},
            }
            progress_callback(payload)
            last_progress_wall=wall_now

        def forecast_heartbeat(forecast_time_s,forecast_end_s):
            emit_progress(forecast_time_s=forecast_time_s,
                          forecast_end_s=forecast_end_s)

        def run_prediction(end,candidate=None):
            if progress_callback is None:
                return self.predict(w,end,candidate)
            return self.predict(w,end,candidate,heartbeat=forecast_heartbeat)

        emit_progress(force=True)
        while w.traffic.t_s<horizon_s-1e-8:
            s=w.traffic;self.prepare(w,events,obs)
            if not s.active and not s.pending:status='all_exited';break
            rollout_signature=tuple(sorted((a.aircraft_id,a.policy,a.lane,a.target_lane)
                                           for a in s.active))
            def review_is_due(aircraft):
                previous=w.candidate_reviews.get(aircraft.aircraft_id)
                return not (previous is not None
                            and previous[1]==aircraft.policy
                            and s.t_s-previous[0]<candidate_recheck_s-1e-8)
            reviewable=any(
                a.move is None and s.t_s>=a.cooldown_until_s-1e-8
                and review_is_due(a) for a in s.active)
            rollout_due=(candidate_recheck_s<=0
                         or s.t_s-w.last_candidate_rollout_s>=candidate_recheck_s-1e-8
                         or rollout_signature!=w.last_candidate_rollout_signature)
            rollout_due=rollout_due and reviewable
            if allow_change and rollout_due and abs(s.t_s/self.cfg.decision_s-round(s.t_s/self.cfg.decision_s))<1e-8 and s.t_s>last+1e-8:
                last=s.t_s
                w.last_candidate_rollout_s=s.t_s
                w.last_candidate_rollout_signature=rollout_signature
                adjacent_moves=[(source,target)
                                for source in range(len(self.offsets))
                                for target in self.candidate_targets(source)]
                if not adjacent_moves:
                    raise ValueError('lane changing requires at least two lanes')
                maximum_lower=max(self.candidate_duration(source,target)
                                  for source,target in adjacent_moves)
                end=s.t_s+max(prediction_s,maximum_lower*max(duration_factors)
                              +self.cfg.window_s+self.cfg.decision_s)
                for own in s.active:
                    if own.move:end=max(end,own.move.start_s+own.move.duration_s+self.cfg.window_s)
                emit_progress(phase='baseline_forecast',decision_time_s=s.t_s,
                              ego_aircraft_id=None,target_lane=None,
                              candidate_duration_s=None,forecast_time_s=s.t_s,
                              forecast_end_s=end)
                base=run_prediction(end)
                base_public={key:value for key,value in base.items()
                             if key!='cost_by_aircraft'}
                base_costs=base.get('cost_by_aircraft',{})
                proposals=[];records=[]
                for ego in sorted(s.active,key=lambda a:a.aircraft_id):
                    record={'t_s':s.t_s,'aircraft_id':ego.aircraft_id,'ego_policy':ego.policy,
                        'baseline':base_public,'candidates':[],'take_change':False,
                        'prediction_end_s':end}
                    if ego.move is not None:
                        record['reason']='committed_maneuver'
                    elif s.t_s<ego.cooldown_until_s:
                        record['reason']='cooldown'
                    elif self.arc.remaining(ego.q_m,ego.lane)<=self.cfg.speed_ceiling_mps*(end-s.t_s):
                        record['reason']='insufficient_route_for_full_candidate'
                        w.candidate_reviews[ego.aircraft_id]=(s.t_s,ego.policy)
                    elif (base['admitted'] and base_costs
                          and base_costs.get(ego.aircraft_id,0.)<=1e-9):
                        record['reason']='no_predicted_policy_loss'
                        w.candidate_reviews[ego.aircraft_id]=(s.t_s,ego.policy)
                    elif (ego.aircraft_id in w.candidate_reviews
                          and w.candidate_reviews[ego.aircraft_id][1]==ego.policy
                          and s.t_s-w.candidate_reviews[ego.aircraft_id][0]
                              <candidate_recheck_s-1e-8):
                        record['reason']='candidate_recheck_wait'
                    else:
                        w.candidate_reviews[ego.aircraft_id]=(s.t_s,ego.policy)
                        targets=self.candidate_targets(ego.lane)
                        candidate_details=[]
                        for target in targets:
                            lower=self.candidate_duration(ego.lane,target)
                            for factor in duration_factors:
                                duration=lower*factor
                                emit_progress(phase='candidate_forecast',
                                    decision_time_s=s.t_s,
                                    ego_aircraft_id=ego.aircraft_id,
                                    target_lane=target,
                                    candidate_duration_s=duration,
                                    forecast_time_s=s.t_s,forecast_end_s=end)
                                outcome=run_prediction(
                                    end,(ego.aircraft_id,target,duration))
                                option={key:value for key,value in outcome.items()
                                        if key!='cost_by_aircraft'}
                                option.update(duration_s=duration,target_lane=target)
                                candidate_costs=outcome.get('cost_by_aircraft',{})
                                if outcome['admitted'] and base['admitted'] and base_costs:
                                    option['ego_policy_cost_improvement_s']=(
                                        base_costs.get(ego.aircraft_id,0.)
                                        -candidate_costs.get(ego.aircraft_id,0.))
                                    compared=set(base_costs)|set(candidate_costs)
                                    option['worsened_aircraft']=[identifier for identifier in sorted(compared)
                                        if candidate_costs.get(identifier,0.)
                                        >base_costs.get(identifier,0.)+1e-8]
                                    option['affected_aircraft']=[identifier for identifier in sorted(compared)
                                        if abs(candidate_costs.get(identifier,0.)
                                               -base_costs.get(identifier,0.))>1e-8]
                                candidate_details.append((option,outcome))
                                record['candidates'].append(option)
                        accepted=[]
                        for option,outcome in candidate_details:
                            if not option['admitted']:
                                continue
                            if base['admitted'] and base_costs:
                                if option.get('worsened_aircraft'):
                                    option['screening_reason']='affected_aircraft_worsened'
                                    continue
                                if option['ego_policy_cost_improvement_s']<=gain_margin:
                                    option['screening_reason']='ego_gain_below_margin'
                                    continue
                            elif (base['admitted'] and base['cost']-option['cost']
                                  <=gain_margin):
                                option['screening_reason']='global_gain_below_margin'
                                continue
                            accepted.append(option)
                        chosen=min(accepted,key=lambda p:p['cost']) if accepted else None
                        record['reason']='no_admissible_candidate'
                        if chosen:
                            gain=base['cost']-chosen['cost'] if base['admitted'] else None
                            record['gain']=gain
                            record['reason']='eligible_recovery' if not base['admitted'] else 'eligible_benefit'
                            proposals.append((chosen['cost'],ego.aircraft_id,chosen,record))
                    records.append(record)
                # One new start per clock: no two independently evaluated
                # candidates can reserve the same gap. Each forecast includes
                # all already committed maneuvers and every aircraft response.
                if proposals:
                    _,ident,chosen,record=min(proposals,key=lambda p:(p[0],p[1]))
                    ego=next(a for a in s.active if a.aircraft_id==ident)
                    record['take_change']=True
                    record['reason']='beneficial_change' if base['admitted'] else 'recovery_change'
                    record['selected_candidate']=chosen
                    for _,other,_,other_record in proposals:
                        if other!=ident:other_record['reason']='deferred_by_one_start_arbitration'
                    w.new_pairs.update(map(tuple,chosen['new_pairs']))
                    ego.move=SpatialMove(s.t_s,g.flow_point(ego.lane,self.offsets,self.heights),g.flow_point(chosen['target_lane'],self.offsets,self.heights),chosen['duration_s'])
                    ego.target_lane=chosen['target_lane']
                    events.append({'t_s':s.t_s,'status':'change_started','aircraft_id':ident,
                        'source_lane':ego.lane,'target_lane':ego.target_lane,
                        'duration_s':chosen['duration_s'],
                        'predicted_global_policy_cost_improvement_s':record.get('gain'),
                        'predicted_ego_policy_cost_improvement_s':
                            chosen.get('ego_policy_cost_improvement_s'),
                        'affected_aircraft':chosen.get('affected_aircraft',[]),
                        'worsened_aircraft':chosen.get('worsened_aircraft',[])})
                decisions.extend(records)
                emit_progress(phase='main_simulation',decision_time_s=s.t_s,
                              ego_aircraft_id=None,target_lane=None,
                              candidate_duration_s=None,forecast_time_s=None,
                              forecast_end_s=None)
                if not base['admitted'] and not proposals:
                    events.append({'t_s':s.t_s,'status':'no_verified_action','reasons':base['reasons']})
                    status='no_verified_action';break
            issue,_=self.step(w,horizon_s,events,obs,trace)
            if issue:status='execution_rejected';events.append({'t_s':s.t_s,'status':status,'reasons':issue});break
            emit_progress(phase='main_simulation',decision_time_s=None,
                          ego_aircraft_id=None,target_lane=None,
                          candidate_duration_s=None,forecast_time_s=None,
                          forecast_end_s=None)
        emit_progress(force=True,phase='finished',final_status=status,
                      decision_time_s=None,
                      ego_aircraft_id=None,target_lane=None,
                      candidate_duration_s=None,forecast_time_s=None,
                      forecast_end_s=None)
        return {'status':status,'end_s':w.traffic.t_s,'trace':trace,'observations':obs,'events':events,'decisions':decisions,'pending':[p.aircraft_id for p in w.traffic.pending],'active':[a.aircraft_id for a in w.traffic.active],'completed':[a.aircraft_id for a in w.traffic.completed],
            'limitations':['numerical midpoint/endpoint envelope audit, not continuous proof','perfect-model radio and scheduled-entry prediction','all aircraft evaluated; one new start per clock, not joint combinatorial optimization','no abort trajectory synthesized','policy targets are explicit fixed research distances',f'candidate topology is {self.candidate_topology}',f'radio sampled every {self.cfg.radio_period_s:g} s and policy updated every {self.cfg.policy_s:g} s with persistence k={self.cfg.persistence_k}']}
