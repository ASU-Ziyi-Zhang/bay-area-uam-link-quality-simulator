"""Stage 1: isolated, constant-altitude movements in a smoothed Bay Area corridor.

No fleet, ACC interaction, capacity estimator, or optimized reference route.
The longitudinal speed is the physical component along the reference tangent,
not the rate of progress of the original GIS coordinate. Radio and policy
thresholds retain the existing project's definitions.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math

import numpy as np

from .dynamic_transitions import smoothstep
from .path_design import SmoothPath
from .policy_motion import BSRadio, Config as PolicyConfig, policy_from_exposure


@dataclass(frozen=True)
class LateralConfig:
    altitude_m: float = 300.
    cruise_mps: float = 50.
    dt_s: float = .5
    forecast_dt_s: float = 1.
    policy_s: float = 5.
    decision_s: float = 20.
    horizon_s: float = 240.
    window_s: float = 30.
    cooldown_s: float = 30.
    threshold_db: float = -1.5
    exposure_c: float = .05
    exposure_r: float = .10
    minimum_exposure_benefit: float = .02
    lateral_speed_limit_mps: float = 8.
    lateral_accel_limit_mps2: float = .3
    min_maneuver_s: float = 30.
    center_control_step_m: float = 500.
    spatial_sample_m: float = 50.
    # Predictive mode is opt-in.  The default remains the reviewed reactive
    # baseline so existing R0009/R0010 runs are byte-for-byte unaffected.
    predictive_enabled: bool = False
    reaction_latency_s: float = 0.
    # A predictive move is useful only when it can complete before a forecast
    # F episode. This opt-in gate preserves historical runs.
    predictive_completion_before_f: bool = False
    # Optional one-step receding-horizon controller. At each decision clock it
    # compares the policy at t + predictive_rollout_s for staying and each
    # feasible candidate. Zero keeps the historical planner active.
    predictive_rollout_s: float = 0.
    # Optional time appended to the warning horizon for evaluating complete
    # maneuvers.  Zero preserves the reviewed single-horizon endpoint rule.
    # A positive value activates the dual-horizon integrated-policy objective.
    predictive_evaluation_s: float = 0.
    policy_cost_r: float = 1.
    policy_cost_f: float = 2.
    predictive_endpoint_upgrade: bool = False

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, (bool, np.bool_, str)) or value is None:
                continue
            nonnegative = {"threshold_db", "reaction_latency_s", "predictive_rollout_s",
                           "predictive_evaluation_s", "speed_min_mps"}
            if not np.isfinite(value) or (name not in nonnegative and value <= 0):
                raise ValueError(f"invalid finite positive parameter: {name}")
            if name in {"reaction_latency_s", "predictive_rollout_s",
                        "predictive_evaluation_s", "speed_min_mps"} and value < 0:
                raise ValueError(f"{name} must be nonnegative")
        if not 0 < self.exposure_c <= self.exposure_r < 1 or self.minimum_exposure_benefit >= 1:
            raise ValueError("invalid exposure limits")
        for step in (self.dt_s, self.forecast_dt_s):
            for clock in (self.policy_s, self.decision_s, self.horizon_s):
                if abs(clock/step-round(clock/step)) > 1e-8:
                    raise ValueError("policy/decision/horizon clocks must align with motion and forecast steps")
        if abs(self.window_s/self.policy_s-round(self.window_s/self.policy_s)) > 1e-8:
            raise ValueError("exposure window must align with policy samples")
        if self.predictive_endpoint_upgrade:
            if self.predictive_rollout_s <= 0:
                raise ValueError("endpoint-upgrade prediction requires a positive rollout")
            if abs(self.predictive_rollout_s/self.forecast_dt_s-round(self.predictive_rollout_s/self.forecast_dt_s)) > 1e-8:
                raise ValueError("predictive rollout must align with the forecast step")
            if abs(self.predictive_rollout_s/self.policy_s-round(self.predictive_rollout_s/self.policy_s)) > 1e-8:
                raise ValueError("predictive rollout must align with policy samples")
            if abs(self.predictive_evaluation_s/self.forecast_dt_s-round(self.predictive_evaluation_s/self.forecast_dt_s)) > 1e-8:
                raise ValueError("predictive evaluation extension must align with the forecast step")
            if abs(self.predictive_evaluation_s/self.policy_s-round(self.predictive_evaluation_s/self.policy_s)) > 1e-8:
                raise ValueError("predictive evaluation extension must align with policy samples")

    def policy_config(self):
        return PolicyConfig(cruise_mps=self.cruise_mps, threshold_db=self.threshold_db,
                            exposure_c=self.exposure_c, exposure_r=self.exposure_r)


class SmoothCorridorFrame:
    """Natural cubic at uniform ORIGINAL route progress, with analytic frame.

    Centerline is C2. Under the fixed physical tangential-speed convention,
    quintic lateral motion has continuous position, velocity and acceleration;
    jerk can jump at cubic knots and maneuver endpoints (no jerk limit).
    Reference q is not the smoothed path's arc length. Never assume dq/dt = V.
    """
    def __init__(self, corridor, control_step_m=500.):
        if not np.isfinite(control_step_m) or control_step_m <= 0:
            raise ValueError("invalid center control step")
        self.original = corridor
        self.length_m = corridor.length_m
        self.controls_q = np.linspace(0, self.length_m, int(np.ceil(self.length_m/control_step_m))+1)
        self.step = self.controls_q[1]-self.controls_q[0]
        self.path = SmoothPath(corridor.interpolate(self.controls_q), corridor.crs, 5.)

    def _frame_uncached(self, q):
        parameter = np.clip(np.asarray(q, float)/self.step, 0, len(self.controls_q)-1)
        index = np.minimum(parameter.astype(int), len(self.controls_q)-2)
        u = parameter-index
        c = self.path.coefficients[index]
        center = self.path.control_xy_m[0]+c[..., 0, :]+u[..., None]*(
            c[..., 1, :]+u[..., None]*(c[..., 2, :]+u[..., None]*c[..., 3, :]))
        first = (c[..., 1, :]+2*u[..., None]*c[..., 2, :]+3*u[..., None]**2*c[..., 3, :])/self.step
        second = (2*c[..., 2, :]+6*u[..., None]*c[..., 3, :])/self.step**2
        norm = np.linalg.norm(first, axis=-1)
        if np.any(norm < 1e-8):
            raise ValueError("nonregular reference centerline")
        tangent = first/norm[..., None]
        normal = np.stack([-tangent[..., 1], tangent[..., 0]], axis=-1)
        curvature = (first[..., 0]*second[..., 1]-first[..., 1]*second[..., 0])/norm**3
        return center, tangent, normal, norm, curvature

    @lru_cache(maxsize=262144)
    def _scalar_frame(self, q):
        """Cache exact scalar evaluations reused across forecast branches."""
        return self._frame_uncached(q)

    def frame(self, q):
        if np.ndim(q)==0:
            return self._scalar_frame(float(q))
        return self._frame_uncached(q)

    def xyz(self, q, offset, altitude):
        center, _, normal, _, _ = self.frame(q)
        xy = center+np.asarray(offset)[..., None]*normal
        return np.concatenate([xy, np.full(xy.shape[:-1]+(1,), altitude)], axis=-1)

    def progress_rate(self, q, offset, speed):
        _, _, _, norm, curvature = self.frame(q)
        factor = 1-curvature*np.asarray(offset)
        if np.any(factor <= .05):
            raise ValueError("folded or nearly singular lateral coordinate")
        return speed/(norm*factor)

    def kinematics(self, q, offset, offset_speed, offset_acceleration, cruise):
        _, tangent, normal, _, curvature = self.frame(q)
        rate_arc = cruise/(1-curvature*offset)
        velocity = cruise*tangent+offset_speed*normal
        acceleration = (-offset_speed*curvature*rate_arc)*tangent+(
            cruise*curvature*rate_arc+offset_acceleration)*normal
        return velocity, acceleration

    def report(self, maximum_offset):
        q = np.unique(np.r_[np.linspace(0, self.length_m, int(np.ceil(self.length_m/5))+1),
                            self.controls_q])
        center, _, _, norm, curvature = self.frame(q)
        deviation = np.linalg.norm(center-self.original.interpolate(q), axis=-1)
        exact = self.path.curvature_report()
        # Exact cubic curvature extrema, rather than only the sampled signs.
        regular_margin = 1-maximum_offset*exact["max_curvature_per_m"]
        if not exact["regular"] or regular_margin <= .05:
            raise ValueError("requested offset envelope is not a regular corridor frame")
        return {"original_length_m": self.length_m, "smoothed_length_m": self.path.length_m,
                "control_count": len(self.controls_q), "control_step_m": self.step,
                "maximum_centerline_deviation_m": float(deviation.max()),
                "rms_centerline_deviation_m": float(np.sqrt(np.mean(deviation**2))),
                "endpoint_error_m": float(np.linalg.norm(center[[0,-1]]-self.original.xy_m[[0,-1]],axis=1).max()),
                "minimum_coordinate_factor_bound": regular_margin,
                "reference_derivative_norm_min": float(norm.min()),
                "reference_derivative_norm_max": float(norm.max()),
                **exact,
                "deviation_check": "equal original-route-progress, sampled every <=5 m and all spline knots",
                "speed_definition": "50 m/s (configured V) physical component along smooth reference tangent; dq/dt is adjusted",
                "smoothness": "C2 center and fixed-tangential-speed motion; jerk can jump at knots/endpoints, no jerk constraint",
                "flight_performance_certified": False}


@dataclass(frozen=True)
class LateralMove:
    start_s: float
    source_m: float
    target_m: float
    duration_s: float

    def sample(self, t):
        f, df, ddf = smoothstep((np.asarray(t)-self.start_s)/self.duration_s)
        delta = self.target_m-self.source_m
        return self.source_m+delta*f, delta*df/self.duration_s, delta*ddf/self.duration_s**2


def maneuver_duration(distance, cfg):
    if not np.isfinite(distance):
        raise ValueError("nonfinite displacement")
    distance = abs(distance)
    if distance == 0:
        return 0.
    raw = max(cfg.min_maneuver_s, 1.875*distance/cfg.lateral_speed_limit_mps,
              math.sqrt(10/math.sqrt(3)*distance/cfg.lateral_accel_limit_mps2))
    # Physical maneuver duration must not change during an integration-step
    # refinement. The event loop splits at the exact completion time instead.
    return raw


def offset_values(t, start, source, targets, durations):
    """Vectorized stay/move candidates; duration zero means stay."""
    f, df, ddf = smoothstep((t-start)/np.maximum(durations, 1e-12))
    delta = np.asarray(targets)-source
    return source+delta*f, delta*df/np.maximum(durations,1e-12), delta*ddf/np.maximum(durations,1e-12)**2


def rk4_progress(frame, q, t, dt, offset_at, cruise, tolerance_m=1e-6):
    """Error-controlled RK4 across cubic knots, preserving physical speed V.

    Curvature is continuous but its derivative can jump at spline knots. A
    single nominal RK4 step can therefore be inaccurate even with a smooth
    position curve. Compare one full step with two half steps, subdividing
    until reference-progress disagreement is <=1e-6 m per requested step.
    """
    def rate(q, t):
        return frame.progress_rate(q, offset_at(t), cruise)
    def step(q,t,dt):
        k1=rate(q,t)
        k2=rate(q+dt*k1/2,t+dt/2)
        k3=rate(q+dt*k2/2,t+dt/2)
        k4=rate(q+dt*k3,t+dt)
        return q+dt*(k1+2*k2+2*k3+k4)/6
    def advance(q,t,dt,tolerance,depth):
        full=step(q,t,dt)
        mid=step(q,t,dt/2)
        half=step(mid,t+dt/2,dt/2)
        # Deep subdivision must not demand sub-ULP agreement at q~50 km.
        # Otherwise roundoff alone can cause endless rejection of a tiny step.
        roundoff=16*np.finfo(float).eps*max(1.,float(np.max(np.abs(q))),float(np.max(np.abs(half))))
        if np.max(np.abs(full-half))<=max(tolerance,roundoff):
            return half
        if depth>=20:
            raise RuntimeError("reference-progress integration did not reach its error tolerance")
        mid=advance(q,t,dt/2,tolerance/2,depth+1)
        return advance(mid,t+dt/2,dt/2,tolerance/2,depth+1)
    return advance(q,t,dt,tolerance_m,0)


def envelope_durations(frame, cfg, envelope, *, q, t, source, targets):
    """Per-candidate durations under the coupled acceleration envelope.

    Returns the duration array used by the forecast plus a parallel list of
    rejection reasons. A rejected candidate keeps its closed-form duration so
    the vectorised forecast stays well formed; the caller overrides its status
    so it can never become eligible.
    """
    from .motion_envelope import feasible_duration
    durations, rejected = [], []
    for target in targets:
        baseline = maneuver_duration(target-source, cfg)
        if target == source:
            durations.append(baseline); rejected.append(None); continue
        report = feasible_duration(frame, cfg, envelope, q0=float(q), t0=float(t),
                                   source=float(source), target=float(target))
        if report["duration_s"] is None:
            durations.append(baseline); rejected.append(report["status"])
        else:
            durations.append(report["duration_s"]); rejected.append(None)
    return np.asarray(durations, float), rejected


def forecast_candidates(frame, radio, cfg, *, q, t, source, targets, history,
                        envelope=None):
    """Paired map-informed single-move forecasts, no future discretionary moves.

    Batch all candidates and staying on one forecast grid. Use only common
    in-corridor policy times for each pair; require a full exposure window after
    maneuver completion in both branches. Historical bad flags are retained.

    ``envelope`` is opt-in. When supplied, each candidate duration comes from
    the coupled acceleration envelope instead of the closed-form component
    bounds, and candidates the envelope cannot accommodate are rejected before
    the communication gate. Passing ``None`` reproduces the archived behaviour
    exactly.
    """
    targets = np.r_[source, np.asarray(targets, float)]
    if envelope is None:
        duration = np.array([maneuver_duration(d-source,cfg) for d in targets])
        envelope_rejected = [None]*len(targets)
    else:
        duration, envelope_rejected = envelope_durations(
            frame, cfg, envelope, q=q, t=t, source=source, targets=targets)
    state = np.full(len(targets), q, float)
    clock = float(t)
    flags = [np.full(len(targets), bad, float) for _,bad in history]
    flag_times = [u for u,_ in history]
    exposures, active, times = [], [], []
    while clock < t+cfg.horizon_s-1e-8:
        # Forecast steps align to policy clocks, including a shortened last step.
        next_policy = (math.floor((clock+1e-8)/cfg.policy_s)+1)*cfg.policy_s
        step = min(cfg.forecast_dt_s, t+cfg.horizon_s-clock, next_policy-clock)
        state = rk4_progress(frame, state, clock, step,
            lambda time:offset_values(time,t,source,targets,duration)[0], cfg.cruise_mps)
        clock += step
        if abs(clock/cfg.policy_s-round(clock/cfg.policy_s)) > 1e-8:
            continue
        d = offset_values(clock,t,source,targets,duration)[0]
        good = state < frame.length_m-1e-7
        values = radio(frame.xyz(np.minimum(state,frame.length_m),d,cfg.altitude_m))["sinr_db"]
        flags.append((values < cfg.threshold_db).astype(float))
        flag_times.append(clock)
        while flag_times and flag_times[0] < clock-cfg.window_s-1e-8:
            flag_times.pop(0); flags.pop(0)
        exposures.append(np.mean(flags,axis=0)); active.append(good); times.append(clock)
        if not good.any():
            break
    exposures, active, times = np.asarray(exposures), np.asarray(active), np.asarray(times)
    policy_cfg = cfg.policy_config()
    # Same C/R/F spacing-priority heuristic as the reviewed controller, but no
    # traffic-spacing enforcement or capacity inference in this isolated study.
    codes = np.where(exposures <= cfg.exposure_c+1e-12, 2,
                     np.where(exposures <= cfg.exposure_r+1e-12,1,0))
    spacing = np.array([policy_cfg.spacing(p,cfg.cruise_mps) for p in "FRC"])[codes]
    records = []
    for k in range(1,len(targets)):
        common = active[:,0] & active[:,k]
        usable = common.any() and times[common][-1] >= t+duration[k]+cfg.window_s-1e-8
        row = {"target_m": float(targets[k]), "duration_s": float(duration[k]),
               "paired_policy_samples": int(common.sum()),
               "status": "insufficient_post_move_horizon"}
        if envelope_rejected[k] is not None:
            row["status"] = f"envelope_rejected:{envelope_rejected[k]}"
            records.append(row)
            continue
        if usable:
            benefit = float(np.mean(exposures[common,0]-exposures[common,k]))
            spacing_benefit = float(np.mean(spacing[common,0]-spacing[common,k]))
            row.update(exposure_benefit=benefit, policy_spacing_priority_benefit_m=spacing_benefit,
                       status="eligible" if benefit >= cfg.minimum_exposure_benefit-1e-10
                       and spacing_benefit >= -1e-8 else "communication_gate_rejected")
        records.append(row)
    return records


def simulate_isolated(frame, scenario, cfg, offsets, initial_offset, transitions,
                      planner=None, envelope=None):
    offsets = np.asarray(offsets, float)
    if offsets.ndim != 1 or not len(offsets) or not np.isfinite(offsets).all() or np.any(np.diff(offsets)<=0):
        raise ValueError("offsets must be a finite increasing array")
    if initial_offset not in offsets:
        raise ValueError("initial offset must be a candidate")
    frame.report(float(np.max(np.abs(offsets))))
    radio = BSRadio(scenario.base_stations, scenario.radio)
    policy_cfg = cfg.policy_config()
    t = q = 0.
    offset = float(initial_offset)
    move = None
    history, trace, observations, events = [], [], [], []
    cooldown_until = 0.
    policy, exposure = "C", None
    next_policy = next_decision = 0.
    completed = 0
    # A large finite bound catches an integration bug instead of hanging forever.
    deadline = 3*frame.length_m/cfg.cruise_mps+cfg.horizon_s
    while q < frame.length_m-1e-7:
        if t > deadline:
            raise RuntimeError("nonadvancing or unexpectedly long flight")
        if move is not None and t >= move.start_s+move.duration_s-1e-8:
            offset = move.target_m
            events.append({"status":"transition_completed","t_s":t,"q_m":q,
                           "source_m":move.source_m,"target_m":move.target_m})
            move = None; completed += 1; cooldown_until = t+cfg.cooldown_s
        def sample(time):
            return move.sample(time) if move is not None else (offset,0.,0.)
        d, vd, ad = map(float,sample(t))
        xyz = frame.xyz(q,d,cfg.altitude_m)
        if t >= next_policy-1e-8:
            values = radio(xyz[None,:])
            sinr = float(values["sinr_db"][0])
            history = [(u,b) for u,b in history if u >= t-cfg.window_s-1e-8]
            history.append((t,int(sinr<cfg.threshold_db)))
            exposure = sum(b for _,b in history)/len(history)
            policy = policy_from_exposure(exposure,policy_cfg)
            observations.append({"t_s":t,"q_m":q,"offset_m":d,"sinr_db":sinr,
                "rsrp_dbm":float(values["serving_rsrp_dbm"][0]),
                "serving_bs":int(values["serving_bs"][0]),
                "exposure":exposure,"policy":policy,"history_count":len(history),"group_size":1})
            next_policy += cfg.policy_s
        if t >= next_decision-1e-8:
            if transitions and move is None and t >= cooldown_until-1e-8:
                choices = [float(value) for value in offsets if value != offset]
                if choices:
                    if planner is None:
                        records = forecast_candidates(frame,radio,cfg,q=q,t=t,source=offset,
                                                      targets=choices,history=history,
                                                      envelope=envelope)
                        eligible = [r for r in records if r["status"]=="eligible"]
                        chosen = min(eligible,key=lambda r:(-r["policy_spacing_priority_benefit_m"],
                            -r["exposure_benefit"],r["duration_s"],r["target_m"])) if eligible else None
                    else:
                        records = planner.candidates(frame,radio,cfg,q=q,t=t,source=offset,
                            targets=choices,history=history,policy=policy)
                        chosen = planner.choose(records,offset)
                    if chosen is not None:
                        move = LateralMove(t,offset,chosen["target_m"],chosen["duration_s"])
                        chosen["status"] = "transition_accepted"
                    for row in records:
                        events.append({"t_s":t,"q_m":q,"source_m":offset,**row})
            next_decision += cfg.decision_s
        end = min(t+cfg.dt_s,next_policy,next_decision)
        if move is not None:
            end = min(end,move.start_s+move.duration_s)
        dt = end-t
        if dt <= 1e-10:
            raise RuntimeError("nonadvancing event loop")
        offset_at = lambda time:sample(time)[0]
        q_next = float(rk4_progress(frame,q,t,dt,offset_at,cfg.cruise_mps))
        if q_next >= frame.length_m:
            lo,hi = 0.,dt
            for _ in range(38):
                mid=(lo+hi)/2
                if rk4_progress(frame,q,t,mid,offset_at,cfg.cruise_mps) < frame.length_m:
                    lo=mid
                else:
                    hi=mid
            dt=(lo+hi)/2; q_next=frame.length_m
        velocity,acceleration = frame.kinematics(q,d,vd,ad,cfg.cruise_mps)
        trace.append({"t_s":t,"end_s":t+dt,"q_m":q,"offset_m":d,
            "offset_velocity_mps":vd,"offset_acceleration_mps2":ad,"position_m":xyz.tolist(),
            "velocity_mps":np.r_[velocity,0.].tolist(),"acceleration_mps2":np.r_[acceleration,0.].tolist(),
            "speed_mps":float(np.linalg.norm(velocity)),"policy":policy,"exposure":exposure,
            "target_m":move.target_m if move else None})
        q,t=q_next,t+dt
    if move is not None:
        raise RuntimeError("unfinished maneuver at corridor exit")
    terminal_d = float(offset)
    terminal = {"t_s":t,"q_m":q,"offset_m":terminal_d,
                "position_m":frame.xyz(q,terminal_d,cfg.altitude_m).tolist()}
    durations=np.array([r["end_s"]-r["t_s"] for r in trace])
    # Simpson integration of sqrt(V^2 + d_dot^2), independent of the frame.
    moves = [LateralMove(e["t_s"],e["source_m"],e["target_m"],e["duration_s"])
             for e in events if e["status"]=="transition_accepted"]
    extra_transverse_length=0.
    for m in moves:
        grid=np.linspace(m.start_s,m.start_s+m.duration_s,1001)
        speed=np.sqrt(cfg.cruise_mps**2+m.sample(grid)[1]**2)-cfg.cruise_mps
        extra_transverse_length += float((grid[1]-grid[0])/3*(speed[0]+speed[-1]+4*speed[1:-1:2].sum()+2*speed[2:-1:2].sum()))
    shares={p:float(sum(dt for dt,r in zip(durations,trace) if r["policy"]==p)/t) for p in "CRF"}
    counts={p:sum(r["policy"]==p for r in observations) for p in "CRF"}
    stations=scenario.base_stations.positions(scenario.radio.assumed_bs_height_m)
    positions=np.array([frame.xyz(r["q_m"],r["offset_m"],cfg.altitude_m) for r in observations])
    distances=np.linalg.norm(positions[:,None,:]-stations[None,:,:],axis=-1)
    selected=np.argsort(distances,axis=1)[:,:scenario.radio.served_set_size]
    horizontal=np.linalg.norm(positions[:,None,:2]-stations[None,:,:2],axis=-1)
    served_horizontal=np.take_along_axis(horizontal,selected,axis=1)
    summary={"transitions_enabled":transitions,"envelope_enabled":envelope is not None,
        "envelope_coupling_mode":getattr(envelope,"coupling_mode",None),
        "envelope_lateral_accel_max_mps2":getattr(envelope,"lateral_accel_max_mps2",None),
        "envelope_lateral_jerk_max_mps3":getattr(envelope,"lateral_jerk_max_mps3",None),
        "initial_offset_m":float(initial_offset),
        "final_offset_m":terminal_d,"allowed_offsets_m":offsets.tolist(),
        "altitude_m":cfg.altitude_m,"group_size":1,"completed_moves":completed,
        "corridor_time_s":t,"path_length_m":cfg.cruise_mps*t+extra_transverse_length,
        "total_maneuver_time_s":sum(m.duration_s for m in moves),
        "policy_time_shares":shares,"policy_observation_counts":counts,
        "time_mean_exposure":float(sum(r["exposure"]*dt for r,dt in zip(trace,durations))/t),
        "mean_sinr_db":float(np.mean([r["sinr_db"] for r in observations])),
        "minimum_sinr_db":float(min(r["sinr_db"] for r in observations)),
        "max_speed_mps":max(r["speed_mps"] for r in trace),
        "max_cartesian_acceleration_mps2":max(float(np.linalg.norm(r["acceleration_mps2"])) for r in trace),
        "max_offset_speed_mps":max(abs(r["offset_velocity_mps"]) for r in trace),
        "max_offset_acceleration_mps2":max(abs(r["offset_acceleration_mps2"]) for r in trace),
        "any_served_link_beyond_4km_observation_fraction":float(np.mean(np.any(served_horizontal>4000,axis=1))),
        "height_extrapolated":not 22.5<cfg.altitude_m<=300,
        "capacity_estimated":False,"kinematic_note":"Cartesian acceleration includes reference-route turning, not bounded by the offset-only 0.3 m/s^2 limit"}
    return {"summary":summary,"trace":trace,"observations":observations,"events":events,"terminal":terminal}


def field_samples(frame, scenario, cfg, offsets):
    q=np.linspace(0,frame.length_m,int(np.ceil(frame.length_m/cfg.spatial_sample_m))+1)
    Q,D=np.meshgrid(q,np.asarray(offsets,float))
    values=BSRadio(scenario.base_stations,scenario.radio)(frame.xyz(Q.ravel(),D.ravel(),cfg.altitude_m))
    raw_center=frame.original.interpolate(q)
    raw_values=BSRadio(scenario.base_stations,scenario.radio)(
        np.c_[raw_center,np.full(len(q),cfg.altitude_m)])
    return {"q_m":q,"offsets_m":np.asarray(offsets,float),
            "sinr_db":values["sinr_db"].reshape(Q.shape),
            "rsrp_dbm":values["serving_rsrp_dbm"].reshape(Q.shape),
            "serving_bs":values["serving_bs"].reshape(Q.shape),
            "smooth_center_xy_m":frame.frame(q)[0],"raw_center_xy_m":raw_center,
            "raw_center_sinr_db":raw_values["sinr_db"]}
