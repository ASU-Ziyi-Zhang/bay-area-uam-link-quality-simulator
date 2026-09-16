"""Opt-in straight-route two-aircraft mechanism harness.

The follower is numerically integrated, not assigned the reference trajectory.
Geographic ACC is called directly for the baseline and observed-leader fallback.
No radio, lane changes, capacity, or full-corridor integration is implied.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

from .aks import minimum_duration, plan_equal_speed_transition, tracking_command
from .geographic_traffic import GeographicTrafficConfig, TrafficAircraft, TrafficState, acc_controls


@dataclass(frozen=True)
class Vehicle:
    cruise_mps: float = 50.0
    speed_min_mps: float = 0.0
    speed_max_mps: float = 80.0
    acceleration_limit_mps2: float = 1.5
    deceleration_limit_mps2: float = 2.0
    reference_jerk_limit_mps3: float | None = None

    def __post_init__(self):
        for key, value in asdict(self).items():
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError(f"invalid {key}")
        if not self.speed_min_mps <= self.cruise_mps <= self.speed_max_mps:
            raise ValueError("cruise must be within speed bounds")
        if min(self.acceleration_limit_mps2, self.deceleration_limit_mps2) <= 0:
            raise ValueError("positive acceleration bounds required")
        if self.reference_jerk_limit_mps3 is not None and self.reference_jerk_limit_mps3 <= 0:
            raise ValueError("positive reference jerk bound required")


@dataclass(frozen=True)
class Spacing:
    d0_m: float = 200.0
    tau_c_s: float = 15.0
    tau_r_s: float = 30.0
    tau_f_s: float = 60.0
    buffer_s2_per_m: float = 0.167

    def __post_init__(self):
        if any(not math.isfinite(v) or v < 0 for v in asdict(self).values()):
            raise ValueError("invalid spacing parameters")
        if not self.tau_c_s <= self.tau_r_s <= self.tau_f_s:
            raise ValueError("unordered policy time gaps")


@dataclass(frozen=True)
class Controller:
    k_speed_per_s: float = 0.2
    k_gap_per_s2: float = 0.005
    k_relative_per_s: float = 0.4
    tracking_speed_gain_per_s: float = 0.4
    acc_policy_recovery: bool = False

    def __post_init__(self):
        if not isinstance(self.acc_policy_recovery, bool):
            raise ValueError("acc_policy_recovery must be boolean")
        if any(not math.isfinite(v) or v <= 0 for k, v in asdict(self).items()
               if k != "acc_policy_recovery"):
            raise ValueError("positive finite gains required")


@dataclass(frozen=True)
class ResolvedPair:
    vehicle: Vehicle
    spacing: Spacing
    controller: Controller
    legacy: GeographicTrafficConfig


def resolve(spec, vehicle_overrides=None):
    """Reject unknown fields rather than silently ignoring an experiment input."""
    if set(spec) != {"vehicle", "spacing", "controller"}:
        raise ValueError("expected exactly vehicle, spacing, controller blocks")
    vehicle = Vehicle(**{**spec["vehicle"], **(vehicle_overrides or {})})
    spacing, controller = Spacing(**spec["spacing"]), Controller(**spec["controller"])
    # The baseline must fly under the same speed envelope as the tracker,
    # otherwise the comparison reports a bound rather than a controller: the
    # legacy config defaults its ceiling to cruise, which alone would stop ACC
    # from ever closing a gap.
    legacy = GeographicTrafficConfig(**asdict(spacing),
        cruise_mps=vehicle.cruise_mps,
        speed_min_mps=vehicle.speed_min_mps,
        speed_max_mps=vehicle.speed_max_mps,
        acceleration_limit_mps2=vehicle.acceleration_limit_mps2,
        deceleration_limit_mps2=vehicle.deceleration_limit_mps2,
        k_speed_per_s=controller.k_speed_per_s,
        k_gap_per_s2=controller.k_gap_per_s2,
        k_relative_per_s=controller.k_relative_per_s)
    return ResolvedPair(vehicle, spacing, controller, legacy)


def plan_request(cfg, actual_gap, policy, objective="minimum_spacing",
                 follower_speed=None, leader_speed=None):
    """Resolve actual-state request before constructing a positive-duration KS2."""
    if objective not in {"minimum_spacing", "exact_target"}:
        raise ValueError("unknown gap objective")
    v = cfg.vehicle
    own_speed = v.cruise_mps if follower_speed is None else follower_speed
    lead_speed = v.cruise_mps if leader_speed is None else leader_speed
    if not all(math.isfinite(x) for x in (actual_gap, own_speed, lead_speed)) or actual_gap <= 0:
        raise ValueError("invalid actual state")
    goal = cfg.legacy.spacing(policy, v.cruise_mps)
    base = {"initial_gap_m": actual_gap, "target_gap_m": goal,
            "objective": objective, "transition": None, "duration_s": 0.0}
    if abs(own_speed - v.cruise_mps) > 1e-9 or abs(lead_speed - own_speed) > 1e-9:
        return {**base, "status": "observed_state_acc", "target_reduction_m": None,
                "reason": "equal-speed reference assumptions do not hold"}
    reduction = actual_gap - goal
    if (objective == "minimum_spacing" and reduction >= 0) or abs(reduction) < 1e-10:
        return {**base, "status": "hold", "target_reduction_m": 0.0,
                "reason": "no spacing adjustment required"}
    bounds = minimum_duration(reduction, own_speed,
        v.acceleration_limit_mps2, v.deceleration_limit_mps2,
        jerk_limit=v.reference_jerk_limit_mps3, speed_max=v.speed_max_mps,
        speed_min=v.speed_min_mps)
    if not math.isfinite(bounds["duration_s"]):
        return {**base, "status": "infeasible", "target_reduction_m": reduction,
                "reason": bounds["binding"], "binding": bounds["binding"]}
    report = plan_equal_speed_transition(own_speed, reduction, bounds["duration_s"],
        accel_limit=v.acceleration_limit_mps2, decel_limit=v.deceleration_limit_mps2,
        jerk_limit=v.reference_jerk_limit_mps3, speed_max=v.speed_max_mps,
        speed_min=v.speed_min_mps)
    if not report["feasible"]:
        raise AssertionError(f"duration solver returned infeasible plan: {report['reasons']}")
    return {**base, **report, "status": "planned", "reason": "actual gap adjustment",
            "target_reduction_m": reduction, "duration_s": bounds["duration_s"],
            "binding": bounds["binding"], "bounds": bounds["bounds"]}


class StraightArc:
    @staticmethod
    def position(q, lane):
        return q

    @staticmethod
    def gap(own, lead, lane):
        return lead - own


def leader_state(t, speed, braking):
    """Prescribed continuous leader motion; perfect current-state observation."""
    if braking is None:
        return speed * t, speed, 0.0
    start, end_speed, decel = braking["start_s"], braking["end_speed_mps"], braking["deceleration_mps2"]
    duration = (speed - end_speed) / decel
    u = min(max(0.0, t - start), duration)
    x = speed * t - 0.5 * decel * u**2 - decel * duration * max(0.0, t - start - duration)
    acceleration = -decel if start <= t < start + duration else 0.0
    return x, speed - decel * u, acceleration


def simulate(case, cfg, mode, dt, tail_s=120.0):
    if mode not in {"acc", "aks"} or dt <= 0 or not math.isfinite(dt):
        raise ValueError("invalid controller or integration step")
    vehicle = cfg.vehicle
    initial_gap = case.get("initial_gap_m")
    if initial_gap is None:
        initial_gap = cfg.legacy.spacing(case["initial_gap_policy"], vehicle.cruise_mps)
    request = plan_request(cfg, initial_gap, case["target_policy"], case["objective"])
    transition = request["transition"]
    end = max(tail_s, request["duration_s"] + tail_s)
    boundaries = [end]
    if transition is not None:
        boundaries += [transition.alpha * transition.duration, transition.duration]
    braking = case.get("leader_braking")
    if braking:
        if not (0 <= braking["end_speed_mps"] < vehicle.cruise_mps and braking["deceleration_mps2"] > 0):
            raise ValueError("invalid prescribed braking")
        brake_end = braking["start_s"] + (vehicle.cruise_mps - braking["end_speed_mps"]) / braking["deceleration_mps2"]
        end = max(end, brake_end + tail_s)
        boundaries += [braking["start_s"], brake_end, end]
    own = TrafficAircraft("follower", q_m=0.0, speed_mps=vehicle.cruise_mps,
                          lane=0, policy=case["target_policy"])
    lead = TrafficAircraft("leader", q_m=initial_gap, speed_mps=vehicle.cruise_mps,
                           lane=0)
    state = TrafficState(0.0, [own, lead], [])
    arc = StraightArc()
    # Policy-transition fixtures explicitly represent an existing follower pair.
    # A sufficient arbitrary starting gap is not an instruction to pursue.
    established = case.get("established_following", "initial_gap_policy" in case)
    recovery_pairs = (frozenset({("follower", "leader")})
                      if established and cfg.controller.acc_policy_recovery
                      else frozenset())
    invalidated = False
    exhausted = [False]      # list so the nested command() can set it
    events = [{"t_s": 0.0, "event": request["status"] if mode == "aks" else "baseline_acc"}]
    # Three records with three different jobs, kept apart on purpose.
    #   events           the decision log. Discrete content and grid times only,
    #                    so two runs at different steps produce identical lists
    #                    unless they genuinely decided something different.
    #   discontinuities  times where the recorded acceleration jumps, so a
    #                    finite-difference audit knows which interval to skip.
    #                    Solved times belong here, not in the decision log.
    #   diagnostics      measured continuous quantities. Reported, never compared
    #                    for equality: they carry the integrator's own error.
    discontinuities = []
    diagnostics = {}
    if braking:
        discontinuities += [float(braking["start_s"]), float(brake_end)]

    def command(t, y):
        lx, lv, la = leader_state(t, vehicle.cruise_mps, braking)
        gap = initial_gap + lx - y[0]
        if mode == "aks" and request["status"] in {"hold", "planned"} and not invalidated:
            if transition is None:
                vr, ar, xr = vehicle.cruise_mps, 0.0, vehicle.cruise_mps * t
            elif t >= transition.duration:
                vr, ar = vehicle.cruise_mps, 0.0
                xr = transition.displacement() + vr * (t - transition.duration)
            else:
                vr, ar, xr = (float(x) for x in transition.sample(t))
            # Constant-leader forecast, fixed at planning time. It is invalidated
            # when the observed leader acceleration/speed departs from that plan.
            gap_ref = initial_gap + vehicle.cruise_mps * t - xr
            raw = float(tracking_command(ar, gap, gap_ref, vr, y[1],
                cfg.controller.k_gap_per_s2, cfg.controller.tracking_speed_gain_per_s))
            source = "ks2_tracker" if transition else "hold_tracker"
        else:
            own.q_m, own.speed_mps = float(y[0]), float(y[1])
            lead.q_m, lead.speed_mps = initial_gap + lx, lv
            raw = acc_controls(state, arc, cfg.legacy,
                               recovery_pairs=recovery_pairs)["follower"]["command"]
            gap_ref = cfg.legacy.spacing(case["target_policy"], y[1])
            source = "acc_fallback" if mode == "aks" else "baseline_acc"
        a = float(np.clip(raw, -vehicle.deceleration_limit_mps2, vehicle.acceleration_limit_mps2))
        if (y[1] <= vehicle.speed_min_mps and a < 0) or (y[1] >= vehicle.speed_max_mps and a > 0):
            a = 0.0
        return a, gap, gap_ref, lv, source

    t, y = 0.0, np.array([0.0, vehicle.cruise_mps])
    trace, max_projection = [], 0.0
    while True:
        _, lv, la = leader_state(t, vehicle.cruise_mps, braking)
        if mode == "aks" and not invalidated and (abs(lv - vehicle.cruise_mps) > 1e-9 or la != 0):
            invalidated = True
            events.append({"t_s": t, "event": "reference_invalidated_leader_changed"})
        # A gap can only open while the follower is slower than the leader. Once
        # the leader is at or below the follower's wing-borne floor that is no
        # longer possible: the follower cannot slow further, cannot wait, and
        # cannot descend out of cruise. The gap freezes wherever it happens to
        # be, below what the policy asks for, with nothing left to recover it.
        # Recorded once, as a countable state rather than a silent saturation.
        if not exhausted[0] and lv <= vehicle.speed_min_mps + 1e-9 \
                and y[1] <= vehicle.speed_min_mps + 1e-6 and gap < gap_ref - 1e-6:
            exhausted[0] = True
            events.append({"t_s": t, "event": "speed_margin_exhausted_gap_frozen"})
            diagnostics["speed_margin_exhausted_gap_frozen"] = {
                "t_s": t, "leader_speed_mps": float(lv),
                "follower_speed_mps": float(y[1]),
                "gap_m": float(gap), "required_gap_m": float(gap_ref)}
        a, gap, gap_ref, lv, source = command(t, y)
        trace.append({"t_s": t, "follower_x_m": float(y[0]), "follower_v_mps": float(y[1]),
            "leader_x_m": initial_gap + leader_state(t, vehicle.cruise_mps, braking)[0],
            "leader_v_mps": lv, "gap_m": gap, "reference_gap_m": gap_ref,
            "acceleration_mps2": a, "controller": source})
        if t >= end - 1e-10:
            break
        h = min(dt, min(b for b in boundaries if b > t + 1e-10) - t)
        def rhs(time, values):
            return np.array([values[1], command(time, values)[0]])
        # Resting on a speed bound is a legitimate state, not an integration
        # error. Stepping RK4 through it anyway produces sub-bound intermediate
        # stages that then get projected back, which reads as integrator drift.
        # When the state sits on a bound and the command pushes into it, the
        # exact solution is constant speed, so integrate that instead.
        on_floor = y[1] <= vehicle.speed_min_mps + 1e-12 and a <= 0.0
        on_ceiling = y[1] >= vehicle.speed_max_mps - 1e-12 and a >= 0.0
        if on_floor or on_ceiling:
            y = np.array([y[0] + h*y[1], y[1]])
        else:
            # A step that would cross a speed bound is cut at the crossing, the
            # same way steps are already cut at reference phases and leader
            # events. Without this the step lands beyond the bound and is
            # projected back, which is a first-order error: halving the step
            # only halves it, so it never reaches a tight tolerance.
            def step(width):
                k1 = rhs(t, y)
                k2 = rhs(t + width/2, y + width*k1/2)
                k3 = rhs(t + width/2, y + width*k2/2)
                k4 = rhs(t + width, y + width*k3)
                return y + width*(k1 + 2*k2 + 2*k3 + k4)/6
            trial = step(h)
            bound = None
            if trial[1] < vehicle.speed_min_mps:
                bound = vehicle.speed_min_mps
            elif trial[1] > vehicle.speed_max_mps:
                bound = vehicle.speed_max_mps
            if bound is not None:
                low, high = 0.0, h
                for _ in range(60):
                    mid = 0.5*(low + high)
                    inside = (step(mid)[1] >= vehicle.speed_min_mps
                              and step(mid)[1] <= vehicle.speed_max_mps)
                    low, high = (mid, high) if inside else (low, mid)
                h = high
                trial = step(h)
                # Contacting a speed bound steps the acceleration discontinuously,
                # exactly like a leader event, so the audit must skip that
                # interval. The contact time is solved, not a grid time: it
                # carries the integrator's truncation error and can never be
                # identical across steps, so it is recorded as a discontinuity.
                # The decision -- that a bound was reached, and which one -- is
                # what the log carries, and that is step-independent.
                discontinuities.append(float(t + h))
                events.append({"event": "speed_bound_contact",
                               "bound_mps": float(bound)})
            clipped = np.clip(trial[1], vehicle.speed_min_mps, vehicle.speed_max_mps)
            max_projection = max(max_projection, abs(clipped - trial[1]))
            y = np.array([trial[0], clipped])
        t = min(end, t + h)
    serial_request = {k: v for k, v in request.items() if k != "transition"}
    if transition:
        serial_request["transition_parameters"] = asdict(transition)
    return {"case_id": case["id"], "mode": mode, "dt_s": dt,
            "established_following": bool(established),
            "acc_policy_recovery_active": bool(recovery_pairs),
            "request": serial_request, "events": events,
            "discontinuities": discontinuities, "diagnostics": diagnostics,
            "trace": trace,
            "resolved": {"vehicle": asdict(vehicle), "spacing": asdict(cfg.spacing),
                         "controller": asdict(cfg.controller)},
            "maximum_speed_projection_mps": float(max_projection)}
