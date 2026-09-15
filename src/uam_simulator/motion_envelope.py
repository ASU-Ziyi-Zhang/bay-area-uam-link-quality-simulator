"""Coupled longitudinal-lateral acceleration envelope for corridor motion.

Stage 1 bounds only the maneuver-induced offset acceleration ``d_ddot``. The
reference route itself turns, so the physical normal acceleration is

    a_N = V^2 kappa / (1 - kappa d) + d_ddot

and the moving frame also produces a tangential term while the offset changes,

    a_T = -d_dot kappa V / (1 - kappa d) + a_lon_cmd

where ``a_lon_cmd`` is any longitudinal (ACC) command. Both expressions match
:meth:`uam_simulator.lateral_study.SmoothCorridorFrame.kinematics`; this module
adds the longitudinal command term and the joint feasibility test.

Measured on ``scenarios/airport_to_airport`` at V = 50 m/s, the smoothed
centreline reaches |a_N| = 2.561 m/s^2 (minimum radius 976.2 m) and exceeds
0.981 m/s^2 over 5.27% of the route with no maneuver at all.

The two normal terms are therefore limited **separately**, which is the default
``maneuver_only`` coupling mode. Following the alignment is a steady coordinated
turn: the aircraft banks, occupants feel an increased vertical load rather than
a side force, and the cost is load factor ``1/cos(phi)`` and power, not
altitude. A 2.561 m/s^2 turn is a 14.6 degree bank and a 3.4% load-factor
increase. That term is checked against its own bank-angle allowance, while the
lane-change term ``d_ddot`` is what the 0.1 g figure bounds. The ``total`` mode
applies the single limit to the sum and is kept as a sensitivity case, because
the source for 0.1 g is a spoken remark made while comparing lane-change path
families rather than a written specification.

Jerk is bounded here too, optionally and off by default. It was named alongside
maximum curvature as the pair a lane-change path should satisfy, and nothing in
the model has bounded it. For the quintic the peak jerk ``60 D / T^3`` occurs
**at the endpoints**, stepping from zero when the maneuver begins.

Nothing here is a certified flight-performance model. The limits are research
assumptions whose sources are recorded in :class:`AccelerationEnvelope`.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

#: Standard gravity used to convert between an acceleration and a coordinated
#: bank angle. Only a reporting convention here.
G_MPS2 = 9.81

#: Peak magnitudes of the normalised quintic ``s(u) = 10u^3 - 15u^4 + 6u^5``
#: and its derivatives, so a maneuver of displacement ``D`` over duration ``T``
#: reaches ``factor * D / T^n``. Verified numerically over a 4e6-point grid.
#:
#: The acceleration peak sits at ``u = 0.211325`` and ``u = 0.788675``, i.e. the
#: roots of ``u^2 - u + 1/6``, **not** at the quarter points.
#:
#: The jerk peak sits at the **endpoints**: before the maneuver the lateral jerk
#: is zero and at ``u = 0`` it steps to ``60 D / T^3``. The quintic is therefore
#: C2 but not C3 across the start and end of a lane change. This is the property
#: a clothoid construction avoids, since linear curvature growth keeps jerk
#: bounded and matched at the joins.
QUINTIC_PEAK_SPEED_FACTOR = 1.875           # 15/8, at u = 1/2
QUINTIC_PEAK_ACCEL_FACTOR = 10 / math.sqrt(3)   # 5.773503, at u = 1/2 -/+ 1/(2*sqrt(3))
QUINTIC_PEAK_JERK_FACTOR = 60.0             # at u = 0 and u = 1

#: Provenance of every default in :class:`AccelerationEnvelope`. ``advised``
#: means a collaborator suggested the magnitude in discussion; ``placeholder``
#: means the project chose it with no external basis.
DEFAULT_PROVENANCE = {
    "longitudinal_accel_max_mps2": "placeholder: simulator default (geographic_traffic.acceleration_limit_mps2)",
    "longitudinal_decel_max_mps2": "placeholder: simulator default (geographic_traffic.deceleration_limit_mps2)",
    "lateral_accel_max_mps2": (
        "advised: 0.1 g discussed 2026-09-01, 0.2 g named as a higher case. "
        "Raised while comparing quintic/Bezier/clothoid lane-change paths "
        "('for those lane changes ... the curvature would give you your maximum "
        "lateral acceleration'), and motivated by steering authority in the car "
        "context, so it is read here as a limit on the MANEUVER component"),
    "route_normal_accel_max_mps2": (
        "placeholder: 9.81*tan(25 deg) = 4.575, i.e. a 25 degree coordinated "
        "bank. Allowance for the steady turn that following the reference "
        "alignment requires; flown by banking, so occupants feel an increased "
        "vertical load rather than a side force. 25 deg is the usual normal-"
        "operations bank limit for transport aircraft, not an eVTOL figure"),
    "lateral_speed_max_mps": (
        "unset by default: falls back to the simulator config value of 8 m/s, "
        "which has no external source and becomes the binding constraint once "
        "the acceleration limit is relaxed"),
    "min_maneuver_s": (
        "unset by default: falls back to the simulator config floor of 30 s, "
        "which has no external source and binds the shortest lane changes"),
    "lateral_jerk_max_mps3": (
        "unset by default: jerk was named alongside maximum curvature as the "
        "pair a lane-change path should satisfy, but no magnitude was given and "
        "the model has never bounded it. Set a value to activate the bound"),
    "vertical_accel_max_mps2": "placeholder: spatial_study default",
}

#: How the route's steady turn is treated relative to the maneuver limit.
COUPLING_MODES = ("maneuver_only", "total")


@dataclass(frozen=True)
class AccelerationEnvelope:
    """Asymmetric longitudinal limits combined with a lateral limit.

    The admissible set is the ellipse ``(a_T/lon)^2 + (a_N/lat)^2 <= 1`` with
    ``lon`` selected per sign of ``a_T``. This is an *assumed* coupling: the
    automotive friction circle arises from tyre-road contact and does not by
    itself establish that an eVTOL shares one budget across axes. Treat it as a
    coupling hypothesis to be tested, not as a vehicle capability model.

    ``coupling_mode`` selects which normal acceleration the lateral limit binds,
    because the two readings give different answers and the source is a spoken
    remark rather than a written specification:

    ``"maneuver_only"`` (default)
        The lateral limit binds only the maneuver-induced ``d_ddot``. Following
        the reference alignment is a steady coordinated turn, flown by banking,
        in which occupants feel an increased vertical load rather than a side
        force; it is checked separately against
        ``route_normal_accel_max_mps2`` and reported as a bank angle.

    ``"total"``
        The lateral limit binds ``V^2 kappa/(1 - kappa d) + d_ddot``. Retained
        as a sensitivity case; on a route whose curvature was not designed for
        this limit it can declare part of the alignment infeasible.
    """

    longitudinal_accel_max_mps2: float = 1.5
    longitudinal_decel_max_mps2: float = 2.0
    lateral_accel_max_mps2: float = 0.981
    route_normal_accel_max_mps2: float = 4.575
    lateral_jerk_max_mps3: float | None = None
    lateral_speed_max_mps: float | None = None
    min_maneuver_s: float | None = None
    vertical_accel_max_mps2: float = 0.2
    coupling_mode: str = "maneuver_only"
    provenance: dict = field(default_factory=lambda: dict(DEFAULT_PROVENANCE))

    def __post_init__(self):
        for name in ("longitudinal_accel_max_mps2", "longitudinal_decel_max_mps2",
                     "lateral_accel_max_mps2", "route_normal_accel_max_mps2",
                     "vertical_accel_max_mps2"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("lateral_speed_max_mps", "min_maneuver_s"):
            value = getattr(self, name)
            if value is not None and (not np.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive when set")
        if self.lateral_jerk_max_mps3 is not None and (
                not np.isfinite(self.lateral_jerk_max_mps3)
                or self.lateral_jerk_max_mps3 <= 0):
            raise ValueError("lateral_jerk_max_mps3 must be finite and positive when set")
        if self.coupling_mode not in COUPLING_MODES:
            raise ValueError(f"coupling_mode must be one of {COUPLING_MODES}")

    @property
    def route_bank_angle_max_deg(self):
        """The steady-turn allowance expressed as a coordinated bank angle."""
        return float(np.degrees(np.arctan(self.route_normal_accel_max_mps2 / G_MPS2)))

    def binding_normal(self, maneuver_normal, route_normal):
        """The normal acceleration the lateral limit applies to in this mode."""
        maneuver_normal = np.asarray(maneuver_normal, float)
        route_normal = np.asarray(route_normal, float)
        if self.coupling_mode == "total":
            return maneuver_normal + route_normal
        return maneuver_normal

    def duration_inputs(self, cfg):
        """Motion limits used to size a maneuver.

        When the envelope carries a value it wins; otherwise the simulator
        config supplies it. Supplying an envelope therefore makes it the single
        authority on the lateral limits, so raising ``lateral_accel_max_mps2``
        actually shortens maneuvers instead of only relaxing the ellipse check.
        """
        return {
            "speed_limit": (cfg.lateral_speed_limit_mps
                            if self.lateral_speed_max_mps is None
                            else self.lateral_speed_max_mps),
            "accel_limit": self.lateral_accel_max_mps2,
            "minimum_s": (cfg.min_maneuver_s if self.min_maneuver_s is None
                          else self.min_maneuver_s),
            "jerk_limit": self.lateral_jerk_max_mps3,
        }

    def route_admissible(self, route_normal, tolerance=1e-9):
        """Whether the steady turn stays inside its own allowance."""
        return (np.abs(np.asarray(route_normal, float))
                <= self.route_normal_accel_max_mps2 + tolerance)

    def longitudinal_limit(self, tangential):
        """Limit that applies to ``tangential``; acceleration and braking differ."""
        tangential = np.asarray(tangential, float)
        return np.where(tangential >= 0, self.longitudinal_accel_max_mps2,
                        self.longitudinal_decel_max_mps2)

    def utilisation(self, tangential, normal, vertical=0.0):
        """Squared ellipse coordinate; ``<= 1`` is admissible.

        Vertical acceleration enters as a third semi-axis when supplied, so the
        same call covers the constant-altitude and level-change cases.
        """
        tangential = np.asarray(tangential, float)
        normal = np.asarray(normal, float)
        vertical = np.asarray(vertical, float)
        return ((tangential / self.longitudinal_limit(tangential)) ** 2
                + (normal / self.lateral_accel_max_mps2) ** 2
                + (vertical / self.vertical_accel_max_mps2) ** 2)

    def admissible(self, tangential, normal, vertical=0.0, tolerance=1e-9):
        return self.utilisation(tangential, normal, vertical) <= 1.0 + tolerance


def frame_accelerations(curvature, offset, offset_speed, offset_acceleration,
                        cruise, longitudinal_command=0.0):
    """Tangential and normal acceleration in the moving corridor frame.

    Mirrors ``SmoothCorridorFrame.kinematics`` and adds the longitudinal
    command. ``1 - kappa*d`` must stay positive; the caller is responsible for
    the corridor-regularity check.
    """
    curvature = np.asarray(curvature, float)
    offset = np.asarray(offset, float)
    factor = 1.0 - curvature * offset
    if np.any(factor <= 0.05):
        raise ValueError("folded or nearly singular lateral coordinate")
    rate_arc = cruise / factor
    tangential = -np.asarray(offset_speed, float) * curvature * rate_arc + longitudinal_command
    normal = cruise * curvature * rate_arc + np.asarray(offset_acceleration, float)
    return tangential, normal


def turning_load(frame, q, offset, cruise):
    """Normal acceleration produced by the route alone (``d_ddot = 0``)."""
    _, _, _, _, curvature = frame.frame(q)
    return frame_accelerations(curvature, offset, 0.0, 0.0, cruise)[1]


def offset_acceleration_budget(frame, q, offset, cruise, envelope,
                               longitudinal_command=0.0):
    """Largest ``|d_ddot|`` still inside the envelope at this corridor point.

    In ``maneuver_only`` mode the route's steady turn does not consume the
    lateral budget: it is flown by banking and is checked separately. Only the
    longitudinal command competes with the maneuver through the ellipse.

    In ``total`` mode the route turning is subtracted as well, and the result
    is ``0.0`` wherever the alignment alone already exhausts the limit.
    """
    _, _, _, _, curvature = frame.frame(q)
    tangential, normal_turn = frame_accelerations(
        curvature, offset, 0.0, 0.0, cruise, longitudinal_command)
    used = (np.asarray(tangential, float) / envelope.longitudinal_limit(tangential)) ** 2
    remaining = np.maximum(1.0 - used, 0.0)
    reachable = envelope.lateral_accel_max_mps2 * np.sqrt(remaining)
    if envelope.coupling_mode == "total":
        reachable = reachable - np.abs(normal_turn)
    return np.maximum(np.broadcast_to(reachable, np.shape(np.asarray(q, float))), 0.0)


def quintic_duration_bounds(distance, speed_limit, accel_limit, minimum_s,
                            jerk_limit=None):
    """Closed-form lower bounds for the quintic profile ``10u^3-15u^4+6u^5``.

    Peak transverse speed is ``1.875 |dy|/T``, peak transverse acceleration is
    ``(10/sqrt(3)) |dy|/T^2``, and peak transverse jerk is ``60 |dy|/T^3``.
    Inverting each gives a lower bound on the duration, and the binding one is
    the largest.

    ``jerk_limit`` is optional and defaults to unenforced, which reproduces the
    archived behaviour. It was named alongside maximum curvature as the pair of
    quantities a lane-change path should satisfy, and nothing in the current
    model bounds it.

    These bound the maneuver components only; the coupled envelope is enforced
    separately because the route turning term varies along the maneuver.
    """
    distance = abs(float(distance))
    if distance == 0:
        return 0.0
    bounds = [float(minimum_s),
              QUINTIC_PEAK_SPEED_FACTOR * distance / speed_limit,
              math.sqrt(QUINTIC_PEAK_ACCEL_FACTOR * distance / accel_limit)]
    if jerk_limit is not None:
        if not np.isfinite(jerk_limit) or jerk_limit <= 0:
            raise ValueError("jerk_limit must be finite and positive")
        bounds.append((QUINTIC_PEAK_JERK_FACTOR * distance / jerk_limit) ** (1 / 3))
    return max(bounds)


def quintic_peak_demands(distance, duration):
    """Peak transverse speed, acceleration and jerk of one quintic maneuver.

    ``endpoint_jerk_step`` records that the jerk peak is reached at the start
    and end of the maneuver, stepping from zero. It is the same magnitude as
    the peak; it is reported separately because a bounded peak and a continuous
    profile are different properties, and only the first is a limit check.
    """
    distance = abs(float(distance))
    duration = float(duration)
    if duration <= 0:
        raise ValueError("duration must be positive")
    jerk = QUINTIC_PEAK_JERK_FACTOR * distance / duration ** 3
    return {"peak_lateral_speed_mps": QUINTIC_PEAK_SPEED_FACTOR * distance / duration,
            "peak_lateral_accel_mps2": QUINTIC_PEAK_ACCEL_FACTOR * distance / duration ** 2,
            "peak_lateral_jerk_mps3": jerk,
            "peak_accel_at_u": (0.5 - 1 / (2 * math.sqrt(3)), 0.5 + 1 / (2 * math.sqrt(3))),
            "peak_jerk_at_u": (0.0, 1.0),
            "endpoint_jerk_step_mps3": jerk,
            "profile_is_c2_not_c3": True}


def envelope_profile(frame, cfg, envelope, *, q0, t0, source, target, duration,
                     longitudinal_command=0.0, min_samples=51, max_sample_step_s=1.0):
    """Worst envelope utilisation over one quintic maneuver.

    Integrates corridor progress with the same error-controlled RK4 the motion
    loop uses, so the reported utilisation refers to the corridor points the
    aircraft actually visits.

    Resolution is fixed in **time**, not as a fixed sample count. A fixed count
    would thin out as the duration grows, and a long maneuver could then alias
    straight through a curvature peak and be reported admissible. Sampling
    remains a check, not a continuous proof; the step is reported so a reviewer
    can vary it.
    """
    from .lateral_study import offset_values, rk4_progress

    if duration <= 0:
        raise ValueError("duration must be positive")
    if max_sample_step_s <= 0:
        raise ValueError("max_sample_step_s must be positive")
    targets = np.array([float(target)])
    durations = np.array([float(duration)])

    def offset_at(time):
        return offset_values(time, t0, float(source), targets, durations)[0]

    samples = max(int(min_samples), int(math.ceil(duration / max_sample_step_s)) + 1)
    times = np.linspace(t0, t0 + duration, int(samples))
    q = np.array([float(q0)])
    worst = 0.0
    worst_time = float(t0)
    worst_route = 0.0
    route_ok = True
    rows = []
    for index, time in enumerate(times):
        if index:
            step = float(time - times[index - 1])
            q = np.asarray(rk4_progress(frame, q, times[index - 1], step,
                                        offset_at, cfg.cruise_mps), float)
        d, vd, ad = (float(value[0]) for value in
                     offset_values(time, t0, float(source), targets, durations))
        _, _, _, _, curvature = frame.frame(q)
        # Split the normal axis: the steady turn the alignment demands, and the
        # maneuver the lane change adds on top of it.
        tangential, route_normal = frame_accelerations(
            curvature, d, vd, 0.0, cfg.cruise_mps, longitudinal_command)
        route_normal = float(np.ravel(route_normal)[0])
        tangential = float(np.ravel(tangential)[0])
        maneuver_normal = float(ad)
        binding = float(envelope.binding_normal(maneuver_normal, route_normal))
        value = float(envelope.utilisation(tangential, binding))
        route_admissible = bool(envelope.route_admissible(route_normal))
        route_ok = route_ok and route_admissible
        rows.append({"t_s": float(time), "q_m": float(q[0]), "offset_m": d,
                     "tangential_mps2": tangential,
                     "route_normal_mps2": route_normal,
                     "maneuver_normal_mps2": maneuver_normal,
                     "total_normal_mps2": route_normal + maneuver_normal,
                     "route_bank_angle_deg": float(np.degrees(
                         np.arctan(abs(route_normal) / G_MPS2))),
                     "route_within_allowance": route_admissible,
                     "utilisation": value})
        worst_route = max(worst_route, abs(route_normal))
        if value > worst:
            worst, worst_time = value, float(time)
    return {"max_utilisation": worst, "max_utilisation_time_s": worst_time,
            "admissible": worst <= 1.0 + 1e-9 and route_ok,
            "envelope_admissible": worst <= 1.0 + 1e-9,
            "route_within_allowance": route_ok,
            "max_route_normal_mps2": worst_route,
            "max_route_bank_angle_deg": float(np.degrees(
                np.arctan(worst_route / G_MPS2))),
            "coupling_mode": envelope.coupling_mode,
            "samples": int(samples),
            "sample_step_s": float(duration / (samples - 1)) if samples > 1 else 0.0,
            "sampled_check_not_continuous_proof": True, "profile": rows}


def feasible_duration(frame, cfg, envelope, *, q0, t0, source, target,
                      longitudinal_command=0.0, growth=1.2, refine_iterations=12,
                      max_duration_s=None, duration_cap_multiple=4.0,
                      min_samples=51, max_sample_step_s=1.0):
    """Search for a maneuver duration admissible under the coupled envelope.

    Starts from the closed-form ``d_dot``/``d_ddot`` bounds and grows the
    duration until the sampled envelope check passes.

    Two properties make this a search rather than a solve. Lengthening a
    maneuver reduces the maneuver-induced terms, which scale as ``1/T`` and
    ``1/T^2``, but it also carries the aircraft over a longer and possibly more
    curved stretch of route, and the route turning term does not depend on
    ``T`` at all. Feasibility is therefore not monotone in duration, and where
    the route alone already consumes the lateral budget no duration helps.

    The duration is capped so the search cannot wander into maneuvers longer
    than the remaining corridor. Without a cap the growth loop will happily
    return a duration whose ground track exceeds the route length. The result
    is *a* feasible duration found by search, not a proven minimum.
    """
    distance = float(target) - float(source)
    limits = envelope.duration_inputs(cfg)
    baseline = quintic_duration_bounds(distance, limits["speed_limit"],
                                       limits["accel_limit"], limits["minimum_s"],
                                       limits["jerk_limit"])
    unbounded = quintic_duration_bounds(distance, limits["speed_limit"],
                                        limits["accel_limit"], limits["minimum_s"])
    if baseline == 0.0:
        return {"duration_s": 0.0, "baseline_duration_s": 0.0, "status": "stay",
                "max_utilisation": 0.0, "attempts": 0, "minimality_proven": False}

    remaining_s = max(float(frame.length_m) - float(q0), 0.0) / cfg.cruise_mps
    cap = min(duration_cap_multiple * baseline, remaining_s)
    if max_duration_s is not None:
        cap = min(cap, float(max_duration_s))

    def check(duration):
        return envelope_profile(frame, cfg, envelope, q0=q0, t0=t0, source=source,
                                target=target, duration=duration,
                                longitudinal_command=longitudinal_command,
                                min_samples=min_samples,
                                max_sample_step_s=max_sample_step_s)

    def turning_blocks(duration):
        """Does the route alone exhaust the lateral budget somewhere on this span?"""
        span = np.linspace(float(q0),
                           min(float(q0) + cfg.cruise_mps * duration, frame.length_m), 400)
        budget = offset_acceleration_budget(frame, span, float(source), cfg.cruise_mps,
                                            envelope, longitudinal_command)
        return bool(np.any(np.asarray(budget) <= 0.0))

    if cap < baseline:
        return {"duration_s": None, "baseline_duration_s": baseline,
                "status": "insufficient_remaining_corridor",
                "max_utilisation": float("nan"), "duration_cap_s": cap,
                "attempts": 0, "minimality_proven": False}

    jerk_binds = baseline > unbounded + 1e-9
    first = check(baseline)
    if first["admissible"]:
        return {"duration_s": baseline, "baseline_duration_s": baseline,
                "status": "baseline_admissible", "max_utilisation": first["max_utilisation"],
                "duration_cap_s": cap, "attempts": 1, "minimality_proven": False,
                "jerk_limit_binds_baseline": jerk_binds,
                "peak_demands": quintic_peak_demands(distance, baseline)}

    low, duration, attempt = baseline, baseline, 1
    while duration < cap:
        duration = min(duration * growth, cap)
        attempt += 1
        report = check(duration)
        if report["admissible"]:
            high = duration
            for _ in range(refine_iterations):
                mid = 0.5 * (low + high)
                if check(mid)["admissible"]:
                    high = mid
                else:
                    low = mid
            final = check(high)
            return {"duration_s": high, "baseline_duration_s": baseline,
                    "status": "extended_for_envelope",
                    "max_utilisation": final["max_utilisation"],
                    "duration_cap_s": cap, "attempts": attempt,
                    "minimality_proven": False}
        low = duration
    final = check(cap)
    blocked = turning_blocks(cap)
    if not final["route_within_allowance"]:
        status = "route_bank_exceeds_allowance"
    elif blocked:
        status = "route_turning_exceeds_limit"
    else:
        status = "no_admissible_duration_within_cap"
    return {"duration_s": None, "baseline_duration_s": baseline,
            "status": status, "max_utilisation": final["max_utilisation"],
            "duration_cap_s": cap, "attempts": attempt,
            "coupling_mode": envelope.coupling_mode,
            "max_route_bank_angle_deg": final["max_route_bank_angle_deg"],
            "route_turning_exhausts_budget": blocked, "minimality_proven": False}


def corridor_turning_report(frame, cfg, envelope, offsets, samples=20001):
    """How much of the route the turning term alone already consumes.

    Reported per candidate offset because ``1 - kappa d`` scales the turning
    term: an outboard offset in a curve carries a larger normal acceleration
    than the centreline does.
    """
    q = np.linspace(0.0, frame.length_m, int(samples))
    _, _, _, _, curvature = frame.frame(q)
    rows = []
    for offset in np.asarray(offsets, float):
        normal = frame_accelerations(curvature, offset, 0.0, 0.0, cfg.cruise_mps)[1]
        magnitude = np.abs(normal)
        bank = np.degrees(np.arctan(magnitude / G_MPS2))
        rows.append({
            "offset_m": float(offset),
            "max_turning_normal_mps2": float(magnitude.max()),
            "mean_turning_normal_mps2": float(magnitude.mean()),
            "p95_turning_normal_mps2": float(np.percentile(magnitude, 95)),
            "max_bank_angle_deg": float(bank.max()),
            "p95_bank_angle_deg": float(np.percentile(bank, 95)),
            "fraction_over_route_allowance": float(np.mean(
                magnitude > envelope.route_normal_accel_max_mps2)),
            # Reported for the "total" reading only; in "maneuver_only" mode the
            # steady turn does not draw on the maneuver budget.
            "fraction_over_lateral_limit": float(np.mean(
                magnitude > envelope.lateral_accel_max_mps2)),
        })
    return {"cruise_mps": cfg.cruise_mps,
            "coupling_mode": envelope.coupling_mode,
            "lateral_accel_max_mps2": envelope.lateral_accel_max_mps2,
            "route_normal_accel_max_mps2": envelope.route_normal_accel_max_mps2,
            "route_bank_angle_max_deg": envelope.route_bank_angle_max_deg,
            "minimum_turn_radius_m": float(1.0 / np.max(np.abs(curvature))),
            "samples": int(samples), "rows": rows}
