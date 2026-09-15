"""Five-aircraft, two-lane rig for the stay-or-change-lane decision.

Roles: ``A`` is the aircraft deciding whether to change lane, the one whose
policy degrades; ``B`` and ``C`` are the leader and follower in its source
lane; ``D`` and ``E`` occupy the target lane ahead of and behind the insertion
point. Any neighbour may be disabled to form the three- and four-aircraft
diagnostic configurations; the remaining aircraft keep their initial
conditions. (Runs before R0035 used the earlier names E, L, B, NF, NR for the
same five roles, in that order.)

Candidates are checked before ranking. Predictions clone the complete physical
and controller state, including policy histories. A future start executes STAY
now; an already started maneuver retains its continuous trajectory. Admission
uses an explicitly supplied research volume, with NMAC reported separately.

Straight lanes at a common altitude for the first round, so route curvature is
zero and the normal acceleration is the maneuver term alone. The frame is kept
explicit so curvature can be reintroduced without redefining the checks.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from copy import deepcopy
from typing import NamedTuple
from itertools import combinations
from numpy.polynomial import Polynomial
from enum import Enum
from functools import cached_property
import math

import numpy as np

from .motion_envelope import (
    QUINTIC_PEAK_ACCEL_FACTOR, QUINTIC_PEAK_JERK_FACTOR, QUINTIC_PEAK_SPEED_FACTOR,
    AccelerationEnvelope,
)
from .safety import SeparationVolume, screen_trajectories


class Action(Enum):
    STAY = "stay"
    CHANGE = "change"


# --- lateral profile -------------------------------------------------------

def quintic(u):
    """``s`` and its first three derivatives with respect to ``u`` on [0, 1].

    Returned analytically rather than by differencing a sampled array: the
    third derivative of a numerically differentiated signal is dominated by
    round-off amplification and produced a wrong endpoint jerk in an earlier
    revision of this project's notes.
    """
    u = np.clip(np.asarray(u, float), 0.0, 1.0)
    s = 10 * u**3 - 15 * u**4 + 6 * u**5
    d1 = 30 * u**2 - 60 * u**3 + 30 * u**4
    d2 = 60 * u - 180 * u**2 + 120 * u**3
    d3 = 60 - 360 * u + 360 * u**2
    return s, d1, d2, d3


@dataclass(frozen=True)
class LateralShape:
    """Normalised lane-change shape ``s(u)`` on [0, 1] as exact polynomial pieces.

    Every member satisfies s(0)=0, s(1)=1 and s'(0)=s'(1)=0. All members except
    the cubic Bezier also have vanishing second derivatives at both ends, so they
    join straight flight with continuous offset, transverse speed and transverse
    acceleration (C2); the cubic Bezier is C1 and its transverse acceleration
    steps at the start and end of the manoeuvre. Keeping each
    piece an exact polynomial in ``u`` means the rig's segment audit remains an
    exact root test for every shape, not a sampled one.

    ``coefficients[k]`` is the piece on ``[breaks[k], breaks[k+1]]``, written in
    the global ``u``. Stored as tuples so a profile stays hashable.
    """

    name: str
    breaks: tuple
    coefficients: tuple
    params: tuple = ()

    def __post_init__(self):
        if len(self.coefficients) != len(self.breaks)-1 or self.breaks[0] != 0. or self.breaks[-1] != 1.:
            raise ValueError("pieces must cover [0, 1] exactly")
        if any(b <= a for a, b in zip(self.breaks, self.breaks[1:])):
            raise ValueError("breaks must increase")

    @property
    def junctions(self):
        return tuple(self.breaks[1:-1])

    def piece_index(self, u):
        return int(np.searchsorted(np.asarray(self.junctions, float), float(u)+1e-12, side="right"))

    def polynomial(self, index):
        return Polynomial(self.coefficients[index])

    def evaluate(self, u):
        """Normalised offset and its first three derivatives with respect to ``u``."""
        u = np.clip(np.atleast_1d(np.asarray(u, float)), 0.0, 1.0)
        out = [np.zeros_like(u) for _ in range(4)]
        index = np.searchsorted(np.asarray(self.junctions, float), u, side="right")
        for k in range(len(self.coefficients)):
            mask = index == k
            if mask.any():
                poly = self.polynomial(k)
                for order in range(4):
                    out[order][mask] = poly.deriv(order)(u[mask])
        return tuple(out)

    @cached_property
    def peak_factors(self):
        """Peak magnitudes of the first, second and third derivative over [0, 1].

        Taken at piece endpoints and at the real roots of the next derivative
        inside each piece, so they are exact rather than sampled.
        """
        out = {}
        for order, key in ((1, "speed"), (2, "accel"), (3, "jerk")):
            best = 0.0
            for k in range(len(self.coefficients)):
                a, b = self.breaks[k], self.breaks[k+1]
                poly = self.polynomial(k).deriv(order)
                points = [a, b]
                slope = poly.deriv()
                if slope.degree() > 0:
                    points += [float(r.real) for r in slope.roots()
                               if abs(r.imag) < 1e-10 and a < r.real < b]
                best = max(best, max(abs(float(poly(x))) for x in points))
            out[key] = best
        return out

    @cached_property
    def midpoint_u(self):
        """``u`` where the shape has covered half the lateral distance.

        The aircraft is counted into the target lane here, so this is a policy
        boundary, not a plotting convenience: it must be exact. ``s`` rises
        monotonically from 0 to 1, so exactly one root exists; it is taken from
        the piece that contains it rather than by sampling. Asymmetric shapes
        cross away from ``u = 0.5``.
        """
        for k in range(len(self.coefficients)):
            a, b = self.breaks[k], self.breaks[k + 1]
            roots = sorted(float(r.real) for r in (self.polynomial(k) - 0.5).roots()
                           if abs(r.imag) < 1e-10 and a - 1e-12 <= r.real <= b + 1e-12)
            if roots:
                return min(max(roots[0], 0.0), 1.0)
        raise ValueError("shape never reaches half the lateral distance")


QUINTIC_SHAPE = LateralShape("quintic", (0., 1.), ((0., 0., 0., 10., -15., 6.),))


def beta_shape(p, q):
    """AKS beta-kernel lane change: transverse speed proportional to u^p (1-u)^q.

    The same kernel as the TR-C manuscript's KS1A velocity transition, applied
    to lateral position. p = q = 2 is exactly the quintic and p = q = 3 the
    septic; p > q shifts the lateral motion later and p < q earlier. Integer
    p, q >= 2 keeps the shape a single exact polynomial with C2 endpoints;
    real-valued exponents are left for a later round because they leave the
    polynomial audit.
    """
    if int(p) != p or int(q) != q or p < 2 or q < 2:
        raise ValueError("integer p, q >= 2 required for an exact C2 polynomial")
    p, q = int(p), int(q)
    kernel = Polynomial([0., 1.])**p * Polynomial([1., -1.])**q
    s = kernel.integ(lbnd=0.)
    s = s / s(1.0)
    return LateralShape(f"beta_{p}_{q}", (0., 1.), (tuple(float(c) for c in s.coef),),
                        (("p", p), ("q", q)))


def three_clothoid_shape(fraction=0.25):
    """Symmetric three-clothoid lane change, transcribed to the Frenet frame.

    Oh et al. (AVEC 2022; IEEE TCST 2025) build the path from three clothoids
    whose curvature is piecewise linear in arclength, with G2 ends and equal
    first and last lengths. On a straight corridor at constant along-track
    speed, curvature is the transverse acceleration over V squared and
    arclength is V t, both to within cos(psi); the rig caps transverse speed at
    8 m/s against 50 m/s, so psi <= 9.09 deg and the transcription error is at
    most 1.26 percent. The result is transverse acceleration piecewise linear
    in time: 0 to +A over ``fraction``, +A to -A over ``1 - 2 fraction``, and
    -A to 0 over ``fraction``. fraction = 0.25 gives equal jerk magnitude on
    all three pieces.
    """
    f = float(fraction)
    if not 0. < f < 0.5:
        raise ValueError("fraction must lie in (0, 0.5)")
    u = Polynomial([0., 1.])
    accel = (u/f, 1 - 2*(u-f)/(1-2*f), -1 + (u-(1-f))/f)
    lows = (0., f, 1-f)
    speed, pos = [], []
    for k, (a, lo) in enumerate(zip(accel, lows)):
        v = a.integ(lbnd=lo, k=0. if k == 0 else speed[-1](lo))
        x = v.integ(lbnd=lo, k=0. if k == 0 else pos[-1](lo))
        speed.append(v); pos.append(x)
    total = pos[-1](1.0)
    return LateralShape(f"clothoid3_{f:g}", (0., f, 1-f, 1.),
                        tuple(tuple(float(c) for c in (x/total).coef) for x in pos),
                        (("fraction", f),))


def bezier3_shape():
    """Cubic Bezier lane change with evenly spaced control points.

    Control points (0, 0), (1/3, 0), (2/3, 1), (1, 1) in normalised (along-track,
    offset) coordinates make the along-track coordinate linear in the Bezier
    parameter, so at constant along-track speed the offset is s(u) = 3u^2 - 2u^3.
    Transverse speed is zero at both ends, but s''(0) = 6 and s''(1) = -6: the
    transverse acceleration steps from zero at the start and back to zero at the
    end (C1, not C2), and the jerk is an impulse there. This is the contrast
    with the other shapes, all of which have C2 ends.
    """
    return LateralShape("bezier3", (0., 1.), ((0., 0., 3., -2.),))


@dataclass(frozen=True)
class LateralProfile:
    """One lane change: ``distance`` metres over ``duration`` seconds, of a given shape."""

    start_s: float
    duration_s: float
    distance_m: float
    shape: LateralShape = QUINTIC_SHAPE

    def __post_init__(self):
        if not math.isfinite(self.duration_s) or self.duration_s <= 0:
            raise ValueError("positive finite duration required")
        if not math.isfinite(self.distance_m) or self.distance_m == 0:
            raise ValueError("non-zero finite lateral distance required")
        if not math.isfinite(self.start_s) or self.start_s < 0:
            raise ValueError("non-negative finite start time required")

    @property
    def end_s(self):
        return self.start_s + self.duration_s

    def sample(self, t):
        """Offset, transverse speed, acceleration and jerk at times ``t``."""
        t = np.asarray(t, float)
        u = (t - self.start_s) / self.duration_s
        s, d1, d2, d3 = self.shape.evaluate(u)
        inside = (t > self.start_s) & (t < self.end_s)
        scale = self.distance_m
        offset = np.where(t <= self.start_s, 0.0,
                          np.where(t >= self.end_s, scale, scale * s))
        speed = np.where(inside, scale * d1 / self.duration_s, 0.0)
        # Acceleration is right-continuous at the start: a C1 shape steps there.
        # For C2 shapes s''(0) = 0, so this changes nothing.
        accel = np.where((t >= self.start_s) & (t < self.end_s), scale * d2 / self.duration_s**2, 0.0)
        jerk = np.where(inside, scale * d3 / self.duration_s**3, 0.0)
        return offset, speed, accel, jerk

    def peaks(self):
        """Closed-form peak magnitudes; the sampled values must match these."""
        distance, duration = abs(self.distance_m), self.duration_s
        f = self.shape.peak_factors
        return {
            "speed_mps": f["speed"] * distance / duration,
            "accel_mps2": f["accel"] * distance / duration**2,
            "jerk_mps3": f["jerk"] * distance / duration**3,
        }


def minimum_duration(distance, envelope, shape=None):
    """Shortest duration of ``shape`` (default quintic) meeting each lateral bound.

    Each bound gives its own lower bound on ``T``; the binding one is the
    largest. Returned with the name of the binding bound so a refusal can say
    which capability ran out rather than only that the action was infeasible.
    """
    distance = abs(float(distance))
    f = (shape or QUINTIC_SHAPE).peak_factors
    bounds = {}
    if envelope.lateral_speed_max_mps:
        bounds["lateral_speed"] = f["speed"] * distance / envelope.lateral_speed_max_mps
    bounds["lateral_accel"] = math.sqrt(f["accel"] * distance / envelope.lateral_accel_max_mps2)
    if envelope.lateral_jerk_max_mps3:
        bounds["lateral_jerk"] = (f["jerk"] * distance / envelope.lateral_jerk_max_mps3) ** (1 / 3)
    if envelope.min_maneuver_s:
        bounds["declared_minimum"] = float(envelope.min_maneuver_s)
    binding = max(bounds, key=bounds.get)
    return {"duration_s": bounds[binding], "binding": binding, "bounds": bounds}


def envelope_utilisation(longitudinal_accel, normal_accel, envelope):
    """``eta`` for the coupled ellipse; admissible when ``eta <= 1``."""
    longitudinal_accel = np.asarray(longitudinal_accel, float)
    normal_accel = np.asarray(normal_accel, float)
    limit = np.where(longitudinal_accel >= 0,
                     envelope.longitudinal_accel_max_mps2,
                     envelope.longitudinal_decel_max_mps2)
    return (longitudinal_accel / limit) ** 2 + (normal_accel / envelope.lateral_accel_max_mps2) ** 2


# --- candidates, filtering, selection ---------------------------------------

@dataclass(frozen=True)
class Proposal:
    """A candidate action, before any check has been applied.

    A ``Proposal`` carries no verdict. It cannot be ranked: ``select`` accepts
    only an :class:`AdmissibleSet`, which only :func:`admissible` can build.
    """

    action: Action
    profile: LateralProfile | None = None
    label: str = ""

    def __post_init__(self):
        if (self.action is Action.CHANGE) != (self.profile is not None):
            raise ValueError("a lane change needs a profile; staying must not carry one")

    @property
    def start_s(self):
        return 0.0 if self.profile is None else self.profile.start_s


@dataclass(frozen=True)
class Verdict:
    """Why one proposal was admitted or rejected, with the measured evidence."""

    proposal: Proposal
    admitted: bool
    reasons: tuple[str, ...] = ()
    peak_utilisation: float = float("nan")
    minimum_separation_m: float = float("nan")
    cost: float = float("nan")


class AdmissibleSet:
    """Proposals that passed every admissibility check, and only those.

    Constructed exclusively by :func:`admissible`. The private token is not a
    security measure; it is a reminder at the call site that ranking a set that
    did not come from the filter is the defect this type exists to prevent.
    """

    _TOKEN = object()

    def __init__(self, token, verdicts):
        if token is not AdmissibleSet._TOKEN:
            raise TypeError("build an AdmissibleSet with admissible(), not directly")
        self._verdicts = tuple(verdicts)
        if any(not v.admitted for v in self._verdicts):
            raise ValueError("an AdmissibleSet cannot contain a rejected proposal")

    def __len__(self):
        return len(self._verdicts)

    def __iter__(self):
        return iter(self._verdicts)

    def __bool__(self):
        return bool(self._verdicts)


def admissible(proposals, check, *, envelope, volume, horizon_s):
    """Apply ``check`` to every proposal and return the survivors plus the log.

    ``check(proposal)`` returns :class:`AdmissionCheck`. Legacy four-field
    callbacks (ok, reasons, peak utilisation, minimum separation) remain supported.
    Both the admitted set and the full log are returned: rejected proposals are
    part of the record, not something to discard silently.
    """
    log = []
    for proposal in proposals:
        checked = check(proposal)
        if isinstance(checked, AdmissionCheck):
            ok, reasons = checked.admitted, checked.reasons
            utilisation, separation = checked.peak_utilisation, checked.minimum_separation_m
        else:
            # Small deterministic decision fixtures may supply the four-field
            # callback; production checkers return the named record below.
            ok, reasons, utilisation, separation = checked
        if proposal.profile is not None and proposal.profile.end_s > horizon_s + 1e-10:
            ok, reasons = False, (*reasons, "prediction_does_not_cover_maneuver")
        log.append(Verdict(proposal, bool(ok), tuple(reasons),
                           float(utilisation), float(separation)))
    return AdmissibleSet(AdmissibleSet._TOKEN, [v for v in log if v.admitted]), tuple(log)


def select(admitted, stay_cost, cost_of, *, now_s, delta_num, delta_pred):
    """Algorithm 1 lines 22-29: gain gate, then the action for *this* instant.

    ``admitted`` must be an :class:`AdmissibleSet`. A proposal whose start time
    has not arrived returns ``STAY`` with the reason ``future_start``: the
    instant's output is not a lane change, and no intent is retained for the
    next cycle to honour.
    """
    if not isinstance(admitted, AdmissibleSet):
        raise TypeError("select ranks an AdmissibleSet; filter before ranking")
    if not all(math.isfinite(x) and x >= 0 for x in (delta_num, delta_pred)):
        raise ValueError("finite non-negative error bands required")
    if not math.isfinite(stay_cost):
        raise ValueError("stay cost must be finite")
    threshold = float(delta_num) + float(delta_pred)
    if not admitted:
        return {"action": Action.STAY, "reason": "no_admissible_candidate",
                "chosen": None, "threshold": threshold, "best_gain": None}
    scored = [(cost_of(v.proposal), v) for v in admitted]
    if any(not math.isfinite(cost) for cost, _ in scored):
        raise ValueError("candidate costs must be finite")
    gaining = [(cost, v) for cost, v in scored if stay_cost - cost > threshold]
    if not gaining:
        return {"action": Action.STAY, "reason": "gain_below_threshold",
                "chosen": None, "threshold": threshold,
                "best_gain": max((stay_cost - c for c, _ in scored), default=float("nan"))}
    cost, best = min(gaining, key=lambda pair: pair[0])
    if best.proposal.start_s > now_s + 1e-12:
        return {"action": Action.STAY, "reason": "future_start",
                "chosen": best.proposal, "threshold": threshold,
                "starts_at_s": best.proposal.start_s, "committed": False,
                "best_gain": stay_cost - cost}
    return {"action": Action.CHANGE, "reason": "gain_above_threshold",
            "chosen": best.proposal, "threshold": threshold,
            "best_gain": stay_cost - cost}


# --- scenario and rollout ---------------------------------------------------

POLICY_COST = {"C": 0.0, "R": 1.0, "F": 2.0}
ROLES = ("B", "A", "C", "D", "E")
#: The aircraft that decides whether to change lane; the other four react.
EGO = "A"


@dataclass(frozen=True)
class WeakZone:
    """A stretch of one lane whose policy is prescribed rather than computed.

    Mode A of the protocol: the policy follows position, not an exposure
    history. Mode B replaces this map with a comms field and leaves everything
    else unchanged, which is why the map is a separate object.
    """

    lane: int
    entry_q_m: float
    length_m: float
    policy: str = "F"

    def policy_at(self, lane, q):
        inside = (lane == self.lane) & (q >= self.entry_q_m) & (q <= self.entry_q_m + self.length_m)
        return np.where(inside, self.policy, "C")


@dataclass(frozen=True)
class Scenario:
    """Initial conditions for the five-aircraft frame.

    Gaps are measured along the corridor from the aircraft ahead. Any neighbour
    may be omitted to form a diagnostic configuration; the remaining aircraft
    keep their values, and the roles actually present are recorded.
    """

    ego_q_m: float
    gap_to_leader_m: float
    gap_from_follower_m: float | None
    target_lane_front_offset_m: float | None
    target_lane_rear_offset_m: float | None
    lane_separation_m: float = 100.0
    altitude_m: float = 300.0
    cruise_mps: float = 50.0
    neighbour_speeds: dict = field(default_factory=dict)
    zones: tuple = ()
    leader_braking: dict | None = None
    # Following relationships already established at t = 0, as (follower,
    # leader). None keeps the original assumption that E is already following
    # L. A leader placed beyond the fallback spacing is not one E is following
    # under any policy, so such scenarios must declare () here; otherwise E
    # starts closing a gap nothing asked it to close.
    established_pairs: tuple | None = None

    def __post_init__(self):
        for name in ("ego_q_m", "altitude_m", "cruise_mps", "lane_separation_m", "gap_to_leader_m"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"finite {name} required")
        if min(self.lane_separation_m, self.cruise_mps, self.gap_to_leader_m) <= 0:
            raise ValueError("positive lane spacing, speed and leader gap required")
        for name in ("gap_from_follower_m", "target_lane_front_offset_m", "target_lane_rear_offset_m"):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"positive finite {name} required when enabled")
        if any(r not in ROLES or not math.isfinite(v) or v < 0
               for r, v in self.neighbour_speeds.items()):
            raise ValueError("invalid neighbour speed")
        if self.established_pairs is not None:
            enabled = set(self.enabled_roles())
            for pair in self.established_pairs:
                if len(pair) != 2 or pair[0] == pair[1] or not set(pair) <= enabled:
                    raise ValueError(f"established pair {pair!r} must name two enabled roles")

    def enabled_roles(self):
        roles = ["B", "A"]
        if self.gap_from_follower_m is not None:
            roles.append("C")
        if self.target_lane_front_offset_m is not None:
            roles.append("D")
        if self.target_lane_rear_offset_m is not None:
            roles.append("E")
        return tuple(roles)

    def policy_at(self, lane, q):
        policy = np.full(np.shape(q), "C", dtype="<U1")
        for zone in self.zones:
            policy = np.where(zone.policy_at(lane, q) != "C", zone.policy, policy)
        return policy


def _initial_states(scenario):
    """Along-corridor position, lane and speed for every enabled role."""
    speeds = {role: scenario.neighbour_speeds.get(role, scenario.cruise_mps)
              for role in ROLES}
    q = {"A": scenario.ego_q_m, "B": scenario.ego_q_m + scenario.gap_to_leader_m}
    lane = {"A": 0, "B": 0}
    if scenario.gap_from_follower_m is not None:
        q["C"] = scenario.ego_q_m - scenario.gap_from_follower_m
        lane["C"] = 0
    if scenario.target_lane_front_offset_m is not None:
        q["D"] = scenario.ego_q_m + scenario.target_lane_front_offset_m
        lane["D"] = 1
    if scenario.target_lane_rear_offset_m is not None:
        q["E"] = scenario.ego_q_m - scenario.target_lane_rear_offset_m
        lane["E"] = 1
    return q, lane, {role: speeds[role] for role in q}


@dataclass
class RigState:
    """Resumable state; forecasts never mutate the executing state."""

    t_s: float
    q: dict
    v: dict
    lane: dict
    policy: dict
    d_m: float = 0.0
    lateral_v_mps: float = 0.0
    lateral_a_mps2: float = 0.0
    profile: LateralProfile | None = None
    recovery_pairs: set = field(default_factory=set)
    leaders: dict = field(default_factory=dict)
    plans: dict = field(default_factory=dict)
    plan_keys: dict = field(default_factory=dict)
    history: dict = field(default_factory=dict)
    observation_rows: list = field(default_factory=list)
    next_observation_s: float = 0.0
    events: list = field(default_factory=list)
    # Held longitudinal commands. A real flight controller updates at a fixed
    # rate; when cfg.control_period_s is set, commands are recomputed only at
    # those instants and held in between, so the integration step can be
    # refined without changing the controller itself.
    commands: dict = field(default_factory=dict)
    modes: dict = field(default_factory=dict)
    next_control_s: float = 0.0


def initial_state(scenario):
    q, lane, v = _initial_states(scenario)
    pairs = ({("A", "B")} if scenario.established_pairs is None
             else {tuple(pair) for pair in scenario.established_pairs})
    return RigState(0.0, q, v, lane, {r: "C" for r in q},
                    recovery_pairs=pairs, history={r: [] for r in q})


def _occupied(state, role):
    p = state.profile
    if role == "A" and p is not None and p.start_s <= state.t_s < p.end_s:
        return {0, 1}
    return {state.lane[role]}


def _positions(state, scenario):
    return np.array([[state.q[r], state.d_m if r == "A" else
                      state.lane[r] * scenario.lane_separation_m, scenario.altitude_m]
                     for r in state.q], float)


def _observe(state, scenario, cfg, radio):
    if state.t_s < state.next_observation_s - 1e-9:
        return
    roles = tuple(state.q)
    if radio is None:
        for r in roles:
            # Spatial prescribed-policy mode. The policy lane is the lane the
            # aircraft is counted in, which switches at the midline (see
            # rollout); occupancy stays two-lane for the whole crossing and is
            # what separation and following relations use. Radio mode needs no
            # equivalent rule: there the signal follows the actual position.
            state.policy[r] = str(scenario.policy_at(state.lane[r], np.array(state.q[r]))[()])
    else:
        from .policy_motion import policy_from_exposure
        values = radio(_positions(state, scenario))
        signal = np.asarray(values["sinr_db"], float)
        if signal.shape != (len(roles),) or not np.isfinite(signal).all():
            raise ValueError("radio must return one finite SINR per aircraft")
        signals = dict(zip(roles, signal))
        for r in roles:
            ordered = sorted((x for x in roles if state.lane[x] == state.lane[r]),
                             key=lambda x: (state.q[x], x))
            index = ordered.index(r)
            n = cfg.neighbors_each_side
            members = tuple(ordered[max(0, index-n):index+n+1])
            radius = getattr(cfg, "group_radius_m", None)
            if radius is not None:
                members = tuple(x for x in members if abs(state.q[x]-state.q[r]) <= radius+1e-9)
            fraction = float(np.mean([signals[x] < cfg.threshold_db for x in members]))
            history = [row for row in state.history[r]
                       if row[0] >= state.t_s - cfg.window_s - 1e-9]
            history.append((state.t_s, fraction, members))
            state.history[r] = history
            exposure = float(np.mean([row[1] for row in history]))
            state.policy[r] = policy_from_exposure(exposure, cfg.policy_config())
            state.observation_rows.append({"t_s": state.t_s, "role": r,
                "sinr_db": signals[r], "members": members, "exposure": exposure,
                "policy": state.policy[r], "position": _positions(state, scenario)[roles.index(r)].tolist()})
    state.next_observation_s += cfg.policy_s


def _sync_relations(state, cfg):
    leaders = {}
    for r in state.q:
        ids = []
        for lane in sorted(_occupied(state, r)):
            ahead = [x for x in state.q if x != r and lane in _occupied(state, x)
                     and state.q[x] > state.q[r] + 1e-9]
            if ahead:
                lead = min(ahead, key=lambda x: state.q[x])
                if lead not in ids:
                    ids.append(lead)
        leaders[r] = tuple(ids)
    current = {(r, lead) for r, ids in leaders.items() for lead in ids}
    before = set(state.recovery_pairs)
    state.recovery_pairs.intersection_update(current)
    for r, lead in current:
        if state.q[lead] - state.q[r] <= cfg.spacing(state.policy[r], state.v[r]) + 1e-8:
            state.recovery_pairs.add((r, lead))
    if leaders != state.leaders or before != state.recovery_pairs:
        state.events.append({"t_s": state.t_s, "event": "relationships_updated",
                             "leaders": deepcopy(leaders),
                             "recovery_pairs": sorted(state.recovery_pairs)})
    state.leaders = leaders


def _pair_config(cfg):
    from .two_uam_longitudinal import resolve
    return resolve({"vehicle": {"cruise_mps": cfg.cruise_mps,
        "speed_min_mps": cfg.speed_min_mps, "speed_max_mps": cfg.speed_ceiling_mps,
        "acceleration_limit_mps2": cfg.acceleration_limit_mps2,
        "deceleration_limit_mps2": cfg.deceleration_limit_mps2},
        "spacing": {k: getattr(cfg, k) for k in
                    ("d0_m", "tau_c_s", "tau_r_s", "tau_f_s", "buffer_s2_per_m")},
        "controller": {"acc_policy_recovery": True,
            "k_speed_per_s": cfg.k_speed_per_s, "k_gap_per_s2": cfg.k_gap_per_s2,
            "k_relative_per_s": cfg.k_relative_per_s,
            "tracking_speed_gain_per_s": cfg.k_relative_per_s}})


def _commands(state, scenario, cfg, control_mode, pair_cfg):
    from .geographic_traffic import TrafficAircraft, TrafficState, acc_controls
    from .two_uam_longitudinal import StraightArc, plan_request
    from .aks import tracking_command
    aircraft = [TrafficAircraft(r, state.q[r], state.v[r], state.lane[r],
                  policy=state.policy[r], target_lane=1 if len(_occupied(state, r)) == 2 else None)
                for r in state.q]
    feedback = acc_controls(TrafficState(state.t_s, aircraft, []), StraightArc(), cfg,
                            recovery_pairs=frozenset(state.recovery_pairs))
    commands, modes = {}, {}
    changing = state.profile is not None and state.profile.start_s <= state.t_s < state.profile.end_s
    for r in state.q:
        a = feedback[r]["command"]
        modes[r] = "feedback" if control_mode == "feedback" else "acc_fallback"
        if r == "A" and changing:
            # An aircraft that is leaving the lane does not brake for the
            # leader it is leaving. It holds the speed it had when the change
            # began; a candidate whose prediction becomes unsafe under that
            # rule is rejected before it starts, and the committed maneuver is
            # rechecked at every decision.
            commands[r], modes[r] = 0.0, "lane_change_hold"
            continue
        ids = state.leaders[r]
        key = (ids, state.policy[r], tuple(x for x in ids if (r, x) in state.recovery_pairs))
        if control_mode == "aks":
            if key != state.plan_keys.get(r):
                state.plan_keys[r] = key
                state.plans.pop(r, None)
                if len(ids) == 1 and (r, ids[0]) in state.recovery_pairs:
                    lead = ids[0]
                    request = plan_request(pair_cfg, state.q[lead]-state.q[r], state.policy[r],
                        "exact_target", follower_speed=state.v[r], leader_speed=state.v[lead])
                    if request["status"] in ("planned", "hold"):
                        state.plans[r] = {"request": request, "t_s": state.t_s,
                            "q_m": state.q[r], "lead_q_m": state.q[lead], "lead": lead}
                    state.events.append({"t_s": state.t_s, "event": "longitudinal_plan",
                                         "role": r, "status": request["status"]})
            plan = state.plans.get(r)
            if plan is not None:
                elapsed = state.t_s - plan["t_s"]
                lead = plan["lead"]
                if abs(state.v[lead]-cfg.cruise_mps) > 1e-6 or abs(
                    state.q[lead] - plan["lead_q_m"] - cfg.cruise_mps*elapsed) > 1e-5:
                    state.plans.pop(r)
                    state.events.append({"t_s": state.t_s, "event": "reference_invalidated",
                                         "role": r, "reason": "leader_changed"})
                else:
                    transition = plan["request"]["transition"]
                    if transition is None:
                        vr, ar, xr = cfg.cruise_mps, 0., cfg.cruise_mps*elapsed
                    elif elapsed >= transition.duration:
                        vr, ar = cfg.cruise_mps, 0.
                        xr = transition.displacement() + vr*(elapsed-transition.duration)
                    else:
                        vr, ar, xr = (float(x) for x in transition.sample(elapsed))
                    gap_ref = plan["lead_q_m"] + cfg.cruise_mps*elapsed - plan["q_m"] - xr
                    a = float(tracking_command(ar, state.q[lead]-state.q[r], gap_ref,
                        vr, state.v[r], cfg.k_gap_per_s2, cfg.k_relative_per_s))
                    modes[r] = "ks2_tracker" if transition else "hold_tracker"
            elif not ids:
                modes[r] = "cruise"
        commands[r] = float(np.clip(a, -cfg.deceleration_limit_mps2, cfg.acceleration_limit_mps2))
    return commands, modes


def _lateral_polynomial(state, h):
    """Integrate the quintic feedforward jerk from the *actual* d, v, a.

    Polynomials use u in [0,1] for numerical conditioning. Corrections to the
    constant/linear/quadratic terms retain actual initial conditions; this is
    integration of prescribed jerk, not assigning position to a reference.
    """
    p = state.profile
    if p is None or state.t_s < p.start_s or state.t_s >= p.end_s - 1e-10:
        return Polynomial([state.d_m, h*state.lateral_v_mps, h*h*state.lateral_a_mps2/2])
    u0 = (state.t_s-p.start_s)/p.duration_s
    u = Polynomial([u0, h/p.duration_s])
    # Steps are cut at every shape junction (see _next_width), so one piece
    # governs the whole step.
    ref = p.distance_m * p.shape.polynomial(p.shape.piece_index(u0))(u)
    return ref + Polynomial([state.d_m-ref(0),
        h*state.lateral_v_mps-ref.deriv()(0),
        (h*h*state.lateral_a_mps2-ref.deriv(2)(0))/2])


def _extrema(poly):
    points = [0., 1.]
    for root in poly.deriv().roots():
        if abs(root.imag) < 1e-8 and 0 < root.real < 1:
            points.append(float(root.real))
    values = np.array([poly(u) for u in points], float)
    return float(values.min()), float(values.max())


def _next_width(state, scenario, cfg, end, dt):
    boundaries = [end, state.t_s+dt, state.next_observation_s]
    if getattr(cfg, "control_period_s", None):
        boundaries.append(state.next_control_s)
    p = state.profile
    if p:
        boundaries.extend([p.start_s, p.end_s])
        # The lane (and with it the policy) switches at the midline, so that
        # instant has to be a step boundary like any other discontinuity.
        boundaries.append(p.start_s + p.shape.midpoint_u*p.duration_s)
        boundaries.extend(p.start_s + b*p.duration_s for b in p.shape.junctions)
    if scenario.leader_braking:
        b = scenario.leader_braking
        boundaries.extend([b["start_s"], b["start_s"] +
                           (scenario.cruise_mps-b["end_speed_mps"])/b["deceleration_mps2"]])
    for plan in state.plans.values():
        tr = plan["request"]["transition"]
        if tr:
            boundaries.extend([plan["t_s"]+tr.alpha*tr.duration, plan["t_s"]+tr.duration])
    # The boundary time itself, not a width: the caller advances to it exactly.
    # Boundaries within a nanosecond of each other are the same instant, and the
    # later one is the exact one: a step grid carried from an earlier off-grid
    # split would otherwise land just short of an observation time, fire the
    # observation early, and leave the clock permanently offset.
    ordered = sorted(x for x in boundaries if x > state.t_s+1e-10)
    target = ordered[0]
    for x in ordered[1:]:
        if x-target > 1e-9:
            break
        target = x
    return target


def rollout(scenario, cfg, profile, *, horizon_s, dt, ego_policy_lane=None,
            state=None, envelope=None, control_mode="aks", radio=None):
    """Predict to an absolute endpoint, or execute the same integrator's prefix.

    Radio is an optional pure callable on the actual straight-frame XYZ states,
    returning ``sinr_db``; histories and logical group membership are stateful.
    Without radio this is explicitly prescribed spatial-policy mode, not a
    wireless-performance result. No state or history passed in is mutated.
    """
    from .two_uam_longitudinal import leader_state
    if ego_policy_lane is not None:
        raise ValueError("policy lane override removed: use actual positions")
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(horizon_s):
        raise ValueError("finite positive step and finite endpoint required")
    if control_mode not in ("aks", "feedback"):
        raise ValueError("unknown longitudinal controller")
    if cfg.policy_s <= 0:
        raise ValueError("positive observation period required")
    envelope = envelope or AccelerationEnvelope()
    st = deepcopy(state) if state is not None else initial_state(scenario)
    if horizon_s <= st.t_s:
        raise ValueError("endpoint must follow current state")
    if profile is not None:
        if not math.isclose(profile.distance_m, scenario.lane_separation_m, abs_tol=1e-9):
            raise ValueError("profile must end at the target lane centre")
        if st.profile is not None and st.profile != profile:
            raise ValueError("cannot replace a started profile without a continuous replan")
        if st.profile is None and profile.start_s < st.t_s-1e-9:
            raise ValueError("new profile starts before actual state")
        st.profile = profile
    profile = st.profile
    if any(not cfg.speed_min_mps <= v <= cfg.speed_ceiling_mps for v in st.v.values()):
        raise ValueError("initial speed outside declared bounds")
    if scenario.leader_braking:
        b = scenario.leader_braking
        if not (0 <= b["start_s"] and cfg.speed_min_mps <= b["end_speed_mps"] <= scenario.cruise_mps
                and 0 < b["deceleration_mps2"] <= min(cfg.deceleration_limit_mps2,
                                                                  envelope.longitudinal_decel_max_mps2)):
            raise ValueError("prescribed leader violates declared bounds")
    pair_cfg = _pair_config(cfg)
    roles = tuple(st.q)
    times, segments = [], []
    keys = ("q_m", "d_m", "v_mps", "a_mps2", "policy", "lane", "controller", "leaders")
    trace = {r: {k: [] for k in keys} for r in roles}
    lateral = {k: [] for k in ("offset_m", "speed_mps", "accel_mps2", "jerk_mps3")}
    lead_origin = scenario.ego_q_m + scenario.gap_to_leader_m
    while True:
        if scenario.leader_braking:
            lx, lv, _ = leader_state(st.t_s, scenario.cruise_mps, scenario.leader_braking)
            st.q["B"], st.v["B"] = lead_origin+lx, lv
        if profile and st.t_s >= profile.start_s + profile.shape.midpoint_u*profile.duration_s - 1e-10:
            st.lane["A"] = 1
        _observe(st, scenario, cfg, radio)
        _sync_relations(st, cfg)
        if not cfg.control_period_s:
            commands, modes = _commands(st, scenario, cfg, control_mode, pair_cfg)
        else:
            if not st.commands or st.t_s >= st.next_control_s - 1e-9:
                st.commands, st.modes = _commands(st, scenario, cfg, control_mode, pair_cfg)
                while st.next_control_s <= st.t_s + 1e-9:
                    st.next_control_s += cfg.control_period_s
            commands, modes = dict(st.commands), dict(st.modes)
        remaining = horizon_s - st.t_s
        target = _next_width(st, scenario, cfg, horizon_s, dt) if remaining > 1e-9 else st.t_s+dt
        h, split = target-st.t_s, False
        # A C1 shape (the cubic Bezier) prescribes a step in transverse acceleration
        # at the manoeuvre's start and end. Both instants are forced step
        # boundaries, so the step is applied here as a declared event. C2 shapes
        # have zero one-sided acceleration at both ends and are left untouched.
        if profile is not None:
            for instant, u_side in ((profile.start_s, 0.), (profile.end_s, None)):
                if abs(st.t_s-instant) <= 1e-9:
                    a_side = 0. if u_side is None else (
                        profile.distance_m*float(profile.shape.evaluate(np.array([u_side]))[2][0])
                        / profile.duration_s**2)
                    if abs(a_side-st.lateral_a_mps2) > 1e-9:
                        st.events.append({"t_s": st.t_s, "event": "lateral_acceleration_step",
                                          "from_mps2": st.lateral_a_mps2, "to_mps2": a_side})
                        st.lateral_a_mps2 = a_side
        # Reserve the normal acceleration demanded throughout this interval.
        # Saturation changes the actual predicted path and is audited below.
        dpoly = _lateral_polynomial(st, h)
        lat_a = max(abs(x) for x in _extrema(dpoly.deriv(2)/h**2))
        for r in roles:
            reserve = math.sqrt(max(0., 1-(lat_a/envelope.lateral_accel_max_mps2)**2)) if r == "A" else 1.
            commands[r] = float(np.clip(commands[r],
                -min(cfg.deceleration_limit_mps2, envelope.longitudinal_decel_max_mps2)*reserve,
                min(cfg.acceleration_limit_mps2, envelope.longitudinal_accel_max_mps2)*reserve))
            if (st.v[r] <= cfg.speed_min_mps+1e-10 and commands[r] < 0 or
                    st.v[r] >= cfg.speed_ceiling_mps-1e-10 and commands[r] > 0):
                commands[r] = 0.
        if scenario.leader_braking:
            commands["B"] = leader_state(st.t_s, scenario.cruise_mps, scenario.leader_braking)[2]
            modes["B"] = "prescribed_leader"
        # Split at the first speed contact, so both position and velocity use
        # exactly the same bounded piecewise-constant acceleration path.
        for r, a in commands.items():
            if a:
                bound = cfg.speed_ceiling_mps if a > 0 else cfg.speed_min_mps
                contact = (bound-st.v[r])/a
                if 1e-10 < contact < h:
                    h, split = contact, True
        dpoly = _lateral_polynomial(st, h)
        times.append(st.t_s)
        for r in roles:
            vals = (st.q[r], st.d_m if r == "A" else 0., st.v[r], commands[r],
                    st.policy[r], st.lane[r], modes[r], st.leaders[r])
            for k, value in zip(keys, vals):
                trace[r][k].append(value)
        jerk = float(dpoly.deriv(3)(0)/h**3)
        for k, value in zip(lateral, (st.d_m, st.lateral_v_mps, st.lateral_a_mps2, jerk)):
            lateral[k].append(value)
        if remaining <= 1e-9:
            break
        xyz = {}
        for r in roles:
            xp = Polynomial([st.q[r], st.v[r]*h, commands[r]*h*h/2])
            yp = dpoly if r == "A" else Polynomial([st.lane[r]*scenario.lane_separation_m])
            xyz[r] = (xp.coef.tolist(), yp.coef.tolist())
        segments.append({"start_s": st.t_s, "end_s": st.t_s+h, "xy": xyz,
                         "policy": dict(st.policy), "lane": dict(st.lane),
                         "occupied": {r: tuple(_occupied(st, r)) for r in roles}})
        for r in roles:
            st.q[r] += st.v[r]*h + .5*commands[r]*h*h
            st.v[r] = float(np.clip(st.v[r]+commands[r]*h, cfg.speed_min_mps, cfg.speed_ceiling_mps))
        st.d_m, st.lateral_v_mps, st.lateral_a_mps2 = (float(dpoly(1)),
                                                float(dpoly.deriv()(1)/h), float(dpoly.deriv(2)(1)/h**2))
        # Advance to the boundary time itself. Accumulating widths drifts by a
        # rounding unit per step, so a run computed in one piece stops matching
        # the same run resumed from a saved state. Only a speed-contact split,
        # which lands between boundaries, advances by the width.
        st.t_s = min(horizon_s, st.t_s+h if split else target)
        if abs(horizon_s-st.t_s) <= 1e-9:
            st.t_s = horizon_s
    out = {}
    for r in roles:
        out[r] = {}
        for k, val in trace[r].items():
            if k == "leaders":
                out[r][k] = val
            else:
                out[r][k] = np.asarray(val, dtype=object if k in ("policy", "controller") else float)
    return {"times_s": np.array(times), "roles": roles, "trace": out,
            "lateral": {k: np.array(v) for k, v in lateral.items()}, "segments": segments,
            "profile": profile, "scenario": scenario, "cfg": cfg, "state": st,
            "events": st.events, "observations": st.observation_rows,
            "policy_mode": "radio_exposure" if radio is not None else "prescribed_spatial_policy",
            "controller_mode": control_mode,
            "lateral_execution": f"integrated_{(profile.shape.name if profile else 'quintic')}_jerk"}


def cartesian(result, scenario):
    roles = result["roles"]
    pos = np.zeros((len(result["times_s"]), len(roles), 3))
    for i, r in enumerate(roles):
        pos[:, i, 0] = result["trace"][r]["q_m"]
        pos[:, i, 1] = result["trace"][r]["d_m"] + (0 if r in ("A", "B", "C") else scenario.lane_separation_m)
        pos[:, i, 2] = scenario.altitude_m
    return pos


def policy_cost(result):
    """Integrate observed piecewise-constant policy, not trapezoids across jumps."""
    t = result["times_s"]
    per_role = {}
    for r in result["roles"]:
        weights = np.array([POLICY_COST[p] for p in result["trace"][r]["policy"]])
        per_role[r] = float(np.dot(weights[:-1], np.diff(t)))
    return {"total": sum(per_role.values()), "per_role": per_role}

class AdmissionCheck(NamedTuple):
    admitted: bool
    reasons: tuple[str, ...]
    peak_utilisation: float
    minimum_separation_m: float
    minimum_margin: float
    screen: dict


def _nonnegative(poly, *, strict=False, depth=16):
    """Conservative numerical Bernstein enclosure on [0,1].

    PASS requires a positive lower enclosure (or the declared 1e-9 numerical
    tolerance for non-strict capability checks). Boundary intervals unresolved
    at the subdivision limit are UNKNOWN, never silently accepted. This is a
    floating-point interval audit, not a formal arithmetic certificate.
    """
    c = poly.trim().coef
    if not np.isfinite(c).all():
        return "UNKNOWN"
    n = len(c)-1
    b = np.array([sum(c[i]*math.comb(k, i)/math.comb(n, i) for i in range(k+1))
                  for k in range(n+1)])
    error = 64*np.finfo(float).eps*max(1., float(np.abs(c).sum()))
    tolerance = 0. if strict else 1e-9
    if b.min()-error >= -tolerance and (not strict or b.min()-error > 0):
        return "PASS"
    sampled = np.array([poly(0), poly(.5), poly(1)])
    if sampled.min() < -tolerance-error or (strict and sampled.min() == 0):
        return "FAIL"
    if depth == 0:
        return "UNKNOWN"
    left = _nonnegative(poly(Polynomial([0., .5])), strict=strict, depth=depth-1)
    if left == "FAIL":
        return left
    right = _nonnegative(poly(Polynomial([.5, .5])), strict=strict, depth=depth-1)
    if right == "FAIL":
        return right
    return "PASS" if left == right == "PASS" else "UNKNOWN"


def _initial_pairs(scenario):
    q, lane, _ = _initial_states(scenario)
    pairs = set()
    for l in (0, 1):
        order = sorted((r for r in q if lane[r] == l), key=q.get)
        pairs.update(zip(order[:-1], order[1:]))
    return pairs


def _segment_leaders(segment, roles):
    pairs = set()
    for lane in (0, 1):
        order = sorted((r for r in roles if lane in segment["occupied"][r]),
                       key=lambda r: segment["xy"][r][0][0])
        pairs.update(zip(order[:-1], order[1:]))
    return pairs


def admissibility_check(result, scenario, envelope, volume, *, cfg=None,
                        require_complete=True):
    """Audit executed segment polynomials and report NMAC separately.

    ``volume`` must explicitly identify the research admission geometry. The
    historical NMAC default cannot silently become that policy. Its geometric
    distance is a relative-centre threshold, not an aircraft body radius.
    """
    if volume is None or volume.label == "nmac_event":
        raise ValueError("explicit admission volume required; NMAC is reported separately")
    if volume.tau_mod_s or volume.lookahead_s:
        raise ValueError("use a geometric admission volume; predictive screens are separate")
    cfg = cfg or result.get("cfg")
    if cfg is None:
        raise ValueError("resolved motion and spacing configuration required")
    t = np.asarray(result["times_s"], float)
    if len(t) < 2 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError("ordered finite trajectory times required")
    reasons = []
    positions = cartesian(result, scenario)
    if not np.isfinite(positions).all() or any(
        not np.isfinite(np.asarray(result["trace"][r][key], float)).all()
        for r in result["roles"] for key in ("v_mps", "a_mps2")) or any(
        not np.isfinite(np.asarray(values, float)).all() for values in result["lateral"].values()):
        return AdmissionCheck(False, ("non_finite_state",), math.inf, math.nan, math.nan, {})
    velocity = np.zeros_like(positions)
    for i, r in enumerate(result["roles"]):
        velocity[:, i, 0] = result["trace"][r]["v_mps"]
        if r == "A":
            velocity[:, i, 1] = result["lateral"]["speed_mps"]
    screen = screen_trajectories(t, positions, volume, velocities=velocity,
                                identifiers=list(result["roles"]))
    nmac = screen_trajectories(t, positions, SeparationVolume(), velocities=velocity,
                              identifiers=list(result["roles"]))
    screen["nmac_diagnostic"] = nmac
    screen["audit_scope"] = "floating_point_polynomial_interval_checks"
    if screen["violation_count"]:
        reasons.append("admission_separation_violation")
    closest = screen["closest_pair"]
    separation = float(closest["horizontal_m"]) if closest else math.inf
    margin = separation/volume.horizontal_m
    peak = 0.
    profile = result.get("profile")
    if profile is not None:
        if envelope.min_maneuver_s and profile.duration_s < envelope.min_maneuver_s:
            reasons.append("declared_minimum_duration_not_met")
        if not math.isclose(profile.distance_m, scenario.lane_separation_m, abs_tol=1e-9):
            reasons.append("profile_misses_target_lane")
        if require_complete and t[-1] < profile.end_s + cfg.decision_s - 1e-9:
            reasons.append("prediction_does_not_cover_maneuver_and_post_state")
        if t[-1] >= profile.end_s:
            for name, expected in (("offset_m", profile.distance_m), ("speed_mps", 0.), ("accel_mps2", 0.)):
                if abs(result["lateral"][name][-1]-expected) > 1e-7:
                    reasons.append(f"lateral_endpoint_{name}")
    # Sample checks also protect callers that provide an independent trajectory.
    for r in result["roles"]:
        a = np.asarray(result["trace"][r]["a_mps2"])
        normal = np.asarray(result["lateral"]["accel_mps2"]) if r == "A" else np.zeros_like(a)
        peak = max(peak, float(np.max(envelope_utilisation(a, normal, envelope))))
        vd = result["lateral"]["speed_mps"] if r == "A" else np.zeros_like(a)
        speed = np.hypot(result["trace"][r]["v_mps"], vd)
        if speed.max() > cfg.speed_ceiling_mps+1e-9 or speed.min() < cfg.speed_min_mps-1e-9:
            reasons.append(f"{r}_total_speed_exceeded")
        if a.max() > cfg.acceleration_limit_mps2+1e-9 or a.min() < -cfg.deceleration_limit_mps2-1e-9:
            reasons.append(f"{r}_longitudinal_acceleration_exceeded")
    if peak > 1+1e-9:
        reasons.append("joint_envelope_exceeded")
    for name, limit, key in (("speed", envelope.lateral_speed_max_mps, "speed_mps"),
                            ("jerk", envelope.lateral_jerk_max_mps3, "jerk_mps3")):
        if limit is not None and np.max(np.abs(result["lateral"][key])) > limit+1e-9:
            reasons.append(f"lateral_{name}_exceeded")
    segments = result.get("segments", [])
    if not segments:
        reasons.append("continuous_trajectory_unavailable")
    elif (abs(segments[0]["start_s"]-t[0]) > 1e-9 or
          abs(segments[-1]["end_s"]-t[-1]) > 1e-9 or
          any(abs(a["end_s"]-b["start_s"]) > 1e-9 for a, b in zip(segments, segments[1:]))):
        reasons.append("continuous_trajectory_incomplete")
    if len(segments) != len(t)-1:
        reasons.append("continuous_trajectory_sample_mismatch")
    baseline_pairs = _initial_pairs(scenario)
    deficits = []
    interval_events = []
    nmac_intervals = []
    def require(poly, tag, strict=False):
        status = _nonnegative(poly, strict=strict)
        if status != "PASS":
            reasons.append(tag + (":unresolved" if status == "UNKNOWN" else ""))
        return status
    # The lateral polynomial within one step has the degree of the manoeuvre's
    # shape piece. The Bernstein enclosure used for every check below is
    # degree-agnostic and reports UNKNOWN, which require() rejects, whenever it
    # cannot resolve an interval; widening this guard to the shape's own degree
    # therefore cannot admit an unaudited trajectory. Five stays the floor, so
    # quintic runs are unchanged.
    lateral_cap = 5
    if profile is not None:
        lateral_cap = max(5, max(len(c)-1 for c in profile.shape.coefficients))
    for index, seg in enumerate(segments):
        h = seg["end_s"]-seg["start_s"]
        if not math.isfinite(h) or h <= 0:
            reasons.append("invalid_trajectory_interval")
            continue
        xy = {r: tuple(Polynomial(c) for c in seg["xy"][r]) for r in result["roles"]}
        if any(not np.isfinite(p.coef).all() for pair in xy.values() for p in pair):
            reasons.append("non_finite_trajectory_coefficients")
            continue
        # The polynomials being certified must be the same trajectory whose
        # sampled states, cost and terminal conditions are being reported.
        if index+1 < len(t):
            if abs(seg["start_s"]-t[index]) > 1e-9 or abs(seg["end_s"]-t[index+1]) > 1e-9:
                reasons.append("continuous_trajectory_sample_mismatch")
            for role_index, r in enumerate(result["roles"]):
                x, y = xy[r]
                if x.degree() > 2 or y.degree() > lateral_cap:
                    reasons.append("unsupported_trajectory_degree")
                for u, sample_index in ((0., index), (1., index+1)):
                    predicted = np.array([x(u), y(u), x.deriv()(u)/h, y.deriv()(u)/h])
                    sampled = np.r_[positions[sample_index, role_index, :2],
                                    velocity[sample_index, role_index, :2]]
                    if np.max(np.abs(predicted-sampled)) > 1e-7:
                        reasons.append("continuous_trajectory_sample_mismatch")
        for r, (x, y) in xy.items():
            vx, vy = x.deriv()/h, y.deriv()/h
            ax, ay = x.deriv(2)/h**2, y.deriv(2)/h**2
            require(cfg.speed_ceiling_mps**2-vx**2-vy**2, f"{r}_total_speed_exceeded")
            require(vx**2+vy**2-cfg.speed_min_mps**2, f"{r}_total_speed_below_floor")
            lo, hi = _extrema(ax)
            require(cfg.acceleration_limit_mps2-ax, f"{r}_longitudinal_acceleration_exceeded")
            require(ax+cfg.deceleration_limit_mps2, f"{r}_longitudinal_acceleration_exceeded")
            # Executed longitudinal segments have constant acceleration.
            limit = envelope.longitudinal_accel_max_mps2 if hi >= 0 else envelope.longitudinal_decel_max_mps2
            eta = (ax/limit)**2+(ay/envelope.lateral_accel_max_mps2)**2
            peak = max(peak, _extrema(eta)[1])
            require(1-eta, "joint_envelope_exceeded")
            if r == "A":
                if envelope.lateral_speed_max_mps is not None:
                    require(envelope.lateral_speed_max_mps**2-vy**2, "lateral_speed_exceeded")
                if envelope.lateral_jerk_max_mps3 is not None:
                    require(envelope.lateral_jerk_max_mps3**2-(y.deriv(3)/h**3)**2,
                            "lateral_jerk_exceeded")
        for first, second in combinations(result["roles"], 2):
            dx, dy = xy[first][0]-xy[second][0], xy[first][1]-xy[second][1]
            distance2 = dx**2+dy**2
            min2 = max(0., _extrema(distance2)[0])
            separation = min(separation, math.sqrt(min2))
            margin = min(margin, math.sqrt(min2)/volume.horizontal_m)
            status = require(distance2-volume.horizontal_m**2, "admission_separation_violation", strict=True)
            if status != "PASS":
                interval_events.append({"pair": (first, second), "start_s": seg["start_s"],
                                        "end_s": seg["end_s"], "status": status})
            ns = _nonnegative(distance2-SeparationVolume().horizontal_m**2, strict=True)
            if ns != "PASS":
                nmac_intervals.append({"pair": (first, second), "start_s": seg["start_s"],
                                      "end_s": seg["end_s"], "status": ns})
        for follower, leader in _segment_leaders(seg, result["roles"]):
            ownv = xy[follower][0].deriv()/h
            tau = {"C": cfg.tau_c_s, "R": cfg.tau_r_s, "F": cfg.tau_f_s}[seg["policy"][follower]]
            gap = xy[leader][0]-xy[follower][0]
            excess = gap-cfg.d0_m-tau*ownv-cfg.buffer_s2_per_m*ownv**2
            shortfall = max(0., -_extrema(excess)[0])
            if shortfall > 1e-8:
                deficits.append({"pair": (follower, leader), "start_s": seg["start_s"],
                                 "end_s": seg["end_s"], "maximum_deficit_m": shortfall,
                                 "new_relationship": (follower, leader) not in baseline_pairs})
            # Only a relationship the maneuvering aircraft itself enters can
            # refuse the maneuver, because it is the one choosing to create it.
            # The pair left behind on departure -- the old follower inheriting
            # the old leader -- is not created by the maneuver and is strictly
            # better off: its new gap is its old gap plus the departing
            # aircraft's own gap. Refusing on it would forbid a change that
            # reduces the very shortfall being measured. It is still recorded
            # as a deficit above and still flies the normal following law,
            # cruising when the gap suffices and closing on the reference when
            # it does not.
            if (profile is not None and (follower, leader) not in baseline_pairs
                    and EGO in (follower, leader)):
                require(excess, f"insertion_gap_insufficient:{follower}:{leader}")
    # Independent endpoint check remains useful even for externally supplied
    # traces lacking segment coefficients; such traces cannot pass continuously.
    if profile and t[-1] >= profile.end_s:
        q = {r: float(result["trace"][r]["q_m"][-1]) for r in result["roles"]}
        lanes = {r: (1 if r in ("A", "D", "E") else 0) for r in result["roles"]}
        for lane in (0, 1):
            order = sorted((r for r in q if lanes[r] == lane), key=q.get)
            for follower, leader in zip(order[:-1], order[1:]):
                # Same rule as the interval check above: the left-behind pair is
                # reported, not refused.
                if (follower, leader) in baseline_pairs or EGO not in (follower, leader):
                    continue
                vf = float(result["trace"][follower]["v_mps"][-1])
                vl = float(result["trace"][leader]["v_mps"][-1])
                policy = str(result["trace"][follower]["policy"][-1])
                gap = q[leader]-q[follower]
                if gap < cfg.spacing(policy, vf)-1e-8:
                    reasons.append(f"terminal_insertion_gap:{follower}:{leader}")
                # Conservative equal-capability braking-to-floor check under
                # simultaneous response, supplementary to post-action rollout.
                loss = max(0., ((vf-cfg.speed_min_mps)**2-(vl-cfg.speed_min_mps)**2)
                           /(2*min(cfg.deceleration_limit_mps2, envelope.longitudinal_decel_max_mps2)))
                if gap <= volume.horizontal_m+loss:
                    reasons.append(f"terminal_braking_margin:{follower}:{leader}")
    screen["interval_events"] = interval_events
    screen["nmac_diagnostic"]["interval_events"] = nmac_intervals
    screen["policy_deficits"] = deficits
    reasons = tuple(dict.fromkeys(reasons))
    return AdmissionCheck(not reasons, reasons, peak, separation, margin, screen)


def run_closed_loop(scenario, cfg, envelope, volume, *, horizon_s, prediction_s,
                    dt, duration_factors=(1., 1.5, 2.), start_delays=(0.,),
                    delta_num, delta_pred, radio=None, allow_lane_change=True, shapes=None,
                    decision_phase_s=0.):
    """One-lane-change experiment with rolling candidate selection and commit.

    The only committed lateral state is a maneuver that has actually begun.
    Future starts are replanned. A candidate is admitted only if its whole
    prediction passes the audit, but execution is never halted: if the stay
    baseline, a committed continuation or an executed step fails the audit, the
    violation is recorded in ``safety_events`` and the flight continues under
    the same rules. Ending the run there would hide what happens next and would
    make the recorded NMAC count meaningless, since it could only ever be zero.
    """
    if cfg.decision_s <= 0 or prediction_s <= 0 or horizon_s <= 0:
        raise ValueError("positive experiment, prediction and decision times required")
    if volume is None or volume.label == "nmac_event":
        raise ValueError("explicit admission volume required")
    if any(not math.isfinite(x) or x < 1 for x in duration_factors) or not duration_factors:
        raise ValueError("duration factors must be finite and at least one")
    if any(not math.isfinite(x) or x < 0 for x in start_delays) or not start_delays:
        raise ValueError("start delays must be finite and nonnegative")
    if not math.isfinite(decision_phase_s) or not 0. <= decision_phase_s < cfg.decision_s:
        raise ValueError("decision phase must lie in [0, decision_s)")
    st = initial_state(scenario)
    decision_log, pieces, safety_events = [], [], []
    status = "completed"
    # Measuring decision-phase sensitivity needs the decision clock shifted
    # against identical physics. Fly the no-change control until the first
    # decision instant, with the same audited-prefix rule as every other step.
    if decision_phase_s > 0.:
        actual = rollout(scenario, cfg, None, horizon_s=decision_phase_s, dt=dt, state=st,
                         envelope=envelope, radio=radio)
        prefix_check = admissibility_check(actual, scenario, envelope, volume, require_complete=False)
        if not prefix_check.admitted:
            safety_events.append({"t_s": 0., "stage": "executed_step",
                                  "reasons": list(prefix_check.reasons)})
            status = "completed_with_safety_events"
        actual["admission"] = prefix_check
        pieces.append(actual)
        st = actual["state"]
        decision_log.append({"t_s": 0., "action": "stay", "reason": "before_first_decision_phase",
                             "executed_safety_reasons": list(prefix_check.reasons)})
    shapes = tuple(shapes) if shapes else (QUINTIC_SHAPE,)
    shortest = {sh: minimum_duration(scenario.lane_separation_m, envelope, sh)["duration_s"] for sh in shapes}
    while st.t_s < horizon_s-1e-9:
        now = st.t_s
        baseline_safety = []
        end = now + prediction_s
        if st.profile:
            end = max(end, st.profile.end_s+cfg.decision_s)
        baseline = rollout(scenario, cfg, None, horizon_s=end, dt=dt, state=st,
                           envelope=envelope, radio=radio)
        base_check = admissibility_check(baseline, scenario, envelope, volume)
        if not base_check.admitted:
            baseline_safety = list(base_check.reasons)
            safety_events.append({"t_s": now, "reasons": baseline_safety,
                "stage": "committed_continuation" if st.profile else "stay_baseline"})
            status = "completed_with_safety_events"
        chosen = None
        record = {"t_s": now, "action": "stay", "reason": "lane_changes_disabled"}
        if st.profile:
            record["reason"] = "continue_committed_maneuver" if now < st.profile.end_s else "target_lane_following"
            record["action"] = "change" if now < st.profile.end_s else "stay"
        elif allow_lane_change:
            proposals = [Proposal(Action.CHANGE,
                LateralProfile(now+delay, shortest[sh]*factor, scenario.lane_separation_m, sh),
                label=(f"shape={sh.name},delay={delay:g},factor={factor:g}" if len(shapes) > 1
                       else f"delay={delay:g},factor={factor:g}"))
                for sh in shapes for delay in start_delays for factor in duration_factors]
            end = max(end, max(p.profile.end_s for p in proposals)+cfg.decision_s)
            # Changing the horizon requires the same recomputation of stay.
            baseline = rollout(scenario, cfg, None, horizon_s=end, dt=dt, state=st,
                               envelope=envelope, radio=radio)
            base_check = admissibility_check(baseline, scenario, envelope, volume)
            if not base_check.admitted:
                baseline_safety = list(base_check.reasons)
                safety_events.append({"t_s": now, "stage": "stay_baseline",
                                      "reasons": baseline_safety})
                status = "completed_with_safety_events"
            forecasts, checks = {}, {}
            for p in proposals:
                forecasts[p] = rollout(scenario, cfg, p.profile, horizon_s=end, dt=dt,
                                       state=st, envelope=envelope, radio=radio)
                checks[p] = admissibility_check(forecasts[p], scenario, envelope, volume)
            accepted, log = admissible(proposals, checks.__getitem__, envelope=envelope,
                                       volume=volume, horizon_s=end)
            selection = select(accepted, policy_cost(baseline)["total"],
                lambda p: policy_cost(forecasts[p])["total"], now_s=now,
                delta_num=delta_num, delta_pred=delta_pred)
            record = {"t_s": now, "action": selection["action"].value,
                      "reason": selection["reason"], "gain": selection.get("best_gain"),
                      "prediction_end_s": end, "baseline_cost": policy_cost(baseline)["total"],
                      "delta_num": delta_num, "delta_pred": delta_pred,
                      "chosen_profile": selection["chosen"].profile if selection.get("chosen") else None,
                      "candidates": [
                        {"label": v.proposal.label, "admitted": v.admitted, "reasons": v.reasons,
                         "cost": policy_cost(forecasts[v.proposal])["total"]} for v in log]}
            if selection["action"] is Action.CHANGE:
                chosen = selection["chosen"].profile
        actual_end = min(horizon_s, now+cfg.decision_s)
        actual = rollout(scenario, cfg, chosen, horizon_s=actual_end, dt=dt, state=st,
                         envelope=envelope, radio=radio)
        # Commit only an independently checked actual prefix. A failed prefix
        # is retained as a diagnostic, never included as successful execution.
        prefix_check = admissibility_check(actual, scenario, envelope, volume, require_complete=False)
        if not prefix_check.admitted:
            record["executed_safety_reasons"] = list(prefix_check.reasons)
            safety_events.append({"t_s": now, "stage": "executed_step",
                                  "reasons": list(prefix_check.reasons)})
            status = "completed_with_safety_events"
        if baseline_safety:
            record["baseline_safety_reasons"] = baseline_safety
        actual["admission"] = prefix_check
        pieces.append(actual)
        st = actual["state"]
        decision_log.append(record)
    return {"status": status, "state": st, "decisions": decision_log, "pieces": pieces,
            "safety_events": safety_events,
            "scenario": scenario, "cfg": cfg, "envelope": envelope, "admission_volume": volume,
            "policy_mode": "radio_exposure" if radio is not None else "prescribed_spatial_policy",
            "scope": "single-change straight-lane mechanism experiment; declared finite prediction"}
