"""Step-3 multi-UAM traffic on the smoothed geographic corridor.

This is an exploratory research controller, not certified separation logic.
It combines the reviewed same-flow policy groups and policy-spacing ACC with
the Step-2 rolling warning/evaluation controller.  Capacity is deliberately
absent.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import math

import numpy as np

from .lateral_study import SmoothCorridorFrame
from .minimum_change import CurvatureEnvelope, MinimumChangeConfig
from .policy_motion import BSRadio, policy_from_exposure
from .spatial_study import SpatialConfig, SpatialMove, duration_min, xyz_at


POLICY_SEVERITY = {"C": 0, "R": 1, "F": 2}


@dataclass(frozen=True)
class GeographicTrafficConfig(SpatialConfig):
    neighbors_each_side: int = 2
    d0_m: float = 200.0
    tau_c_s: float = 15.0
    tau_r_s: float = 30.0
    tau_f_s: float = 60.0
    buffer_s2_per_m: float = 0.167
    k_speed_per_s: float = 0.2
    k_gap_per_s2: float = 0.005
    k_relative_per_s: float = 0.4
    acceleration_limit_mps2: float = 1.5
    deceleration_limit_mps2: float = 2.0
    #: Wing-borne flight bounds, separate from the nominal cruise speed. The
    #: model previously used ``cruise_mps`` as both the set speed and the hard
    #: cap, so a gap could only ever open: closing one needs a brief overshoot
    #: above cruise, which the cap forbade. ``None`` reproduces that behaviour
    #: exactly, so archived runs are unaffected until a config sets a value.
    speed_min_mps: float = 0.0
    speed_max_mps: float | None = None
    horizontal_separation_m: float = 200.0
    vertical_separation_m: float = 100.0
    maximum_time_s: float = 1700.0
    admission_retry_s: float = 0.5
    # Policy-group boundary. A neighbour farther than this along the corridor
    # is left out of the focal aircraft's exposure group. None keeps the
    # original index-only rule (up to neighbors_each_side each way), so the
    # TRB reproduction is unchanged. The research setting is the fallback
    # spacing at cruise: beyond it no policy outcome of that neighbour can
    # change the separation the focal aircraft already keeps.
    group_radius_m: float | None = None
    # How exposure groups are formed.
    #   "lane_order": within the focal aircraft's own flow, neighbors_each_side
    #     aircraft ahead and behind by index (the TRB rule; archived runs).
    #   "neighbourhood": the focal aircraft plus its 2 * neighbors_each_side
    #     nearest aircraft by along-track distance, drawn from its own cell and
    #     the adjacent grid cells, within an along-track cap. The cap is
    #     group_radius_m when set, otherwise neighbors_each_side fallback
    #     spacings at cruise, so it follows the policy headways. Adjacency is
    #     read from the flow coordinates, not configured separately.
    group_mode: str = "lane_order"
    # Fixed longitudinal controller update period. None recomputes the command
    # at every integration step, which makes the step size double as the
    # controller sample period: refining it then changes the controller rather
    # than only the integration. Archived runs keep None; new studies set the
    # period explicitly, as a real flight controller has one.
    control_period_s: float | None = None

    def __post_init__(self):
        if self.group_mode not in ("lane_order", "neighbourhood"):
            raise ValueError("group_mode must be 'lane_order' or 'neighbourhood'")
        if self.group_radius_m is not None and (not np.isfinite(self.group_radius_m)
                                                or self.group_radius_m <= 0):
            raise ValueError("group_radius_m must be positive and finite, or None")
        if self.control_period_s is not None and (not np.isfinite(self.control_period_s)
                                                  or self.control_period_s <= 0):
            raise ValueError("control_period_s must be positive and finite, or None")
        super().__post_init__()
        if not isinstance(self.neighbors_each_side, int) or self.neighbors_each_side < 0:
            raise ValueError("neighbors_each_side must be a nonnegative integer")
        if not self.tau_c_s <= self.tau_r_s <= self.tau_f_s:
            raise ValueError("policy headways must be ordered")
        if not 0.0 <= self.speed_min_mps < self.cruise_mps:
            raise ValueError("speed_min_mps must be nonnegative and below cruise")
        if self.speed_max_mps is not None and self.speed_max_mps < self.cruise_mps:
            raise ValueError("speed_max_mps must be at least the cruise speed")
        if self.maximum_time_s <= 0 or self.admission_retry_s <= 0:
            raise ValueError("positive traffic horizon and retry interval required")
        if abs(self.admission_retry_s / self.dt_s
               - round(self.admission_retry_s / self.dt_s)) > 1e-8:
            raise ValueError("admission retry must align with the motion step")

    @property
    def speed_ceiling_mps(self):
        """Hard upper bound; falls back to cruise when no ceiling is declared."""
        return self.cruise_mps if self.speed_max_mps is None else self.speed_max_mps

    def spacing(self, policy, speed):
        tau = {"C": self.tau_c_s, "R": self.tau_r_s, "F": self.tau_f_s}[policy]
        return self.d0_m + tau * speed + self.buffer_s2_per_m * speed**2

    def policy_loss(self, policy):
        return {"C": 0.0, "R": self.policy_cost_r, "F": self.policy_cost_f}[policy]


@dataclass(frozen=True)
class EntryRequest:
    aircraft_id: str
    requested_time_s: float
    lane: int


@dataclass
class TrafficAircraft:
    aircraft_id: str
    q_m: float
    speed_mps: float
    lane: int
    nominal_lane: int | None = None
    policy: str = "C"
    exposure: float | None = None
    history: list = field(default_factory=list)  # (t, group fraction, member IDs)
    move: SpatialMove | None = None
    target_lane: int | None = None
    cooldown_until_s: float = 0.0
    requested_time_s: float = 0.0
    entry_time_s: float = 0.0
    completion_time_s: float | None = None
    last_acceleration_mps2: float = 0.0

    def __post_init__(self):
        if self.nominal_lane is None:
            self.nominal_lane = self.lane

    def occupied_lanes(self):
        return {self.lane} if self.target_lane is None else {self.lane, self.target_lane}

    def transverse(self, t, offsets, altitude):
        if self.move is None:
            heights = np.asarray(altitude, float)
            height = float(heights if heights.ndim == 0 else heights[self.lane])
            return (np.array([offsets[self.lane], height], float),
                    np.zeros(2), np.zeros(2))
        return self.move.sample(t)


@dataclass
class TrafficState:
    t_s: float
    active: list[TrafficAircraft]
    pending: list[EntryRequest]
    completed: list[TrafficAircraft] = field(default_factory=list)
    deferrals: dict[str, int] = field(default_factory=dict)


class OffsetArcCoordinates:
    """Physical along-offset distance built from the analytic corridor frame."""

    def __init__(self, frame, offsets, sample_m=5.0):
        self.frame = frame
        self.offsets = np.asarray(offsets, float)
        q = np.unique(np.r_[np.linspace(0, frame.length_m,
            int(np.ceil(frame.length_m / sample_m)) + 1), frame.controls_q])
        _, _, _, norm, curvature = frame.frame(q)
        self.q = q
        self.arc = []
        for offset in self.offsets:
            integrand = norm * (1.0 - curvature * offset)
            if np.any(integrand <= 0.05 * norm):
                raise ValueError("lane offset creates a folded corridor coordinate")
            cumulative = np.r_[0.0, np.cumsum((integrand[1:] + integrand[:-1])
                                               * np.diff(q) / 2.0)]
            self.arc.append(cumulative)
        self.arc = np.asarray(self.arc)

    def position(self, q, lane):
        return float(np.interp(float(q), self.q, self.arc[lane]))

    def gap(self, follower_q, leader_q, lane):
        return self.position(leader_q, lane) - self.position(follower_q, lane)

    def remaining(self, q, lane):
        return float(self.arc[lane, -1] - self.position(q, lane))


def lane_neighbors(lane, count):
    return [candidate for candidate in (lane - 1, lane + 1)
            if 0 <= candidate < count]


def flow_point(lane, offsets, altitudes):
    """Return the fixed ``(lateral offset, altitude)`` of one logical flow."""
    heights = np.asarray(altitudes, float)
    height = float(heights if heights.ndim == 0 else heights[lane])
    return (float(offsets[lane]), height)


def recovery_neighbors(lane, nominal_lane, offsets, altitudes):
    """Immediate 8-connected grid moves that approach the nominal flow.

    The one-dimensional Step-3 recovery rule moved one lane-index step toward
    nominal.  For a lateral-by-altitude lattice, the direct extension is one
    adjacent lateral and/or vertical level per decision.  Diagonal neighbors
    are included because joint lateral/vertical motion is already part of the
    frozen Step-2 candidate library.
    """
    points = np.asarray([flow_point(i, offsets, altitudes)
                         for i in range(len(offsets))], float)
    current, nominal = points[lane], points[nominal_lane]
    lateral_values = sorted(set(points[:, 0]))
    height_values = sorted(set(points[:, 1]))
    lateral_index = {value: index for index, value in enumerate(lateral_values)}
    height_index = {value: index for index, value in enumerate(height_values)}
    ci = (lateral_index[current[0]], height_index[current[1]])
    current_distance = float(np.linalg.norm(current - nominal))
    rows = []
    for candidate, point in enumerate(points):
        if candidate == lane:
            continue
        pi = (lateral_index[point[0]], height_index[point[1]])
        if max(abs(pi[0] - ci[0]), abs(pi[1] - ci[1])) != 1:
            continue
        distance = float(np.linalg.norm(point - nominal))
        if distance < current_distance - 1e-8:
            rows.append((candidate, distance, float(np.linalg.norm(point-current))))
    return [row[0] for row in sorted(rows, key=lambda row: (row[1], row[2], row[0]))]


def _physical_state(aircraft, t, frame, offsets, altitude,
                    longitudinal_acceleration=None):
    dh, vd, ad = aircraft.transverse(t, offsets, altitude)
    xyz = xyz_at(frame, aircraft.q_m, dh[0], dh[1])
    _, tangent, _, _, _ = frame.frame(aircraft.q_m)
    vxy, axy = frame.kinematics(aircraft.q_m, dh[0], vd[0], ad[0], aircraft.speed_mps)
    command = (aircraft.last_acceleration_mps2 if longitudinal_acceleration is None
               else longitudinal_acceleration)
    axy = axy + command * tangent
    return xyz, np.r_[vxy, vd[1]], np.r_[axy, ad[1]], dh, vd, ad


def separation_score(active, t, frame, offsets, altitude, cfg):
    if len(active) < 2:
        return math.inf, None
    q = np.asarray([a.q_m for a in active], float)
    dh = np.asarray([a.transverse(t, offsets, altitude)[0] for a in active])
    center, _, normal, _, _ = frame.frame(q)
    positions = np.c_[center + dh[:, 0, None] * normal, dh[:, 1]]
    minimum, issue = math.inf, None
    for i in range(len(active)):
        for j in range(i + 1, len(active)):
            delta = positions[i] - positions[j]
            score = ((delta[0] ** 2 + delta[1] ** 2) / cfg.horizontal_separation_m**2
                     + delta[2] ** 2 / cfg.vertical_separation_m**2)
            if score < minimum:
                minimum = float(score)
            if score < 1.0 - 1e-9 and issue is None:
                issue = {"reason": "three_dimensional_separation",
                         "pair": [active[i].aircraft_id, active[j].aircraft_id],
                         "score": float(score), "t_s": float(t)}
    return minimum, issue


def grid_reach(values):
    """Smallest spacing between distinct flow coordinates; 0 for a single level."""
    levels = np.unique(np.atleast_1d(np.asarray(values, float)))
    return float(np.diff(levels).min()) if len(levels) > 1 else 0.0


def neighbourhood_members(index, ids, q, d, h, *, count, along_cap_m,
                          lateral_reach_m, vertical_reach_m):
    """Focal aircraft plus its ``count`` nearest aircraft by along-track distance.

    Candidates lie in the focal cell or an adjacent one (within one grid step
    laterally and vertically, measured at the flown offset and height) and no
    farther along track than ``along_cap_m``. Returned in along-track order.
    """
    q, d, h = (np.asarray(x, float) for x in (q, d, h))
    tolerance = 1e-6
    along = np.abs(q - q[index])
    eligible = [j for j in range(len(ids)) if j != index
                and along[j] <= along_cap_m + tolerance
                and abs(d[j] - d[index]) <= lateral_reach_m + tolerance
                and abs(h[j] - h[index]) <= vertical_reach_m + tolerance]
    nearest = sorted(eligible, key=lambda j: (along[j], ids[j]))[:count]
    chosen = sorted([index, *nearest], key=lambda j: (q[j], ids[j]))
    return tuple(ids[j] for j in chosen)


def observe_groups(state, frame, offsets, altitude, cfg, radio):
    if not state.active:
        return []
    states = [_physical_state(a, state.t_s, frame, offsets, altitude)
              for a in state.active]
    positions = np.asarray([row[0] for row in states])
    values = radio(positions)
    signal = {a.aircraft_id: float(values["sinr_db"][i])
              for i, a in enumerate(state.active)}
    members_by_id = {}
    if cfg.group_mode == "neighbourhood":
        ids = [a.aircraft_id for a in state.active]
        q = [a.q_m for a in state.active]
        dh = np.asarray([row[3] for row in states], float)
        cap = (cfg.group_radius_m if cfg.group_radius_m is not None
               else cfg.neighbors_each_side * cfg.spacing("F", cfg.cruise_mps))
        lateral, vertical = grid_reach(offsets), grid_reach(altitude)
        for index, own in enumerate(state.active):
            members_by_id[own.aircraft_id] = neighbourhood_members(
                index, ids, q, dh[:, 0], dh[:, 1], count=2 * cfg.neighbors_each_side,
                along_cap_m=cap, lateral_reach_m=lateral, vertical_reach_m=vertical)
    else:
        for lane in range(len(offsets)):
            ordered = sorted([a for a in state.active if a.lane == lane],
                             key=lambda a: (a.q_m, a.aircraft_id))
            for index, own in enumerate(ordered):
                n = cfg.neighbors_each_side
                members = ordered[max(0, index - n):index + n + 1]
                if cfg.group_radius_m is not None:
                    members = [a for a in members if abs(a.q_m - own.q_m) <= cfg.group_radius_m + 1e-9]
                members_by_id[own.aircraft_id] = tuple(a.aircraft_id for a in members)
    groups = {}
    for own in state.active:
        ids = members_by_id[own.aircraft_id]
        fraction = float(np.mean([signal[identifier] < cfg.threshold_db
                                  for identifier in ids]))
        own.history = [row for row in own.history
                       if row[0] >= state.t_s - cfg.window_s - 1e-8]
        own.history.append((float(state.t_s), fraction, ids))
        own.exposure = float(np.mean([row[1] for row in own.history]))
        own.policy = policy_from_exposure(own.exposure, cfg.policy_config())
        groups[own.aircraft_id] = (ids, fraction)
    rows = []
    for index, own in enumerate(state.active):
        ids, fraction = groups[own.aircraft_id]
        rows.append({"t_s": float(state.t_s), "aircraft_id": own.aircraft_id,
            "lane": own.lane, "target_lane": own.target_lane,
            "members": list(ids), "group_size": len(ids),
            "group_bad_fraction": fraction, "history_count": len(own.history),
            "exposure": own.exposure, "policy": own.policy,
            "sinr_db": signal[own.aircraft_id],
            "serving_bs": int(values["serving_bs"][index]),
            "rsrp_dbm": float(values["serving_rsrp_dbm"][index])})
    return rows


def acc_controls(state, arc, cfg, *, recovery_pairs=frozenset()):
    """Bounded ACC, with optional target recovery for established follower pairs.

    The caller owns relationship history. Merely detecting a distant leader
    must not enable pursuit. During recovery, omit the nominal-cruise veto but
    retain every occupied-lane leader's following constraint and physical bounds.
    With no matching pair (including a departed leader), use ordinary cruise ACC.
    """
    controls = {}
    for own in state.active:
        leaders = []
        for lane in sorted(own.occupied_lanes()):
            ahead = [candidate for candidate in state.active
                     if candidate is not own and lane in candidate.occupied_lanes()
                     and candidate.q_m > own.q_m + 1e-9]
            if ahead:
                leader = min(ahead, key=lambda a: arc.position(a.q_m, lane))
                if leader.aircraft_id not in [row[0] for row in leaders]:
                    leaders.append((leader.aircraft_id,
                                    arc.gap(own.q_m, leader.q_m, lane),
                                    leader.speed_mps - own.speed_mps, lane))
        free = cfg.k_speed_per_s * (cfg.cruise_mps - own.speed_mps)
        following = [cfg.k_gap_per_s2 * (gap - cfg.spacing(own.policy, own.speed_mps))
                     + cfg.k_relative_per_s * relative
                     for _, gap, relative, _ in leaders]
        recovering = any((own.aircraft_id, leader_id) in recovery_pairs
                         for leader_id, _, _, _ in leaders)
        raw = min(following) if recovering else min([free] + following)
        command = float(np.clip(raw, -cfg.deceleration_limit_mps2,
                                cfg.acceleration_limit_mps2))
        if own.speed_mps <= 1e-9 and command < 0:
            command = 0.0
        if own.speed_mps >= cfg.speed_ceiling_mps - 1e-9 and command > 0:
            command = 0.0
        if own.speed_mps <= cfg.speed_min_mps + 1e-9 and command < 0:
            command = 0.0
        controls[own.aircraft_id] = {"raw": float(raw), "command": command,
                                     "leaders": leaders}
    return controls


def insertion_deficits(own, target_lane, state, arc, cfg):
    candidates = [a for a in state.active if a is not own
                  and target_lane in a.occupied_lanes()]
    ahead = [a for a in candidates if a.q_m > own.q_m + 1e-9]
    behind = [a for a in candidates if a.q_m < own.q_m - 1e-9]
    deficits = []
    if ahead:
        leader = min(ahead, key=lambda a: arc.position(a.q_m, target_lane))
        gap = arc.gap(own.q_m, leader.q_m, target_lane)
        desired = cfg.spacing(own.policy, own.speed_mps)
        if gap < desired - 1e-8:
            deficits.append({"relationship": "target_predecessor",
                "follower": own.aircraft_id, "leader": leader.aircraft_id,
                "gap_m": gap, "desired_gap_m": desired})
    if behind:
        follower = max(behind, key=lambda a: arc.position(a.q_m, target_lane))
        gap = arc.gap(follower.q_m, own.q_m, target_lane)
        desired = cfg.spacing(follower.policy, follower.speed_mps)
        if gap < desired - 1e-8:
            deficits.append({"relationship": "target_follower",
                "follower": follower.aircraft_id, "leader": own.aircraft_id,
                "gap_m": gap, "desired_gap_m": desired})
    return deficits


def _progress_step(frame, aircraft, t, dt, command, offsets, altitude, cfg):
    q0, v0 = aircraft.q_m, aircraft.speed_mps
    def rate(q, tau):
        dh = aircraft.transverse(t + tau, offsets, altitude)[0]
        speed = float(np.clip(v0 + command * tau, cfg.speed_min_mps, cfg.speed_ceiling_mps))
        return frame.progress_rate(q, dh[0], speed)
    k1 = rate(q0, 0.0)
    k2 = rate(q0 + dt * k1 / 2, dt / 2)
    k3 = rate(q0 + dt * k2 / 2, dt / 2)
    k4 = rate(q0 + dt * k3, dt)
    return float(q0 + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6)


def _progress_collection(frame, collection, t, dt, controls, offsets, altitude, cfg):
    """Vectorized RK4 equivalent of ``_progress_step`` for one state copy."""
    if not collection:
        return
    q0 = np.asarray([a.q_m for a in collection], float)
    v0 = np.asarray([a.speed_mps for a in collection], float)
    commands = np.asarray([controls[a.aircraft_id]["command"] for a in collection], float)

    def rate(q, tau):
        lateral = np.asarray([a.transverse(t + tau, offsets, altitude)[0][0]
                              for a in collection], float)
        speed = np.clip(v0 + commands * tau, cfg.speed_min_mps, cfg.speed_ceiling_mps)
        return frame.progress_rate(q, lateral, speed)

    k1 = rate(q0, 0.0)
    k2 = rate(q0 + dt * k1 / 2, dt / 2)
    k3 = rate(q0 + dt * k2 / 2, dt / 2)
    k4 = rate(q0 + dt * k3, dt)
    q1 = q0 + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    v1 = np.clip(v0 + commands * dt, cfg.speed_min_mps, cfg.speed_ceiling_mps)
    for index, own in enumerate(collection):
        own.q_m = float(q1[index])
        own.speed_mps = float(v1[index])
        own.last_acceleration_mps2 = float(commands[index])


def _settle(state, cfg, events=None):
    for own in list(state.active):
        if own.move and state.t_s >= own.move.start_s + own.move.duration_s - 1e-8:
            source_lane, target_lane = own.lane, own.target_lane
            own.lane = own.target_lane
            own.target_lane = None
            own.move = None
            own.cooldown_until_s = state.t_s + cfg.cooldown_s
            if events is not None:
                events.append({"t_s": state.t_s, "aircraft_id": own.aircraft_id,
                    "status": "transition_completed", "source_lane": source_lane,
                    "target_lane": target_lane, "nominal_lane": own.nominal_lane,
                    "on_nominal_after": target_lane == own.nominal_lane})


def release_due(state, frame, offsets, altitude, cfg, arc, events):
    for request in list(state.pending):
        if request.requested_time_s > state.t_s + 1e-8:
            continue
        candidate = TrafficAircraft(request.aircraft_id, 0.0, cfg.cruise_mps,
            request.lane, nominal_lane=request.lane,
            requested_time_s=request.requested_time_s,
            entry_time_s=state.t_s)
        deficits = insertion_deficits(candidate, request.lane,
            TrafficState(state.t_s, [*state.active, candidate], []), arc, cfg)
        _, issue = separation_score([*state.active, candidate], state.t_s,
                                    frame, offsets, altitude, cfg)
        if deficits or issue:
            state.deferrals[request.aircraft_id] = state.deferrals.get(request.aircraft_id, 0) + 1
            continue
        state.active.append(candidate)
        state.pending.remove(request)
        events.append({"t_s": state.t_s, "aircraft_id": request.aircraft_id,
            "status": "corridor_entry", "requested_time_s": request.requested_time_s,
            "entry_delay_s": state.t_s - request.requested_time_s, "lane": request.lane,
            "nominal_lane": request.lane})


def advance_state(state, frame, offsets, altitude, cfg, arc, dt, controls,
                  events=None):
    midpoint = deepcopy(state.active)
    endpoint = deepcopy(state.active)
    for collection, fraction in ((midpoint, 0.5), (endpoint, 1.0)):
        _progress_collection(frame, collection, state.t_s, dt * fraction,
                             controls, offsets, altitude, cfg)
    min_mid, issue_mid = separation_score(midpoint, state.t_s + dt / 2,
                                           frame, offsets, altitude, cfg)
    min_end, issue_end = separation_score(endpoint, state.t_s + dt,
                                           frame, offsets, altitude, cfg)
    if issue_mid or issue_end:
        return min(min_mid, min_end), issue_mid or issue_end
    state.active = endpoint
    state.t_s = float(state.t_s + dt)
    _settle(state, cfg, events)
    for own in list(state.active):
        if own.q_m >= frame.length_m - 1e-7:
            own.q_m = frame.length_m
            own.completion_time_s = state.t_s
            state.active.remove(own)
            state.completed.append(own)
    return min(min_mid, min_end), None


def _next_step(state, cfg):
    end = state.t_s + cfg.dt_s
    clocks = [cfg.policy_s, cfg.decision_s]
    for period in clocks:
        next_clock = (math.floor((state.t_s + 1e-8) / period) + 1) * period
        end = min(end, next_clock)
    for own in state.active:
        if own.move:
            end = min(end, own.move.start_s + own.move.duration_s)
    future_entries = [row.requested_time_s for row in state.pending
                      if row.requested_time_s > state.t_s + 1e-8]
    if future_entries:
        end = min(end, min(future_entries))
    return end - state.t_s


def forecast(state, frame, offsets, altitude, cfg, arc, radio, horizon_s,
             focal_ids, candidate=None):
    trial = deepcopy(state)
    if candidate is not None:
        aircraft_id, target_lane = candidate
        own = next(a for a in trial.active if a.aircraft_id == aircraft_id)
        source = flow_point(own.lane, offsets, altitude)
        target = flow_point(target_lane, offsets, altitude)
        own.move = SpatialMove(trial.t_s, source, target, duration_min(source, target, cfg))
        own.target_lane = target_lane
    end = state.t_s + horizon_s
    costs = {identifier: 0.0 for identifier in focal_ids}
    policies = {identifier: [(state.t_s,
        next(a.policy for a in trial.active if a.aircraft_id == identifier))]
        for identifier in focal_ids if any(a.aircraft_id == identifier for a in trial.active)}
    command_integrals = {identifier: 0.0 for identifier in focal_ids}
    events, minimum = [], math.inf
    issue = None
    while trial.t_s < end - 1e-8 and not issue:
        _settle(trial, cfg)
        release_due(trial, frame, offsets, altitude, cfg, arc, events)
        if abs(trial.t_s / cfg.policy_s - round(trial.t_s / cfg.policy_s)) < 1e-8 \
                and trial.t_s > state.t_s + 1e-8:
            rows = observe_groups(trial, frame, offsets, altitude, cfg, radio)
            for row in rows:
                if row["aircraft_id"] in policies:
                    policies[row["aircraft_id"]].append((trial.t_s, row["policy"]))
        controls = acc_controls(trial, arc, cfg)
        dt = min(_next_step(trial, cfg), end - trial.t_s)
        for own in trial.active:
            if own.aircraft_id in costs:
                costs[own.aircraft_id] += cfg.policy_loss(own.policy) * dt
                command_integrals[own.aircraft_id] += max(0.0,
                    -controls[own.aircraft_id]["command"]) * dt
        score, issue = advance_state(trial, frame, offsets, altitude, cfg,
                                     arc, dt, controls)
        minimum = min(minimum, score)
    return {"state": trial, "costs": costs, "policies": policies,
            "braking_integrals": command_integrals,
            "minimum_separation_score": minimum, "issue": issue}


def rolling_decision(state, frame, offsets, altitude, cfg, arc, radio,
                     curvature, events):
    eligible_aircraft = [a for a in state.active if a.move is None
        and state.t_s >= a.cooldown_until_s - 1e-8
        and arc.remaining(a.q_m, a.lane) > cfg.cruise_mps
        * (cfg.predictive_rollout_s + cfg.predictive_evaluation_s + 5.0)]
    if not eligible_aircraft:
        return
    identifiers = [a.aircraft_id for a in eligible_aircraft]
    horizon = cfg.predictive_rollout_s + cfg.predictive_evaluation_s
    baseline = forecast(state, frame, offsets, altitude, cfg, arc, radio,
                        horizon, identifiers)
    if baseline["issue"]:
        events.append({"t_s": state.t_s, "status": "baseline_forecast_infeasible",
                       "detail": baseline["issue"]})
        return
    return_proposals = []
    avoidance_proposals = []

    def evaluate(own, target_lane, decision_type, degradation):
        event = {"t_s": state.t_s, "aircraft_id": own.aircraft_id,
            "source_lane": own.lane, "target_lane": target_lane,
            "nominal_lane": own.nominal_lane, "decision_type": decision_type,
            "trigger_policy": own.policy,
            "predicted_degradation_time_s": None if degradation is None else degradation[0],
            "predicted_degradation_policy": None if degradation is None else degradation[1]}
        deficits = insertion_deficits(own, target_lane, state, arc, cfg)
        if deficits:
            events.append({**event, "status": "insertion_gap_rejected",
                           "deficits": deficits})
            return None
        source = flow_point(own.lane, offsets, altitude)
        target = flow_point(target_lane, offsets, altitude)
        duration = duration_min(source, target, cfg)
        if duration > horizon + 1e-8:
            events.append({**event, "status": "maneuver_exceeds_planning_horizon",
                           "duration_s": duration, "planning_horizon_s": horizon})
            return None
        q_after = min(frame.length_m, own.q_m + cfg.cruise_mps * duration)
        geometry = curvature.offset_bound(own.q_m, q_after,
                                           [source[0], target[0]])
        maneuver_bound = (geometry + 10 / np.sqrt(3) * np.linalg.norm(
                          np.asarray(target) - source)
                          / duration**2 / max(own.speed_mps, 1e-6)**2)
        if maneuver_bound > 1 / 976.0 + 1e-12:
            events.append({**event, "status": "curvature_rejected",
                           "curvature_bound_per_m": float(maneuver_bound)})
            return None
        branch = forecast(state, frame, offsets, altitude, cfg, arc, radio,
                          horizon, identifiers,
                          candidate=(own.aircraft_id, target_lane))
        if branch["issue"]:
            events.append({**event, "status": "three_dimensional_forecast_rejected",
                           "detail": branch["issue"]})
            return None
        stay_cost = baseline["costs"][own.aircraft_id]
        move_cost = branch["costs"][own.aircraft_id]
        improvement = stay_cost - move_cost
        stay_endpoint = baseline["policies"].get(
            own.aircraft_id, [(state.t_s, own.policy)])[-1][1]
        move_endpoint = branch["policies"].get(
            own.aircraft_id, [(state.t_s, own.policy)])[-1][1]
        if decision_type == "nominal_return":
            acceptable = (move_cost <= stay_cost + 1e-8
                          and POLICY_SEVERITY[move_endpoint]
                          <= POLICY_SEVERITY[stay_endpoint])
            rejected_status = "nominal_return_policy_worse"
        else:
            # Frozen Step-2 acceptance: any strict integrated-policy
            # improvement is sufficient; no additional ad-hoc threshold.
            acceptable = move_cost < stay_cost - 1e-8
            rejected_status = "policy_cost_not_improved"
        if not acceptable:
            events.append({**event, "status": rejected_status,
                "stay_policy_cost_s": stay_cost, "move_policy_cost_s": move_cost,
                "policy_cost_improvement_s": improvement,
                "stay_endpoint_policy": stay_endpoint,
                "move_endpoint_policy": move_endpoint})
            return None
        disturbance = sum(max(0.0,
            branch["braking_integrals"].get(identifier, 0.0)
            - baseline["braking_integrals"].get(identifier, 0.0))
            for identifier in identifiers if identifier != own.aircraft_id)
        return {**event, "status": "eligible", "duration_s": duration,
            "stay_policy_cost_s": stay_cost, "move_policy_cost_s": move_cost,
            "policy_cost_improvement_s": improvement,
            "stay_endpoint_policy": stay_endpoint,
            "move_endpoint_policy": move_endpoint,
            "displacement_m": float(np.linalg.norm(np.asarray(target) - source)),
            "distance_to_nominal_after_m": float(np.linalg.norm(
                np.asarray(target) - flow_point(own.nominal_lane, offsets, altitude))),
            "extra_nonfocal_braking_mps": disturbance,
            "minimum_forecast_separation_score": branch["minimum_separation_score"]}

    for own in eligible_aircraft:
        warning_end = state.t_s + cfg.predictive_rollout_s
        future = baseline["policies"].get(own.aircraft_id, [])
        degradation = next(((time, policy) for time, policy in future
            if time <= warning_end + 1e-8
            and POLICY_SEVERITY[policy] > POLICY_SEVERITY[own.policy]), None)
        # Newly agreed Step-3 state: when away from the assigned nominal
        # flow, first test one safe step back toward it.  Returning may tie the
        # stay policy cost but may never make policy or endpoint policy worse.
        if own.lane != own.nominal_lane:
            records = []
            for target_lane in recovery_neighbors(
                    own.lane, own.nominal_lane, offsets, altitude):
                record = evaluate(own, target_lane, "nominal_return", degradation)
                if record is not None:
                    records.append(record)
            if records:
                chosen = min(records, key=lambda row: (
                    row["distance_to_nominal_after_m"], row["move_policy_cost_s"],
                    POLICY_SEVERITY[row["move_endpoint_policy"]],
                    row["displacement_m"], row["duration_s"], row["target_lane"]))
                for record in records:
                    if record is not chosen:
                        events.append({**record,
                            "status": "eligible_not_selected_nominal_recovery"})
                return_proposals.append((own, chosen["target_lane"], chosen))
                continue
        if degradation is None:
            continue
        records = []
        for target_lane in range(len(offsets)):
            if target_lane == own.lane:
                continue
            record = evaluate(own, target_lane, "policy_avoidance", degradation)
            if record is not None:
                records.append(record)
        if records:
            # Frozen Step-2 within-aircraft selector: minimum integrated
            # policy loss, better endpoint policy, minimum displacement,
            # minimum maneuver time, deterministic coordinate tie-break.
            chosen = min(records, key=lambda row: (
                row["move_policy_cost_s"],
                POLICY_SEVERITY[row["move_endpoint_policy"]],
                row["displacement_m"], row["duration_s"], row["target_lane"]))
            for record in records:
                if record is not chosen:
                    events.append({**record,
                                   "status": "eligible_not_selected_step2_selector"})
            avoidance_proposals.append((own, chosen["target_lane"], chosen))
    proposals = return_proposals if return_proposals else avoidance_proposals
    if not proposals:
        return
    if return_proposals:
        proposals.sort(key=lambda row: (
            row[2]["distance_to_nominal_after_m"],
            row[2]["move_policy_cost_s"],
            row[2]["extra_nonfocal_braking_mps"], row[2]["duration_s"],
            row[0].aircraft_id, row[1]))
    else:
        # Existing conservative multi-aircraft arbitration is retained for
        # this pilot and remains explicitly provisional.
        proposals.sort(key=lambda row: (
            -row[2]["policy_cost_improvement_s"],
            row[2]["extra_nonfocal_braking_mps"], row[2]["duration_s"],
            row[0].aircraft_id, row[1]))
    own, target_lane, chosen = proposals[0]
    for _, _, record in proposals[1:]:
        events.append({**record, "status": "eligible_not_selected_global_arbitration"})
    source = flow_point(own.lane, offsets, altitude)
    target = flow_point(target_lane, offsets, altitude)
    own.move = SpatialMove(state.t_s, source, target, chosen["duration_s"])
    own.target_lane = target_lane
    events.append({**chosen, "status": "transition_accepted",
                   "source": list(source), "target": list(target)})


def simulate_geographic_traffic(frame, scenario, cfg, offsets, entries,
                                *, transitions, flow_altitudes=None):
    offsets = np.asarray(offsets, float)
    altitudes = (np.full(len(offsets), cfg.altitude_m, float)
                 if flow_altitudes is None else np.asarray(flow_altitudes, float))
    if len(offsets) < 1 or altitudes.shape != offsets.shape \
            or not np.isfinite(offsets).all() or not np.isfinite(altitudes).all():
        raise ValueError("finite lateral/altitude coordinate required for every flow")
    coordinates = list(zip(offsets.tolist(), altitudes.tolist()))
    if len(set(coordinates)) != len(coordinates):
        raise ValueError("logical flow coordinates must be unique")
    if not cfg.predictive_endpoint_upgrade or cfg.predictive_rollout_s <= 0:
        raise ValueError("Step-3 requires the rolling dual-horizon controller")
    ids = [row.aircraft_id for row in entries]
    if len(ids) != len(set(ids)) or any(not identifier for identifier in ids):
        raise ValueError("unique nonempty aircraft IDs required")
    if any(row.lane < 0 or row.lane >= len(offsets)
           or row.requested_time_s < 0 for row in entries):
        raise ValueError("invalid entry schedule")
    arc = OffsetArcCoordinates(frame, offsets)
    curvature = CurvatureEnvelope(frame)
    radio = BSRadio(scenario.base_stations, scenario.radio)
    state = TrafficState(0.0, [], sorted(list(entries),
        key=lambda row: (row.requested_time_s, row.aircraft_id)))
    observations, events, trace = [], [], []
    minimum_separation = math.inf
    issue = None
    last_policy = -math.inf
    while state.t_s < cfg.maximum_time_s - 1e-8 and not issue:
        _settle(state, cfg, events)
        release_due(state, frame, offsets, altitudes, cfg, arc, events)
        if abs(state.t_s / cfg.policy_s - round(state.t_s / cfg.policy_s)) < 1e-8 \
                and state.t_s > last_policy + 1e-8:
            observations.extend(observe_groups(state, frame, offsets,
                                               altitudes, cfg, radio))
            last_policy = state.t_s
        if transitions and state.active \
                and abs(state.t_s / cfg.decision_s - round(state.t_s / cfg.decision_s)) < 1e-8:
            rolling_decision(state, frame, offsets, altitudes, cfg,
                             arc, radio, curvature, events)
        if not state.active:
            if not state.pending:
                break
            future = [row.requested_time_s for row in state.pending
                      if row.requested_time_s > state.t_s + 1e-8]
            state.t_s = (min(future) if future else
                         state.t_s + cfg.admission_retry_s)
            continue
        controls = acc_controls(state, arc, cfg)
        dt = _next_step(state, cfg)
        for own in state.active:
            control = controls[own.aircraft_id]
            position, velocity, acceleration, dh, vd, ad = _physical_state(
                own, state.t_s, frame, offsets, altitudes,
                control["command"])
            gap = min((row[1] for row in control["leaders"]), default=None)
            trace.append({"t_s": state.t_s, "end_s": state.t_s + dt,
                "aircraft_id": own.aircraft_id, "q_m": own.q_m,
                "lane": own.lane, "target_lane": own.target_lane,
                "nominal_lane": own.nominal_lane,
                "offset_m": float(dh[0]), "altitude_m": float(dh[1]),
                "position_m": position.tolist(), "velocity_mps": velocity.tolist(),
                "acceleration_mps2": acceleration.tolist(),
                "longitudinal_speed_mps": own.speed_mps,
                "transverse_velocity_mps": vd.tolist(),
                "transverse_acceleration_mps2": ad.tolist(),
                "policy": own.policy, "exposure": own.exposure,
                "leaders": [list(row) for row in control["leaders"]],
                "desired_gap_m": cfg.spacing(own.policy, own.speed_mps),
                "actual_gap_m": gap, "raw_acceleration_mps2": control["raw"],
                "command_acceleration_mps2": control["command"]})
        score, issue = advance_state(state, frame, offsets, altitudes,
                                     cfg, arc, dt, controls, events)
        minimum_separation = min(minimum_separation, score)
        if not state.pending and not state.active:
            break
    accepted = [row for row in events if row["status"] == "transition_accepted"]
    completed_count = sum(1 for row in events
                          if row["status"] == "transition_completed")
    completed_returns = sum(1 for row in events
        if row["status"] == "transition_completed"
        and row.get("target_lane") == next(
            (request.lane for request in entries
             if request.aircraft_id == row["aircraft_id"]), None))
    policy_time = {policy: 0.0 for policy in "CRF"}
    for row in trace:
        policy_time[row["policy"]] += row["end_s"] - row["t_s"]
    total_aircraft_time = sum(policy_time.values())
    summary = {
        "status": "stopped_on_safety_issue" if issue else
                  "completed" if not state.pending and not state.active else "time_limit",
        "aircraft_count": len(entries), "lateral_flows_m": offsets.tolist(),
        "flow_altitudes_m": altitudes.tolist(),
        "flow_coordinates_m": [list(row) for row in coordinates],
        "altitude_m": cfg.altitude_m, "transitions_enabled": bool(transitions),
        "accepted_transitions": len(accepted),
        "completed_transitions": completed_count,
        "completed_nominal_returns": completed_returns,
        "policy_aircraft_time_s": policy_time,
        "policy_aircraft_time_shares": {key: value / total_aircraft_time
            if total_aircraft_time else 0.0 for key, value in policy_time.items()},
        "minimum_longitudinal_speed_mps": min(
            (row["longitudinal_speed_mps"] for row in trace), default=None),
        "minimum_three_dimensional_separation_score": minimum_separation,
        "entry_deferrals": state.deferrals,
        "requested_entries": len(entries),
        "realized_entries": sum(row["status"] == "corridor_entry" for row in events),
        "completed_missions": len(state.completed),
        "completed_on_nominal_lane": sum(
            own.lane == own.nominal_lane for own in state.completed),
        "pending_entries": len(state.pending),
        "safety_issue": issue,
        "capacity_estimated": False,
        "capacity_status": "capacity_not_estimated",
        "scientific_status": "multi-UAM communication/ACC/lane-change mechanism test; not capacity or certified safety",
        "safety_screen": "all-pair Cartesian ellipsoid at execution endpoints and midpoints; engineering screen, no continuous proof",
        "arbitration": "at most one globally ranked transition starts on each decision clock",
    }
    return {"summary": summary, "trace": trace, "observations": observations,
            "events": events, "state": state}
