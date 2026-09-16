"""Reviewed policy-spacing loop on a LOCAL STRAIGHT corridor.

Research point-mass model, not certified flight control. The historic synthetic
prototype is deliberately untouched. Geographic dynamics are not substituted
with a polyline approximation. All distances here are in a common metric frame.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, replace
from itertools import combinations
import math

import numpy as np
from numpy.polynomial import polynomial as poly

from capacity_policy.radio import compute_link_state
from capacity_policy.trajectory import TrajectoryState
from .dynamic_transitions import Layout, Maneuver, make_maneuver


@dataclass(frozen=True)
class Config:
    dt_s: float = 0.5
    policy_s: float = 5.0
    decision_s: float = 20.0
    horizon_s: float = 240.0
    duration_s: float = 1600.0
    window_s: float = 30.0
    cruise_mps: float = 50.0
    d0_m: float = 200.0
    tau_c_s: float = 15.0
    tau_r_s: float = 30.0
    tau_f_s: float = 60.0
    buffer_s2_per_m: float = 0.167
    k_speed: float = 0.2
    k_gap: float = 0.005
    k_relative: float = 0.4
    accel_limit_mps2: float = 1.5
    decel_limit_mps2: float = 2.0
    accept_braking_mps2: float = 2.0  # executed braking, only if worse than staying
    threshold_db: float = -1.5
    exposure_c: float = 0.05
    exposure_r: float = 0.10
    neighbors: int = 2
    benefit: float = 0.02
    horizontal_separation_m: float = 200.0
    vertical_separation_m: float = 100.0
    min_maneuver_s: float = 30.0
    lateral_speed_limit_mps: float = 8.0
    vertical_speed_limit_mps: float = 3.0
    lateral_accel_limit_mps2: float = 0.3
    vertical_accel_limit_mps2: float = 0.2
    cooldown_s: float = 30.0
    connector_s: float = 400.0
    entry_m: float = 10000.0
    exit_m: float = 30000.0
    destination_m: float = 40000.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not np.isfinite(value):
                raise ValueError(f"nonfinite configuration: {name}")
            if name != "threshold_db" and value < 0:
                raise ValueError(f"negative configuration: {name}")
        for name in ("dt_s", "policy_s", "decision_s", "horizon_s", "duration_s",
                     "window_s", "cruise_mps", "connector_s", "horizontal_separation_m",
                     "vertical_separation_m", "accel_limit_mps2", "decel_limit_mps2",
                     "lateral_speed_limit_mps", "vertical_speed_limit_mps",
                     "lateral_accel_limit_mps2", "vertical_accel_limit_mps2"):
            if getattr(self, name) <= 0:
                raise ValueError(f"positive configuration required: {name}")
        if not isinstance(self.neighbors, int) or self.neighbors < 0:
            raise ValueError("neighbor count must be a nonnegative integer")
        if not 0 <= self.exposure_c <= self.exposure_r <= 1 or not 0 < self.benefit < 1:
            raise ValueError("invalid exposure thresholds")
        if not 0 < self.accept_braking_mps2 <= self.decel_limit_mps2:
            raise ValueError("accepted braking must not exceed normal braking")
        if not self.tau_c_s <= self.tau_r_s <= self.tau_f_s:
            raise ValueError("policy time-gap ordering")
        if not 0 < self.entry_m < self.exit_m < self.destination_m:
            raise ValueError("invalid corridor gates")
        for value in (self.policy_s, self.decision_s, self.duration_s, self.horizon_s):
            if abs(value/self.dt_s-round(value/self.dt_s)) > 1e-8:
                raise ValueError("clocks must be multiples of motion step")

    def spacing(self, policy, speed):
        tau = {"C": self.tau_c_s, "R": self.tau_r_s, "F": self.tau_f_s}[policy]
        return self.d0_m + tau*speed + self.buffer_s2_per_m*speed**2


def policy_from_exposure(value, cfg):
    if not np.isfinite(value) or not 0 <= value <= 1:
        raise ValueError("invalid exposure")
    return "C" if value <= cfg.exposure_c+1e-12 else "R" if value <= cfg.exposure_r+1e-12 else "F"


def real_roots(coefficients, lo=0.0, hi=1.0):
    """Numerical polynomial extrema; not a formal interval-arithmetic proof."""
    c = poly.polytrim(np.asarray(coefficients, float), tol=1e-14)
    if len(c) <= 1:
        return []
    return [float(x.real) for x in poly.polyroots(c)
            if abs(x.imag) < 1e-7 and lo+1e-10 < x.real < hi-1e-10]


@dataclass(frozen=True)
class Quintic:
    start: float
    duration: float
    coefficients: np.ndarray  # ascending normalized-time power, (6, 3)

    @classmethod
    def boundary(cls, start, duration, p0, p1, v0, v1, a0=None, a1=None):
        p0, p1, v0, v1 = map(lambda x: np.asarray(x, float), (p0, p1, v0, v1))
        a0 = np.zeros(3) if a0 is None else np.asarray(a0, float)
        a1 = np.zeros(3) if a1 is None else np.asarray(a1, float)
        if duration <= 0 or not np.isfinite([start, duration]).all():
            raise ValueError("invalid connector duration")
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in (p0, p1, v0, v1, a0, a1)):
            raise ValueError("finite three-dimensional boundary states required")
        A = p1-p0-duration*v0-.5*duration**2*a0
        B = duration*(v1-v0)-duration**2*a0
        C = duration**2*(a1-a0)
        return cls(start, duration, np.array([p0, duration*v0, .5*duration**2*a0,
                                             10*A-4*B+C/2, -15*A+7*B-C, 6*A-3*B+C/2]))

    def sample(self, t):
        u = min(1., max(0., (t-self.start)/self.duration))
        c = self.coefficients
        return (poly.polyval(u, c), poly.polyval(u, poly.polyder(c))/self.duration,
                poly.polyval(u, poly.polyder(c, 2))/self.duration**2)

    def interval(self, t, dt):
        """Exact coefficients in u=(time-t)/dt on one execution interval."""
        a, b = (t-self.start)/self.duration, dt/self.duration
        result = np.zeros((6, 3))
        for j in range(6):
            for k in range(j+1):
                result[k] += self.coefficients[j]*math.comb(j, k)*a**(j-k)*b**k
        return result

    def feasible(self, cfg):
        limits = [
            (0, 0, math.inf), (2, 0, math.inf),
        ]
        for axis, low, high in limits:
            c = self.coefficients[:, axis]
            values = poly.polyval([0, 1, *real_roots(poly.polyder(c))], c)
            if min(values) < low-1e-7 or max(values) > high:
                return False
        for order, bounds in [
            (1, [(0, cfg.cruise_mps), (-cfg.lateral_speed_limit_mps, cfg.lateral_speed_limit_mps),
                 (-cfg.vertical_speed_limit_mps, cfg.vertical_speed_limit_mps)]),
            (2, [(-cfg.decel_limit_mps2, cfg.accel_limit_mps2),
                 (-cfg.lateral_accel_limit_mps2, cfg.lateral_accel_limit_mps2),
                 (-cfg.vertical_accel_limit_mps2, cfg.vertical_accel_limit_mps2)]),
        ]:
            for axis, (low, high) in enumerate(bounds):
                c = poly.polyder(self.coefficients[:, axis], order)/self.duration**order
                values = poly.polyval([0, 1, *real_roots(poly.polyder(c))], c)
                if min(values) < low-1e-7 or max(values) > high+1e-7:
                    return False
        return True


@dataclass
class Vehicle:
    aircraft_id: str
    s_m: float
    v_mps: float
    cell: tuple[int, int]
    phase: str = "CORRIDOR"
    policy: str = "C"
    history: list = field(default_factory=list)  # (time, fraction, tuple(member IDs))
    exposure: float | None = None
    maneuver: Maneuver | None = None
    connector: Quintic | None = None
    cooldown_until: float = 0.
    request_s: float | None = None
    release_s: float | None = None
    entry_s: float | None = None
    exit_s: float | None = None
    completion_s: float | None = None
    last_acceleration: float = 0.

    @property
    def corridor(self):
        return self.phase == "CORRIDOR"

    def cells(self):
        return {self.cell} if self.maneuver is None else {self.maneuver.source, self.maneuver.target}

    def pose(self, t, layout):
        if self.connector:
            return self.connector.sample(t)
        if self.maneuver:
            yz, vyz, ayz = self.maneuver.sample(t, layout)
        else:
            yz, vyz, ayz = layout.position(self.cell), np.zeros(2), np.zeros(2)
        return np.r_[self.s_m, yz], np.r_[self.v_mps, vyz], np.r_[self.last_acceleration, ayz]

    def interval(self, t, dt, acceleration, layout):
        if self.connector:
            return self.connector.interval(t, dt)
        c = np.zeros((6, 3))
        c[:3, 0] = [self.s_m, self.v_mps*dt, .5*acceleration*dt**2]
        if self.maneuver:
            m = self.maneuver
            p0, p1 = np.r_[0., layout.position(m.source)], np.r_[0., layout.position(m.target)]
            transverse = Quintic.boundary(m.start_s, m.duration_s, p0, p1, np.zeros(3), np.zeros(3))
            c[:, 1:] = transverse.interval(t, dt)[:, 1:]
        else:
            c[0, 1:] = layout.position(self.cell)
        return c


class BSRadio:
    """Adapter to the existing radio kernel, with complete per-site behavior."""
    def __init__(self, stations, config):
        self.stations, self.config = stations, config

    def __call__(self, positions):
        positions = np.asarray(positions, float)
        n = len(positions)
        state = TrajectoryState(np.array([0.]), positions[:, None, :],
                                positions[:, 0, None], np.ones((n, 1), bool))
        radio = compute_link_state(state, self.stations, self.config)
        return {key: radio[key][:, 0] for key in ("sinr_db", "serving_bs", "serving_rsrp_dbm")}


def radio_values(radio, positions):
    result = radio(np.asarray(positions, float))
    if not isinstance(result, dict):
        result = {"sinr_db": np.asarray(result, float)}
    if np.shape(result["sinr_db"]) != (len(positions),) or not np.isfinite(result["sinr_db"]).all():
        raise ValueError("radio must return finite SINR for every aircraft")
    return result


def observe(fleet, t, layout, cfg, radio):
    """Same-flow rank groups; history is the mean of actual past group fractions."""
    active = sorted([a for a in fleet if a.phase != "COMPLETE"], key=lambda a: a.aircraft_id)
    if not active:
        return []
    values = radio_values(radio, [a.pose(t, layout)[0] for a in active])
    signals = {a.aircraft_id: float(values["sinr_db"][i]) for i, a in enumerate(active)}
    groups = {}
    for cell in sorted({a.cell for a in active if a.corridor}):
        ordered = sorted([a for a in active if a.corridor and a.cell == cell],
                         key=lambda a: (a.s_m, a.aircraft_id))
        for k, a in enumerate(ordered):
            members = ordered[max(0, k-cfg.neighbors):k+cfg.neighbors+1]
            ids = tuple(m.aircraft_id for m in members)
            fraction = sum(signals[i] < cfg.threshold_db for i in ids)/len(ids)
            if a.history and abs(a.history[-1][0]-t) < 1e-8:
                raise ValueError("duplicate policy observation")
            a.history = [h for h in a.history if h[0] >= t-cfg.window_s-1e-9]
            a.history.append((float(t), float(fraction), ids))
            a.exposure = sum(h[1] for h in a.history)/len(a.history)
            a.policy = policy_from_exposure(a.exposure, cfg)
            groups[a.aircraft_id] = (ids, fraction)
    rows = []
    for i, a in enumerate(active):
        ids, fraction = groups.get(a.aircraft_id, ((), None))
        rows.append({
            "t_s": float(t), "aircraft_id": a.aircraft_id, "phase": a.phase,
            "cell": list(a.cell), "members": list(ids), "group_size": len(ids),
            "group_bad_fraction": fraction, "exposure": a.exposure if a.corridor else None,
            "history_count": len(a.history) if a.corridor else 0,
            "policy": a.policy, "policy_source": "observed" if a.corridor else "connector_not_assessed",
            "sinr_db": signals[a.aircraft_id],
            "serving_bs": int(values["serving_bs"][i]) if "serving_bs" in values else None,
            "rsrp_dbm": float(values["serving_rsrp_dbm"][i]) if "serving_rsrp_dbm" in values else None,
        })
    return rows


def commands(fleet, t, layout, cfg, mode):
    if mode not in ("acc", "fixed_cruise"):
        raise ValueError("unknown longitudinal mode")
    result = {}
    poses = {a.aircraft_id: a.pose(t, layout) for a in fleet if a.phase != "COMPLETE"}
    for a in fleet:
        if a.phase == "COMPLETE":
            continue
        if a.connector:
            raw = cmd = float(poses[a.aircraft_id][2][0])
            leaders = []
        else:
            leaders = []
            for cell in sorted(a.cells()):
                ahead = [b for b in fleet if b is not a and b.phase != "COMPLETE"
                         and cell in b.cells() and poses[b.aircraft_id][0][0] > a.s_m+1e-9]
                if ahead:
                    b = min(ahead, key=lambda b: poses[b.aircraft_id][0][0])
                    if b.aircraft_id not in [x[0] for x in leaders]:
                        gap = float(poses[b.aircraft_id][0][0]-a.s_m)
                        relative = float(poses[b.aircraft_id][1][0]-a.v_mps)
                        leaders.append((b.aircraft_id, gap, relative))
            free = cfg.k_speed*(cfg.cruise_mps-a.v_mps)
            raw = min([free]+[cfg.k_gap*(g-cfg.spacing(a.policy, a.v_mps))+cfg.k_relative*r
                              for _, g, r in leaders])
            cmd = 0. if mode == "fixed_cruise" else float(np.clip(
                raw, -cfg.decel_limit_mps2, cfg.accel_limit_mps2))
            # At an attained speed bound, further outward acceleration is zero.
            if a.v_mps <= 1e-10 and cmd < 0 or a.v_mps >= cfg.cruise_mps-1e-10 and cmd > 0:
                cmd = 0.
        result[a.aircraft_id] = {"raw": float(raw), "cmd": float(cmd), "leaders": leaders}
    return result


def new_insertion_deficits(fleet, before, after, cfg):
    """Check newly introduced follower/leader pairs, not inherited source gaps.

    Source/target occupancy still governs ACC throughout the maneuver. Existing
    policy-target deficits are tracked and propagated with bounded acceleration;
    they are not an instantaneous hard separation violation.
    """
    deficits = []
    for a in fleet:
        if not a.corridor:
            continue
        existing = {leader for leader, _, _ in before[a.aircraft_id]["leaders"]}
        desired = cfg.spacing(a.policy, a.v_mps)
        for leader, gap, _ in after[a.aircraft_id]["leaders"]:
            if leader not in existing and gap < desired-1e-8:
                deficits.append({"follower": a.aircraft_id, "leader": leader,
                                 "gap_m": gap, "desired_gap_m": desired})
    return deficits


def interval_separation(coefficients, cfg):
    """All-airborne-pair interval screen, including internal polynomial extrema.

    Far pairs use a conservative displacement lower bound. The returned minimum
    is therefore a lower bound/estimate, not always the actual closest distance.
    """
    scale = np.array([cfg.horizontal_separation_m]*2+[cfg.vertical_separation_m])
    minimum = math.inf
    for (ida, ca), (idb, cb) in combinations(coefficients.items(), 2):
        delta = (ca-cb)/scale
        reach = np.abs(delta[1:]).sum(axis=0)
        bound = float(np.sum(np.maximum(0, np.abs(delta[0])-reach)**2))
        if bound >= 1+1e-8:
            minimum = min(minimum, bound)
            continue
        squared = np.zeros(11)
        for axis in range(3):
            term = poly.polymul(delta[:, axis], delta[:, axis])
            squared[:len(term)] += term
        times = [0., 1., *real_roots(poly.polyder(squared))]
        vals = poly.polyval(times, squared)
        k = int(np.argmin(vals))
        score = float(vals[k])
        minimum = min(minimum, score)
        if score < 1-1e-7:
            return minimum, {"reason": "three_dimensional_interval_conflict",
                             "pair": [ida, idb], "interval_fraction": times[k], "score": score}
    return minimum, None


def point_separation(fleet, t, layout, cfg):
    coefficients = {}
    for a in fleet:
        if a.phase != "COMPLETE":
            c = np.zeros((6, 3))
            c[0] = a.pose(t, layout)[0]
            coefficients[a.aircraft_id] = c
    return interval_separation(coefficients, cfg)


def connector_for(a, t, layout, cfg, departure):
    if departure:
        curve = Quintic.boundary(t, cfg.connector_s, [0, 0, 0],
            np.r_[cfg.entry_m, layout.position(a.cell)], [0, 0, 0], [cfg.cruise_mps, 0, 0])
    else:
        p, v, acc = a.pose(t, layout)
        curve = Quintic.boundary(t, cfg.connector_s, p, [cfg.destination_m, 0, 0],
                                v, [0, 0, 0], acc, [0, 0, 0])
    if not curve.feasible(cfg):
        raise ValueError("connector violates declared component limits or monotone progress")
    return curve


@dataclass(frozen=True)
class Request:
    aircraft_id: str
    time_s: float
    cell: tuple[int, int]


class Simulator:
    """Shared execution/forecast event loop. Forecasts never make future lane decisions."""
    def __init__(self, layout, cfg, radio, mode="acc", *, initial=(), requests=()):
        self.layout, self.cfg, self.radio, self.mode = layout, cfg, radio, mode
        self.fleet = deepcopy(list(initial))
        self.pending = sorted(list(requests), key=lambda r: (r.time_s, r.aircraft_id))
        self.t = 0.
        self.events, self.observations, self.trace = [], [], []
        self.issue = None
        self.minimum_separation = math.inf
        self.completed_moves = 0
        self.last_policy = -math.inf
        self.next_release_check = 0.
        self.predicting = False
        self.command_history = {}
        self.control_rows = []
        self.entry_gap_checks = {}
        self.leader_history = {}
        self.deferrals = {}
        self.accepted_moves = 0
        self.total_count = len(self.fleet)+len(self.pending)
        self._validate()

    def _validate(self):
        if self.mode not in ("acc", "fixed_cruise"):
            raise ValueError("unknown mode")
        ids = [a.aircraft_id for a in self.fleet]+[r.aircraft_id for r in self.pending]
        if len(ids) != len(set(ids)):
            raise ValueError("duplicate aircraft IDs")
        for a in self.fleet:
            self.layout.position(a.cell)
            if a.phase != "CORRIDOR" or a.maneuver or a.connector or not np.isfinite([a.s_m, a.v_mps]).all():
                raise ValueError("initial fleet must be finite stationary-flow corridor states")
            if not self.cfg.entry_m <= a.s_m < self.cfg.exit_m or not 0 <= a.v_mps <= self.cfg.cruise_mps:
                raise ValueError("initial state outside corridor or speed bounds")
            if self.mode == "fixed_cruise" and abs(a.v_mps-self.cfg.cruise_mps) > 1e-9:
                raise ValueError("fixed-cruise initialization must match cruise speed")
            if a.policy not in {"C", "R", "F"}:
                raise ValueError("invalid policy")
            a.entry_s = 0.
        for r in self.pending:
            if not np.isfinite(r.time_s) or r.time_s < 0:
                raise ValueError("invalid requested release time")
            self.layout.position(r.cell)
            # Reject structurally impossible connector settings before starting.
            connector_for(Vehicle(r.aircraft_id, 0, 0, r.cell), 0, self.layout, self.cfg, True)

    def clone(self):
        clone = object.__new__(Simulator)
        clone.__dict__ = dict(self.__dict__)
        clone.fleet = deepcopy(self.fleet)
        clone.pending = list(self.pending)
        clone.events, clone.observations, clone.trace, clone.control_rows = [], [], [], []
        clone.command_history = {}
        clone.entry_gap_checks = {}
        clone.leader_history = {}
        clone.deferrals = dict(self.deferrals)
        clone.predicting = True
        return clone

    def settle(self):
        entrants = []
        for a in self.fleet:
            if a.maneuver and self.t >= a.maneuver.start_s+a.maneuver.duration_s-1e-8:
                a.cell = a.maneuver.target
                a.cooldown_until = self.t+self.cfg.cooldown_s
                a.maneuver = None
                self.completed_moves += 1
                self.events.append({"t_s": self.t, "aircraft_id": a.aircraft_id, "status": "transition_completed"})
            if a.connector and self.t >= a.connector.start+a.connector.duration-1e-8:
                if a.phase == "DEPARTURE_CONNECTOR":
                    a.s_m, a.v_mps = self.cfg.entry_m, self.cfg.cruise_mps
                    a.connector, a.phase, a.entry_s = None, "CORRIDOR", self.t
                    a.last_acceleration = 0.
                    entrants.append(a.aircraft_id)
                    # Start C without fabricating radio/group history.
                    self.events.append({"t_s": self.t, "aircraft_id": a.aircraft_id, "status": "corridor_entry"})
                else:
                    a.phase, a.completion_s = "COMPLETE", self.t
                    a.connector = None
                    a.s_m, a.v_mps = self.cfg.destination_m, 0.
                    self.events.append({"t_s": self.t, "aircraft_id": a.aircraft_id, "status": "mission_complete"})
            if a.corridor and a.s_m >= self.cfg.exit_m-1e-7:
                if a.maneuver:
                    self.issue = {"reason": "unfinished_transition_at_exit", "aircraft_id": a.aircraft_id}
                    return
                try:
                    a.connector = connector_for(a, self.t, self.layout, self.cfg, False)
                except ValueError as exc:
                    self.issue = {"reason": "arrival_connector_infeasible", "aircraft_id": a.aircraft_id,
                                  "detail": str(exc)}
                    return
                a.phase, a.exit_s = "ARRIVAL_CONNECTOR", self.t
                self.events.append({"t_s": self.t, "aircraft_id": a.aircraft_id, "status": "corridor_exit"})
        if entrants:
            controls = commands(self.fleet, self.t, self.layout, self.cfg, self.mode)
            for entrant in entrants:
                self.entry_gap_checks[entrant] = not any(
                    gap < self.cfg.spacing(a.policy, a.v_mps)-1e-8
                    for a in self.fleet if a.corridor
                    for leader, gap, _ in controls[a.aircraft_id]["leaders"]
                    if a.aircraft_id == entrant or leader == entrant)

    def policy_tick(self):
        k = round(self.t/self.cfg.policy_s)
        if abs(self.t-k*self.cfg.policy_s) < 1e-7 and self.t > self.last_policy+1e-8:
            self.observations.extend(observe(self.fleet, self.t, self.layout, self.cfg, self.radio))
            self.last_policy = self.t

    def advance(self, end):
        """Advance at most one motion interval, split at all physical events."""
        cfg, t = self.cfg, self.t
        controls = commands(self.fleet, t, self.layout, cfg, self.mode)
        next_grid = (math.floor((t+1e-8)/cfg.dt_s)+1)*cfg.dt_s
        dt = min(end-t, next_grid-t)
        for a in self.fleet:
            if a.phase == "COMPLETE":
                continue
            if a.connector:
                dt = min(dt, a.connector.start+a.connector.duration-t)
            elif a.corridor:
                cmd = controls[a.aircraft_id]["cmd"]
                if cmd > 1e-12 and a.v_mps < cfg.cruise_mps-1e-9:
                    dt = min(dt, (cfg.cruise_mps-a.v_mps)/cmd)
                if cmd < -1e-12 and a.v_mps > 1e-9:
                    dt = min(dt, -a.v_mps/cmd)
                distance = cfg.exit_m-a.s_m
                if distance > 1e-7:
                    if abs(cmd) < 1e-12:
                        crossing = distance/a.v_mps if a.v_mps > 0 else math.inf
                    else:
                        disc = a.v_mps*a.v_mps+2*cmd*distance
                        crossing = 2*distance/(a.v_mps+math.sqrt(disc)) if disc >= 0 else math.inf
                    if crossing > 1e-8:
                        dt = min(dt, crossing)
            if a.maneuver:
                dt = min(dt, a.maneuver.start_s+a.maneuver.duration_s-t)
        if dt <= 1e-9:
            raise RuntimeError("nonadvancing event loop")
        coefficients = {}
        for a in self.fleet:
            if a.phase == "COMPLETE":
                continue
            ctrl = controls[a.aircraft_id]
            coefficients[a.aircraft_id] = a.interval(t, dt, ctrl["cmd"], self.layout)
            self.control_rows.append((t, t+dt, a.aircraft_id, ctrl["cmd"], ctrl["raw"]))
            self.leader_history[(a.aircraft_id, round(t, 8))] = tuple(x[0] for x in ctrl["leaders"])
        bound, issue = interval_separation(coefficients, cfg)
        self.minimum_separation = min(self.minimum_separation, bound)
        if issue:
            self.issue = {**issue, "t_s": t+dt*issue["interval_fraction"]}
            return  # retain the last valid state, not a post-conflict state
        if not self.predicting:
            for a in self.fleet:
                if a.phase == "COMPLETE":
                    continue
                p, vel, accel = a.pose(t, self.layout)
                ctrl = controls[a.aircraft_id]
                accel[0] = ctrl["cmd"]
                gap = min((x[1] for x in ctrl["leaders"]), default=None)
                self.trace.append({"t_s": t, "aircraft_id": a.aircraft_id, "phase": a.phase,
                    "position_m": p.tolist(), "velocity_mps": vel.tolist(),
                    "acceleration_mps2": accel.tolist(), "speed_3d_mps": float(np.linalg.norm(vel)),
                    "policy": a.policy, "policy_source": "observed" if a.corridor and a.history else
                    "initialization" if a.corridor else "connector_not_assessed",
                    "exposure": a.exposure if a.corridor else None, "cell": list(a.cell),
                    "target": list(a.maneuver.target) if a.maneuver else None,
                    "raw_acceleration": ctrl["raw"], "command_acceleration": ctrl["cmd"],
                    "leaders": ctrl["leaders"], "desired_gap_m": cfg.spacing(a.policy, a.v_mps) if a.corridor else None,
                    "actual_gap_m": gap, "spacing_deficit_m": max(0., cfg.spacing(a.policy, a.v_mps)-gap)
                    if a.corridor and gap is not None else None})
        for a in self.fleet:
            if a.phase == "COMPLETE":
                continue
            if a.connector:
                p, v, _ = a.connector.sample(t+dt)
                a.s_m, a.v_mps = float(p[0]), float(v[0])
            else:
                cmd = controls[a.aircraft_id]["cmd"]
                a.s_m += a.v_mps*dt+.5*cmd*dt**2
                a.v_mps = float(np.clip(a.v_mps+cmd*dt, 0, cfg.cruise_mps))
                a.last_acceleration = cmd
        self.t = float(t+dt)
        self.settle()

    def preview(self, end, *, allow_releases=True):
        trial = self.clone()
        while trial.t < end-1e-8 and not trial.issue:
            trial.settle()
            if allow_releases:
                trial.release_due()
            trial.policy_tick()
            trial.advance(end)
        if not trial.issue:
            trial.settle()
            trial.policy_tick()
        return trial

    def release_due(self):
        # Forecast admission previews suppress *further* release recursion.
        if self.t < self.next_release_check-1e-8:
            return
        self.next_release_check = self.t+self.cfg.dt_s
        for request in list(self.pending):
            if request.time_s > self.t+1e-8:
                break
            a = Vehicle(request.aircraft_id, 0, 0, request.cell, phase="DEPARTURE_CONNECTOR",
                        request_s=request.time_s, release_s=self.t)
            a.connector = connector_for(a, self.t, self.layout, self.cfg, True)
            immediate = point_separation([*self.fleet, a], self.t, self.layout, self.cfg)[1]
            if immediate:
                self.deferrals[request.aircraft_id] = self.deferrals.get(request.aircraft_id, 0)+1
                break  # declared FCFS boundary admission, no overtaking
            trial = self.clone()
            trial.fleet.append(a)
            trial.pending.remove(request)
            horizon = self.t+self.cfg.connector_s+self.cfg.window_s
            trial = trial.preview(horizon, allow_releases=False)
            issue = trial.issue
            if not issue and not trial.entry_gap_checks.get(a.aircraft_id, False):
                issue = {"reason": "entry_spacing_not_met"}
            if issue:
                self.deferrals[request.aircraft_id] = self.deferrals.get(request.aircraft_id, 0)+1
                break
            self.fleet.append(a)
            self.pending.remove(request)
            self.events.append({"t_s": self.t, "aircraft_id": a.aircraft_id,
                                "status": "released", "request_s": request.time_s})

    def choose(self):
        cfg = self.cfg
        for own in sorted(self.fleet, key=lambda a: a.aircraft_id):
            if not own.corridor or own.maneuver or self.t < own.cooldown_until-1e-8:
                continue
            event = {"t_s": self.t, "aircraft_id": own.aircraft_id}
            # Avoid a forecast crossing the focal's corridor exit in either branch.
            if own.s_m+cfg.cruise_mps*cfg.horizon_s >= cfg.exit_m-1e-8:
                continue
            baseline = self.preview(self.t+cfg.horizon_s)
            if baseline.issue:
                self.events.append({**event, "status": "baseline_forecast_infeasible", "detail": baseline.issue})
                continue
            current_controls = commands(self.fleet, self.t, self.layout, cfg, self.mode)
            scored = []
            for target in self.layout.neighbors(own.cell):
                maneuver = make_maneuver(own.cell, target, self.t, self.layout, cfg)
                maneuver = replace(maneuver, duration_s=math.ceil(maneuver.duration_s/cfg.dt_s)*cfg.dt_s)
                if maneuver.duration_s+cfg.window_s > cfg.horizon_s+1e-8:
                    self.events.append({**event, "target": list(target), "status": "insufficient_horizon"})
                    continue
                trial = self.clone()
                candidate = next(a for a in trial.fleet if a.aircraft_id == own.aircraft_id)
                candidate.maneuver = maneuver
                # Only NEW following relationships must satisfy insertion targets.
                # Inherited source gaps remain in ACC and the physical forecast.
                controls = commands(trial.fleet, self.t, self.layout, cfg, self.mode)
                deficit = new_insertion_deficits(trial.fleet, current_controls, controls, cfg)
                if deficit:
                    self.events.append({**event, "target": list(target), "status": "insertion_target_gap_rejected",
                                        "affected": sorted({d["follower"] for d in deficit}),
                                        "new_relationship_deficits": deficit})
                    continue
                trial = trial.preview(self.t+cfg.horizon_s)
                if trial.issue:
                    self.events.append({**event, "target": list(target), "status": "candidate_infeasible", "detail": trial.issue})
                    continue
                score = compare_predictions(baseline, trial, own.aircraft_id, cfg)
                if score is not None and score["braking_violation"] is not None:
                    self.events.append({**event, "target": list(target),
                                        "status": "induced_command_braking_rejected",
                                        "detail": score["braking_violation"]})
                    continue
                if score is None or score["exposure_benefit"] < cfg.benefit-1e-10 or score["spacing_benefit_m"] < -1e-8:
                    self.events.append({**event, "target": list(target), "status": "communication_gate_rejected", "score": score})
                    continue
                scored.append(((-score["spacing_benefit_m"], -score["exposure_benefit"],
                                score["extra_braking_mps"], maneuver.duration_s, target), maneuver, score))
            if scored:
                _, maneuver, score = min(scored, key=lambda x: x[0])
                own.maneuver = maneuver
                self.accepted_moves += 1
                self.events.append({**event, "status": "transition_accepted", "target": list(maneuver.target),
                                    "source": list(maneuver.source), "duration_s": maneuver.duration_s, "score": score})

    def run(self, transitions=False):
        _, self.issue = point_separation(self.fleet, self.t, self.layout, self.cfg)
        while self.t < self.cfg.duration_s-1e-8 and not self.issue:
            self.settle()
            self.release_due()
            self.policy_tick()
            if transitions and abs(self.t/self.cfg.decision_s-round(self.t/self.cfg.decision_s)) < 1e-8:
                self.choose()
            self.advance(self.cfg.duration_s)
            if not self.pending and all(a.phase == "COMPLETE" for a in self.fleet):
                break
        if not self.issue:
            self.policy_tick()
        assert len(self.pending)+len(self.fleet) == self.total_count
        return self


def compare_predictions(baseline, candidate, own_id, cfg):
    """Paired actual group-exposure predictions, not personal outage samples."""
    def index(sim):
        return {(r["aircraft_id"], round(r["t_s"], 8)): r for r in sim.observations if r["phase"] == "CORRIDOR"}
    before, after = index(baseline), index(candidate)
    common = before.keys() & after.keys()
    own = sorted(k for k in common if k[0] == own_id)
    if not own:
        return None
    affected = {own_id}
    for key in baseline.leader_history.keys() & candidate.leader_history.keys():
        if baseline.leader_history[key] != candidate.leader_history[key]:
            affected.add(key[0])
    for key in common:
        if before[key]["members"] != after[key]["members"] or before[key]["policy"] != after[key]["policy"]:
            affected.add(key[0])
    exposure = float(np.mean([before[k]["exposure"]-after[k]["exposure"] for k in own]))
    per_aircraft = {}
    for aircraft_id in sorted(affected):
        keys = [k for k in common if k[0] == aircraft_id]
        if keys:
            per_aircraft[aircraft_id] = float(np.mean([
                cfg.spacing(before[k]["policy"], cfg.cruise_mps)-cfg.spacing(after[k]["policy"], cfg.cruise_mps) for k in keys]))
    # Exact overlap integral for piecewise-constant commands on different event grids.
    extra = 0.
    braking_violation = None
    for aircraft_id in sorted(affected):
        left = [r for r in baseline.control_rows if r[2] == aircraft_id]
        right = [r for r in candidate.control_rows if r[2] == aircraft_id]
        i = j = 0
        while i < len(left) and j < len(right):
            a, b = left[i], right[j]
            duration = max(0., min(a[1], b[1])-max(a[0], b[0]))
            stay_braking, change_braking = max(0., -a[3]), max(0., -b[3])
            if aircraft_id != own_id:
                extra += duration*max(0., change_braking-stay_braking)
            # A saturated feedback demand is not a physical braking requirement.
            # The optional stricter voluntary-braking limit applies to EXECUTED
            # commands only when the candidate makes braking worse than staying.
            if (duration > 1e-10 and braking_violation is None
                    and change_braking > cfg.accept_braking_mps2+1e-8
                    and change_braking > stay_braking+1e-8):
                braking_violation = {"aircraft_id": aircraft_id,
                    "start_s": max(a[0], b[0]), "end_s": min(a[1], b[1]),
                    "stay_command_mps2": a[3], "change_command_mps2": b[3],
                    "accept_braking_mps2": cfg.accept_braking_mps2}
            if a[1] <= b[1]+1e-9:
                i += 1
            else:
                j += 1
    return {"exposure_benefit": exposure,
            "spacing_benefit_m": float(np.mean(list(per_aircraft.values()))),
            "extra_braking_mps": extra, "individual_spacing_benefits_m": per_aircraft,
            "braking_violation": braking_violation,
            "affected_ids": sorted(affected),
            "paired_focal_samples": len(own),
            "unpaired_samples": len(before.keys() ^ after.keys())}
