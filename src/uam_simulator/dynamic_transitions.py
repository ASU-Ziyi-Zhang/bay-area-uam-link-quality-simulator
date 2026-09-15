"""Local-straight-corridor mechanism prototype, NOT an operational UAM controller.

Longitudinal speed is ds/dt; total 3-D speed is recorded separately. A synthetic
radio callable is deliberately explicit. No GIS geometry, C/R/F controller or
capacity result is silently substituted into this local closed-fleet benchmark.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import combinations
import math

import numpy as np


@dataclass(frozen=True)
class TransitionConfig:
    dt_s: float = 0.5
    forecast_dt_s: float = 1.0
    duration_s: float = 240.0
    decision_interval_s: float = 10.0
    forecast_horizon_s: float = 240.0
    cooldown_s: float = 30.0
    cruise_mps: float = 50.0
    min_gap_m: float = 300.0
    horizontal_separation_m: float = 200.0
    vertical_separation_m: float = 100.0
    min_maneuver_s: float = 30.0
    lateral_speed_limit_mps: float = 8.0
    vertical_speed_limit_mps: float = 3.0
    lateral_accel_limit_mps2: float = 0.3
    vertical_accel_limit_mps2: float = 0.2
    accel_limit_mps2: float = 1.0
    decel_limit_mps2: float = 1.5
    acc_standstill_m: float = 200.0
    acc_headway_s: float = 15.0
    acc_speed_gain_per_s: float = 0.2
    acc_gap_gain_per_s2: float = 0.005
    acc_relative_gain_per_s: float = 0.4
    sinr_threshold_db: float = -1.5
    minimum_benefit_fraction: float = 0.05

    def __post_init__(self):
        for name, value in vars(self).items():
            if not np.isfinite(value) or (name != "sinr_threshold_db" and value <= 0):
                raise ValueError(f"invalid positive finite parameter: {name}")
        for period, step in [(self.duration_s, self.dt_s), (self.decision_interval_s, self.dt_s),
                             (self.forecast_horizon_s, self.forecast_dt_s)]:
            if not np.isclose(period / step, round(period / step), rtol=0, atol=1e-9):
                raise ValueError("clock periods must be integer multiples of their step")
        if self.minimum_benefit_fraction >= 1:
            raise ValueError("benefit fraction must be below one")


@dataclass(frozen=True)
class Layout:
    offsets_m: tuple[float, ...]
    heights_m: tuple[float, ...]

    def __post_init__(self):
        for values in [self.offsets_m, self.heights_m]:
            if not len(values) or not np.isfinite(values).all() or np.any(np.diff(values) <= 0):
                raise ValueError("layout coordinates must be finite and strictly increasing")
        if min(self.heights_m) <= 0:
            raise ValueError("heights must be positive")

    def position(self, cell):
        lane, level = cell
        if not (isinstance(lane, int) and isinstance(level, int)
                and 0 <= lane < len(self.offsets_m) and 0 <= level < len(self.heights_m)):
            raise ValueError("invalid lane/level index")
        return np.array([self.offsets_m[lane], self.heights_m[level]], dtype=float)

    def neighbors(self, cell):
        lane, level = cell
        return [(i, j) for i, j in [(lane - 1, level), (lane + 1, level),
                                    (lane, level - 1), (lane, level + 1)]
                if 0 <= i < len(self.offsets_m) and 0 <= j < len(self.heights_m)]


def smoothstep(u):
    """Quintic displacement and first two derivatives w.r.t. normalized time."""
    q = np.clip(np.asarray(u, dtype=float), 0, 1)
    return (q**3 * (10 - 15*q + 6*q*q),
            30*q*q*(1-q)**2, 60*q*(1-q)*(1-2*q))


@dataclass(frozen=True)
class Maneuver:
    source: tuple[int, int]
    target: tuple[int, int]
    start_s: float
    duration_s: float

    def __post_init__(self):
        if not np.isfinite([self.start_s, self.duration_s]).all() or self.duration_s <= 0:
            raise ValueError("finite start time and positive maneuver duration required")

    def sample(self, t, layout):
        start = layout.position(self.source)
        delta = layout.position(self.target) - start
        p, velocity, acceleration = smoothstep((np.asarray(t) - self.start_s) / self.duration_s)
        return (start + p[..., None]*delta,
                velocity[..., None]*delta/self.duration_s,
                acceleration[..., None]*delta/self.duration_s**2)


def make_maneuver(source, target, start_s, layout, config):
    if target not in layout.neighbors(source):
        raise ValueError("only adjacent axis-aligned transitions are allowed")
    delta = np.abs(layout.position(target) - layout.position(source))
    # Exact extrema of quintic smoothstep: max f'=15/8, max |f''|=10/sqrt(3).
    duration = max(config.min_maneuver_s,
        1.875*delta[0]/config.lateral_speed_limit_mps,
        1.875*delta[1]/config.vertical_speed_limit_mps,
        math.sqrt(10/math.sqrt(3)*delta[0]/config.lateral_accel_limit_mps2),
        math.sqrt(10/math.sqrt(3)*delta[1]/config.vertical_accel_limit_mps2))
    return Maneuver(source, target, float(start_s), float(duration))


@dataclass
class Aircraft:
    aircraft_id: str
    s_m: float
    v_mps: float
    cell: tuple[int, int]
    maneuver: Maneuver | None = None
    cooldown_until_s: float = 0.0

    def occupied_cells(self):
        # Conservative dual membership throughout a maneuver, for gaps/ACC.
        return {self.cell} if self.maneuver is None else {self.maneuver.source, self.maneuver.target}

    def pose(self, t, layout):
        if self.maneuver is None:
            yz, lateral_v, lateral_a = layout.position(self.cell), np.zeros(2), np.zeros(2)
        else:
            yz, lateral_v, lateral_a = self.maneuver.sample(t, layout)
        return np.r_[self.s_m, yz], np.r_[self.v_mps, lateral_v], lateral_a


def acc_acceleration(v, gap, leader_v, config):
    """Explicit ACC-like proportional controller, not IDM or a certified ACC law.

The lower of speed regulation and spacing regulation applies. Spacing feedback
is continuous and anticipates closing speed; accelerations are bounded, but jerk
is not. These gains and headway are exploratory, independent of C/R/F policy.
    """
    free = config.acc_speed_gain_per_s*(config.cruise_mps-v)
    desired_gap = max(config.min_gap_m, config.acc_standstill_m+config.acc_headway_s*v)
    follow = math.inf if gap is None else (
        config.acc_gap_gain_per_s2*(gap-desired_gap)
        + config.acc_relative_gain_per_s*(leader_v-v))
    return float(np.clip(min(free, follow), -config.decel_limit_mps2, config.accel_limit_mps2))


def fleet_accelerations(fleet, mode, config):
    if mode not in ("fixed_cruise", "acc"):
        raise ValueError("unknown longitudinal mode")
    accelerations = []
    for own in fleet:
        ahead = [other for other in fleet if other is not own and other.s_m > own.s_m
                 and own.occupied_cells() & other.occupied_cells()]
        leader = min(ahead, key=lambda a: a.s_m) if ahead else None
        accelerations.append(0.0 if mode == "fixed_cruise" else acc_acceleration(
            own.v_mps, None if leader is None else leader.s_m-own.s_m,
            None if leader is None else leader.v_mps, config))
    return np.asarray(accelerations)


def finish_maneuvers(fleet, t, config):
    for own in fleet:
        if own.maneuver and t >= own.maneuver.start_s + own.maneuver.duration_s - 1e-10:
            own.cell = own.maneuver.target
            own.cooldown_until_s = own.maneuver.start_s + own.maneuver.duration_s + config.cooldown_s
            own.maneuver = None


def advance(fleet, t, dt, mode, config):
    accelerations = fleet_accelerations(fleet, mode, config)
    for own, a in zip(fleet, accelerations):
        # Finite acceleration throughout a step; never snap velocity or move backwards.
        a = float(np.clip(a, -own.v_mps/dt, (config.cruise_mps-own.v_mps)/dt))
        own.s_m += own.v_mps*dt + 0.5*a*dt*dt
        own.v_mps += a*dt
    finish_maneuvers(fleet, t+dt, config)


def violation(fleet, t, layout, config):
    """Sampled engineering screen, not a continuous-time separation guarantee."""
    for a, b in combinations(fleet, 2):
        if a.occupied_cells() & b.occupied_cells() and abs(a.s_m-b.s_m) < config.min_gap_m-1e-9:
            return {"reason": "longitudinal_gap", "pair": [a.aircraft_id, b.aircraft_id]}
        delta = a.pose(t, layout)[0] - b.pose(t, layout)[0]
        normalized = ((delta[0]**2+delta[1]**2)/config.horizontal_separation_m**2
                      + delta[2]**2/config.vertical_separation_m**2)
        if normalized < 1-1e-9:
            return {"reason": "protected_ellipsoid", "pair": [a.aircraft_id, b.aircraft_id]}
    return None


def preview(fleet, t, mode, layout, config):
    """Predict existing maneuvers and ACC, with no future lane decisions or entries."""
    fleet = [replace(a) for a in fleet]
    times = t + np.arange(round(config.forecast_horizon_s/config.forecast_dt_s)+1)*config.forecast_dt_s
    poses = []
    for index, moment in enumerate(times):
        issue = violation(fleet, moment, layout, config)
        if issue:
            return None, {**issue, "forecast_time_s": float(moment)}
        poses.append([a.pose(moment, layout)[0] for a in fleet])
        if index < len(times)-1:
            advance(fleet, moment, config.forecast_dt_s, mode, config)
    return np.asarray(poses), None


def outage_fraction(radio, positions, threshold):
    values = np.asarray(radio(positions), dtype=float)
    if values.shape != positions.shape[:-1] or not np.isfinite(values).all():
        raise ValueError("radio must return one finite SINR per supplied position")
    # Uniform samples over the common future horizon, not full-route or fleet statistics.
    return float(np.mean(values < threshold))


def choose_transitions(fleet, t, mode, layout, config, radio):
    events = []
    # Serial arbitration: accepted maneuver becomes visible to the next decision.
    # ID ordering is deterministic and may be unfair; record it rather than hide it.
    for own in sorted(fleet, key=lambda a: a.aircraft_id):
        if own.maneuver or t < own.cooldown_until_s:
            continue
        index = fleet.index(own)
        baseline, baseline_issue = preview(fleet, t, mode, layout, config)
        if baseline is None:
            events.append({"t_s": t, "aircraft_id": own.aircraft_id,
                           "status": "baseline_forecast_unsafe", **baseline_issue})
            continue  # This is not an emergency-escape planner.
        before = outage_fraction(radio, baseline[:, index], config.sinr_threshold_db)
        candidates = []
        for target in layout.neighbors(own.cell):
            maneuver = make_maneuver(own.cell, target, t, layout, config)
            event = {"t_s": t, "aircraft_id": own.aircraft_id, "source": list(own.cell),
                     "target": list(target), "maneuver_duration_s": maneuver.duration_s}
            if maneuver.duration_s > config.forecast_horizon_s or t+maneuver.duration_s > config.duration_s:
                events.append({**event, "status": "insufficient_horizon"})
                continue
            hypothetical = [replace(a) for a in fleet]
            hypothetical[index].maneuver = maneuver
            predicted, issue = preview(hypothetical, t, mode, layout, config)
            if predicted is None:
                events.append({**event, "status": "gap_or_conflict_rejected", **issue})
                continue
            after = outage_fraction(radio, predicted[:, index], config.sinr_threshold_db)
            benefit = before-after
            event.update(before_outage_fraction=before, after_outage_fraction=after, benefit_fraction=benefit)
            if benefit < config.minimum_benefit_fraction:
                events.append({**event, "status": "insufficient_benefit"})
            else:
                candidates.append((benefit, maneuver, event))
        if candidates:
            best = max(candidates, key=lambda c: c[0])  # Stable tie break: layout neighbor order.
            own.maneuver = best[1]
            for candidate in candidates:
                events.append({**candidate[2], "status": "accepted" if candidate is best else "lower_ranked"})
    return events


def simulate(initial, layout, config, radio, *, mode, transitions):
    """Closed fleet on an unbounded local straight axis; no entry/exit/capacity model."""
    fleet = [replace(a) for a in initial]
    if not fleet or len({a.aircraft_id for a in fleet}) != len(fleet):
        raise ValueError("nonempty fleet with unique IDs required")
    if mode not in ("fixed_cruise", "acc"):
        raise ValueError("unknown longitudinal mode")
    for a in fleet:
        layout.position(a.cell)
        if not a.aircraft_id or not np.isfinite([a.s_m, a.v_mps]).all() or not 0 <= a.v_mps <= config.cruise_mps:
            raise ValueError("invalid initial aircraft state")
        if a.maneuver is not None:
            raise ValueError("initial maneuvers must be absent")
        if mode == "fixed_cruise" and not np.isclose(a.v_mps, config.cruise_mps):
            raise ValueError("fixed-cruise initialization must already have the prescribed speed")
    events, trace = [], []
    dt = config.dt_s
    failure = None
    initial_s = {a.aircraft_id: a.s_m for a in fleet}
    for step in range(round(config.duration_s/dt)+1):
        t = step*dt
        failure = violation(fleet, t, layout, config)
        if failure:
            events.append({"t_s": t, "status": "simulation_stopped", **failure})
        elif transitions and step % round(config.decision_interval_s/dt) == 0 and t < config.duration_s:
            events.extend(choose_transitions(fleet, t, mode, layout, config, radio))
        poses = np.asarray([a.pose(t, layout)[0] for a in fleet])
        sinr = np.asarray(radio(poses), dtype=float)
        if sinr.shape != (len(fleet),) or not np.isfinite(sinr).all():
            raise ValueError("invalid radio values")
        accelerations = fleet_accelerations(fleet, mode, config)
        for own, db, acceleration in zip(fleet, sinr, accelerations):
            position, velocity, lateral_a = own.pose(t, layout)
            trace.append({"t_s": t, "aircraft_id": own.aircraft_id,
                "s_m": own.s_m, "offset_m": float(position[1]), "altitude_m": float(position[2]),
                "v_longitudinal_mps": own.v_mps, "speed_3d_mps": float(np.linalg.norm(velocity)),
                "a_longitudinal_command_mps2": float(acceleration),
                "v_lateral_mps": float(velocity[1]), "v_vertical_mps": float(velocity[2]),
                "a_lateral_mps2": float(lateral_a[0]), "a_vertical_mps2": float(lateral_a[1]),
                "lane": own.cell[0], "level": own.cell[1], "in_transition": own.maneuver is not None,
                "sinr_db": float(db), "poor_link": bool(db < config.sinr_threshold_db)})
        if failure or step == round(config.duration_s/dt):
            break
        advance(fleet, t, dt, mode, config)
    end = trace[-1]["t_s"]
    accepted = [e for e in events if e["status"] == "accepted"]
    return {"trace": trace, "events": events, "summary": {
        "status": "stopped_on_violation" if failure else "completed_sampled_checks",
        "mode": mode, "transitions_enabled": transitions, "aircraft_count": len(fleet),
        "elapsed_s": end, "accepted_transitions": len(accepted),
        "completed_transitions": sum(e["t_s"]+e["maneuver_duration_s"] <= end+1e-9 for e in accepted),
        "poor_link_sample_fraction": float(np.mean([r["poor_link"] for r in trace])),
        "minimum_longitudinal_speed_mps": min(r["v_longitudinal_mps"] for r in trace),
        "maximum_total_speed_mps": max(r["speed_3d_mps"] for r in trace),
        "mean_progress_loss_m": float(np.mean([config.cruise_mps*end-(a.s_m-initial_s[a.aircraft_id]) for a in fleet])),
        "capacity_uam_h": None, "arrival_throughput_uam_h": None,
        "scientific_status": "local synthetic closed-fleet mechanism check, not Bay Area capacity or aviation safety validation",
        "prediction": "known static radio field; coupled longitudinal forecast; existing maneuvers only; no future decisions/entries",
        "separation_check": "sampled ellipsoid plus conservative dual-cell longitudinal gaps; no continuous safety proof",
    }}


def synthetic_radio(positions):
    """Deterministic software fixture, NOT measured or simulated Bay Area radio.

The right and upper cells have better SINR near a prescribed along-axis region.
This deliberately makes desired transitions available to exercise the controller.
"""
    p = np.asarray(positions, dtype=float)
    weak_region = np.exp(-((p[..., 0]-7000)/5000)**4)
    return 2.0 - 6.0*weak_region + 0.008*p[..., 1] + 0.012*(p[..., 2]-300)
