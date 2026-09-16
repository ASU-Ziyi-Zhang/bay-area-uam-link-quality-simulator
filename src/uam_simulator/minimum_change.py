"""Degradation-triggered, minimum-displacement singleton planning.

The 976 m radius is a provisional geometric benchmark, not aircraft performance.
The original gain-priority planner remains the default in lateral_study.
"""
from dataclasses import dataclass
import math

import numpy as np
from numpy.polynomial import Polynomial as Poly

from .lateral_study import maneuver_duration, offset_values, rk4_progress
from .path_design import _unit_roots


@dataclass(frozen=True)
class MinimumChangeConfig:
    minimum_radius_m: float = 976.
    candidate_spacing_m: float = 100.
    duration_search_step_s: float = 5.

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError("positive finite planning parameters required")


class CurvatureEnvelope:
    """Reference curvature extrema at polynomial stationary roots, not samples."""
    def __init__(self, frame):
        self.frame = frame
        points = []
        for index, segment in enumerate(frame.path.coefficients):
            scale = max(float(np.abs(segment[1:]).max()), 1.)
            dx = Poly(np.r_[0, segment[1:, 0] / scale]).deriv()
            dy = Poly(np.r_[0, segment[1:, 1] / scale]).deriv()
            speed2 = dx * dx + dy * dy
            cross = dx * dy.deriv() - dy * dx.deriv()
            stationary = 2 * cross.deriv() * speed2 - 3 * cross * speed2.deriv()
            points.extend((index + u) * frame.step for u in [0., 1., *_unit_roots(stationary)])
        self.q = np.unique(np.clip(points, 0., frame.length_m))
        self.k = frame.frame(self.q)[-1]

    def bounds(self, lo, hi):
        lo, hi = np.clip([lo, hi], 0., self.frame.length_m)
        k = np.r_[self.k[(self.q >= lo) & (self.q <= hi)], self.frame.frame(np.array([lo, hi]))[-1]]
        return float(k.min()), float(k.max())

    def offset_bound(self, lo, hi, offsets):
        k = np.asarray(self.bounds(lo, hi))[:, None]
        factor = 1 - k * np.asarray(offsets)[None, :]
        if np.any(factor <= .05):
            return math.inf
        return float(np.max(np.abs(k / factor)))

    def maneuver_bound(self, lo, hi, source, target, duration, speed):
        geometric = self.offset_bound(lo, hi, [source, target])
        quintic_acceleration = 10 / np.sqrt(3) * abs(target - source) / duration**2
        return geometric + quintic_acceleration / speed**2


def first_stable_c(times, coordinated, window):
    """Start of first sampled continuous-C episode spanning at least window."""
    start = None
    for time, good in zip(times, coordinated):
        if not good:
            start = None
        elif start is None:
            start = float(time)
        if start is not None and time - start >= window - 1e-8:
            return start
    return None


def completed_recovery(times, coordinated, completion, window):
    """C must persist through a full sampled window following completion."""
    post = np.flatnonzero(times >= completion - 1e-8)
    if not len(post):
        return None
    first = post[0]
    ends = np.flatnonzero(times >= times[first] + window - 1e-8)
    if not len(ends):
        return None
    end = ends[0]
    if not coordinated[first:end+1].all():
        return None
    start = first
    while start > 0 and coordinated[start-1]:
        start -= 1
    return float(times[start])


class MinimumChangePlanner:
    def __init__(self, frame, settings=MinimumChangeConfig()):
        self.settings = settings
        self.curvature = CurvatureEnvelope(frame)

    def evaluate(self, frame, radio, cfg, *, q, t, source, target, durations, history):
        """Forecast staying plus the duration alternatives for one endpoint."""
        durations = np.r_[0., durations]
        targets = np.r_[source, np.full(len(durations)-1, target)]
        state = np.full(len(targets), q, float)
        clock = float(t)
        flags = [np.full(len(targets), bad, float) for _, bad in history]
        flag_times = [u for u, _ in history]
        good_c = [np.full(len(targets), np.mean(flags) <= cfg.exposure_c + 1e-12)]
        active = [np.ones(len(targets), bool)]
        times = [clock]
        before_completion = np.full(len(targets), q)
        after_completion = np.full(len(targets), np.nan)
        while clock < t + cfg.horizon_s - 1e-8:
            next_policy = (math.floor((clock+1e-8)/cfg.policy_s)+1)*cfg.policy_s
            step = min(cfg.forecast_dt_s, t+cfg.horizon_s-clock, next_policy-clock)
            prior = state.copy()
            state = rk4_progress(frame, state, clock, step,
                lambda time: offset_values(time,t,source,targets,durations)[0], cfg.cruise_mps)
            previous_time = clock
            clock += step
            crossed = (previous_time <= t+durations+1e-8) & (clock >= t+durations-1e-8) & np.isnan(after_completion)
            before_completion[crossed] = prior[crossed]
            after_completion[crossed] = state[crossed]
            if abs(clock/cfg.policy_s-round(clock/cfg.policy_s)) > 1e-8:
                continue
            offsets = offset_values(clock,t,source,targets,durations)[0]
            inside = state < frame.length_m-1e-7
            values = radio(frame.xyz(np.minimum(state,frame.length_m), offsets, cfg.altitude_m))["sinr_db"]
            flags.append((values < cfg.threshold_db).astype(float))
            flag_times.append(clock)
            while flag_times and flag_times[0] < clock-cfg.window_s-1e-8:
                flag_times.pop(0); flags.pop(0)
            good_c.append(np.mean(flags, axis=0) <= cfg.exposure_c+1e-12)
            active.append(inside); times.append(clock)
            if not inside.any():
                break
        times, good_c, active = np.array(times), np.array(good_c), np.array(active)
        records = []
        for k in range(1, len(targets)):
            duration = float(durations[k])
            row = {"target_m": float(target), "duration_s": duration,
                   "planner": "minimum_change", "status": "insufficient_recovery_horizon"}
            common = active[:, 0] & active[:, k]
            paired = times[common]
            recovery = completed_recovery(paired, good_c[common,k], t+duration, cfg.window_s)
            stay = first_stable_c(paired, good_c[common,0], cfg.window_s)
            row.update(recovery_time_s=recovery, stay_recovery_time_s=stay)
            if recovery is None:
                row["status"] = "no_stable_c_after_completion"
            elif stay is not None and recovery > stay-cfg.policy_s+1e-8:
                row["status"] = "staying_recovers_no_later"
            else:
                upper = after_completion[k]
                if not np.isfinite(upper) or upper >= frame.length_m:
                    records.append(row)
                    continue
                moving_bound = self.curvature.maneuver_bound(q, upper, source, target, duration, cfg.cruise_mps)
                remaining_bound = self.curvature.offset_bound(before_completion[k], frame.length_m, [target])
                # Infinity is a rejection flag, never serialized as a JSON number.
                row.update(maneuver_curvature_bound_per_m=float(moving_bound) if np.isfinite(moving_bound) else None,
                    remaining_curvature_bound_per_m=float(remaining_bound) if np.isfinite(remaining_bound) else None,
                    minimum_radius_m=self.settings.minimum_radius_m)
                bound = max(moving_bound, remaining_bound)
                row["status"] = "eligible" if bound <= 1/self.settings.minimum_radius_m+1e-12 else "curvature_rejected"
            records.append(row)
        return records

    def candidates(self, frame, radio, cfg, *, q, t, source, targets, history, policy):
        if policy == "C":
            return [{"target_m":float(source), "duration_s":0., "status":"policy_satisfied_hold",
                     "planner":"minimum_change"}]
        spacing = self.settings.candidate_spacing_m
        if any(abs((target-source)/spacing-round((target-source)/spacing)) > 1e-8 for target in targets):
            raise ValueError("candidate displacements must use the configured grid")
        records = []
        for distance in sorted({abs(target-source) for target in targets}):
            shell = []
            for target in sorted(target for target in targets if abs(abs(target-source)-distance) < 1e-8):
                base = maneuver_duration(distance, cfg)
                durations = np.arange(base, cfg.horizon_s-cfg.window_s+1e-8, self.settings.duration_search_step_s)
                if not len(durations):
                    shell.append({"target_m":float(target),"duration_s":base,
                                  "planner":"minimum_change","status":"maneuver_exceeds_horizon"})
                    continue
                shell.extend(self.evaluate(frame,radio,cfg,q=q,t=t,source=source,target=target,
                                           durations=durations,history=history))
            records.extend(shell)
            if any(r["status"] == "eligible" for r in shell):
                break  # Farther endpoints cannot win the displacement-first objective.
        return records

    @staticmethod
    def choose(records, source):
        eligible = [r for r in records if r["status"] == "eligible"]
        return min(eligible,key=lambda r:(abs(r["target_m"]-source),r["duration_s"],
                                          r["recovery_time_s"],r["target_m"])) if eligible else None
