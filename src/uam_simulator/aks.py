"""Adaptive Kinematic Smoothing: two-phase (KS2) longitudinal transitions.

A spacing policy change alters the target gap without altering cruise speed. The
follower must therefore travel a different distance from the leader over the
transition while starting and finishing at the same speed. Writing the required
follower displacement as

    dx_req = v T + dS_real

separates it into the part a Newell-style translation already accounts for, `v T`,
and the part that does not appear there at all: `dS_real`, the change in the
spacing buffer. That buffer change has to be transported, and this module is the
analytical description of the transport.

The follower keeps following the leader throughout. What the reference adds is a
planned shape for the transition between two spacings, so the gap trajectory is
constructed rather than left to whatever the feedback loop happens to do. The
tracking interface compares the actual gap to the leader against a planned gap
derived from the leader forecast, so the leader never leaves the loop.

Single-phase members cannot do this job. KS1 and KS1A are transitions between two
endpoint speeds; when those speeds are equal they reduce to constant speed and
close only `v T`, leaving `dS_real` unrealised. The two-phase form supplies a
temporary excursion whose area is exactly the buffer change: a dip below cruise
opens a gap, an overshoot above cruise closes one.

Sign convention follows the manuscript: `dS_real` is the signed *reduction* in
target gap. Closing a gap has `dS_real > 0` and needs an overshoot; opening a gap
has `dS_real < 0` and needs a dip.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def hermite(xi):
    """Cubic `h = 3xi^2 - 2xi^3` with zero endpoint slopes, and derivatives.

    Returns `(h, dh/dxi, d2h/dxi2)`. `h(0) = 0`, `h(1) = 1`, `h'(0) = h'(1) = 0`,
    so a phase joins its neighbours with continuous speed and zero acceleration.
    """
    xi = np.clip(np.asarray(xi, float), 0.0, 1.0)
    return (3 * xi ** 2 - 2 * xi ** 3,
            6 * xi * (1 - xi),
            6 - 12 * xi)


def junction_speed(v1, v2, duration, displacement, alpha):
    """Junction speed from displacement closure, manuscript Eq. (13).

        v_J(alpha) = 2 dx_req / T - alpha v1 - (1 - alpha) v2
    """
    if duration <= 0:
        raise ValueError("duration must be positive")
    if not 0.0 < alpha < 1.0:
        raise ValueError("phase split must lie strictly inside (0, 1)")
    return 2 * displacement / duration - alpha * v1 - (1 - alpha) * v2


def equal_speed_junction(speed, target_reduction, duration):
    """Junction speed for equal endpoint speeds, manuscript Eq. (22).

        v_J = v + 2 dS_real / T

    Independent of the phase split, which is why the split is then free to be
    chosen against the motion limits rather than against displacement closure.
    """
    if duration <= 0:
        raise ValueError("duration must be positive")
    return speed + 2 * target_reduction / duration


@dataclass(frozen=True)
class KS2Transition:
    """Two cubic phases joined at a junction speed.

    Velocity and acceleration are continuous at the junction because both phases
    carry zero endpoint slope. Jerk is generally one-sided there, and steps from
    zero at the start and end of the transition.
    """

    v1: float
    v2: float
    junction: float
    duration: float
    alpha: float

    def __post_init__(self):
        if self.duration <= 0:
            raise ValueError("duration must be positive")
        if not 0.0 < self.alpha < 1.0:
            raise ValueError("phase split must lie strictly inside (0, 1)")
        for name in ("v1", "v2", "junction"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")

    @property
    def phase_durations(self):
        return self.alpha * self.duration, (1 - self.alpha) * self.duration

    def sample(self, t):
        """Speed, acceleration and displacement at time `t` from the start."""
        t = np.asarray(t, float)
        t1, t2 = self.phase_durations
        first = t <= t1

        xi1 = np.clip(t / t1, 0.0, 1.0)
        h1, dh1, ddh1 = hermite(xi1)
        d1 = self.junction - self.v1
        v_a = self.v1 + d1 * h1
        a_a = d1 * dh1 / t1
        # Integral of the cubic: t*v1 + dv*T*(xi^3 - xi^4/2).
        l_a = self.v1 * np.minimum(t, t1) + d1 * t1 * (xi1 ** 3 - xi1 ** 4 / 2)

        first_total = (self.v1 + self.junction) * t1 / 2
        xi2 = np.clip((t - t1) / t2, 0.0, 1.0)
        h2, dh2, ddh2 = hermite(xi2)
        d2 = self.v2 - self.junction
        v_b = self.junction + d2 * h2
        a_b = d2 * dh2 / t2
        l_b = first_total + self.junction * np.clip(t - t1, 0.0, t2) \
            + d2 * t2 * (xi2 ** 3 - xi2 ** 4 / 2)

        return (np.where(first, v_a, v_b),
                np.where(first, a_a, a_b),
                np.where(first, l_a, l_b))

    def displacement(self):
        """Total follower travel, manuscript Eq. (13)."""
        t1, t2 = self.phase_durations
        return t1 * (self.v1 + self.junction) / 2 + t2 * (self.junction + self.v2) / 2

    def peak_accelerations(self):
        """Peak magnitude in each phase; a cubic peaks at `3|dv| / (2 T_phase)`."""
        t1, t2 = self.phase_durations
        return (1.5 * abs(self.junction - self.v1) / t1,
                1.5 * abs(self.v2 - self.junction) / t2)

    def peak_jerk(self):
        """Largest one-sided jerk magnitude; a cubic phase peaks at its ends."""
        t1, t2 = self.phase_durations
        return max(6 * abs(self.junction - self.v1) / t1 ** 2,
                   6 * abs(self.v2 - self.junction) / t2 ** 2)


def optimal_split(target_reduction, duration, accel_limit, decel_limit,
                  jerk_limit=None):
    """Phase split that balances the two limits, and whether one exists.

    With equal endpoint speeds each phase carries a speed increment of magnitude
    `2 |dS_real| / T`, so a cubic phase of duration `beta T` peaks at
    `3 |dS_real| / (beta T^2)`. Opening a gap decelerates first and accelerates
    back, so phase one is bounded by the braking limit and phase two by the
    acceleration limit; closing a gap exchanges them.

    Requiring both phases to fit gives

        alpha >= 3|dS| / (first_limit T^2),  1 - alpha >= 3|dS| / (second_limit T^2)

    which is feasible exactly when

        T^2 >= 3|dS| (1/accel_limit + 1/decel_limit)

    The equal-limit shortcut `alpha = 1/2` is optimal only when the two limits
    are equal, and is optimistic otherwise.
    """
    for name, value in (("duration", duration), ("accel_limit", accel_limit),
                        ("decel_limit", decel_limit)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(target_reduction):
        raise ValueError("target_reduction must be finite")
    if jerk_limit is not None and (not math.isfinite(jerk_limit) or jerk_limit <= 0):
        raise ValueError("jerk_limit must be finite and positive")
    magnitude = abs(float(target_reduction))
    if magnitude == 0.0:
        return 0.5, True
    if duration <= 0:
        raise ValueError("duration must be positive")
    opening = target_reduction < 0
    first_limit = decel_limit if opening else accel_limit
    second_limit = accel_limit if opening else decel_limit
    low = 3 * magnitude / (first_limit * duration ** 2)
    high = 1.0 - 3 * magnitude / (second_limit * duration ** 2)
    if jerk_limit is not None:
        jerk_fraction = math.sqrt(12 * magnitude / (jerk_limit * duration ** 3))
        low, high = max(low, jerk_fraction), min(high, 1 - jerk_fraction)
    if low > high + 1e-14:
        return float("nan"), False
    if low > high:  # Analytic equality rounded in opposite directions.
        low = high = (low + high) / 2
    # Any split in [low, high] satisfies both. The split that equalises the two
    # normalised demands follows from letting both bounds bind at once:
    #   1 = 3|dS|/T^2 (1/a1 + 1/a2)  =>  3|dS|/T^2 = a1 a2/(a1 + a2)
    #   alpha = [a1 a2/(a1 + a2)] / a1 = a2/(a1 + a2)
    # so the phase governed by the *larger* limit gets the *shorter* duration.
    balanced = second_limit / (first_limit + second_limit)
    return float(min(max(balanced, low), high)), True


def minimum_duration(target_reduction, speed, accel_limit, decel_limit,
                     jerk_limit=None, speed_max=None, speed_min=0.0):
    """Shortest transition duration satisfying motion and junction constraints.

    Uses analytic lower bounds, then solves the JOINT phase interval when jerk
    and unequal acceleration bounds compete. This is a minimum within the
    equal-endpoint, two-cubic family, not a global motion optimum.

    * acceleration, generalised to unequal limits as
      `T >= sqrt(3 |dS| (1/a_max + 1/b_max))`, which reduces to the manuscript's
      `sqrt(6 |dS| / a_max)` when the limits are equal;
    * jerk, `T >= (48 |dS| / j_max)^(1/3)` at a symmetric split;
    * the junction speed staying inside `[speed_min, speed_max]`.
    """
    optimal_split(target_reduction, 1.0, accel_limit, decel_limit, jerk_limit)
    if not math.isfinite(speed) or not math.isfinite(speed_min) or speed_min < 0:
        raise ValueError("invalid speed bounds")
    if speed < speed_min or (speed_max is not None and
            (not math.isfinite(speed_max) or speed_max < speed)):
        raise ValueError("endpoint speed is outside the speed bounds")
    magnitude = abs(float(target_reduction))
    if magnitude == 0.0:
        return {"duration_s": 0.0, "binding": "none", "bounds": {}}
    bounds = {"acceleration": math.sqrt(3 * magnitude * (1 / accel_limit + 1 / decel_limit))}
    if jerk_limit is not None:
        if not np.isfinite(jerk_limit) or jerk_limit <= 0:
            raise ValueError("jerk_limit must be finite and positive when set")
        bounds["jerk"] = (48 * magnitude / jerk_limit) ** (1 / 3)
    if target_reduction < 0:
        headroom = speed - speed_min
        bounds["junction_floor"] = 2 * magnitude / headroom if headroom > 0 else math.inf
    elif speed_max is not None:
        headroom = float(speed_max) - speed
        if headroom <= 0:
            bounds["junction_ceiling"] = float("inf")
        else:
            bounds["junction_ceiling"] = 2 * magnitude / headroom
    binding = max(bounds, key=bounds.get)
    duration = bounds[binding]
    if math.isfinite(duration) and not optimal_split(
            target_reduction, duration, accel_limit, decel_limit, jerk_limit)[1]:
        lo, hi = duration, duration * 2
        while not optimal_split(target_reduction, hi, accel_limit, decel_limit, jerk_limit)[1]:
            hi *= 2
        for _ in range(70):
            mid = (lo + hi) / 2
            if optimal_split(target_reduction, mid, accel_limit, decel_limit, jerk_limit)[1]:
                hi = mid
            else:
                lo = mid
        duration = hi
        bounds["joint_phase_constraints"] = duration
        binding = "joint_phase_constraints"
    return {"duration_s": float(duration), "binding": binding, "bounds": bounds}


def plan_equal_speed_transition(speed, target_reduction, duration, *,
                                accel_limit, decel_limit, jerk_limit=None,
                                speed_max=None, speed_min=0.0):
    """Build the KS2 transition for a policy change at constant cruise speed.

    Returns the transition plus the checks that decide whether it is usable, so a
    caller can report *why* a transition was refused rather than only that it was.
    """
    if duration <= 0:
        raise ValueError("duration must be positive")
    junction = equal_speed_junction(speed, target_reduction, duration)
    minimum_duration(target_reduction, speed, accel_limit, decel_limit,
                     jerk_limit, speed_max, speed_min)
    alpha, splittable = optimal_split(target_reduction, duration, accel_limit, decel_limit, jerk_limit)
    reasons = []
    if not splittable:
        reasons.append("no phase split satisfies acceleration and enabled jerk limits")
    if junction < 0:
        reasons.append("junction speed is negative")
    elif junction < speed_min - 1e-10:
        reasons.append("junction speed is below the minimum speed")
    if speed_max is not None and junction > speed_max + 1e-10:
        reasons.append("junction speed exceeds the maximum speed")
    if not reasons:
        transition = KS2Transition(speed, speed, junction, duration, alpha)
        peaks = transition.peak_accelerations()
        jerk = transition.peak_jerk()
        if jerk_limit is not None and jerk > jerk_limit + 1e-9:
            reasons.append("peak jerk exceeds the limit")
    else:
        transition, peaks, jerk = None, (float("nan"), float("nan")), float("nan")
    return {"transition": transition, "junction_speed_mps": float(junction),
            "phase_split": float(alpha), "peak_accelerations_mps2": peaks,
            "peak_jerk_mps3": float(jerk), "feasible": not reasons,
            "reasons": reasons,
            "displacement_m": float(transition.displacement()) if transition else float("nan"),
            "required_displacement_m": float(speed * duration + target_reduction)}


def reference_gap(transition, initial_gap, leader_displacement):
    """Planned gap trajectory, manuscript Eq. (14).

        s_ref(t) = s0 + l_L(t) - l_AKS(t)

    The reference is expressed relative to the leader, so the follower is still
    following it; the plan only fixes the shape of the approach.
    """
    leader_displacement = np.asarray(leader_displacement, float)
    return initial_gap + leader_displacement - np.asarray(transition, float)


def tracking_command(planned_accel, gap, reference_gap_value, planned_speed,
                     follower_speed, gain_gap, gain_speed):
    """Feedforward plus feedback command, manuscript Eq. (18).

        a_cmd = a_AKS + k_s (s - s_ref) + k_v (v_AKS - v_F)

    The gap error is measured against the actual leader gap. Feedforward supplies
    the planned motion; feedback corrects departures from it.
    """
    return (np.asarray(planned_accel, float)
            + gain_gap * (np.asarray(gap, float) - np.asarray(reference_gap_value, float))
            + gain_speed * (np.asarray(planned_speed, float) - np.asarray(follower_speed, float)))
