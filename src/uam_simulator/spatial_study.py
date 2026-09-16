"""Stage 2: isolated 3-D transverse motion, retaining the R0009 policy gate."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .lateral_study import LateralConfig, offset_values, rk4_progress
from .minimum_change import CurvatureEnvelope, MinimumChangeConfig, completed_recovery, first_stable_c
from .policy_motion import BSRadio, policy_from_exposure


@dataclass(frozen=True)
class SpatialConfig(LateralConfig):
    vertical_speed_limit_mps: float = 3.
    vertical_accel_limit_mps2: float = .2


def xyz_at(frame, q, d, h):
    q, d, h = np.broadcast_arrays(q, d, h)
    center, _, normal, _, _ = frame.frame(q)
    return np.concatenate((center + d[..., None] * normal, h[..., None]), axis=-1)


def envelope_duration_min(frame, cfg, envelope, *, q, t, source, target):
    """Envelope-aware replacement for :func:`duration_min`, lateral moves only.

    The coupled envelope currently models the lateral offset profile only, so a
    candidate with a vertical component is refused rather than silently sized by
    the lateral bounds. Extending it to level changes is separate work.

    Returns ``(duration, rejection_reason)``; the duration falls back to the
    closed-form value when the candidate is rejected, so the vectorised forecast
    stays well formed while the caller suppresses the candidate.
    """
    from .motion_envelope import feasible_duration
    delta = np.abs(np.asarray(target, float) - np.asarray(source, float))
    base = duration_min(source, target, cfg)
    if delta.size > 1 and delta[1] > 1e-9:
        return base, "envelope_supports_lateral_only"
    if not np.any(delta):
        return 0., None
    report = feasible_duration(frame, cfg, envelope, q0=float(q), t0=float(t),
                               source=float(np.asarray(source, float)[0]),
                               target=float(np.asarray(target, float)[0]))
    if report["duration_s"] is None:
        return base, report["status"]
    return float(report["duration_s"]), None


def duration_min(source, target, cfg):
    delta = np.abs(np.asarray(target) - source)
    if not np.any(delta):
        return 0.
    return max(cfg.min_maneuver_s, 1.875 * delta[0] / cfg.lateral_speed_limit_mps,
        np.sqrt(10 / np.sqrt(3) * delta[0] / cfg.lateral_accel_limit_mps2),
        1.875 * delta[1] / cfg.vertical_speed_limit_mps,
        np.sqrt(10 / np.sqrt(3) * delta[1] / cfg.vertical_accel_limit_mps2))


@dataclass(frozen=True)
class SpatialMove:
    start_s: float
    source: tuple[float, float]
    target: tuple[float, float]
    duration_s: float

    def sample(self, t):
        values = [offset_values(t, self.start_s, self.source[i], self.target[i], self.duration_s)
                  for i in range(2)]
        return tuple(np.stack(pair, axis=-1) for pair in zip(*values))


def direction(source, target):
    dd, dh = np.asarray(target) - source
    if dd and dh:
        return "diagonal_climb" if dh > 0 else "diagonal_descent"
    if dh:
        return "climb" if dh > 0 else "descent"
    return "lateral" if dd else "stay"


def candidate_pairs(offsets, heights, source, mode):
    if mode not in {"none", "lateral", "vertical", "joint"}:
        raise ValueError("invalid spatial control mode")
    if mode == "none":
        return []
    return [(float(d), float(h)) for d in offsets for h in heights
            if (d, h) != tuple(source) and (mode != "lateral" or h == source[1])
            and (mode != "vertical" or d == source[0])]


class SpatialPlanner:
    def __init__(self, frame, settings=MinimumChangeConfig()):
        self.settings = settings
        self.curvature = CurvatureEnvelope(frame)

    def evaluate(self, frame, radio, cfg, *, q, t, source, alternatives, history):
        """Batch all endpoint/duration choices in one displacement shell."""
        targets = np.array([source] + [a[0] for a in alternatives], float)
        durations = np.array([0.] + [a[1] for a in alternatives], float)
        state = np.full(len(targets), q, float)
        clock = float(t)
        flags = [np.full(len(targets), bad, float) for _, bad in history]
        flag_times = [u for u, _ in history]
        good = [np.full(len(targets), np.mean(flags) <= cfg.exposure_c + 1e-12)]
        active, times = [np.ones(len(targets), bool)], [clock]
        before, after = np.full(len(targets), q), np.full(len(targets), np.nan)
        while clock < t + cfg.horizon_s - 1e-8:
            next_policy = (math.floor((clock + 1e-8) / cfg.policy_s) + 1) * cfg.policy_s
            step = min(cfg.forecast_dt_s, t + cfg.horizon_s - clock, next_policy - clock)
            previous = state.copy()
            state = rk4_progress(frame, state, clock, step,
                lambda time: offset_values(time, t, source[0], targets[:, 0], durations)[0], cfg.cruise_mps)
            old_clock = clock
            clock += step
            crossed = (old_clock <= t + durations + 1e-8) & (clock >= t + durations - 1e-8) & np.isnan(after)
            before[crossed], after[crossed] = previous[crossed], state[crossed]
            if abs(clock / cfg.policy_s - round(clock / cfg.policy_s)) > 1e-8:
                continue
            dh = [offset_values(clock, t, source[i], targets[:, i], durations)[0] for i in range(2)]
            inside = state < frame.length_m - 1e-7
            values = radio(xyz_at(frame, np.minimum(state, frame.length_m), *dh))["sinr_db"]
            flags.append((values < cfg.threshold_db).astype(float)); flag_times.append(clock)
            while flag_times and flag_times[0] < clock - cfg.window_s - 1e-8:
                flag_times.pop(0); flags.pop(0)
            good.append(np.mean(flags, axis=0) <= cfg.exposure_c + 1e-12)
            active.append(inside); times.append(clock)
            if not inside.any():
                break
        times, good, active = np.array(times), np.array(good), np.array(active)
        rows = []
        for k in range(1, len(targets)):
            target, duration = targets[k], float(durations[k])
            rho = float(np.linalg.norm(target - source))
            row = {"target": target.tolist(), "duration_s": duration, "displacement_m": rho,
                   "direction": direction(source, target), "status": "insufficient_recovery_horizon"}
            common = active[:, 0] & active[:, k]
            paired = times[common]
            recovery = completed_recovery(paired, good[common, k], t + duration, cfg.window_s)
            stay = first_stable_c(paired, good[common, 0], cfg.window_s)
            row.update(recovery_time_s=recovery, stay_recovery_time_s=stay)
            if recovery is None:
                row["status"] = "no_stable_c_after_completion"
            elif stay is not None and recovery > stay - cfg.policy_s + 1e-8:
                row["status"] = "staying_recovers_no_later"
            elif np.isfinite(after[k]) and after[k] < frame.length_m:
                geometry = self.curvature.offset_bound(q, after[k], [source[0], target[0]])
                moving = geometry + 10 / np.sqrt(3) * rho / duration**2 / cfg.cruise_mps**2
                remaining = self.curvature.offset_bound(before[k], frame.length_m, [target[0]])
                row.update(maneuver_curvature_bound_per_m=float(moving) if np.isfinite(moving) else None,
                           remaining_curvature_bound_per_m=float(remaining) if np.isfinite(remaining) else None)
                row["status"] = "eligible" if max(moving, remaining) <= 1 / self.settings.minimum_radius_m + 1e-12 else "curvature_rejected"
            rows.append(row)
        return rows

    def candidates(self, frame, radio, cfg, *, q, t, source, targets, history, policy,
                   allow_current_c=False):
        if policy == "C" and not allow_current_c:
            return [{"target": list(source), "duration_s": 0., "status": "policy_satisfied_hold"}]
        targets = np.array(targets, float)
        if not len(targets):
            return []
        grid = (targets - source) / self.settings.candidate_spacing_m
        if np.any(np.abs(grid - np.round(grid)) > 1e-8):
            raise ValueError("candidate coordinate changes must follow the configured grid")
        distance2 = np.round(np.sum((targets - source)**2, axis=1), 8)
        records = []
        for shell in sorted(set(distance2)):
            alternatives = []
            for target in targets[distance2 == shell]:
                base = duration_min(source, target, cfg)
                ds = np.arange(base, cfg.horizon_s - cfg.window_s + 1e-8, self.settings.duration_search_step_s)
                if not len(ds):
                    records.append({"target": target.tolist(), "duration_s": float(base),
                        "displacement_m": float(np.sqrt(shell)), "status": "maneuver_exceeds_horizon"})
                alternatives.extend((target, float(d)) for d in ds)
            if alternatives:
                rows = self.evaluate(frame, radio, cfg, q=q, t=t, source=source, alternatives=alternatives, history=history)
                records.extend(rows)
                if any(r["status"] == "eligible" for r in rows):
                    break
        return records

    @staticmethod
    def choose(records):
        eligible = [r for r in records if r["status"] == "eligible"]
        return min(eligible, key=lambda r: (r["displacement_m"], r["duration_s"],
                                            r["recovery_time_s"], *r["target"])) if eligible else None


def _policy_from_history(history, cfg):
    exposure = sum(b for _, b in history) / len(history) if history else 0.0
    return policy_from_exposure(exposure, cfg.policy_config()), exposure


POLICY_SCORE = {"F": 0, "R": 1, "C": 2}


def policy_cost(policy_times, cfg):
    """Ordinal demonstration cost; weights are explicit, configurable inputs."""
    return (cfg.policy_cost_r * float(policy_times.get("R", 0.))
            + cfg.policy_cost_f * float(policy_times.get("F", 0.)))


def forecast_stay(frame, radio, cfg, *, q, t, point, history, horizon_s=None):
    """Forecast the no-action policy sequence from the current state.

    This is deliberately a scalar, policy-clock forecast: it reuses the
    already reconstructed corridor frame and radio kernel and does not run any
    candidate maneuvers.  It is the predictive trigger, not a new simulation
    result or a capacity estimator.
    """
    state, clock = float(q), float(t)
    horizon = cfg.horizon_s if horizon_s is None else float(horizon_s)
    local_history = list(history)
    rows = []
    while clock < t + horizon - 1e-8:
        next_policy = (math.floor((clock + 1e-8) / cfg.policy_s) + 1) * cfg.policy_s
        step = min(cfg.forecast_dt_s, t + horizon - clock, next_policy - clock)
        state = rk4_progress(frame, state, clock, step,
                             lambda _: float(point[0]), cfg.cruise_mps)
        clock += step
        if abs(clock / cfg.policy_s - round(clock / cfg.policy_s)) > 1e-8:
            continue
        position = xyz_at(frame, min(state, frame.length_m), float(point[0]), float(point[1]))
        values = radio(position[None, :]); sinr = float(values["sinr_db"][0])
        local_history = [(u, b) for u, b in local_history if u >= clock - cfg.window_s - 1e-8]
        local_history.append((clock, int(sinr < cfg.threshold_db)))
        policy, exposure = _policy_from_history(local_history, cfg)
        rows.append({"t_s": float(clock), "q_m": float(state), "sinr_db": sinr,
                     "exposure": float(exposure), "policy": policy,
                     "history_count": len(local_history)})
        if state >= frame.length_m - 1e-7:
            break
    return rows


def rolling_endpoint_candidates(frame, radio, cfg, planner, *, q, t, source, targets,
                                history, envelope=None):
    """Forecast one continuous move per candidate to the rolling endpoint.

    The warning horizon remains ``predictive_rollout_s``.  All alternatives
    share a planning clock equal to that warning plus the optional evaluation
    extension and retain the available exposure history.  With a zero
    extension, eligibility is the reviewed endpoint-policy upgrade.  With a
    positive extension, eligibility requires a lower integrated policy cost
    over the common planning interval.  In both modes a maneuver must complete
    within the planning clock and pass the curvature guard.
    """
    warning_horizon = float(cfg.predictive_rollout_s)
    evaluation_extension = float(cfg.predictive_evaluation_s)
    horizon = warning_horizon + evaluation_extension
    points = np.array([source] + list(targets), float)
    if envelope is None:
        durations = np.array([0.] + [duration_min(source, target, cfg) for target in points[1:]], float)
        envelope_rejected = [None]*len(points)
    else:
        pairs = [envelope_duration_min(frame, cfg, envelope, q=q, t=t, source=source, target=target)
                 for target in points[1:]]
        durations = np.array([0.] + [d for d, _ in pairs], float)
        envelope_rejected = [None] + [reason for _, reason in pairs]
    state = np.full(len(points), float(q))
    clock = float(t)
    flags = [np.full(len(points), bad, float) for _, bad in history]
    flag_times = [u for u, _ in history]
    before = np.full(len(points), float(q)); after = np.full(len(points), np.nan)
    endpoint_sinr = np.full(len(points), np.nan)
    endpoint_exposure = np.full(len(points), np.nan)
    endpoint_policy = np.full(len(points), "", dtype="U1")
    policy_times = {name: np.zeros(len(points), float) for name in "CRF"}
    active = np.ones(len(points), bool)
    while clock < t + horizon - 1e-8:
        next_policy = (math.floor((clock + 1e-8) / cfg.policy_s) + 1) * cfg.policy_s
        step = min(cfg.forecast_dt_s, t + horizon - clock, next_policy - clock)
        previous = state.copy(); old_clock = clock
        state = rk4_progress(frame, state, clock, step,
            lambda time: offset_values(time, t, source[0], points[:, 0], durations)[0], cfg.cruise_mps)
        clock += step
        crossed = (old_clock <= t + durations + 1e-8) & (clock >= t + durations - 1e-8) & np.isnan(after)
        before[crossed], after[crossed] = previous[crossed], state[crossed]
        if abs(clock / cfg.policy_s - round(clock / cfg.policy_s)) > 1e-8:
            continue
        dh = [offset_values(clock, t, source[i], points[:, i], durations)[0] for i in range(2)]
        active = state < frame.length_m - 1e-7
        values = radio(xyz_at(frame, np.minimum(state, frame.length_m), *dh))["sinr_db"]
        flags.append((values < cfg.threshold_db).astype(float)); flag_times.append(clock)
        while flag_times and flag_times[0] < clock - cfg.window_s - 1e-8:
            flag_times.pop(0); flags.pop(0)
        exposure = np.mean(flags, axis=0)
        policies = np.array([policy_from_exposure(float(e), cfg.policy_config()) for e in exposure])
        interval = min(float(cfg.policy_s), float(clock - t))
        for name in "CRF":
            policy_times[name] += interval * (policies == name)
        endpoint_sinr, endpoint_exposure, endpoint_policy = values, exposure, policies
        if not active.any():
            break
    stay_policy = str(endpoint_policy[0])
    stay_times = {name: float(policy_times[name][0]) for name in "CRF"}
    stay_cost = policy_cost(stay_times, cfg)
    rows = []
    for k, target in enumerate(points):
        rho = float(np.linalg.norm(target - source))
        row = {"target": target.tolist(), "duration_s": float(durations[k]),
               "displacement_m": rho, "direction": direction(source, target),
               "planner": "rolling_endpoint", "forecast_start_s": float(t),
               "warning_horizon_s": warning_horizon,
               "evaluation_extension_s": evaluation_extension,
               "planning_horizon_s": horizon,
               "forecast_endpoint_s": float(clock), "forecast_endpoint_q_m": float(state[k]),
               "endpoint_sinr_db": float(endpoint_sinr[k]),
               "endpoint_exposure": float(endpoint_exposure[k]),
               "endpoint_policy": str(endpoint_policy[k]), "stay_endpoint_policy": stay_policy,
               "forecast_policy_time_s": {name: float(policy_times[name][k]) for name in "CRF"},
               "stay_forecast_policy_time_s": stay_times,
               "policy_cost_s": policy_cost({name: policy_times[name][k] for name in "CRF"}, cfg),
               "stay_policy_cost_s": stay_cost,
               "status": "rolling_stay_reference" if k == 0 else "endpoint_policy_not_improved"}
        if k == 0:
            rows.append(row); continue
        if envelope_rejected[k] is not None:
            row["status"] = f"envelope_rejected:{envelope_rejected[k]}"
            rows.append(row); continue
        if durations[k] > horizon + 1e-8:
            row["status"] = "maneuver_exceeds_planning_horizon"
        elif not active[k] or not np.isfinite(after[k]):
            row["status"] = "endpoint_beyond_corridor"
        else:
            geometry = planner.curvature.offset_bound(q, after[k], [source[0], target[0]])
            moving = geometry + 10 / np.sqrt(3) * rho / durations[k]**2 / cfg.cruise_mps**2
            remaining = planner.curvature.offset_bound(before[k], frame.length_m, [target[0]])
            row.update(maneuver_curvature_bound_per_m=float(moving) if np.isfinite(moving) else None,
                       remaining_curvature_bound_per_m=float(remaining) if np.isfinite(remaining) else None)
            if max(moving, remaining) > 1 / planner.settings.minimum_radius_m + 1e-12:
                row["status"] = "curvature_rejected"
            elif evaluation_extension > 0:
                row["status"] = ("eligible" if row["policy_cost_s"] < stay_cost - 1e-8
                                 else "integrated_policy_not_improved")
            elif POLICY_SCORE[row["endpoint_policy"]] > POLICY_SCORE[stay_policy]:
                row["status"] = "eligible"
        rows.append(row)
    return rows


def advance_stay(frame, radio, cfg, *, q, t, point, history, duration_s):
    """Advance a no-action branch through a command latency."""
    if duration_s <= 1e-12:
        return float(q), list(history)
    state, clock = float(q), float(t)
    local_history = list(history)
    target = t + float(duration_s)
    while clock < target - 1e-8:
        next_policy = (math.floor((clock + 1e-8) / cfg.policy_s) + 1) * cfg.policy_s
        step = min(cfg.forecast_dt_s, target - clock, next_policy - clock)
        state = rk4_progress(frame, state, clock, step,
                             lambda _: float(point[0]), cfg.cruise_mps)
        clock += step
        if abs(clock / cfg.policy_s - round(clock / cfg.policy_s)) > 1e-8:
            continue
        position = xyz_at(frame, min(state, frame.length_m), float(point[0]), float(point[1]))
        sinr = float(radio(position[None, :])["sinr_db"][0])
        local_history = [(u, b) for u, b in local_history if u >= clock - cfg.window_s - 1e-8]
        local_history.append((clock, int(sinr < cfg.threshold_db)))
    return float(state), local_history

def simulate_spatial(frame, scenario, cfg, offsets, heights, mode,
                     settings=MinimumChangeConfig(), envelope=None):
    for values in (offsets, heights):
        if not len(values) or not np.isfinite(values).all() or np.any(np.diff(values) <= 0):
            raise ValueError("candidate arrays must be finite, unique and increasing")
    if min(heights) <= 0 or 0. not in offsets or cfg.altitude_m not in heights:
        raise ValueError("positive heights and initial (0,altitude) candidate required")
    frame.report(float(np.max(np.abs(offsets))))
    candidate_pairs(offsets, heights, (0., cfg.altitude_m), mode)
    planner = SpatialPlanner(frame, settings)
    radio, policy_cfg = BSRadio(scenario.base_stations, scenario.radio), cfg.policy_config()
    t = q = 0.
    point = (0., cfg.altitude_m)
    move = None
    history, trace, observations, events = [], [], [], []
    next_policy = next_decision = cooldown_until = 0.
    policy, exposure = "C", None
    while q < frame.length_m - 1e-7:
        if t > 3 * frame.length_m / cfg.cruise_mps + cfg.horizon_s:
            raise RuntimeError("nonadvancing spatial flight")
        if move and t >= move.start_s + move.duration_s - 1e-8:
            point = move.target
            events.append({"status": "transition_completed", "t_s": t, "q_m": q,
                           "source": list(move.source), "target": list(point)})
            move = None; cooldown_until = t + cfg.cooldown_s
        def sample(time):
            return move.sample(time) if move else (np.array(point), np.zeros(2), np.zeros(2))
        dh, vd, ad = sample(t)
        xyz = xyz_at(frame, q, *dh)
        if t >= next_policy - 1e-8:
            values = radio(xyz[None, :]); sinr = float(values["sinr_db"][0])
            history = [(u, b) for u, b in history if u >= t - cfg.window_s - 1e-8]
            history.append((t, int(sinr < cfg.threshold_db)))
            exposure = sum(b for _, b in history) / len(history)
            policy = policy_from_exposure(exposure, policy_cfg)
            observations.append({"t_s": t, "q_m": q, "offset_m": float(dh[0]), "altitude_m": float(dh[1]),
                "sinr_db": sinr, "rsrp_dbm": float(values["serving_rsrp_dbm"][0]), "serving_bs": int(values["serving_bs"][0]),
                "exposure": exposure, "policy": policy, "history_count": len(history), "group_size": 1})
            next_policy += cfg.policy_s
        if t >= next_decision - 1e-8:
            if mode != "none" and move is None and t >= cooldown_until - 1e-8:
                if cfg.predictive_endpoint_upgrade:
                    future = forecast_stay(frame, radio, cfg, q=q, t=t, point=point,
                                           history=history, horizon_s=cfg.predictive_rollout_s)
                    stay = future[-1] if future else None
                    if cfg.predictive_evaluation_s > 0:
                        degraded = [row for row in future
                                    if POLICY_SCORE[row["policy"]] < POLICY_SCORE[policy]]
                        degradation = bool(degraded)
                        warning = degraded[0] if degraded else None
                    else:
                        degradation = stay is not None and POLICY_SCORE[stay["policy"]] < POLICY_SCORE[policy]
                        warning = stay if degradation else None
                    records = []
                    if degradation:
                        records = rolling_endpoint_candidates(frame, radio, cfg, planner, q=q, t=t,
                            source=point, targets=candidate_pairs(offsets, heights, point, mode),
                            history=history, envelope=envelope)
                    eligible = [r for r in records if r["status"] == "eligible"]
                    if cfg.predictive_evaluation_s > 0:
                        chosen = min(eligible, key=lambda r: (r["policy_cost_s"],
                            -POLICY_SCORE[r["endpoint_policy"]], r["displacement_m"],
                            r["duration_s"], *r["target"])) if eligible else None
                    else:
                        chosen = min(eligible, key=lambda r: (-POLICY_SCORE[r["endpoint_policy"]],
                            r["displacement_m"], r["duration_s"], *r["target"])) if eligible else None
                    if chosen:
                        move = SpatialMove(t, tuple(point), tuple(chosen["target"]), chosen["duration_s"])
                        chosen["status"] = "transition_accepted"
                    if records:
                        for row in records:
                            events.append({"t_s": t, "q_m": q, "source": list(point),
                                "trigger_policy": policy, "decision_trigger": "rolling_policy_degradation",
                                "predictive_trigger": True, "planning_time_s": t,
                                "movement_start_s": t, "reaction_latency_s": 0.,
                                "predicted_degradation_time_s": None if warning is None else warning["t_s"],
                                "predicted_degradation_policy": None if warning is None else warning["policy"],
                                **row})
                    else:
                        events.append({"t_s": t, "q_m": q, "source": list(point),
                            "target": list(point), "duration_s": 0., "displacement_m": 0.,
                            "direction": "stay", "planner": "rolling_endpoint",
                            "trigger_policy": policy, "decision_trigger": "rolling_policy_hold",
                            "predictive_trigger": False, "planning_time_s": t,
                            "movement_start_s": t, "reaction_latency_s": 0.,
                            "forecast_start_s": t,
                            "warning_horizon_s": cfg.predictive_rollout_s,
                            "evaluation_extension_s": cfg.predictive_evaluation_s,
                            "planning_horizon_s": cfg.predictive_rollout_s + cfg.predictive_evaluation_s,
                            "predicted_degradation_time_s": None if warning is None else warning["t_s"],
                            "forecast_endpoint_s": None if stay is None else stay["t_s"],
                            "forecast_endpoint_q_m": None if stay is None else stay["q_m"],
                            "endpoint_sinr_db": None if stay is None else stay["sinr_db"],
                            "endpoint_exposure": None if stay is None else stay["exposure"],
                            "endpoint_policy": None if stay is None else stay["policy"],
                            "stay_endpoint_policy": None if stay is None else stay["policy"],
                            "status": "rolling_hold_no_degradation"})
                else:
                    future = forecast_stay(frame, radio, cfg, q=q, t=t, point=point, history=history) \
                        if cfg.predictive_enabled else []
                    future_f_rows = [r for r in future if r["policy"] == "F"]
                    # The earliest forecast F is the action deadline. A move that
                    # cannot finish before it is not a predictive response to the
                    # upcoming degradation; later F episodes require a separate
                    # forecast after the first episode has cleared.
                    future_f = future_f_rows[0] if future_f_rows else None
                    trigger = policy != "C" or (cfg.predictive_enabled and future_f is not None)
                    records = []
                    plan_q, plan_t, plan_history = q, t, history
                    if trigger and cfg.predictive_enabled and cfg.reaction_latency_s > 1e-12:
                        plan_q, plan_history = advance_stay(frame, radio, cfg, q=q, t=t, point=point,
                                                            history=history, duration_s=cfg.reaction_latency_s)
                        plan_t = t + cfg.reaction_latency_s
                    if trigger:
                        records = planner.candidates(frame, radio, cfg, q=plan_q, t=plan_t, source=point,
                            targets=candidate_pairs(offsets, heights, point, mode), history=plan_history,
                            policy=policy, allow_current_c=cfg.predictive_enabled)
                        if (cfg.predictive_enabled and cfg.predictive_completion_before_f
                                and future_f is not None):
                            cutoff = float(future_f["t_s"])
                            for row in records:
                                if row.get("status") == "eligible":
                                    row["planned_completion_s"] = float(plan_t + row["duration_s"])
                                    row["forecast_f_time_s"] = cutoff
                                    if row["planned_completion_s"] > cutoff + 1e-8:
                                        row["status"] = "candidate_too_late_for_forecast_f"
                    chosen = planner.choose(records)
                    if chosen:
                        start_t = t + cfg.reaction_latency_s if cfg.predictive_enabled else t
                        move = SpatialMove(start_t, tuple(point), tuple(chosen["target"]), chosen["duration_s"])
                        chosen["status"] = "transition_accepted"
                    prediction_trigger = bool(cfg.predictive_enabled and policy == "C" and future_f is not None)
                    for row in records:
                        events.append({"t_s": t, "q_m": q, "source": list(point),
                            "trigger_policy": policy,
                            "decision_trigger": ("future_f_forecast" if prediction_trigger
                                else "observed_non_c" if policy != "C" else "none"),
                            "predictive_trigger": prediction_trigger,
                            "predicted_future_f_time_s": row.get("forecast_f_time_s",
                                None if future_f is None else future_f["t_s"]),
                            "planning_time_s": plan_t,
                            "movement_start_s": (t + cfg.reaction_latency_s if cfg.predictive_enabled else t),
                            "reaction_latency_s": cfg.reaction_latency_s, **row})
            next_decision += cfg.decision_s
        end = min(t + cfg.dt_s, next_policy, next_decision)
        if move:
            end = min(end, move.start_s + move.duration_s)
            if t < move.start_s - 1e-8:
                end = min(end, move.start_s)
        dt = end - t
        if dt <= 1e-10:
            raise RuntimeError("nonadvancing spatial event loop")
        offset_at = lambda time: sample(time)[0][0]
        q_next = float(rk4_progress(frame, q, t, dt, offset_at, cfg.cruise_mps))
        if q_next >= frame.length_m:
            lo, hi = 0., dt
            for _ in range(38):
                mid = (lo + hi) / 2
                if rk4_progress(frame, q, t, mid, offset_at, cfg.cruise_mps) < frame.length_m:
                    lo = mid
                else:
                    hi = mid
            dt, q_next = (lo + hi) / 2, frame.length_m
        vxy, axy = frame.kinematics(q, dh[0], vd[0], ad[0], cfg.cruise_mps)
        velocity, acceleration = np.r_[vxy, vd[1]], np.r_[axy, ad[1]]
        trace.append({"t_s": t, "end_s": t + dt, "q_m": q, "offset_m": float(dh[0]), "altitude_m": float(dh[1]),
            "transverse_velocity_mps": vd.tolist(), "transverse_acceleration_mps2": ad.tolist(),
            "position_m": xyz.tolist(), "velocity_mps": velocity.tolist(), "acceleration_mps2": acceleration.tolist(),
            "speed_mps": float(np.linalg.norm(velocity)), "policy": policy, "exposure": exposure,
            "target": list(move.target) if move else None})
        q, t = q_next, t + dt
    if move:
        raise RuntimeError("unfinished spatial maneuver at exit")
    terminal = {"t_s": t, "q_m": q, "offset_m": point[0], "altitude_m": point[1],
                "position_m": xyz_at(frame, q, *point).tolist()}
    accepted = [r for r in events if r["status"] == "transition_accepted"]
    durations = np.array([r["end_s"] - r["t_s"] for r in trace])
    extra = 0.
    for e in accepted:
        m = SpatialMove(e["t_s"], tuple(e["source"]), tuple(e["target"]), e["duration_s"])
        grid = np.linspace(m.start_s, m.start_s + m.duration_s, 1001)
        speeds = np.sqrt(cfg.cruise_mps**2 + np.sum(m.sample(grid)[1]**2, axis=-1)) - cfg.cruise_mps
        extra += float((grid[1] - grid[0]) / 3 * (speeds[0] + speeds[-1] + 4 * speeds[1:-1:2].sum() + 2 * speeds[2:-1:2].sum()))
    policy_times = {p: float(sum(dt for r, dt in zip(trace, durations) if r["policy"] == p)) for p in "CRF"}
    deltas = np.array([np.array(e["target"]) - e["source"] for e in accepted]).reshape(-1, 2)
    positions = np.array([xyz_at(frame, r["q_m"], r["offset_m"], r["altitude_m"]) for r in observations])
    stations = scenario.base_stations.positions(scenario.radio.assumed_bs_height_m)
    dist = np.linalg.norm(positions[:, None, :] - stations[None, :, :], axis=-1)
    selected = np.argsort(dist, axis=1)[:, :scenario.radio.served_set_size]
    horizontal = np.linalg.norm(positions[:, None, :2] - stations[None, :, :2], axis=-1)
    summary = {"mode": mode, "completed_moves": sum(e["status"] == "transition_completed" for e in events),
        "directions": {name: sum(e["direction"] == name for e in accepted)
                       for name in ["lateral", "climb", "descent", "diagonal_climb", "diagonal_descent"]},
        "offset_height_sequence_m": [[0., cfg.altitude_m]] + [e["target"] for e in accepted],
        "corridor_time_s": t, "path_length_m": cfg.cruise_mps * t + extra,
        "policy_time_s": policy_times, "policy_time_shares": {p: v / t for p, v in policy_times.items()},
        "total_displacement_m": float(np.linalg.norm(deltas, axis=1).sum()),
        "lateral_displacement_m": float(np.abs(deltas[:, 0]).sum()),
        "total_climb_m": float(np.maximum(deltas[:, 1], 0).sum()),
        "total_descent_m": float(np.maximum(-deltas[:, 1], 0).sum()),
        "total_maneuver_time_s": sum(e["duration_s"] for e in accepted),
        "time_mean_exposure": float(sum(r["exposure"] * dt for r, dt in zip(trace, durations)) / t),
        "max_speed_mps": max(r["speed_mps"] for r in trace),
        "max_cartesian_acceleration_mps2": max(float(np.linalg.norm(r["acceleration_mps2"])) for r in trace),
        "height_extrapolation_time_fraction": float(sum(dt for r, dt in zip(trace, durations) if not 22.5 < r["altitude_m"] <= 300) / t),
        "any_selected_link_beyond_4km_fraction": float(np.any(np.take_along_axis(horizontal, selected, axis=1) > 4000, axis=1).mean()),
        "capacity_estimated": False, "group_size": 1,
        "controller_logic": ("rolling_endpoint_upgrade" if cfg.predictive_endpoint_upgrade
                             else "predictive" if cfg.predictive_enabled else "reactive"),
        "warning_horizon_s": float(cfg.predictive_rollout_s if cfg.predictive_endpoint_upgrade else cfg.horizon_s),
        "evaluation_extension_s": float(cfg.predictive_evaluation_s if cfg.predictive_endpoint_upgrade else 0.),
        "prediction_horizon_s": float((cfg.predictive_rollout_s + cfg.predictive_evaluation_s)
                                      if cfg.predictive_endpoint_upgrade else cfg.horizon_s),
        "reaction_latency_s": float(cfg.reaction_latency_s),
        "prediction_note": ((("At each decision, any degradation inside the warning horizon opens a common extended evaluation; only a lower integrated R/F policy cost can move."
            if cfg.predictive_evaluation_s > 0 else
            "At each decision, the no-action policy at the rolling endpoint is compared with feasible candidate endpoints; only a strict policy upgrade can move.")
            if cfg.predictive_endpoint_upgrade else ("Future no-action policy is forecast before candidate search; accepted moves still require stable C, natural-recovery and curvature checks."
            + (" Predictive candidates must also complete before a forecast F episode." if cfg.predictive_completion_before_f else ""))
            if cfg.predictive_enabled else "Reactive baseline: candidate search starts after an observed non-C policy."))}
    return {"summary": summary, "trace": trace, "observations": observations, "events": events, "terminal": terminal}


def field_samples_spatial(frame, scenario, cfg, offsets, heights):
    q = np.linspace(0, frame.length_m, int(np.ceil(frame.length_m / cfg.spatial_sample_m)) + 1)
    Q, D, H = np.meshgrid(q, offsets, heights, indexing="ij")
    values = BSRadio(scenario.base_stations, scenario.radio)(xyz_at(frame, Q.ravel(), D.ravel(), H.ravel()))
    return {"q_m": q, "offsets_m": np.asarray(offsets), "heights_m": np.asarray(heights),
            "sinr_db": values["sinr_db"].reshape(Q.shape), "rsrp_dbm": values["serving_rsrp_dbm"].reshape(Q.shape),
            "serving_bs": values["serving_bs"].reshape(Q.shape)}
