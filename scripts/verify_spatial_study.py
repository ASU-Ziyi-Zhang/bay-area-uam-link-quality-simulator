"""Independent archived XYZ/radio/policy audit of Stage-2 singleton flights."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from verify_lateral_study import independent_reference, read
from plot_wide_radio_map import radio_at, sha

POLICY_SCORE = {"F": 0, "R": 1, "C": 2}


def transverse(times, moves, initial_height):
    times = np.asarray(times)
    p = np.zeros((*times.shape, 2)); p[..., 1] = initial_height
    v, a = np.zeros_like(p), np.zeros_like(p)
    for move in moves:
        start, duration = move.get("movement_start_s", move["t_s"]), move["duration_s"]
        active = times >= start - 1e-9
        u = np.clip((times - start) / duration, 0, 1)
        delta = np.array(move["target"]) - move["source"]
        f = 10*u**3 - 15*u**4 + 6*u**5
        df = 30*u**2*(1-u)**2
        ddf = 60*u*(1-u)*(1-2*u)
        p[active] = (np.array(move["source"]) + f[..., None]*delta)[active]
        v[active] = (df[..., None]*delta/duration)[active]
        a[active] = (ddf[..., None]*delta/duration**2)[active]
    return p, v, a


def audit(run):
    run = Path(run).resolve(); manifest = read(run / "manifest.json")
    if manifest["status"] != "completed":
        raise ValueError("run is incomplete or changed during execution")
    for e in manifest["outputs"]:
        if sha(run / e["path"]) != e["sha256"]:
            raise ValueError(f"output hash mismatch: {e['path']}")
    spec = read(run / "resolved_config.json")
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        for e in manifest["code"] + manifest["inputs"]:
            if hashlib.sha256(archive.read(e["path"])).hexdigest() != e["sha256"]:
                raise ValueError(f"archive hash mismatch: {e['path']}")
        frame = independent_reference(archive, manifest, spec["parameters"]["center_control_step_m"])
    reports = []
    for case in spec["cases"]:
        folder = run / case["id"]; cfg = case["parameters"]
        trace = read(folder / "trace.json.gz"); obs = read(folder / "observations.json.gz")
        events = read(folder / "events.json"); terminal = read(folder / "terminal.json")
        summary = read(folder / "summary.json")
        moves = [e for e in events if e["status"] == "transition_accepted"]
        completed = [e for e in events if e["status"] == "transition_completed"]
        if len(moves) != len(completed) or len(moves) != summary["completed_moves"]:
            raise ValueError("maneuver count mismatch")
        times = np.array([r["t_s"] for r in trace]); ends = np.array([r["end_s"] for r in trace])
        q = np.array([r["q_m"] for r in trace]); dt = ends - times
        if not np.allclose(ends[:-1], times[1:], atol=1e-8, rtol=0) or abs(ends[-1] - terminal["t_s"]) > 1e-8 or np.any(dt <= 0):
            raise ValueError("noncontiguous motion intervals")
        p, v, a = transverse(times, moves, cfg["altitude_m"])
        center, tangent, normal, norm, curvature = frame(q)
        xyz = np.c_[center + p[:, 0, None]*normal, p[:, 1]]
        expected_v = np.c_[cfg["cruise_mps"]*tangent + v[:, 0, None]*normal, v[:, 1]]
        K = curvature/(1-curvature*p[:, 0])
        expected_a = np.c_[-(v[:, 0]*K*cfg["cruise_mps"])[:, None]*tangent
                           +(cfg["cruise_mps"]**2*K+a[:, 0])[:, None]*normal, a[:, 1]]
        error = float(np.max(np.linalg.norm(xyz - [r["position_m"] for r in trace], axis=1)))
        if error > 1e-6 or not np.allclose(expected_v, [r["velocity_mps"] for r in trace], atol=1e-8, rtol=0) or not np.allclose(expected_a, [r["acceleration_mps2"] for r in trace], atol=1e-8, rtol=0):
            raise ValueError("independent Cartesian derivatives mismatch")
        if np.any(abs(v) > np.array([cfg["lateral_speed_limit_mps"], cfg["vertical_speed_limit_mps"]]) + 1e-8) or np.any(abs(a) > np.array([cfg["lateral_accel_limit_mps2"], cfg["vertical_accel_limit_mps2"]]) + 1e-8):
            raise ValueError("component limits exceeded")
        if np.any(p[:, 0] < min(spec["offsets_m"]) - 1e-8) or np.any(p[:, 0] > max(spec["offsets_m"]) + 1e-8) or np.any(p[:, 1] < min(spec["heights_m"]) - 1e-8) or np.any(p[:, 1] > max(spec["heights_m"]) + 1e-8):
            raise ValueError("candidate envelope exceeded")
        # Independent vectorized RK4: sixteen substeps per archived interval,
        # no use of the simulator's adaptive progress integrator.
        state = q.copy(); clocks = times.copy(); step = dt/16
        def rate(progress, clock):
            d = transverse(clock, moves, cfg["altitude_m"])[0][:, 0]
            _, _, _, norm, k = frame(progress)
            return cfg["cruise_mps"]/(norm*(1-k*d))
        for _ in range(16):
            k1 = rate(state, clocks); k2 = rate(state+step*k1/2, clocks+step/2)
            k3 = rate(state+step*k2/2, clocks+step/2); k4 = rate(state+step*k3, clocks+step)
            state += step*(k1+2*k2+2*k3+k4)/6; clocks += step
        progress_error = float(np.max(abs(state - np.r_[q[1:], terminal["q_m"]])))
        if progress_error > 2e-5:
            raise ValueError(f"independent progress check failed: {progress_error}")
        oq = np.array([r["q_m"] for r in obs]); ot = np.array([r["t_s"] for r in obs])
        op = transverse(ot, moves, cfg["altitude_m"])[0]
        oc, _, on, _, _ = frame(oq)
        radio = radio_at(np.c_[oc + op[:, 0, None]*on, op[:, 1]], spec)
        signal_error = float(np.max(abs(radio["sinr_db"] - [r["sinr_db"] for r in obs])))
        if signal_error > 1e-8 or not np.array_equal(radio["serving_bs"], [r["serving_bs"] for r in obs]):
            raise ValueError("independent radio mismatch")
        for i, row in enumerate(obs):
            mask = (ot >= row["t_s"]-cfg["window_s"]-1e-8) & (ot <= row["t_s"]+1e-8)
            exposure = float(np.mean(radio["sinr_db"][mask] < cfg["threshold_db"]))
            policy = "C" if exposure <= cfg["exposure_c"]+1e-12 else "R" if exposure <= cfg["exposure_r"]+1e-12 else "F"
            if abs(exposure-row["exposure"]) > 1e-10 or policy != row["policy"] or row["group_size"] != 1:
                raise ValueError("exposure/policy mismatch")
        for move in moves:
            source, target = np.array(move["source"]), np.array(move["target"])
            delta = target-source
            if case["mode"] == "none" or (case["mode"] == "lateral" and delta[1]) or (case["mode"] == "vertical" and delta[0]):
                raise ValueError("move violates directional control")
            if target[0] not in spec["offsets_m"] or target[1] not in spec["heights_m"]:
                raise ValueError("target not on configured grid")
            rho = float(np.linalg.norm(delta))
            if abs(rho-move["displacement_m"]) > 1e-8:
                raise ValueError("incorrect displacement objective")
            current = obs[np.searchsorted(ot, move["t_s"]+1e-8)-1]
            if current["policy"] == "C" and not move.get("predictive_trigger", False):
                raise ValueError("gratuitous move while C")
            siblings = [e for e in events if abs(e["t_s"]-move["t_s"]) < 1e-8 and e["status"] in {"eligible", "transition_accepted"}]
            if move.get("planner") == "rolling_endpoint":
                evaluation = cfg.get("predictive_evaluation_s", 0.)
                if evaluation > 0:
                    winner = min(siblings, key=lambda e: (e["policy_cost_s"],
                        -POLICY_SCORE[e["endpoint_policy"]], e["displacement_m"],
                        e["duration_s"], *e["target"]))
                else:
                    winner = min(siblings, key=lambda e: (-POLICY_SCORE[e["endpoint_policy"]],
                        e["displacement_m"], e["duration_s"], *e["target"]))
            else:
                winner = min(siblings, key=lambda e: (e["displacement_m"], e["duration_s"], e["recovery_time_s"], *e["target"]))
            if winner != move:
                raise ValueError("selection-order mismatch")
            finish = move.get("movement_start_s", move["t_s"]) + move["duration_s"]
            if move.get("planner") == "rolling_endpoint":
                planning_horizon = cfg["predictive_rollout_s"] + cfg.get("predictive_evaluation_s", 0.)
                if move["duration_s"] > planning_horizon + 1e-8:
                    raise ValueError("rolling move exceeds prediction horizon")
                if abs(move["forecast_endpoint_s"] - (move["t_s"] + planning_horizon)) > 1e-8:
                    raise ValueError("rolling endpoint clock mismatch")
                if evaluation > 0:
                    expected = (cfg.get("policy_cost_r", 1.) * move["forecast_policy_time_s"]["R"]
                                + cfg.get("policy_cost_f", 2.) * move["forecast_policy_time_s"]["F"])
                    stay_expected = (cfg.get("policy_cost_r", 1.) * move["stay_forecast_policy_time_s"]["R"]
                                     + cfg.get("policy_cost_f", 2.) * move["stay_forecast_policy_time_s"]["F"])
                    if abs(expected-move["policy_cost_s"]) > 1e-8 or abs(stay_expected-move["stay_policy_cost_s"]) > 1e-8:
                        raise ValueError("rolling integrated policy cost mismatch")
                    if move["policy_cost_s"] >= move["stay_policy_cost_s"] - 1e-8:
                        raise ValueError("rolling move does not reduce integrated policy cost")
                elif POLICY_SCORE[move["endpoint_policy"]] <= POLICY_SCORE[move["stay_endpoint_policy"]]:
                    raise ValueError("rolling move does not upgrade endpoint policy")
            else:
                post = np.flatnonzero(ot >= finish-1e-8)
                if not len(post):
                    raise ValueError("missing post-completion history")
                until = ot[post[0]]+cfg["window_s"]
                selected = [r for r in obs if ot[post[0]]-1e-8 <= r["t_s"] <= until+1e-8]
                if selected[-1]["t_s"] < until-1e-8 or any(r["policy"] != "C" for r in selected):
                    raise ValueError("realized stable-C window failed")
            if max(move["maneuver_curvature_bound_per_m"], move["remaining_curvature_bound_per_m"]) > 1/spec["planning"]["minimum_radius_m"]+1e-12:
                raise ValueError("recorded curvature guard failed")
        expected_codes = np.array([obs[max(0, np.searchsorted(ot, t+1e-8)-1)]["policy"] for t in times])
        if not np.array_equal(expected_codes, [r["policy"] for r in trace]):
            raise ValueError("motion policy is not held between observations")
        for policy in "CRF":
            if abs(dt[expected_codes == policy].sum()-summary["policy_time_s"][policy]) > 1e-7:
                raise ValueError("policy-time summary mismatch")
        reports.append({"case": case["id"], "observations": len(obs), "motion_intervals": len(trace),
            "moves": len(moves), "position_error_m": error, "progress_error_m": progress_error,
            "sinr_error_db": signal_error, "sampled_max_curvature_per_m": float(np.max(
                np.linalg.norm(np.cross(expected_v, expected_a), axis=1)/np.linalg.norm(expected_v, axis=1)**3))})
    return {"status": "passed", "run": str(run), "run_manifest_sha256": sha(run/"manifest.json"),
        "checks": "archive hashes, independent XYZ/derivatives/progress/radio/exposure, mode/target/selection logs, realized stable C",
        "limitations": "recorded conservative bounds checked; forecast optimality not independently re-solved; no certified flight envelope",
        "cases": reports}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path); parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        raise FileExistsError("choose a new audit file")
    result = audit(args.run)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
