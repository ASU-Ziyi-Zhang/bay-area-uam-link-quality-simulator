"""Independent saved-record verifier for the restored step-2 five-aircraft study.

Reads only the archived records, the resolved configuration and the source
snapshot. It does not import the motion implementation. Sampled checks show
that no saved sample violates a bound; they are not a continuous proof, which
is what the rig's own polynomial interval audit provides. This file re-derives
the sampled facts, the Stage-0 error bands and the tightened thresholds
independently of the runner.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

WEIGHT = {"C": 0.0, "R": 1.0, "F": 2.0}
TRAJ_KEYS = ("lon_quadrature_m", "lon_fd_accel_mps2", "lat_quadrature_m", "lat_fd_accel_mps2")
REFINE_KEYS = ("lon_refine_curve_m", "lon_refine_endpoint_m", "lat_refine_curve_m", "lat_refine_endpoint_m")
# The lateral state is integrated from the analytic reference with no feedback,
# so executed and analytic values must agree to rounding. These tolerances are
# fixed here, before any formal run, and are not tightened or loosened by Stage 0.
REFERENCE_TOL = {"offset_m": 1e-6, "speed_mps": 1e-7, "accel_mps2": 1e-7}


def load(path):
    with gzip.open(path, "rt") as stream:
        return json.load(stream)


def shape_eval(shape, u, order):
    breaks = np.asarray(shape["breaks"], float)
    u = np.clip(np.atleast_1d(np.asarray(u, float)), 0.0, 1.0)
    index = np.searchsorted(breaks[1:-1], u, side="right")
    out = np.zeros_like(u)
    for k, coef in enumerate(shape["coefficients"]):
        mask = index == k
        if mask.any():
            poly = np.polynomial.Polynomial(coef)
            out[mask] = poly.deriv(order)(u[mask]) if order else poly(u[mask])
    return out


def shape_peaks(shape, samples=400001):
    u = np.linspace(0.0, 1.0, samples)
    return {key: float(np.abs(shape_eval(shape, u, order)).max())
            for order, key in ((1, "speed"), (2, "accel"), (3, "jerk"))}


def reference_lateral(profile, t):
    t = np.asarray(t, float)
    duration, distance = profile["duration_s"], profile["distance_m"]
    u = (t - profile["start_s"]) / duration
    inside = (u > 0) & (u < 1)
    d = np.where(u <= 0, 0.0, np.where(u >= 1, distance, distance * shape_eval(profile["shape"], u, 0)))
    v = np.where(inside, distance * shape_eval(profile["shape"], u, 1) / duration, 0.0)
    # Right-continuous at the start, as recorded: a C1 shape steps there (zero for C2 shapes).
    a = np.where((u >= 0) & (u < 1), distance * shape_eval(profile["shape"], u, 2) / duration**2, 0.0)
    return d, v, a


def prescribed_roles(record):
    return ("B",) if (record.get("scenario") or {}).get("leader_braking") else ()


def record_policy_cost(record, start=None, end=None):
    """Policy is observed piecewise constant: each sample holds until the next."""
    t = np.asarray(record["times_s"], float)
    if len(t) < 2:
        return 0.0
    lo = -math.inf if start is None else start
    hi = math.inf if end is None else end
    width = np.clip(np.minimum(t[1:], hi) - np.maximum(t[:-1], lo), 0.0, None)
    return float(sum(np.dot(np.array([WEIGHT[p] for p in tr["policy"]], float)[:-1], width)
                     for tr in record["trace"].values()))


def trajectory_metrics(record):
    out = {k: 0.0 for k in TRAJ_KEYS}
    out.update({f"lat_reference_{k}": 0.0 for k in REFERENCE_TOL})
    t = np.asarray(record["times_s"], float)
    if len(t) < 2:
        return out
    h = np.diff(t)
    keep = h > 1e-12
    for role, tr in record["trace"].items():
        q, v, a = (np.asarray(tr[k], float) for k in ("q_m", "v_mps", "a_mps2"))
        area = np.r_[0.0, np.cumsum(h * (v[1:] + v[:-1]) / 2)]
        out["lon_quadrature_m"] = max(out["lon_quadrature_m"], float(np.max(np.abs(area - (q - q[0])))))
        if role not in prescribed_roles(record) and keep.any():
            fd = (v[1:] - v[:-1])[keep] / h[keep]
            out["lon_fd_accel_mps2"] = max(out["lon_fd_accel_mps2"], float(np.max(np.abs(fd - a[:-1][keep]))))
    lat = record["lateral"]
    d, dv, da = (np.asarray(lat[k], float) for k in ("offset_m", "speed_mps", "accel_mps2"))
    area = np.r_[0.0, np.cumsum(h * (dv[1:] + dv[:-1]) / 2)]
    out["lat_quadrature_m"] = float(np.max(np.abs(area - (d - d[0]))))
    lat_keep = keep.copy()
    profile = record.get("profile")
    if profile:
        # A declared acceleration step (cubic Bezier ends) is recorded as its right
        # limit, so the one interval ending at that instant cannot be compared with a
        # trapezoid of the recorded acceleration. C2 shapes have no step: nothing is skipped.
        ends = shape_eval(profile["shape"], np.array([0.0, 1.0]), 2)
        for instant, jump in ((profile["start_s"], ends[0]), (profile["start_s"] + profile["duration_s"], ends[1])):
            if abs(jump) > 1e-12:
                lat_keep &= ~(np.abs(t[1:] - instant) <= 1e-9)
    if lat_keep.any():
        fd = (dv[1:] - dv[:-1])[lat_keep] / h[lat_keep]
        out["lat_fd_accel_mps2"] = float(np.max(np.abs(fd - ((da[1:] + da[:-1]) / 2)[lat_keep])))
    if record.get("profile"):
        rd, rv, ra = reference_lateral(record["profile"], t)
        for key, ref, got in (("offset_m", rd, d), ("speed_mps", rv, dv), ("accel_mps2", ra, da)):
            out[f"lat_reference_{key}"] = float(np.max(np.abs(ref - got)))
    return out


def refinement_metrics(coarse, fine):
    tc, tf = (np.asarray(r["times_s"], float) for r in (coarse, fine))
    out = {k: 0.0 for k in REFINE_KEYS}
    if len(tc) < 2 or len(tf) < 2:
        out["same_decisions"] = (len(tc) < 2) == (len(tf) < 2)
        return out
    for role in coarse["trace"]:
        qc = np.asarray(coarse["trace"][role]["q_m"], float)
        qf = np.asarray(fine["trace"][role]["q_m"], float)
        out["lon_refine_curve_m"] = max(out["lon_refine_curve_m"], float(np.max(np.abs(qc - np.interp(tc, tf, qf)))))
        out["lon_refine_endpoint_m"] = max(out["lon_refine_endpoint_m"], abs(float(qc[-1] - qf[-1])))
    dc = np.asarray(coarse["lateral"]["offset_m"], float)
    df = np.asarray(fine["lateral"]["offset_m"], float)
    out["lat_refine_curve_m"] = float(np.max(np.abs(dc - np.interp(tc, tf, df))))
    out["lat_refine_endpoint_m"] = abs(float(dc[-1] - df[-1]))
    key = lambda r: [(d.get("t_s"), d.get("action"), d.get("reason")) for d in r["decisions"]]
    out["same_decisions"] = key(coarse) == key(fine)
    return out


def separation(record, horizontal_m):
    t = np.asarray(record["times_s"], float)
    if len(t) == 0:
        return {"minimum_horizontal_m": None, "closest_pair": None, "samples_inside_volume": 0, "nmac_samples": 0}
    sep = record["scenario"]["lane_separation_m"]
    roles = list(record["trace"])
    q = {r: np.asarray(record["trace"][r]["q_m"], float) for r in roles}
    y = {r: (np.asarray(record["lateral"]["offset_m"], float) if r == "A"
             else np.asarray(record["trace"][r]["lane"], float) * sep) for r in roles}
    best, pair, inside, nmac = math.inf, None, 0, 0
    for i, a in enumerate(roles):
        for b in roles[:i]:
            dist = np.hypot(q[a] - q[b], y[a] - y[b])
            if dist.min() < best:
                best, pair = float(dist.min()), (a, b)
            inside += int((dist <= horizontal_m).sum())
            nmac += int((dist <= 152.4).sum())
    return {"minimum_horizontal_m": best, "closest_pair": pair,
            "samples_inside_volume": inside, "nmac_samples": nmac}


def envelope_peak(record, envelope):
    t = np.asarray(record["times_s"], float)
    if len(t) == 0:
        return 0.0
    lon = np.asarray(record["trace"]["A"]["a_mps2"], float)
    lat = np.asarray(record["lateral"]["accel_mps2"], float)
    limit = np.where(lon >= 0, envelope["longitudinal_accel_max_mps2"],
                     envelope["longitudinal_decel_max_mps2"])
    return float(np.max((lon / limit) ** 2 + (lat / envelope["lateral_accel_max_mps2"]) ** 2))


def decision_problems(record):
    gate = record.get("gate")
    problems = []
    for d in record["decisions"]:
        candidates = d.get("candidates")
        if not candidates:
            continue
        admitted = [c["cost"] for c in candidates if c["admitted"]]
        reason = d.get("reason")
        if reason == "no_admissible_candidate" and admitted:
            problems.append(f"t={d['t_s']}: 'no admissible' although {len(admitted)} admitted")
        if reason in ("gain_above_threshold", "future_start", "gain_below_threshold") and not admitted:
            problems.append(f"t={d['t_s']}: {reason} with no admitted candidate")
        if admitted and d.get("gain") is not None:
            # Selection used only admitted candidates iff the recorded gain is
            # the stay cost minus the cheapest *admitted* candidate.
            if abs(d["gain"] - (d["baseline_cost"] - min(admitted))) > 1e-9:
                problems.append(f"t={d['t_s']}: gain does not come from the cheapest admitted candidate")
            if gate is not None:
                above = d["gain"] > gate
                if reason in ("gain_above_threshold", "future_start") and not above:
                    problems.append(f"t={d['t_s']}: {reason} with gain {d['gain']:.6f} <= gate {gate:.6f}")
                if reason == "gain_below_threshold" and above:
                    problems.append(f"t={d['t_s']}: gain {d['gain']:.6f} above gate {gate:.6f} but stayed")
    return problems


def expected_checks(record, expected):
    decisions = [d for d in record["decisions"] if d.get("reason") != "before_first_decision_phase"]
    reasons = [d.get("reason") for d in decisions]
    changes = [d for d in decisions if d.get("reason") == "gain_above_threshold"]
    ok = {}
    if "first_action" in expected:
        ok["expected_first_action"] = bool(decisions) and decisions[0].get("action") == expected["first_action"]
    if "first_reason" in expected:
        ok["expected_first_reason"] = bool(decisions) and decisions[0].get("reason") == expected["first_reason"]
    if expected.get("never_change"):
        ok["expected_never_change"] = not changes
    if "reason_present" in expected:
        ok["expected_reason_present"] = expected["reason_present"] in reasons
    return ok


def crossing_and_exposure(record, half_width_m, threshold_db):
    t = np.asarray(record["times_s"], float)
    d = np.asarray(record["lateral"]["offset_m"], float)
    crossed = d >= half_width_m
    cross = float(t[np.argmax(crossed)]) if crossed.any() else None
    bad = sum(1 for o in record["observations"] if o["role"] == "A" and o["sinr_db"] < threshold_db)
    return cross, bad


def derive_stage0(records, spec):
    s0, numerics = spec["numerics"]["stage0"], spec["numerics"]
    dts = sorted(s0["dt_s"])
    main, finer = numerics["refinement_dt_s"]
    cost = record_policy_cost
    benefit = {dt: cost(records[f"S0__A1__stay__dt{dt:g}"]) - cost(records[f"S0__A1__allow__dt{dt:g}"])
               for dt in dts}
    delta_int = abs(benefit[dts[0]] - benefit[dts[1]])
    stay_main = cost(records[f"S0__A1__stay__dt{main:g}"])
    phase_benefit = {0.0: benefit[main]}
    for phase in s0["decision_phases_s"]:
        if phase:
            phase_benefit[float(phase)] = stay_main - cost(records[f"S0__A1__phase{phase:g}__dt{main:g}"])
    delta_phase = max(phase_benefit.values()) - min(phase_benefit.values())
    allow, stay = records[f"S0__A1__allow__dt{main:g}"], records[f"S0__A1__stay__dt{main:g}"]
    first = next(d for d in allow["decisions"] if d.get("reason") == "gain_above_threshold")
    executed_gain = (cost(stay, first["t_s"], first["prediction_end_s"])
                     - cost(allow, first["t_s"], first["prediction_end_s"]))
    delta_pred = abs(first["gain"] - executed_gain)
    # A record that stops before the prediction window ends truncates the
    # executed gain and inflates delta_pred; such a Stage 0 is not usable.
    covered = min(allow["times_s"][-1], stay["times_s"][-1]) >= first["prediction_end_s"] - 1e-9
    measured = {k: 0.0 for k in TRAJ_KEYS + REFINE_KEYS}
    for name, rec in records.items():
        if name.endswith((f"__dt{main:g}", f"__dt{finer:g}")):
            m = trajectory_metrics(rec)
            for k in TRAJ_KEYS:
                measured[k] = max(measured[k], m[k])
    pairs = [("S0__A1__allow", "S0__A1__allow")] + [
        (n.rsplit("__dt", 1)[0],) * 2 for n in records if n.startswith("S0__shape_") and n.endswith(f"__dt{main:g}")]
    for stem, _ in pairs:
        r = refinement_metrics(records[f"{stem}__dt{main:g}"], records[f"{stem}__dt{finer:g}"])
        for k in REFINE_KEYS:
            measured[k] = max(measured[k], r[k])
    declared = spec["acceptance"]["declared"]
    thresholds = {k: min(declared[k], max(10.0 * measured[k], 1e-6)) for k in TRAJ_KEYS + REFINE_KEYS}
    return {"benefit_by_dt": {f"{k:g}": v for k, v in benefit.items()},
            "benefit_by_phase": {f"{k:g}": v for k, v in phase_benefit.items()},
            "delta_int": delta_int, "delta_phase": delta_phase, "delta_pred": delta_pred,
            "prediction_window_covered": covered,
            "first_decision": {"t_s": first["t_s"], "prediction_end_s": first["prediction_end_s"],
                               "predicted_gain": first["gain"], "executed_gain": executed_gain},
            "measured": measured, "declared": {k: declared[k] for k in TRAJ_KEYS + REFINE_KEYS},
            "thresholds": thresholds,
            "gates": {"revised": delta_int + delta_pred, "original": delta_int + delta_phase + delta_pred}}


def verify(folder):
    folder = Path(folder)
    spec = json.loads((folder / "resolved_config.json").read_text())
    manifest = json.loads((folder / "manifest.json").read_text())
    failures = []
    with zipfile.ZipFile(folder / "source_snapshot.zip") as archive:
        for row in manifest["sources"]:
            if hashlib.sha256(archive.read(row["path"])).hexdigest() != row["sha256"]:
                failures.append(f"source hash mismatch: {row['path']}")
    records = {p.name[:-len(".json.gz")]: load(p) for p in sorted(folder.glob("*.json.gz"))}
    stage0_saved = json.loads((folder / "stage0.json").read_text())
    stage0 = derive_stage0({k: v for k, v in records.items() if k.startswith("S0__")}, spec)
    for key in ("delta_int", "delta_phase", "delta_pred"):
        if abs(stage0[key] - stage0_saved[key]) > 1e-9:
            failures.append(f"stage0 {key}: saved {stage0_saved[key]} vs re-derived {stage0[key]}")
    for group in ("thresholds", "gates"):
        for key, value in stage0[group].items():
            if abs(value - stage0_saved[group][key]) > 1e-12:
                failures.append(f"stage0 {group}.{key}: saved {stage0_saved[group][key]} vs re-derived {value}")
    if not stage0["prediction_window_covered"]:
        failures.append("stage0: executed records end before the first prediction window; delta_pred truncated")
    thr, gates = stage0["thresholds"], stage0["gates"]
    envelope = spec["resolved_objects"]["envelope"]
    horizontal = spec["parameters"]["admission_volume"]["horizontal_m"]
    threshold_db = spec["parameters"]["traffic"]["threshold_db"]
    expected = {c["id"]: c["expected"] for c in spec["part_A"]}
    results = []
    for name, rec in records.items():
        if name.startswith("S0__"):
            continue
        checks, notes = {}, {}
        m = trajectory_metrics(rec)
        for k in TRAJ_KEYS:
            checks[k] = m[k] <= thr[k]
        for k, tol in REFERENCE_TOL.items():
            checks[f"lat_reference_{k}"] = m[f"lat_reference_{k}"] <= tol
        eta = envelope_peak(rec, envelope)
        checks["joint_envelope"] = eta <= spec["acceptance"]["declared"]["eta_max"] + spec["acceptance"]["declared"]["eta_tolerance"]
        sep = separation(rec, horizontal)
        # The admission volume is now the NMAC cylinder itself, and a failed
        # audit no longer stops the run. Entering it is therefore a result to
        # report, not a validation failure; what must not happen is a record
        # that shows samples inside the volume while reporting no safety event.
        checks["safety_events_recorded"] = (sep["samples_inside_volume"] == 0
                                            or bool(rec.get("safety_events")))
        if sep["samples_inside_volume"]:
            notes["samples_inside_admission_volume"] = sep["samples_inside_volume"]
        if sep["nmac_samples"]:
            notes["nmac_samples"] = sep["nmac_samples"]
        checks["policy_cost_recomputed"] = abs(record_policy_cost(rec) - rec["actual_policy_cost"]) <= 1e-9 * max(1.0, abs(rec["actual_policy_cost"]))
        profile = rec.get("profile")
        if profile and rec["times_s"] and rec["times_s"][-1] >= profile["start_s"] + profile["duration_s"]:
            peaks = shape_peaks(profile["shape"])
            closed = peaks["speed"] * abs(profile["distance_m"]) / profile["duration_s"]
            executed = float(np.max(np.abs(rec["lateral"]["speed_mps"])))
            checks["lateral_peak_matches_closed_form"] = abs(executed - closed) <= spec["acceptance"]["declared"]["lateral_peak_rel"] * closed
            notes["lateral_speed_peak"] = {"executed": executed, "closed_form": closed}
        problems = decision_problems(rec)
        checks["decisions_consistent"] = not problems
        if problems:
            notes["decision_problems"] = problems
        if rec["part"] == "A" and rec["variant"].startswith("allow"):
            checks.update(expected_checks(rec, expected[rec["case_id"]]))
        if rec["part"] == "A" and rec["variant"] == "stay":
            checks["stay_never_changes"] = all(d.get("action") != "change" for d in rec["decisions"])
        if rec["part"] in ("B1", "B2", "C1", "C2") and rec["times_s"]:
            cross, bad = crossing_and_exposure(rec, rec["scenario"]["lane_separation_m"] / 2, threshold_db)
            saved = rec.get("metrics", {})
            checks["crossing_recomputed"] = saved.get("crossing_s") == cross
            checks["bad_samples_recomputed"] = saved.get("ego_bad_samples") == bad
        for k, ok in checks.items():
            if not ok:
                failures.append(f"{name}: {k}")
        results.append({"name": name, "part": rec["part"], "case_id": rec["case_id"],
                        "variant": rec["variant"], "dt_s": rec["dt_s"], "status": rec["status"],
                        "gate": rec.get("gate"), "checks": checks, "metrics": m, "joint_envelope_peak": eta,
                        "separation": sep, "actual_policy_cost": rec["actual_policy_cost"], "notes": notes})
    main, finer = spec["numerics"]["refinement_dt_s"]
    refinement = []
    for name, rec in records.items():
        if name.startswith("S0__") or not name.endswith(f"__dt{main:g}"):
            continue
        partner = records.get(name.rsplit("__dt", 1)[0] + f"__dt{finer:g}")
        if partner is None:
            continue
        r = refinement_metrics(rec, partner)
        passed = all(r[k] <= thr[k] for k in REFINE_KEYS) and r["same_decisions"]
        refinement.append({"name": name.rsplit("__dt", 1)[0], "steps_s": [main, finer], **r, "passed": passed})
        if not passed:
            failures.append(f"{name}: refinement {main}->{finer}")
    return {"status": "passed" if not failures else "failed", "record_count": len(results),
            "failures": failures, "stage0": stage0, "results": results, "refinement": refinement,
            "scope": "saved-record checks, re-derived error bands and thresholds; sampled, not a continuous "
                     "safety proof; not physical calibration"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = verify(args.run)
    print(json.dumps({k: report[k] for k in ("status", "record_count", "failures")}, indent=2, ensure_ascii=False))
    raise SystemExit(0 if report["status"] == "passed" else 1)
