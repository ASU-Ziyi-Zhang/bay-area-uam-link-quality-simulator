"""Independent saved-trace verifier; does not import the motion implementation."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np


def verify(folder):
    folder = Path(folder)
    spec = json.loads((folder / "resolved_config.json").read_text())
    manifest = json.loads((folder / "manifest.json").read_text())
    with zipfile.ZipFile(folder / "source_snapshot.zip") as archive:
        for row in manifest["sources"]:
            assert hashlib.sha256(archive.read(row["path"])).hexdigest() == row["sha256"], row["path"]
    cases = {c["id"]: c for c in spec["cases"]}
    thresholds = spec["acceptance"]
    results, groups, failures = [], {}, []
    for case_id, case in cases.items():
        for mode in ("acc", "aks"):
            for step in spec["numerics"]["steps_s"]:
                path = folder / f"{case_id}__{mode}__dt{step:g}.json.gz"
                with gzip.open(path, "rt") as stream:
                    data = json.load(stream)
                tr = data["trace"]
                array = lambda key: np.array([r[key] for r in tr], float)
                t, x, v, a, gap = (array(k) for k in ("t_s", "follower_x_m", "follower_v_mps", "acceleration_mps2", "gap_m"))
                vehicle = data["resolved"]["vehicle"]
                checks = {}
                def check(name, value):
                    checks[name] = bool(value)
                check("finite", all(np.isfinite(z).all() for z in (t,x,v,a,gap)))
                check("increasing_time", (np.diff(t)>0).all())
                check("speed_bounds", v.min() >= vehicle["speed_min_mps"]-1e-8 and v.max() <= vehicle["speed_max_mps"]+1e-8)
                check("acceleration_bounds", a.min() >= -vehicle["deceleration_limit_mps2"]-1e-8 and a.max() <= vehicle["acceleration_limit_mps2"]+1e-8)
                check("exploratory_gap_floor", gap.min() >= thresholds["exploratory_gap_floor_m"])
                check("gap_identity", np.max(np.abs(array("leader_x_m")-x-gap)) < 1e-8)
                check("negligible_projection", data["maximum_speed_projection_mps"] <= thresholds["projection_mps"])
                integrated = np.r_[0, np.cumsum(np.diff(t)*(v[1:]+v[:-1])/2)]
                quad_error = float(np.max(np.abs(integrated-(x-x[0]))))
                check("independent_displacement_quadrature", quad_error <= thresholds["quadrature_error_m"])
                dvdt = np.diff(v)/np.diff(t)
                avg_a = (a[1:]+a[:-1])/2
                keep = np.ones(len(dvdt), bool)
                # Skip only the intervals the run declares as acceleration
                # discontinuities. Previously this read the event log, which
                # excluded intervals on the strength of a decision having been
                # recorded there rather than the acceleration actually jumping.
                for moment in data["discontinuities"]:
                    if moment > 0:
                        keep &= ~((t[:-1] <= moment+1e-9) & (t[1:] >= moment-1e-9))
                fd_error = float(np.max(np.abs(dvdt[keep]-avg_a[keep])))
                check("independent_acceleration_difference", fd_error <= thresholds["acceleration_fd_error_mps2"])
                request = data["request"]
                check("request_from_actual_state", request["status"] == case["expected_request"])
                target_error = float(gap[-1]-request["target_gap_m"])
                spacing = data["resolved"]["spacing"]
                tau = spacing[{"C":"tau_c_s","R":"tau_r_s","F":"tau_f_s"}[case["target_policy"]]]
                instantaneous_target = spacing["d0_m"]+tau*v[-1]+spacing["buffer_s2_per_m"]*v[-1]**2
                # Recovery endpoint and duration are measured identically for
                # both controllers, against the final cruise-speed equilibrium.
                good = ((abs(gap-request["target_gap_m"]) <= thresholds["terminal_gap_error_m"])
                        & (abs(v-vehicle["cruise_mps"]) <= thresholds["terminal_speed_error_mps"]))
                bad = np.flatnonzero(~good)
                settled_index = int(bad[-1]+1) if bad.size else 0
                settling = float(t[settled_index]) if settled_index < len(t) else None
                controller = data["resolved"]["controller"]
                recovery = controller.get("acc_policy_recovery", False)
                if recovery and mode == "acc":
                    established = case.get("established_following", "initial_gap_policy" in case)
                    check("recovery_relationship_gate", data["acc_policy_recovery_active"] == established)
                    targets = spacing["d0_m"]+tau*v+spacing["buffer_s2_per_m"]*v**2
                    following = (controller["k_gap_per_s2"]*(gap-targets)
                                 + controller["k_relative_per_s"]*(array("leader_v_mps")-v))
                    free = controller["k_speed_per_s"]*(vehicle["cruise_mps"]-v)
                    expected = following if established else np.minimum(free, following)
                    expected = np.clip(expected, -vehicle["deceleration_limit_mps2"], vehicle["acceleration_limit_mps2"])
                    expected[((v <= vehicle["speed_min_mps"]+1e-9) & (expected < 0))
                             | ((v >= vehicle["speed_max_mps"]-1e-9) & (expected > 0))] = 0
                    check("independent_acc_command", np.max(abs(a-expected)) < 1e-7)
                    if request["status"] == "planned" and not case.get("leader_braking"):
                        check("recovery_terminal_gap", abs(target_error) <= thresholds["terminal_gap_error_m"])
                        check("recovery_terminal_speed", abs(v[-1]-vehicle["cruise_mps"]) <= thresholds["terminal_speed_error_mps"])
                        if request["target_reduction_m"] > 0:
                            check("closing_accelerates", a[0] > 0 and v.max() > vehicle["cruise_mps"]+.1)
                if request["status"] == "planned":
                    ref = request["transition_parameters"]
                    t1, t2 = ref["alpha"]*ref["duration"], (1-ref["alpha"])*ref["duration"]
                    dx = (ref["v1"]+ref["junction"])*t1/2 + (ref["junction"]+ref["v2"])*t2/2
                    check("reference_closure", abs(dx - (ref["v1"]*ref["duration"]+request["target_reduction_m"])) < 1e-7)
                    check("reference_speed_bounds", vehicle["speed_min_mps"]-1e-8 <= ref["junction"] <= vehicle["speed_max_mps"]+1e-8)
                    dv1, dv2 = ref["junction"]-ref["v1"], ref["v2"]-ref["junction"]
                    for index, (dv, duration) in enumerate(((dv1,t1),(dv2,t2))):
                        limit = vehicle["acceleration_limit_mps2"] if dv > 0 else vehicle["deceleration_limit_mps2"]
                        check(f"reference_phase_{index+1}_acceleration", 1.5*abs(dv)/duration <= limit+1e-8)
                    if vehicle["reference_jerk_limit_mps3"] is not None:
                        check("reference_jerk", max(6*abs(dv1)/t1**2,6*abs(dv2)/t2**2) <= vehicle["reference_jerk_limit_mps3"]+1e-8)
                    if mode == "aks" and not case.get("leader_braking"):
                        check("terminal_gap", abs(target_error) <= thresholds["terminal_gap_error_m"])
                        check("terminal_speed", abs(v[-1]-vehicle["cruise_mps"]) <= thresholds["terminal_speed_error_mps"])
                        # Include the action endpoint, not just the post-action tail.
                        i = int(np.argmin(abs(t-ref["duration"])))
                        check("action_endpoint_gap", abs(gap[i]-request["target_gap_m"]) <= thresholds["terminal_gap_error_m"])
                        check("action_endpoint_speed", abs(v[i]-vehicle["cruise_mps"]) <= thresholds["terminal_speed_error_mps"])
                if case_id == "oversized_gap_hold":
                    check("hold_no_spurious_action", np.max(abs(v-vehicle["cruise_mps"])) < 1e-6)
                if case.get("leader_braking"):
                    check("responds_to_leader", v.min() < vehicle["cruise_mps"]-1)
                    if mode == "aks":
                        check("invalidates_reference", any(e["event"] == "reference_invalidated_leader_changed" and abs(e["t_s"]-case["leader_braking"]["start_s"])<1e-8 for e in data["events"]))
                if case_id == "close_without_headroom" and mode == "aks":
                    check("refusal_uses_acc", all(r["controller"] == "acc_fallback" for r in tr))
                for name, passed in checks.items():
                    if not passed:
                        failures.append(f"{case_id}/{mode}/{step}: {name}")
                results.append({"case_id":case_id,"mode":mode,"dt_s":step,"checks":checks,
                    "request_status":request["status"],"duration_s":request["duration_s"],
                    "observation_end_s": float(t[-1]),
                    "settling_time_s": settling,
                    "gap_error_at_reference_end_m": float(np.interp(request["duration_s"],t,gap)-request["target_gap_m"]),
                    "gap_error_at_legacy_horizon_m": float(np.interp(request["duration_s"]+120,t,gap)-request["target_gap_m"]),
                    "final_gap_error_m":target_error,"minimum_gap_m":float(gap.min()),
                    "final_gap_m":float(gap[-1]),"final_speed_mps":float(v[-1]),
                    "final_instantaneous_target_gap_m":float(instantaneous_target),
                    "final_instantaneous_gap_error_m":float(gap[-1]-instantaneous_target),
                    "minimum_speed_mps":float(v.min()),"maximum_speed_mps":float(v.max()),
                    "minimum_acceleration_mps2":float(a.min()),"maximum_acceleration_mps2":float(a.max()),
                    "displacement_quadrature_error_m":quad_error,"acceleration_fd_error_mps2":fd_error})
                groups.setdefault((case_id,mode),[]).append((step,t,gap,data["events"]))
    refinement = []
    for (case_id,mode), members in groups.items():
        members.sort(key=lambda row: row[0], reverse=True)
        for coarse, fine in zip(members,members[1:]):
            error = float(np.max(abs(coarse[2]-np.interp(coarse[1],fine[1],fine[2]))))
            endpoint = abs(float(coarse[2][-1]-fine[2][-1]))
            passed = error <= thresholds["refinement_gap_curve_m"] and endpoint <= thresholds["refinement_endpoint_gap_m"] and coarse[3]==fine[3]
            refinement.append({"case_id":case_id,"mode":mode,"steps_s":[coarse[0],fine[0]],
                "maximum_gap_curve_difference_m":error,"endpoint_gap_difference_m":endpoint,"passed":passed})
            if not passed:
                failures.append(f"{case_id}/{mode}: refinement {coarse[0]}->{fine[0]}")
    return {"status":"passed" if not failures else "failed", "trace_count":len(results),
            "failures":failures,"results":results,"refinement":refinement,
            "scope":"saved-trace and analytic-reference checks; not physical calibration or continuous safety proof"}


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("run",type=Path)
    args=parser.parse_args()
    report=verify(args.run)
    print(json.dumps(report,indent=2,allow_nan=False))
    raise SystemExit(0 if report["status"]=="passed" else 1)
