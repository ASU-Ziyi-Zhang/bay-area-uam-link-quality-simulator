"""Field equivalence, legacy preservation and step-refinement comparisons."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import zipfile

import numpy as np

from verify_lateral_study import independent_reference, read
from plot_wide_radio_map import radio_at, sha


def compare(run):
    run = Path(run).resolve(); manifest = read(run/"manifest.json")
    if manifest["status"] != "completed":
        raise ValueError("completed run required")
    for entry in manifest["outputs"]:
        if sha(run/entry["path"]) != entry["sha256"]:
            raise ValueError("run output hash mismatch")
    spec = read(run/"resolved_config.json")
    with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
        frame = independent_reference(archive, manifest, spec["parameters"]["center_control_step_m"])
    with np.load(run/"field.npz") as archive:
        field = {key: archive[key].copy() for key in archive.files}
    Q, D, H = np.meshgrid(field["q_m"], field["offsets_m"], field["heights_m"], indexing="ij")
    center, _, normal, _, _ = frame(Q.ravel())
    xyz = np.c_[center + D.ravel()[:, None]*normal, H.ravel()]
    radio = radio_at(xyz, spec)
    field_error = float(np.max(abs(radio["sinr_db"].reshape(Q.shape)-field["sinr_db"])))
    association_match = np.array_equal(radio["serving_bs"].reshape(Q.shape), field["serving_bs"])
    rc = spec["radio"]
    stations = np.array([[s["x_m"], s["y_m"], s["height_m"] if s["height_m"] is not None
                          else rc["assumed_bs_height_m"]] for s in spec["base_stations"]])
    squared = np.sum((xyz[:, None, :]-stations[None, :, :])**2, axis=-1).min(axis=1)
    rsrp = (rc["eirp_dbm"]+rc["receiver_gain_db"]-28-11*np.log10(squared)
            -20*np.log10(rc["frequency_ghz"])-10*np.log10(rc["resource_elements"]))
    rsrp_error = float(np.max(abs(rsrp.reshape(Q.shape)-field["rsrp_dbm"])))
    if field_error > 1e-8 or rsrp_error > 1e-8 or not association_match:
        raise ValueError("independent spatial field mismatch")
    record = {"run_manifest_sha256": sha(run/"manifest.json"), "status": "passed",
              "field_samples": int(Q.size), "field_sinr_error_db": field_error, "field_rsrp_error_db": rsrp_error,
              "field_association_match": association_match, "legacy": [], "refinements": [], "motion_summaries": []}
    center_profile = field["sinr_db"][:, list(field["offsets_m"]).index(0), list(field["heights_m"]).index(spec["parameters"]["altitude_m"])]
    worst = int(np.argmin(center_profile)); plane = field["sinr_db"][worst]
    di, hi = np.unravel_index(np.argmax(plane), plane.shape)
    record["worst_baseline_section"] = {"q_m": float(field["q_m"][worst]),
        "fixed_sinr_db": float(center_profile[worst]), "best_sampled_sinr_db": float(plane.max()),
        "best_sampled_offset_height_m": [float(field["offsets_m"][di]), float(field["heights_m"][hi])],
        "weak_sampled_points": int((plane < spec["parameters"]["threshold_db"]).sum()), "sampled_points": int(plane.size)}
    # Independent Gaussian quadrature instead of the simulator's Simpson sum.
    nodes, weights = np.polynomial.legendre.leggauss(64)
    u = (nodes+1)/2
    for case in spec["cases"]:
        folder = run/case["id"]; cfg = case["parameters"]
        trace = read(folder/"trace.json.gz"); summary = read(folder/"summary.json")
        moves = [e for e in read(folder/"events.json") if e["status"] == "transition_accepted"]
        qs = np.array([r["q_m"] for r in trace]); pos = np.array([r["position_m"] for r in trace])
        c, _, n, _, _ = frame(qs)
        displayed = np.array([[r["offset_m"], r["altitude_m"]] for r in trace])
        reconstructed = np.c_[np.sum((pos[:, :2]-c)*n, axis=1), pos[:, 2]]
        coordinate_error = float(np.max(abs(displayed-reconstructed)))
        extra, displacement = 0., 0.
        for e in moves:
            rho = np.linalg.norm(np.array(e["target"])-e["source"]); T = e["duration_s"]
            speed = np.sqrt(cfg["cruise_mps"]**2+(rho*30*u*u*(1-u)**2/T)**2)
            extra += T/2*float(weights @ (speed-cfg["cruise_mps"]))
            displacement += float(rho)
        path = cfg["cruise_mps"]*summary["corridor_time_s"]+extra
        if coordinate_error > 1e-6 or abs(path-summary["path_length_m"]) > 1e-6 or abs(displacement-summary["total_displacement_m"]) > 1e-8:
            raise ValueError("coordinate fields or motion-cost summary mismatch")
        record["motion_summaries"].append({"case": case["id"], "coordinate_error_m": coordinate_error,
             "path_length_quadrature_error_m": float(path-summary["path_length_m"]),
             "transverse_speed_extra_distance_m": extra})
    legacy = run.parent/"R0009"
    if legacy.exists():
        record["legacy_manifest_sha256"] = sha(legacy/"manifest.json")
        for name, old_name in [("fixed", "static_plus_0"), ("lateral", "minimal_900")]:
            old = read(legacy/old_name/"trace.json.gz"); new = read(run/name/"trace.json.gz")
            keys = ["t_s", "end_s", "q_m", "position_m", "velocity_mps", "acceleration_mps2", "policy", "exposure"]
            exact = len(old) == len(new) and all(all(a[k] == b[k] for k in keys) for a, b in zip(old, new))
            record["legacy"].append({"case": name, "old_case": old_name, "exact_trace_match": exact})
            if not exact:
                raise ValueError("legacy control changed")
        with np.load(legacy/"field.npz") as archive:
            plane = field["sinr_db"][:, :, list(field["heights_m"]).index(300)].T
            record["legacy_300m_field_exact_match"] = np.array_equal(plane, archive["sinr_db"])
            if not record["legacy_300m_field_exact_match"]:
                raise ValueError("legacy 300 m radio field changed")
    reference = read(run/"joint/observations.json.gz")
    summary = read(run/"joint/summary.json")
    moves = [e for e in read(run/"joint/events.json") if e["status"] == "transition_accepted"]
    for name in ["joint_motion_refined", "joint_forecast_refined"]:
        obs = read(run/name/"observations.json.gz"); local = read(run/name/"summary.json")
        accepted = [e for e in read(run/name/"events.json") if e["status"] == "transition_accepted"]
        same_clock = len(reference) == len(obs) and np.allclose([r["t_s"] for r in reference], [r["t_s"] for r in obs], atol=1e-8, rtol=0)
        keys = ["t_s", "source", "target", "duration_s"]
        same_decisions = len(moves) == len(accepted) and all(all(a[k] == b[k] for k in keys) for a, b in zip(moves, accepted))
        row = {"case": name, "same_observation_clock": bool(same_clock), "same_move_decisions": same_decisions,
               "exit_time_difference_s": local["corridor_time_s"]-summary["corridor_time_s"],
               "F_share_difference": local["policy_time_shares"]["F"]-summary["policy_time_shares"]["F"],
               "path_length_difference_m": local["path_length_m"]-summary["path_length_m"]}
        if same_clock:
            row.update(maximum_q_difference_m=float(np.max(abs(np.array([r["q_m"] for r in reference])-[r["q_m"] for r in obs]))),
                       maximum_sinr_difference_db=float(np.max(abs(np.array([r["sinr_db"] for r in reference])-[r["sinr_db"] for r in obs]))),
                       same_policy_sequence=all(a["policy"] == b["policy"] for a, b in zip(reference, obs)))
        record["refinements"].append(row)
    # Rejection explanations come from saved decisions, not newly invented probes.
    selected_vertical = [e for e in read(run/"vertical/events.json") if e["status"] == "transition_accepted"]
    if selected_vertical:
        from collections import Counter
        t = selected_vertical[0]["t_s"]
        siblings = [e for e in read(run/"vertical/events.json") if e["t_s"] == t]
        record["first_vertical_decision"] = {"t_s": t, "selected": selected_vertical[0], "targets": []}
        for target in sorted({tuple(e["target"]) for e in siblings}):
            rows = [e for e in siblings if tuple(e["target"]) == target]
            record["first_vertical_decision"]["targets"].append({"target": list(target),
                "status_counts": dict(Counter(e["status"] for e in rows)),
                "eligible_durations_s": [e["duration_s"] for e in rows if e["status"] in {"eligible", "transition_accepted"}]})
    # Describe the actual searched action set without claiming that larger
    # displacement shells were evaluated after a smaller eligible shell won.
    from collections import Counter
    joint_events = read(run/"joint/events.json")
    tested = [e for e in joint_events if e.get("direction")]
    record["joint_search"] = {
        "evaluated_endpoint_duration_records": len(tested),
        "by_direction": dict(Counter(e["direction"] for e in tested)),
        "by_status": dict(Counter(e["status"] for e in tested)),
        "selected_moves": moves,
        "note": "Each record is an endpoint-duration alternative, not an independent flight or a unique maneuver request."
    }
    fixed_obs = read(run/"fixed/observations.json.gz")
    threshold = spec["parameters"]["threshold_db"]
    record["paired_communication"] = []
    for name in ["lateral", "vertical", "joint"]:
        compared = read(run/name/"observations.json.gz")
        if [r["t_s"] for r in fixed_obs] != [r["t_s"] for r in compared]:
            raise ValueError("paired diagnostic requires common policy clocks")
        changed = [{"t_s": a["t_s"], "fixed_sinr_db": a["sinr_db"], "case_sinr_db": b["sinr_db"],
                    "case_offset_m": b["offset_m"], "case_altitude_m": b["altitude_m"]}
                   for a, b in zip(fixed_obs, compared) if (a["sinr_db"] < threshold) != (b["sinr_db"] < threshold)]
        policy_times = [a["t_s"] for a, b in zip(fixed_obs, compared) if a["policy"] != b["policy"]]
        record["paired_communication"].append({"case": name, "different_weak_flags": changed,
                                               "different_policy_times_s": policy_times})
    return record


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path); parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    snapshot = args.output.with_suffix(".zip")
    if args.output.exists() or snapshot.exists():
        raise FileExistsError("analysis outputs are immutable")
    result = compare(args.run)
    sources = [Path(__file__).resolve(), Path(__file__).with_name("verify_lateral_study.py"), Path(__file__).with_name("plot_wide_radio_map.py")]
    with zipfile.ZipFile(snapshot, "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.name)
    result.update(source_zip_sha256=sha(snapshot), sources=[{"name": p.name, "sha256": sha(p)} for p in sources])
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
