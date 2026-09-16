"""Independent archive, geometry, radio, group-policy and ACC audit for Step 3."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from plot_wide_radio_map import radio_at, sha
from verify_lateral_study import independent_reference, read


def arc_tables(frame, offsets, length_m, sample_m=0.25):
    # The verifier deliberately reconstructs the reference as a plain callable,
    # independent of the simulator class.  A dense grid is sufficient for this
    # secondary quadrature and avoids relying on simulator-owned control knots.
    q = np.linspace(0, length_m, int(np.ceil(length_m / sample_m)) + 1)
    _, _, _, norm, curvature = frame(q)
    tables = []
    for offset in offsets:
        values = norm * (1 - curvature * offset)
        tables.append(np.r_[0.0, np.cumsum((values[:-1] + values[1:])
                                           * np.diff(q) / 2)])
    return q, np.asarray(tables)


def arc_position(q, lane, q_grid, tables):
    return float(np.interp(q, q_grid, tables[lane]))


def spacing(cfg, policy, speed):
    tau = {"C": cfg["tau_c_s"], "R": cfg["tau_r_s"], "F": cfg["tau_f_s"]}[policy]
    return cfg["d0_m"] + tau * speed + cfg["buffer_s2_per_m"] * speed**2


def flow_coordinates(spec):
    traffic = spec["traffic"]
    if "flow_coordinates_m" in traffic:
        return np.asarray(traffic["flow_coordinates_m"], float)
    offsets = np.asarray(traffic["offsets_m"], float)
    return np.c_[offsets, np.full(len(offsets), spec["parameters"]["altitude_m"])]


def adjacent_flow(source, target, coordinates):
    lateral = {value: index for index, value in enumerate(sorted(set(coordinates[:, 0])))}
    vertical = {value: index for index, value in enumerate(sorted(set(coordinates[:, 1])))}
    a, b = coordinates[source], coordinates[target]
    return max(abs(lateral[a[0]] - lateral[b[0]]),
               abs(vertical[a[1]] - vertical[b[1]])) == 1


def audit_case(run, case, spec, frame, length_m):
    folder = run / case
    trace = read(folder / "trace.json.gz")
    observations = read(folder / "observations.json.gz")
    events = read(folder / "events.json")
    summary = read(folder / "summary.json")
    cfg = spec["parameters"]
    coordinates = flow_coordinates(spec)
    offsets = coordinates[:, 0]
    q_grid, tables = arc_tables(frame, offsets, length_m)
    trace_index = {(row["aircraft_id"], round(row["t_s"], 8)): row for row in trace}
    entry_nominal = {row["aircraft_id"]: row["lane"] for row in events
                     if row["status"] == "corridor_entry"}
    for row in trace:
        if "nominal_lane" in row and row["nominal_lane"] != entry_nominal[row["aircraft_id"]]:
            raise ValueError(f"{case}: nominal lane changed after entry")

    q = np.asarray([row["q_m"] for row in trace])
    d = np.asarray([row["offset_m"] for row in trace])
    h = np.asarray([row["altitude_m"] for row in trace])
    center, tangent, normal, _, _ = frame(q)
    xyz = np.c_[center + d[:, None] * normal, h]
    position_error = float(np.max(np.linalg.norm(xyz -
        np.asarray([row["position_m"] for row in trace]), axis=1)))
    if position_error > 1e-6:
        raise ValueError(f"{case}: geographic position mismatch")
    transverse_v = np.asarray([row["transverse_velocity_mps"] for row in trace])
    expected_velocity = np.c_[
        np.asarray([row["longitudinal_speed_mps"] for row in trace])[:, None]
        * tangent + transverse_v[:, 0, None] * normal,
        transverse_v[:, 1]]
    velocity_error = float(np.max(np.linalg.norm(expected_velocity -
        np.asarray([row["velocity_mps"] for row in trace]), axis=1)))
    if velocity_error > 1e-8:
        raise ValueError(f"{case}: geographic velocity mismatch")

    obs_positions = []
    for row in observations:
        motion = trace_index[(row["aircraft_id"], round(row["t_s"], 8))]
        obs_positions.append(motion["position_m"])
    signals = radio_at(np.asarray(obs_positions), spec)
    signal_error = float(np.max(np.abs(signals["sinr_db"] -
        np.asarray([row["sinr_db"] for row in observations]))))
    if signal_error > 1e-8 or not np.array_equal(signals["serving_bs"],
                                                 [row["serving_bs"] for row in observations]):
        raise ValueError(f"{case}: radio mismatch")

    observation_times = sorted({row["t_s"] for row in observations})
    histories = {}
    observation_index = {(row["aircraft_id"], row["t_s"]): row for row in observations}
    maximum_exposure_error = 0.0
    for time in observation_times:
        rows = [row for row in observations if row["t_s"] == time]
        by_lane = {}
        for row in rows:
            motion = trace_index[(row["aircraft_id"], round(time, 8))]
            by_lane.setdefault(row["lane"], []).append((motion["q_m"], row))
        for lane_rows in by_lane.values():
            lane_rows.sort(key=lambda item: (item[0], item[1]["aircraft_id"]))
            for index, (_, row) in enumerate(lane_rows):
                n = cfg["neighbors_each_side"]
                members = [member[1]["aircraft_id"]
                           for member in lane_rows[max(0, index - n):index + n + 1]]
                if members != row["members"] or len(members) != row["group_size"]:
                    raise ValueError(f"{case}: policy-group membership mismatch")
                fraction = float(np.mean([observation_index[(identifier, time)]["sinr_db"]
                                          < cfg["threshold_db"] for identifier in members]))
                if abs(fraction - row["group_bad_fraction"]) > 1e-12:
                    raise ValueError(f"{case}: group bad fraction mismatch")
                history = histories.setdefault(row["aircraft_id"], [])
                history[:] = [item for item in history
                              if item[0] >= time - cfg["window_s"] - 1e-8]
                history.append((time, fraction))
                exposure = float(np.mean([item[1] for item in history]))
                maximum_exposure_error = max(maximum_exposure_error,
                                             abs(exposure - row["exposure"]))
                policy = ("C" if exposure <= cfg["exposure_c"] + 1e-12 else
                          "R" if exposure <= cfg["exposure_r"] + 1e-12 else "F")
                if policy != row["policy"] or len(history) != row["history_count"]:
                    raise ValueError(f"{case}: exposure/policy history mismatch")

    by_time = {}
    for row in trace:
        by_time.setdefault(round(row["t_s"], 8), []).append(row)
    maximum_gap_error = maximum_raw_error = maximum_command_error = 0.0
    minimum_sampled_separation = float("inf")
    for rows in by_time.values():
        for i in range(len(rows)):
            for j in range(i + 1, len(rows)):
                delta = np.asarray(rows[i]["position_m"]) - rows[j]["position_m"]
                score = ((delta[0] ** 2 + delta[1] ** 2) / cfg["horizontal_separation_m"]**2
                         + delta[2] ** 2 / cfg["vertical_separation_m"]**2)
                minimum_sampled_separation = min(minimum_sampled_separation, float(score))
        for own in rows:
            occupied = {own["lane"]}
            if own["target_lane"] is not None:
                occupied.add(own["target_lane"])
            leaders = []
            for lane in sorted(occupied):
                ahead = [row for row in rows if row is not own and row["q_m"] > own["q_m"] + 1e-9
                         and (row["lane"] == lane or row["target_lane"] == lane)]
                if ahead:
                    leader = min(ahead, key=lambda row: arc_position(row["q_m"], lane, q_grid, tables))
                    if leader["aircraft_id"] not in [item[0] for item in leaders]:
                        gap = (arc_position(leader["q_m"], lane, q_grid, tables)
                               - arc_position(own["q_m"], lane, q_grid, tables))
                        leaders.append((leader["aircraft_id"], gap,
                                        leader["longitudinal_speed_mps"]
                                        - own["longitudinal_speed_mps"], lane))
            archived = own["leaders"]
            if [row[0] for row in leaders] != [row[0] for row in archived]:
                raise ValueError(f"{case}: ACC leader identity mismatch")
            if leaders:
                maximum_gap_error = max(maximum_gap_error,
                    max(abs(row[1] - archived[index][1])
                        for index, row in enumerate(leaders)))
            speed = own["longitudinal_speed_mps"]
            free = cfg["k_speed_per_s"] * (cfg["cruise_mps"] - speed)
            following = [cfg["k_gap_per_s2"] * (row[1] - spacing(cfg, own["policy"], speed))
                         + cfg["k_relative_per_s"] * row[2] for row in leaders]
            raw = min([free] + following)
            command = float(np.clip(raw, -cfg["deceleration_limit_mps2"],
                                    cfg["acceleration_limit_mps2"]))
            if speed <= 1e-9 and command < 0 or speed >= cfg["cruise_mps"] - 1e-9 and command > 0:
                command = 0.0
            maximum_raw_error = max(maximum_raw_error,
                                    abs(raw - own["raw_acceleration_mps2"]))
            maximum_command_error = max(maximum_command_error,
                abs(command - own["command_acceleration_mps2"]))
    # The production lane-arc table is integrated on its archived 5 m grid;
    # this verifier deliberately uses a different 0.25 m grid.  Their gap
    # values agree within millimetres.  The 150-aircraft 3x3 audit reaches a
    # 5.023 mm worst case over the complete mission, so the independent
    # quadrature allowance is declared as 6 mm; control quantities retain the
    # much tighter acceleration tolerance below.
    if (maximum_gap_error > 6e-3 or maximum_raw_error > 2e-4
            or maximum_command_error > 2e-4):
        raise ValueError(
            f"{case}: ACC reconstruction mismatch "
            f"(gap={maximum_gap_error:.9g} m, raw={maximum_raw_error:.9g} m/s2, "
            f"command={maximum_command_error:.9g} m/s2)"
        )
    if minimum_sampled_separation < 1 - 1e-8:
        raise ValueError(f"{case}: sampled realized separation violation")
    if summary["minimum_three_dimensional_separation_score"] < 1 - 1e-8:
        raise ValueError(f"{case}: archived midpoint/endpoint separation violation")

    accepted = [row for row in events if row["status"] == "transition_accepted"]
    completed = [row for row in events if row["status"] == "transition_completed"]
    if len(accepted) != summary["accepted_transitions"] or len(completed) != summary["completed_transitions"]:
        raise ValueError(f"{case}: transition count mismatch")
    for row in accepted:
        if not adjacent_flow(row["source_lane"], row["target_lane"], coordinates):
            raise ValueError(f"{case}: nonadjacent accepted movement")
        if row.get("decision_type") == "nominal_return":
            before = np.linalg.norm(coordinates[row["source_lane"]]
                                    - coordinates[row["nominal_lane"]])
            after = np.linalg.norm(coordinates[row["target_lane"]]
                                   - coordinates[row["nominal_lane"]])
            if (after >= before - 1e-8
                    or row["move_policy_cost_s"] > row["stay_policy_cost_s"] + 1e-8
                    or {"C": 0, "R": 1, "F": 2}[row["move_endpoint_policy"]]
                    > {"C": 0, "R": 1, "F": 2}[row["stay_endpoint_policy"]]):
                raise ValueError(f"{case}: invalid nominal-return acceptance")
        elif row.get("decision_type") == "policy_avoidance":
            if row["move_policy_cost_s"] >= row["stay_policy_cost_s"] - 1e-8:
                raise ValueError(f"{case}: avoidance did not strictly improve policy cost")
        elif accepted:
            raise ValueError(f"{case}: accepted movement lacks declared decision type")
        if row["duration_s"] > cfg["predictive_rollout_s"] + cfg["predictive_evaluation_s"] + 1e-8:
            raise ValueError(f"{case}: accepted movement exceeds common planning horizon")
        if row["minimum_forecast_separation_score"] < 1 - 1e-8:
            raise ValueError(f"{case}: accepted unsafe forecast")
    policy_time = {policy: sum(row["end_s"] - row["t_s"]
        for row in trace if row["policy"] == policy) for policy in "CRF"}
    for policy in "CRF":
        if abs(policy_time[policy] - summary["policy_aircraft_time_s"][policy]) > 1e-7:
            raise ValueError(f"{case}: policy aircraft-time mismatch")
    if summary["capacity_estimated"] is not False or summary["capacity_status"] != "capacity_not_estimated":
        raise ValueError(f"{case}: capacity boundary missing")
    return {"case": case, "trace_intervals": len(trace),
        "policy_observations": len(observations), "accepted_transitions": len(accepted),
        "position_error_m": position_error, "velocity_error_mps": velocity_error,
        "sinr_error_db": signal_error, "exposure_error": maximum_exposure_error,
        "acc_gap_error_m": maximum_gap_error, "acc_raw_error_mps2": maximum_raw_error,
        "acc_command_error_mps2": maximum_command_error,
        "minimum_sampled_separation_score": minimum_sampled_separation,
        "minimum_archived_midpoint_endpoint_score":
            summary["minimum_three_dimensional_separation_score"]}


def audit(run):
    run = Path(run).resolve()
    manifest = read(run / "manifest.json")
    if manifest["status"] != "completed":
        raise ValueError("run is incomplete or changed during execution")
    for item in manifest["outputs"]:
        if sha(run / item["path"]) != item["sha256"]:
            raise ValueError(f"output hash mismatch: {item['path']}")
    spec = read(run / "resolved_config.json")
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        for item in manifest["code"] + manifest["inputs"]:
            if hashlib.sha256(archive.read(item["path"])).hexdigest() != item["sha256"]:
                raise ValueError(f"snapshot hash mismatch: {item['path']}")
        frame = independent_reference(archive, manifest,
                                      spec["parameters"]["center_control_step_m"])
    length_m = read(run / "geometry.json")["original_length_m"]
    cases = [row["id"] for row in spec["cases"]]
    reports = [audit_case(run, case, spec, frame, length_m) for case in cases]
    return {"status": "passed", "run": str(run),
        "run_manifest_sha256": sha(run / "manifest.json"),
        "checks": "archive hashes, independent geographic XYZ/velocity/radio, immutable nominal flows, same-flow groups, exposure/policy, physical-gap ACC, nominal-return/avoidance acceptance, realized/forecast safety and capacity exclusion",
        "limitations": "midpoint/endpoint engineering safety screen is audited at archived samples; no continuous aviation separation proof or independent re-optimization of every rejected candidate",
        "cases": reports}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    if arguments.output and arguments.output.exists():
        raise FileExistsError("choose a new audit file")
    result = audit(arguments.run)
    if arguments.output:
        arguments.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
