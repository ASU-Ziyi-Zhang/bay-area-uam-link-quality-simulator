#!/usr/bin/env python3
"""Independent numerical checks for the 3x3 single-stream capacity archive."""
from __future__ import annotations

import argparse
import csv
import gzip
import json
from pathlib import Path


def load(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    args=parser.parse_args()
    run=args.run.resolve()
    cfg=load(run/"resolved_config.json")
    manifest=load(run/"manifest.json")
    failures=[]
    if cfg.get("study") != "bay_area_single_stream_capacity_3x3":
        failures.append("study")
    points=[(float(row["offset_m"]),float(row["altitude_m"]))
            for row in cfg["traffic"]["flow_points"]]
    expected=[(d,h) for d in (-300.,0.,300.) for h in (200.,300.,400.)]
    if points != expected or cfg["traffic"]["entry_flow_index"] != 4:
        failures.append("grid")
    if cfg["traffic"].get("candidate_topology") != "grid_8_connected":
        failures.append("topology")
    parameters=cfg["parameters"]
    if (parameters["horizontal_separation_m"] != 152.4
            or parameters["vertical_separation_m"] != 30.48):
        failures.append("nmac_dimensions")
    cases={row["id"]:row for row in cfg["cases"]}
    # A single-case archive (e.g. a settings-only reference) is valid; every declared case is checked in full.
    known_cases={"fixed_centerline","spatial_grid"}
    expected_cases=set(cases)
    if not expected_cases or not expected_cases <= known_cases:
        failures.append("cases")
    summaries={}
    for case in expected_cases:
        case_dir=run/case
        summary=load(case_dir/"summary.json")
        summaries[case]=summary
        if summary["status"] != "all_exited":
            failures.append(case+":status")
        if summary["completed_requests"] != summary["scheduled_requests"]:
            failures.append(case+":completion")
        metrics=summary["policy_and_longitudinal_metrics"]
        if metrics["nmac_sampled"]:
            failures.append(case+":nmac")
        if metrics["maximum_joint_envelope_utilisation"] > 1+1e-6:
            failures.append(case+":envelope")
        if metrics["minimum_total_speed_mps"] < parameters["speed_min_mps"]-1e-6:
            failures.append(case+":speed_min")
        if metrics["maximum_total_speed_mps"] > parameters["speed_max_mps"]+1e-6:
            failures.append(case+":speed_max")
        with (case_dir/"capacity_trace.csv").open(encoding="utf-8-sig",newline="") as stream:
            capacity=list(csv.DictReader(stream))
        if len(capacity) != summary["conditional_planning_capacity"]["snapshot_count"]:
            failures.append(case+":capacity_count")
        for row in capacity:
            if int(row["active_aircraft"]) != sum(int(row[f"n_{p}"]) for p in "CRF"):
                failures.append(case+":capacity_partition");break
            expected_q=3600*parameters["cruise_mps"]/float(row["mean_target_spacing_m"])
            if abs(float(row["q_mix_uam_h"])-expected_q)>1e-8:
                failures.append(case+":capacity_formula");break
    if summaries.get("fixed_centerline",{}).get("lane_change_metrics",{}).get("started_lane_changes") != 0:
        failures.append("fixed_centerline:moves")
    grid_events=load(run/"spatial_grid"/"events.json") if (run/"spatial_grid"/"events.json").exists() else []
    starts=[row for row in grid_events if row.get("status")=="change_started"]
    lateral=sorted(set(point[0] for point in points));vertical=sorted(set(point[1] for point in points))
    for row in starts:
        source,target=points[row["source_lane"]],points[row["target_lane"]]
        step=max(abs(lateral.index(source[0])-lateral.index(target[0])),
                 abs(vertical.index(source[1])-vertical.index(target[1])))
        if step != 1:
            failures.append("spatial_grid:nonadjacent_move");break
    if manifest.get("status") not in ("completed","requires_review"):
        failures.append("manifest")
    report={
        "status":"pass" if not failures else "fail",
        "failures":failures,
        "cases":{
            case:{
                "completed":summaries.get(case,{}).get("completed_requests"),
                "sampled_nmac":summaries.get(case,{}).get(
                    "policy_and_longitudinal_metrics",{}).get("nmac_sampled"),
                "q_mix_rho_uam_h":summaries.get(case,{}).get(
                    "conditional_planning_capacity",{}).get("q_mix_rho_uam_h"),
            } for case in expected_cases
        },
        "move_count":len(starts),
        "interpretation":"Numerical archive and formula verification; not operational capacity or separation certification."
    }
    (run/"validation.json").write_text(
        json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if not failures else 1)


if __name__ == "__main__":
    main()
