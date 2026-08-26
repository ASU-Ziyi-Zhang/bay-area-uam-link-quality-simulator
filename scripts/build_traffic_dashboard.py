"""Build a compact multi-UAM policy/capacity dashboard bundle."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path

import numpy as np
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[1]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    scenario_path = args.scenario.resolve()
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    if summary["scenario_id"] != scenario["scenario_id"]:
        raise ValueError("group run and scenario IDs do not match")

    entrants = read_csv(run_dir / "entrants.csv")
    policy_rows = read_csv(run_dir / "group_policy_trace.csv")
    capacity_rows = read_csv(run_dir / "capacity_trace.csv")
    policies_by_time: dict[float, dict[str, str]] = defaultdict(dict)
    exposure_by_time: dict[float, dict[str, float]] = defaultdict(dict)
    for row in policy_rows:
        timestamp = round(float(row["timestamp_s"]), 9)
        policies_by_time[timestamp][row["focal_uam_id"]] = row["policy"]
        exposure_by_time[timestamp][row["focal_uam_id"]] = float(
            row["exposure_fraction"]
        )

    frames = []
    for row in capacity_rows:
        timestamp = round(float(row["timestamp_s"]), 9)
        policies = policies_by_time[timestamp]
        expected = int(row["classified_group_count"])
        if len(policies) != expected:
            raise ValueError(
                f"policy count mismatch at {timestamp}: {len(policies)} != {expected}"
            )
        frames.append(
            {
                "t": timestamp,
                "active_count": int(row["active_uam_count"]),
                "classified_count": expected,
                "n_C": int(row["n_C"]),
                "n_R": int(row["n_R"]),
                "n_F": int(row["n_F"]),
                "mean_spacing_m": float(row["mean_spacing_m"]),
                "q_mix": float(row["q_mix_uam_h"]),
                "q_bottleneck": float(row["q_bottleneck_uam_h"]),
                "policies": policies,
                "exposure": exposure_by_time[timestamp],
            }
        )

    site_path = (scenario_path.parent / scenario["base_stations"]["path"]).resolve()
    corridor_path = (scenario_path.parent / scenario["corridor"]["path"]).resolve()
    site_rows = read_csv(site_path)
    active_ids = scenario["base_stations"].get("active_site_ids")
    if active_ids:
        by_id = {row["public_site_id"]: row for row in site_rows}
        site_rows = [by_id[site_id] for site_id in active_ids]
    route = json.loads(corridor_path.read_text(encoding="utf-8"))["features"][0][
        "geometry"
    ]["coordinates"]
    to_metric = Transformer.from_crs("EPSG:4326", "EPSG:26910", always_xy=True)
    route_array = np.asarray(route, dtype=float)
    route_x, route_y = to_metric.transform(route_array[:, 0], route_array[:, 1])
    route_xy = np.column_stack([route_x, route_y])
    route_s = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(route_xy, axis=0), axis=1)))
    )
    stations = []
    for row in site_rows:
        x_m, y_m = to_metric.transform(float(row["longitude"]), float(row["latitude"]))
        stations.append(
            {
                "id": row["public_site_id"],
                "lat": float(row["latitude"]),
                "lon": float(row["longitude"]),
                "x_m": float(x_m),
                "y_m": float(y_m),
                "height_m": float(row["antenna_height_m"]),
                "site_class": row["fcc_site_type"],
                "physical_form": row["physical_form"],
            }
        )

    bundle = {
        "summary": {
            **summary,
            "run_id": run_dir.name,
            "display": scenario.get("display", {}),
            "radio": scenario["radio"],
        },
        "route": route,
        "route_metric": [
            {
                "s_m": float(route_s[index]),
                "x_m": float(route_x[index]),
                "y_m": float(route_y[index]),
                "lon": float(route_array[index, 0]),
                "lat": float(route_array[index, 1]),
            }
            for index in range(len(route_array))
        ],
        "stations": stations,
        "entrants": [
            {
                "id": row["uam_id"],
                "index": int(row["uam_index"]),
                "entry": float(row["entry_time_s"]),
                "exit": float(row["exit_time_s"]),
            }
            for row in entrants
        ],
        "frames": frames,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "window.UAM_TRAFFIC_DATA = "
        + json.dumps(bundle, separators=(",", ":"))
        + ";\n",
        encoding="utf-8",
    )
    print(f"Traffic dashboard data: {args.output.resolve()}")
    print(
        f"Frames: {len(frames)}; entrants: {len(entrants)}; "
        f"policy observations: {len(policy_rows)}"
    )


if __name__ == "__main__":
    main()
