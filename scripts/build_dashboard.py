"""Build the dashboard data bundle from a simulator run."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = ROOT / "results" / "simulator"
DEFAULT_SCENARIO = ROOT / "scenarios" / "sf_sj_full" / "scenario.json"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument("--output", type=Path, default=ROOT / "dashboard" / "data" / "dashboard_data.js")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    scenario_path = args.scenario.resolve()
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    if summary["scenario_id"] != scenario["scenario_id"]:
        raise ValueError(
            f"run scenario {summary['scenario_id']} does not match {scenario['scenario_id']}"
        )
    trace_rows = read_csv(run_dir / "uam_state_trace.csv")
    site_path = (scenario_path.parent / scenario["base_stations"]["path"]).resolve()
    corridor_path = (scenario_path.parent / scenario["corridor"]["path"]).resolve()
    site_rows = read_csv(site_path)
    active_ids = scenario["base_stations"].get("active_site_ids")
    if active_ids:
        by_id = {row["public_site_id"]: row for row in site_rows}
        site_rows = [by_id[site_id] for site_id in active_ids]
    corridor = json.loads(corridor_path.read_text(encoding="utf-8"))
    route = corridor["features"][0]["geometry"]["coordinates"]

    to_wgs84 = Transformer.from_crs("EPSG:26910", "EPSG:4326", always_xy=True)
    trace = []
    for row in trace_rows:
        lon, lat = to_wgs84.transform(float(row["x_m"]), float(row["y_m"]))
        trace.append({
            "t": float(row["timestamp_s"]), "s_km": float(row["s_m"]) / 1000.0,
            "x_m": float(row["x_m"]), "y_m": float(row["y_m"]),
            "lat": float(lat), "lon": float(lon), "altitude_m": float(row["z_m"]),
            "serving": row["serving_bs_id"], "rsrp": float(row["serving_rsrp_dbm_per_re"]),
            "sinr": float(row["sinr_db"]), "handoff": row["handoff"].lower() == "true",
            "control_tick": row["control_tick"].lower() == "true",
        })

    to_metric = Transformer.from_crs("EPSG:4326", "EPSG:26910", always_xy=True)
    stations = []
    for row in site_rows:
        x_m, y_m = to_metric.transform(float(row["longitude"]), float(row["latitude"]))
        stations.append({
            "id": row["public_site_id"], "lat": float(row["latitude"]), "lon": float(row["longitude"]),
            "x_m": float(x_m), "y_m": float(y_m),
            "chainage_km": float(row["chainage_km"]), "offset_km": float(row["lateral_offset_km"]),
            "address": row["address"], "site_class": row["fcc_site_type"],
            "physical_form": row["physical_form"], "operator": row["operator_evidence"],
            "height_m": float(row["antenna_height_m"]), "height_basis": row["height_basis"],
            "official_source_url": row["official_source_url"],
        })

    rsrp = np.asarray([row["rsrp"] for row in trace])
    sinr = np.asarray([row["sinr"] for row in trace])
    bundle = {
        "summary": {
            "run_id": run_dir.name,
            "scenario_id": scenario["scenario_id"],
            "display": scenario.get("display", {}),
            "corridor_length_km": float(summary["corridor_length_km"]),
            "site_count": int(summary["site_count"]),
            "clock": summary["clock"],
            "rsrp_min": float(rsrp.min()), "rsrp_max": float(rsrp.max()),
            "sinr_min": float(sinr.min()), "sinr_max": float(sinr.max()),
            "handoff_count": int(summary["samples"]["handoff_count"]),
            "defaults": summary["trajectory"], "radio": scenario["radio"],
        },
        "route": route, "stations": stations, "trace": trace,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "window.UAM_DASHBOARD_DATA = " + json.dumps(bundle, separators=(",", ":")) + ";\n",
        encoding="utf-8",
    )
    print(f"Dashboard data: {args.output.resolve()}")
    print(f"Trace rows: {len(trace)}; stations: {len(stations)}")


if __name__ == "__main__":
    main()
