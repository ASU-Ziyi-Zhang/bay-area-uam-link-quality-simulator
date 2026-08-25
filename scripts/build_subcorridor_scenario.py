"""Derive a station-proxy subcorridor and its eligible macro-site table."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DEFINITION = ROOT / "scenarios" / "airport_to_airport" / "build.json"


def resolve(base: Path, value: str) -> Path:
    return (base / value).resolve()


def cumulative_distance(xy: np.ndarray) -> np.ndarray:
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))


def project_point(xy: np.ndarray, point: np.ndarray) -> tuple[float, float, np.ndarray]:
    """Return chainage, perpendicular distance, and closest point."""
    segment = np.diff(xy, axis=0)
    length_sq = np.sum(segment * segment, axis=1)
    fraction = np.clip(
        np.sum((point - xy[:-1]) * segment, axis=1) / length_sq,
        0.0,
        1.0,
    )
    candidates = xy[:-1] + fraction[:, None] * segment
    distance = np.linalg.norm(candidates - point, axis=1)
    index = int(np.argmin(distance))
    cumulative = cumulative_distance(xy)
    chainage = cumulative[index] + fraction[index] * np.sqrt(length_sq[index])
    return float(chainage), float(distance[index]), candidates[index]


def interpolate_at(xy: np.ndarray, chainage_m: float) -> np.ndarray:
    cumulative = cumulative_distance(xy)
    clipped = float(np.clip(chainage_m, 0.0, cumulative[-1]))
    index = int(np.searchsorted(cumulative, clipped, side="right") - 1)
    index = min(max(index, 0), len(cumulative) - 2)
    span = cumulative[index + 1] - cumulative[index]
    fraction = (clipped - cumulative[index]) / span
    return xy[index] + fraction * (xy[index + 1] - xy[index])


def slice_polyline(xy: np.ndarray, start_m: float, end_m: float) -> np.ndarray:
    if end_m <= start_m:
        raise ValueError("the second endpoint must occur after the first on the source route")
    cumulative = cumulative_distance(xy)
    interior = xy[(cumulative > start_m) & (cumulative < end_m)]
    return np.vstack([interpolate_at(xy, start_m), interior, interpolate_at(xy, end_m)])


def read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader), list(reader.fieldnames or [])


def write_csv(path: Path, rows: list[dict[str, str]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def build(definition_path: Path) -> dict[str, object]:
    definition_path = definition_path.resolve()
    definition = json.loads(definition_path.read_text(encoding="utf-8"))
    base = definition_path.parent
    source_corridor_path = resolve(base, definition["source_corridor"])
    source_sites_path = resolve(base, definition["source_base_stations"])
    output_corridor_path = resolve(base, definition["output_corridor"])
    output_sites_path = resolve(base, definition["output_base_stations"])
    report_path = resolve(base, definition["output_report"])

    source_geojson = json.loads(source_corridor_path.read_text(encoding="utf-8"))
    source_feature = source_geojson["features"][0]
    lon_lat = np.asarray(source_feature["geometry"]["coordinates"], dtype=float)
    metric_crs = definition.get("metric_crs", "EPSG:26910")
    to_metric = Transformer.from_crs("EPSG:4326", metric_crs, always_xy=True)
    to_wgs84 = Transformer.from_crs(metric_crs, "EPSG:4326", always_xy=True)
    x, y = to_metric.transform(lon_lat[:, 0], lon_lat[:, 1])
    source_xy = np.column_stack([x, y])

    endpoint_results: list[dict[str, object]] = []
    endpoint_chainage: list[float] = []
    for endpoint in definition["endpoints"]:
        px, py = to_metric.transform(float(endpoint["longitude"]), float(endpoint["latitude"]))
        chainage_m, error_m, closest = project_point(source_xy, np.asarray([px, py]))
        endpoint_chainage.append(chainage_m)
        lon, lat = to_wgs84.transform(float(closest[0]), float(closest[1]))
        endpoint_results.append(
            {
                "name": endpoint["name"],
                "role": endpoint["role"],
                "input_longitude": float(endpoint["longitude"]),
                "input_latitude": float(endpoint["latitude"]),
                "route_longitude": float(lon),
                "route_latitude": float(lat),
                "source_chainage_km": chainage_m / 1000.0,
                "projection_error_m": error_m,
            }
        )

    start_m, end_m = endpoint_chainage
    sub_xy = slice_polyline(source_xy, start_m, end_m)
    lon, lat = to_wgs84.transform(sub_xy[:, 0], sub_xy[:, 1])
    sub_lon_lat = np.column_stack([lon, lat])
    sub_length_m = cumulative_distance(sub_xy)[-1]

    rows, fields = read_csv(source_sites_path)
    for extra in ("source_chainage_km", "source_lateral_offset_km"):
        if extra not in fields:
            fields.append(extra)
    buffer_m = float(definition["site_inclusion_distance_m"])
    retained: list[dict[str, str]] = []
    excluded_ids: list[str] = []
    for row in rows:
        sx, sy = to_metric.transform(float(row["longitude"]), float(row["latitude"]))
        chainage_m, distance_m, _ = project_point(sub_xy, np.asarray([sx, sy]))
        if distance_m <= buffer_m + 1e-9:
            updated = dict(row)
            updated["source_chainage_km"] = row.get("chainage_km", "")
            updated["source_lateral_offset_km"] = row.get("lateral_offset_km", "")
            updated["chainage_km"] = f"{chainage_m / 1000.0:.3f}"
            updated["lateral_offset_km"] = f"{distance_m / 1000.0:.3f}"
            retained.append(updated)
        else:
            excluded_ids.append(row["public_site_id"])

    output_corridor_path.parent.mkdir(parents=True, exist_ok=True)
    feature = {
        "type": "Feature",
        "properties": {
            "scenario_id": definition["scenario_id"],
            "reference": source_feature.get("properties", {}).get("reference", "source corridor"),
            "derived_from": definition["source_scenario_id"],
            "endpoint_basis": "Caltrain station proxies for airport access",
            "start": definition["endpoints"][0]["name"],
            "end": definition["endpoints"][1]["name"],
            "length_km": round(float(sub_length_m / 1000.0), 6),
        },
        "geometry": {"type": "LineString", "coordinates": sub_lon_lat.tolist()},
    }
    output_corridor_path.write_text(
        json.dumps({"type": "FeatureCollection", "features": [feature]}, indent=2) + "\n",
        encoding="utf-8",
    )
    write_csv(output_sites_path, retained, fields)

    source_length_m = cumulative_distance(source_xy)[-1]
    report = {
        "schema_version": 1,
        "scenario_id": definition["scenario_id"],
        "source_scenario_id": definition["source_scenario_id"],
        "metric_crs": metric_crs,
        "endpoint_results": endpoint_results,
        "source_corridor_length_km": source_length_m / 1000.0,
        "subcorridor_length_km": sub_length_m / 1000.0,
        "north_trim_km": start_m / 1000.0,
        "south_trim_km": (source_length_m - end_m) / 1000.0,
        "site_inclusion_distance_km": buffer_m / 1000.0,
        "retained_site_ids": [row["public_site_id"] for row in retained],
        "excluded_site_ids": excluded_ids,
        "retained_site_count": len(retained),
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    args = parser.parse_args()
    report = build(args.definition)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
