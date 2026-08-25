"""Build a deterministic scenario-specific corridor and macro-site layout SVG."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[1]
mpl.rcParams["svg.hashsalt"] = "bay-area-uam-site-layout-v1"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    scenario_path = args.scenario.resolve()
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    display = scenario.get("display", {})
    corridor_path = (scenario_path.parent / scenario["corridor"]["path"]).resolve()
    site_path = (scenario_path.parent / scenario["base_stations"]["path"]).resolve()
    corridor = json.loads(corridor_path.read_text(encoding="utf-8"))
    lon_lat = np.asarray(corridor["features"][0]["geometry"]["coordinates"], dtype=float)
    to_metric = Transformer.from_crs("EPSG:4326", "EPSG:26910", always_xy=True)
    route_x, route_y = to_metric.transform(lon_lat[:, 0], lon_lat[:, 1])
    origin_x, origin_y = float(route_x[0]), float(route_y[0])
    route_xy = np.column_stack([(route_x - origin_x) / 1000.0, (route_y - origin_y) / 1000.0])

    rows_by_id = {row["public_site_id"]: row for row in read_csv(site_path)}
    active_ids = scenario["base_stations"].get("active_site_ids", list(rows_by_id))
    rows = [rows_by_id[site_id] for site_id in active_ids]
    site_lon = np.asarray([float(row["longitude"]) for row in rows])
    site_lat = np.asarray([float(row["latitude"]) for row in rows])
    site_x, site_y = to_metric.transform(site_lon, site_lat)
    site_xy = np.column_stack([(site_x - origin_x) / 1000.0, (site_y - origin_y) / 1000.0])

    # Match the accepted 18-site figure: rotate the map so the corridor's two
    # endpoints share a horizontal reference while the north arrow preserves
    # the geographic orientation.
    endpoint_vector = route_xy[-1] - route_xy[0]
    rotation_angle = float(np.arctan2(endpoint_vector[1], endpoint_vector[0]))
    cosine, sine = np.cos(rotation_angle), np.sin(rotation_angle)
    rotation = np.asarray([[cosine, sine], [-sine, cosine]])
    route_xy = route_xy @ rotation.T
    site_xy = site_xy @ rotation.T

    fig = plt.figure(figsize=(13.7, 6.4), facecolor="white")
    ax = fig.add_axes([0.04, 0.18, 0.92, 0.64])
    ax.set_facecolor("white")
    route_color = "#273a50"
    ax.plot(route_xy[:, 0], route_xy[:, 1], color=route_color, linewidth=5.0, solid_capstyle="round", zorder=2)
    ax.scatter(route_xy[[0, -1], 0], route_xy[[0, -1], 1], s=58, color=route_color, zorder=3)

    class_style = {
        "Macro_Tower": ("^", "#159a7e", "Macro_Tower"),
        "Macro_Building": ("s", "#2b83c0", "Macro_Building"),
        "Macro_Other": ("D", "#7b55b7", "Macro_Other"),
    }
    label_offsets = {
        "BS05": (-8, 20), "BS06": (28, 0), "BS07": (-8, -27),
        "BS08": (10, 22), "BS09": (0, -30), "BS10": (0, 25),
        "BS11": (-8, -29), "BS12": (5, 25), "BS13": (-24, -50),
        "BS14": (-10, 30), "BS15": (10, 28), "BS16": (0, -30),
    }
    for row, (x_km, y_km) in zip(rows, site_xy):
        marker, color, _ = class_style.get(row["fcc_site_type"], ("o", "#64727d", row["fcc_site_type"]))
        ax.scatter(x_km, y_km, marker=marker, s=92, color=color, edgecolor="white", linewidth=0.9, zorder=5)
        if row.get("local_photo", "").strip():
            ax.scatter(x_km, y_km, marker="o", s=210, facecolor="none", edgecolor="#111111", linewidth=1.7, zorder=4)
        dx, dy = label_offsets.get(row["public_site_id"], (0, 23))
        ax.annotate(
            row["public_site_id"], (x_km, y_km), xytext=(dx, dy), textcoords="offset points",
            ha="center", va="bottom" if dy > 0 else "top", fontsize=8.2, fontweight="bold",
            color=route_color, arrowprops={"arrowstyle": "-", "color": "#9aa4ad", "lw": 0.75}, zorder=6,
        )

    y_span = max(float(np.ptp(np.concatenate([route_xy[:, 1], site_xy[:, 1]]))), 8.0)
    ax.text(route_xy[0, 0], route_xy[0, 1] + 0.34 * y_span, display.get("layout_origin_label", display.get("origin_label", "Corridor origin")), ha="left", va="bottom", fontsize=9.2, fontweight="bold", color=route_color)
    ax.text(route_xy[-1, 0], route_xy[-1, 1] + 0.34 * y_span, display.get("layout_destination_label", display.get("destination_label", "Corridor destination")), ha="right", va="bottom", fontsize=9.2, fontweight="bold", color=route_color)

    present = {row["fcc_site_type"] for row in rows}
    handles = [
        Line2D([0], [0], marker=marker, linestyle="none", markersize=8, markerfacecolor=color, markeredgecolor="white", label=label)
        for site_class, (marker, color, label) in class_style.items() if site_class in present
    ]
    handles.append(Line2D([0], [0], marker="o", linestyle="none", markersize=9, markerfacecolor="none", markeredgecolor="#111111", label="Site image retained"))
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.072), frameon=False, fontsize=9.1, ncol=4, handletextpad=0.7, columnspacing=1.8)

    # Match the original 10 km scale-bar treatment.
    data_x_min, data_x_max = float(np.min(route_xy[:, 0])), float(np.max(route_xy[:, 0]))
    data_y_min, data_y_max = float(np.min(np.concatenate([route_xy[:, 1], site_xy[:, 1]]))), float(np.max(np.concatenate([route_xy[:, 1], site_xy[:, 1]])))
    scale_y = data_y_min - 0.23 * max(data_y_max - data_y_min, 8.0)
    scale_x = data_x_min + 0.01 * (data_x_max - data_x_min)
    ax.plot([scale_x, scale_x + 10], [scale_y, scale_y], color="#202833", linewidth=2.4)
    ax.plot([scale_x, scale_x], [scale_y - 0.55, scale_y + 0.55], color="#202833", linewidth=2.0)
    ax.plot([scale_x + 10, scale_x + 10], [scale_y - 0.55, scale_y + 0.55], color="#202833", linewidth=2.0)
    ax.text(scale_x + 5, scale_y + 1.15, "10 km", ha="center", va="bottom", fontsize=8.3, color="#202833")

    north_rotated = np.asarray([sine, cosine])
    north_rotated /= np.linalg.norm(north_rotated)
    north_anchor = np.asarray([0.72, 0.78])
    north_tip = north_anchor + 0.09 * north_rotated
    ax.annotate("", xy=north_tip, xytext=north_anchor, xycoords="axes fraction", textcoords="axes fraction", arrowprops={"arrowstyle": "-|>", "color": "#202833", "lw": 1.8})
    ax.text(north_tip[0] - 0.012, north_tip[1] + 0.02, "N", transform=ax.transAxes, ha="center", va="bottom", fontsize=9.2, fontweight="bold", color="#111111")

    fig.suptitle(display.get("layout_title", f"Documented macro sites along the {display.get('route_label', scenario['scenario_id'])}"), y=0.955, fontsize=18, fontweight="bold", color="#111111")
    fig.text(0.5, 0.885, display.get("layout_subtitle", f"Caltrain-referenced alignment · {len(rows)} retained physical sites"), ha="center", va="center", fontsize=10.2, color="#64748b")
    fig.text(0.5, 0.018, "FCC macro-site classes; black rings mark sites with a retained street/site image.", ha="center", va="bottom", fontsize=8.0, color="#64748b")
    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(x=0.055, y=0.22)
    ax.axis("off")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output_format = args.output.suffix.lower().lstrip(".") or "svg"
    if output_format not in {"svg", "png"}:
        raise ValueError("output must use .svg or .png")
    metadata = (
        {"Date": None, "Creator": "bay-area-uam-link-quality-simulator"}
        if output_format == "svg"
        else {"Software": "bay-area-uam-link-quality-simulator"}
    )
    fig.savefig(args.output, format=output_format, dpi=180, metadata=metadata)
    plt.close(fig)
    print(f"Site layout: {args.output.resolve()}")


if __name__ == "__main__":
    main()
