"""Diagnose the modeled BS13/BS14 co-channel interaction in R0055.

The muted-BS14 curve is a radio-only counterfactual evaluated at archived
R0055 positions. It does not alter or rerun the traffic/controller simulation.
"""
from __future__ import annotations

import csv
import gzip
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capacity_policy import load_scenario  # noqa: E402


CASE = ROOT / "research/dynamic-transitions/runs/R0055/three_lane_lane_change"
SCENARIO_PATH = ROOT / "scenarios/airport_to_airport/scenario.json"
OUTPUT = ROOT / "research/dynamic-transitions/figures/R0055"

BLUE = "#204F7A"
LIGHT_BLUE = "#2C86C5"
RED = "#C53A2E"
INK = "#243447"
GREY = "#5E7187"
PALE_BLUE = "#E8F3F5"
TEAL = "#168A78"
ORANGE = "#D78A18"


def read_json(path: Path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def radio(positions: np.ndarray, stations: np.ndarray, ids: np.ndarray, config):
    d2 = np.sum((positions[:, None, :] - stations[None, :, :]) ** 2, axis=2)
    received_dbm = (
        config.eirp_dbm + config.receiver_gain_db - 28.0
        - 11.0 * np.log10(np.maximum(d2, np.finfo(float).tiny))
        - 20.0 * np.log10(config.frequency_ghz)
    )
    count = int(config.served_set_size)
    nearest = np.argpartition(d2, count - 1, axis=1)[:, :count]
    mask = np.zeros_like(d2, dtype=bool)
    np.put_along_axis(mask, nearest, True, axis=1)
    serving = np.argmax(np.where(mask, received_dbm, -np.inf), axis=1)
    received_mw = 10.0 ** (received_dbm / 10.0)
    desired = received_mw[np.arange(len(positions)), serving]
    interference = np.sum(np.where(mask, received_mw, 0.0), axis=1) - desired
    noise = 10.0 ** (config.noise_dbm / 10.0)
    sinr_db = 10.0 * np.log10(desired / (interference + noise))
    return {
        "ids": ids,
        "received_mw": received_mw,
        "mask": mask,
        "serving": serving,
        "interference_mw": interference,
        "sinr_db": sinr_db,
    }


def profile(q_km: np.ndarray, values: np.ndarray, width_km: float = 0.5):
    edges = np.arange(34.0, 41.0 + width_km, width_km)
    rows = []
    for left, right in zip(edges[:-1], edges[1:]):
        selected = values[(q_km >= left) & (q_km < right)]
        rows.append({
            "q_km": (left + right) / 2.0,
            "p10": float(np.quantile(selected, 0.10)),
            "median": float(np.median(selected)),
            "p90": float(np.quantile(selected, 0.90)),
        })
    return rows


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    scenario = load_scenario(SCENARIO_PATH)
    trace = read_json(CASE / "trace.json.gz")
    observations = read_json(CASE / "observations.json.gz")
    trace_lookup = {(row["aircraft_id"], float(row["t_s"])): row for row in trace}
    selected_observations = [
        row for row in observations
        if 34000.0 <= trace_lookup[(row["aircraft_id"], float(row["t_s"]))]["q_m"] < 41000.0
    ]
    positions = np.asarray([
        trace_lookup[(row["aircraft_id"], float(row["t_s"]))]["xyz"]
        for row in selected_observations
    ], dtype=float)
    q_km = np.asarray([
        float(trace_lookup[(row["aircraft_id"], float(row["t_s"]))]["q_m"]) / 1000.0
        for row in selected_observations
    ])
    sites = list(scenario.base_stations.stations)
    ids = np.asarray([site.site_id for site in sites])
    station_xyz = np.asarray([[site.x_m, site.y_m, site.height_m] for site in sites])
    full = radio(positions, station_xyz, ids, scenario.radio)
    keep = ids != "BS14"
    muted = radio(positions, station_xyz[keep], ids[keep], scenario.radio)

    i13 = int(np.flatnonzero(ids == "BS13")[0])
    i14 = int(np.flatnonzero(ids == "BS14")[0])
    bs13_serves = full["serving"] == i13
    bs14_serves = full["serving"] == i14
    bs14_interference_share = np.where(
        bs13_serves & full["mask"][:, i14],
        full["received_mw"][:, i14] / full["interference_mw"],
        np.nan,
    )
    pair_distance_m = float(np.linalg.norm(station_xyz[i13, :2] - station_xyz[i14, :2]))
    metrics = {
        "pair_distance_m": pair_distance_m,
        "bs13_serving_fraction": float(np.mean(bs13_serves)),
        "bs14_serving_fraction": float(np.mean(bs14_serves)),
        "bs14_in_nearest_three_when_bs13_serves_fraction":
            float(np.mean(full["mask"][bs13_serves, i14])),
        "bs14_median_interference_share_when_bs13_serves":
            float(np.nanmedian(bs14_interference_share)),
        "baseline_median_sinr_db": float(np.median(full["sinr_db"])),
        "muted_bs14_median_sinr_db": float(np.median(muted["sinr_db"])),
        "median_sinr_gain_db": float(np.median(muted["sinr_db"] - full["sinr_db"])),
        "baseline_below_threshold_fraction": float(np.mean(full["sinr_db"] < -2.0)),
        "muted_bs14_below_threshold_fraction": float(np.mean(muted["sinr_db"] < -2.0)),
    }
    full_profile = profile(q_km, full["sinr_db"])
    muted_profile = profile(q_km, muted["sinr_db"])

    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.spines.right": False,
        "axes.spines.top": False,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
    })
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    fig.text(0.045, 0.945, "DIAGNOSIS  |  MODELED BS13–BS14 INTERACTION",
             color=LIGHT_BLUE, fontsize=11.5, fontweight="bold")
    fig.text(0.045, 0.890,
             "In the Current Model, the 590 m Site Pair Creates a Strong Interference Penalty",
             color=BLUE, fontsize=23, fontweight="bold")
    fig.add_artist(plt.Line2D([0.045, 0.165], [0.848, 0.848],
                              transform=fig.transFigure, color=RED, linewidth=3))
    fig.text(0.045, 0.815,
             "Archived R0055 positions, q = 34–41 km  ·  identical equal-EIRP radio model  ·  "
             "diagnostic muted-BS14 calculation only; traffic and control are not rerun",
             color=GREY, fontsize=10.5)

    map_ax = fig.add_axes([0.06, 0.20, 0.39, 0.52])
    route_rows = [row for row in trace if 34000.0 <= float(row["q_m"]) < 41000.0]
    by_aircraft = defaultdict(list)
    for row in route_rows:
        by_aircraft[row["aircraft_id"]].append(row)
    origin = station_xyz[i13, :2]
    for rows in by_aircraft.values():
        rows = rows[::10]
        xy = (np.asarray([row["xyz"][:2] for row in rows]) - origin) / 1000.0
        map_ax.plot(xy[:, 0], xy[:, 1], color="#A8BBC8", linewidth=0.55,
                    alpha=0.15, zorder=1)
    focus = {"BS12", "BS13", "BS14", "BS15"}
    station_lookup = {site.site_id: site for site in sites}
    for site_id in focus:
        site = station_lookup[site_id]
        xy = (np.asarray([site.x_m, site.y_m]) - origin) / 1000.0
        if site_id in {"BS13", "BS14"}:
            colour = BLUE if site_id == "BS13" else RED
            size = 95
        else:
            colour = GREY
            size = 50
        map_ax.scatter(*xy, marker="^", s=size, facecolor="white", edgecolor=colour,
                       linewidth=1.5, zorder=4)
        if site_id == "BS13":
            text_xy = xy + np.asarray([0.10, -0.02])
        elif site_id == "BS14":
            text_xy = xy + np.asarray([0.10, 0.17])
        else:
            text_xy = xy + np.asarray([0.10, 0.10])
        map_ax.text(text_xy[0], text_xy[1], site_id, color=colour,
                    fontsize=9, fontweight="bold", zorder=5)
    p13 = np.zeros(2)
    p14 = (station_xyz[i14, :2] - origin) / 1000.0
    map_ax.plot([p13[0], p14[0]], [p13[1], p14[1]], color=RED, linewidth=1.2,
                linestyle="--", zorder=3)
    midpoint = (p13 + p14) / 2
    map_ax.annotate(f"{pair_distance_m:.0f} m", xy=midpoint,
                    xytext=(midpoint[0] - 0.75, midpoint[1] + 0.65),
                    color=RED, fontsize=9, fontweight="bold", ha="center",
                    arrowprops=dict(arrowstyle="-", color=RED, linewidth=0.9))
    map_ax.set_aspect("equal", adjustable="datalim")
    map_ax.set_xlabel("Easting relative to BS13 (km)")
    map_ax.set_ylabel("Northing relative to BS13 (km)")
    map_ax.set_title("a  BS13 and BS14 are nearly co-located beside the corridor",
                     loc="left", color=INK, pad=10)
    map_ax.grid(color="#EDF1F4", linewidth=0.6)

    line_ax = fig.add_axes([0.53, 0.46, 0.41, 0.26])
    x = np.asarray([row["q_km"] for row in full_profile])
    full_med = np.asarray([row["median"] for row in full_profile])
    full_p10 = np.asarray([row["p10"] for row in full_profile])
    full_p90 = np.asarray([row["p90"] for row in full_profile])
    muted_med = np.asarray([row["median"] for row in muted_profile])
    line_ax.fill_between(x, full_p10, full_p90, color="#CADCE8", alpha=0.7,
                         label="baseline 10th–90th percentile")
    line_ax.plot(x, full_med, color=BLUE, linewidth=2.2, label="baseline median")
    line_ax.plot(x, muted_med, color=TEAL, linewidth=2.2, linestyle="--",
                 label="median with BS14 muted")
    line_ax.axhline(-2.0, color=RED, linewidth=1.3, linestyle=":", label="Θ = −2 dB")
    line_ax.set_xlim(34, 41)
    line_ax.set_ylabel("SINR (dB)")
    line_ax.set_xlabel("Longitudinal distance, q (km)")
    line_ax.set_title("b  Muting BS14 removes most of the modeled SINR depression",
                      loc="left", color=INK, pad=10)
    line_ax.grid(color="#EDF1F4", linewidth=0.6)
    line_ax.legend(loc="upper left", fontsize=7.4, ncol=2)

    role_ax = fig.add_axes([0.53, 0.20, 0.19, 0.17])
    role_values = [100 * metrics["bs13_serving_fraction"],
                   100 * metrics["bs14_serving_fraction"]]
    role_ax.barh([1, 0], role_values, color=[BLUE, RED], height=0.48)
    for y, value in zip([1, 0], role_values):
        if value > 20:
            role_ax.text(value - 2, y, f"{value:.1f}%", va="center", ha="right",
                         fontsize=9, fontweight="bold", color="white")
        else:
            role_ax.text(value + 2, y, f"{value:.1f}%", va="center", fontsize=9,
                         fontweight="bold", color=INK)
    role_ax.set_yticks([1, 0], ["BS13", "BS14"])
    role_ax.set_xlim(0, 105)
    role_ax.set_xlabel("Serving samples (%)")
    role_ax.set_title("c  Serving role", loc="left", color=INK, pad=8)
    role_ax.grid(axis="x", color="#EDF1F4", linewidth=0.6)

    bad_ax = fig.add_axes([0.77, 0.20, 0.17, 0.17])
    bad_values = [100 * metrics["baseline_below_threshold_fraction"],
                  100 * metrics["muted_bs14_below_threshold_fraction"]]
    bad_ax.barh([1, 0], bad_values, color=[ORANGE, TEAL], height=0.48)
    for y, value in zip([1, 0], bad_values):
        bad_ax.text(value + 0.4, y, f"{value:.1f}%", va="center", fontsize=9,
                    fontweight="bold", color=INK)
    bad_ax.set_yticks([1, 0], ["Baseline", "BS14 muted"])
    bad_ax.set_xlim(0, 13)
    bad_ax.set_xlabel("Samples below Θ (%)")
    bad_ax.set_title("d  Threshold crossings", loc="left", color=INK, pad=8)
    bad_ax.grid(axis="x", color="#EDF1F4", linewidth=0.6)

    takeaway = fig.add_axes([0.06, 0.045, 0.88, 0.075])
    takeaway.set_facecolor(PALE_BLUE)
    takeaway.set_xticks([])
    takeaway.set_yticks([])
    for spine in takeaway.spines.values():
        spine.set_visible(False)
    takeaway.text(
        0.015, 0.62,
        f"When BS13 serves, BS14 is always in the nearest-three set and contributes a "
        f"median {100*metrics['bs14_median_interference_share_when_bs13_serves']:.1f}% "
        f"of modeled interference.",
        color=BLUE, fontsize=10.6, fontweight="bold", va="center")
    takeaway.text(
        0.015, 0.25,
        "This supports the mechanism inside the current equal-EIRP/co-channel model; it is "
        "not evidence that the real Verizon and T-Mobile/AT&T sites share spectrum or interfere identically.",
        color=INK, fontsize=9.1, va="center")

    stem = OUTPUT / "05-bs13-bs14-interference-diagnosis"
    for suffix, kwargs in [(".png", {"dpi": 220}), (".svg", {}), (".pdf", {})]:
        fig.savefig(stem.with_suffix(suffix), bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    with (stem.parent / f"{stem.name}-source.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        fields = ["q_km", "baseline_sinr_p10_db", "baseline_sinr_median_db",
                  "baseline_sinr_p90_db", "muted_bs14_sinr_median_db"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for baseline, alternative in zip(full_profile, muted_profile):
            writer.writerow({
                "q_km": baseline["q_km"],
                "baseline_sinr_p10_db": baseline["p10"],
                "baseline_sinr_median_db": baseline["median"],
                "baseline_sinr_p90_db": baseline["p90"],
                "muted_bs14_sinr_median_db": alternative["median"],
            })
    (stem.parent / f"{stem.name}-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
