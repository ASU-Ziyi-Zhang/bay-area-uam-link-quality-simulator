"""Run the reproducible 18-site SF-SJ serving-RSRP and SINR diagnostic."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from capacity_policy import TrajectoryState, compute_link_state, load_scenario


RUN_DIR = PROJECT_ROOT / "runs" / "link_quality"
DATA_DIR = RUN_DIR
FIGURE_DIR = RUN_DIR / "figures"

SCENARIO_PATH = PROJECT_ROOT / "configs" / "scenario.json"
SITE_PATH = PROJECT_ROOT / "data" / "base_stations.csv"
CORRIDOR_PATH = PROJECT_ROOT / "data" / "corridor.geojson"

N_LONGITUDINAL = 2_001
REFERENCE_ALTITUDE_M = 300.0
ALTITUDE_HALF_BAND_M = 60.0
N_ALTITUDE = 7
LATERAL_HALF_WIDTH_M = 500.0
N_LATERAL = 21
CHUNK_LONGITUDINAL = 80


mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 9.0,
        "axes.labelsize": 9.5,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 7.8,
        "axes.linewidth": 0.8,
        "xtick.major.width": 0.8,
        "ytick.major.width": 0.8,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
    }
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def repository_path(path: Path) -> str:
    """Return a portable repository-relative path when possible."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(PROJECT_ROOT).as_posix()
    except ValueError:
        return resolved.name


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def prepare_run(overwrite: bool) -> None:
    if RUN_DIR.exists() and not overwrite:
        raise FileExistsError(
            f"Run already exists: {RUN_DIR}. Choose another output or pass --overwrite."
        )
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)


def trajectory_for_positions(position_m: np.ndarray, along_m: np.ndarray) -> TrajectoryState:
    n = position_m.shape[0]
    return TrajectoryState(
        time_s=np.asarray([0.0]),
        position_m=position_m[:, None, :],
        along_m=along_m[:, None],
        active=np.ones((n, 1), dtype=bool),
    )


def flatten_state(state: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {
        "serving_index": state["serving_bs"][:, 0].astype(int),
        "serving_rx_dbm": state["serving_rx_dbm"][:, 0],
        "serving_rsrp_dbm": state["serving_rsrp_dbm"][:, 0],
        "interference_dbm": state["interference_dbm"][:, 0],
        "sinr_db": state["sinr_db"][:, 0],
        "noise_fraction": state["noise_fraction"][:, 0],
        "received_power_dbm": state["received_power_dbm"][:, 0, :],
        "served_mask": state["served_mask"][:, 0, :],
    }


def evaluate_positions(position_m: np.ndarray, along_m: np.ndarray, scenario):
    trajectory = trajectory_for_positions(position_m, along_m)
    return flatten_state(compute_link_state(trajectory, scenario.base_stations, scenario.radio))


def generate_source_data(scenario):
    along_m = np.linspace(0.0, scenario.corridor.length_m, N_LONGITUDINAL)
    center_xy = scenario.corridor.interpolate(along_m, 0.0)
    center_position = np.column_stack(
        [center_xy, np.full(N_LONGITUDINAL, REFERENCE_ALTITUDE_M)]
    )
    center = evaluate_positions(center_position, along_m, scenario)

    lateral_grid = np.linspace(-LATERAL_HALF_WIDTH_M, LATERAL_HALF_WIDTH_M, N_LATERAL)
    altitude_grid = np.linspace(
        REFERENCE_ALTITUDE_M - ALTITUDE_HALF_BAND_M,
        REFERENCE_ALTITUDE_M + ALTITUDE_HALF_BAND_M,
        N_ALTITUDE,
    )
    n_cross = N_LATERAL * N_ALTITUDE
    rsrp_quantiles = np.empty((3, N_LONGITUDINAL), dtype=float)
    sinr_quantiles = np.empty((3, N_LONGITUDINAL), dtype=float)

    for start in range(0, N_LONGITUDINAL, CHUNK_LONGITUDINAL):
        stop = min(start + CHUNK_LONGITUDINAL, N_LONGITUDINAL)
        ss, ll, hh = np.meshgrid(
            along_m[start:stop], lateral_grid, altitude_grid, indexing="ij"
        )
        flat_s = ss.ravel()
        cross_xy = scenario.corridor.interpolate(flat_s, ll.ravel())
        cross_position = np.column_stack([cross_xy, hh.ravel()])
        cross = evaluate_positions(cross_position, flat_s, scenario)
        shape = (stop - start, n_cross)
        rsrp_quantiles[:, start:stop] = np.quantile(
            cross["serving_rsrp_dbm"].reshape(shape), [0.05, 0.50, 0.95], axis=1
        )
        sinr_quantiles[:, start:stop] = np.quantile(
            cross["sinr_db"].reshape(shape), [0.05, 0.50, 0.95], axis=1
        )

    serving = center["serving_index"]
    transition_indices = np.flatnonzero(np.diff(serving) != 0) + 1
    transition_m = 0.5 * (
        along_m[transition_indices - 1] + along_m[transition_indices]
    )
    site_ids = np.asarray(scenario.base_stations.site_ids)

    rows: list[dict] = []
    for i, distance_m in enumerate(along_m):
        site_index = int(serving[i])
        rows.append(
            {
                "corridor_distance_km": float(distance_m / 1000.0),
                "time_at_50_mps_s": float(distance_m / scenario.speed_mps),
                "serving_site_id": str(site_ids[site_index]),
                "serving_site_index_zero_based": site_index,
                "serving_rx_full_carrier_dbm": float(center["serving_rx_dbm"][i]),
                "serving_rsrp_centerline_dbm_per_re": float(center["serving_rsrp_dbm"][i]),
                "serving_rsrp_cross_section_p05_dbm_per_re": float(rsrp_quantiles[0, i]),
                "serving_rsrp_cross_section_median_dbm_per_re": float(rsrp_quantiles[1, i]),
                "serving_rsrp_cross_section_p95_dbm_per_re": float(rsrp_quantiles[2, i]),
                "aggregate_interference_full_carrier_dbm": float(center["interference_dbm"][i]),
                "sinr_centerline_db": float(center["sinr_db"][i]),
                "sinr_cross_section_p05_db": float(sinr_quantiles[0, i]),
                "sinr_cross_section_median_db": float(sinr_quantiles[1, i]),
                "sinr_cross_section_p95_db": float(sinr_quantiles[2, i]),
                "noise_fraction_centerline": float(center["noise_fraction"][i]),
            }
        )

    station_rows: list[dict] = []
    for index, station in enumerate(scenario.base_stations.stations):
        mask = serving == index
        served_samples = int(mask.sum())
        station_rows.append(
            {
                "site_id": station.site_id,
                "chainage_km": station.attributes.get("chainage_km", ""),
                "lateral_offset_km": station.attributes.get("lateral_offset_km", ""),
                "height_m": float(station.height_m),
                "height_basis": station.attributes.get("height_basis", ""),
                "operator_evidence": station.attributes.get("operator_evidence", ""),
                "serves_centerline": bool(served_samples),
                "centerline_sample_count": served_samples,
                "first_served_km": float(along_m[mask][0] / 1000.0) if served_samples else "",
                "last_served_km": float(along_m[mask][-1] / 1000.0) if served_samples else "",
                "served_centerline_length_km": float(
                    served_samples * scenario.corridor.length_m / N_LONGITUDINAL / 1000.0
                ),
            }
        )

    transition_rows = []
    for sequence, (index, distance_m) in enumerate(
        zip(transition_indices, transition_m), start=1
    ):
        transition_rows.append(
            {
                "transition_sequence": sequence,
                "corridor_distance_km": float(distance_m / 1000.0),
                "from_site_id": str(site_ids[serving[index - 1]]),
                "to_site_id": str(site_ids[serving[index]]),
            }
        )

    # The association and interference identities are checked over the served
    # set, which is what the kernel sums; received_power_dbm still reports every
    # site. With a common EIRP the nearest set is also the strongest set, so
    # serving remains the global maximum and both forms are checked.
    served = center["served_mask"]
    desired_is_max = np.allclose(
        center["serving_rx_dbm"],
        np.max(np.where(served, center["received_power_dbm"], -np.inf), axis=1),
        atol=1e-12,
        rtol=0.0,
    )
    serving_is_global_max = np.allclose(
        center["serving_rx_dbm"],
        np.max(center["received_power_dbm"], axis=1),
        atol=1e-12,
        rtol=0.0,
    )
    received_mw = np.power(10.0, center["received_power_dbm"] / 10.0)
    row_index = np.arange(N_LONGITUDINAL)
    desired_mw = received_mw[row_index, serving]
    interference_identity = np.sum(np.where(served, received_mw, 0.0), axis=1) - desired_mw
    reported_interference_mw = np.power(10.0, center["interference_dbm"] / 10.0)

    qa = {
        "site_count": len(scenario.base_stations.stations),
        "site_ids_sequential": list(scenario.base_stations.site_ids)
        == [f"BS{i:02d}" for i in range(1, 19)],
        "all_site_heights_resolved": all(
            station.height_m is not None and station.height_m > 0
            for station in scenario.base_stations.stations
        ),
        "corridor_length_km": scenario.corridor.length_m / 1000.0,
        "n_longitudinal_positions": N_LONGITUDINAL,
        "n_cross_section_points_per_position": n_cross,
        "n_cross_section_evaluations": N_LONGITUDINAL * n_cross,
        "all_values_finite": bool(
            np.all(np.isfinite(center["serving_rsrp_dbm"]))
            and np.all(np.isfinite(center["sinr_db"]))
            and np.all(np.isfinite(rsrp_quantiles))
            and np.all(np.isfinite(sinr_quantiles))
        ),
        "served_set_size": scenario.radio.served_set_size,
        "interferers_per_position": int(np.max(np.sum(center["served_mask"], axis=1)) - 1),
        "serving_is_max_received_power": bool(desired_is_max),
        "serving_is_global_max_received_power": bool(serving_is_global_max),
        "interference_identity_max_abs_mw": float(
            np.max(np.abs(interference_identity - reported_interference_mw))
        ),
        "association_transition_count": int(len(transition_indices)),
        "serving_site_count": int(len(np.unique(serving))),
        "non_serving_site_ids_on_centerline": [
            station.site_id
            for index, station in enumerate(scenario.base_stations.stations)
            if not np.any(serving == index)
        ],
        "serving_rsrp_centerline_dbm_per_re": {
            "min": float(np.min(center["serving_rsrp_dbm"])),
            "median": float(np.median(center["serving_rsrp_dbm"])),
            "max": float(np.max(center["serving_rsrp_dbm"])),
        },
        "sinr_centerline_db": {
            "min": float(np.min(center["sinr_db"])),
            "median": float(np.median(center["sinr_db"])),
            "max": float(np.max(center["sinr_db"])),
        },
        "cross_section_p05_min": {
            "rsrp_dbm_per_re": float(np.min(rsrp_quantiles[0])),
            "sinr_db": float(np.min(sinr_quantiles[0])),
        },
        "maximum_noise_fraction": float(np.max(center["noise_fraction"])),
        "policy_thresholds_applied": False,
        "shadow_fading_applied": False,
    }
    return rows, station_rows, transition_rows, transition_m / 1000.0, qa


def clean_axes(ax: plt.Axes) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", color="#d9dee7", linewidth=0.55, alpha=0.7)
    ax.set_axisbelow(True)


def site_marker_data(station_rows: list[dict]):
    chainage = np.asarray([float(row["chainage_km"]) for row in station_rows])
    serves = np.asarray([row["serves_centerline"] for row in station_rows], dtype=bool)
    ids = [row["site_id"] for row in station_rows]
    return chainage, serves, ids


def draw_site_markers(ax, station_rows: list[dict], y_min: float, y_max: float) -> None:
    chainage, serves, ids = site_marker_data(station_rows)
    marker_y = y_max - 0.035 * (y_max - y_min)
    for index, (x, active, site_id) in enumerate(zip(chainage, serves, ids)):
        ax.scatter(
            x,
            marker_y,
            marker="v",
            s=23,
            facecolor="#59636f" if active else "white",
            edgecolor="#59636f",
            linewidth=0.65,
            zorder=7,
        )
        row_offset = 0.060 if index % 2 == 0 else 0.105
        ax.text(
            x,
            y_max - row_offset * (y_max - y_min),
            site_id,
            rotation=55,
            ha="right",
            va="top",
            fontsize=5.8,
            color="#4a5563",
        )


def configure_x_axis(ax, corridor_length_km: float) -> None:
    tick_stop = math.floor(corridor_length_km / 10.0) * 10.0
    ticks = list(np.arange(0.0, tick_stop + 0.1, 10.0))
    if corridor_length_km - ticks[-1] > 0.5:
        ticks.append(corridor_length_km)
    ax.set_xlim(0.0, corridor_length_km)
    ax.set_xticks(ticks)
    ax.set_xticklabels(
        [f"{tick:.0f}" if abs(tick - round(tick)) < 0.05 else f"{tick:.1f}" for tick in ticks]
    )
    ax.set_xlabel("Distance along SF–SJ corridor (km)")


def add_common_legend(ax, line, band, transition_handle) -> None:
    ax.legend(
        handles=[line, band, transition_handle],
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=3,
        handlelength=2.2,
        columnspacing=1.1,
    )


def plot_profile(
    rows: list[dict],
    station_rows: list[dict],
    transitions_km: np.ndarray,
    metric: str,
):
    x = np.asarray([row["corridor_distance_km"] for row in rows])
    if metric == "rsrp":
        center_key = "serving_rsrp_centerline_dbm_per_re"
        p05_key = "serving_rsrp_cross_section_p05_dbm_per_re"
        p95_key = "serving_rsrp_cross_section_p95_dbm_per_re"
        line_color, band_color = "#174a7e", "#a8c7e6"
        line_label = "Centerline serving RSRP"
        ylabel = "Serving RSRP (dBm per RE)"
        stem = "rsrp"
    else:
        center_key = "sinr_centerline_db"
        p05_key = "sinr_cross_section_p05_db"
        p95_key = "sinr_cross_section_p95_db"
        line_color, band_color = "#145c58", "#add8d1"
        line_label = "Centerline SINR"
        ylabel = "C2 SINR (dB)"
        stem = "sinr"

    center = np.asarray([row[center_key] for row in rows])
    p05 = np.asarray([row[p05_key] for row in rows])
    p95 = np.asarray([row[p95_key] for row in rows])
    fig, ax = plt.subplots(figsize=(9.2, 3.25))
    band = ax.fill_between(
        x,
        p05,
        p95,
        color=band_color,
        alpha=0.56,
        linewidth=0,
        label="Cross-section 5th–95th percentile",
        zorder=1,
    )
    line = ax.plot(x, center, color=line_color, linewidth=1.75, label=line_label, zorder=4)[0]
    for transition in transitions_km:
        ax.axvline(transition, color="#7a8594", linewidth=0.65, linestyle=":", zorder=0)
    y_min = float(np.floor(np.min(p05) - 1.0))
    y_max = float(np.ceil(np.max(p95) + 4.0))
    ax.set_ylim(y_min, y_max)
    configure_x_axis(ax, float(x[-1]))
    ax.set_ylabel(ylabel)
    clean_axes(ax)
    draw_site_markers(ax, station_rows, y_min, y_max)
    transition_handle = Line2D(
        [0], [0], color="#7a8594", linewidth=0.8, linestyle=":", label="Association transition"
    )
    add_common_legend(ax, line, band, transition_handle)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    return fig, stem


def plot_combined(rows, station_rows, transitions_km):
    x = np.asarray([row["corridor_distance_km"] for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 6.15), sharex=True)
    settings = [
        (
            axes[0],
            "serving_rsrp_centerline_dbm_per_re",
            "serving_rsrp_cross_section_p05_dbm_per_re",
            "serving_rsrp_cross_section_p95_dbm_per_re",
            "#174a7e",
            "#a8c7e6",
            "Serving RSRP (dBm per RE)",
            "Centerline serving RSRP",
        ),
        (
            axes[1],
            "sinr_centerline_db",
            "sinr_cross_section_p05_db",
            "sinr_cross_section_p95_db",
            "#145c58",
            "#add8d1",
            "C2 SINR (dB)",
            "Centerline SINR",
        ),
    ]
    for panel, (ax, center_key, p05_key, p95_key, color, fill, ylabel, label) in enumerate(settings):
        center = np.asarray([row[center_key] for row in rows])
        p05 = np.asarray([row[p05_key] for row in rows])
        p95 = np.asarray([row[p95_key] for row in rows])
        ax.fill_between(x, p05, p95, color=fill, alpha=0.56, linewidth=0, zorder=1)
        ax.plot(x, center, color=color, linewidth=1.75, zorder=4)
        for transition in transitions_km:
            ax.axvline(transition, color="#7a8594", linewidth=0.65, linestyle=":", zorder=0)
        y_min = float(np.floor(np.min(p05) - 1.0))
        y_max = float(np.ceil(np.max(p95) + (4.0 if panel == 0 else 3.0)))
        ax.set_ylim(y_min, y_max)
        ax.set_ylabel(ylabel)
        clean_axes(ax)
        ax.text(-0.055, 1.02, "ab"[panel], transform=ax.transAxes, fontweight="bold", fontsize=11)
        if panel == 0:
            draw_site_markers(ax, station_rows, y_min, y_max)
        ax.text(0.01, 0.06, label, transform=ax.transAxes, color=color, fontsize=8.2, fontweight="bold")
    configure_x_axis(axes[-1], float(x[-1]))
    fig.legend(
        handles=[
            Line2D([0], [0], color="#174a7e", linewidth=2.2, label="Centerline RSRP"),
            Line2D([0], [0], color="#145c58", linewidth=2.2, label="Centerline SINR"),
            Patch(
                facecolor="#b8d1dc",
                edgecolor="none",
                alpha=0.55,
                label="Cross-section 5th–95th percentile",
            ),
            Line2D(
                [0],
                [0],
                color="#7a8594",
                linewidth=0.8,
                linestyle=":",
                label="Association transition",
            ),
        ],
        frameon=False,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=4,
        fontsize=8.2,
    )
    fig.subplots_adjust(left=0.09, right=0.985, top=0.91, bottom=0.10, hspace=0.20)
    return fig, "profile"


def save_figure(fig: plt.Figure, stem: str) -> list[Path]:
    paths = []
    for suffix, kwargs in ((".svg", {}), (".png", {"dpi": 300})):
        path = FIGURE_DIR / f"{stem}{suffix}"
        fig.savefig(path, **kwargs)
        paths.append(path)
    plt.close(fig)
    return paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "runs" / "link_quality",
        help="New output directory (must not already contain a run).",
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main() -> None:
    global RUN_DIR, DATA_DIR, FIGURE_DIR
    args = parse_args()
    RUN_DIR = args.output.resolve()
    DATA_DIR = RUN_DIR
    FIGURE_DIR = RUN_DIR / "figures"
    prepare_run(args.overwrite)
    scenario = load_scenario(SCENARIO_PATH)
    if not scenario.radio_ready:
        raise RuntimeError("Scenario is not radio-ready.")

    rows, station_rows, transition_rows, transitions_km, qa = generate_source_data(scenario)
    source_path = DATA_DIR / "profile.csv"
    station_path = DATA_DIR / "sites.csv"
    transition_path = DATA_DIR / "transitions.csv"
    write_csv(source_path, rows)
    write_csv(station_path, station_rows)
    write_csv(transition_path, transition_rows)

    config = {
        "run_id": RUN_DIR.name,
        "scenario_id": scenario.scenario_id,
        "scenario_path": repository_path(SCENARIO_PATH),
        "corridor_length_km": scenario.corridor.length_m / 1000.0,
        "site_count": len(scenario.base_stations.stations),
        "site_ids": list(scenario.base_stations.site_ids),
        "reference_altitude_m": REFERENCE_ALTITUDE_M,
        "altitude_half_band_m": ALTITUDE_HALF_BAND_M,
        "altitude_points": N_ALTITUDE,
        "lateral_half_width_m": LATERAL_HALF_WIDTH_M,
        "lateral_points": N_LATERAL,
        "longitudinal_points": N_LONGITUDINAL,
        "speed_mps": scenario.speed_mps,
        "radio": {
            "frequency_ghz": scenario.radio.frequency_ghz,
            "full_carrier_eirp_dbm": scenario.radio.eirp_dbm,
            "receiver_gain_db": scenario.radio.receiver_gain_db,
            "full_carrier_noise_dbm": scenario.radio.noise_dbm,
            "resource_elements": scenario.radio.resource_elements,
            "rsrp_definition": "serving_rx_full_carrier_dbm - 10log10(resource_elements)",
        },
        "served_set_size": scenario.radio.served_set_size,
        "interference_assumption": (
            "all 18 retained sites co-channel and simultaneously active"
            if scenario.radio.served_set_size is None
            else (
                f"UAM associates with the nearest {scenario.radio.served_set_size} sites; "
                f"the strongest serves and the remaining "
                f"{scenario.radio.served_set_size - 1} are co-channel interferers"
            )
        ),
        "policy_thresholds_applied": False,
        "random_seed": None,
        "nondeterminism": "none; deterministic geometry and LOS path loss",
    }
    config_path = DATA_DIR / "config.json"
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")

    figure_paths = []
    for metric in ("rsrp", "sinr"):
        figure_paths.extend(save_figure(*plot_profile(rows, station_rows, transitions_km, metric)))
    figure_paths.extend(save_figure(*plot_combined(rows, station_rows, transitions_km)))

    qa["source_files_present"] = all(
        path.exists() for path in (source_path, station_path, transition_path, config_path)
    )
    qa["figure_export_count"] = len(figure_paths)
    qa["all_figure_exports_present"] = all(path.exists() and path.stat().st_size > 0 for path in figure_paths)
    qa["status"] = "pass" if (
        qa["site_count"] == 18
        and qa["site_ids_sequential"]
        and qa["all_site_heights_resolved"]
        and qa["all_values_finite"]
        and qa["serving_is_max_received_power"]
        and qa["interference_identity_max_abs_mw"] < 1e-18
        and qa["source_files_present"]
        and qa["all_figure_exports_present"]
    ) else "fail"
    qa_path = DATA_DIR / "qa.json"
    qa_path.write_text(json.dumps(qa, indent=2), encoding="utf-8")

    outputs = [source_path, station_path, transition_path, config_path, qa_path, *figure_paths]
    manifest = {
        "schema_version": 1,
        "run_id": RUN_DIR.name,
        "stage": "link-quality",
        "standalone": True,
        "status": "awaiting_review" if qa["status"] == "pass" else "failed",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "objective": "Generate serving RSRP and SINR profiles for the 18-site SF-SJ real corridor.",
        "inputs": [
            {"path": repository_path(path), "sha256": sha256(path)}
            for path in (SCENARIO_PATH, SITE_PATH, CORRIDOR_PATH)
        ],
        "command": "python scripts/run_link_quality.py --output runs/link_quality",
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
            "matplotlib": mpl.__version__,
        },
        "outputs": [
            {"path": path.relative_to(RUN_DIR).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)}
            for path in outputs
        ],
        "qa_file": qa_path.relative_to(RUN_DIR).as_posix(),
        "qa_status": qa["status"],
        "evidence_grade": "estimated within a deterministic planning model",
        "evidence_boundary": (
            "Co-channel LOS planning diagnostic; not measured coverage, verified operator interoperability, "
            "airborne antenna validation, policy classification, or corridor capacity."
        ),
    }
    manifest_path = RUN_DIR / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    review = f"""# Link-quality review packet - SF-SJ real corridor

## Objective and acceptance criteria

Generate full-corridor serving-RSRP and SINR profiles from the retained 18-site scenario. Required checks are encoded in `qa.json`.

## Inputs and provenance

- Scenario, corridor and site-table checksums are recorded in `manifest.json`.
- 18 active sites; BS19 is not part of this run.
- Site-specific/class-imputed heights are read from the versioned station CSV.

## Specification

- Centerline trajectory: altitude 300 m.
- Spatial envelope: lateral ±500 m and altitude 300±60 m; 147 points at each of 2,001 longitudinal positions.
- Radio: 5 GHz, 46 dBm full-carrier EIRP, 0 dB receiver gain, −99 dBm full-carrier noise, deterministic LOS.
- Served set: the nearest {qa['served_set_size']} sites; the strongest of that set serves and the remaining {qa['interferers_per_position']} are co-channel interferers.
- Strict RSRP is reported per resource element using 300 RE; SINR uses full-carrier desired/interference/noise powers.

## Outputs

- `profile.csv`
- `sites.csv`
- `transitions.csv`
- Three figures in editable SVG and preview PNG formats.

## Validation

- QA status: **{qa['status']}**.
- Longitudinal positions: {qa['n_longitudinal_positions']:,}.
- Cross-section evaluations: {qa['n_cross_section_evaluations']:,}.
- Serving sites on centerline: {qa['serving_site_count']} of 18.
- Association transitions: {qa['association_transition_count']}.
- All values finite: {qa['all_values_finite']}.
- Serving association equals maximum received power within the served set: {qa['serving_is_max_received_power']}.
- Serving association also equals the global maximum across all 18 sites: {qa['serving_is_global_max_received_power']}.

## Results boundary

These are model-estimated spatial diagnostics. The shaded envelope is prescribed geometric variation, not statistical uncertainty. No C/R/F policy or capacity conclusion is made.

## Decision requested

Review the two profile figures and either approve this baseline, request a figure revision, or request a new sensitivity branch (for example operator-specific active sets or antenna-pattern assumptions).
"""
    (RUN_DIR / "REVIEW.md").write_text(review, encoding="utf-8")

    if qa["status"] != "pass":
        raise RuntimeError(f"QA failed; inspect {qa_path}")
    print(json.dumps({"run_dir": str(RUN_DIR), "qa": qa}, indent=2))


if __name__ == "__main__":
    main()
