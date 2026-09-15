#!/usr/bin/env python3
"""Build the external-facing R0056 capacity story as one academic figure.

The figure is intentionally organized for a first-time reader: matched study
design, capacity definition, full-corridor behavior, aggregate outcome, and the
finite-demand tail diagnostic are presented in that order.
"""
from __future__ import annotations

import csv
import gzip
import json
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "research/dynamic-transitions/runs/R0056"
OUT = ROOT / "research/dynamic-transitions/figures/R0056"
SCENARIO = ROOT / "scenarios/airport_to_airport"

INK = "#183247"
GREY = "#667580"
LIGHT_GREY = "#DCE3E8"
GRID_GREY = "#E8EDF0"
FIXED = "#6E7781"
SPATIAL = "#2B6F9F"
POLICY = {"C": "#218A78", "R": "#E4A72A", "F": "#C9554A"}
SPACING = {"C": 1319.9, "R": 2069.9, "F": 3569.9}
Q_REF = 180000.0


mpl.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
    "font.size": 8.2,
    "axes.titlesize": 9.2,
    "axes.labelsize": 8.2,
    "xtick.labelsize": 7.2,
    "ytick.labelsize": 7.2,
    "axes.spines.right": False,
    "axes.spines.top": False,
    "axes.linewidth": 0.7,
    "legend.frameon": False,
    "svg.fonttype": "none",
    "pdf.fonttype": 42,
})


def load_json(path: Path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def capacity_rows(case: str) -> list[dict[str, float]]:
    with (RUN / case / "capacity_trace.csv").open(
            encoding="utf-8-sig", newline="") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def group_trace(trace: list[dict]) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in trace:
        grouped[row["aircraft_id"]].append(row)
    for rows in grouped.values():
        rows.sort(key=lambda row: float(row["t_s"]))
    return grouped


def station_rows() -> list[dict]:
    scenario = load_json(SCENARIO / "scenario.json")
    active = set(scenario["base_stations"]["active_site_ids"])
    with (SCENARIO / "data/base_stations.csv").open(
            encoding="utf-8-sig", newline="") as stream:
        return [
            {"site": row["public_site_id"],
             "q_km": float(row["chainage_km"]),
             "offset_km": float(row["lateral_offset_km"])}
            for row in csv.DictReader(stream)
            if row["public_site_id"] in active
        ]


def local_profile(trace: list[dict], bin_width_m: float = 1000.0) -> list[dict]:
    buckets: dict[int, Counter] = defaultdict(Counter)
    max_q = max(float(row["q_m"]) for row in trace)
    count = int(np.ceil(max_q / bin_width_m))
    for row in trace:
        index = min(int(float(row["q_m"]) // bin_width_m), count - 1)
        buckets[index][row["policy"]] += float(row["dt_s"])
    result = []
    for index in range(count):
        counts = buckets[index]
        total = sum(counts.values())
        shares = {policy: counts[policy] / total for policy in "CRF"}
        mean_spacing = sum(shares[p] * SPACING[p] for p in "CRF")
        result.append({
            "q_km": (index + 0.5) * bin_width_m / 1000.0,
            **{f"share_{p}": shares[p] for p in "CRF"},
            "local_capacity_uam_h": Q_REF / mean_spacing,
        })
    return result


def colored_line(ax, x, y, policies, *, lw=0.8, alpha=0.55, zorder=2):
    xy = np.column_stack([x, y])
    if len(xy) < 2:
        return
    segments = np.stack([xy[:-1], xy[1:]], axis=1)
    ax.add_collection(LineCollection(
        segments, colors=[POLICY[p] for p in policies[:-1]],
        linewidths=lw, alpha=alpha, zorder=zorder,
        capstyle="round", joinstyle="round"))


def panel_label(ax, label: str, title: str):
    ax.set_title(f"{label}  {title}", loc="left", color=INK,
                 fontweight="bold", pad=7)


def draw_grid_design(ax):
    ax.set_xlim(-0.2, 6.8)
    ax.set_ylim(-0.65, 2.85)
    ax.axis("off")
    panel_label(ax, "a", "Matched comparison: same space and same centre entry")

    for origin, free in ((0.7, False), (4.15, True)):
        for d in range(3):
            for h in range(3):
                ax.scatter(origin + d * 0.68, h * 0.68, s=42,
                           facecolor="white", edgecolor=LIGHT_GREY,
                           linewidth=0.9, zorder=3)
        center = np.array([origin + 0.68, 0.68])
        if free:
            for dd in (-1, 0, 1):
                for dh in (-1, 0, 1):
                    if dd == dh == 0:
                        continue
                    target = center + np.array([dd, dh]) * 0.68
                    ax.annotate("", xy=target, xytext=center,
                                arrowprops=dict(arrowstyle="-|>", color=SPATIAL,
                                                lw=0.7, alpha=0.5))
            ax.scatter(*center, s=70, facecolor=SPATIAL, edgecolor="white",
                       linewidth=0.9, zorder=5)
            heading = "Spatial control ON"
            detail = "adjacent moves available\nacross the 3 × 3 grid"
        else:
            ax.scatter(*center, s=70, facecolor=FIXED, edgecolor="white",
                       linewidth=0.9, zorder=5)
            for d in range(3):
                for h in range(3):
                    if d == h == 1:
                        continue
                    ax.plot(origin + d * 0.68, h * 0.68, marker="x",
                            color="#B9C1C6", ms=4, mew=0.8, zorder=4)
            heading = "Spatial control OFF"
            detail = "centre state only\n(d = 0 m, h = 300 m)"
        ax.annotate("centre entry", xy=center, xytext=(center[0], 2.40),
                    ha="center", color=GREY, fontsize=7.2,
                    arrowprops=dict(arrowstyle="->", color=GREY, lw=0.8))
        ax.text(center[0], -0.18, heading, ha="center", color=INK,
                fontsize=8.1, fontweight="bold")
        ax.text(center[0], -0.45, detail, ha="center", va="top",
                color=GREY, fontsize=7.0, linespacing=1.25)


def draw_capacity_definition(ax):
    ax.axis("off")
    panel_label(ax, "b", "Conditional capacity follows the policy–spacing law")
    ax.text(0.01, 0.76,
            r"At each 5 s snapshot:  $\bar{s}(t)=\frac{n_Cs_C+n_Rs_R+n_Fs_F}{N(t)}$",
            transform=ax.transAxes, color=INK, fontsize=9.0)
    ax.text(0.01, 0.49,
            r"$q(t)=\frac{3600\times 50}{\bar{s}(t)}$  and  "
            r"$q_{95}=\mathrm{5th\ percentile}\{q(t)\}$",
            transform=ax.transAxes, color=INK, fontsize=10.2)
    items = (("C", 1319.9, 136.4), ("R", 2069.9, 87.0),
             ("F", 3569.9, 50.4))
    for index, (policy, spacing, rate) in enumerate(items):
        x = 0.05 + index * 0.31
        ax.plot([x, x + 0.07], [0.19, 0.19], transform=ax.transAxes,
                color=POLICY[policy], lw=5, solid_capstyle="butt")
        ax.text(x + 0.085, 0.19,
                f"{policy}: {spacing:,.1f} m  →  {rate:.1f} UAM/h",
                transform=ax.transAxes, va="center", color=GREY, fontsize=7.2)
    ax.text(0.01, 0.01,
            "Each active aircraft is counted once; empty grid cells do not add capacity.",
            transform=ax.transAxes, color=GREY, fontsize=7.1)


def draw_station_markers(ax, stations, *, labels=False, inside=False,
                         label_sites=None):
    ymax = ax.get_ylim()[1]
    marker_y = ymax * (0.72 if inside else 1.0)
    for row in stations:
        ax.scatter(row["q_km"], marker_y, marker="v", s=15,
                   facecolor="white", edgecolor="#7D8991", linewidth=0.65,
                   clip_on=inside, zorder=6)
        if labels and (label_sites is None or row["site"] in label_sites):
            rotation = 52 if row["site"] not in {"BS13", "BS14"} else 72
            label_y = ymax * (0.47 if inside else 1.015)
            ax.text(row["q_km"], label_y, row["site"], rotation=rotation,
                    ha="left", va="bottom", fontsize=5.8, color=GREY,
                    clip_on=inside)


def draw_trajectories(container, fixed_trace, spatial_trace, stations):
    container.axis("off")
    panel_label(container, "c", "Full-corridor policy and realized spatial trajectories")
    sub = container.get_subplotspec().subgridspec(
        4, 1, height_ratios=(0.10, 0.60, 1, 1), hspace=0.15)

    # Fixed centerline: dominant policy in 0.5 km bins.
    ax0 = container.figure.add_subplot(sub[1, 0])
    buckets: dict[int, Counter] = defaultdict(Counter)
    for row in fixed_trace:
        buckets[int(float(row["q_m"]) // 500.0)][row["policy"]] += 1
    q = np.asarray([(index + 0.5) * 0.5 for index in sorted(buckets)])
    policy = [buckets[index].most_common(1)[0][0] for index in sorted(buckets)]
    for x0, x1, p in zip(q[:-1], q[1:], policy[:-1]):
        ax0.plot([x0, x1], [0, 0], color=POLICY[p], lw=7,
                 solid_capstyle="butt")
    ax0.set_xlim(0, 50)
    ax0.set_ylim(-0.5, 0.5)
    ax0.set_yticks([0], ["Fixed\ncentre"])
    ax0.set_xticks([])
    ax0.spines[["left", "bottom"]].set_visible(False)
    draw_station_markers(ax0, stations, inside=True)
    close_pair = [row for row in stations if row["site"] in {"BS13", "BS14"}]
    for row in close_pair:
        ax0.scatter(row["q_km"], 0.36, marker="v", s=20,
                    facecolor="white", edgecolor=POLICY["F"], linewidth=0.9,
                    zorder=7)
    ax0.text(np.mean([row["q_km"] for row in close_pair]), 0.14,
             "BS13–BS14", ha="center", va="center", fontsize=5.9,
             color=POLICY["F"])
    ax0.text(0.99, 0.93, "triangles: base stations",
             transform=ax0.transAxes, ha="right", va="top",
             fontsize=6.0, color=GREY)

    grouped = group_trace(spatial_trace)
    identifiers = sorted(grouped)
    stride = max(1, len(identifiers) // 18)
    chosen = identifiers[::stride][:18]
    for axis_index, (field, ylabel, ylim, yticks) in enumerate((
            ("offset_m", "Lateral offset d (m)", (-420, 420), [-300, 0, 300]),
            ("altitude", "Altitude h (m)", (170, 430), [200, 300, 400])), start=2):
        ax = container.figure.add_subplot(sub[axis_index, 0])
        for aircraft_id in chosen:
            rows = grouped[aircraft_id][::4]
            x = np.asarray([float(row["q_m"]) / 1000.0 for row in rows])
            if field == "altitude":
                y = np.asarray([float(row["xyz"][2]) for row in rows])
            else:
                y = np.asarray([float(row[field]) for row in rows])
            colored_line(ax, x, y, [row["policy"] for row in rows],
                         lw=0.85, alpha=0.52)
        ax.set_xlim(0, 50)
        ax.set_ylim(*ylim)
        ax.set_yticks(yticks)
        ax.set_ylabel(ylabel)
        ax.grid(color=GRID_GREY, lw=0.55)
        if axis_index == 2:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("Corridor progress q (km)")


def draw_profile_axis(ax, profile, title, stations, *, station_labels=False):
    x = np.asarray([row["q_km"] for row in profile])
    shares = [np.asarray([100 * row[f"share_{p}"] for row in profile])
              for p in "CRF"]
    ax.stackplot(x, *shares, colors=[POLICY[p] for p in "CRF"], alpha=0.88,
                 linewidth=0)
    ax.set_xlim(0, 50)
    ax.set_ylim(0, 100)
    ax.set_ylabel("Policy share (%)")
    ax.set_title(title, loc="left", fontsize=8.2, fontweight="bold",
                 color=INK, pad=4)
    ax.grid(axis="x", color=GRID_GREY, lw=0.55)
    draw_station_markers(ax, stations, labels=station_labels)
    capacity = np.asarray([row["local_capacity_uam_h"] for row in profile])
    ax2 = ax.twinx()
    ax2.plot(x, capacity, color=INK, lw=1.25)
    ax2.set_ylim(45, 142)
    ax2.set_ylabel("Local conditional\ncapacity (UAM/h)", color=INK)
    ax2.tick_params(axis="y", colors=INK, labelsize=6.8)
    ax2.spines["top"].set_visible(False)
    return ax2


def draw_profiles(container, fixed_profile, spatial_profile, stations):
    container.axis("off")
    panel_label(container, "d", "Where policy changes create the capacity difference")
    sub = container.get_subplotspec().subgridspec(
        3, 1, height_ratios=(0.10, 1, 1), hspace=0.30)
    ax0 = container.figure.add_subplot(sub[1, 0])
    draw_profile_axis(ax0, fixed_profile, "Spatial control OFF", stations)
    ax0.set_xticklabels([])
    ax1 = container.figure.add_subplot(sub[2, 0])
    draw_profile_axis(ax1, spatial_profile, "Spatial control ON", stations)
    ax1.set_xlabel("Corridor progress q (km)")
    handles = [mpl.patches.Patch(color=POLICY[p], label={
        "C": "Coordinated", "R": "Reactive", "F": "Fallback"}[p])
        for p in "CRF"]
    handles.append(Line2D([0], [0], color=INK, lw=1.4,
                          label="Local conditional capacity"))
    ax1.legend(handles=handles, ncol=4, loc="upper center",
               bbox_to_anchor=(0.5, -0.34), fontsize=6.7,
               handlelength=1.6, columnspacing=1.3)


def draw_outcomes(container, summaries, move_types):
    container.axis("off")
    panel_label(container, "e", "Aggregate outcome")
    sub = container.get_subplotspec().subgridspec(
        1, 3, width_ratios=(1.45, 0.82, 1.15), wspace=0.38)

    ax0 = container.figure.add_subplot(sub[0, 0])
    labels = ["Spatial control OFF", "Spatial control ON"]
    left = np.zeros(2)
    for policy in "CRF":
        values = 100 * np.asarray([
            summaries[case]["policy_and_longitudinal_metrics"]["policy_shares"][policy]
            for case in ("fixed_centerline", "spatial_grid")])
        ax0.barh([0, 1], values, left=left, height=0.48,
                 color=POLICY[policy])
        for y, start, value in zip([0, 1], left, values):
            if value >= 5:
                ax0.text(start + value / 2, y, f"{value:.1f}%",
                         ha="center", va="center", fontsize=6.9,
                         color="white" if policy != "R" else INK,
                         fontweight="bold")
        left += values
    ax0.set_yticks([0, 1], labels)
    ax0.set_xlim(0, 100)
    ax0.set_xlabel("Aircraft-time share (%)")
    ax0.invert_yaxis()
    ax0.grid(axis="x", color=GRID_GREY, lw=0.55)

    ax1 = container.figure.add_subplot(sub[0, 1])
    fixed = summaries["fixed_centerline"]["conditional_planning_capacity"]["q_mix_rho_uam_h"]
    spatial = summaries["spatial_grid"]["conditional_planning_capacity"]["q_mix_rho_uam_h"]
    ax1.plot([0, 1], [fixed, spatial], color=LIGHT_GREY, lw=4, zorder=1)
    ax1.scatter([0, 1], [fixed, spatial], s=58,
                color=[FIXED, SPATIAL], edgecolor="white", linewidth=0.8, zorder=3)
    ax1.text(0, fixed - 4.2, f"{fixed:.1f}", ha="center", color=FIXED,
             fontsize=8.3, fontweight="bold")
    ax1.text(1, spatial + 3.2, f"{spatial:.1f}", ha="center", color=SPATIAL,
             fontsize=8.3, fontweight="bold")
    ax1.text(0.5, (fixed + spatial) / 2 + 3.2,
             f"+{spatial-fixed:.1f} UAM/h\n(+{100*(spatial/fixed-1):.1f}%)",
             ha="center", va="center", color=SPATIAL, fontsize=7.4,
             fontweight="bold")
    ax1.set_xlim(-0.35, 1.35)
    ax1.set_ylim(86, 114)
    ax1.set_xticks([0, 1], ["OFF", "ON"])
    ax1.set_ylabel(r"$q_{95}$ (UAM/h)")
    ax1.grid(axis="y", color=GRID_GREY, lw=0.55)

    ax2 = container.figure.add_subplot(sub[0, 2])
    names = ["Lateral\nonly", "Height\nonly", "Lateral +\nheight"]
    values = [move_types["lateral"], move_types["vertical"],
              move_types["combined"]]
    bars = ax2.bar(names, values, color=["#3D78A1", "#75A6C5", "#AFCBDC"],
                   width=0.62)
    for bar, value in zip(bars, values):
        ax2.text(bar.get_x() + bar.get_width() / 2, value + 1.2, str(value),
                 ha="center", va="bottom", fontsize=7.5, fontweight="bold",
                 color=INK)
    ax2.set_ylim(0, 65)
    ax2.set_ylabel("Completed maneuvers")
    ax2.grid(axis="y", color=GRID_GREY, lw=0.55)
    ax2.text(0.5, 0.97, f"{sum(values)} completed  ·  sampled NMAC = 0",
             transform=ax2.transAxes, ha="center", va="top",
             color=GREY, fontsize=6.9)


def rolling_mean(values: np.ndarray, width: int = 13) -> np.ndarray:
    if len(values) < width:
        return values
    kernel = np.ones(width) / width
    padded = np.pad(values, (width // 2, width // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def draw_tail(container, capacities, last_release_s):
    container.axis("off")
    panel_label(container, "f", "Why the 3×3 curve briefly drops near 50–60 min")
    ax = container.inset_axes([0.02, 0.06, 0.96, 0.77])
    for case, color, label in (("fixed_centerline", FIXED, "Spatial control OFF"),
                               ("spatial_grid", SPATIAL, "Spatial control ON")):
        rows = capacities[case]
        x = np.asarray([row["timestamp_s"] / 60 for row in rows])
        y = np.asarray([row["q_mix_uam_h"] for row in rows])
        ax.plot(x, rolling_mean(y), color=color, lw=1.35, label=label)
    release_min = last_release_s / 60
    ax.axvspan(release_min, 66, color="#EFF2F4", zorder=0)
    ax.axvline(release_min, color=GREY, lw=0.8, ls="--")
    ax.text(release_min + 0.55, 139, "drain phase: no new entries",
            color=GREY, fontsize=6.7, va="top")
    spatial_min = min(capacities["spatial_grid"],
                      key=lambda row: row["q_mix_uam_h"])
    min_x = spatial_min["timestamp_s"] / 60
    min_y = spatial_min["q_mix_uam_h"]
    ax.scatter(min_x, min_y, s=26, color=POLICY["F"], edgecolor="white",
               linewidth=0.7, zorder=4)
    ax.annotate("17 active: 7 C / 7 R / 3 F\ninstantaneous q = 88.9 UAM/h",
                xy=(min_x, min_y), xytext=(50.0, 85),
                arrowprops=dict(arrowstyle="->", color=POLICY["F"], lw=0.8),
                fontsize=6.7, color=INK, ha="left", va="top")
    ax.set_xlim(42, 66)
    ax.set_ylim(78, 142)
    ax.set_xlabel("Simulation time (min)")
    ax.set_ylabel("Conditional capacity (UAM/h)")
    ax.grid(axis="y", color=GRID_GREY, lw=0.55)
    ax.legend(loc="lower left", fontsize=6.7)
    ax.text(0.985, 0.955,
            "Small-cohort tail effect after release ends\n"
            "Longest interval below fixed: 1.5 min\n"
            "Spatial run exits 203 s earlier → no deadlock",
            transform=ax.transAxes, ha="right", va="top", color=GREY,
            fontsize=6.5, linespacing=1.25,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.82, pad=2.0))


def movement_types(events: list[dict], points: list[dict]) -> dict[str, int]:
    counts = {"lateral": 0, "vertical": 0, "combined": 0}
    for event in events:
        if event.get("status") != "change_started":
            continue
        source = points[int(event["source_lane"])]
        target = points[int(event["target_lane"])]
        lateral = source["offset_m"] != target["offset_m"]
        vertical = source["altitude_m"] != target["altitude_m"]
        key = "combined" if lateral and vertical else "lateral" if lateral else "vertical"
        counts[key] += 1
    return counts


def write_source_data(fixed_profile, spatial_profile, capacities, summaries,
                      move_types, last_release_s):
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "03-capacity-story-profile-source.csv").open(
            "w", encoding="utf-8-sig", newline="") as stream:
        fields = ["case", "q_km", "share_C", "share_R", "share_F",
                  "local_capacity_uam_h"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case, profile in (("spatial_control_off", fixed_profile),
                              ("spatial_control_on", spatial_profile)):
            for row in profile:
                writer.writerow({"case": case, **row})
    with (OUT / "03-capacity-story-time-source.csv").open(
            "w", encoding="utf-8-sig", newline="") as stream:
        fields = ["case", "timestamp_s", "active_aircraft", "n_C", "n_R",
                  "n_F", "mean_target_spacing_m", "q_mix_uam_h",
                  "q_bottleneck_uam_h"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case, rows in capacities.items():
            for row in rows:
                writer.writerow({"case": case, **row})
    summary_rows = []
    for case in ("fixed_centerline", "spatial_grid"):
        summary = summaries[case]
        shares = summary["policy_and_longitudinal_metrics"]["policy_shares"]
        capacity = summary["conditional_planning_capacity"]
        summary_rows.append({
            "case": case,
            "C_share": shares["C"], "R_share": shares["R"],
            "F_share": shares["F"],
            "q95_uam_h": capacity["q_mix_rho_uam_h"],
            "mean_capacity_uam_h": capacity["q_mix_mean_uam_h"],
            "last_exit_s": summary["observed_throughput"]["last_exit_s"],
            "last_release_s": last_release_s,
            "moves_lateral_only": move_types["lateral"],
            "moves_height_only": move_types["vertical"],
            "moves_lateral_and_height": move_types["combined"],
        })
    with (OUT / "03-capacity-story-summary-source.csv").open(
            "w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)


def main():
    config = load_json(RUN / "resolved_config.json")
    fixed_trace = load_json(RUN / "fixed_centerline/trace.json.gz")
    spatial_trace = load_json(RUN / "spatial_grid/trace.json.gz")
    fixed_profile = local_profile(fixed_trace)
    spatial_profile = local_profile(spatial_trace)
    capacities = {case: capacity_rows(case)
                  for case in ("fixed_centerline", "spatial_grid")}
    summaries = {case: load_json(RUN / case / "summary.json")
                 for case in ("fixed_centerline", "spatial_grid")}
    events = load_json(RUN / "spatial_grid/events.json")
    move_types = movement_types(events, config["traffic"]["flow_points"])
    stations = station_rows()
    last_release_s = max(float(row["requested_time_s"])
                         for row in config["entries"])

    fig = plt.figure(figsize=(18.0, 10.2), facecolor="white")
    outer = fig.add_gridspec(
        3, 4, height_ratios=(0.88, 2.65, 1.48),
        left=0.045, right=0.975, top=0.885, bottom=0.08,
        hspace=0.47, wspace=0.48)
    ax_a = fig.add_subplot(outer[0, :2])
    ax_b = fig.add_subplot(outer[0, 2:])
    ax_c = fig.add_subplot(outer[1, :2])
    ax_d = fig.add_subplot(outer[1, 2:])
    ax_e = fig.add_subplot(outer[2, :2])
    ax_f = fig.add_subplot(outer[2, 2:])

    draw_grid_design(ax_a)
    draw_capacity_definition(ax_b)
    draw_trajectories(ax_c, fixed_trace, spatial_trace, stations)
    draw_profiles(ax_d, fixed_profile, spatial_profile, stations)
    draw_outcomes(ax_e, summaries, move_types)
    draw_tail(ax_f, capacities, last_release_s)

    fig.text(0.045, 0.962,
             "How spatial control changes policy and conditional corridor capacity",
             color=INK, fontsize=18, fontweight="bold", ha="left", va="top")
    fig.text(0.045, 0.925,
             "Matched Bay Area centre-entry stream  |  90 s assessment window  |  "
             "control OFF: centre state only  |  control ON: adjacent movement on a 3 × 3 lateral–height grid",
             color=GREY, fontsize=8.8, ha="left", va="top")
    fig.text(0.045, 0.026,
             "Capacity is the policy-conditioned planning rate implied by the fixed C/R/F spacing law; it is not an operational saturation-capacity estimate. "
             "The deterministic run passed completion, adjacency, motion-envelope and sampled-NMAC checks.",
             color=GREY, fontsize=7.0, ha="left", va="bottom")

    OUT.mkdir(parents=True, exist_ok=True)
    stem = OUT / "03-capacity-story"
    fig.savefig(stem.with_suffix(".png"), dpi=300, bbox_inches="tight",
                facecolor="white")
    fig.savefig(stem.with_suffix(".svg"), bbox_inches="tight",
                facecolor="white")
    fig.savefig(stem.with_suffix(".pdf"), bbox_inches="tight",
                facecolor="white")
    fig.savefig(stem.with_suffix(".tiff"), dpi=600, bbox_inches="tight",
                facecolor="white", pil_kwargs={"compression": "tiff_lzw"})
    plt.close(fig)
    write_source_data(fixed_profile, spatial_profile, capacities, summaries,
                      move_types, last_release_s)


if __name__ == "__main__":
    main()
