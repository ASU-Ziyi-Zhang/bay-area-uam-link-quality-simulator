"""Create external-facing visual results for the calibrated R0055 study.

The figures deliberately report the final fixed configuration only. They do
not compare the 90 s assessment window with earlier calibration candidates.
"""
from __future__ import annotations

import csv
import gzip
import hashlib
import json
from collections import Counter, defaultdict, deque
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
import numpy as np  # noqa: E402
from pyproj import Transformer  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "research/dynamic-transitions/runs/R0055"
CASE = RUN / "three_lane_lane_change"
SINGLE_RUN = ROOT / "research/dynamic-transitions/runs/R0048"
OUTPUT = ROOT / "research/dynamic-transitions/figures/R0055"
SCENARIO = ROOT / "scenarios/airport_to_airport"

BLUE = "#204F7A"
LIGHT_BLUE = "#2C86C5"
RED = "#C53A2E"
INK = "#243447"
GREY = "#5E7187"
LIGHT_GREY = "#DCE4EA"
PALE_BLUE = "#E8F3F5"
POLICY_COLOURS = {"C": "#168A78", "R": "#E6A625", "F": "#C84A40"}
POLICY_CODE = {"C": 0, "R": 1, "F": 2}
DIRECTION_COLOURS = {"0→1": "#2C86C5", "1→2": "#7B61A8", "1→0": "#D77A2E"}


def read_json(path: Path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def style() -> None:
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
    })


def heading(fig, eyebrow: str, title: str, subtitle: str) -> None:
    fig.text(0.045, 0.945, eyebrow, color=LIGHT_BLUE, fontsize=11.5,
             fontweight="bold")
    fig.text(0.045, 0.890, title, color=BLUE, fontsize=23,
             fontweight="bold")
    fig.add_artist(plt.Line2D([0.045, 0.165], [0.848, 0.848],
                              transform=fig.transFigure, color=RED, linewidth=3))
    fig.text(0.045, 0.815, subtitle, color=GREY, fontsize=10.5)


def add_card(fig, x: float, y: float, width: float, height: float,
             value: str, label: str, colour: str = BLUE) -> None:
    ax = fig.add_axes([x, y, width, height])
    ax.set_facecolor("#F5F8FA")
    for spine in ax.spines.values():
        spine.set_edgecolor("#D8E1E8")
        spine.set_linewidth(0.8)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.06, 0.63, value, transform=ax.transAxes, color=colour,
            fontsize=18, fontweight="bold", va="center")
    ax.text(0.06, 0.25, label, transform=ax.transAxes, color=GREY,
            fontsize=8.8, va="center")


def export(fig, stem: Path) -> list[Path]:
    outputs = []
    for suffix, kwargs in [
        (".png", {"dpi": 220}),
        (".svg", {}),
        (".pdf", {}),
    ]:
        path = stem.with_suffix(suffix)
        fig.savefig(path, bbox_inches="tight", facecolor="white", **kwargs)
        outputs.append(path)
    plt.close(fig)
    return outputs


def flight_order(config: dict) -> tuple[list[str], dict[str, int], dict[str, int]]:
    entries = sorted(config["entries"], key=lambda row: float(row["requested_time_s"]))
    ids = [row["aircraft_id"] for row in entries]
    sequence = {aircraft_id: index + 1 for index, aircraft_id in enumerate(ids)}
    nominal_lane = {row["aircraft_id"]: int(row["lane"]) for row in entries}
    return ids, sequence, nominal_lane


def group_trace(trace: list[dict]) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in trace:
        grouped[row["aircraft_id"]].append(row)
    for rows in grouped.values():
        rows.sort(key=lambda row: float(row["t_s"]))
    return grouped


def nearest_row(rows: list[dict], t_s: float) -> dict:
    return min(rows, key=lambda row: abs(float(row["t_s"]) - t_s))


def load_stations(origin_xy: np.ndarray) -> list[dict]:
    scenario = read_json(SCENARIO / "scenario.json")
    active = set(scenario["base_stations"]["active_site_ids"])
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:26910", always_xy=True)
    stations = []
    with (SCENARIO / "data/base_stations.csv").open(
            "r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["public_site_id"] not in active:
                continue
            x_m, y_m = transformer.transform(float(row["longitude"]),
                                              float(row["latitude"]))
            stations.append({
                "site_id": row["public_site_id"],
                "x_km": (x_m - origin_xy[0]) / 1000.0,
                "y_km": (y_m - origin_xy[1]) / 1000.0,
            })
    return stations


def pair_moves(events: list[dict], grouped: dict[str, list[dict]]) -> list[dict]:
    pending: dict[str, deque[dict]] = defaultdict(deque)
    moves = []
    for event in sorted(events, key=lambda row: float(row["t_s"])):
        status = event["status"]
        aircraft_id = event.get("aircraft_id")
        if status == "change_started":
            move = dict(event)
            start_row = nearest_row(grouped[aircraft_id], float(event["t_s"]))
            move["q_start_km"] = float(start_row["q_m"]) / 1000.0
            move["start_xyz"] = start_row["xyz"]
            pending[aircraft_id].append(move)
            moves.append(move)
        elif status == "transition_completed" and pending[aircraft_id]:
            move = pending[aircraft_id].popleft()
            move["completion_s"] = float(event["t_s"])
            end_row = nearest_row(grouped[aircraft_id], float(event["t_s"]))
            move["q_end_km"] = float(end_row["q_m"]) / 1000.0
            move["end_xyz"] = end_row["xyz"]
    return moves


def policy_matrix(ids: list[str], grouped: dict[str, list[dict]],
                  bin_width_m: float = 500.0) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    max_q = max(float(row["q_m"]) for rows in grouped.values() for row in rows)
    edges = np.arange(0.0, np.ceil(max_q / bin_width_m) * bin_width_m + bin_width_m,
                      bin_width_m)
    matrix = np.full((len(ids), len(edges) - 1), np.nan)
    source = []
    for row_index, aircraft_id in enumerate(ids):
        counts = np.zeros((len(edges) - 1, 3), dtype=int)
        for row in grouped[aircraft_id]:
            column = min(int(float(row["q_m"]) // bin_width_m), len(edges) - 2)
            counts[column, POLICY_CODE[row["policy"]]] += 1
        available = counts.sum(axis=1) > 0
        matrix[row_index, available] = np.argmax(counts[available], axis=1)
        present = np.flatnonzero(~np.isnan(matrix[row_index]))
        for column in np.flatnonzero(np.isnan(matrix[row_index])):
            nearest = present[np.argmin(abs(present - column))]
            matrix[row_index, column] = matrix[row_index, nearest]
        for column, code in enumerate(matrix[row_index].astype(int)):
            source.append({
                "aircraft_id": aircraft_id,
                "distance_bin_start_km": edges[column] / 1000.0,
                "distance_bin_end_km": edges[column + 1] / 1000.0,
                "policy": "CRF"[code],
            })
    return edges, matrix, source


def dominant_lane_segments(trace: list[dict], bin_width_m: float = 250.0):
    max_q = max(float(row["q_m"]) for row in trace)
    n_bins = int(np.ceil(max_q / bin_width_m))
    buckets: dict[tuple[int, int], list[dict]] = defaultdict(list)
    for row in trace:
        column = min(int(float(row["q_m"]) // bin_width_m), n_bins - 1)
        buckets[(int(row["lane"]), column)].append(row)
    output = {}
    for lane in range(3):
        points = []
        policies = []
        for column in range(n_bins):
            rows = buckets.get((lane, column), [])
            if not rows:
                continue
            xyz = np.asarray([row["xyz"][:2] for row in rows], dtype=float)
            points.append(np.median(xyz, axis=0))
            policies.append(Counter(row["policy"] for row in rows).most_common(1)[0][0])
        output[lane] = (np.asarray(points), policies)
    return output


def coloured_line(ax, x: np.ndarray, y: np.ndarray, policies: list[str],
                  linewidth: float, alpha: float, zorder: int = 2) -> None:
    if len(x) < 2:
        return
    points = np.column_stack([x, y])
    segments = np.stack([points[:-1], points[1:]], axis=1)
    colours = [POLICY_COLOURS[p] for p in policies[:-1]]
    ax.add_collection(LineCollection(segments, colors=colours, linewidths=linewidth,
                                     alpha=alpha, zorder=zorder,
                                     capstyle="round", joinstyle="round"))


def figure_corridor(trace: list[dict], grouped: dict[str, list[dict]], moves: list[dict],
                    summary: dict, config: dict, stations: list[dict]) -> tuple[list[Path], list[dict]]:
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    heading(
        fig,
        "RESULT  |  CALIBRATED THREE-LANE BAY AREA CORRIDOR",
        "All 93 Requests Complete; 12 Safe Lane Changes Are Executed",
        "Three lanes at 300 m  ·  32 s global requested-arrival interval  ·  "
        "W = 90 s  ·  radio sample = 2 s  ·  policy update = 5 s  ·  k = 3",
    )

    origin = np.asarray(trace[0]["xyz"][:2], dtype=float)
    map_ax = fig.add_axes([0.055, 0.17, 0.43, 0.58])
    lane_segments = dominant_lane_segments(trace)
    for lane, (xy, policies) in lane_segments.items():
        rel = (xy - origin) / 1000.0
        coloured_line(map_ax, rel[:, 0], rel[:, 1], policies, 3.0, 0.95, 3)
    for station in stations:
        map_ax.scatter(station["x_km"], station["y_km"], marker="^", s=38,
                       facecolor="white", edgecolor=INK, linewidth=0.9, zorder=4)
        map_ax.text(station["x_km"] + 0.25, station["y_km"] + 0.15,
                    station["site_id"], fontsize=6.5, color=GREY, zorder=5)
    for number, move in enumerate(moves, 1):
        xy = (np.asarray(move["start_xyz"][:2]) - origin) / 1000.0
        map_ax.scatter(*xy, s=45, facecolor=BLUE, edgecolor="white", linewidth=0.8,
                       zorder=6)
        map_ax.text(xy[0], xy[1], str(number), ha="center", va="center",
                    color="white", fontsize=6.5, fontweight="bold", zorder=7)
    map_ax.set_aspect("equal", adjustable="datalim")
    map_ax.set_xlabel("Easting relative to corridor origin (km)")
    map_ax.set_ylabel("Northing relative to corridor origin (km)")
    map_ax.set_title("a  Geographic corridor, policy state and base stations",
                     loc="left", color=INK, pad=10)
    map_ax.grid(color="#EDF1F4", linewidth=0.6, zorder=0)

    route_ax = fig.add_axes([0.535, 0.33, 0.42, 0.42])
    changed_ids = {move["aircraft_id"] for move in moves}
    for aircraft_id, rows in grouped.items():
        sample = rows[::4]
        q = np.asarray([float(row["q_m"]) / 1000.0 for row in sample])
        offset = np.asarray([float(row["offset_m"]) for row in sample])
        policies = [row["policy"] for row in sample]
        if aircraft_id in changed_ids:
            coloured_line(route_ax, q, offset, policies, 1.8, 0.85, 3)
        else:
            coloured_line(route_ax, q, offset, policies, 0.55, 0.10, 1)
    for number, move in enumerate(moves, 1):
        route_ax.scatter(move["q_start_km"],
                         (-300.0, 0.0, 300.0)[int(move["source_lane"])],
                         s=52, facecolor=BLUE, edgecolor="white", linewidth=0.8,
                         zorder=6)
        route_ax.text(move["q_start_km"],
                      (-300.0, 0.0, 300.0)[int(move["source_lane"])], str(number),
                      ha="center", va="center", color="white", fontsize=6.5,
                      fontweight="bold", zorder=7)
    route_ax.set_yticks([-300, 0, 300], ["Lane 1  (−300 m)", "Lane 2  (0 m)",
                                        "Lane 3  (+300 m)"])
    route_ax.set_xlabel("Longitudinal distance along corridor, q (km)")
    route_ax.set_ylabel("Realized lateral position")
    route_ax.set_ylim(-440, 440)
    route_ax.set_title("b  Realized trajectories; numbered markers are maneuver starts",
                       loc="left", color=INK, pad=10)
    route_ax.grid(axis="x", color="#EDF1F4", linewidth=0.6)

    for index, policy in enumerate("CRF"):
        route_ax.plot([], [], color=POLICY_COLOURS[policy], linewidth=3,
                      label={"C": "Coordinated (C)", "R": "Reactive (R)",
                             "F": "Fallback (F)"}[policy])
    route_ax.scatter([], [], marker="^", facecolor="white", edgecolor=INK,
                     label="Base station")
    route_ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.21), ncol=4,
                    fontsize=8.2, handlelength=2.2)

    metrics = summary["policy_and_longitudinal_metrics"]
    add_card(fig, 0.535, 0.115, 0.095, 0.085, "93 / 93", "requests completed")
    add_card(fig, 0.642, 0.115, 0.095, 0.085, "12 / 12", "moves completed")
    add_card(fig, 0.749, 0.115, 0.095, 0.085, "777 m", "minimum separation")
    add_card(fig, 0.856, 0.115, 0.095, 0.085, "0", "sampled NMAC")

    takeaway = fig.add_axes([0.055, 0.025, 0.90, 0.055])
    takeaway.set_facecolor(PALE_BLUE)
    takeaway.set_xticks([])
    takeaway.set_yticks([])
    for spine in takeaway.spines.values():
        spine.set_visible(False)
    shares = metrics["policy_shares"]
    takeaway.text(
        0.015, 0.5,
        f"The complete demand stream exits the corridor with "
        f"{100*shares['C']:.1f}% C, {100*shares['R']:.1f}% R and "
        f"{100*shares['F']:.1f}% F; every accepted lane change completes.",
        color=BLUE, fontsize=10.8, fontweight="bold", va="center")

    source = []
    for number, move in enumerate(moves, 1):
        source.append({
            "maneuver": number,
            "aircraft_id": move["aircraft_id"],
            "start_s": move["t_s"],
            "q_start_km": move["q_start_km"],
            "source_lane": int(move["source_lane"]) + 1,
            "target_lane": int(move["target_lane"]) + 1,
        })
    return export(fig, OUTPUT / "01-corridor-trajectory-and-lane-changes"), source


def figure_policy_heatmap(ids: list[str], grouped: dict[str, list[dict]],
                          nominal_lane: dict[str, int], moves: list[dict],
                          summary: dict) -> tuple[list[Path], list[dict]]:
    lane_ids = [[aircraft_id for aircraft_id in ids if nominal_lane[aircraft_id] == lane]
                for lane in range(3)]
    all_ordered = [aircraft_id for lane in lane_ids for aircraft_id in lane]
    edges, all_matrix, source = policy_matrix(all_ordered, grouped)
    matrix_by_id = {aircraft_id: all_matrix[index]
                    for index, aircraft_id in enumerate(all_ordered)}

    fig = plt.figure(figsize=(16, 9), facecolor="white")
    heading(
        fig,
        "RESULT  |  REALIZED POLICY OVER THE COMPLETE DEMAND STREAM",
        "Fallback Remains 1.0% Across 93 Completed Flights",
        "Rows are requests within their assigned entry lane  ·  columns are 0.5 km "
        "corridor bins  ·  circles locate accepted lane-change starts",
    )
    cmap = ListedColormap([POLICY_COLOURS[p] for p in "CRF"])
    panel_positions = [(0.055, 0.18, 0.285, 0.50),
                       (0.357, 0.18, 0.285, 0.50),
                       (0.659, 0.18, 0.285, 0.50)]
    move_lookup = defaultdict(list)
    for number, move in enumerate(moves, 1):
        move_lookup[move["aircraft_id"]].append((number, move["q_start_km"]))
    for lane, (x, y, w, h) in enumerate(panel_positions):
        ax = fig.add_axes([x, y, w, h])
        lane_matrix = np.vstack([matrix_by_id[aircraft_id] for aircraft_id in lane_ids[lane]])
        ax.imshow(lane_matrix, aspect="auto", origin="lower", interpolation="nearest",
                  cmap=cmap, vmin=-0.5, vmax=2.5,
                  extent=[edges[0] / 1000.0, edges[-1] / 1000.0,
                          0.5, len(lane_ids[lane]) + 0.5])
        for row_number, aircraft_id in enumerate(lane_ids[lane], 1):
            for move_number, q_start in move_lookup[aircraft_id]:
                ax.scatter(q_start, row_number, s=36, facecolor="none", edgecolor="white",
                           linewidth=1.1, zorder=4)
                ax.text(q_start, row_number, str(move_number), ha="center", va="center",
                        fontsize=5.5, fontweight="bold", color="white", zorder=5)
        ax.set_title(f"{chr(97 + lane)}  Entry lane {lane + 1}", loc="left",
                     color=INK, pad=9)
        ax.set_xlabel("Corridor distance, q (km)")
        if lane == 0:
            ax.set_ylabel("Request number within entry lane")
        ax.set_yticks([1, 10, 20, 31])
        for spine in ax.spines.values():
            spine.set_visible(True)
            spine.set_color("#AAB7C3")
            spine.set_linewidth(0.7)

    shares = summary["policy_and_longitudinal_metrics"]["policy_shares"]
    share_ax = fig.add_axes([0.055, 0.715, 0.89, 0.055])
    left = 0.0
    for policy in "CRF":
        value = float(shares[policy])
        share_ax.barh([0], [value], left=left, height=0.58,
                      color=POLICY_COLOURS[policy])
        label = f"{policy}  {100*value:.1f}%"
        if value > 0.04:
            share_ax.text(left + value / 2, 0, label, ha="center", va="center",
                          color="white" if policy != "R" else INK,
                          fontsize=10.5, fontweight="bold")
        left += value
    share_ax.annotate("F  1.0%", xy=(1 - shares["F"] / 2, 0.28),
                      xytext=(0.91, 0.96), textcoords="axes fraction", ha="left",
                      color=POLICY_COLOURS["F"], fontsize=9.5, fontweight="bold",
                      arrowprops=dict(arrowstyle="-", color=POLICY_COLOURS["F"], lw=1.0))
    share_ax.set_xlim(0, 1)
    share_ax.set_ylim(-0.55, 0.9)
    share_ax.axis("off")

    takeaway = fig.add_axes([0.055, 0.055, 0.89, 0.065])
    takeaway.set_facecolor(PALE_BLUE)
    takeaway.set_xticks([])
    takeaway.set_yticks([])
    for spine in takeaway.spines.values():
        spine.set_visible(False)
    takeaway.text(
        0.015, 0.5,
        "The calibrated 90 s policy window produces a consistent corridor-level pattern: "
        "C dominates the first half, R dominates the central bottleneck, and F is rare.",
        color=BLUE, fontsize=10.7, fontweight="bold", va="center")
    return export(fig, OUTPUT / "02-policy-along-corridor"), source


def figure_move_results(moves: list[dict], summary: dict, config: dict) -> tuple[list[Path], list[dict]]:
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    heading(
        fig,
        "RESULT  |  LANE-CHANGE EXECUTION AND PREDICTED BENEFIT",
        "All 12 Accepted Maneuvers Complete and Pass the No-Worsening Gate",
        "Numbers match the corridor map  ·  benefit is the controller forecast's "
        "weighted policy-cost reduction over the common horizon",
    )

    timeline_ax = fig.add_axes([0.075, 0.29, 0.44, 0.42])
    benefit_ax = fig.add_axes([0.60, 0.29, 0.34, 0.42])
    y = np.arange(1, len(moves) + 1)
    source = []
    for number, move in enumerate(moves, 1):
        start_min = float(move["t_s"]) / 60.0
        end_min = float(move["completion_s"]) / 60.0
        direction = f"{int(move['source_lane'])}→{int(move['target_lane'])}"
        colour = DIRECTION_COLOURS[direction]
        timeline_ax.plot([start_min, end_min], [number, number], color=colour,
                         linewidth=6, solid_capstyle="round")
        timeline_ax.scatter(start_min, number, s=34, facecolor="white",
                            edgecolor=colour, linewidth=1.2, zorder=3)
        timeline_ax.text(start_min - 0.55, number, str(number), ha="right", va="center",
                         color=INK, fontsize=8, fontweight="bold")
        timeline_ax.text(end_min + 0.55, number, move["aircraft_id"], ha="left",
                         va="center", color=GREY, fontsize=7.2)

        ego = float(move["predicted_ego_policy_cost_improvement_s"])
        group = float(move["predicted_global_policy_cost_improvement_s"])
        benefit_ax.plot([0, group], [number, number], color="#BBD2E3", linewidth=5,
                        solid_capstyle="round", zorder=1)
        benefit_ax.scatter(group, number, s=35, color="#75A9CD", zorder=2)
        benefit_ax.plot([0, ego], [number, number], color=BLUE, linewidth=5,
                        solid_capstyle="round", zorder=3)
        benefit_ax.scatter(ego, number, s=35, color=BLUE, zorder=4)
        benefit_ax.text(group + 6, number, f"{group:.0f}", va="center", fontsize=7.3,
                        color=GREY)
        source.append({
            "maneuver": number,
            "aircraft_id": move["aircraft_id"],
            "start_s": float(move["t_s"]),
            "completion_s": float(move["completion_s"]),
            "duration_s": float(move["completion_s"]) - float(move["t_s"]),
            "q_start_km": float(move["q_start_km"]),
            "source_lane": int(move["source_lane"]) + 1,
            "target_lane": int(move["target_lane"]) + 1,
            "predicted_ego_policy_cost_improvement_s": ego,
            "predicted_affected_group_policy_cost_improvement_s": group,
            "affected_aircraft_count": len(move["affected_aircraft"]),
            "worsened_aircraft_count": len(move["worsened_aircraft"]),
        })

    timeline_ax.invert_yaxis()
    timeline_ax.set_xlabel("Simulation time (min)")
    timeline_ax.set_ylabel("Maneuver number")
    timeline_ax.set_title("a  When each maneuver starts and completes", loc="left",
                          color=INK, pad=10)
    timeline_ax.set_yticks(y)
    timeline_ax.grid(axis="x", color="#EDF1F4", linewidth=0.7)
    for direction, colour in DIRECTION_COLOURS.items():
        timeline_ax.plot([], [], color=colour, linewidth=5,
                         label=f"Lane {int(direction[0])+1} → {int(direction[-1])+1}")
    timeline_ax.legend(loc="upper left", ncol=3, fontsize=7.6,
                       bbox_to_anchor=(0, -0.13))

    benefit_ax.invert_yaxis()
    benefit_ax.set_xlabel("Predicted policy-cost reduction (s)")
    benefit_ax.set_title("b  Benefit retained by the accepted maneuver", loc="left",
                         color=INK, pad=10)
    benefit_ax.set_yticks(y, [str(value) for value in y])
    benefit_ax.grid(axis="x", color="#EDF1F4", linewidth=0.7)
    benefit_ax.scatter([], [], s=35, color=BLUE, label="ego aircraft")
    benefit_ax.scatter([], [], s=35, color="#75A9CD", label="affected group")
    benefit_ax.legend(loc="upper left", ncol=2, fontsize=8,
                      bbox_to_anchor=(0, -0.13))

    metrics = summary["policy_and_longitudinal_metrics"]
    minimum = float(metrics["minimum_sampled_horizontal_separation_m"])
    required = float(config["parameters"]["horizontal_separation_m"])
    changed_aircraft = summary["lane_change_metrics"]["aircraft_with_lane_change"]
    add_card(fig, 0.075, 0.055, 0.145, 0.080, "12 / 12", "started / completed moves")
    add_card(fig, 0.235, 0.055, 0.145, 0.080, str(changed_aircraft),
             "aircraft that changed lane")
    add_card(fig, 0.395, 0.055, 0.145, 0.080, "0", "worsened affected aircraft")
    add_card(fig, 0.600, 0.055, 0.155, 0.080, f"{minimum:.0f} m",
             "minimum sampled separation")
    add_card(fig, 0.770, 0.055, 0.17, 0.080, f"{minimum/required:.1f}×",
             "required horizontal separation")
    return export(fig, OUTPUT / "03-lane-change-timing-benefit-safety"), source


def weighted_shares(rows: list[dict]) -> dict[str, float]:
    totals = Counter()
    for row in rows:
        totals[row["policy"]] += float(row["dt_s"])
    total = sum(totals.values())
    return {policy: totals[policy] / total for policy in "CRF"}


def radio_profile(trace: list[dict], observations: list[dict], bin_width_m: float = 1000.0):
    lookup = {(row["aircraft_id"], float(row["t_s"])): float(row["q_m"])
              for row in trace}
    maximum_q = max(float(row["q_m"]) for row in trace)
    edges = np.arange(0.0, np.ceil(maximum_q / bin_width_m) * bin_width_m + bin_width_m,
                      bin_width_m)
    bins: list[list[dict]] = [[] for _ in range(len(edges) - 1)]
    for row in observations:
        q_m = lookup[(row["aircraft_id"], float(row["t_s"]))]
        index = min(int(q_m // bin_width_m), len(bins) - 1)
        bins[index].append(row)
    site_ids = [f"BS{number:02d}" for number in range(5, 17)]
    result = []
    for index, rows in enumerate(bins):
        sinr = np.asarray([float(row["sinr_db"]) for row in rows])
        exposure = np.asarray([float(row["exposure"]) for row in rows])
        serving = Counter(int(row["serving_bs"]) for row in rows)
        result.append({
            "q_km": (edges[index] + edges[index + 1]) / 2000.0,
            "sinr_p10_db": float(np.quantile(sinr, 0.10)),
            "sinr_median_db": float(np.median(sinr)),
            "sinr_p90_db": float(np.quantile(sinr, 0.90)),
            "current_below_threshold_pct": 100.0 * float(np.mean(sinr < -2.0)),
            "exposure_p10_pct": 100.0 * float(np.quantile(exposure, 0.10)),
            "exposure_median_pct": 100.0 * float(np.median(exposure)),
            "exposure_p90_pct": 100.0 * float(np.quantile(exposure, 0.90)),
            "dominant_serving_bs": site_ids[serving.most_common(1)[0][0]],
        })
    return result


def serving_segments(profile: list[dict]) -> list[tuple[float, float, str]]:
    segments = []
    start = profile[0]["q_km"] - 0.5
    previous = profile[0]["dominant_serving_bs"]
    for row in profile[1:]:
        if row["dominant_serving_bs"] != previous:
            boundary = row["q_km"] - 0.5
            segments.append((start, boundary, previous))
            start, previous = boundary, row["dominant_serving_bs"]
    segments.append((start, profile[-1]["q_km"] + 0.5, previous))
    return segments


def figure_policy_comparison(single_trace: list[dict], multi_trace: list[dict],
                             observations: list[dict], nominal_lane: dict[str, int],
                             summary: dict) -> tuple[list[Path], list[dict]]:
    series = [
        ("Single lane\n0 m", weighted_shares(single_trace)),
        ("Three lanes\noverall", weighted_shares(multi_trace)),
    ]
    for lane, offset in enumerate((-300, 0, 300)):
        rows = [row for row in multi_trace
                if nominal_lane[row["aircraft_id"]] == lane]
        series.append((f"Entry lane {lane + 1}\n{offset:+d} m", weighted_shares(rows)))

    profile = radio_profile(multi_trace, observations)
    q = np.asarray([row["q_km"] for row in profile])
    sinr_p10 = np.asarray([row["sinr_p10_db"] for row in profile])
    sinr_median = np.asarray([row["sinr_median_db"] for row in profile])
    sinr_p90 = np.asarray([row["sinr_p90_db"] for row in profile])
    exp_p10 = np.asarray([row["exposure_p10_pct"] for row in profile])
    exp_median = np.asarray([row["exposure_median_pct"] for row in profile])
    exp_p90 = np.asarray([row["exposure_p90_pct"] for row in profile])
    current_bad = np.asarray([row["current_below_threshold_pct"] for row in profile])

    fig = plt.figure(figsize=(16, 9), facecolor="white")
    heading(
        fig,
        "DIAGNOSIS  |  SINGLE-LANE VERSUS THREE-LANE POLICY",
        "Reactive Does Not Increase Overall; the −300 m Entry Lane Carries the Penalty",
        "All cases use Θ = −2.0 dB  ·  radio sample = 2 s  ·  W = 90 s  ·  "
        "policy update = 5 s  ·  5%/10% exposure gates  ·  k = 3",
    )

    bar_ax = fig.add_axes([0.065, 0.22, 0.37, 0.50])
    y = np.arange(len(series))
    left = np.zeros(len(series))
    for policy in "CRF":
        values = np.asarray([100.0 * shares[policy] for _, shares in series])
        bar_ax.barh(y, values, left=left, height=0.58,
                    color=POLICY_COLOURS[policy], label=policy)
        for row_y, start, value in zip(y, left, values):
            if value >= 5.0:
                bar_ax.text(start + value / 2, row_y, f"{value:.1f}%", ha="center",
                            va="center", fontsize=8.5, fontweight="bold",
                            color="white" if policy != "R" else INK)
            elif value >= 0.1:
                bar_ax.text(start + value + 0.7, row_y, f"{value:.1f}%", ha="left",
                            va="center", fontsize=7.5, fontweight="bold",
                            color=POLICY_COLOURS[policy])
        left += values
    bar_ax.set_yticks(y, [label for label, _ in series])
    bar_ax.invert_yaxis()
    bar_ax.set_xlim(0, 105)
    bar_ax.set_xlabel("Aircraft-time share (%)")
    bar_ax.set_title("a  Final policy composition", loc="left", color=INK, pad=10)
    bar_ax.grid(axis="x", color="#EDF1F4", linewidth=0.7)
    bar_ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3,
                  fontsize=8.5)

    sinr_ax = fig.add_axes([0.52, 0.49, 0.42, 0.23])
    sinr_ax.fill_between(q, sinr_p10, sinr_p90, color="#CFE2EF", alpha=0.75,
                         label="10th–90th percentile")
    sinr_ax.plot(q, sinr_median, color=BLUE, linewidth=2.1, label="median SINR")
    sinr_ax.axhline(-2.0, color=RED, linewidth=1.4, linestyle="--",
                    label="Θ = −2 dB")
    sinr_ax.set_xlim(0, 50)
    sinr_ax.set_ylim(-5, 22)
    sinr_ax.set_ylabel("SINR (dB)")
    sinr_ax.set_title("b  Base stations remain available, but the SINR tail crosses Θ",
                      loc="left", color=INK, pad=10)
    sinr_ax.grid(color="#EDF1F4", linewidth=0.6)
    sinr_ax.legend(loc="upper right", fontsize=7.5, ncol=3)
    for index, (start, end, site) in enumerate(serving_segments(profile)):
        sinr_ax.plot([start, end], [20.2, 20.2], color="#7895AA", linewidth=2.2)
        sinr_ax.text((start + end) / 2, 18.3 - 1.4 * (index % 2), site,
                     ha="center", va="center", fontsize=6.5, color=GREY)

    exp_ax = fig.add_axes([0.52, 0.20, 0.42, 0.21], sharex=sinr_ax)
    exp_ax.fill_between(q, exp_p10, exp_p90, color="#F4D78F", alpha=0.55,
                        label="exposure 10th–90th percentile")
    exp_ax.plot(q, exp_median, color="#C28712", linewidth=2.1,
                label="median rolling group exposure")
    exp_ax.plot(q, current_bad, color=BLUE, linewidth=1.5, linestyle=":",
                label="current samples below Θ")
    exp_ax.axhline(5.0, color=POLICY_COLOURS["C"], linewidth=1.3, linestyle="--",
                   label="C/R gate = 5%")
    exp_ax.axhline(10.0, color=POLICY_COLOURS["F"], linewidth=1.3, linestyle="--",
                   label="R/F gate = 10%")
    exp_ax.axvspan(30, 40, color=POLICY_COLOURS["R"], alpha=0.08)
    exp_ax.set_xlim(0, 50)
    exp_ax.set_ylim(0, max(15, float(np.max(exp_p90)) + 1))
    exp_ax.set_xlabel("Longitudinal distance along corridor, q (km)")
    exp_ax.set_ylabel("Share / exposure (%)")
    exp_ax.set_title("c  The 90 s group history keeps exposure above the 5% C/R gate",
                     loc="left", color=INK, pad=10)
    exp_ax.grid(color="#EDF1F4", linewidth=0.6)
    exp_ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.26), ncol=3,
                  fontsize=7.2)

    takeaway = fig.add_axes([0.065, 0.045, 0.875, 0.065])
    takeaway.set_facecolor(PALE_BLUE)
    takeaway.set_xticks([])
    takeaway.set_yticks([])
    for spine in takeaway.spines.values():
        spine.set_visible(False)
    single_r = 100 * series[0][1]["R"]
    multi_r = 100 * series[1][1]["R"]
    takeaway.text(
        0.015, 0.5,
        f"Three-lane R is {multi_r:.1f}% versus {single_r:.1f}% in the final single-lane run. "
        "The long yellow band is an exposure-memory effect near the 5% gate, not evidence "
        "that the corridor has no serving base station.",
        color=BLUE, fontsize=10.4, fontweight="bold", va="center")

    source = []
    for label, shares in series:
        for policy in "CRF":
            source.append({
                "record_type": "policy_share",
                "series": label.replace("\n", " "),
                "q_km": "",
                "metric": f"policy_{policy}_share_pct",
                "value": 100.0 * shares[policy],
                "dominant_serving_bs": "",
            })
    for row in profile:
        for metric in ("sinr_p10_db", "sinr_median_db", "sinr_p90_db",
                       "current_below_threshold_pct", "exposure_p10_pct",
                       "exposure_median_pct", "exposure_p90_pct"):
            source.append({
                "record_type": "corridor_profile",
                "series": "three_lane_overall",
                "q_km": row["q_km"],
                "metric": metric,
                "value": row[metric],
                "dominant_serving_bs": row["dominant_serving_bs"],
            })
    return export(fig, OUTPUT / "04-single-vs-three-lane-policy-diagnosis"), source


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    style()
    config = read_json(RUN / "resolved_config.json")
    summary = read_json(CASE / "summary.json")
    trace = read_json(CASE / "trace.json.gz")
    events = read_json(CASE / "events.json")
    observations = read_json(CASE / "observations.json.gz")
    single_trace = read_json(SINGLE_RUN / "trace.json.gz")
    ids, _, nominal_lane = flight_order(config)
    grouped = group_trace(trace)
    moves = pair_moves(events, grouped)
    origin = np.asarray(trace[0]["xyz"][:2], dtype=float)
    stations = load_stations(origin)

    if len(ids) != 93 or summary["completed_requests"] != 93:
        raise RuntimeError("R0055 is not the expected completed 93-request run")
    if len(moves) != 12 or any("completion_s" not in move for move in moves):
        raise RuntimeError("R0055 does not contain 12 completed maneuvers")

    outputs = []
    outputs_1, source_1 = figure_corridor(trace, grouped, moves, summary, config, stations)
    outputs.extend(outputs_1)
    outputs_2, source_2 = figure_policy_heatmap(ids, grouped, nominal_lane, moves, summary)
    outputs.extend(outputs_2)
    outputs_3, source_3 = figure_move_results(moves, summary, config)
    outputs.extend(outputs_3)
    outputs_4, source_4 = figure_policy_comparison(single_trace, trace, observations,
                                                    nominal_lane, summary)
    outputs.extend(outputs_4)

    source_paths = [
        OUTPUT / "01-corridor-trajectory-and-lane-changes-source.csv",
        OUTPUT / "02-policy-along-corridor-source.csv",
        OUTPUT / "03-lane-change-timing-benefit-safety-source.csv",
        OUTPUT / "04-single-vs-three-lane-policy-diagnosis-source.csv",
    ]
    write_csv(source_paths[0], source_1)
    write_csv(source_paths[1], source_2)
    write_csv(source_paths[2], source_3)
    write_csv(source_paths[3], source_4)

    shares = summary["policy_and_longitudinal_metrics"]["policy_shares"]
    caption = (
        "# R0055 figure captions\n\n"
        "## Figure 1 — Corridor trajectory and lane changes\n\n"
        "All 93 scheduled requests enter and exit the same-altitude three-lane Bay Area "
        "corridor. The geographic panel shows the retained base stations and the dominant "
        "realized C/R/F state along each lane; numbered points identify the 12 accepted "
        "maneuver starts. The route-relative panel shows all realized trajectories and makes "
        "the lane transitions visible. All 12 maneuvers complete, the minimum sampled "
        "horizontal separation is 777.2 m, and no sampled NMAC occurs.\n\n"
        "## Figure 2 — Policy along the corridor\n\n"
        "Realized policy for all 93 completed flights, grouped by assigned entry lane and "
        "binned at 0.5 km along the corridor. The fixed calibrated configuration produces "
        f"{100*shares['C']:.1f}% C, {100*shares['R']:.1f}% R, and "
        f"{100*shares['F']:.1f}% F by aircraft time. Numbered circles locate the accepted "
        "lane-change starts.\n\n"
        "## Figure 3 — Lane-change timing, predicted benefit and safety\n\n"
        "Start and completion times for the 12 accepted maneuvers, with predicted ego and "
        "affected-group reductions in weighted policy cost over the common forecast horizon. "
        "Every accepted maneuver has zero predicted worsening among affected aircraft and "
        "completes in the realized simulation. The 777.2 m minimum sampled separation is "
        "5.1 times the configured 152.4 m horizontal threshold. These benefit values are "
        "controller forecasts, not a causal comparison against a separately simulated "
        "no-lane-change run.\n"
        "\n## Figure 4 — Single-lane versus three-lane policy diagnosis\n\n"
        "The final 90 s single-lane run produces 63.2% C, 36.0% R and 0.8% F; "
        "the three-lane run produces 64.5% C, 34.5% R and 1.0% F. Thus R does not "
        "increase overall. The three entry lanes are asymmetric: the −300 m lane is "
        "48.8% C/48.3% R/2.9% F, the centre lane is 67.6%/32.3%/0.2%, and the +300 m "
        "lane is 77.2%/22.8%/0.0%. In the 30–40 km region the median instantaneous "
        "SINR remains 0.5 dB, but 5.9% of samples fall below −2 dB and the median 90 s "
        "group exposure is 6.4%, above the 5% C/R gate. The dominant serving site "
        "continues to progress from BS12 to BS13; the R band is therefore caused by "
        "SINR-tail events retained by group/window exposure rather than an absence of "
        "base-station service.\n"
    )
    (OUTPUT / "captions.md").write_text(caption, encoding="utf-8")

    input_paths = [
        RUN / "resolved_config.json",
        CASE / "summary.json",
        CASE / "trace.json.gz",
        CASE / "events.json",
        CASE / "observations.json.gz",
        RUN / "validation.json",
        SINGLE_RUN / "trace.json.gz",
    ]
    manifest = {
        "run_id": "R0055",
        "purpose": "external-facing final-result visualisation for the fixed 90 s configuration",
        "statistics": "one deterministic run; descriptive values; no uncertainty interval",
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
        "outputs": {path.name: sha256(path) for path in [*outputs, *source_paths]},
    }
    (OUTPUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                           encoding="utf-8")
    print("\n".join(str(path) for path in outputs if path.suffix == ".png"))


if __name__ == "__main__":
    main()
