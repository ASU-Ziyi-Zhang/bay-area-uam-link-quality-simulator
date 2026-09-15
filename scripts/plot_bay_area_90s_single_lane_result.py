"""Create the external-facing R0048 single-lane Bay Area result figure."""
from __future__ import annotations

import csv
import gzip
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "research/dynamic-transitions/runs/R0048"
OUTPUT = ROOT / "research/dynamic-transitions/figures/R0048"

BLUE = "#204F7A"
LIGHT_BLUE = "#2C86C5"
RED = "#C53A2E"
INK = "#243447"
GREY = "#5E7187"
LIGHT_GREY = "#E4E9EE"
PALE_BLUE = "#E8F3F5"
POLICY_COLOURS = {"C": "#168A78", "R": "#E6A625", "F": "#C84A40"}
POLICY_CODE = {"C": 0, "R": 1, "F": 2}


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


def aircraft_number(aircraft_id: str) -> int:
    return int(aircraft_id.rsplit("-", 1)[-1])


def corridor_policy_matrix(trace: list[dict], bin_width_m: float = 500.0):
    aircraft_ids = sorted({row["aircraft_id"] for row in trace}, key=aircraft_number)
    maximum_q_m = max(float(row["q_m"]) for row in trace)
    edges = np.arange(0.0, np.ceil(maximum_q_m / bin_width_m) * bin_width_m + bin_width_m,
                      bin_width_m)
    matrix = np.full((len(aircraft_ids), len(edges) - 1), np.nan)
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in trace:
        grouped[row["aircraft_id"]].append(row)

    source_rows = []
    for row_index, aircraft_id in enumerate(aircraft_ids):
        counts = np.zeros((len(edges) - 1, 3), dtype=int)
        for row in grouped[aircraft_id]:
            bin_index = min(int(float(row["q_m"]) // bin_width_m), len(edges) - 2)
            counts[bin_index, POLICY_CODE[row["policy"]]] += 1
        available = counts.sum(axis=1) > 0
        matrix[row_index, available] = np.argmax(counts[available], axis=1)
        # The trace is dense and monotone, but fill a possible empty edge bin with
        # the nearest observed category so the visual represents the full mission.
        present = np.flatnonzero(~np.isnan(matrix[row_index]))
        missing = np.flatnonzero(np.isnan(matrix[row_index]))
        for column in missing:
            nearest = present[np.argmin(abs(present - column))]
            matrix[row_index, column] = matrix[row_index, nearest]
        for column, code in enumerate(matrix[row_index].astype(int)):
            source_rows.append({
                "record_type": "corridor_policy",
                "aircraft_id": aircraft_id,
                "distance_bin_start_km": edges[column] / 1000.0,
                "distance_bin_end_km": edges[column + 1] / 1000.0,
                "policy": "CRF"[code],
                "metric": "",
                "value": "",
                "unit": "",
            })
    return aircraft_ids, edges, matrix, source_rows


def add_card(fig, x: float, y: float, width: float, height: float,
             value: str, label: str, value_colour: str = BLUE) -> None:
    ax = fig.add_axes([x, y, width, height])
    ax.set_facecolor("#F5F8FA")
    for spine in ax.spines.values():
        spine.set_edgecolor("#D8E1E8")
        spine.set_linewidth(0.8)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.05, 0.62, value, transform=ax.transAxes, color=value_colour,
            fontsize=19, fontweight="bold", va="center")
    ax.text(0.05, 0.24, label, transform=ax.transAxes, color=GREY,
            fontsize=9.2, va="center")


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    summary = read_json(RUN / "summary.json")
    trace = read_json(RUN / "trace.json.gz")
    metrics = summary["metrics"]
    aircraft_ids, edges, matrix, source_rows = corridor_policy_matrix(trace)

    delays = list(summary["entry_delays_s"].values())
    shares = metrics["policy_shares"]
    metric_rows = [
        ("completed_requests", summary["completed_aircraft"], "requests"),
        ("scheduled_requests", summary["scheduled_aircraft"], "requests"),
        ("sampled_nmac", metrics["nmac_sampled"], "events"),
        ("minimum_sampled_horizontal_separation", metrics["minimum_sampled_horizontal_separation_m"], "m"),
        ("mean_entry_delay", sum(delays) / len(delays), "s"),
        ("operational_policy_switches_per_flight", metrics["mean_switches_per_flight"], "per flight"),
        ("gap_error_improved_fraction", metrics["gap_error_improved_fraction"], "fraction"),
        ("timing_compatible_fraction", metrics["timing_compatible_fraction"], "fraction"),
        ("fully_settled_fraction", metrics["completion_before_next_change_fraction"], "fraction"),
        ("speed_direction_reversals_per_flight", metrics["mean_speed_direction_reversals_per_flight"], "per flight"),
    ]
    for policy in "CRF":
        metric_rows.append((f"policy_{policy}_share", shares[policy], "fraction"))
    source_rows.extend({
        "record_type": "metric", "aircraft_id": "", "distance_bin_start_km": "",
        "distance_bin_end_km": "", "policy": "", "metric": name,
        "value": value, "unit": unit,
    } for name, value, unit in metric_rows)

    source_path = OUTPUT / "02-single-lane-final-result-source.csv"
    with source_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(source_rows[0]))
        writer.writeheader()
        writer.writerows(source_rows)

    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
    })
    fig = plt.figure(figsize=(16, 9), facecolor="white")

    # Slide-style heading matching the established meeting deck.
    fig.text(0.045, 0.938, "RESULT  |  CALIBRATED BAY AREA BASELINE",
             color=LIGHT_BLUE, fontsize=12, fontweight="bold")
    fig.text(0.045, 0.876,
             "The Single-Lane Corridor Completes the Full Demand Stream",
             color=BLUE, fontsize=25, fontweight="bold")
    fig.add_artist(plt.Line2D([0.045, 0.17], [0.83, 0.83], transform=fig.transFigure,
                              color=RED, linewidth=3))
    fig.text(0.045, 0.795,
             "Bay Area airport-to-airport corridor  ·  one lane at 300 m  ·  "
             "32 s requested-arrival interval  ·  W = 90 s  ·  longitudinal control only",
             color=GREY, fontsize=11)

    # Hero panel: each flight's realized policy over corridor distance.
    ax = fig.add_axes([0.06, 0.20, 0.58, 0.54])
    cmap = ListedColormap([POLICY_COLOURS[p] for p in "CRF"])
    ax.imshow(matrix, aspect="auto", origin="lower", interpolation="nearest",
              cmap=cmap, vmin=-0.5, vmax=2.5,
              extent=[edges[0] / 1000.0, edges[-1] / 1000.0, 0.5, len(aircraft_ids) + 0.5])
    ax.set_xlabel("Longitudinal distance along corridor, q (km)", labelpad=8)
    ax.set_ylabel("Sequential request", labelpad=8)
    ax.set_yticks([1, 20, 40, 60, 80, 93])
    ax.set_title("REALIZED POLICY ALONG THE CORRIDOR", loc="left", color=GREY, pad=12,
                 fontsize=11)
    for spine in ax.spines.values():
        spine.set_color("#AAB7C3")
        spine.set_linewidth(0.8)
    ax.tick_params(length=3, color="#AAB7C3")
    legend_x = 0.385
    for index, policy in enumerate("CRF"):
        fig.add_artist(plt.Line2D([legend_x + index * 0.065, legend_x + 0.025 + index * 0.065],
                                  [0.752, 0.752], transform=fig.transFigure,
                                  color=POLICY_COLOURS[policy], linewidth=4))
        fig.text(legend_x + 0.03 + index * 0.065, 0.744, policy,
                 color=INK, fontsize=10, fontweight="bold")

    # Absolute policy composition, without comparison to an earlier setting.
    share_ax = fig.add_axes([0.69, 0.64, 0.265, 0.10])
    share_ax.set_title("OVERALL POLICY SHARE", loc="left", color=GREY, pad=10,
                       fontsize=11, fontweight="bold")
    left = 0.0
    for policy in "CRF":
        width = float(shares[policy])
        share_ax.barh([0], [width], left=left, height=0.48,
                      color=POLICY_COLOURS[policy])
        if width > 0.06:
            share_ax.text(left + width / 2, 0, f"{policy}  {100*width:.1f}%",
                          ha="center", va="center", color="white" if policy != "R" else INK,
                          fontsize=10.5, fontweight="bold")
        left += width
    share_ax.annotate("F  0.8%", xy=(1 - shares["F"] / 2, 0.24), xytext=(0.82, 0.72),
                      textcoords="axes fraction", ha="left", va="center",
                      color=POLICY_COLOURS["F"], fontsize=9.5, fontweight="bold",
                      arrowprops=dict(arrowstyle="-", color=POLICY_COLOURS["F"], lw=1.1))
    share_ax.set_xlim(0, 1)
    share_ax.set_ylim(-0.55, 0.8)
    share_ax.axis("off")

    add_card(fig, 0.69, 0.485, 0.125, 0.105, "93 / 93", "requests completed")
    add_card(fig, 0.83, 0.485, 0.125, 0.105, "0", "sampled NMAC")
    add_card(fig, 0.69, 0.355, 0.125, 0.105, "1.22 km", "minimum separation")
    add_card(fig, 0.83, 0.355, 0.125, 0.105, "28.1 s", "mean entry delay")

    response_ax = fig.add_axes([0.69, 0.20, 0.265, 0.105])
    response_ax.set_title("LONGITUDINAL RESPONSE", loc="left", color=GREY, pad=10,
                          fontsize=11, fontweight="bold")
    labels = ["Gap error improved", "Timing compatible", "Fully settled"]
    values = [metrics["gap_error_improved_fraction"], metrics["timing_compatible_fraction"],
              metrics["completion_before_next_change_fraction"]]
    y = np.arange(3)[::-1]
    response_ax.barh(y, [1, 1, 1], color="#E7EDF1", height=0.46)
    response_ax.barh(y, values, color=[POLICY_COLOURS["C"], "#4B82A8", "#91A8B8"],
                     height=0.46)
    for ypos, label, value in zip(y, labels, values):
        response_ax.text(0.01, ypos, label, va="center", ha="left", color=INK,
                         fontsize=8.8, fontweight="bold")
        response_ax.text(1.02, ypos, f"{100*value:.1f}%", va="center", ha="left",
                         color=BLUE, fontsize=9.2, fontweight="bold")
    response_ax.set_xlim(0, 1.16)
    response_ax.set_ylim(-0.6, 2.6)
    response_ax.axis("off")

    takeaway = fig.add_axes([0.06, 0.055, 0.895, 0.085])
    takeaway.set_facecolor(PALE_BLUE)
    for spine in takeaway.spines.values():
        spine.set_visible(False)
    takeaway.set_xticks([])
    takeaway.set_yticks([])
    takeaway.text(0.018, 0.62,
                  "The calibrated policy supports continuous single-lane operation with only 0.8% fallback.",
                  color=BLUE, fontsize=12.2, fontweight="bold", va="center")
    takeaway.text(0.018, 0.25,
                  "Spacing error improves in 87.1% of actions, but only 6.0% fully settle; "
                  "speed-control damping remains a monitored item for the lane-change stage.",
                  color=INK, fontsize=10.2, va="center")

    stem = OUTPUT / "02-single-lane-final-result"
    fig.savefig(stem.with_suffix(".png"), dpi=200, facecolor="white")
    fig.savefig(stem.with_suffix(".svg"), facecolor="white")
    fig.savefig(stem.with_suffix(".pdf"), facecolor="white")
    fig.savefig(stem.with_suffix(".tiff"), dpi=600, facecolor="white")
    plt.close(fig)

    outputs = [stem.with_suffix(ext) for ext in (".png", ".svg", ".pdf", ".tiff")]
    manifest = {
        "run_id": "R0048",
        "purpose": "external-facing absolute result; no parameter-selection comparison",
        "statistics": "one deterministic run; descriptive fractions; no uncertainty interval",
        "sources": {
            str((RUN / "summary.json").relative_to(ROOT)): sha256(RUN / "summary.json"),
            str((RUN / "trace.json.gz").relative_to(ROOT)): sha256(RUN / "trace.json.gz"),
        },
        "outputs": {path.name: sha256(path) for path in [source_path, *outputs]},
    }
    manifest_path = OUTPUT / "02-single-lane-final-result-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
