"""Plot the R0040 32-second-demand Bay Area longitudinal-control result."""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
COLOUR = {"C": "#0072B2", "R": "#E69F00", "F": "#CC79A7"}
INK, GREY, GRID = "#202124", "#6B7280", "#E5E7EB"


def read_json(path: Path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return json.load(stream)
    return json.loads(path.read_text(encoding="utf-8"))


def read_dashboard(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    return json.loads(text[text.index("=") + 1:].strip().rstrip(";"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def style() -> None:
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 7.2,
        "axes.labelsize": 7.4,
        "axes.titlesize": 8.6,
        "axes.titleweight": "bold",
        "xtick.labelsize": 6.8,
        "ytick.labelsize": 6.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.65,
        "legend.frameon": False,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path,
                        default=ROOT / "research/dynamic-transitions/runs/R0040")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "research/dynamic-transitions/figures/R0040")
    args = parser.parse_args()
    run, output = args.run.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    trace_path = run / "trace.json.gz"
    summary_path = run / "summary.json"
    dashboard_path = ROOT / "dashboard/data/airport_to_airport_traffic.js"
    trace = read_json(trace_path)
    summary = read_json(summary_path)
    dashboard = read_dashboard(dashboard_path)
    metrics = summary["metrics"]
    common = metrics["common_horizon_comparison"]
    baseline = dashboard["summary"]["policy"]["shares"]
    controlled = common["policy_update_shares"]

    sampled = [row for row in trace
               if abs(float(row["t_s"]) / 5.0 - round(float(row["t_s"]) / 5.0)) < 1e-8]
    by_aircraft: dict[str, list[dict]] = {}
    for row in sampled:
        by_aircraft.setdefault(row["aircraft_id"], []).append(row)

    source_rows = []
    for aircraft_id, rows in sorted(by_aircraft.items()):
        rows.sort(key=lambda item: float(item["t_s"]))
        for row in rows:
            source_rows.append({
                "record_type": "trajectory",
                "aircraft_id": aircraft_id,
                "t_s": row["t_s"],
                "q_km": float(row["q_m"]) / 1000.0,
                "policy": row["policy"],
            })
    for dataset, shares in (("Fixed-speed GitHub", baseline),
                            ("Longitudinal control", controlled)):
        for policy in "CRF":
            source_rows.append({"record_type": "policy_share", "dataset": dataset,
                                "policy": policy, "value": shares[policy]})
    metric_rows = [
        ("completed_aircraft", summary["completed_aircraft"]),
        ("scheduled_aircraft", summary["scheduled_aircraft"]),
        ("mean_switches_per_flight", metrics["mean_switches_per_flight"]),
        ("settled_task_count", metrics["completed_before_next_change_count"]),
        ("longitudinal_task_count", metrics["longitudinal_task_count"]),
        ("minimum_horizontal_separation_m", metrics["minimum_sampled_horizontal_separation_m"]),
        ("sampled_nmac", int(metrics["nmac_sampled"])),
        ("full_mission_f_share", metrics["policy_shares"]["F"]),
        ("delayed_entries", sum(value > 0 for value in summary["entry_delays_s"].values())),
        ("maximum_entry_delay_s", max(summary["entry_delays_s"].values())),
    ]
    for name, value in metric_rows:
        source_rows.append({"record_type": "metric", "metric": name, "value": value})
    source_path = output / "source-data.csv"
    fields = ["record_type", "aircraft_id", "t_s", "q_km", "policy",
              "dataset", "metric", "value"]
    with source_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(source_rows)

    style()
    fig = plt.figure(figsize=(183.0 / 25.4, 4.9))
    grid = GridSpec(2, 2, figure=fig, width_ratios=[1.55, 1.0],
                    height_ratios=[1.15, 0.85], wspace=0.32, hspace=0.40)
    ax_space = fig.add_subplot(grid[:, 0])
    ax_share = fig.add_subplot(grid[0, 1])
    ax_metric = fig.add_subplot(grid[1, 1])

    fig.suptitle("32-s-demand longitudinal control: C remains dominant and all flights complete",
                 fontsize=10.5, fontweight="bold", x=0.075, ha="left", y=0.975)
    fig.text(0.075, 0.94,
             "Real Bay Area route · 93 scheduled requests in this finite run · one lane · no lateral or vertical changes",
             fontsize=7.2, color=GREY, ha="left")

    collections = {policy: [] for policy in "CRF"}
    for rows in by_aircraft.values():
        rows.sort(key=lambda item: float(item["t_s"]))
        for left, right in zip(rows, rows[1:]):
            collections[left["policy"]].append([
                (float(left["t_s"]) / 60.0, float(left["q_m"]) / 1000.0),
                (float(right["t_s"]) / 60.0, float(right["q_m"]) / 1000.0),
            ])
    for policy in "CRF":
        ax_space.add_collection(LineCollection(collections[policy], colors=COLOUR[policy],
                                                linewidths=0.48, alpha=0.72,
                                                label=policy, rasterized=False))
    ax_space.autoscale()
    ax_space.set_xlim(0, summary["end_s"] / 60.0)
    ax_space.set_ylim(0, 50)
    ax_space.set_xlabel("mission time (min)")
    ax_space.set_ylabel("distance along route (km)")
    ax_space.set_title("a  Space–time trajectories under the 32 s arrival stream", loc="left", pad=7)
    ax_space.grid(color=GRID, linewidth=0.45)
    ax_space.legend(title="Policy", ncol=3, loc="lower right",
                    handlelength=1.4, columnspacing=0.8)

    datasets = ["Fixed-speed\nGitHub", "Longitudinal\ncontrol"]
    values = [baseline, controlled]
    bottoms = np.zeros(2)
    for policy in "CRF":
        heights = [row[policy] for row in values]
        ax_share.bar([0, 1], heights, width=0.62, bottom=bottoms,
                     color=COLOUR[policy])
        for x, height, bottom in zip([0, 1], heights, bottoms):
            if height >= 0.055:
                ax_share.text(x, bottom + height / 2, f"{100 * height:.1f}%",
                              ha="center", va="center", fontsize=6.7,
                              fontweight="bold", color="white" if policy != "R" else INK)
        bottoms += np.asarray(heights)
    ax_share.annotate("F 2.8%", xy=(0, 1 - baseline["F"] / 2), xytext=(-0.55, 1.08),
                      arrowprops=dict(arrowstyle="-", color=COLOUR["F"], lw=0.7),
                      color=COLOUR["F"], fontsize=6.6, fontweight="bold")
    ax_share.set_xticks([0, 1], datasets)
    ax_share.set_ylim(0, 1.13)
    ax_share.set_yticks([0, .25, .5, .75, 1], ["0", "25", "50", "75", "100%"])
    ax_share.set_ylabel("policy observations")
    ax_share.set_title("b  Matched schedule and horizon", loc="left", pad=7)
    ax_share.grid(axis="y", color=GRID, linewidth=0.45)

    ax_metric.axis("off")
    ax_metric.set_title("c  Longitudinal-control outcome", loc="left", pad=2)
    cards = [
        ("93 / 93", "missions completed"),
        (f'{sum(value > 0 for value in summary["entry_delays_s"].values())} / 93',
         f'entries delayed; max {max(summary["entry_delays_s"].values()):.1f} s'),
        (f'{metrics["mean_switches_per_flight"]:.2f}', "switches per flight"),
        (f'{metrics["completed_before_next_change_count"]} / '
         f'{metrics["longitudinal_task_count"]}', "tasks fully settled"),
    ]
    for index, (value, label) in enumerate(cards):
        col, row = index % 2, 1 - index // 2
        x, y = 0.02 + col * 0.50, 0.08 + row * 0.47
        ax_metric.text(x, y + 0.18, value, transform=ax_metric.transAxes,
                       fontsize=11.0, fontweight="bold", color=INK)
        ax_metric.text(x, y + 0.04, label, transform=ax_metric.transAxes,
                       fontsize=6.6, color=GREY)
    fig.text(0.075, 0.018,
             "Common-horizon F: 2.83% fixed speed → 6.77% with longitudinal control.  "
             "Full run: F = 6.11%; no sampled NMAC; minimum separation = 1,218 m.",
             fontsize=6.7, color=INK, ha="left")
    fig.subplots_adjust(left=0.075, right=0.985, top=0.89, bottom=0.115)

    stem = output / "01-longitudinal-93-uam-result"
    fig.savefig(stem.with_suffix(".svg"))
    fig.savefig(stem.with_suffix(".pdf"))
    fig.savefig(stem.with_suffix(".png"), dpi=300)
    plt.close(fig)

    manifest = {
        "run_id": summary["run_id"],
        "figure": stem.name,
        "source_files": {
            str(trace_path.relative_to(ROOT)): sha256(trace_path),
            str(summary_path.relative_to(ROOT)): sha256(summary_path),
            str(dashboard_path.relative_to(ROOT)): sha256(dashboard_path),
        },
        "outputs": {path.name: sha256(path) for path in sorted(output.iterdir())
                    if path.is_file() and path.name != "manifest.json"},
        "statistics": "one deterministic run; descriptive fractions; no uncertainty interval",
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
