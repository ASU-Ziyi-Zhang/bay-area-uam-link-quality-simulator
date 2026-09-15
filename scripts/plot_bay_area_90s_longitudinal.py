"""Plot the matched 30 s versus 90 s Bay Area longitudinal comparison."""
from __future__ import annotations

import csv
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "research/dynamic-transitions/runs"
OUTPUT = ROOT / "research/dynamic-transitions/figures/R0048"
POLICY_COLOUR = {"C": "#0072B2", "R": "#E69F00", "F": "#CC79A7"}
WINDOW_COLOUR = {"30 s": "#9CA3AF", "90 s": "#2F6B8A"}
INK, GREY, GRID = "#202124", "#6B7280", "#E5E7EB"


def read_json(path: Path):
    with (gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz"
          else path.open(encoding="utf-8")) as stream:
        return json.load(stream)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def derived(run_id: str) -> dict:
    run = RUNS / run_id
    summary = read_json(run / "summary.json")
    trace = read_json(run / "trace.json.gz")
    observations = read_json(run / "observations.json.gz")
    metrics = summary["metrics"]
    by_aircraft = {}
    for row in trace:
        by_aircraft.setdefault(row["aircraft_id"], []).append(row)
    reversals = []
    for rows in by_aircraft.values():
        directions = []
        for left, right in zip(rows, rows[1:]):
            delta = float(right["v_mps"]) - float(left["v_mps"])
            if abs(delta) > 1e-7:
                directions.append(1 if delta > 0 else -1)
        reversals.append(sum(a != b for a, b in zip(directions, directions[1:])))
    by_observed = {}
    for row in observations:
        by_observed.setdefault(row["aircraft_id"], []).append(row)
    raw_switches = []
    for rows in by_observed.values():
        rows.sort(key=lambda row: float(row["t_s"]))
        values = [row.get("raw_policy", row["policy"]) for row in rows]
        raw_switches.append(sum(a != b for a, b in zip(values, values[1:])))
    delays = list(summary["entry_delays_s"].values())
    return {
        "run_id": run_id,
        "summary": summary,
        "policy": metrics["common_horizon_comparison"]["policy_update_shares"],
        "operational_switches": metrics["mean_switches_per_flight"],
        "raw_switches": sum(raw_switches) / len(raw_switches),
        "speed_reversals": sum(reversals) / len(reversals),
        "timing_compatible": metrics["timing_compatible_fraction"],
        "settled": metrics["completion_before_next_change_fraction"],
        "mean_entry_delay_s": sum(delays) / len(delays),
        "maximum_entry_delay_s": max(delays),
        "minimum_separation_m": metrics["minimum_sampled_horizontal_separation_m"],
    }


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    values = {"30 s": derived("R0040"), "90 s": derived("R0048")}
    rows = []
    for label, item in values.items():
        for policy in "CRF":
            rows.append({"window": label, "metric": f"policy_{policy}_share",
                         "value": item["policy"][policy]})
        for metric in ("operational_switches", "raw_switches", "speed_reversals",
                       "timing_compatible", "settled", "mean_entry_delay_s",
                       "maximum_entry_delay_s", "minimum_separation_m"):
            rows.append({"window": label, "metric": metric, "value": item[metric]})
    source = OUTPUT / "source-data.csv"
    with source.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["window", "metric", "value"])
        writer.writeheader(); writer.writerows(rows)

    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 7, "axes.labelsize": 7.3, "axes.titlesize": 8.2,
        "axes.titleweight": "bold", "xtick.labelsize": 6.6,
        "ytick.labelsize": 6.6, "axes.spines.top": False,
        "axes.spines.right": False, "axes.linewidth": 0.65,
        "legend.frameon": False, "svg.fonttype": "none", "pdf.fonttype": 42,
    })
    fig = plt.figure(figsize=(183 / 25.4, 4.25))
    grid = fig.add_gridspec(2, 2, width_ratios=[1.1, 1.0], hspace=0.56, wspace=0.35)
    ax_policy = fig.add_subplot(grid[:, 0])
    ax_switch = fig.add_subplot(grid[0, 1])
    ax_task = fig.add_subplot(grid[1, 1])
    fig.suptitle("A 90 s window stabilizes policy, but longitudinal control remains unsettled",
                 x=0.08, y=0.98, ha="left", fontsize=10.1, fontweight="bold")
    fig.text(0.08, 0.938,
             "Matched 32 s Bay Area demand stream · one lane at 300 m · no lateral or vertical changes",
             color=GREY, fontsize=7.0)

    labels = list(values)
    bottoms = np.zeros(2)
    for policy in "CRF":
        heights = [values[label]["policy"][policy] for label in labels]
        bars = ax_policy.bar(labels, heights, bottom=bottoms, width=0.58,
                             color=POLICY_COLOUR[policy], label=policy)
        for bar, height, bottom in zip(bars, heights, bottoms):
            if height > 0.055:
                ax_policy.text(bar.get_x() + bar.get_width()/2, bottom + height/2,
                               f"{100*height:.1f}%", ha="center", va="center",
                               color="white" if policy != "R" else INK,
                               fontsize=7.2, fontweight="bold")
        bottoms += heights
    ax_policy.annotate("F 0.8%", xy=(1, 0.996), xytext=(1.16, 1.055),
                       arrowprops=dict(arrowstyle="-", color=POLICY_COLOUR["F"], lw=0.8),
                       color=POLICY_COLOUR["F"], fontsize=7, fontweight="bold")
    ax_policy.set_ylim(0, 1.1)
    ax_policy.set_yticks([0, .25, .5, .75, 1], ["0", "25", "50", "75", "100%"])
    ax_policy.set_ylabel("policy-update observations")
    ax_policy.set_title("a  Matched-horizon policy composition", loc="left", pad=7)
    ax_policy.grid(axis="y", color=GRID, linewidth=0.45)
    ax_policy.legend(title="Policy", ncol=3, loc="upper left")

    x = np.arange(2); width = 0.34
    operational = [values[label]["operational_switches"] for label in labels]
    raw = [values[label]["raw_switches"] for label in labels]
    ax_switch.bar(x-width/2, operational, width, color="#2F6B8A", label="operational")
    ax_switch.bar(x+width/2, raw, width, color="#A9C5D3", label="raw")
    for xpos, height in zip(np.r_[x-width/2, x+width/2], operational+raw):
        ax_switch.text(xpos, height+0.5, f"{height:.1f}", ha="center", fontsize=6.4)
    ax_switch.set_xticks(x, labels); ax_switch.set_ylim(0, 27)
    ax_switch.set_ylabel("switches per flight")
    ax_switch.set_title("b  Policy switching decreases", loc="left", pad=7)
    ax_switch.grid(axis="y", color=GRID, linewidth=0.45)
    ax_switch.legend(ncol=2, loc="upper right", fontsize=6.2)

    timing = [100*values[label]["timing_compatible"] for label in labels]
    settled = [100*values[label]["settled"] for label in labels]
    ax_task.bar(x-width/2, timing, width, color="#5B8C85", label="timing-compatible")
    ax_task.bar(x+width/2, settled, width, color="#D4A72C", label="fully settled")
    for xpos, height in zip(np.r_[x-width/2, x+width/2], timing+settled):
        ax_task.text(xpos, height+1.1, f"{height:.1f}%", ha="center", fontsize=6.4)
    ax_task.set_xticks(x, labels); ax_task.set_ylim(0, 65)
    ax_task.set_ylabel("longitudinal tasks")
    ax_task.set_title("c  More dwell, but only 6.0% settle", loc="left", pad=7)
    ax_task.grid(axis="y", color=GRID, linewidth=0.45)
    ax_task.legend(ncol=2, loc="upper left", fontsize=6.0)

    fig.text(0.08, 0.028,
             "Control caveat: speed reversals rise 35.1 → 49.4/flight.  "
             "Mean entry delay falls 47.3 → 28.1 s.  Both runs: all requests complete; no sampled NMAC.",
             fontsize=6.7, color=INK)
    fig.subplots_adjust(left=0.08, right=0.985, top=0.88, bottom=0.14)
    stem = OUTPUT / "01-window-30s-vs-90s"
    fig.savefig(stem.with_suffix(".svg"))
    fig.savefig(stem.with_suffix(".pdf"))
    fig.savefig(stem.with_suffix(".png"), dpi=300)
    fig.savefig(stem.with_suffix(".tiff"), dpi=600)
    plt.close(fig)

    inputs = [RUNS/"R0040"/"summary.json", RUNS/"R0040"/"trace.json.gz",
              RUNS/"R0040"/"observations.json.gz", RUNS/"R0048"/"summary.json",
              RUNS/"R0048"/"trace.json.gz", RUNS/"R0048"/"observations.json.gz"]
    manifest = {
        "run_id": "R0048", "comparison": "R0040",
        "statistics": "two deterministic matched runs; descriptive fractions; no uncertainty interval",
        "sources": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
        "outputs": {path.name: sha256(path) for path in OUTPUT.iterdir()
                    if path.is_file() and path.name != "manifest.json"},
    }
    (OUTPUT/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
