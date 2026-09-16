"""Create the source-backed Dev3A longitudinal gate figure for R0039."""
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
from matplotlib.gridspec import GridSpec  # noqa: E402
import numpy as np  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
POLICY_COLOUR = {"C": "#0072B2", "R": "#E69F00", "F": "#CC79A7"}
INK = "#202124"
GREY = "#6B7280"
LIGHT_GREY = "#E5E7EB"
GREEN = "#009E73"
RED = "#D55E00"


def load_json(path: Path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return json.load(stream)
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def policy_segments(rows: list[dict]) -> list[dict]:
    segments: list[dict] = []
    by_aircraft: dict[str, list[dict]] = {}
    for row in rows:
        by_aircraft.setdefault(row["aircraft_id"], []).append(row)
    for aircraft_id, aircraft_rows in sorted(by_aircraft.items()):
        aircraft_rows.sort(key=lambda item: float(item["t_s"]))
        start = float(aircraft_rows[0]["t_s"])
        end = start + float(aircraft_rows[0]["dt_s"])
        policy = aircraft_rows[0]["policy"]
        for row in aircraft_rows[1:]:
            row_start = float(row["t_s"])
            row_end = row_start + float(row["dt_s"])
            if row["policy"] == policy and abs(row_start - end) <= 1e-8:
                end = row_end
                continue
            segments.append({"aircraft_id": aircraft_id, "start_s": start,
                             "end_s": end, "policy": policy})
            start, end, policy = row_start, row_end, row["policy"]
        segments.append({"aircraft_id": aircraft_id, "start_s": start,
                         "end_s": end, "policy": policy})
    return segments


def calibration_row(path: Path) -> dict:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    matches = [row for row in rows
               if float(row["dt_radio_s"]) == 2.0
               and int(row["group_size"]) == 5
               and float(row["window_s"]) == 30.0
               and int(row["persistence_k"]) == 3
               and float(row["policy_update_s"]) == 5.0
               and float(row["threshold_db"]) == -2.0]
    if len(matches) != 1:
        raise ValueError(f"expected one open-loop working-point row, found {len(matches)}")
    return matches[0]


def write_source_data(path: Path, segments: list[dict], hero: list[dict],
                      tasks: list[dict], shares: list[dict], metrics: dict) -> None:
    fields = [
        "record_type", "aircraft_id", "start_s", "end_s", "policy", "t_s",
        "actual_gap_m", "target_gap_m", "relative_speed_mps",
        "normalised_absolute_gap_error", "normalised_absolute_relative_speed",
        "transition", "dwell_s", "isolated_reference_s", "timing_compatible",
        "gap_error_improved", "dataset", "share", "metric", "value", "unit",
    ]
    rows: list[dict] = []
    rows.extend({"record_type": "policy_segment", **row} for row in segments)
    rows.extend({"record_type": "hero_response", **row} for row in hero)
    rows.extend({"record_type": "longitudinal_task", **row} for row in tasks)
    rows.extend({"record_type": "policy_share", **row} for row in shares)
    for metric, value, unit in [
        ("completed_aircraft", metrics["completed_aircraft"], "aircraft"),
        ("scheduled_aircraft", metrics["scheduled_aircraft"], "aircraft"),
        ("minimum_horizontal_separation", metrics["minimum_horizontal_separation"], "m"),
        ("mean_switches_per_flight", metrics["mean_switches_per_flight"], "switches/flight"),
        ("timing_compatible_fraction", metrics["timing_compatible_fraction"], "fraction"),
        ("gap_error_improved_fraction", metrics["gap_error_improved_fraction"], "fraction"),
        ("completion_fraction", metrics["completion_fraction"], "fraction"),
        ("maximum_lateral_offset", metrics["maximum_lateral_offset"], "m"),
        ("nmac_sampled", int(metrics["nmac_sampled"]), "boolean"),
    ]:
        rows.append({"record_type": "gate_metric", "metric": metric,
                     "value": value, "unit": unit})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def style() -> None:
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 7.4,
        "axes.labelsize": 7.4,
        "axes.titlesize": 8.4,
        "axes.titleweight": "bold",
        "legend.fontsize": 6.8,
        "xtick.labelsize": 6.8,
        "ytick.labelsize": 6.8,
        "text.color": INK,
        "axes.labelcolor": INK,
        "axes.edgecolor": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "axes.linewidth": 0.65,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path,
                        default=ROOT / "research/dynamic-transitions/runs/R0039")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "research/dynamic-transitions/figures/R0039")
    args = parser.parse_args()
    run, output = args.run.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    trace_path = run / "trace.json.gz"
    summary_path = run / "summary.json"
    calibration_path = (ROOT / "research/policy-calibration/deliverables/TEMA/figures/"
                         "source-data.csv")
    trace = load_json(trace_path)
    summary = load_json(summary_path)
    m = summary["metrics"]
    segments = policy_segments(trace)

    hero_aircraft, hero_start, hero_end = "UAM-04", 250.0, 565.0
    hero = []
    for row in trace:
        if (row["aircraft_id"] == hero_aircraft and hero_start <= float(row["t_s"]) < hero_end
                and row["leaders"] and row["leaders"][0][0] == "UAM-03"):
            gap = float(row["leaders"][0][1])
            relative = float(row["leaders"][0][2])
            target = float(row["target_gap_m"])
            hero.append({
                "aircraft_id": hero_aircraft,
                "t_s": float(row["t_s"]),
                "actual_gap_m": gap,
                "target_gap_m": target,
                "relative_speed_mps": relative,
                "normalised_absolute_gap_error": abs(gap - target) / 1.0,
                "normalised_absolute_relative_speed": abs(relative) / 0.1,
            })
    if not hero:
        raise ValueError("representative UAM-04 R-to-C response is absent")

    tasks = [{
        "aircraft_id": row["aircraft_id"],
        "transition": f'{row["from_policy"]}→{row["to_policy"]}',
        "start_s": row["start_s"],
        "end_s": row["next_change_s"],
        "dwell_s": row["dwell_s"],
        "isolated_reference_s": row["nominal_isolated_settling_s"],
        "timing_compatible": int(row["timing_compatible_with_isolated_reference"]),
        "gap_error_improved": int(row["gap_error_improved"]),
    } for row in m["longitudinal_tasks"]]

    open_loop = calibration_row(calibration_path)
    shares = []
    for policy, key in [("C", "c_share"), ("R", "r_share"), ("F", "f_share")]:
        shares.append({"dataset": "Open-loop calibration", "policy": policy,
                       "share": float(open_loop[key])})
        shares.append({"dataset": "Dev3A closed loop", "policy": policy,
                       "share": float(m["policy_shares"][policy])})

    gate_metrics = {
        "completed_aircraft": summary["completed_aircraft"],
        "scheduled_aircraft": summary["scheduled_aircraft"],
        "minimum_horizontal_separation": m["minimum_sampled_horizontal_separation_m"],
        "mean_switches_per_flight": m["mean_switches_per_flight"],
        "timing_compatible_fraction": m["timing_compatible_fraction"],
        "gap_error_improved_fraction": m["gap_error_improved_fraction"],
        "completion_fraction": m["completion_before_next_change_fraction"],
        "maximum_lateral_offset": m["maximum_absolute_lateral_offset_m"],
        "nmac_sampled": m["nmac_sampled"],
    }
    source_path = output / "source-data.csv"
    write_source_data(source_path, segments, hero, tasks, shares, gate_metrics)

    style()
    width_in = 183.0 / 25.4
    fig = plt.figure(figsize=(width_in, 7.75))
    grid = GridSpec(3, 2, figure=fig, height_ratios=[1.28, 1.0, 1.04],
                    hspace=0.66, wspace=0.42)
    ax_timeline = fig.add_subplot(grid[0, :])
    ax_share = fig.add_subplot(grid[1, 0])
    ax_dwell = fig.add_subplot(grid[1, 1])
    ax_gap = fig.add_subplot(grid[2, :])

    fig.suptitle("Dev3A: the calibrated policy runs safely, but longitudinal settling lags",
                 fontsize=11.2, fontweight="bold", x=0.08, ha="left", y=0.985)
    fig.text(0.08, 0.957,
             "Real Bay Area route · five aircraft · one lane at 300 m · Θ = −2.0 dB · "
             "radio 2 s · update 5 s · W = 30 s · k = 3",
             fontsize=7.4, color=GREY, ha="left")

    aircraft_ids = sorted({row["aircraft_id"] for row in trace})
    y_map = {identifier: len(aircraft_ids) - index - 1
             for index, identifier in enumerate(aircraft_ids)}
    for segment in segments:
        y = y_map[segment["aircraft_id"]]
        ax_timeline.broken_barh(
            [(segment["start_s"], segment["end_s"] - segment["start_s"])],
            (y - 0.34, 0.68), facecolors=POLICY_COLOUR[segment["policy"]],
            edgecolors="none")
    for policy in "CRF":
        ax_timeline.plot([], [], lw=5.5, color=POLICY_COLOUR[policy], label=policy)
    ax_timeline.set_yticks([y_map[row] for row in aircraft_ids], aircraft_ids)
    ax_timeline.set_xlim(0, max(float(row["t_s"]) + float(row["dt_s"]) for row in trace))
    ax_timeline.set_xlabel("mission time (s)")
    ax_timeline.set_title("a  Operational policy timeline: switching remains frequent after k = 3",
                          loc="left", pad=7)
    ax_timeline.grid(axis="x", color=LIGHT_GREY, lw=0.5)
    ax_timeline.legend(frameon=False, ncol=3, loc="upper right",
                       bbox_to_anchor=(1.0, 1.20), handlelength=1.3)

    datasets = ["Open-loop calibration", "Dev3A closed loop"]
    bottoms = np.zeros(2)
    for policy in "CRF":
        values = [next(row["share"] for row in shares
                       if row["dataset"] == dataset and row["policy"] == policy)
                  for dataset in datasets]
        ax_share.bar(np.arange(2), values, bottom=bottoms, width=0.64,
                     color=POLICY_COLOUR[policy], label=policy)
        for x, value, bottom in zip(np.arange(2), values, bottoms):
            if value >= 0.055:
                ax_share.text(x, bottom + value / 2, f"{100 * value:.1f}%",
                              ha="center", va="center", fontsize=6.6,
                              color="white" if policy != "R" else INK,
                              fontweight="bold")
        bottoms += np.asarray(values)
    ax_share.set_xticks(np.arange(2), ["Open-loop\ncalibration", "Dev3A\nclosed loop"])
    ax_share.set_ylim(0, 1)
    ax_share.set_ylabel("aircraft-time share")
    ax_share.set_yticks([0, .25, .5, .75, 1], ["0", "25", "50", "75", "100%"])
    ax_share.set_title("b  Closed-loop motion changes the policy mix", loc="left", pad=7)
    ax_share.spines[["top", "right"]].set_visible(False)

    for improved, marker, face, label in [
        (1, "o", GREEN, "gap error improved"),
        (0, "x", RED, "did not improve"),
    ]:
        selected = [row for row in tasks if row["gap_error_improved"] == improved]
        scatter_kwargs = ({"facecolor": face, "edgecolor": INK}
                          if marker == "o" else {"color": RED})
        ax_dwell.scatter([row["isolated_reference_s"] for row in selected],
                         [row["dwell_s"] for row in selected], s=24,
                         marker=marker, linewidth=0.65, alpha=0.78,
                         label=label, zorder=3, **scatter_kwargs)
    limit = 330
    ax_dwell.plot([0, limit], [0, limit], ls="--", lw=0.9, color=INK,
                  label="dwell = isolated settling")
    ax_dwell.fill_between([0, limit], [0, limit], [limit, limit], color=GREEN,
                          alpha=0.06, zorder=0)
    ax_dwell.set_xlim(0, limit)
    ax_dwell.set_ylim(0, limit)
    ax_dwell.set_xlabel("isolated R0036 settling reference (s)")
    ax_dwell.set_ylabel("Dev3A policy dwell (s)")
    ax_dwell.set_title("c  Only 8 of 55 dwells are long enough", loc="left", pad=7)
    ax_dwell.grid(color=LIGHT_GREY, lw=0.45)
    ax_dwell.legend(frameon=False, loc="upper left", fontsize=6.2,
                    handletextpad=0.35, borderaxespad=0.2)

    t_rel = np.asarray([row["t_s"] - hero_start for row in hero])
    gap_norm = np.maximum(np.asarray([row["normalised_absolute_gap_error"]
                                      for row in hero]), 1e-3)
    speed_norm = np.maximum(np.asarray([row["normalised_absolute_relative_speed"]
                                        for row in hero]), 1e-3)
    ax_gap.semilogy(t_rel, gap_norm, color=POLICY_COLOUR["C"], lw=1.35,
                    label="|gap − target| / 1 m")
    ax_gap.semilogy(t_rel, speed_norm, color=RED, lw=1.05, ls=(0, (4, 2)),
                    label="|relative speed| / 0.1 m/s")
    ax_gap.axhspan(1e-3, 1.0, color=GREEN, alpha=0.08)
    ax_gap.axhline(1.0, color=GREEN, lw=0.8)
    ax_gap.set_xlim(0, hero_end - hero_start)
    ax_gap.set_ylim(1e-2, max(1e4, float(max(gap_norm.max(), speed_norm.max())) * 1.25))
    ax_gap.set_xlabel("time since UAM-04 changes R → C at t = 250 s (s)")
    ax_gap.set_ylabel("normalised absolute error (log scale)")
    ax_gap.set_title("d  A 315 s dwell nearly reaches the gap target, but the two settling tests never overlap",
                     loc="left", pad=7)
    ax_gap.grid(which="major", color=LIGHT_GREY, lw=0.5)
    ax_gap.legend(frameon=False, ncol=2, loc="upper right")
    ax_gap.text(0.012, 0.055, "pass region: both curves ≤ 1",
                transform=ax_gap.transAxes, fontsize=6.6, color=GREEN,
                bbox=dict(boxstyle="round,pad=0.22", facecolor="white",
                          edgecolor=GREEN, linewidth=0.6))

    for ax in (ax_timeline, ax_gap):
        ax.spines[["top", "right"]].set_visible(False)
    fig.text(0.08, 0.016,
             "Gate outcome: 5/5 missions completed; no sampled NMAC; 0 m lateral motion; "
             "minimum horizontal separation 1,282.7 m.\n"
             "Only 14.5% of 55 longitudinal tasks were timing-compatible, 0% fully settled, "
             "and 81.8% reduced gap error; mean switching rose from 2.39 to 13.00 per flight.",
             fontsize=6.8, color=INK, ha="left", linespacing=1.25)
    fig.subplots_adjust(left=0.09, right=0.985, top=0.925, bottom=0.095)

    stem = output / "01-dev3a-longitudinal-gate"
    fig.savefig(stem.with_suffix(".svg"))
    fig.savefig(stem.with_suffix(".pdf"))
    fig.savefig(stem.with_suffix(".png"), dpi=300)
    plt.close(fig)

    manifest = {
        "figure": stem.name,
        "run_id": summary["run_id"],
        "source_files": {
            str(trace_path.relative_to(ROOT)): sha256(trace_path),
            str(summary_path.relative_to(ROOT)): sha256(summary_path),
            str(calibration_path.relative_to(ROOT)): sha256(calibration_path),
        },
        "outputs": {path.name: sha256(path) for path in sorted(output.iterdir())
                    if path.is_file() and path.name != "manifest.json"},
        "representative_interval": {
            "aircraft_id": hero_aircraft,
            "leader_id": "UAM-03",
            "transition": "R→C",
            "start_s": hero_start,
            "end_s": hero_end,
            "selection_reason": "long timing-compatible dwell with sub-metre best gap error",
        },
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
