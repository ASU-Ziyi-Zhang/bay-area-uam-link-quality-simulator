"""Single-lane policy and planning rate before and after the Bay Area calibration.

Compares the ``fixed_centerline`` case of two archives of the single-stream
capacity study: one run with the TRB classifier settings and one with the
calibrated settings. The compared window is the calibrated run's own matched
window (first exit to last entry over its stay and move cases), so the
calibrated numbers are identical to that run's briefing tables.

    python scripts/plot_calibration_comparison.py \
        --trb research/dynamic-transitions/runs/R0064 \
        --calibrated research/dynamic-transitions/runs/R0062 \
        --output research/dynamic-transitions/results/R0064-calibration-before-after
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_single_stream_briefing as sb  # noqa: E402

CASE = "fixed_centerline"


def validated(path):
    status = json.loads((path / "validation.json").read_text()).get("status")
    if status not in ("pass", "passed"):
        raise ValueError(f"refusing to plot {path}: validation status {status!r}")
    return sb.Run(path, None)


def settings(run):
    p = run.spec["parameters"]
    return {k: p[k] for k in ("threshold_db", "radio_s", "window_s", "persistence_k", "group_mode", "policy_s")}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trb", type=Path, required=True)
    parser.add_argument("--calibrated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sets = [("TRB settings", validated(args.trb), "#8a8a8a"), ("Calibrated", validated(args.calibrated), sb.NAVY)]
    args.output.mkdir(parents=True, exist_ok=True)
    sb.style()
    lo, hi = sb.steady_window(sets[1][1])

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 3.9), sharex=True, gridspec_kw={"hspace": .38})
    for ax, (label, run, colour) in zip(axes, sets):
        km, shares = sb.profile(run, CASE)
        ax.stackplot(km, *[shares[p] for p in "CRF"], colors=[sb.POLICY[p] for p in "CRF"], alpha=.92, lw=0)
        ax.set_ylim(0, 100); ax.set_xlim(0, km.max() + .5); ax.set_ylabel("share (%)", fontsize=11)
        ax.set_title(label, loc="left", fontsize=12, fontweight="bold", color=colour)
    axes[1].set_xlabel("distance along the corridor (km)", fontsize=11)
    handles = [Rectangle((0, 0), 1, 1, color=sb.POLICY[p]) for p in "CRF"]
    fig.legend(handles, ["C", "R", "F"], frameon=False, ncol=3, loc="upper right", bbox_to_anchor=(.99, 1.02))
    for ext in ("png", "svg"):
        fig.savefig(args.output / f"01-policy-along-corridor.{ext}", dpi=220, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.4, 3.9))
    rows, end = {}, 0.
    for label, run, colour in sets:
        part = run.capacity(CASE)
        t = np.array([float(r["timestamp_s"]) for r in part]); q = np.array([float(r["q_mix_uam_h"]) for r in part])
        ax.plot(t / 60, q, color=colour, lw=1.6, label=label)
        m = (t >= lo) & (t <= hi)
        s = run.summary(CASE); pm = s["policy_and_longitudinal_metrics"]
        rows[label] = {
            "run": run.path.name, "settings": settings(run), "completed": s["completed_requests"],
            "scheduled": s["scheduled_requests"], "policy_shares": pm["policy_shares"],
            "planning_rate_mean_uam_h": float(q[m].mean()), "planning_rate_q95_uam_h": sb.lower_tail(q[m]),
            "policy_switches_per_flight": float(np.mean(list(pm["switches_by_aircraft"].values()))),
            "aircraft_held_at_entry": sum(1 for v in s["entry_delays_s"].values() if v),
            "longest_entry_hold_s": max(s["entry_delays_s"].values()), "last_exit_min": s["end_s"] / 60,
            "sampled_nmac": pm["nmac_sampled"]}
        end = max(end, t.max())
    ax.axvspan(lo / 60, hi / 60, color="#eef2f7", lw=0, zorder=0)
    ax.text((lo + hi) / 120, 1.01, "compared window", transform=ax.get_xaxis_transform(), ha="center", fontsize=10, color=sb.NAVY)
    ax.set_xlim(0, end / 60); ax.set_xlabel("time (min)", fontsize=11); ax.set_ylabel("planning rate (UAM/h)", fontsize=11)
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(.5, -.2), ncol=2)
    for ext in ("png", "svg"):
        fig.savefig(args.output / f"02-planning-rate.{ext}", dpi=220, bbox_inches="tight")
    plt.close(fig)

    rows["compared_window_s"] = [lo, hi]
    (args.output / "numbers.json").write_text(json.dumps(rows, indent=2) + "\n")
    t, k = rows["TRB settings"], rows["Calibrated"]
    pct = lambda d: " / ".join(f"{100 * d[p]:.1f}" for p in "CRF")
    lines = [
        f"# {t['run']} (TRB settings) against {k['run']} (calibrated), single lane", "",
        "| | TRB settings | Calibrated |", "|---|---:|---:|",
        f"| SINR threshold (dB) | {t['settings']['threshold_db']} | {k['settings']['threshold_db']} |",
        f"| Radio sampling (s) | {t['settings']['radio_s']} | {k['settings']['radio_s']} |",
        f"| Assessment window (s) | {t['settings']['window_s']} | {k['settings']['window_s']} |",
        f"| Persistence k | {t['settings']['persistence_k']} | {k['settings']['persistence_k']} |",
        f"| Exposure group | {t['settings']['group_mode']} | {k['settings']['group_mode']} |",
        f"| C / R / F aircraft-time (%) | {pct(t['policy_shares'])} | {pct(k['policy_shares'])} |",
        f"| Planning rate, mean (UAM/h) | {t['planning_rate_mean_uam_h']:.1f} | {k['planning_rate_mean_uam_h']:.1f} |",
        f"| Planning rate, 95% reliable (UAM/h) | {t['planning_rate_q95_uam_h']:.1f} | {k['planning_rate_q95_uam_h']:.1f} |",
        f"| Policy switches per flight | {t['policy_switches_per_flight']:.1f} | {k['policy_switches_per_flight']:.1f} |",
        f"| Aircraft held at entry | {t['aircraft_held_at_entry']} | {k['aircraft_held_at_entry']} |",
        f"| Longest entry hold (s) | {t['longest_entry_hold_s']:.1f} | {k['longest_entry_hold_s']:.1f} |",
        f"| Last aircraft out (min) | {t['last_exit_min']:.1f} | {k['last_exit_min']:.1f} |",
        f"| Completed / scheduled | {t['completed']}/{t['scheduled']} | {k['completed']}/{k['scheduled']} |",
        f"| Sampled NMAC | {'yes' if t['sampled_nmac'] else 'none'} | {'yes' if k['sampled_nmac'] else 'none'} |",
        "", f"Compared window: {lo / 60:.1f}–{hi / 60:.1f} min.", ""]
    (args.output / "tables.md").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
