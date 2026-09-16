"""Briefing figures for one single-entry 3 x 3 run (stay vs move), one message per figure.

Read in order: the grid, where the centreline signal is weak, where the moving
aircraft go, how the policy changes along the route, and the planning rate over
time. Scalar outcomes are written as tables (``tables.md``), not bar charts.
Everything is computed from the run archive, so any run of this study can be
passed with ``--run``.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
NAVY, INK, GREY, LIGHT = "#2C4E78", "#1a1a1a", "#6b6b6b", "#e8edf4"
POLICY = {"C": "#2e7d32", "R": "#d4a017", "F": "#c62828"}
OFF, ON = "#8a8a8a", NAVY
CASES = ("fixed_centerline", "spatial_grid")
LABEL = {"fixed_centerline": "Stay at centre", "spatial_grid": "Move within 3 × 3"}
SECTIONS_KM = (5, 20, 28, 34, 38, 44)


def style():
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"], "font.size": 13,
        "axes.labelsize": 13, "axes.titlesize": 14, "text.color": INK, "axes.labelcolor": INK,
        "axes.edgecolor": INK, "xtick.color": INK, "ytick.color": INK, "axes.linewidth": .8,
        "svg.fonttype": "none", "figure.facecolor": "white", "savefig.facecolor": "white",
        "legend.fontsize": 12, "axes.spines.top": False, "axes.spines.right": False,
    })


class Run:
    def __init__(self, path, out):
        self.path, self.out = path, out
        self.spec = json.load(open(path / "resolved_config.json"))
        self.points = [(p["offset_m"], p["altitude_m"]) for p in self.spec["traffic"]["flow_points"]]
        self.entry = self.points[self.spec["traffic"]["entry_flow_index"]]
        self._trace = {}

    def summary(self, case):
        s = json.load(open(self.path / case / "summary.json"))
        return s[0] if isinstance(s, list) else s

    def events(self, case):
        e = json.load(open(self.path / case / "events.json"))
        return e if isinstance(e, list) else e.get("events", [])

    def trace(self, case):
        if case not in self._trace:
            self._trace[case] = json.load(gzip.open(self.path / case / "trace.json.gz", "rt"))
        return self._trace[case]

    def capacity(self, case):
        with open(self.path / case / "capacity_trace.csv", encoding="utf-8-sig") as stream:
            return list(csv.DictReader(stream))

    def save(self, fig, name):
        self.out.mkdir(parents=True, exist_ok=True)
        for ext in ("png", "svg"):
            fig.savefig(self.out / f"{name}.{ext}", dpi=200, bbox_inches="tight")
        plt.close(fig)


def lower_tail(values, rho=0.95):
    v = np.sort(np.asarray(values, float))
    return float(v[int(np.floor((1 - rho) * (len(v) - 1)))])


def profile(run, case, bin_km=1.0):
    """Aircraft-time share of each policy in 1 km bins along the corridor."""
    time = defaultdict(Counter)
    for r in run.trace(case):
        time[int(r["q_m"] / 1000 // bin_km)][r["policy"]] += r["dt_s"]
    bins = sorted(time)
    km = np.array([(b + .5) * bin_km for b in bins])
    shares = {p: np.array([time[b][p] / sum(time[b].values()) for b in bins]) * 100 for p in "CRF"}
    return km, shares


def weak_segments(km, shares):
    """Stretches where the centreline spends at least half its time outside C."""
    out, start = [], None
    for k, w in zip(km, shares["R"] + shares["F"] >= 50):
        if w and start is None:
            start = k - .5
        if not w and start is not None:
            out.append((start, k - .5)); start = None
    if start is not None:
        out.append((start, km[-1] + .5))
    return out


def steady_window(run):
    """From the first exit (corridor fully populated) to the last entry, common to both cases."""
    first_exit = max(min(float(r["t_s"]) for r in run.events(c) if r.get("status") == "exit") for c in CASES)
    last_entry = min(max(float(r["t_s"]) for r in run.events(c) if r.get("status") == "corridor_entry") for c in CASES)
    return first_exit, last_entry


# ---------------------------------------------------------------- 1 setup
def fig_setup(run):
    xs = sorted({p[0] for p in run.points}); hs = sorted({p[1] for p in run.points})
    dx, dh = xs[1] - xs[0], hs[1] - hs[0]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, moving in ((axes[0], False), (axes[1], True)):
        for x, h in run.points:
            centre = (x, h) == run.entry
            ax.add_patch(Rectangle((x - .37 * dx, h - .36 * dh), .74 * dx, .72 * dh, facecolor=NAVY if centre else LIGHT,
                                   edgecolor=NAVY if (centre or moving) else "#c9d2de", lw=1.3, zorder=2))
            if moving and not centre:
                ex, eh = run.entry
                ax.add_patch(FancyArrowPatch((ex, eh), (ex + (x - ex) * .78, eh + (h - eh) * .72),
                             arrowstyle="-|>", mutation_scale=14, color=NAVY, lw=1.4, zorder=3))
        ax.text(*run.entry, "entry", ha="center", va="center", color="white", fontsize=12, fontweight="bold", zorder=4)
        ax.set_xlim(xs[0] - .5 * dx, xs[-1] + .5 * dx); ax.set_ylim(hs[0] - .7 * dh, hs[-1] + .7 * dh)
        ax.set_xticks(xs); ax.set_xticklabels([f"{x:+.0f} m" if x else "0" for x in xs])
        ax.set_yticks(hs); ax.set_yticklabels([f"{h:.0f} m" for h in hs])
        ax.set_xlabel("lateral offset")
        ax.set_title(LABEL[CASES[1] if moving else CASES[0]], fontsize=15, fontweight="bold", color=NAVY if moving else GREY, pad=10)
        ax.spines["left"].set_visible(False); ax.spines["bottom"].set_visible(False); ax.tick_params(length=0)
    axes[0].set_ylabel("altitude")
    fig.tight_layout(w_pad=3)
    run.save(fig, "01-setup")


# ------------------------------------------------- 2 where the signal is weak
def fig_centreline_policy(run, km, shares, segments):
    fig, ax = plt.subplots(figsize=(12, 3.9))
    ax.stackplot(km, shares["C"], shares["R"], shares["F"], colors=[POLICY[p] for p in "CRF"],
                 labels=["coordinated (C)", "reactive (R)", "fallback (F)"], alpha=.92, lw=0)
    ax.set_xlim(0, km.max() + .5); ax.set_ylim(0, 100)
    ax.set_xlabel("distance along the corridor (km)"); ax.set_ylabel("time share (%)")
    ax.legend(frameon=False, ncol=3, loc="upper left", bbox_to_anchor=(0, -.22))
    for a, b in segments:
        ax.annotate("", (a, 104), (b, 104), arrowprops=dict(arrowstyle="<->", color=INK, lw=1), annotation_clip=False)
    if segments:
        a, b = max(segments, key=lambda s: s[1] - s[0])
        ax.text((a + b) / 2, 107, f"{a:.0f}–{b:.0f} km: mostly reactive", ha="center", fontsize=12.5)
    run.save(fig, "02-centreline-policy")


# ------------------------------------------ 3 where the moving aircraft fly
def fig_cross_sections(run, segments):
    """Grid seen head-on at several distances: share of aircraft-time in each cell, and its F share."""
    xs = sorted({p[0] for p in run.points}); hs = sorted({p[1] for p in run.points})
    dx, dh = xs[1] - xs[0], hs[1] - hs[0]
    cells = defaultdict(lambda: defaultdict(Counter))
    for r in run.trace(CASES[1]):
        k = int(round(r["q_m"] / 1000))
        if k not in SECTIONS_KM or abs(r["q_m"] / 1000 - k) > .5:
            continue
        cell = (xs[int(np.argmin([abs(r["offset_m"] - x) for x in xs]))], hs[int(np.argmin([abs(r["xyz"][2] - h) for h in hs]))])
        cells[k][cell][r["policy"]] += r["dt_s"]
    fig, axes = plt.subplots(1, len(SECTIONS_KM), figsize=(2.35 * len(SECTIONS_KM), 3.6), sharey=True)
    record = {}
    for ax, k in zip(axes, SECTIONS_KM):
        total = sum(sum(c.values()) for c in cells[k].values())
        record[k] = {}
        for x in xs:
            for h in hs:
                use = cells[k].get((x, h), Counter()); t = sum(use.values())
                share = 100 * t / total if total else 0
                if t:
                    top = max("CRF", key=lambda p: use[p]); f = 100 * use["F"] / t
                    ax.add_patch(Rectangle((x - .45 * dx, h - .42 * dh), .9 * dx, .84 * dh, facecolor=POLICY[top],
                                           alpha=.25 + .75 * min(1, share / 60), lw=0))
                    ax.text(x, h + .1 * dh, f"{share:.0f}%", ha="center", va="center", fontsize=11, fontweight="bold",
                            color="white" if share > 30 else INK)
                    if f >= .5:
                        ax.text(x, h - .22 * dh, f"F {f:.0f}%", ha="center", va="center", fontsize=9, color=INK)
                    record[k][f"{x:+.0f},{h:.0f}"] = {"share_pct": round(share, 1), "F_pct": round(f, 1)}
                else:
                    ax.add_patch(Rectangle((x - .45 * dx, h - .42 * dh), .9 * dx, .84 * dh, facecolor="#f3f3f3", lw=0))
        weak = any(a <= k <= b for a, b in segments)
        ax.set_title(f"{k} km" + ("\nweak signal" if weak else "\n"), fontsize=12.5, color="#8a6d00" if weak else INK)
        ax.set_xlim(xs[0] - .55 * dx, xs[-1] + .55 * dx); ax.set_ylim(hs[0] - .55 * dh, hs[-1] + .55 * dh)
        ax.set_xticks(xs); ax.set_xticklabels([f"{x:+.0f}" if x else "0" for x in xs], fontsize=10)
        ax.set_yticks(hs); ax.tick_params(length=0)
        for s in ax.spines.values():
            s.set_visible(False)
    axes[0].set_ylabel("altitude (m)")
    fig.supxlabel("lateral offset (m)", fontsize=12, y=-.02)
    handles = [Rectangle((0, 0), 1, 1, color=POLICY[p]) for p in "CRF"]
    fig.legend(handles, ["cell mostly C", "mostly R", "mostly F"], frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(.5, -.14))
    fig.tight_layout(w_pad=.6)
    run.save(fig, "03-cross-sections")
    return record


# ------------------------------------------------ 4 policy along the route
def fig_policy_off_on(run):
    fig, axes = plt.subplots(2, 1, figsize=(12, 5.6), sharex=True, gridspec_kw={"hspace": .28})
    for ax, case in zip(axes, CASES):
        km, shares = profile(run, case)
        ax.stackplot(km, *[shares[p] for p in "CRF"], colors=[POLICY[p] for p in "CRF"], alpha=.92, lw=0)
        ax.set_ylim(0, 100); ax.set_xlim(0, km.max() + .5); ax.set_ylabel("time share (%)")
        ax.set_title(LABEL[case], loc="left", fontsize=14, fontweight="bold", color=OFF if case == CASES[0] else ON)
    axes[1].set_xlabel("distance along the corridor (km)")
    handles = [Rectangle((0, 0), 1, 1, color=POLICY[p]) for p in "CRF"]
    axes[1].legend(handles, ["C", "R", "F"], frameon=False, ncol=3, loc="upper left", bbox_to_anchor=(0, -.3))
    run.save(fig, "04-policy-off-on")


# ----------------------------------------------------------------- 5 capacity
def fig_capacity(run):
    lo, hi = steady_window(run)
    fig, ax = plt.subplots(figsize=(12, 4.4))
    stats, end = {}, 0.
    for c, colour in ((CASES[0], OFF), (CASES[1], ON)):
        part = run.capacity(c)
        t = np.array([float(r["timestamp_s"]) for r in part]); q = np.array([float(r["q_mix_uam_h"]) for r in part])
        ax.plot(t / 60, q, color=colour, lw=1.8, label=LABEL[c])
        m = (t >= lo) & (t <= hi)
        stats[c] = {"mean": float(q[m].mean()), "q95": lower_tail(q[m]), "snapshots": int(m.sum())}
        end = max(end, t.max())
    ax.set_xlim(0, end / 60)
    ax.axvspan(0, lo / 60, color="#f0f0f0", lw=0, zorder=0)
    ax.axvspan(hi / 60, end / 60, color="#f0f0f0", lw=0, zorder=0)
    ax.text(lo / 120, 1.01, "filling", transform=ax.get_xaxis_transform(), ha="center", fontsize=11.5, color=GREY)
    ax.text((lo + hi) / 120, 1.01, "compared window", transform=ax.get_xaxis_transform(), ha="center", fontsize=11.5, color=NAVY)
    ax.text((hi + end) / 120, 1.01, "emptying", transform=ax.get_xaxis_transform(), ha="center", fontsize=11.5, color=GREY)
    ax.set_xlabel("time (min)"); ax.set_ylabel("planning rate (UAM/h)")
    ax.legend(frameon=False, loc="lower left")
    run.save(fig, "05-capacity")
    return (lo, hi), stats


# ------------------------------------------------------------------- tables
def tables(run, window, stats):
    s = {c: run.summary(c) for c in CASES}
    pm = {c: s[c]["policy_and_longitudinal_metrics"] for c in CASES}
    cap = {c: s[c]["conditional_planning_capacity"] for c in CASES}
    moves = s[CASES[1]].get("lane_change_metrics", {}).get("completed_lane_changes", 0)
    pct = lambda v: f"{100 * v:.1f}%"
    gain = lambda key: f"{100 * (stats[CASES[1]][key] / stats[CASES[0]][key] - 1):+.1f}%"
    lines = [
        f"# {run.path.name} — stay at centre vs move within 3 × 3", "",
        "## Policy and capacity", "",
        "| | Stay at centre | Move within 3 × 3 | Change |", "|---|---:|---:|---:|",
        *[f"| {p} aircraft-time | {pct(pm[CASES[0]]['policy_shares'][p])} | {pct(pm[CASES[1]]['policy_shares'][p])} | "
          f"{100 * (pm[CASES[1]]['policy_shares'][p] - pm[CASES[0]]['policy_shares'][p]):+.1f} pts |" for p in "CRF"],
        f"| Planning rate, compared window, mean (UAM/h) | {stats[CASES[0]]['mean']:.1f} | {stats[CASES[1]]['mean']:.1f} | {gain('mean')} |",
        f"| Planning rate, compared window, 95% reliable (UAM/h) | {stats[CASES[0]]['q95']:.1f} | {stats[CASES[1]]['q95']:.1f} | {gain('q95')} |",
        f"| Planning rate, whole run, 95% reliable (UAM/h) | {cap[CASES[0]]['q_mix_rho_uam_h']:.1f} | {cap[CASES[1]]['q_mix_rho_uam_h']:.1f} | |",
        "", f"Compared window: {window[0] / 60:.1f}–{window[1] / 60:.1f} min (first exit to last entry).", "",
        "## Operations", "",
        "| | Stay at centre | Move within 3 × 3 |", "|---|---:|---:|",
        f"| Completed moves | 0 | {moves} |",
        f"| Aircraft held at entry | {sum(1 for v in s[CASES[0]]['entry_delays_s'].values() if v)} | {sum(1 for v in s[CASES[1]]['entry_delays_s'].values() if v)} |",
        f"| Longest entry hold (s) | {max(s[CASES[0]]['entry_delays_s'].values()):.1f} | {max(s[CASES[1]]['entry_delays_s'].values()):.1f} |",
        f"| Last aircraft out (min) | {s[CASES[0]]['end_s'] / 60:.1f} | {s[CASES[1]]['end_s'] / 60:.1f} |",
        f"| Speed reversals per flight | {pm[CASES[0]]['mean_speed_direction_reversals_per_flight']:.1f} | {pm[CASES[1]]['mean_speed_direction_reversals_per_flight']:.1f} |",
        f"| Spacing error reduced | {pct(pm[CASES[0]]['gap_error_improved_fraction'])} | {pct(pm[CASES[1]]['gap_error_improved_fraction'])} |",
        f"| Spacing task finished before the next policy change | {pct(pm[CASES[0]]['completion_before_next_change_fraction'])} | {pct(pm[CASES[1]]['completion_before_next_change_fraction'])} |",
        f"| Sampled NMAC | {'yes' if pm[CASES[0]]['nmac_sampled'] else 'none'} | {'yes' if pm[CASES[1]]['nmac_sampled'] else 'none'} |",
        "",
    ]
    (run.out / "tables.md").write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", default="R0062")
    args = parser.parse_args()
    runs = ROOT / "research/dynamic-transitions/runs"
    run = Run(runs / args.run, ROOT / f"research/dynamic-transitions/figures/{args.run}-briefing")
    style()
    km, shares = profile(run, CASES[0])
    segments = weak_segments(km, shares)
    fig_setup(run)
    fig_centreline_policy(run, km, shares, segments)
    sections = fig_cross_sections(run, segments)
    fig_policy_off_on(run)
    window, stats = fig_capacity(run)
    tables(run, window, stats)
    record = {"run": args.run, "weak_segments_km": segments, "steady_window_s": window,
              "steady_window_capacity_uam_h": stats, "cross_sections": sections}
    (run.out / "numbers.json").write_text(json.dumps(record, indent=2) + "\n")
    print((run.out / "tables.md").read_text())


if __name__ == "__main__":
    main()
