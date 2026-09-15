"""Briefing figures for R0056, one message per figure.

The archived composite answers every question on one canvas. These figures are
meant to be read in order: where the signal is weak, what the aircraft do about
it, what that changes, and what it costs. They carry no titles of their own so
the slide supplies the headline. Numbers are read from the run archive.
"""
from __future__ import annotations

import csv
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "research/dynamic-transitions/runs/R0056"
SRC = ROOT / "research/dynamic-transitions/figures/R0056"
OUT = ROOT / "research/dynamic-transitions/figures/R0056-briefing"

NAVY, INK, GREY, LIGHT = "#2C4E78", "#1a1a1a", "#6b6b6b", "#e8edf4"
POLICY = {"C": "#2e7d32", "R": "#d4a017", "F": "#c62828"}
OFF, ON = "#8a8a8a", NAVY
CASES = ("fixed_centerline", "spatial_grid")
LABEL = {"fixed_centerline": "Stay at centre", "spatial_grid": "Move within 3 × 3"}


def style():
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"], "font.size": 13,
        "axes.labelsize": 13, "axes.titlesize": 14, "text.color": INK, "axes.labelcolor": INK,
        "axes.edgecolor": INK, "xtick.color": INK, "ytick.color": INK, "axes.linewidth": .8,
        "svg.fonttype": "none", "figure.facecolor": "white", "savefig.facecolor": "white",
        "legend.fontsize": 12, "axes.spines.top": False, "axes.spines.right": False,
    })


def rows(path):
    with open(path, encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def summary(case):
    s = json.load(open(RUN / case / "summary.json"))
    return s[0] if isinstance(s, list) else s


def events(case):
    e = json.load(open(RUN / case / "events.json"))
    return e if isinstance(e, list) else e.get("events", [])


def steady_window():
    """From the first exit (corridor fully populated) to the last entry, common to both cases."""
    first_exit = max(min(float(r["t_s"]) for r in events(c) if r.get("status") == "exit") for c in CASES)
    last_entry = min(max(float(r["t_s"]) for r in events(c) if r.get("status") == "corridor_entry") for c in CASES)
    return first_exit, last_entry


def lower_tail(values, rho=0.95):
    v = np.sort(np.asarray(values, float))
    return float(v[int(np.floor((1 - rho) * (len(v) - 1)))])


def weak_segments():
    """Stretches where the centreline spends at least half its time outside C."""
    prof = [r for r in rows(SRC / "03-capacity-story-profile-source.csv") if r["case"] == "spatial_control_off"]
    km = np.array([float(r["q_km"]) for r in prof])
    weak = np.array([float(r["share_R"]) + float(r["share_F"]) >= 0.5 for r in prof])
    out, start = [], None
    for k, w in zip(km, weak):
        if w and start is None:
            start = k - 0.5
        if not w and start is not None:
            out.append((start, k - 0.5)); start = None
    if start is not None:
        out.append((start, km[-1] + 0.5))
    return out


def shade_weak(ax, segments, label=True):
    for i, (a, b) in enumerate(segments):
        ax.axvspan(a, b, color=POLICY["R"], alpha=.10, lw=0, zorder=0)
    if label and segments:
        a, b = max(segments, key=lambda s: s[1] - s[0])
        ax.text((a + b) / 2, 1.01, "weak-signal segment", transform=ax.get_xaxis_transform(),
                ha="center", va="bottom", fontsize=11.5, color="#8a6d00")


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg"):
        fig.savefig(OUT / f"{name}.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------- 1 setup
def fig_setup():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    xs, hs = (-300, 0, 300), (200, 300, 400)
    for ax, moving, title in ((axes[0], False, "Stay at centre"), (axes[1], True, "Move within 3 × 3")):
        for x in xs:
            for h in hs:
                centre = (x, h) == (0, 300)
                ax.add_patch(Rectangle((x - 110, h - 36), 220, 72, facecolor=NAVY if centre else LIGHT,
                                       edgecolor=NAVY if (centre or moving) else "#c9d2de", lw=1.3, zorder=2))
        if moving:
            for x in xs:
                for h in hs:
                    if (x, h) != (0, 300):
                        ax.add_patch(FancyArrowPatch((0, 300), (x * .78, 300 + (h - 300) * .72),
                                     arrowstyle="-|>", mutation_scale=14, color=NAVY, lw=1.4, zorder=3))
        ax.text(0, 300, "entry", ha="center", va="center", color="white", fontsize=12, fontweight="bold", zorder=4)
        ax.set_xlim(-450, 450); ax.set_ylim(130, 470)
        ax.set_xticks(xs); ax.set_xticklabels(["−300 m", "0", "+300 m"])
        ax.set_yticks(hs); ax.set_yticklabels(["200 m", "300 m", "400 m"])
        ax.set_xlabel("lateral offset"); ax.set_title(title, fontsize=15, fontweight="bold",
                                                       color=NAVY if moving else GREY, pad=10)
        ax.spines["left"].set_visible(False); ax.spines["bottom"].set_visible(False)
        ax.tick_params(length=0)
    axes[0].set_ylabel("altitude")
    fig.tight_layout(w_pad=3)
    save(fig, "01-setup")


# ------------------------------------------------- 2 where the signal is weak
def fig_centreline_policy(segments):
    prof = [r for r in rows(SRC / "03-capacity-story-profile-source.csv") if r["case"] == "spatial_control_off"]
    km = np.array([float(r["q_km"]) for r in prof])
    shares = {p: np.array([float(r[f"share_{p}"]) for r in prof]) * 100 for p in "CRF"}
    fig, ax = plt.subplots(figsize=(12, 3.9))
    ax.stackplot(km, shares["C"], shares["R"], shares["F"], colors=[POLICY[p] for p in "CRF"],
                 labels=["coordinated (C)", "reactive (R)", "fallback (F)"], alpha=.92, lw=0)
    ax.set_xlim(0, km.max() + .5); ax.set_ylim(0, 100)
    ax.set_xlabel("distance along the corridor (km)"); ax.set_ylabel("time share (%)")
    ax.legend(frameon=False, ncol=3, loc="upper left", bbox_to_anchor=(0, -.22))
    for a, b in segments:
        ax.annotate("", (a, 104), (b, 104), xycoords="data", arrowprops=dict(arrowstyle="<->", color=INK, lw=1),
                    annotation_clip=False)
    a, b = max(segments, key=lambda s: s[1] - s[0])
    ax.text((a + b) / 2, 107, f"{a:.0f}–{b:.0f} km: mostly reactive", ha="center", fontsize=12.5)
    save(fig, "02-centreline-policy")


# --------------------------------------------------- 3 what the aircraft do
def load_trace(case, every_s=2.0):
    t = json.load(gzip.open(RUN / case / "trace.json.gz", "rt"))
    by = {}
    for r in t:
        if abs(float(r["t_s"]) / every_s - round(float(r["t_s"]) / every_s)) > 1e-9:
            continue
        by.setdefault(r["aircraft_id"], []).append(
            (float(r["q_m"]) / 1000, float(r["offset_m"]), float(r["xyz"][2]), r["policy"]))
    return by


def coloured(ax, x, y, pol):
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    seg = np.concatenate([pts[:-1], pts[1:]], axis=1)
    ax.add_collection(LineCollection(seg, colors=[POLICY[p] for p in pol[:-1]], lw=1.1, alpha=.8))


def fig_trajectories(segments):
    by = load_trace("spatial_grid")
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(12, 6.2), sharex=True, gridspec_kw={"hspace": .12})
    moved = 0
    for ident, pts in by.items():
        q, d, h, pol = map(list, zip(*pts))
        if max(abs(v) for v in d) > 1 or max(abs(v - 300) for v in h) > 1:
            moved += 1
        coloured(top, q, d, pol); coloured(bottom, q, h, pol)
    for ax in (top, bottom):
        shade_weak(ax, segments, label=ax is top)
        ax.set_xlim(0, 50)
    top.set_ylim(-360, 360); top.set_yticks([-300, 0, 300]); top.set_ylabel("lateral offset (m)")
    bottom.set_ylim(170, 430); bottom.set_yticks([200, 300, 400]); bottom.set_ylabel("altitude (m)")
    bottom.set_xlabel("distance along the corridor (km)")
    handles = [plt.Line2D([], [], color=POLICY[p], lw=3) for p in "CRF"]
    bottom.legend(handles, ["C", "R", "F"], frameon=False, ncol=3, loc="upper left", bbox_to_anchor=(0, -.28))
    save(fig, "03-trajectories")
    return moved


# ------------------------------------------------ 4 policy along the route
def fig_policy_off_on(segments):
    prof = rows(SRC / "03-capacity-story-profile-source.csv")
    fig, axes = plt.subplots(2, 1, figsize=(12, 5.6), sharex=True, gridspec_kw={"hspace": .28})
    for ax, key, case in ((axes[0], "spatial_control_off", CASES[0]), (axes[1], "spatial_control_on", CASES[1])):
        part = [r for r in prof if r["case"] == key]
        km = np.array([float(r["q_km"]) for r in part])
        ax.stackplot(km, *[np.array([float(r[f"share_{p}"]) for r in part]) * 100 for p in "CRF"],
                     colors=[POLICY[p] for p in "CRF"], alpha=.92, lw=0)
        ax.set_ylim(0, 100); ax.set_xlim(0, km.max() + .5); ax.set_ylabel("time share (%)")
        ax.set_title(LABEL[case], loc="left", fontsize=14, fontweight="bold", color=OFF if case == CASES[0] else ON)
    axes[1].set_xlabel("distance along the corridor (km)")
    handles = [Rectangle((0, 0), 1, 1, color=POLICY[p]) for p in "CRF"]
    axes[1].legend(handles, ["C", "R", "F"], frameon=False, ncol=3, loc="upper left", bbox_to_anchor=(0, -.3))
    save(fig, "04-policy-off-on")


# ------------------------------------------------------ 5 operational outcome
def fig_outcome():
    s = {c: summary(c) for c in CASES}
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.9), gridspec_kw={"width_ratios": [2.1, 1, 1], "wspace": .45})
    ax = axes[0]
    for y, c in ((1, CASES[0]), (0, CASES[1])):
        left = 0
        for p in "CRF":
            v = s[c]["policy_and_longitudinal_metrics"]["policy_shares"][p] * 100
            ax.barh(y, v, left=left, color=POLICY[p], height=.55)
            if v > 6:
                ax.text(left + v / 2, y, f"{v:.1f}%", ha="center", va="center", color="white", fontsize=12.5, fontweight="bold")
            left += v
    ax.set_yticks([1, 0]); ax.set_yticklabels([LABEL[c] for c in CASES]); ax.set_xlim(0, 100)
    ax.set_xlabel("aircraft-time in each policy (%)"); ax.spines["left"].set_visible(False); ax.tick_params(axis="y", length=0)
    ax.set_title("policy mix", loc="left", fontsize=13.5)
    delayed = [sum(1 for v in s[c]["entry_delays_s"].values() if v) for c in CASES]
    last = [s[c]["end_s"] for c in CASES]
    for ax, vals, title, fmt in ((axes[1], delayed, "aircraft held at entry", "{:.0f}"),
                                 (axes[2], [v / 60 for v in last], "last aircraft out (min)", "{:.1f}")):
        bars = ax.bar([0, 1], vals, color=[OFF, ON], width=.6)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v, fmt.format(v), ha="center", va="bottom", fontsize=13, fontweight="bold")
        ax.set_xticks([0, 1]); ax.set_xticklabels(["stay", "move"]); ax.set_title(title, loc="left", fontsize=13.5)
        ax.set_ylim(0, max(vals) * 1.22); ax.spines["left"].set_visible(False); ax.set_yticks([])
    save(fig, "05-outcome")
    return delayed, last


# ----------------------------------------------------------------- 6 capacity
def fig_capacity():
    lo, hi = steady_window()
    ts = rows(SRC / "03-capacity-story-time-source.csv")
    fig, ax = plt.subplots(figsize=(12, 4.4))
    stats = {}
    for c, colour in ((CASES[0], OFF), (CASES[1], ON)):
        part = [r for r in ts if r["case"] == c]
        t = np.array([float(r["timestamp_s"]) for r in part]); q = np.array([float(r["q_mix_uam_h"]) for r in part])
        ax.plot(t / 60, q, color=colour, lw=1.8, label=LABEL[c])
        m = (t >= lo) & (t <= hi)
        stats[c] = (float(q[m].mean()), lower_tail(q[m]))
    end_min = max(float(r["timestamp_s"]) for r in ts) / 60
    ax.set_xlim(0, end_min)
    ax.axvspan(0, lo / 60, color="#f0f0f0", lw=0, zorder=0)
    ax.axvspan(hi / 60, end_min, color="#f0f0f0", lw=0, zorder=0)
    ax.text(lo / 120, 1.01, "filling", transform=ax.get_xaxis_transform(), ha="center", fontsize=11.5, color=GREY)
    ax.text((lo + hi) / 120, 1.01, "compared window", transform=ax.get_xaxis_transform(), ha="center", fontsize=11.5, color=NAVY)
    ax.text(hi / 60 + 4, 1.01, "emptying", transform=ax.get_xaxis_transform(), ha="left", fontsize=11.5, color=GREY)
    ax.set_xlabel("time (min)"); ax.set_ylabel("planning rate (UAM/h)")
    ax.legend(frameon=False, loc="lower left")
    save(fig, "06-capacity")
    return (lo, hi), stats


def main():
    style()
    segments = weak_segments()
    fig_setup()
    fig_centreline_policy(segments)
    moved = fig_trajectories(segments)
    fig_policy_off_on(segments)
    delayed, last = fig_outcome()
    window, stats = fig_capacity()
    record = {"weak_segments_km": segments, "aircraft_moved": moved, "entry_delayed": dict(zip(CASES, delayed)),
              "last_exit_s": dict(zip(CASES, last)), "steady_window_s": window,
              "steady_window_capacity_uam_h": {c: {"mean": m, "q95": q} for c, (m, q) in stats.items()}}
    (OUT / "numbers.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
