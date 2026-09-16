"""Two follow-up figures for R0056: the group-size confound, and the costs.

Moving aircraft across the grid splits the single stream into smaller exposure
groups. The classifier was calibrated on five-aircraft groups, so the raw policy
mix of the two cases is measured under different group sizes. The first figure
separates that effect by reweighting the moving case to the stationary case's
group-size composition. The second shows what did not improve.
"""
from __future__ import annotations

import gzip
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "research/dynamic-transitions/runs/R0056"
OUT = ROOT / "research/dynamic-transitions/figures/R0056-briefing"
NAVY, INK, GREY = "#2C4E78", "#1a1a1a", "#6b6b6b"
POLICY = {"C": "#2e7d32", "R": "#d4a017", "F": "#c62828"}
OFF, ON = "#8a8a8a", NAVY
CASES = ("fixed_centerline", "spatial_grid")
BINS = ("1", "2–4", "5")


def style():
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"], "font.size": 13, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK, "ytick.color": INK,
        "axes.linewidth": .8, "svg.fonttype": "none", "figure.facecolor": "white",
        "savefig.facecolor": "white", "axes.spines.top": False, "axes.spines.right": False,
    })


def binned(case):
    rows = json.load(gzip.open(RUN / case / "observations.json.gz", "rt"))
    rows = rows if isinstance(rows, list) else rows.get("observations", [])
    tab = {b: defaultdict(int) for b in BINS}
    for r in rows:
        g = int(r["group_size"])
        tab["1" if g == 1 else ("2–4" if g < 5 else "5")][r["policy"]] += 1
    total = sum(sum(t.values()) for t in tab.values())
    composition = {b: sum(tab[b].values()) / total for b in BINS}
    within = {b: {p: tab[b][p] / max(1, sum(tab[b].values())) for p in "CRF"} for b in BINS}
    raw = {p: sum(tab[b][p] for b in BINS) / total for p in "CRF"}
    return composition, within, raw


def stacked(ax, y, shares, label_min=5):
    left = 0
    for p in "CRF":
        v = shares[p] * 100
        ax.barh(y, v, left=left, color=POLICY[p], height=.6)
        if v >= label_min:
            ax.text(left + v / 2, y, f"{v:.1f}%", ha="center", va="center", color="white", fontsize=12, fontweight="bold")
        left += v


def fig_group_size():
    off_comp, off_within, off_raw = binned(CASES[0])
    on_comp, on_within, on_raw = binned(CASES[1])
    reweighted = {p: sum(off_comp[b] * on_within[b][p] for b in BINS) for p in "CRF"}
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.2), gridspec_kw={"width_ratios": [1, 1.35], "wspace": .5})
    x = np.arange(len(BINS)); w = .36
    a.bar(x - w / 2, [off_comp[k] * 100 for k in BINS], w, color=OFF, label="stay at centre")
    a.bar(x + w / 2, [on_comp[k] * 100 for k in BINS], w, color=ON, label="move within 3 × 3")
    for i, k in enumerate(BINS):
        for dx, v in ((-w / 2, off_comp[k]), (w / 2, on_comp[k])):
            a.text(i + dx, v * 100 + 1.5, f"{v * 100:.0f}%", ha="center", fontsize=11.5)
    a.set_xticks(x); a.set_xticklabels([f"{k} aircraft" for k in BINS]); a.set_ylim(0, 100)
    a.set_ylabel("share of observations (%)"); a.set_title("exposure-group size", loc="left", fontsize=13.5)
    a.legend(frameon=False, loc="upper left")
    rows = [("stay at centre", off_raw), ("move, as measured", on_raw), ("move, reweighted", reweighted)]
    for i, (label, shares) in enumerate(rows):
        stacked(b, len(rows) - 1 - i, shares)
    b.set_yticks(range(len(rows))); b.set_yticklabels([r[0] for r in rows][::-1]); b.set_xlim(0, 100)
    b.set_xlabel("radio-sample observations in each policy (%)")
    b.set_title("policy mix", loc="left", fontsize=13.5)
    b.spines["left"].set_visible(False); b.tick_params(axis="y", length=0)
    for i, (_, shares) in enumerate(rows):
        b.text(101, len(rows) - 1 - i, f"F {shares['F'] * 100:.2f}%", va="center", fontsize=11.5, color=POLICY["F"])
    OUT.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "svg"):
        fig.savefig(OUT / f"07-group-size.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)
    return {"composition": {"stay": off_comp, "move": on_comp},
            "policy_mix": {"stay": off_raw, "move_measured": on_raw, "move_reweighted": reweighted}}


def fig_costs():
    s = {}
    for c in CASES:
        v = json.load(open(RUN / c / "summary.json")); s[c] = v[0] if isinstance(v, list) else v
    pm = {c: s[c]["policy_and_longitudinal_metrics"] for c in CASES}
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.9), gridspec_kw={"wspace": .5})
    panels = (
        ("spacing task finished before\nthe next policy change (%)",
         [pm[c]["completion_before_next_change_fraction"] * 100 for c in CASES], "{:.1f}"),
        ("spacing error reduced (%)", [pm[c]["gap_error_improved_fraction"] * 100 for c in CASES], "{:.1f}"),
        ("computation time (min)", [s[c]["wall_time_s"] / 60 for c in CASES], "{:.0f}"),
    )
    for ax, (title, vals, fmt) in zip(axes, panels):
        bars = ax.bar([0, 1], vals, color=[OFF, ON], width=.6)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, v, fmt.format(v), ha="center", va="bottom", fontsize=13, fontweight="bold")
        ax.set_xticks([0, 1]); ax.set_xticklabels(["stay", "move"]); ax.set_title(title, loc="left", fontsize=13)
        ax.set_ylim(0, max(max(vals) * 1.25, 1)); ax.spines["left"].set_visible(False); ax.set_yticks([])
    for ext in ("png", "svg"):
        fig.savefig(OUT / f"08-costs.{ext}", dpi=200, bbox_inches="tight")
    plt.close(fig)
    return {c: {"finished": pm[c]["completion_before_next_change_fraction"],
                "improved": pm[c]["gap_error_improved_fraction"], "wall_s": s[c]["wall_time_s"]} for c in CASES}


if __name__ == "__main__":
    style()
    out = {"group_size": fig_group_size(), "costs": fig_costs()}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "numbers-extra.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(out, indent=2))
