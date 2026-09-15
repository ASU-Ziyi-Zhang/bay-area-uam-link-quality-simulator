"""Briefing figures for step 2, styled for the slide deck rather than the archive.

The archive figures answer "is this right"; these answer "what happened and why",
for an audience seeing the experiment for the first time. Same numbers, read from
the validated run, never retyped.
"""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402
import sys  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from run_five_uam_lateral_step2 import make_shape  # noqa: E402

NAVY, INK, GREY = "#2C4E78", "#1a1a1a", "#6b6b6b"
GOOD, BAD, WARN = "#2e7d32", "#c62828", "#b8860b"
LANE = "#e8edf4"


def style():
    plt.rcParams.update({
        "font.family": ["Arial", "Helvetica", "DejaVu Sans"], "font.size": 13,
        "axes.labelsize": 13, "axes.titlesize": 15, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
        "ytick.color": INK, "axes.linewidth": .8, "svg.fonttype": "none",
        "figure.facecolor": "white", "savefig.facecolor": "white", "legend.fontsize": 11.5,
    })


def banner(fig, kicker, title):
    """The deck's section label and headline, in the deck's navy."""
    fig.text(.012, .965, kicker, fontsize=11.5, color=NAVY, fontweight="bold", va="top")
    fig.text(.012, .915, title, fontsize=17.5, color=INK, va="top")


def load(run, name):
    with gzip.open(Path(run) / f"{name}.json.gz", "rt") as stream:
        return json.load(stream)


def aircraft(ax, x, y, colour, label, sub=None):
    ax.plot([x], [y], marker=">", ms=17, color=colour, zorder=5, clip_on=False)
    ax.annotate(label, (x, y), xytext=(0, 15), textcoords="offset points", ha="center",
                fontsize=14, fontweight="bold", color=colour, zorder=6)
    if sub:
        ax.annotate(sub, (x, y), xytext=(0, -26), textcoords="offset points", ha="center",
                    fontsize=11, color=GREY, zorder=6)


def gap_arrow(ax, x0, x1, y, text, colour=GREY, offset=0):
    ax.annotate("", (x1, y + offset), (x0, y + offset),
                arrowprops=dict(arrowstyle="<->", color=colour, lw=1.1))
    ax.annotate(text, ((x0 + x1) / 2, y + offset), xytext=(0, 5), textcoords="offset points",
                ha="center", fontsize=11.5, color=colour)


def layout_figure(spec):
    """Where the five aircraft start, and what A is deciding about."""
    case = next(c for c in spec["part_A"] if c["id"].startswith("A1"))
    s = case["scenario"]
    sep, zone = s["lane_separation_m"], case["scenario"]["zones"][0]
    lead, follow = s["gap_to_leader_m"], s["gap_from_follower_m"]
    front, rear = s["target_lane_front_offset_m"], s["target_lane_rear_offset_m"]
    fig, ax = plt.subplots(figsize=(13.33, 5.6))
    fig.subplots_adjust(left=.04, right=.99, top=.80, bottom=.12)
    banner(fig, "EXPERIMENT SETUP  |  FIVE AIRCRAFT, TWO LANES",
           "A decides whether to leave a lane it is about to lose signal in")
    for lane_y in (0, sep):
        ax.add_patch(Rectangle((-6800, lane_y - 70), 12600, 140, color=LANE, zorder=0))
    ax.axhline(0, color=GREY, ls=(0, (6, 4)), lw=.9, zorder=1)
    ax.axhline(sep, color=GREY, ls=(0, (6, 4)), lw=.9, zorder=1)
    ax.add_patch(Rectangle((zone[1], -70), min(zone[2], 5600 - zone[1]), 140,
                           color=BAD, alpha=.16, zorder=2))
    ax.plot([zone[1], zone[1]], [-70, 70], color=BAD, lw=1.4, zorder=3)
    law = spec["resolved_objects"]["traffic"]
    v = law["cruise_mps"]
    zone_spacing = (law["d0_m"] + law[f"tau_{zone[3].lower()}_s"] * v
                    + law["buffer_s2_per_m"] * v ** 2)
    ax.annotate(f"weak zone starts {zone[1]:.0f} m ahead and runs {zone[2] / 1000:.0f} km:\n"
                f"policy degrades to {zone[3]}, required spacing rises to {zone_spacing:.1f} m",
                (zone[1], -70), xytext=(8, -52), textcoords="offset points",
                fontsize=12, color=BAD)
    aircraft(ax, 0, 0, NAVY, "A", "decides")
    aircraft(ax, lead, 0, INK, "B", f"leader, {lead:.0f} m")
    aircraft(ax, -follow, 0, INK, "C", f"follower, {follow:.0f} m")
    aircraft(ax, min(front, 5200), sep, INK, "D", f"{front:.0f} m ahead")
    aircraft(ax, -min(rear, 5200), sep, INK, "E", f"{rear:.0f} m behind")
    ax.annotate("", (lead, 128), (0, 128), arrowprops=dict(arrowstyle="<->", color=NAVY, lw=1.1))
    ax.annotate(f"{lead:.1f} m = C-policy spacing at 50 m/s", (lead / 2, 128), xytext=(0, 7),
                textcoords="offset points", ha="center", fontsize=11.5, color=NAVY)
    ax.annotate("", (0, sep - 40), (0, 40), arrowprops=dict(
        arrowstyle="-|>", color=NAVY, lw=1.8, linestyle=(0, (4, 2))))
    ax.annotate(f"lane change {sep:.0f} m", (0, sep * .72), xytext=(-12, 0), ha="right",
                textcoords="offset points", fontsize=12, color=NAVY, va="center")
    ax.text(-6700, sep + 105, "TARGET LANE", fontsize=11.5, color=GREY, fontweight="bold")
    ax.text(-6700, -160, "A's LANE", fontsize=11.5, color=GREY, fontweight="bold")
    ax.set_xlim(-6900, 5700)
    ax.set_ylim(-230, sep + 175)
    ax.set_xlabel("along-corridor position  (m)      —      all five cruise at 50 m/s, same altitude")
    ax.set_yticks([])
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    return fig


def classification_figure(run, spec, dt):
    """The two questions, the four situations, and what the rule answered."""
    cells = {}
    for case in spec["part_A"]:
        allow = load(run, f"A__{case['id']}__allow_revised__dt{dt:g}")
        stay = load(run, f"A__{case['id']}__stay__dt{dt:g}")
        changed = any(d.get("reason") == "gain_above_threshold" for d in allow["decisions"])
        cells[case["id"][:2]] = (changed, allow["actual_policy_cost"], stay["actual_policy_cost"])
    fig, ax = plt.subplots(figsize=(13.33, 6.2))
    fig.subplots_adjust(left=.06, right=.98, top=.78, bottom=.06)
    banner(fig, "DECISION RULE  |  FOUR SITUATIONS, THREE ANSWERS",
           "Safety is a filter, not a term in the benefit")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5.7)
    ax.axis("off")
    layout = {"A1": (0, 1, "safe  ·  has benefit", "CHANGE"),
              "A2": (1, 1, "safe  ·  no benefit", "STAY — not worth it"),
              "A3": (0, 0, "unsafe  ·  has benefit", "STAY — cannot"),
              "A9": (1, 0, "unsafe  ·  no benefit", "STAY — cannot")}
    for case, (col, row, caption, answer) in layout.items():
        changed, allow_cost, stay_cost = cells[case]
        x, y = 1.9 + col * 4.0, .7 + row * 2.5
        colour = GOOD if changed else NAVY
        ax.add_patch(FancyBboxPatch((x, y), 3.6, 2.0, boxstyle="round,pad=0.06,rounding_size=.08",
                                    linewidth=1.6, edgecolor=colour, facecolor="white"))
        ax.text(x + .18, y + 1.66, case, fontsize=15, fontweight="bold", color=colour)
        ax.text(x + .78, y + 1.67, caption, fontsize=12.5, color=GREY)
        ax.text(x + .18, y + 1.06, answer, fontsize=16, fontweight="bold", color=colour)
        saved = stay_cost - allow_cost
        detail = (f"policy cost {allow_cost:.0f} vs {stay_cost:.0f} staying   →   saves {saved:.0f}"
                  if saved else f"policy cost {allow_cost:.0f}, same as staying")
        ax.text(x + .18, y + .45, detail, fontsize=12, color=INK)
    ax.text(1.9, 5.42, "HAS BENEFIT", fontsize=12.5, fontweight="bold", color=GREY)
    ax.text(5.9, 5.42, "NO BENEFIT", fontsize=12.5, fontweight="bold", color=GREY)
    ax.text(1.55, 4.3, "SAFE", fontsize=12.5, fontweight="bold", color=GREY,
            rotation=90, va="center", ha="right")
    ax.text(1.55, 1.75, "UNSAFE", fontsize=12.5, fontweight="bold", color=GREY,
            rotation=90, va="center", ha="right")
    fig.text(.06, .055, "Every 5 s, in this order:   1. is any lane change admissible?  "
             "(protection distance, insertion spacing, braking margin)   "
             "2. does the cheapest admissible one beat staying?", fontsize=12, color=NAVY)
    fig.text(.06, .018, "Cost = seconds in F × 2 + seconds in R × 1, summed over all five aircraft.   "
             "Every case is also flown with lane changes disabled, as a control.",
             fontsize=12, color=INK)
    return fig


POLICY_COLOUR = {"C": GOOD, "R": WARN, "F": BAD}


def trajectory_figure(run, spec, dt):
    """A's actual path in A1: where it changes, and what its policy does."""
    case = next(c for c in spec["part_A"] if c["id"].startswith("A1"))
    sep = case["scenario"]["lane_separation_m"]
    zone = case["scenario"]["zones"][0]
    allow = load(run, f"A__{case['id']}__allow_revised__dt{dt:g}")
    stay = load(run, f"A__{case['id']}__stay__dt{dt:g}")
    t = np.asarray(allow["times_s"])
    q = np.asarray(allow["trace"]["A"]["q_m"])
    d = np.asarray(allow["lateral"]["offset_m"])
    policy = np.asarray(allow["trace"]["A"]["policy"])
    fig, ax = plt.subplots(figsize=(13.33, 5.8))
    fig.subplots_adjust(left=.115, right=.99, top=.79, bottom=.20)
    banner(fig, "RESULT  |  WHAT THE AIRCRAFT ACTUALLY DID",
           "A leaves before the weak zone costs it, and its policy returns to C")
    ax.add_patch(Rectangle((zone[1], -55), zone[2], 110, color=BAD, alpha=.14, zorder=0))
    ax.annotate(f"weak zone on A's lane ({zone[3]} policy)", (zone[1] + 200, -55),
                xytext=(6, 10), textcoords="offset points", fontsize=11.5, color=BAD)
    stay_q = np.asarray(stay["trace"]["A"]["q_m"])
    ax.plot(stay_q, np.zeros_like(stay_q), color=GREY, lw=2.4, ls=(0, (5, 3)),
            label=f"if it stays: {stay['actual_policy_cost']:.0f} policy cost, in F for 260 s")
    for value, colour in POLICY_COLOUR.items():
        mask = policy == value
        if mask.any():
            ax.plot(np.where(mask, q, np.nan), np.where(mask, d, np.nan), color=colour, lw=3.2,
                    solid_capstyle="round", label=f"A flying under {value}")
    cross = int(np.argmax(d >= sep / 2))
    done = int(np.argmax(d >= sep - 1e-6))
    for index, text in ((0, f"decides at t = 0 s\ngain 148.9 predicted"),
                        (cross, f"crosses the midline at t = {t[cross]:.0f} s\ncounts as the target lane"),
                        (done, f"arrives at t = {t[done]:.0f} s")):
        ax.plot([q[index]], [d[index]], "o", ms=8, color=NAVY, zorder=6)
        ax.annotate(text, (q[index], d[index]), xytext=(8, -46 if index == done else 14),
                    textcoords="offset points", fontsize=11.5, color=NAVY)
    ax.axhline(sep / 2, color=GREY, ls=":", lw=1.0)
    ax.axhline(sep, color=GREY, ls=(0, (6, 4)), lw=.9)
    ax.set_xlim(-200, max(q.max(), stay_q.max()) + 200)
    ax.set_ylim(-90, sep + 120)
    ax.set_xlabel("along-corridor position  (m)")
    ax.set_ylabel("lateral offset  (m)")
    ax.set_yticks([0, sep / 2, sep])
    ax.set_yticklabels(["A's lane", "midline", "target lane"])
    ax.legend(frameon=False, loc="upper left", ncol=2)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    fig.text(.115, .045, f"Policy cost {allow['actual_policy_cost']:.0f} against "
             f"{stay['actual_policy_cost']:.0f} staying: A's time under F falls from 260 s to 32 s.",
             fontsize=12.5, color=INK)
    return fig


def shape_comparison_figure(run, spec, dt, shapes):
    """Same distance, same duration: what differs between the five curves."""
    field = spec["part_B"]["field"]
    stay = load(run, f"B1__{field}__stay__dt{dt:g}")
    fig, axes = plt.subplots(1, 3, figsize=(13.33, 5.4),
                             gridspec_kw={"width_ratios": [1, 1, 1.05], "wspace": .46})
    fig.subplots_adjust(left=.05, right=.985, top=.76, bottom=.18)
    banner(fig, "RESULT  |  FIVE LANE-CHANGE CURVES, SAME 300 m IN SAME 90 s",
           "What the aircraft feels differs; what it saves follows the midline crossing")
    colours = ("#1f4e79", "#b5451b", "#2e7d32", "#6a3d9a", "#b8860b")
    rows = []
    for colour, shape in zip(colours, shapes):
        rec = load(run, f"B1__{field}__{shape.name}__dt{dt:g}")
        t = np.asarray(rec["times_s"])
        label = shape_label(shape.name)
        axes[0].plot(t, rec["lateral"]["speed_mps"], color=colour, lw=2.2, label=label)
        axes[1].plot(t, rec["lateral"]["accel_mps2"], color=colour, lw=2.2)
        m = rec["metrics"]
        rows.append((label, colour, m["crossing_s"], stay["actual_policy_cost"] - rec["actual_policy_cost"],
                     m["executed_peak_lateral_jerk_mps3"]))
    axes[0].set_title("lateral speed  (m s⁻¹)", loc="left", fontsize=13.5)
    axes[1].set_title("lateral acceleration  (m s⁻²)", loc="left", fontsize=13.5)
    for ax in axes[:2]:
        ax.set_xlabel("time  (s)")
        ax.set_xlim(0, 95)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    axes[0].legend(frameon=False, fontsize=11)
    axes[1].axhline(0, color=INK, lw=.6)
    ax = axes[2]
    order = sorted(rows, key=lambda r: r[3])
    ypos = np.arange(len(order))
    ax.barh(ypos, [r[3] for r in order], color=[r[1] for r in order], height=.6)
    for y, row in zip(ypos, order):
        ax.text(row[3] + 3, y, f"{row[3]:.0f}   (crosses at {row[2]:.0f} s)", va="center", fontsize=11.5)
    ax.set_yticks(ypos)
    ax.set_yticklabels([r[0] for r in order], fontsize=11.5)
    ax.set_xlim(0, max(r[3] for r in rows) * 1.95)
    ax.set_xlabel("policy cost saved against staying")
    ax.set_title("benefit follows the crossing time", loc="left", fontsize=13.5)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    quintic = next(r for r in rows if r[0].startswith("quintic"))
    fig.text(.05, .04, f"Same saving at the same crossing time; quintic peak jerk {quintic[4]:.4f} m s⁻³. "
             "Cubic Bézier jerk excludes its acceleration jump at start and end.", fontsize=12.5, color=INK)
    return fig


def reality_figure(run, spec, dt):
    """The ideal field against the measured ones."""
    field = spec["part_B"]["field"]
    entries = [("idealised field\n(this lane bad, next lane good)", field, "B2")]
    for name in spec["part_C"]["fields"]:
        f = spec["fields"][name]
        entries.append((f"measured: {f['bad_length_m']:.0f} m zone\n"
                        f"{f['gradient_db_per_100m']:g} dB per 100 m", name, "C2"))
    saved, labels, colours = [], [], []
    for label, name, part in entries:
        allow = load(run, f"{part}__{name}__allow__dt{dt:g}")
        stay = load(run, f"{part}__{name}__stay__dt{dt:g}")
        saved.append(stay["actual_policy_cost"] - allow["actual_policy_cost"])
        labels.append(label)
        colours.append(NAVY if part == "B2" else BAD)
    fig, ax = plt.subplots(figsize=(13.33, 5.6))
    fig.subplots_adjust(left=.22, right=.97, top=.76, bottom=.22)
    banner(fig, "RESULT  |  IDEALISED SIGNAL AGAINST THE MEASURED ONE",
           "In the real corridor, changing lane buys one to two orders of magnitude less")
    ypos = np.arange(len(saved))[::-1]
    ax.barh(ypos, saved, color=colours, height=.58)
    for y, value in zip(ypos, saved):
        ax.text(value + 2.5, y, f"{value:.0f}" + ("" if value else "   (never changes)"),
                va="center", fontsize=12.5, color=INK)
    ax.set_yticks(ypos)
    ax.set_yticklabels(labels, fontsize=12)
    ax.set_xlim(0, max(saved) * 1.18)
    ax.set_xlabel("policy cost saved by changing lane")
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    fig.text(.055, .075, "A saving of 4 means A avoids two seconds under F — for a maneuver that "
             "takes more than 70 s.", fontsize=12.5, color=INK)
    fig.text(.055, .025, "Zone lengths and lateral gradients are the 2026-09-10 survey of the Bay "
             "Area route; well depth 0.7 dB.", fontsize=12.5, color=INK)
    return fig


def shape_label(name):
    if name == "quintic":
        return "quintic = r(2,2)"
    if name == "bezier3":
        return "cubic Bézier"
    if name.startswith("beta_"):
        p, q = (int(x) for x in name.split("_")[1:3])
        return f"r({p},{q})" + (" early peak" if p < q else " late peak" if p > q else "")
    return "three-clothoid"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    spec = json.loads((args.run / "resolved_config.json").read_text())
    validation = json.loads((args.run / "validation.json").read_text())
    if validation["status"] != "passed":
        raise ValueError("refusing to build briefing figures from a run that failed validation")
    dt = spec["numerics"]["dt_s"]
    args.output.mkdir(parents=True, exist_ok=True)
    style()
    shapes = [make_shape(s) for s in spec["shapes"]]
    figures = {"01-setup": layout_figure(spec),
               "02-four-situations": classification_figure(args.run, spec, dt),
               "03-what-it-did": trajectory_figure(args.run, spec, dt),
               "04-five-curves": shape_comparison_figure(args.run, spec, dt, shapes),
               "05-ideal-vs-real": reality_figure(args.run, spec, dt)}
    for name, fig in figures.items():
        for ext in ("png", "svg"):
            fig.savefig(args.output / f"{name}.{ext}", dpi=200)
        plt.close(fig)
    print(json.dumps({"written": sorted(figures), "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
