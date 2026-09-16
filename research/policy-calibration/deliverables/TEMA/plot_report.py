"""Create figures for the staged Bay Area C/R/F calibration report."""
from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np


plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Arial", "DejaVu Sans", "Liberation Sans"]
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams.update({
    "pdf.fonttype": 42,
    "font.size": 7.2,
    "axes.linewidth": 0.75,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
    "xtick.major.width": 0.7,
    "ytick.major.width": 0.7,
})

HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parents[1]
ROOT = HERE.parents[3]
RESULTS = CALIBRATION / "results"
OUTPUT = HERE / "figures"
POLICY_COLORS = {"C": "#356B9A", "R": "#D59A2D", "F": "#B64A45"}
STEP_COLORS = {1.0: "#2D8D88", 2.0: "#5B7FCA", 5.0: "#484878"}


def read_csv(name: str) -> list[dict[str, str]]:
    with (RESULTS / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def number(row: dict[str, str], key: str) -> float:
    return float(row[key]) if row[key] else np.nan


def select(rows, **conditions):
    return [row for row in rows if all(row[key] == str(value) for key, value in conditions.items())]


def panel_label(ax, label: str) -> None:
    ax.text(-0.12, 1.04, label, transform=ax.transAxes, fontweight="bold",
            fontsize=9, ha="left", va="bottom")


def finish(fig, stem: str) -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT / f"{stem}.svg", bbox_inches="tight")
    fig.savefig(OUTPUT / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(OUTPUT / f"{stem}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def policy_stack(ax, rows, title: str) -> None:
    rows = sorted(rows, key=lambda row: number(row, "threshold_db"))
    theta = np.asarray([number(row, "threshold_db") for row in rows])
    shares = [np.asarray([100 * number(row, f"{policy.lower()}_share") for row in rows])
              for policy in ("C", "R", "F")]
    ax.stackplot(theta, shares, colors=[POLICY_COLORS[p] for p in ("C", "R", "F")],
                 labels=("C", "R", "F"), alpha=0.94, linewidth=0)
    ax.axvline(-1.5, color="#333333", lw=0.8, ls=(0, (3, 2)))
    ax.axvline(-2.0, color="#111111", lw=1.05)
    ax.set(xlim=(-3.0, 0.0), ylim=(0, 100), xlabel="SINR threshold, $\\Theta$ (dB)",
           ylabel="Policy observations (%)", title=title)
    ax.set_yticks([0, 50, 100])


def figure_threshold_screen() -> None:
    rows = read_csv("adaptive_focal_curves.csv")
    fixed = [row for row in rows if int(row["group_size"]) == 5
             and number(row, "window_s") == 30.0 and int(row["persistence_k"]) == 1]
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.25), constrained_layout=True)
    for ax, dt, label in zip(axes.flat[:3], (1.0, 2.0, 5.0), "abc"):
        policy_stack(ax, select(fixed, dt_radio_s=dt),
                     f"Sampling {dt:g} s; W=30 s; group=5; k=1")
        panel_label(ax, label)
    axes[1, 0].set_ylabel("Policy observations (%)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    axes[0, 0].legend(handles, labels, ncol=3, loc="lower left", columnspacing=0.8,
                      handlelength=1.1)

    ax = axes[1, 1]
    for dt in (1.0, 2.0, 5.0):
        subset = sorted(select(fixed, dt_radio_s=dt), key=lambda row: number(row, "threshold_db"))
        x = np.asarray([number(row, "threshold_db") for row in subset])
        y = np.asarray([number(row, "mean_switches_per_sequence") for row in subset])
        ax.plot(x, y, color=STEP_COLORS[dt], lw=1.3, label=f"{dt:g} s")
    ax.axvline(-1.5, color="#333333", lw=0.8, ls=(0, (3, 2)))
    ax.axvline(-2.0, color="#111111", lw=1.05)
    ax.set(xlim=(-3.0, 0.0), xlabel="SINR threshold, $\\Theta$ (dB)",
           ylabel="Mean switches per flight", title="Switching under the same k=1 rule")
    ax.grid(axis="y", color="#DDDDDD", lw=0.5)
    ax.legend(title="Sampling", ncol=3, loc="upper left", fontsize=6.3, title_fontsize=6.3)
    panel_label(ax, "d")
    fig.suptitle("Stage 1 — Threshold sensitivity with exposure and persistence held fixed",
                 fontsize=9.5, fontweight="bold")
    finish(fig, "01-matched-threshold-screen")


def matrix(rows, key: str, thresholds: np.ndarray) -> np.ndarray:
    out = np.full((3, len(thresholds)), np.nan)
    for i, dt in enumerate((1.0, 2.0, 5.0)):
        for j, theta in enumerate(thresholds):
            row = next(row for row in rows if number(row, "dt_radio_s") == dt
                       and np.isclose(number(row, "threshold_db"), theta))
            out[i, j] = number(row, key)
    return out


def heatmap(ax, values, thresholds, title, label, *, cmap, vmin, vmax, fmt=None):
    image = ax.imshow(values, aspect="auto", origin="upper", cmap=cmap, vmin=vmin, vmax=vmax,
                      extent=(thresholds[0] - 0.05, thresholds[-1] + 0.05, 5.75, 0.25))
    ax.set_yticks([1, 3, 5], ["1", "2", "5"])
    ax.set_xticks([-2.5, -2.0, -1.5])
    ax.set_xlabel("SINR threshold, $\\Theta$ (dB)")
    ax.set_ylabel("Sampling interval (s)")
    ax.set_title(title)
    ax.axvline(-2.0, color="white", lw=1.1, ls=(0, (2, 2)))
    if fmt:
        for i, dt in enumerate((1.0, 2.0, 5.0)):
            y = (1, 3, 5)[i]
            for theta in (-2.5, -2.0, -1.5):
                j = int(round((theta - thresholds[0]) / 0.1))
                if not np.isfinite(values[i, j]):
                    continue
                ax.text(theta, y, fmt.format(values[i, j]), ha="center", va="center",
                        fontsize=6.1, color="white" if image.norm(values[i, j]) > 0.55 else "#202020")
    bar = ax.figure.colorbar(image, ax=ax, fraction=0.045, pad=0.025)
    bar.set_label(label)


def figure_interaction() -> None:
    rows = read_csv("adaptive_focal_curves.csv")
    rows = [row for row in rows if int(row["group_size"]) == 5
            and number(row, "window_s") == 30.0 and int(row["persistence_k"]) == 1]
    thresholds = np.round(np.arange(-2.5, -1.49, 0.1), 1)
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 4.55), constrained_layout=True)
    specifications = [
        ("c_share", "C share", "Policy observations (%)", "Blues", 0, 100, "{:.0f}%"),
        ("r_share", "R share", "Policy observations (%)", "YlOrBr", 0, 50, "{:.0f}%"),
        ("f_share", "F share", "Policy observations (%)", "Reds", 0, 50, "{:.0f}%"),
        ("aks_completion_before_next_switch_fraction", "AKS timing-compatible transitions",
         "Timing-compatible transitions (%)", "viridis", 0, 100, "{:.0f}%"),
    ]
    for label, ax, (key, title, cbar, cmap, vmin, vmax, fmt) in zip("abcd", axes.flat, specifications):
        heatmap(ax, 100 * matrix(rows, key, thresholds), thresholds, title, cbar,
                cmap=cmap, vmin=vmin, vmax=vmax, fmt=fmt)
        panel_label(ax, label)
    fig.suptitle("Two-variable view — threshold dominates; 1 s and 2 s sampling are similar",
                 fontsize=9.5, fontweight="bold")
    finish(fig, "02-threshold-sampling-interaction")


def stacked_bars(ax, categories, arrays, title):
    bottom = np.zeros(len(categories))
    for policy, values in zip(("C", "R", "F"), arrays):
        ax.bar(categories, values, bottom=bottom, color=POLICY_COLORS[policy], label=policy,
               width=0.62)
        bottom += values
    ax.set(ylim=(0, 100), ylabel="Policy observations (%)", title=title)
    ax.set_yticks([0, 50, 100])


def figure_group_size() -> None:
    rows = read_csv("adaptive_focal_curves.csv")
    selected = []
    for group in (3, 5, 7):
        selected.append(next(row for row in rows if number(row, "dt_radio_s") == 2.0
                             and int(row["group_size"]) == group
                             and number(row, "window_s") == 30.0
                             and int(row["persistence_k"]) == 1
                             and number(row, "threshold_db") == -2.0))
    groups = np.asarray([3, 5, 7])
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.55), constrained_layout=True)
    stacked_bars(axes[0], groups,
                 [[100 * number(row, f"{p.lower()}_share") for row in selected] for p in ("C", "R", "F")],
                 "Policy distribution")
    axes[0].legend(ncol=3, loc="lower center", fontsize=6.2, columnspacing=0.7)
    switches = [number(row, "mean_switches_per_sequence") for row in selected]
    axes[1].bar(groups, switches, color="#5B7FCA", width=0.62)
    axes[1].set(ylabel="Mean switches per flight", title="Classification switching", xticks=groups)
    action = [100 * number(row, "aks_completion_before_next_switch_fraction") for row in selected]
    axes[2].bar(groups, action, color="#2D8D88", width=0.62)
    axes[2].set(ylim=(0, 100), ylabel="Timing-compatible transitions (%)",
                title="AKS timing compatibility", xticks=groups)
    for ax in axes:
        ax.set_xlabel("Target group size")
        ax.grid(axis="y", color="#DDDDDD", lw=0.5, zorder=0)
    for ax, values, suffix in ((axes[1], switches, ""), (axes[2], action, "%")):
        for x, value in zip(groups, values):
            ax.text(x, value + (1.5 if suffix else 0.12), f"{value:.1f}{suffix}",
                    ha="center", va="bottom", fontsize=6.5)
    for label, ax in zip("abc", axes):
        panel_label(ax, label)
    fig.suptitle("Stage 2 preview — group-size screen after fixing provisional $\\Theta=-2.0$ dB",
                 fontsize=9.5, fontweight="bold")
    finish(fig, "03-group-size-preview")


def load_calibration_module():
    spec = importlib.util.spec_from_file_location("policy_calibration_run", CALIBRATION / "run.py")
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError("cannot load calibration module")
    spec.loader.exec_module(module)
    return module


def timeline(calibration, scenario, k: int):
    trace = calibration.fixed_trace(scenario, 2.0)
    exposure, valid, times = calibration.adaptive_exposure_for_threshold(trace, -2.0, 5, 30.0, 2.0)
    update = np.isclose(times / 5.0, np.round(times / 5.0), atol=1e-9)
    times, valid = times[update], valid[:, update]
    raw = calibration.raw_policy(exposure[:, update], 0.05, 0.10)
    output = np.full(raw.shape, -1, dtype=np.int8)
    for aircraft in range(raw.shape[0]):
        indices = np.flatnonzero(valid[aircraft])
        cuts = np.flatnonzero(np.diff(indices) > 1) + 1
        for block in np.split(indices, cuts):
            if len(block):
                output[aircraft, block] = calibration.persistent_sequence(raw[aircraft, block], k)
    return times, output


def figure_persistence() -> None:
    rows = read_csv("adaptive_focal_curves.csv")
    selected = [next(row for row in rows if number(row, "dt_radio_s") == 2.0
                     and int(row["group_size"]) == 5 and number(row, "window_s") == 30.0
                     and int(row["persistence_k"]) == k and number(row, "threshold_db") == -2.0)
                for k in (1, 2, 3)]
    calibration = load_calibration_module()
    sys.path.insert(0, str(ROOT / "src"))
    from capacity_policy import load_scenario
    scenario = load_scenario(ROOT / "scenarios/airport_to_airport/scenario.json")

    fig = plt.figure(figsize=(7.2, 5.7), constrained_layout=True)
    gs = fig.add_gridspec(4, 3, height_ratios=(1.05, 1, 1, 1))
    top = [fig.add_subplot(gs[0, i]) for i in range(3)]
    stacked_bars(top[0], np.asarray([1, 2, 3]),
                 [[100 * number(row, f"{p.lower()}_share") for row in selected] for p in ("C", "R", "F")],
                 "Policy distribution")
    top[0].set_xlabel("Persistence, k")
    top[0].legend(ncol=3, loc="lower center", fontsize=6.0, columnspacing=0.6)
    switch_values = [number(row, "mean_switches_per_sequence") for row in selected]
    top[1].bar([1, 2, 3], switch_values, color="#5B7FCA", width=0.62)
    top[1].set(xlabel="Persistence, k", ylabel="Mean switches per flight", title="Switch suppression",
               xticks=[1, 2, 3])
    action_values = [100 * number(row, "aks_completion_before_next_switch_fraction") for row in selected]
    top[2].bar([1, 2, 3], action_values, color="#2D8D88", width=0.62)
    top[2].set(xlabel="Persistence, k", ylabel="Timing-compatible transitions (%)",
               title="AKS timing compatibility", xticks=[1, 2, 3], ylim=(0, 100))
    for label, ax in zip("abc", top):
        panel_label(ax, label)
        ax.grid(axis="y", color="#DDDDDD", lw=0.5, zorder=0)
    for ax, values, suffix, offset in ((top[1], switch_values, "", 0.08),
                                       (top[2], action_values, "%", 1.5)):
        for x, value in zip((1, 2, 3), values):
            ax.text(x, value + offset, f"{value:.1f}{suffix}", ha="center", va="bottom",
                    fontsize=6.3)

    cmap = ListedColormap(["#FFFFFF", POLICY_COLORS["C"], POLICY_COLORS["R"], POLICY_COLORS["F"]])
    norm = BoundaryNorm([-1.5, -0.5, 0.5, 1.5, 2.5], cmap.N)
    time_axes = [fig.add_subplot(gs[i, :]) for i in range(1, 4)]
    source = {}
    for label, k, ax in zip("def", (1, 2, 3), time_axes):
        times, policy = timeline(calibration, scenario, k)
        source[f"time_k{k}_s"] = times
        source[f"policy_k{k}"] = policy
        ax.imshow(policy, aspect="auto", interpolation="nearest", cmap=cmap, norm=norm,
                  extent=(times[0] / 60, times[-1] / 60, policy.shape[0], 1))
        ax.set_ylabel("Aircraft")
        ax.set_yticks([1, 45, 90])
        ax.set_title(f"k={k}: switch after {k} consecutive policy assessment(s)",
                     loc="left", fontsize=7.2, fontweight="bold")
        panel_label(ax, label)
    time_axes[-1].set_xlabel("Simulation time (min)")
    handles = [mpl.patches.Patch(color=POLICY_COLORS[p], label=p) for p in ("C", "R", "F")]
    time_axes[0].legend(handles=handles, ncol=3, loc="upper right", fontsize=6.2,
                        columnspacing=0.8, handlelength=1.1)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUTPUT / "04-persistence-timeline-source.npz", **source)
    fig.suptitle("Stage 3 — Persistence is a separate anti-chatter rule, not part of threshold calibration",
                 fontsize=9.5, fontweight="bold")
    finish(fig, "04-persistence-separated")


def write_source_data() -> None:
    rows = read_csv("adaptive_focal_curves.csv")
    selected = [row for row in rows if number(row, "window_s") == 30.0
                and number(row, "threshold_db") >= -3.0 and number(row, "threshold_db") <= 0.0
                and ((int(row["group_size"]) == 5 and int(row["persistence_k"]) in (1, 2, 3))
                     or (number(row, "dt_radio_s") == 2.0 and number(row, "threshold_db") == -2.0))]
    with (OUTPUT / "source-data.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected[0]))
        writer.writeheader()
        writer.writerows(selected)


def main() -> None:
    figure_threshold_screen()
    figure_interaction()
    figure_group_size()
    figure_persistence()
    write_source_data()


if __name__ == "__main__":
    main()
