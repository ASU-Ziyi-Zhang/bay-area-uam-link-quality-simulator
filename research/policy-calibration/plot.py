"""Create publication-ready figures for the Bay Area policy parameter screen."""
from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.patches import Rectangle
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
ROOT = HERE.parents[1]
RESULTS = HERE / "results"
OUTPUT = HERE / "figures"
POLICY_COLORS = {"C": "#356B9A", "R": "#D59A2D", "F": "#B64A45"}
CONFIG_COLORS = {
    "5 s, k=1": "#484878",
    "5 s, k=3": "#8E8EB6",
    "1 s, k=1": "#2D8D88",
    "1 s, k=3": "#B64342",
}


def read_csv(name: str) -> list[dict[str, str]]:
    with (RESULTS / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def number(row: dict[str, str], key: str) -> float:
    return float(row[key]) if row[key] != "" else np.nan


def select(rows, **conditions):
    return [row for row in rows if all(row[key] == str(value) for key, value in conditions.items())]


def panel_label(ax, label: str) -> None:
    ax.text(-0.11, 1.04, label, transform=ax.transAxes, fontweight="bold",
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
    values = [np.asarray([number(row, f"{name.lower()}_share") for row in rows])
              for name in ("C", "R", "F")]
    ax.stackplot(theta, values, colors=[POLICY_COLORS[name] for name in ("C", "R", "F")],
                 labels=("C", "R", "F"), alpha=0.92, linewidth=0)
    ax.axvline(-1.5, color="#272727", lw=0.9, ls="--")
    ax.text(-1.47, 0.985, "historical $\\Theta$", fontsize=6.3, ha="left", va="top")
    ax.set(xlim=(-4, 1), ylim=(0, 1), xlabel="SINR threshold, $\\Theta$ (dB)",
           ylabel="Policy share", title=title)
    ax.set_yticks([0, 0.5, 1])
    ax.set_yticklabels(["0", "0.5", "1.0"])


def grid_matrix(rows, metric: str, windows=(15.0, 30.0, 60.0, 120.0), groups=(3, 5, 7)):
    matrix = np.full((len(groups), len(windows)), np.nan)
    for i, group in enumerate(groups):
        for j, window in enumerate(windows):
            match = select(rows, group_size=group, window_s=window)
            if len(match) != 1:
                raise RuntimeError(f"expected one row for group={group}, window={window}")
            matrix[i, j] = number(match[0], metric)
    return matrix


def annotated_heatmap(ax, matrix, xlabels, ylabels, *, cmap, label, fmt, vmin=None, vmax=None):
    image = ax.imshow(matrix, aspect="auto", cmap=cmap, vmin=vmin, vmax=vmax)
    ax.set_xticks(np.arange(len(xlabels)), xlabels)
    ax.set_yticks(np.arange(len(ylabels)), ylabels)
    ax.set_xlabel("Assessment window (s)")
    ax.set_ylabel("Target group size")
    for (i, j), value in np.ndenumerate(matrix):
        rgba = image.cmap(image.norm(value))
        luminance = 0.299 * rgba[0] + 0.587 * rgba[1] + 0.114 * rgba[2]
        ax.text(j, i, fmt.format(value), ha="center", va="center",
                color="white" if luminance < 0.48 else "#272727", fontsize=6.4)
    bar = ax.figure.colorbar(image, ax=ax, fraction=0.045, pad=0.025)
    bar.set_label(label)
    return image


def figure_parameter_overview() -> None:
    curves = read_csv("adaptive_focal_curves.csv")
    summary = read_csv("adaptive_focal_summary.csv")
    legacy = select(curves, dt_radio_s=5.0, group_size=5, window_s=30.0, persistence_k=1)
    candidate = select(curves, dt_radio_s=1.0, group_size=5, window_s=30.0, persistence_k=3)
    grid_rows = select(summary, dt_radio_s=1.0, persistence_k=3)

    fig = plt.figure(figsize=(7.2, 5.35), constrained_layout=True)
    gs = fig.add_gridspec(2, 2, height_ratios=(1.0, 1.08))
    axes = [fig.add_subplot(gs[i, j]) for i in range(2) for j in range(2)]
    policy_stack(axes[0], legacy, "Legacy: 5 s sampling, 30 s window, k=1")
    policy_stack(axes[1], candidate, "Candidate: 1 s sampling, 30 s window, k=3")
    axes[1].set_ylabel("")
    handles, labels = axes[0].get_legend_handles_labels()
    axes[1].legend(handles, labels, ncol=3, loc="lower right", handlelength=1.2,
                   columnspacing=0.9)
    switch_matrix = grid_matrix(grid_rows, "nominal_mean_switches_per_sequence")
    action_matrix = 100 * grid_matrix(grid_rows, "nominal_aks_completion_before_next_switch_fraction")
    annotated_heatmap(axes[2], switch_matrix, (15, 30, 60, 120), (3, 5, 7),
                      cmap="magma", label="Mean switches per flight", fmt="{:.1f}")
    annotated_heatmap(axes[3], action_matrix, (15, 30, 60, 120), (3, 5, 7),
                      cmap="viridis", label="AKS-actionable transitions (%)", fmt="{:.0f}",
                      vmin=0, vmax=100)
    axes[2].set_title("Switching at $\\Theta=-1.5$ dB (1 s, k=3)")
    axes[3].set_title("Nominal AKS completion before next switch")
    axes[3].set_ylabel("")
    for label, ax in zip("abcd", axes):
        panel_label(ax, label)
    fig.suptitle("Observation settings reshape the Bay Area policy field", fontsize=9.5, fontweight="bold")
    finish(fig, "01-parameter-overview")


def integer_matrix(rows, metric: str):
    bcs, brs = range(0, 5), range(1, 6)
    matrix = np.full((5, 5), np.nan)
    for row in rows:
        bc, br = int(row["allowed_bad_c"]), int(row["allowed_bad_r"])
        if bc in bcs and br in brs:
            matrix[bc, br - 1] = number(row, metric)
    return matrix


def triangular_heatmap(ax, matrix, title, cbar_label, cmap, fmt, vmin=None, vmax=None):
    masked = np.ma.masked_invalid(matrix)
    colour = mpl.colormaps[cmap].copy()
    colour.set_bad("#F2F2F2")
    image = ax.imshow(masked, origin="lower", aspect="equal", cmap=colour, vmin=vmin, vmax=vmax)
    ax.set_xticks(np.arange(5), np.arange(1, 6))
    ax.set_yticks(np.arange(5), np.arange(0, 5))
    ax.set_xlabel("Allowed bad observations for R ($b_R$)")
    ax.set_ylabel("Allowed bad observations for C ($b_C$)")
    ax.set_title(title)
    for (i, j), value in np.ndenumerate(matrix):
        if not np.isfinite(value):
            continue
        rgba = image.cmap(image.norm(value))
        lum = 0.299 * rgba[0] + 0.587 * rgba[1] + 0.114 * rgba[2]
        ax.text(j, i, fmt.format(value), ha="center", va="center", fontsize=6.2,
                color="white" if lum < 0.48 else "#272727")
    ax.add_patch(Rectangle((2 - 0.48, 1 - 0.48), 0.96, 0.96, fill=False,
                           edgecolor="#111111", linewidth=1.5))
    bar = ax.figure.colorbar(image, ax=ax, fraction=0.046, pad=0.025)
    bar.set_label(cbar_label)


def figure_exposure_budget() -> None:
    rows = select(read_csv("integer_budget_summary.csv"), persistence_k=1)
    fshare = 100 * integer_matrix(rows, "nominal_f_share")
    tc = integer_matrix(rows, "t_c_db")
    tr = integer_matrix(rows, "t_r_db")
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.55), constrained_layout=True)
    triangular_heatmap(axes[0], fshare, "F share at $\\Theta=-1.5$ dB",
                       "F policy (%)", "magma", "{:.0f}", 0, 100)
    triangular_heatmap(axes[1], tc, "Coordinated boundary", "$T_C$ (dB)",
                       "viridis", "{:.1f}", -2.8, -1.8)
    triangular_heatmap(axes[2], tr, "At-least-reactive boundary", "$T_R$ (dB)",
                       "viridis", "{:.1f}", -2.8, -1.8)
    for label, ax in zip("abc", axes):
        ax.text(-0.18, 1.07, label, transform=ax.transAxes, fontweight="bold",
                fontsize=9, ha="left", va="bottom")
    fig.suptitle("One-observation changes move policy and capability boundaries (N=35)",
                 fontsize=9.5, fontweight="bold")
    finish(fig, "02-integer-exposure-budget")


def grouped_noise(rows, dt: str, k: str, key: str):
    result = []
    for sigma in (0.0, 0.5, 1.0):
        values = [number(row, key) for row in rows
                  if row["dt_radio_s"] == dt and row["persistence_k"] == k
                  and float(row["sigma_db"]) == sigma]
        result.append((np.mean(values), np.percentile(values, 5), np.percentile(values, 95)))
    return np.asarray(result)


def noise_line(ax, rows, key, ylabel, scale=1.0):
    sigma = np.asarray([0.0, 0.5, 1.0])
    for dt, k in (("5.0", "1"), ("5.0", "3"), ("1.0", "1"), ("1.0", "3")):
        label = f"{dt[:-2]} s, k={k}"
        values = grouped_noise(rows, dt, k, key) * scale
        ax.plot(sigma, values[:, 0], marker="o", ms=3.2, lw=1.25,
                color=CONFIG_COLORS[label], label=label)
        ax.fill_between(sigma, values[:, 1], values[:, 2], color=CONFIG_COLORS[label], alpha=0.10)
    ax.set(xlabel="Illustrative SINR error, $\\sigma$ (dB)", ylabel=ylabel, xticks=sigma)
    ax.grid(axis="y", color="#DDDDDD", lw=0.5)


def figure_noise() -> None:
    rows = read_csv("iid_noise_seed_summary.csv")
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.0), constrained_layout=True)
    noise_line(axes[0, 0], rows, "nominal_f_share", "F policy (%)", 100)
    noise_line(axes[0, 1], rows, "nominal_mean_switches_per_sequence", "Mean switches per complete group")
    noise_line(axes[1, 0], rows, "nominal_aks_completion_before_next_switch_fraction",
               "AKS-actionable transitions (%)", 100)

    ax = axes[1, 1]
    sigma = np.asarray([0.0, 0.5, 1.0])
    offsets = np.linspace(-0.15, 0.15, 4)
    for offset, (dt, k) in zip(offsets, (("5.0", "1"), ("5.0", "3"), ("1.0", "1"), ("1.0", "3"))):
        label = f"{dt[:-2]} s, k={k}"
        tc = grouped_noise(rows, dt, k, "t_c_db")[:, 0]
        tr = grouped_noise(rows, dt, k, "t_r_db")[:, 0]
        for x, lo, hi in zip(sigma + offset, tc, tr):
            ax.plot([x, x], [lo, hi], color=CONFIG_COLORS[label], lw=1.5)
        ax.scatter(sigma + offset, tc, s=15, marker="o", color=CONFIG_COLORS[label])
        ax.scatter(sigma + offset, tr, s=17, marker="s", color=CONFIG_COLORS[label], label=label)
    ax.set(xlabel="Illustrative SINR error, $\\sigma$ (dB)", ylabel="Capability boundary (dB)",
           xticks=sigma)
    ax.grid(axis="y", color="#DDDDDD", lw=0.5)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    axes[0, 0].legend(handles, labels, ncol=2, fontsize=6.2, loc="upper left")
    ax.text(0.02, 0.04, "circle: $T_C$   square: $T_R$", transform=ax.transAxes, fontsize=6.2)
    for label, axis in zip("abcd", axes.flat):
        panel_label(axis, label)
    fig.suptitle("Receiver-scale uncertainty overwhelms fine threshold differences",
                 fontsize=9.5, fontweight="bold")
    finish(fig, "03-noise-resolution-stress")


def load_calibration_module():
    spec = importlib.util.spec_from_file_location("policy_calibration_run", HERE / "run.py")
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError("cannot load calibration module")
    spec.loader.exec_module(module)
    return module


def timeline(calibration, scenario, *, dt_s, window_s, k):
    trace = calibration.fixed_trace(scenario, dt_s)
    exposure, valid, times = calibration.adaptive_exposure_for_threshold(
        trace, -1.5, 5, window_s, dt_s
    )
    update = np.isclose(times / 5.0, np.round(times / 5.0), atol=1e-9)
    times = times[update]
    valid = valid[:, update]
    raw = calibration.raw_policy(exposure[:, update], 0.05, 0.10)
    matrix = np.full(raw.shape, -1, dtype=np.int8)
    for aircraft in range(raw.shape[0]):
        indices = np.flatnonzero(valid[aircraft])
        cuts = np.flatnonzero(np.diff(indices) > 1) + 1
        for block in np.split(indices, cuts):
            if len(block):
                matrix[aircraft, block] = calibration.persistent_sequence(raw[aircraft, block], k)
    return times, matrix


def figure_timelines() -> None:
    calibration = load_calibration_module()
    from capacity_policy import load_scenario
    scenario = load_scenario(ROOT / "scenarios/airport_to_airport/scenario.json")
    configs = [
        ("Legacy: 5 s sampling, 30 s window, k=1", 5.0, 30.0, 1),
        ("Minimum change: 1 s sampling, 30 s window, k=3", 1.0, 30.0, 3),
        ("Stability: 1 s sampling, 60 s window, k=3", 1.0, 60.0, 3),
    ]
    cmap = ListedColormap(["#FFFFFF", POLICY_COLORS["C"], POLICY_COLORS["R"], POLICY_COLORS["F"]])
    norm = BoundaryNorm([-1.5, -0.5, 0.5, 1.5, 2.5], cmap.N)
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 5.35), sharex=True, constrained_layout=True)
    stored = {}
    for label, ax, (title, dt_s, window_s, k) in zip("abc", axes, configs):
        times, matrix = timeline(calibration, scenario, dt_s=dt_s, window_s=window_s, k=k)
        stored[f"time_{label}_s"] = times
        stored[f"policy_{label}"] = matrix
        ax.imshow(matrix, aspect="auto", interpolation="nearest", cmap=cmap, norm=norm,
                  extent=(times[0] / 60, times[-1] / 60, matrix.shape[0], 1))
        ax.set_ylabel("Aircraft")
        ax.set_yticks([1, 30, 60, 90])
        ax.set_title(title, loc="left", fontsize=7.5, fontweight="bold")
        panel_label(ax, label)
    axes[-1].set_xlabel("Simulation time (min)")
    handles = [mpl.patches.Patch(color=POLICY_COLORS[name], label=name) for name in ("C", "R", "F")]
    axes[0].legend(handles=handles, ncol=3, loc="upper right", columnspacing=0.8,
                   handlelength=1.2, borderpad=0.2)
    fig.suptitle("Policy histories become more persistent after finer observation and confirmation",
                 fontsize=9.5, fontweight="bold")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUTPUT / "04-policy-timeline-source.npz", **stored)
    finish(fig, "04-policy-timelines")


def write_captions() -> None:
    text = """# Figure captions

## Figure 1 — Parameter overview

Fixed Bay Area geographic route and traffic, with no longitudinal feedback or lane change. Panels a–b show adaptive-focal C/R/F shares over the tested SINR threshold range. The dashed line marks the historical -1.5 dB setting. Panels c–d use 1 s SINR sampling, 5 s policy updates, k=3 persistence and Theta=-1.5 dB. Actionability is the fraction of evaluable policy transitions whose observed dwell lasts at least the corresponding nominal R0036 AKS settling time.

## Figure 2 — Integer exposure budget

Complete five-aircraft groups, 30 s window, 5 s sampling and k=1. Each cell is one valid pair b_C<b_R among 35 member-time observations. The black outline marks the historical (1,3) budget. Blank cells violate the ordered budget. Capability boundaries use a 95% policy-share requirement and the tested 0.1 dB threshold grid.

## Figure 3 — Noise-resolution stress

Complete five-aircraft groups, 30 s window and historical 5%/10% exposure tolerances. Nonzero sigma conditions show means and 5th–95th percentiles across 20 iid Gaussian seeds; sigma=0 is deterministic. This is an illustrative stress test, not receiver calibration. It excludes quantization, temporal correlation and slow bias.

## Figure 4 — Policy timelines

Adaptive-focal C/R/F policy for all 93 fixed-trajectory aircraft at Theta=-1.5 dB. White denotes time outside an aircraft's active route interval. SINR sampling is 5 or 1 s as labeled; policy updates remain fixed at 5 s in every panel. These traces visualize classification persistence only and do not include longitudinal or lateral feedback.
"""
    (OUTPUT / "captions.md").write_text(text, encoding="utf-8")


def main() -> None:
    figure_parameter_overview()
    figure_exposure_budget()
    figure_noise()
    figure_timelines()
    write_captions()


if __name__ == "__main__":
    main()
