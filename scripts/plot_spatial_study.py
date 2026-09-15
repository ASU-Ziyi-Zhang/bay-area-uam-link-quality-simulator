"""Deterministic academic Stage-2 figures from an audited spatial run."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter, MaxNLocator
import matplotlib.patheffects as pe
import numpy as np

from verify_lateral_study import read
from plot_wide_radio_map import sha, chord_frame


CASES = ["fixed", "lateral", "vertical", "joint"]
LABELS = ["Fixed", "Lateral only", "Vertical only", "Joint"]
COLORS = ["#686868", "#28658B", "#198577", "#F2BD3C"]
STYLES = ["--", "-", "-.", (0, (6, 3))]
POLICY = {"C": "#198577", "R": "#E7AF3C", "F": "#B64C42"}
INK = "#202020"


def plot(run, output, audit_path):
    run, output, audit_path = Path(run).resolve(), Path(output).resolve(), Path(audit_path).resolve()
    if output.exists():
        raise FileExistsError("choose a new figure revision directory")
    manifest = read(run/"manifest.json"); audit = read(audit_path)
    if manifest["status"] != "completed" or audit["status"] != "passed" or audit["run_manifest_sha256"] != sha(run/"manifest.json"):
        raise ValueError("a matching passed audit is required")
    for e in manifest["outputs"]:
        if sha(run/e["path"]) != e["sha256"]:
            raise ValueError("archived output changed")
    spec = read(run/"resolved_config.json"); cfg = spec["parameters"]
    summaries = {r["case_id"]: r for r in read(run/"summary.json")}
    traces = {name: read(run/name/"trace.json.gz")+[read(run/name/"terminal.json")] for name in CASES}
    obs = {name: read(run/name/"observations.json.gz") for name in CASES}
    event_log = {name: read(run/name/"events.json") for name in CASES}
    events = {name: [e for e in event_log[name] if e["status"] == "transition_accepted"] for name in CASES}
    with np.load(run/"field.npz") as archive:
        field = {k: archive[k].copy() for k in archive.files}
    plt.rcParams.update({"font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 12,
        "axes.labelsize": 13, "axes.titlesize": 14, "text.color": INK, "axes.labelcolor": INK,
        "axes.edgecolor": INK, "xtick.color": INK, "ytick.color": INK, "axes.linewidth": .7,
        "svg.fonttype": "none", "svg.hashsalt": "uam-spatial-stage2-v1",
        "figure.facecolor": "white", "savefig.facecolor": "white"})
    output.mkdir(parents=True)
    captions = {}
    def save(fig, name, caption):
        for ext in ["png", "svg"]:
            fig.savefig(output/f"{name}.{ext}", dpi=300, bbox_inches="tight", pad_inches=.12,
                        metadata={"Description": caption})
        captions[name] = caption
        plt.close(fig)

    # The progress plots below are useful for auditing, but q is not a map
    # coordinate.  Build one geography-aware figure from the same archived
    # spline and Cartesian traces so that the decision location, base-station
    # inventory, and before/after state can be read together.
    from verify_lateral_study import independent_reference
    with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
        reference_frame = independent_reference(archive, manifest, cfg["center_control_step_m"])
    q_grid = np.linspace(0., float(field["q_m"][-1]), 1800)
    route_xy = reference_frame(q_grid)[0]
    start_xy = route_xy[0]
    def relative_xy(points):
        return (np.asarray(points)[..., :2] - start_xy) / 1000.
    stations_xy = np.array([[s["x_m"], s["y_m"]] for s in spec["base_stations"]])
    accepted_joint = events["joint"][0] if events["joint"] else None
    completion_events = {}
    for name in CASES:
        completion_events[name] = next((e for e in event_log[name]
                                        if e["status"] == "transition_completed"), None)
    fixed_f_start = next((row["t_s"] for row in obs["fixed"] if row["policy"] == "F"), None)
    if accepted_joint:
        decision_time = float(accepted_joint["t_s"])
        event_times = [decision_time] + [float(e["t_s"]) for e in completion_events.values() if e]
        t_lo = max(0., min(event_times + ([float(fixed_f_start)] if fixed_f_start is not None else [])) - 60.)
        t_hi = max(event_times) + 60.
    else:
        decision_time, t_lo, t_hi = None, 0., 600.

    # (00) Geographic overview.  Panel (a) establishes the full route and
    # all 12 BS locations; panel (b) enlarges the first decision neighborhood;
    # panels (c)-(d) tie the map markers to state and policy before/after.
    fig = plt.figure(figsize=(16, 11))
    grid = fig.add_gridspec(2, 2, left=.055, right=.975, top=.925, bottom=.07,
                            hspace=.34, wspace=.23)
    full_ax, local_ax = fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1])
    state_ax, policy_ax = fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])
    route_rel = relative_xy(route_xy)
    full_ax.plot(route_rel[:, 0], route_rel[:, 1], color="#A6A6A6", lw=1.0, ls=(0, (2, 2)),
                 label="Reference route")
    full_handles = []
    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        xy = relative_xy([row["position_m"] for row in traces[name]])
        line, = full_ax.plot(xy[:, 0], xy[:, 1], color=color, lw=1.7, ls=style, label=label,
                             path_effects=[pe.Stroke(linewidth=2.7, foreground=INK), pe.Normal()] if name == "joint" else [])
        full_handles.append(line)
    bs_rel = relative_xy(stations_xy)
    full_ax.scatter(bs_rel[:, 0], bs_rel[:, 1], marker="^", s=34, color="#4D6475",
                    edgecolor="white", linewidth=.6, zorder=6, label="Base station")
    for station, point in zip(spec["base_stations"], bs_rel):
        full_ax.annotate(station["site_id"], point, xytext=(3, 3), textcoords="offset points",
                         fontsize=7.5, color="#26343D", zorder=7)
    full_ax.scatter([0], [0], marker="o", s=42, color=INK, zorder=8)
    full_ax.annotate("Departure", (0, 0), xytext=(5, -12), textcoords="offset points", fontsize=9)
    full_ax.annotate("Destination", route_rel[-1], xytext=(-8, 8), textcoords="offset points",
                     ha="right", fontsize=9)
    if accepted_joint:
        a_xy = relative_xy([next(row["position_m"] for row in traces["joint"]
                                 if abs(row["t_s"] - accepted_joint["t_s"]) < 1e-9)])[0]
        full_ax.scatter(*a_xy, marker="*", s=100, color="#111111", edgecolor="white", linewidth=.7,
                        zorder=10)
        full_ax.annotate("A  decision\nF observed", a_xy, xytext=(8, 8), textcoords="offset points",
                         fontsize=8.5, ha="left", va="bottom",
                         bbox=dict(boxstyle="round,pad=.25", fc="white", ec="#777777", alpha=.9))
    full_ax.set_title("(a) Full route context and base-station inventory", loc="left", pad=8)
    full_ax.set_xlabel("Relative easting from departure (km)")
    full_ax.set_ylabel("Relative northing from departure (km)")
    full_ax.set_aspect("equal", adjustable="datalim")
    full_ax.grid(color="#E5E5E5", lw=.45)
    full_ax.legend(handles=full_handles + [Line2D([], [], marker="^", color="none", mfc="#4D6475",
                                                    mec="white", label="Base station")],
                   loc="lower left", frameon=True, framealpha=.92, fontsize=8.5, ncol=2)

    # Local window is data-driven by the accepted event and completion; no
    # visual move is fabricated for the vertical-only case, whose ground
    # projection is exactly the reference route.
    if accepted_joint:
        q_a = float(accepted_joint["q_m"])
        q_b = max(float(e["q_m"]) for e in completion_events.values() if e)
        q_pad = 1800.
        q_mask = (q_grid >= max(0., q_a-q_pad)) & (q_grid <= min(float(field["q_m"][-1]), q_b+q_pad))
        local_points = [route_rel[q_mask]]
        for name in CASES:
            qv = np.array([row["q_m"] for row in traces[name]])
            mask = (qv >= q_a-q_pad) & (qv <= q_b+q_pad)
            local_points.append(relative_xy([row["position_m"] for row, keep in zip(traces[name], mask) if keep]))
        local_xy = np.vstack(local_points)
        xlo, ylo = local_xy.min(axis=0)-.18
        xhi, yhi = local_xy.max(axis=0)+.18
        bs_local = bs_rel[(bs_rel[:, 0] >= xlo-.4) & (bs_rel[:, 0] <= xhi+.4) &
                          (bs_rel[:, 1] >= ylo-.4) & (bs_rel[:, 1] <= yhi+.4)]
        local_ax.plot(route_rel[q_mask, 0], route_rel[q_mask, 1], color="#A6A6A6", lw=1.0,
                      ls=(0, (2, 2)), label="Reference route")
        for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
            qv = np.array([row["q_m"] for row in traces[name]])
            mask = (qv >= q_a-q_pad) & (qv <= q_b+q_pad)
            xy = relative_xy([row["position_m"] for row, keep in zip(traces[name], mask) if keep])
            local_ax.plot(xy[:, 0], xy[:, 1], color=color, lw=2.0, ls=style,
                          path_effects=[pe.Stroke(linewidth=3.0, foreground=INK), pe.Normal()] if name == "joint" else [])
        if len(bs_local):
            local_ax.scatter(bs_local[:, 0], bs_local[:, 1], marker="^", s=54, color="#4D6475",
                             edgecolor="white", linewidth=.7, zorder=6)
            for station, point in zip(spec["base_stations"], bs_rel):
                if any(np.allclose(point, candidate) for candidate in bs_local):
                    local_ax.annotate(station["site_id"], point, xytext=(4, 4), textcoords="offset points",
                                      fontsize=8, color="#26343D", zorder=7)
        local_ax.scatter(*a_xy, marker="*", s=120, color="#111111", edgecolor="white", linewidth=.8,
                         zorder=10, label="A: planner decision")
        for name, marker, text in [("lateral", "s", "B-L/J: +100 m lateral"),
                                   ("vertical", "D", "B-V: −200 m vertical")]:
            completion = completion_events[name]
            point = relative_xy([next(row["position_m"] for row in traces[name]
                                      if abs(row["t_s"] - completion["t_s"]) < 1e-8)])[0]
            local_ax.scatter(*point, marker=marker, s=65, color=COLORS[CASES.index(name)],
                             edgecolor=INK, linewidth=.7, zorder=10)
            local_ax.annotate(text, point, xytext=(7, -11 if name == "vertical" else 8),
                              textcoords="offset points", fontsize=8.5,
                              va="top" if name == "vertical" else "bottom")
        local_ax.set_xlim(xlo, xhi); local_ax.set_ylim(ylo, yhi)
        local_ax.set_aspect("equal", adjustable="box")
        local_ax.text(.02, .96, f"A: t = {decision_time:.0f} s\nB: exact maneuver endpoints",
                      transform=local_ax.transAxes, va="top", fontsize=8.5,
                      bbox=dict(boxstyle="round,pad=.25", fc="white", ec="#777777", alpha=.9))
    else:
        local_ax.text(.5, .5, "No accepted maneuver in archived run", ha="center", va="center")
    local_ax.set_title("(b) Enlarged first-decision neighborhood", loc="left", pad=8)
    local_ax.set_xlabel("Relative easting from departure (km)")
    local_ax.set_ylabel("Relative northing from departure (km)")
    local_ax.grid(color="#E5E5E5", lw=.45)

    time_grid = np.array([row["t_s"] for row in traces["fixed"]])
    state_lines = []
    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        rows = [r for r in traces[name] if t_lo <= r["t_s"] <= t_hi]
        line, = state_ax.plot([r["t_s"] for r in rows], [r["offset_m"] for r in rows], color=color,
                              lw=1.7, ls=style, label=label)
        state_lines.append(line)
    height_ax = state_ax.twinx()
    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        rows = [r for r in traces[name] if t_lo <= r["t_s"] <= t_hi]
        height_ax.plot([r["t_s"] for r in rows], [r["altitude_m"] for r in rows], color=color,
                       lw=1.1, ls=":", alpha=.85)
    state_ax.set_title("(c) Physical state around the communication decision", loc="left", pad=8)
    state_ax.set_xlabel("Time since departure (s)"); state_ax.set_ylabel("Lateral offset, d (m)")
    height_ax.set_ylabel("Altitude, h (m)")
    state_ax.set_xlim(t_lo, t_hi); state_ax.grid(axis="y", color="#E5E5E5", lw=.5)
    state_ax.legend(handles=state_lines + [Line2D([], [], color=INK, lw=1.7, ls="-", label="Offset d"),
                                           Line2D([], [], color=INK, lw=1.1, ls=":", label="Height h")],
                    loc="upper left", frameon=False, fontsize=8.5, ncol=2)

    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        rows = [r for r in obs[name] if t_lo <= r["t_s"] <= t_hi]
        for i, row in enumerate(rows):
            end = rows[i+1]["t_s"] if i+1 < len(rows) else min(t_hi, row["t_s"]+cfg["policy_s"])
            policy_ax.broken_barh([(row["t_s"], max(0., end-row["t_s"]))],
                                  (CASES.index(name)-.31, .62), facecolors=POLICY[row["policy"]],
                                  edgecolors="none", rasterized=True)
    policy_ax.set_title("(d) Policy state before and after the decision", loc="left", pad=8)
    policy_ax.set_xlabel("Time since departure (s)"); policy_ax.set_yticks(range(4)); policy_ax.set_yticklabels(LABELS)
    policy_ax.set_xlim(t_lo, t_hi); policy_ax.set_ylim(3.5, -.5); policy_ax.grid(axis="x", color="#E5E5E5", lw=.5)
    policy_ax.legend(handles=[Patch(color=POLICY[p], label=p) for p in "CRF"], loc="upper left",
                     frameon=False, ncol=3, fontsize=8.5)
    if decision_time is not None:
        for ax in [state_ax, height_ax, policy_ax]:
            ax.axvline(decision_time, color=INK, lw=.8, ls="--", alpha=.8)
        state_ax.annotate("A: planner decision", (decision_time, state_ax.get_ylim()[1]),
                          xytext=(4, -4), textcoords="offset points", fontsize=8, va="top")
        for name, completion in completion_events.items():
            if completion:
                policy_ax.axvline(completion["t_s"], color=COLORS[CASES.index(name)], lw=.7, ls=":", alpha=.9)
    fig.suptitle("Stage-2 spatial transition: where the aircraft moves and what changes", fontsize=17, y=.965)
    save(fig, "00-geographic-transition-overview", "Geographic and temporal overview of the audited R0010 singleton study. Panel (a) shows the full smoothed airport-to-airport reference route, all archived base stations, and the realized Cartesian traces. A marks the common planner decision after F was observed. Panel (b) enlarges that route neighborhood and marks exact maneuver endpoints for lateral/joint and vertical controls; the vertical-only move has no ground-track displacement. Panel (c) shows offset and height with solid and dotted line conventions. Panel (d) shows the exposure-derived C/R/F policy state. This figure is a map-context companion to the progress-coordinate audit figures; no capacity or traffic interaction is included.")

    # Shared progress coordinate is not rescaled separately for each control.
    fig, axes = plt.subplots(4, 1, figsize=(15, 10), sharex=True,
                            gridspec_kw={"height_ratios": [1, 1, 1.3, 1.1]})
    fig.subplots_adjust(left=.075, right=.98, top=.90, bottom=.08, hspace=.36)
    handles = []
    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        rows = traces[name]; q = np.array([r["q_m"] for r in rows])/1000
        effect = [pe.Stroke(linewidth=2.8, foreground=INK), pe.Normal()] if name == "joint" else []
        for ax, key in zip(axes[:2], ["offset_m", "altitude_m"]):
            line, = ax.plot(q, [r[key] for r in rows], color=color, ls=style, lw=1.6,
                            path_effects=effect, label=label)
        handles.append(Line2D([], [], color=color, ls=style, lw=1.8, label=label, path_effects=effect))
        oq = np.array([r["q_m"] for r in obs[name]])/1000
        axes[2].plot(oq, [r["sinr_db"] for r in obs[name]], color=color, ls=style, lw=1.1,
                     path_effects=[pe.Stroke(linewidth=1.9, foreground=INK), pe.Normal()] if name == "joint" else [])
        y = CASES.index(name)
        for policy in "CRF":
            intervals = [(q[i], q[i+1]-q[i]) for i, row in enumerate(rows[:-1]) if row["policy"] == policy]
            axes[3].broken_barh(intervals, (y-.28, .56), facecolors=POLICY[policy], edgecolors="none", rasterized=True)
    for index, e in enumerate(events["joint"], 1):
        for ax, dim in [(axes[0], 0), (axes[1], 1)]:
            ax.plot(e["q_m"]/1000, e["source"][dim], "o", ms=4.2, mfc="white", mec=INK, zorder=9)
            ax.annotate(str(index), (e["q_m"]/1000, e["source"][dim]), xytext=(0, 8),
                        textcoords="offset points", ha="center", fontsize=10)
    axes[0].set(ylabel=r"Offset, $d$ (m)", title="(a) Lateral motion")
    axes[1].set(ylabel=r"Height, $h$ (m)", title="(b) Vertical motion")
    same_path = len(traces["joint"]) == len(traces["lateral"]) and np.array_equal(
        [[r["t_s"], *r["position_m"]] for r in traces["joint"]],
        [[r["t_s"], *r["position_m"]] for r in traces["lateral"]])
    if same_path:
        axes[0].text(.012, .90, "Joint and lateral trajectories coincide", transform=axes[0].transAxes,
                     fontsize=11, va="top")
    # Data-adaptive margins keep unchanged controls visible without inventing motion.
    for ax, key in zip(axes[:2], ["offset_m", "altitude_m"]):
        values = np.array([r[key] for rows in traces.values() for r in rows])
        margin = max(30., .22*float(np.ptp(values)))
        ax.set_ylim(values.min()-margin, values.max()+margin)
        ax.grid(axis="y", color="#DDDDDD", lw=.5)
    axes[2].axhline(cfg["threshold_db"], color=POLICY["F"], ls=":", lw=.8)
    axes[2].set(ylabel="SINR (dB)", title="(c) Realized communication")
    axes[3].set(yticks=range(4), yticklabels=LABELS, ylim=(3.5, -.5),
                xlabel=r"Reference-route progress, $q$ (km)", title="(d) Singleton policy")
    axes[3].legend(handles=[Patch(color=POLICY[p], label=p) for p in "CRF"], ncol=3,
                    loc="upper right", bbox_to_anchor=(1, 1.33), frameon=False)
    axes[3].set_xlim(0, field["q_m"][-1]/1000)
    handles.append(Line2D([], [], color=POLICY["F"], lw=.8, ls=":", label=rf"$\Theta={cfg['threshold_db']:g}$ dB"))
    fig.legend(handles=handles, ncol=5, loc="upper center", bbox_to_anchor=(.52, .99), frameon=False)
    save(fig, "01-flight-profiles", "Matched flights from d=0 m, h=300 m. Actual offset, height, SINR and singleton C/R/F policy "
        "are aligned by reference-route progress q; q is not smoothed arc length. Numbers mark joint-control maneuver onsets. "
        "Overlapping curves represent overlapping outcomes, not missing controls. Policy is time-window exposure based; "
        "instantaneous SINR is not itself policy. Heights use the inherited numerical convention. No traffic or capacity is included.")

    # Three reproducible cross-sections: first joint onset/completion and worst
    # fixed-center SINR location, with fallbacks only if no move is selected.
    profile = field["sinr_db"][:, list(field["offsets_m"]).index(0), list(field["heights_m"]).index(cfg["altitude_m"])]
    worst = float(field["q_m"][np.argmin(profile)])
    if events["joint"]:
        e = events["joint"][0]
        end = min(traces["joint"], key=lambda r: abs(r["t_s"]-e["t_s"]-e["duration_s"]))
        requested = [e["q_m"], end["q_m"], worst]
        section_names = ["First maneuver onset", "First maneuver completion", "Lowest baseline SINR"]
    else:
        requested = [field["q_m"][-1]*.25, field["q_m"][-1]*.5, worst]
        section_names = ["Quarter route", "Mid-route", "Lowest baseline SINR"]
    indices = [int(np.argmin(abs(field["q_m"]-q))) for q in requested]
    z = field["sinr_db"][indices]
    norm = TwoSlopeNorm(vmin=min(float(z.min()), cfg["threshold_db"]-.05),
                        vcenter=cfg["threshold_db"], vmax=max(float(z.max()), cfg["threshold_db"]+.05))
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.5))
    fig.subplots_adjust(left=.06, right=.91, top=.79, bottom=.18, wspace=.25)
    for i, (ax, index, title) in enumerate(zip(axes, indices, section_names)):
        mesh = ax.pcolormesh(field["offsets_m"], field["heights_m"], field["sinr_db"][index].T,
                            shading="nearest", cmap="RdBu", norm=norm, rasterized=True)
        values = field["sinr_db"][index].T
        if values.min() < cfg["threshold_db"] < values.max():
            ax.contour(field["offsets_m"], field["heights_m"], values, levels=[cfg["threshold_db"]],
                       colors=INK, linewidths=.8, linestyles=":")
        aq = np.array([r["q_m"] for r in traces["joint"]])
        point = [np.interp(field["q_m"][index], aq, [r[key] for r in traces["joint"]]) for key in ["offset_m", "altitude_m"]]
        ax.plot(0, cfg["altitude_m"], "+", color=INK, ms=9, mew=1.5)
        ax.plot(*point, "o", ms=6, mfc=COLORS[-1], mec=INK, mew=.9)
        ax.set(xlabel=r"Offset, $d$ (m)", xlim=(field["offsets_m"][0], field["offsets_m"][-1]),
               ylim=(field["heights_m"][0], field["heights_m"][-1]), xticks=[-900, 0, 900],
               yticks=field["heights_m"], title=f"({chr(97+i)}) {title}\n$q$ = {field['q_m'][index]/1000:.2f} km")
        if i == 0:
            ax.set_ylabel(r"Height, $h$ (m)")
        ax.axhline(300, color=INK, lw=.6, ls="--", alpha=.6)
    cax = fig.add_axes([.93, .18, .012, .61]); cb = fig.colorbar(mesh, cax=cax); cb.set_label("SINR (dB)")
    fig.legend(handles=[Line2D([], [], marker="+", color=INK, ls="none", label="Initial state"),
        Line2D([], [], marker="o", color="none", mfc=COLORS[-1], mec=INK, label="Joint-flight position"),
        Line2D([], [], color=INK, ls="--", lw=.7, label="Above 300 m: propagation extrapolation")],
        loc="upper center", ncol=3, frameon=False)
    save(fig, "02-offset-height-sections", "SINR cross-sections on the archived 100 m offset/height grid at three explicitly identified "
        "route locations, rounded to the nearest archived q sample. All panels share the same threshold-centered color scale. "
        "A dot is the joint flight position at that section, not the projection of its whole moving path. Dashed h=300 m separates "
        "higher-altitude model extrapolations. The dotted contour, when present, is SINR=−1.5 dB, not a C-policy boundary. "
        "The d and h display scales differ; these sections do not depict physical turn angles.")

    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.4))
    fig.subplots_adjust(left=.07, right=.985, bottom=.20, top=.83, wspace=.34)
    for ax, key, title, unit in zip(axes, ["F", "total_displacement_m", "total_maneuver_time_s"],
        ["(a) Fallback duration", "(b) Transverse displacement", "(c) Maneuver duration"], ["Time (s)", "Distance (m)", "Time (s)"]):
        values = [summaries[name]["policy_time_s"]["F"] if key == "F" else summaries[name][key] for name in CASES]
        ax.bar(range(4), values, color=COLORS, width=.62)
        ax.set(xticks=range(4), xticklabels=["Fixed", "Lateral", "Vertical", "Joint"], ylabel=unit, title=title,
               ylim=(0, max(max(values)*1.22, 1)))
        for x, value in enumerate(values):
            ax.annotate(f"{value:.1f}", (x, value), xytext=(0, 6), textcoords="offset points", ha="center", fontsize=11)
        ax.spines[["top", "right"]].set_visible(False)
    save(fig, "03-policy-motion-tradeoff", "Matched full-corridor outcomes. Transverse displacement is the sum of Euclidean (Delta_d,Delta_h) "
        "move lengths; it is neither extra geographic path length nor energy. Fallback and maneuver durations are accumulated "
        "over each actual flight. More permitted directions do not imply global optimality or greater capacity.")

    # A local Cartesian 3-D view is supplementary. State all axis enlargement
    # factors explicitly rather than implying physical angles from the drawing.
    spatial_events = [e for e in events["joint"] if e["source"][1] != e["target"][1]]
    detail_case = "joint" if spatial_events else "vertical"
    spatial_events = spatial_events or events["vertical"]
    detail = None
    if spatial_events:
        e = spatial_events[0]
        detail_label = LABELS[CASES.index(detail_case)]
        selected = [r for r in traces[detail_case] if e["t_s"]-10 <= r["t_s"] <= e["t_s"]+e["duration_s"]+10]
        start = np.array(traces["fixed"][0]["position_m"][:2]); end = np.array(traces["fixed"][-1]["position_m"][:2])
        origin, rotation = chord_frame(start, end)
        points = np.array([r["position_m"] for r in selected])
        points[:, :2] = (points[:, :2]-origin) @ rotation
        # Use identical q values when comparing against the reference curve.
        from verify_lateral_study import independent_reference
        with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
            frame = independent_reference(archive, manifest, cfg["center_control_step_m"])
        centers = frame(np.array([r["q_m"] for r in selected]))[0]
        baseline = np.c_[(centers-origin) @ rotation, np.full(len(centers), cfg["altitude_m"])]
        all_points = np.vstack((points, baseline))
        lo, hi = all_points.min(axis=0), all_points.max(axis=0)
        span = np.maximum(hi-lo, [200, 100, 100]); mid = (lo+hi)/2
        lo, hi = mid-span*.65, mid+span*.65
        aspect = np.array([3., 1., 1.])
        metric_scale = aspect/(hi-lo); exaggeration = metric_scale/metric_scale[0]
        detail = {"case": detail_case, "start_s": e["t_s"], "end_s": e["t_s"]+e["duration_s"],
                  "source_offset_height_m": e["source"], "target_offset_height_m": e["target"],
                  "relative_metric_scale_xyz": exaggeration.tolist(), "projection": "orthographic"}
        fig = plt.figure(figsize=(11, 7)); ax = fig.add_axes([0., .22, 1., .77], projection="3d")
        ax.plot(*baseline.T, color=INK, lw=1.1, ls="--", label="Reference at 300 m")
        ax.plot(*points.T, color=COLORS[CASES.index(detail_case)], lw=2.6, label=f"{detail_label} trajectory",
                path_effects=[pe.Stroke(linewidth=3.5, foreground=INK), pe.Normal()])
        for target_time, label in [(e["t_s"], "A"), (e["t_s"]+e["duration_s"], "B")]:
            idx = int(np.argmin([abs(r["t_s"]-target_time) for r in selected]))
            if abs(selected[idx]["t_s"]-target_time) > 1e-8:
                raise ValueError("3-D endpoint annotation requires an exact archived event boundary")
            ax.scatter(*points[idx], color=INK, s=23); ax.text(*points[idx], label, fontsize=12)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(lo[2], hi[2]))
        ax.ticklabel_format(style="plain", useOffset=False)
        for axis in [ax.xaxis, ax.yaxis]:
            axis.set_major_formatter(FuncFormatter(lambda value, _: f"{value/1000:g}"))
        ax.xaxis.set_major_locator(MaxNLocator(4)); ax.yaxis.set_major_locator(MaxNLocator(3))
        ax.zaxis.set_major_locator(MaxNLocator(3)); ax.tick_params(labelsize=10, pad=1)
        ax.set_xlabel("Along-chord X (km)", labelpad=12, fontsize=12)
        ax.set_ylabel("Cross-chord Y (km)", labelpad=10, fontsize=12)
        ax.set_zlabel("Height h (m)", labelpad=8, fontsize=12)
        ax.set_box_aspect(aspect, zoom=1.18); ax.view_init(elev=24, azim=-65); ax.set_proj_type("ortho")
        fig.suptitle(f"{detail_label} control: {e['direction'].replace('_', ' ')}", y=.98, fontsize=15)
        fig.text(.5, .925, f"Metric-scale enlargement relative to X: Y ×{exaggeration[1]:.1f}, h ×{exaggeration[2]:.1f}",
                 ha="center", fontsize=11)
        fig.legend(*ax.get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(.5, .90), ncol=2, frameon=False, fontsize=11)
        save(fig, "04-local-spatial-maneuver", f"Supplementary local 3-D Cartesian view of an actually selected {detail_case} maneuver. "
            "A and B are its exact archived start and completion. Axis enlargement factors are stated in the title; apparent slope, "
            "turn radius and comfort must not be inferred from this anisotropically scaled drawing. No diagonal is fabricated if "
            "the planner selected a pure-axis action. The reference comparator uses the same q positions at d=0,h=300 m.")

    (output/"captions.md").write_text("# Stage-2 figure captions\n\n"+"\n\n".join(f"## {k}\n\n{v}" for k, v in captions.items())+"\n")
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), Path(__file__).with_name("plot_wide_radio_map.py"), Path(__file__).with_name("verify_lateral_study.py")]
    with zipfile.ZipFile(output/"plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(root))
    record = {"run": str(run), "run_manifest_sha256": sha(run/"manifest.json"), "audit_sha256": sha(audit_path),
        "joint_lateral_paths_identical": same_path,
        "spatial_detail": detail,
        "section_indices": indices, "section_q_m": field["q_m"][indices].tolist(), "captions": captions,
        "sources": [{"path": p.relative_to(root).as_posix(), "sha256": sha(p)} for p in sources],
        "outputs": [{"path": p.name, "sha256": sha(p)} for p in sorted(output.iterdir())]}
    (output/"manifest.json").write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps({"output": str(output), "figures": list(captions)}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path); parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    args = parser.parse_args(); plot(args.run, args.output, args.audit)
