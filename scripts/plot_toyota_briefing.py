"""Create a compact, presentation-oriented R0010 Toyota briefing figure set."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Patch
from matplotlib.colors import TwoSlopeNorm
import numpy as np

from verify_lateral_study import independent_reference, read


CASES = ["fixed", "lateral", "vertical", "joint"]
LABELS = ["Fixed", "Lateral only", "Vertical only", "Joint"]
COLORS = ["#686868", "#28658B", "#198577", "#F2BD3C"]
STYLES = ["--", "-", "-.", (0, (6, 3))]
POLICY = {"C": "#198577", "R": "#E7AF3C", "F": "#B64C42"}
INK = "#202020"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plot(run, output, audit_path):
    run, output, audit_path = Path(run).resolve(), Path(output).resolve(), Path(audit_path).resolve()
    if output.exists():
        raise FileExistsError("choose a new Toyota briefing revision directory")
    manifest = read(run / "manifest.json")
    audit = read(audit_path)
    if manifest["status"] != "completed" or audit["status"] != "passed":
        raise ValueError("a completed run and passed audit are required")
    for entry in manifest["outputs"]:
        if digest(run / entry["path"]) != entry["sha256"]:
            raise ValueError(f"archived output changed: {entry['path']}")
    spec = read(run / "resolved_config.json")
    cfg = spec["parameters"]
    summaries = {row["case_id"]: row for row in read(run / "summary.json")}
    traces = {name: read(run / name / "trace.json.gz") + [read(run / name / "terminal.json")] for name in CASES}
    observations = {name: read(run / name / "observations.json.gz") for name in CASES}
    logs = {name: read(run / name / "events.json") for name in CASES}
    accepted = {name: [e for e in logs[name] if e["status"] == "transition_accepted"] for name in CASES}
    completed = {name: next((e for e in logs[name] if e["status"] == "transition_completed"), None) for name in CASES}
    with np.load(run / "field.npz") as archive:
        field = {key: archive[key].copy() for key in archive.files}
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        frame = independent_reference(archive, manifest, cfg["center_control_step_m"])
    accepted_joint = accepted["joint"][0]
    decision_t = float(accepted_joint["t_s"])
    a_trace = min(traces["joint"], key=lambda row: abs(row["t_s"] - decision_t))
    a_xy_abs = np.asarray(a_trace["position_m"][:2])
    b_xy_abs = {name: np.asarray(min(traces[name], key=lambda row: abs(row["t_s"] - completed[name]["t_s"]))["position_m"][:2])
                for name in ["lateral", "vertical"]}
    origin = frame(np.array([0.]))[0][0]
    def rel(points):
        return (np.asarray(points)[..., :2] - origin) / 1000.
    q_grid = np.linspace(0., float(field["q_m"][-1]), 1800)
    route = rel(frame(q_grid)[0])
    stations = np.array([[s["x_m"], s["y_m"]] for s in spec["base_stations"]])
    station_rel = rel(stations)

    plt.rcParams.update({"font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 12,
                         "axes.labelsize": 12, "axes.titlesize": 14, "text.color": INK,
                         "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
                         "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
                         "svg.hashsalt": "uam-toyota-briefing-v1", "figure.facecolor": "white",
                         "savefig.facecolor": "white"})
    output.mkdir(parents=True)
    captions = {}

    def save(fig, name, caption):
        for ext in ["png", "svg"]:
            fig.savefig(output / f"{name}.{ext}", dpi=300, bbox_inches="tight", pad_inches=.12,
                        metadata={"Description": caption})
        captions[name] = caption
        plt.close(fig)

    # Figure 1: a method diagram for a non-specialist audience.
    fig, ax = plt.subplots(figsize=(16, 4.8))
    ax.set_xlim(0, 6.2); ax.set_ylim(0, 1); ax.axis("off")
    steps = [
        ("1  Matched start", "one UAM\nq=0, d=0, h=300 m\nV=50 m/s", "#E8EEF1"),
        ("2  Radio state", "12 base stations\nnearest-three links\nSINR and serving site", "#E5F0ED"),
        ("3  Policy state", "5 s samples\n30 s exposure window\nC / R / F", "#F7EED6"),
        ("4  Decision gate", "F observed\n20 s decisions\n95 spatial states", "#F5E1DD"),
        ("5  Continuous move", "quintic d(t), h(t)\nspeed/acceleration\ncurvature checks", "#E7E2F0"),
        ("6  Measured result", "selected endpoint\nrecovery time\nF duration and path", "#E6EDE1"),
    ]
    for i, (title, body, color) in enumerate(steps):
        x = .08 + i * 1.01
        box = FancyBboxPatch((x, .25), .86, .52, boxstyle="round,pad=.015,rounding_size=.025",
                             linewidth=.8, edgecolor=INK, facecolor=color)
        ax.add_patch(box)
        ax.text(x + .43, .67, title, ha="center", va="center", fontsize=11, weight="bold")
        ax.text(x + .43, .47, body, ha="center", va="center", fontsize=9.2, linespacing=1.35)
        if i < len(steps) - 1:
            ax.add_patch(FancyArrowPatch((x + .87, .51), (x + .99, .51), arrowstyle="-|>",
                                         mutation_scale=13, linewidth=1.1, color=INK))
    ax.set_title("R0010 method: communication state drives a bounded spatial transition", pad=12, fontsize=16)
    fig.text(.5, .065, "The four controls are matched single-aircraft runs. Traffic interaction, ACC, group exposure and capacity are outside this experiment.",
             ha="center", fontsize=10.5, color="#4A5961")
    save(fig, "01-method-workflow", "Conceptual workflow for the R0010 Toyota briefing. The diagram follows one aircraft from the matched initial state through radio evaluation, exposure-based C/R/F policy, the F-triggered candidate search, continuous quintic motion, and measured outcomes. R0010 contains no neighboring traffic, ACC interaction, or capacity estimate.")

    # Figure 2: geographic context and local decision map.
    fig, (full_ax, local_ax) = plt.subplots(1, 2, figsize=(16, 6.2))
    full_ax.plot(route[:, 0], route[:, 1], color="#A6A6A6", lw=1, ls=(0, (2, 2)), label="Reference route")
    handles = []
    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        xy = rel([r["position_m"] for r in traces[name]])
        line, = full_ax.plot(xy[:, 0], xy[:, 1], color=color, lw=1.6, ls=style, label=label)
        handles.append(line)
    full_ax.scatter(station_rel[:, 0], station_rel[:, 1], marker="^", s=34, color="#4D6475",
                    edgecolor="white", linewidth=.5, zorder=5)
    for station, point in zip(spec["base_stations"], station_rel):
        full_ax.annotate(station["site_id"], point, xytext=(3, 3), textcoords="offset points", fontsize=7.3)
    a_xy = rel(a_xy_abs)
    full_ax.scatter(*a_xy, marker="*", s=100, color=INK, edgecolor="white", linewidth=.7, zorder=8)
    full_ax.annotate("A: decision", a_xy, xytext=(7, 8), textcoords="offset points", fontsize=9,
                     bbox=dict(boxstyle="round,pad=.2", fc="white", ec="#777777", alpha=.9))
    full_ax.set_title("(a) Full route and all base stations", loc="left")
    full_ax.set_xlabel("Relative easting from departure (km)"); full_ax.set_ylabel("Relative northing from departure (km)")
    full_ax.set_aspect("equal", adjustable="datalim"); full_ax.grid(color="#E5E5E5", lw=.45)
    full_ax.legend(handles=handles + [Line2D([], [], marker="^", color="none", mfc="#4D6475", mec="white", label="Base station")],
                   loc="lower left", frameon=True, framealpha=.92, fontsize=8.5, ncol=2)

    q_a = float(accepted_joint["q_m"])
    q_b = max(float(completed[name]["q_m"]) for name in ["lateral", "vertical"])
    pad = 1800.
    qmask = (q_grid >= max(0., q_a-pad)) & (q_grid <= min(float(field["q_m"][-1]), q_b+pad))
    local_ax.plot(route[qmask, 0], route[qmask, 1], color="#A6A6A6", lw=1, ls=(0, (2, 2)), label="Reference route")
    for name, color, style in zip(CASES, COLORS, STYLES):
        qv = np.array([r["q_m"] for r in traces[name]])
        mask = (qv >= q_a-pad) & (qv <= q_b+pad)
        local_xy = rel([r["position_m"] for r, keep in zip(traces[name], mask) if keep])
        local_ax.plot(local_xy[:, 0], local_xy[:, 1], color=color, lw=2, ls=style)
    local_stations = (station_rel[:, 0] >= route[qmask, 0].min()-.4) & (station_rel[:, 0] <= route[qmask, 0].max()+.4) & \
                     (station_rel[:, 1] >= route[qmask, 1].min()-.4) & (station_rel[:, 1] <= route[qmask, 1].max()+.4)
    local_ax.scatter(station_rel[local_stations, 0], station_rel[local_stations, 1], marker="^", s=48,
                     color="#4D6475", edgecolor="white", linewidth=.6, zorder=5)
    for station, point, keep in zip(spec["base_stations"], station_rel, local_stations):
        if keep:
            local_ax.annotate(station["site_id"], point, xytext=(4, 4), textcoords="offset points", fontsize=8)
    local_ax.scatter(*a_xy, marker="*", s=115, color=INK, edgecolor="white", linewidth=.8, zorder=8)
    local_ax.annotate("A: F observed → planner decides", a_xy, xytext=(7, 8), textcoords="offset points", fontsize=8.8,
                      bbox=dict(boxstyle="round,pad=.2", fc="white", ec="#777777", alpha=.9))
    for name, marker, text in [("lateral", "s", "B-L/J: +100 m lateral"), ("vertical", "D", "B-V: −200 m vertical")]:
        point = rel(b_xy_abs[name])
        local_ax.scatter(*point, marker=marker, s=65, color=COLORS[CASES.index(name)], edgecolor=INK, linewidth=.7, zorder=8)
        local_ax.annotate(text, point, xytext=(7, 8 if name == "lateral" else -13), textcoords="offset points", fontsize=8.6)
    local_ax.set_xlim(route[qmask, 0].min()-.18, route[qmask, 0].max()+.18)
    local_ax.set_ylim(route[qmask, 1].min()-.18, route[qmask, 1].max()+.18)
    local_ax.set_title("(b) First-decision neighborhood", loc="left")
    local_ax.set_xlabel("Relative easting from departure (km)"); local_ax.set_ylabel("Relative northing from departure (km)")
    local_ax.set_aspect("equal", adjustable="box"); local_ax.grid(color="#E5E5E5", lw=.45)
    fig.suptitle("Where the spatial decision occurs", fontsize=16, y=.98)
    save(fig, "02-geographic-transition", "Geographic context for the first accepted R0010 transition. Panel (a) shows the complete smoothed reference route, all 12 archived base stations, and the four matched ground traces. Panel (b) enlarges the decision neighborhood. A is the common t=460 s planning point; B-L/J and B-V are the exact lateral/joint and vertical completion positions. The vertical maneuver has no ground-track displacement and is therefore distinguished by its endpoint label rather than a separate ground curve.")

    # Figure 3: candidate space at the first accepted decision.
    field_index = int(np.argmin(abs(field["q_m"] - q_a)))
    values = field["sinr_db"][field_index].T
    norm = TwoSlopeNorm(vmin=min(float(values.min()), cfg["threshold_db"]-.05),
                        vcenter=cfg["threshold_db"], vmax=max(float(values.max()), cfg["threshold_db"]+.05))
    fig, ax = plt.subplots(figsize=(9, 6.6))
    fig.subplots_adjust(left=.11, right=.90, bottom=.22, top=.88)
    mesh = ax.pcolormesh(field["offsets_m"], field["heights_m"], values, shading="nearest", cmap="RdBu", norm=norm, rasterized=True)
    dmesh, hmesh = np.meshgrid(field["offsets_m"], field["heights_m"])
    ax.scatter(dmesh.ravel(), hmesh.ravel(), s=8, color="#FFFFFF", edgecolor="#777777", linewidth=.25, alpha=.65, zorder=3)
    initial = np.array([0., cfg["altitude_m"]])
    lateral_target = np.array(accepted["lateral"][0]["target"])
    vertical_target = np.array(accepted["vertical"][0]["target"])
    ax.scatter(*initial, marker="+", s=130, color=INK, linewidth=2, zorder=7, label="Initial state")
    for target, color, marker, label in [(lateral_target, COLORS[1], "s", "Lateral / joint selected"),
                                          (vertical_target, COLORS[2], "D", "Vertical selected")]:
        ax.add_patch(FancyArrowPatch(initial, target, arrowstyle="-|>", mutation_scale=14, linewidth=1.4,
                                     color=color, zorder=6))
        ax.scatter(*target, marker=marker, s=75, color=color, edgecolor=INK, linewidth=.8, zorder=7, label=label)
    ax.axhline(cfg["altitude_m"], color=INK, lw=.7, ls="--", alpha=.7)
    ax.axvline(0, color=INK, lw=.7, ls="--", alpha=.7)
    ax.set(xlabel="Signed lateral offset, d (m)", ylabel="Altitude, h (m)",
           title=f"Candidate space at planner decision (q = {field['q_m'][field_index]/1000:.2f} km)")
    ax.set_xticks([-900, -600, -300, 0, 300, 600, 900]); ax.set_yticks(field["heights_m"])
    ax.grid(color="#FFFFFF", lw=.4, alpha=.5)
    ax.legend(loc="lower left", frameon=True, framealpha=.92, fontsize=9)
    cb = fig.colorbar(mesh, ax=ax, pad=.03); cb.set_label("Instantaneous SINR (dB)")
    fig.text(.5, .045, "Color is an instantaneous radio value. Acceptance additionally required a complete forecast, stable post-completion C, earlier recovery than staying, and curvature feasibility.",
             ha="center", fontsize=9.7, color="#4A5961")
    save(fig, "03-candidate-selection", "SINR section and candidate actions at the first accepted planner decision. The grid is the 100 m offset/height search space. The arrows identify the selected pure-axis endpoints for lateral/joint and vertical controls. The color field alone does not decide acceptance: the planner also evaluates continuous motion, the 240 s forecast, stable post-completion C, natural recovery and curvature.")

    # Figure 4: concise numerical result for the briefing.
    fig = plt.figure(figsize=(15, 8.7))
    grid = fig.add_gridspec(2, 2, left=.07, right=.98, top=.91, bottom=.13, hspace=.42, wspace=.28,
                            height_ratios=[1, 1.15])
    f_ax = fig.add_subplot(grid[0, 0]); move_ax = fig.add_subplot(grid[0, 1]); policy_ax = fig.add_subplot(grid[1, :])
    f_values = [summaries[name]["policy_time_s"]["F"] for name in CASES]
    f_ax.bar(range(4), f_values, color=COLORS, width=.62)
    f_ax.set(xticks=range(4), xticklabels=LABELS, ylabel="Fallback duration (s)", title="(a) F duration")
    f_ax.set_ylim(0, max(f_values)*1.22); f_ax.spines[["top", "right"]].set_visible(False)
    for x, value in enumerate(f_values):
        f_ax.annotate(f"{value:.0f}", (x, value), xytext=(0, 5), textcoords="offset points", ha="center", fontsize=10)
    displacements = [summaries[name]["total_displacement_m"] for name in CASES]
    move_ax.bar(range(4), displacements, color=COLORS, width=.62)
    move_ax.set(xticks=range(4), xticklabels=LABELS, ylabel="Spatial displacement (m)", title="(b) Selected spatial change")
    move_ax.set_ylim(0, max(displacements)*1.28); move_ax.spines[["top", "right"]].set_visible(False)
    for x, value in enumerate(displacements):
        move_ax.annotate(f"{value:.0f}", (x, value), xytext=(0, 5), textcoords="offset points", ha="center", fontsize=10)
    t_lo, t_hi = 400., 620.
    for name in CASES:
        rows = [r for r in observations[name] if t_lo <= r["t_s"] <= t_hi]
        for i, row in enumerate(rows):
            end = rows[i+1]["t_s"] if i + 1 < len(rows) else min(t_hi, row["t_s"] + cfg["policy_s"])
            policy_ax.broken_barh([(row["t_s"], max(0., end-row["t_s"]))],
                                  (CASES.index(name)-.31, .62), facecolors=POLICY[row["policy"]], edgecolors="none", rasterized=True)
    policy_ax.axvline(450, color=POLICY["F"], lw=.9, ls="--")
    policy_ax.axvline(decision_t, color=INK, lw=.9, ls="--")
    policy_ax.annotate("F observed", (450, 3.5), xytext=(4, -4), textcoords="offset points", fontsize=8.8, va="top")
    policy_ax.annotate("A: decision", (decision_t, 3.5), xytext=(4, -4), textcoords="offset points", fontsize=8.8, va="top")
    for name, completion in completed.items():
        if completion:
            policy_ax.axvline(completion["t_s"], color=COLORS[CASES.index(name)], lw=.7, ls=":")
    policy_ax.set(xlim=(t_lo, t_hi), ylim=(3.5, -.5), yticks=range(4), yticklabels=LABELS,
                  xlabel="Time since departure (s)", title="(c) Policy state around the accepted transition")
    policy_ax.legend(handles=[Patch(color=POLICY[p], label=p) for p in "CRF"], loc="upper left", frameon=False, ncol=3, fontsize=9)
    policy_ax.grid(axis="x", color="#E5E5E5", lw=.5)
    fig.suptitle("R0010 result: a small spatial change can remove one policy interval of fallback", fontsize=16, y=.97)
    save(fig, "04-results-and-policy", "Briefing summary of R0010 outcomes. Panel (a) compares total fallback duration; panel (b) compares the selected spatial displacement; panel (c) aligns the C/R/F policy state around the first accepted transition. Lateral and joint both select +100 m and reduce F from 280 to 270 s; vertical selects a 200 m descent and reduces F to 275 s. These are singleton communication-policy results, not capacity results.")

    (output / "captions.md").write_text("# Toyota briefing figure captions\n\n" +
                                           "\n\n".join(f"## {key}\n\n{value}" for key, value in captions.items()) + "\n")
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), Path(__file__).with_name("verify_lateral_study.py")]
    with zipfile.ZipFile(output / "plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(root))
    record = {"run": str(run), "run_manifest_sha256": digest(run / "manifest.json"), "audit_sha256": digest(audit_path),
              "figures": list(captions), "decision_time_s": decision_t,
              "accepted_targets": {name: accepted[name][0]["target"] for name in ["lateral", "vertical", "joint"]},
              "sources": [{"path": p.relative_to(root).as_posix(), "sha256": digest(p)} for p in sources],
              "outputs": [{"path": p.name, "sha256": digest(p)} for p in sorted(output.iterdir()) if p.name != "manifest.json"]}
    (output / "manifest.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"output": str(output), "figures": list(captions)}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    args = parser.parse_args()
    plot(args.run, args.output, args.audit)
