"""Create the clearer, decision-centered R0010 Toyota briefing figures."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Patch
from mpl_toolkits.mplot3d.art3d import Line3DCollection
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
    q_decision = float(accepted_joint["q_m"])
    field_index = int(np.argmin(abs(field["q_m"] - q_decision)))
    initial = np.array([0., cfg["altitude_m"]])
    lateral_target = np.array(accepted["lateral"][0]["target"])
    vertical_target = np.array(accepted["vertical"][0]["target"])

    plt.rcParams.update({"font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 12,
                         "axes.labelsize": 12, "axes.titlesize": 14, "text.color": INK,
                         "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
                         "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
                         "svg.hashsalt": "uam-toyota-briefing-v3", "figure.facecolor": "white",
                         "savefig.facecolor": "white"})
    output.mkdir(parents=True)
    captions = {}

    def save(fig, name, caption):
        for ext in ["png", "svg"]:
            fig.savefig(output / f"{name}.{ext}", dpi=300, bbox_inches="tight", pad_inches=.12,
                        metadata={"Description": caption})
        captions[name] = caption
        plt.close(fig)

    # 1. Method flow, intentionally free of route geometry.
    fig, ax = plt.subplots(figsize=(16, 4.8))
    ax.set_xlim(0, 6.2); ax.set_ylim(0, 1); ax.axis("off")
    steps = [
        ("1  Matched start", "one UAM\nq=0, d=0, h=300 m\nV=50 m/s", "#E8EEF1"),
        ("2  Radio state", "12 base stations\nnearest-three links\nSINR and serving site", "#E5F0ED"),
        ("3  Policy state", "5 s samples\n30 s exposure window\nC / R / F", "#F7EED6"),
        ("4  Decision gate", "F observed\n20 s decisions\n(d,h) candidates", "#F5E1DD"),
        ("5  Continuous move", "quintic d(t), h(t)\nspeed/acceleration\ncurvature checks", "#E7E2F0"),
        ("6  Measured result", "endpoint and timing\npolicy recovery\nfull-corridor comparison", "#E6EDE1"),
    ]
    for i, (title, body, color) in enumerate(steps):
        x = .08 + i * 1.01
        ax.add_patch(FancyBboxPatch((x, .25), .86, .52, boxstyle="round,pad=.015,rounding_size=.025",
                                    linewidth=.8, edgecolor=INK, facecolor=color))
        ax.text(x + .43, .67, title, ha="center", va="center", fontsize=11, weight="bold")
        ax.text(x + .43, .47, body, ha="center", va="center", fontsize=9.2, linespacing=1.35)
        if i < len(steps) - 1:
            ax.add_patch(FancyArrowPatch((x + .87, .51), (x + .99, .51), arrowstyle="-|>",
                                         mutation_scale=13, linewidth=1.1, color=INK))
    ax.set_title("R0010 method: a radio observation is converted into a bounded movement decision", pad=12, fontsize=16)
    fig.text(.5, .065, "This is a matched single-aircraft experiment. The later multi-UAM study adds neighboring traffic, group exposure, ACC and capacity.",
             ha="center", fontsize=10.5, color="#4A5961")
    save(fig, "01-method-workflow", "Method overview for the R0010 Toyota briefing. One aircraft begins from a matched state; radio measurements feed an exposure-based C/R/F policy; an F observation opens a bounded offset/height search; continuous quintic motion and curvature checks gate the action; and the result is measured over the full corridor. Traffic interaction, ACC and capacity are outside R0010.")

    # 2. One decision example: instantaneous candidate plane, forecast gate,
    # and realized 3-D trajectory. This replaces the map-heavy panel for the
    # briefing because it explains the mechanism directly.
    fig = plt.figure(figsize=(16, 11))
    grid = fig.add_gridspec(2, 2, left=.06, right=.97, top=.91, bottom=.10,
                            hspace=.34, wspace=.25, height_ratios=[1, 1.55])
    plane_ax = fig.add_subplot(grid[0, 0])
    forecast_ax = fig.add_subplot(grid[0, 1])
    traj_ax = fig.add_subplot(grid[1, :], projection="3d")
    values = field["sinr_db"][field_index].T
    norm = TwoSlopeNorm(vmin=min(float(values.min()), cfg["threshold_db"]-.05),
                        vcenter=cfg["threshold_db"], vmax=max(float(values.max()), cfg["threshold_db"]+.05))
    mesh = plane_ax.pcolormesh(field["offsets_m"], field["heights_m"], values, shading="nearest",
                               cmap="RdBu", norm=norm, rasterized=True)
    dmesh, hmesh = np.meshgrid(field["offsets_m"], field["heights_m"])
    plane_ax.scatter(dmesh.ravel(), hmesh.ravel(), s=8, color="white", edgecolor="#777777",
                     linewidth=.25, alpha=.65, zorder=3)
    plane_ax.scatter(*initial, marker="+", s=130, color=INK, linewidth=2, zorder=7, label="Current state")
    for target, color, marker, label in [(lateral_target, COLORS[1], "s", "Lateral / joint target"),
                                         (vertical_target, COLORS[2], "D", "Vertical target")]:
        plane_ax.add_patch(FancyArrowPatch(initial, target, arrowstyle="-|>", mutation_scale=14,
                                           linewidth=1.4, color=color, zorder=6))
        plane_ax.scatter(*target, marker=marker, s=75, color=color, edgecolor=INK, linewidth=.8,
                         zorder=7, label=label)
    plane_ax.axhline(cfg["altitude_m"], color=INK, lw=.7, ls="--", alpha=.7)
    plane_ax.axvline(0, color=INK, lw=.7, ls="--", alpha=.7)
    plane_ax.set(xlabel="Signed lateral offset, d (m)", ylabel="Altitude, h (m)",
                 title=f"(a) Instantaneous candidate plane at q = {field['q_m'][field_index]/1000:.2f} km")
    plane_ax.set_xticks([-900, -600, -300, 0, 300, 600, 900]); plane_ax.set_yticks(field["heights_m"])
    plane_ax.grid(color="white", lw=.4, alpha=.5); plane_ax.legend(loc="lower left", frameon=True, framealpha=.92, fontsize=8.7)
    cb = fig.colorbar(mesh, ax=plane_ax, pad=.03); cb.set_label("Instantaneous SINR (dB)")

    # The forecast panel deliberately reports the policy gate rather than
    # pretending that every candidate is a separate realized aircraft.
    # Use the planner's stable-C time for the stay comparator, rather than the
    # first isolated C sample (which can be followed by another F episode).
    fixed_c = accepted_joint["stay_recovery_time_s"]
    forecast_rows = [("Stay at (0,300)", fixed_c, None, "stay"),
                     ("Candidate d=+100", accepted["lateral"][0]["recovery_time_s"], completed["lateral"]["t_s"], "lateral"),
                     ("Candidate h=100", accepted["vertical"][0]["recovery_time_s"], completed["vertical"]["t_s"], "vertical")]
    t_lo, t_hi = decision_t - 35., max(620., max(row[1] for row in forecast_rows if row[1] is not None) + 35.)
    for i, (label, stable, completion_t, kind) in enumerate(forecast_rows):
        stable = float(stable) if stable is not None else t_hi
        forecast_ax.barh(i, max(0., stable-decision_t), left=decision_t, height=.56,
                         color=POLICY["F"], edgecolor="none")
        forecast_ax.barh(i, max(0., t_hi-stable), left=stable, height=.56,
                         color=POLICY["C"], edgecolor="none")
        forecast_ax.scatter(decision_t, i, marker="|", s=120, color=INK, zorder=5)
        if completion_t is not None:
            forecast_ax.scatter(completion_t, i, marker="o", s=38,
                                color=COLORS[CASES.index(kind)], edgecolor=INK, zorder=6)
            forecast_ax.annotate("completion", (completion_t, i), xytext=(3, 10), textcoords="offset points", fontsize=8)
        stable_text = "stable C episode" if kind == "vertical" else "stable C"
        text_offset = (4, 12) if kind == "vertical" else (4, -4)
        forecast_ax.annotate(f"{stable_text} ≈ {stable:.0f} s", (stable, i), xytext=text_offset,
                             textcoords="offset points", fontsize=8.4, va="top")
    forecast_ax.axvline(decision_t, color=INK, lw=.9, ls="--")
    forecast_ax.set(yticks=range(3), yticklabels=[r[0] for r in forecast_rows], xlim=(t_lo, t_hi), ylim=(2.5, -.5),
                    xlabel="Time since departure (s)", title="(b) Forecast gate at the F decision")
    forecast_ax.grid(axis="x", color="#E5E5E5", lw=.5)
    forecast_ax.legend(handles=[Patch(color=POLICY["F"], label="Predicted F"), Patch(color=POLICY["C"], label="Stable C"),
                                Line2D([], [], marker="o", color="none", mfc=COLORS[1], mec=INK, label="Maneuver completion")],
                       loc="lower right", frameon=False, fontsize=8.5)
    forecast_ax.text(.02, -.20, "The plane supplies candidate SINR; acceptance also checks the full motion and policy forecast.\nVertical: completion 585 s; first post-completion C sample 590 s.",
                     transform=forecast_ax.transAxes, fontsize=8.3, color="#4A5961", va="top")

    # Realized joint trajectory, with the line color encoding policy rather
    # than control mode. This is the requested visual distinction.
    rows = [r for r in traces["joint"] if 400. <= r["t_s"] <= 620.]
    xyz = np.array([[r["q_m"] / 1000., r["offset_m"], r["altitude_m"]] for r in rows])
    segments = np.stack([xyz[:-1], xyz[1:]], axis=1)
    segment_colors = [POLICY[r["policy"]] for r in rows[:-1]]
    traj_ax.add_collection3d(Line3DCollection(segments, colors=segment_colors, linewidths=3.2))
    fixed_rows = [r for r in traces["fixed"] if 400. <= r["t_s"] <= 620.]
    fixed_xyz = np.array([[r["q_m"] / 1000., 0., 300.] for r in fixed_rows])
    traj_ax.plot(fixed_xyz[:, 0], fixed_xyz[:, 1], fixed_xyz[:, 2], color="#777777", lw=1.1, ls="--", label="Stay at d=0,h=300")
    a_row = min(rows, key=lambda r: abs(r["t_s"] - decision_t))
    b_row = min(rows, key=lambda r: abs(r["t_s"] - completed["joint"]["t_s"]))
    for point, label in [(a_row, "A: decision"), (b_row, "B: +100 m lateral complete")]:
        p = np.array([point["q_m"] / 1000., point["offset_m"], point["altitude_m"]])
        traj_ax.scatter(*p, color=INK, s=35, zorder=8)
        traj_ax.text(*p, label, fontsize=9)
    traj_ax.set_xlim(20.5, 31.5); traj_ax.set_ylim(-35, 135); traj_ax.set_zlim(75, 325)
    traj_ax.set_xlabel("Route progress q (km)", labelpad=10); traj_ax.set_ylabel("Offset d (m)", labelpad=8); traj_ax.set_zlabel("Altitude h (m)", labelpad=8)
    traj_ax.set_title("(c) Realized joint trajectory; color denotes C/R/F policy", pad=10, loc="left")
    traj_ax.set_box_aspect([8.0, 2.4, 2.8]); traj_ax.view_init(elev=24, azim=-66); traj_ax.set_proj_type("ortho")
    traj_ax.legend(handles=[Patch(color=POLICY["C"], label="C"), Patch(color=POLICY["R"], label="R"), Patch(color=POLICY["F"], label="F"),
                            Line2D([], [], color="#777777", ls="--", label="Unchanged reference")],
                   loc="upper left", bbox_to_anchor=(0., .98), frameon=False, ncol=4, fontsize=9)
    fig.suptitle("One R0010 decision: from a q-slice to a predicted and realized movement", fontsize=17, y=.965)
    save(fig, "02-decision-plane-forecast-trajectory", "Decision-centered explanation of the R0010 mechanism. Panel (a) is a fixed-q cross-section in the signed offset/height plane; its colors are instantaneous SINR and its arrows are candidate endpoints. Panel (b) shows the policy forecast gate for staying, lateral and vertical candidates, including maneuver completion and stable-C timing. Panel (c) shows the realized joint trajectory in (q,d,h) coordinates with line color encoding C/R/F policy. The candidate plane is not sufficient by itself: the accepted move also passed continuous-motion, stable-C, earlier-than-staying and curvature checks.")

    # 3. Full-corridor comparison: make the four matched cases explicit and
    # align the communication, physical state and policy views by q.
    fig, axes = plt.subplots(4, 1, figsize=(15, 10), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1, 1.1, 1.1]})
    fig.subplots_adjust(left=.08, right=.98, top=.87, bottom=.08, hspace=.34)
    handles = []
    for name, label, color, style in zip(CASES, LABELS, COLORS, STYLES):
        rows = traces[name]
        qv = np.array([r["q_m"] for r in rows]) / 1000.
        line, = axes[0].plot(qv, [r["offset_m"] for r in rows], color=color, lw=1.6, ls=style, label=label)
        axes[1].plot(qv, [r["altitude_m"] for r in rows], color=color, lw=1.6, ls=style)
        oq = np.array([r["q_m"] for r in observations[name]]) / 1000.
        axes[2].plot(oq, [r["sinr_db"] for r in observations[name]], color=color, lw=1.1, ls=style)
        for i, row in enumerate(rows[:-1]):
            if row["policy"] in POLICY:
                axes[3].broken_barh([(qv[i], qv[i+1]-qv[i])], (CASES.index(name)-.28, .56),
                                    facecolors=POLICY[row["policy"]], edgecolors="none", rasterized=True)
        handles.append(line)
    for ax in axes[:3]:
        ax.axvline(q_decision/1000., color=INK, lw=.8, ls="--")
        ax.grid(axis="y", color="#E5E5E5", lw=.5)
    axes[0].set_ylabel("Offset d (m)"); axes[0].set_title("(a) Lateral state")
    axes[1].set_ylabel("Height h (m)"); axes[1].set_title("(b) Vertical state")
    axes[2].axhline(cfg["threshold_db"], color=POLICY["F"], lw=.8, ls=":")
    axes[2].set_ylabel("SINR (dB)"); axes[2].set_title("(c) Communication quality")
    axes[3].set(yticks=range(4), yticklabels=LABELS, ylim=(3.5, -.5), xlabel="Reference-route progress, q (km)", title="(d) Policy state")
    axes[3].legend(handles=[Patch(color=POLICY[p], label=p) for p in "CRF"], loc="upper right", frameon=False, ncol=3)
    axes[3].grid(axis="x", color="#E5E5E5", lw=.5)
    fig.legend(handles=handles, ncol=4, loc="upper center", bbox_to_anchor=(.52, .945), frameon=False)
    axes[0].annotate("accepted move starts here", (q_decision/1000., accepted_joint["source"][0]),
                     xytext=(8, 9), textcoords="offset points", fontsize=9)
    fig.suptitle("Full-corridor comparison of the four matched controls", fontsize=17, y=.985)
    save(fig, "03-full-corridor-comparison", "Matched full-corridor comparison for the four principal R0010 controls. Offset, height, SINR and exposure-derived C/R/F policy share the same reference-route progress axis. The controls overlap until the first accepted decision; afterward, lateral and joint share the +100 m path, while vertical changes height without changing the ground track. The vertical line identifies the common q≈23.03 km decision location.")

    # 4. Compact result for the final briefing slide.
    fig = plt.figure(figsize=(15, 8.7))
    grid = fig.add_gridspec(2, 2, left=.07, right=.98, top=.90, bottom=.12, hspace=.43, wspace=.28,
                            height_ratios=[1, 1.15])
    f_ax = fig.add_subplot(grid[0, 0]); move_ax = fig.add_subplot(grid[0, 1]); table_ax = fig.add_subplot(grid[1, :])
    f_values = [summaries[name]["policy_time_s"]["F"] for name in CASES]
    f_ax.bar(range(4), f_values, color=COLORS, width=.62)
    f_ax.set(xticks=range(4), xticklabels=LABELS, ylabel="Fallback duration (s)", title="(a) Full-corridor F time")
    f_ax.set_ylim(0, max(f_values)*1.22); f_ax.spines[["top", "right"]].set_visible(False)
    for x, value in enumerate(f_values):
        f_ax.annotate(f"{value:.0f}", (x, value), xytext=(0, 5), textcoords="offset points", ha="center", fontsize=10)
    displacements = [summaries[name]["total_displacement_m"] for name in CASES]
    move_ax.bar(range(4), displacements, color=COLORS, width=.62)
    move_ax.set(xticks=range(4), xticklabels=LABELS, ylabel="Selected displacement (m)", title="(b) Spatial change")
    move_ax.set_ylim(0, max(displacements)*1.28); move_ax.spines[["top", "right"]].set_visible(False)
    for x, value in enumerate(displacements):
        move_ax.annotate(f"{value:.0f}", (x, value), xytext=(0, 5), textcoords="offset points", ha="center", fontsize=10)
    table_ax.axis("off")
    columns = ["Control", "Allowed movement", "Selected endpoint", "Completion", "Interpretation"]
    rows = [
        ["Fixed", "None", "(0,300) m", "—", "Reference baseline"],
        ["Lateral only", "d only", "(100,300) m", "568.9 s", "Smallest accepted spatial change"],
        ["Vertical only", "h only", "(0,100) m", "585.0 s", "200 m descent; smaller benefit"],
        ["Joint", "d, h or both", "(100,300) m", "568.9 s", "Diagonal allowed, not needed"],
    ]
    table = table_ax.table(cellText=rows, colLabels=columns, cellLoc="left", colLoc="left",
                           loc="center", colWidths=[.15, .18, .18, .15, .34])
    table.auto_set_font_size(False); table.set_fontsize(10.5); table.scale(1, 1.65)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor("#C8C8C8")
        if row == 0:
            cell.set_facecolor("#E9EEF1"); cell.set_text_props(weight="bold")
        elif row % 2 == 0:
            cell.set_facecolor("#F8F8F8")
    table_ax.set_title("(c) What the planner selected and why", loc="left", pad=12, fontsize=14)
    fig.suptitle("R0010 result: communication recovery is achieved with a small, bounded movement", fontsize=17, y=.965)
    fig.text(.5, .035, "Selection order: stable C recovery → minimum spatial displacement → shortest accepted duration. This is a singleton baseline, not a capacity conclusion.",
             ha="center", fontsize=10.2, color="#4A5961")
    save(fig, "04-results-summary", "Final R0010 briefing summary. The bars compare full-corridor fallback time and selected spatial displacement. The table states the allowed movement set, selected endpoint, completion time and interpretation for each matched control. The joint controller permits diagonal actions but chooses the same +100 m lateral endpoint as the lateral-only controller because it is the smallest eligible change. Results are singleton communication-policy outcomes, not capacity estimates.")

    (output / "captions.md").write_text("# Toyota briefing revision-03 captions\n\n" +
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
