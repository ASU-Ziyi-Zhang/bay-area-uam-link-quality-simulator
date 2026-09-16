"""Create the two-figure, decision-centered R0010 Toyota briefing set.

Revision 06 follows the agreed explanation order: Figure 1 locates one
longitudinal decision and shows its offset--height candidate plane; Figure 2
shows the candidate motion forecast and the realized full-corridor outcome.
Previous briefing revisions are never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
from matplotlib.patches import ConnectionPatch, Patch
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
from verify_lateral_study import independent_reference, read  # noqa: E402


CASES = ["fixed", "lateral", "vertical", "joint"]
POLICY = {"C": "#198577", "R": "#E7AF3C", "F": "#B64C42"}
INK = "#202020"
BLUE = "#28658B"
GREY = "#737373"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quintic(u):
    u = np.asarray(u, dtype=float)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def segments_by_policy(rows, x_key="q_m", x_scale=1000.0):
    """Return short line segments and their policy colours."""
    xyz = np.asarray([[r[x_key] / x_scale, r["offset_m"]] for r in rows], dtype=float)
    return xyz[:-1], xyz[1:], [POLICY.get(r["policy"], GREY) for r in rows[:-1]]


def station_projection(frame, stations, q_grid):
    centers, _, _, _, _ = frame(q_grid)
    base = centers[0]
    dxy = centers[:, None, :] - stations[None, :, :2]
    nearest = np.argmin(np.linalg.norm(dxy, axis=2), axis=0)
    return q_grid[nearest], stations[:, :2] - base[:2], centers - base[:2]


def event_at(events, t_s, target):
    target = np.asarray(target, dtype=float)
    rows = [e for e in events if abs(float(e.get("t_s", -1)) - t_s) < 1e-8
            and np.allclose(e.get("target", [np.nan, np.nan]), target)]
    return rows


def first_duration(events, t_s, target, accepted_only=False):
    rows = event_at(events, t_s, target)
    if accepted_only:
        rows = [r for r in rows if r["status"] in {"transition_accepted", "eligible"}]
    rows = [r for r in rows if float(r.get("duration_s", 0.0)) > 0]
    return float(min(rows, key=lambda r: r["duration_s"])["duration_s"]) if rows else None


def plot(run, output, audit_path):
    run, output, audit_path = Path(run).resolve(), Path(output).resolve(), Path(audit_path).resolve()
    if output.exists():
        raise FileExistsError("choose a new Toyota briefing revision directory")
    manifest = read(run / "manifest.json")
    audit = read(audit_path)
    if manifest.get("status") != "completed" or audit.get("status") != "passed":
        raise ValueError("a completed run and passed audit are required")
    for entry in manifest["outputs"]:
        if digest(run / entry["path"]) != entry["sha256"]:
            raise ValueError(f"archived output changed: {entry['path']}")
    spec = read(run / "resolved_config.json")
    cfg = spec["parameters"]
    summaries = {row["case_id"]: row for row in read(run / "summary.json")}
    traces = {name: read(run / name / "trace.json.gz") for name in CASES}
    observations = {name: read(run / name / "observations.json.gz") for name in CASES}
    logs = {name: read(run / name / "events.json") for name in CASES}
    accepted = {name: [e for e in logs[name] if e["status"] == "transition_accepted"] for name in CASES}
    with np.load(run / "field.npz") as archive:
        field = {key: archive[key].copy() for key in archive.files}
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        frame = independent_reference(archive, manifest, cfg["center_control_step_m"])

    decision = accepted["joint"][0]
    decision_t = float(decision["t_s"])
    q_decision = float(decision["q_m"])
    joint_completion = next(e for e in logs["joint"] if e["status"] == "transition_completed")
    field_index = int(np.argmin(abs(field["q_m"] - q_decision)))
    q_plot = np.linspace(0.0, float(field["q_m"][-1]), 1600)
    stations = np.asarray([[s["x_m"], s["y_m"], s.get("height_m") or 0.0]
                           for s in spec["base_stations"]], dtype=float)
    station_q, station_xy, route_xy = station_projection(frame, stations, q_plot)
    center0 = frame(np.array([0.0]))[0][0]
    decision_center, _, decision_normal, _, _ = frame(np.array([q_decision]))
    decision_center = decision_center[0] - center0[:2]
    decision_normal = decision_normal[0]
    decision_xy = decision_center

    plt.rcParams.update({
        "font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 11,
        "axes.labelsize": 11.5, "axes.titlesize": 13, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
        "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
        "svg.hashsalt": "uam-toyota-briefing-v6", "figure.facecolor": "white",
        "savefig.facecolor": "white", "legend.fontsize": 9,
    })
    output.mkdir(parents=True)
    captions = {}

    def save(fig, name, caption):
        for ext in ("png", "svg"):
            fig.savefig(output / f"{name}.{ext}", dpi=300, bbox_inches="tight", pad_inches=.12,
                        metadata={"Description": caption})
        captions[name] = caption
        plt.close(fig)

    # ------------------------------------------------------------------
    # Figure 1: route locator + one fixed-q candidate plane.
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(16, 8.8))
    grid = fig.add_gridspec(1, 2, left=.055, right=.975, top=.88, bottom=.12,
                            wspace=.18, width_ratios=[1.06, 1.6])
    map_ax = fig.add_subplot(grid[0, 0])
    plane_ax = fig.add_subplot(grid[0, 1])

    route_x, route_y = route_xy[:, 0] / 1000., route_xy[:, 1] / 1000.
    map_ax.plot(route_x, route_y, color=BLUE, lw=2.2, label="Reference route")
    # Show the three broad lane bands only as context; the candidate plane is
    # the quantitative search space used at the marked q location.
    for offset, style, color in [(-900, ":", "#9BB9C7"), (0, "--", "#58646B"),
                                  (900, ":", "#9BB9C7")]:
        c, _, n, _, _ = frame(q_plot)
        xy = c + offset * n - center0[:2]
        map_ax.plot(xy[:, 0] / 1000., xy[:, 1] / 1000., color=color, ls=style, lw=.8, alpha=.8)
    map_ax.scatter(station_xy[:, 0] / 1000., station_xy[:, 1] / 1000., s=36,
                   marker="^", facecolor="white", edgecolor="#4A4A4A", linewidth=.75,
                   zorder=4, label="Base station")
    for i, (xy, q) in enumerate(zip(station_xy, station_q)):
        dx = 0.025 if i % 2 == 0 else -0.025
        dy = 0.025 if i % 3 else -0.035
        map_ax.text(xy[0] / 1000. + dx, xy[1] / 1000. + dy, spec["base_stations"][i]["site_id"],
                    fontsize=6.7, color="#4A4A4A", ha="left" if dx > 0 else "right", va="center")
    map_ax.scatter(*decision_xy / 1000., s=105, color=POLICY["F"], edgecolor="white",
                   linewidth=1.2, zorder=6, label=f"Decision q = {q_decision/1000:.2f} km")
    map_ax.annotate(f"q0 = {q_decision/1000:.2f} km\ncurrent policy F", decision_xy / 1000.,
                    xytext=(10, 13), textcoords="offset points", fontsize=9, color=INK,
                    bbox=dict(boxstyle="round,pad=.25", fc="white", ec="#B64C42", lw=.75))
    map_ax.scatter(route_x[0], route_y[0], marker="o", s=30, color=INK, zorder=5)
    map_ax.scatter(route_x[-1], route_y[-1], marker="s", s=30, color=INK, zorder=5)
    map_ax.set_aspect("equal", adjustable="datalim")
    map_ax.set(xlabel="Easting relative to route origin (km)", ylabel="Northing relative to route origin (km)",
               title="(a) Where the decision occurs")
    map_ax.grid(color="#E5E5E5", lw=.5)
    map_ax.legend(loc="lower left", frameon=True, framealpha=.93, fontsize=8.3)

    values = field["sinr_db"][field_index].T
    norm = TwoSlopeNorm(vmin=min(float(values.min()), cfg["threshold_db"] - .05),
                        vcenter=cfg["threshold_db"], vmax=max(float(values.max()), cfg["threshold_db"] + .05))
    mesh = plane_ax.pcolormesh(field["offsets_m"], field["heights_m"], values, shading="nearest",
                               cmap="RdBu", norm=norm, rasterized=True)
    dm, hm = np.meshgrid(field["offsets_m"], field["heights_m"])
    plane_ax.scatter(dm.ravel(), hm.ravel(), s=9, color="white", edgecolor="#6B6B6B",
                     linewidth=.25, alpha=.65, zorder=2)
    plane_ax.axvline(0, color=INK, lw=.7, ls="--", alpha=.7)
    plane_ax.axhline(cfg["altitude_m"], color=INK, lw=.7, ls="--", alpha=.7)

    # Candidate outcomes are the result of the continuous forecast, not a
    # claim that instantaneous SINR alone determines C/R/F.
    candidates = [
        (0., 300., POLICY["F"], "F", "current"),
        (-100., 300., "#6F7880", "not selected", "lateral reject"),
        (100., 300., POLICY["C"], "C", "selected lateral / joint"),
        (0., 100., POLICY["C"], "C", "vertical alternative"),
        (0., 200., "#6F7880", "not selected", "vertical reject"),
        (0., 400., "#6F7880", "not selected", "vertical reject"),
    ]
    for d, h, color, result, label in candidates:
        if label == "current":
            marker, size = "+", 170
            edge = INK
        elif result == "C":
            marker, size, edge = "o", 92, INK
        else:
            marker, size, edge = "x", 70, color
        plane_ax.scatter(d, h, marker=marker, s=size, color=color, edgecolor=edge,
                         linewidth=1.5, zorder=6)
        if label == "current":
            text = "current: F"
            xytext = (9, 10)
        elif label == "selected lateral / joint":
            text = "+100 m: C\nselected"
            xytext = (9, -22)
        elif label == "vertical alternative":
            text = "h=100 m: C\nvertical alternative"
            xytext = (9, -26)
        else:
            text = f"{label}: {result}"
            xytext = (7, 7)
        plane_ax.annotate(text, (d, h), xytext=xytext, textcoords="offset points", fontsize=8.1,
                          color=INK, bbox=dict(boxstyle="round,pad=.16", fc="white", ec="none", alpha=.82))
    plane_ax.set(xlabel="Signed lateral offset, d (m)", ylabel="Altitude, h (m)",
               title=f"(b) Candidate plane at the same longitudinal position, q0 = {q_decision/1000:.2f} km")
    plane_ax.set_xticks(field["offsets_m"]); plane_ax.set_yticks(field["heights_m"])
    plane_ax.tick_params(axis="x", rotation=35, labelsize=8.5)
    plane_ax.grid(color="white", lw=.4, alpha=.5)
    plane_ax.legend(handles=[Line2D([], [], marker="+", color=POLICY["F"], linestyle="None", markersize=10, label="Current state: F"),
                             Line2D([], [], marker="o", color=POLICY["C"], markeredgecolor=INK, linestyle="None", markersize=7, label="Predicted stable C"),
                             Line2D([], [], marker="x", color="#6F7880", linestyle="None", markersize=7, label="Not selected / no stable C")],
                    loc="upper left", frameon=True, framealpha=.93, fontsize=8.2)
    cb = fig.colorbar(mesh, ax=plane_ax, pad=.025, fraction=.046)
    cb.set_label("Instantaneous SINR (dB)")

    # Two connector lines make the geographic locator and the candidate plane
    # read as one decision, rather than two unrelated plots.
    for target in [(0., 300.), (100., 300.)]:
        connector = ConnectionPatch(xyA=target, coordsA=plane_ax.transData,
                                    xyB=decision_xy / 1000., coordsB=map_ax.transData,
                                    color="#7D858A", lw=.85, ls=(0, (3, 2)), alpha=.9)
        connector.set_clip_on(False)
        fig.add_artist(connector)
    fig.suptitle("Figure 1. One policy-degradation decision: location, link quality and candidate outcomes",
                 fontsize=17, y=.955)
    fig.text(.5, .045,
             "The heatmap is instantaneous SINR at fixed q0. Candidate labels are the outcome of the local-group exposure and continuous-motion forecast; the +100 m lateral move is the smallest candidate that provides stable C recovery.",
             ha="center", fontsize=10.2, color="#4A5961")
    save(fig, "01-decision-location-and-candidate-plane",
         "Decision-centered R0010 Figure 1. Panel (a) locates the common longitudinal decision q0 on the full corridor with all archived base stations. Panel (b) is the fixed-q offset/height candidate plane; heatmap colour is instantaneous SINR, while candidate labels report the policy outcome of continuous forecast checks based on local-group exposure. The current state is F and the +100 m lateral candidate is selected because it is the smallest candidate with stable C recovery. The connector lines link the geographic decision location to the local candidate plane.")

    # ------------------------------------------------------------------
    # Figure 2: candidate forecast + full-corridor before/after outcome.
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(16, 10.8))
    grid = fig.add_gridspec(2, 2, left=.065, right=.975, top=.89, bottom=.095,
                            hspace=.36, wspace=.24, height_ratios=[1.2, 1.0])
    candidate_ax = fig.add_subplot(grid[0, 0], projection="3d")
    timeline_ax = fig.add_subplot(grid[0, 1])
    path_ax = fig.add_subplot(grid[1, 0])
    policy_ax = fig.add_subplot(grid[1, 1])

    # Candidate paths use the current speed and quintic endpoint motion. The
    # selected joint path is overlaid with the realized archived trajectory.
    t_specs = [
        ("Stay at (0,300)", 0., 300., float(decision["stay_recovery_time_s"] - decision_t), "stay", GREY, "C at 575 s"),
        ("d=-100 m", -100., 300., first_duration(logs["lateral"], decision_t, [-100., 300.]), "reject", "#8B9398", "no stable C"),
        ("d=+100 m", 100., 300., float(accepted["lateral"][0]["duration_s"]), "selected", POLICY["C"], "stable C; selected"),
        ("h=100 m", 0., 100., float(accepted["vertical"][0]["duration_s"]), "alternative", POLICY["C"], "stable C; vertical"),
        ("h=200 m", 0., 200., first_duration(logs["vertical"], decision_t, [0., 200.]), "reject", "#8B9398", "no stable C"),
    ]
    for label, target_d, target_h, duration, kind, color, result in t_specs:
        if duration is None:
            continue
        duration = float(duration)
        times = np.linspace(0., duration, 80)
        u = times / duration if duration else np.zeros(1)
        progress = q_decision + cfg["cruise_mps"] * times
        d = target_d * quintic(u)
        h = cfg["altitude_m"] + (target_h - cfg["altitude_m"]) * quintic(u)
        ls = "-" if kind in {"selected", "alternative"} else "--"
        lw = 2.8 if kind == "selected" else 1.15
        alpha = .98 if kind in {"selected", "alternative"} else .62
        candidate_ax.plot(progress / 1000., d, h, color=color, lw=lw, ls=ls, alpha=alpha)
        candidate_ax.scatter(progress[-1] / 1000., d[-1], h[-1], color=color, s=28 if kind != "selected" else 48,
                             edgecolor=INK, linewidth=.55, zorder=7)
        # Keep endpoint labels short and separated in the 3-D projection. The
        # policy forecast panel carries the complete explanation for each lane.
        short = {
            "stay": "stay: C at 575 s",
            "reject": f"{label}: no stable C",
            "selected": "d=+100 m: C (selected)",
            "alternative": "h=100 m: C",
        }[kind]
        q_shift, d_shift, h_shift = {
            "stay": (.10, 0., 15.),
            "reject": (-.18, -2., -18.),
            "selected": (.06, 2., 18.),
            "alternative": (.05, -15., -12.),
        }[kind]
        candidate_ax.text(progress[-1] / 1000. + q_shift, d[-1] + d_shift, h[-1] + h_shift,
                          short, fontsize=7.2, color=INK, zorder=9)
    # Actual selected joint trace, with policy colours.
    actual = [r for r in traces["joint"] if decision_t <= r["t_s"] <= decision_t + accepted["joint"][0]["duration_s"] + 1e-8]
    xyz = np.asarray([[r["q_m"] / 1000., r["offset_m"], r["altitude_m"]] for r in actual])
    if len(xyz) > 1:
        seg = np.stack([xyz[:-1], xyz[1:]], axis=1)
        candidate_ax.add_collection3d(Line3DCollection(seg, colors=[POLICY[r["policy"]] for r in actual[:-1]], linewidths=3.7))
    candidate_ax.scatter(q_decision / 1000., 0., 300., color=POLICY["F"], s=58, edgecolor="white", linewidth=.9)
    candidate_ax.text(q_decision / 1000., 0., 300., " current F", fontsize=8.5)
    candidate_ax.set(xlabel="Route progress q (km)", ylabel="Offset d (m)", zlabel="Altitude h (m)",
                     title="(a) Candidate motion forecast")
    candidate_ax.set_xlim(q_decision / 1000. - .15, q_decision / 1000. + 7.1)
    candidate_ax.set_ylim(-125, 125); candidate_ax.set_zlim(70, 325)
    candidate_ax.set_box_aspect([7.2, 2.1, 2.6]); candidate_ax.view_init(elev=23, azim=-63); candidate_ax.set_proj_type("ortho")
    candidate_ax.legend(handles=[Line2D([], [], color=POLICY["C"], lw=3, label="Selected / stable C"),
                                 Line2D([], [], color="#8B9398", lw=1.5, ls="--", label="Candidate not selected"),
                                 Line2D([], [], color=POLICY["F"], marker="o", lw=0, label="Current policy F")],
                        loc="upper left", bbox_to_anchor=(0., .98), frameon=False, fontsize=8.2)

    # Policy forecast lanes: candidate movement and post-completion policy
    # are aligned to the same decision clock.
    lane_specs = [
        ("Stay", float(decision["stay_recovery_time_s"]), None, "stable C at 575 s"),
        ("d=+100 m", float(accepted["lateral"][0]["recovery_time_s"]), float(accepted["lateral"][0]["t_s"] + accepted["lateral"][0]["duration_s"]), "selected; C at 570 s"),
        ("h=100 m", float(accepted["vertical"][0]["recovery_time_s"]), float(accepted["vertical"][0]["t_s"] + accepted["vertical"][0]["duration_s"]), "vertical; C at 570 s"),
        ("d=-100 m", None, decision_t + float(t_specs[1][3] or 0.), "no stable C"),
        ("h=200 m", None, decision_t + float(t_specs[4][3] or 0.), "no stable C"),
    ]
    timeline_ax.axvline(decision_t, color=INK, lw=.8, ls="--")
    timeline_ax.text(decision_t + 1.5, -0.62, "F decision", fontsize=8, color=INK)
    for i, (label, stable, completion, result) in enumerate(lane_specs):
        end = stable if stable is not None else (completion or decision_t + 155.)
        timeline_ax.barh(i, max(0., end - decision_t), left=decision_t, height=.54,
                         color=POLICY["F"], alpha=.95 if stable is not None else .63,
                         hatch=None if stable is not None else "///", edgecolor="white", linewidth=.4)
        if stable is not None:
            timeline_ax.barh(i, 630. - stable, left=stable, height=.54, color=POLICY["C"], edgecolor="none")
            timeline_ax.scatter(stable, i, s=22, color=POLICY["C"], edgecolor=INK, linewidth=.5, zorder=5)
            if completion is not None:
                timeline_ax.scatter(completion, i, s=24, marker="o", facecolor="white", edgecolor=BLUE, linewidth=1., zorder=6)
        label_x = 608. if stable is not None else (end + 3.)
        timeline_ax.text(label_x, i, result, va="center", fontsize=7.8)
    timeline_ax.set(yticks=range(len(lane_specs)), yticklabels=[x[0] for x in lane_specs],
                    xlim=(decision_t - 18., 630.), ylim=(len(lane_specs) - .4, -.7),
                    xlabel="Time since departure (s)", title="(b) Candidate policy forecast")
    timeline_ax.grid(axis="x", color="#E5E5E5", lw=.5)
    timeline_ax.legend(handles=[Patch(color=POLICY["F"], label="Predicted F"), Patch(color=POLICY["C"], label="Stable C"),
                                Line2D([], [], marker="o", color="none", mfc="white", mec=BLUE, label="Maneuver completion")],
                       loc="lower right", frameon=False, fontsize=8)
    timeline_ax.text(.02, -.18, "Each lane is evaluated from the same current state. Acceptance requires stable C after the continuous move, not a favourable instantaneous SINR alone.",
                     transform=timeline_ax.transAxes, fontsize=8.1, color="#4A5961", va="top")

    # Full-corridor before/after path, with the common q axis.
    fixed_rows, joint_rows = traces["fixed"], traces["joint"]
    qf = np.asarray([r["q_m"] / 1000. for r in fixed_rows])
    qj = np.asarray([r["q_m"] / 1000. for r in joint_rows])
    path_ax.plot(qf, np.zeros_like(qf), color="#727272", ls="--", lw=1.5, label="No-move reference")
    for lo, hi, colors in zip(*segments_by_policy(joint_rows)):
        path_ax.plot([lo[0], hi[0]], [lo[1], hi[1]], color=colors, lw=2.4)
    path_ax.axvline(q_decision / 1000., color=INK, lw=.8, ls=":")
    q_complete = float(joint_completion["q_m"])
    path_ax.axvline(q_complete / 1000., color=BLUE, lw=.8, ls=":")
    path_ax.scatter(q_decision / 1000., 0., color=POLICY["F"], s=42, edgecolor="white", zorder=5)
    path_ax.scatter(q_complete / 1000., 100., color=POLICY["C"], s=42, edgecolor="white", zorder=5)
    path_ax.annotate("decision", (q_decision / 1000., 0.), xytext=(5, 9), textcoords="offset points", fontsize=8)
    path_ax.annotate("+100 m complete", (q_complete / 1000., 100.), xytext=(5, 9), textcoords="offset points", fontsize=8)
    path_ax.set(xlabel="Reference-route progress q (km)", ylabel="Lateral offset d (m)",
                title="(c) Full-corridor trajectory before and after the decision")
    path_ax.grid(axis="both", color="#E5E5E5", lw=.5)
    path_ax.legend(handles=[Line2D([], [], color="#727272", ls="--", label="No move"),
                            Line2D([], [], color=POLICY["F"], lw=2.5, label="F segment"),
                            Line2D([], [], color=POLICY["C"], lw=2.5, label="C segment")],
                   loc="upper right", frameon=False, fontsize=8.2)

    # A compact policy strip and outcome metrics are more useful than another
    # dense line chart for the second figure.
    for row_index, (label, name) in enumerate([("No move", "fixed"), ("Selected joint", "joint")]):
        rows = traces[name]
        q = np.asarray([r["q_m"] / 1000. for r in rows])
        for i, r in enumerate(rows[:-1]):
            policy_ax.broken_barh([(q[i], q[i + 1] - q[i])], (row_index - .28, .56),
                                  facecolors=POLICY.get(r["policy"], GREY), edgecolors="none", rasterized=True)
    policy_ax.axvline(q_decision / 1000., color=INK, lw=.8, ls=":")
    policy_ax.set(yticks=[0, 1], yticklabels=["No move", "Selected joint"], ylim=(1.55, -.55),
                  xlabel="Reference-route progress q (km)", title="(d) Policy state along the corridor")
    policy_ax.grid(axis="x", color="#E5E5E5", lw=.5)
    f_fixed = summaries["fixed"]["policy_time_s"]["F"]
    f_joint = summaries["joint"]["policy_time_s"]["F"]
    policy_ax.text(.02, -.28,
                   f"One accepted move  |  F time: {f_fixed:.0f} → {f_joint:.0f} s  |  endpoint: d=+100 m, h=300 m",
                   transform=policy_ax.transAxes, fontsize=8.7, color="#4A5961", va="top")
    policy_ax.legend(handles=[Patch(color=POLICY["C"], label="C"), Patch(color=POLICY["R"], label="R"), Patch(color=POLICY["F"], label="F")],
                     loc="upper right", frameon=False, ncol=3, fontsize=8.1)
    fig.suptitle("Figure 2. Candidate motion prediction and realized full-corridor outcome", fontsize=17, y=.955)
    fig.text(.5, .035,
             "The same decision node is followed from candidate endpoints to a bounded continuous move. The realized joint controller selects one +100 m lateral maneuver; the unchanged reference is retained to show the policy and trajectory difference.",
             ha="center", fontsize=10.2, color="#4A5961")
    save(fig, "02-candidate-motion-and-corridor-outcome",
         "Decision-centered R0010 Figure 2. Panel (a) shows candidate endpoint motions from the common state using the current cruise speed and quintic transverse motion; the selected joint path is overlaid with the realized archive. Panel (b) aligns candidate movement with the predicted C/R/F policy gate. Panel (c) compares the unchanged reference and the realized selected path over the full corridor. Panel (d) displays the exposure-derived policy state before and after the maneuver. The selected move is one +100 m lateral transition, reducing full-corridor F time from the fixed reference while preserving the bounded-motion rule.")

    (output / "captions.md").write_text(
        "# Toyota briefing revision-09 captions\n\n" +
        "\n\n".join(f"## {key}\n\n{value}" for key, value in captions.items()) + "\n"
    )
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), Path(__file__).with_name("verify_lateral_study.py")]
    with zipfile.ZipFile(output / "plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(root))
    record = {
        "revision": "09",
        "run": str(run),
        "run_manifest_sha256": digest(run / "manifest.json"),
        "audit_sha256": digest(audit_path),
        "figures": list(captions),
        "decision_time_s": decision_t,
        "decision_q_m": q_decision,
        "current_policy": "F",
        "accepted_targets": {name: accepted[name][0]["target"] for name in ["lateral", "vertical", "joint"]},
        "figure_order": ["01-decision-location-and-candidate-plane", "02-candidate-motion-and-corridor-outcome"],
        "sources": [{"path": p.relative_to(root).as_posix(), "sha256": digest(p)} for p in sources],
        "outputs": [{"path": p.name, "sha256": digest(p)} for p in sorted(output.iterdir()) if p.name != "manifest.json"],
    }
    (output / "manifest.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"output": str(output), "figures": list(captions)}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    args = parser.parse_args()
    plot(args.run, args.output, args.audit)
