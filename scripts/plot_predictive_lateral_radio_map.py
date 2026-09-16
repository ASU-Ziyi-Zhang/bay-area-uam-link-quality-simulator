"""Plot the rolling-prediction Step-1 lateral study in the approved map style."""
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
from matplotlib.patches import ConnectionPatch, Rectangle
import matplotlib.patheffects as pe
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = Path(__file__).resolve().parent
for folder in (ROOT / "src", SCRIPTS):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from capacity_policy import load_scenario  # noqa: E402
from plot_dual_horizon_briefing import forecast_branch  # noqa: E402
from plot_wide_radio_map import chord_frame, display_bounds, radio_at  # noqa: E402
from uam_simulator.lateral_study import SmoothCorridorFrame  # noqa: E402
from uam_simulator.policy_motion import BSRadio  # noqa: E402
from uam_simulator.spatial_study import SpatialConfig  # noqa: E402
from verify_lateral_study import independent_reference, read  # noqa: E402


INK = "#202020"
GREY = "#646464"
GOLD = "#F2BD3C"
GREEN = "#198577"
RED = "#B64C42"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def branch_xy(rows, frame):
    q = np.asarray([row["q_m"] for row in rows])
    d = np.asarray([row["d_m"] for row in rows])
    center, _, normal, _, _ = frame(q)
    return center + d[:, None] * normal


def load_inputs(run, audit_path, case_id):
    run, audit_path = Path(run).resolve(), Path(audit_path).resolve()
    manifest, audit = read(run / "manifest.json"), read(audit_path)
    if manifest["status"] != "completed" or audit["status"] != "passed":
        raise ValueError("completed run and passed audit required")
    if audit["run_manifest_sha256"] != sha(run / "manifest.json"):
        raise ValueError("audit does not identify this immutable run")
    for item in manifest["outputs"]:
        if sha(run / item["path"]) != item["sha256"]:
            raise ValueError(f"run hash mismatch: {item['path']}")
    spec = read(run / "resolved_config.json")
    case = next(c for c in spec["cases"] if c["id"] == case_id)
    if case["mode"] != "lateral" or spec["heights_m"] != [case["parameters"]["altitude_m"]]:
        raise ValueError("Step-1 figure requires a fixed-altitude lateral-only case")
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        for item in manifest["code"] + manifest["inputs"]:
            if hashlib.sha256(archive.read(item["path"])).hexdigest() != item["sha256"]:
                raise ValueError(f"source archive mismatch: {item['path']}")
        frame = independent_reference(archive, manifest, case["parameters"]["center_control_step_m"])
    scenario_item = next(item for item in manifest["inputs"] if item["path"].endswith("scenario.json"))
    scenario = load_scenario(ROOT / scenario_item["path"])
    trace = read(run / case_id / "trace.json.gz")
    terminal = read(run / case_id / "terminal.json")
    events = read(run / case_id / "events.json")
    observations = read(run / case_id / "observations.json.gz")
    summary = read(run / case_id / "summary.json")
    fixed = read(run / "fixed_reference" / "summary.json")
    accepted = [event for event in events if event["status"] == "transition_accepted"]
    completed = [event for event in events if event["status"] == "transition_completed"]
    if len(accepted) != 2 or len(completed) != 2:
        raise ValueError("the Step-1 predictive figure expects two completed lateral moves")
    decision = accepted[0]
    t0, q0 = float(decision["t_s"]), float(decision["q_m"])
    cfg = SpatialConfig(**summary["parameters"])
    history_rows = [row for row in observations
                    if t0 - cfg.window_s - 1e-8 <= row["t_s"] <= t0 + 1e-8]
    history = [(float(row["t_s"]), int(row["sinr_db"] < cfg.threshold_db))
               for row in history_rows]
    forecast_frame = SmoothCorridorFrame(scenario.corridor, cfg.center_control_step_m)
    radio = BSRadio(scenario.base_stations, scenario.radio)
    source, target = tuple(decision["source"]), tuple(decision["target"])
    horizon = float(decision["planning_horizon_s"])
    stay, _ = forecast_branch(forecast_frame, radio, cfg, q=q0, t=t0,
        source=source, target=source, duration=0.0, history=history, horizon=horizon)
    move, _ = forecast_branch(forecast_frame, radio, cfg, q=q0, t=t0,
        source=source, target=target, duration=decision["duration_s"],
        history=history, horizon=horizon)
    warning_end = t0 + float(decision["warning_horizon_s"])
    stay60 = [row for row in stay if row["t_s"] <= warning_end + 1e-8]
    move60 = [row for row in move if row["t_s"] <= warning_end + 1e-8]
    if stay60[-1]["policy"] != "F" or move60[-1]["policy"] != "C":
        raise ValueError("archived first decision no longer matches the documented F-to-C forecast")
    trace_xyz = np.asarray([row["position_m"] for row in trace] + [terminal["position_m"]])
    trace_t = np.asarray([row["t_s"] for row in trace] + [terminal["t_s"]])
    return {
        "run": run, "audit": audit_path, "manifest": manifest, "spec": spec,
        "case": case, "scenario": scenario, "frame": frame, "cfg": cfg,
        "trace_xyz": trace_xyz, "trace_t": trace_t, "events": accepted,
        "completed": completed, "summary": summary, "fixed": fixed,
        "decision": decision, "stay60": stay60, "move60": move60,
    }


def plot(run, output, audit_path, case_id="rolling_60plus60_lateral", grid_m=50.0):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError("choose a new figure revision directory")
    if not np.isfinite(grid_m) or grid_m < 10:
        raise ValueError("display grid must be finite and at least 10 m")
    data = load_inputs(run, audit_path, case_id)
    spec, cfg, frame = data["spec"], data["cfg"], data["frame"]
    scenario = data["scenario"]
    length = scenario.corridor.length_m
    origin, rotation = chord_frame(frame(0.0)[0], frame(length)[0])
    local = lambda xy: (np.asarray(xy) - origin) @ rotation

    q = np.linspace(0.0, length, int(np.ceil(length / 10.0)) + 1)
    center, _, normal, _, _ = frame(q)
    width = float(max(abs(np.asarray(spec["offsets_m"], float))))
    lower_band = local(center - width * normal)
    upper_band = local(center + width * normal)
    reference = local(center)
    trajectory = local(data["trace_xyz"][:, :2])
    stations = np.asarray([[site["x_m"], site["y_m"]] for site in spec["base_stations"]])
    ground = local(stations)
    lo, hi = display_bounds(np.vstack((ground, reference, trajectory, lower_band, upper_band)))
    xe = np.arange(lo[0], hi[0] + grid_m * .999, grid_m)
    ye = np.arange(lo[1], hi[1] + grid_m * .999, grid_m)
    hi = np.asarray([xe[-1], ye[-1]])
    xx, yy = np.meshgrid((xe[:-1] + xe[1:]) / 2, (ye[:-1] + ye[1:]) / 2)
    real_xy = origin + np.stack((xx, yy), axis=-1) @ rotation.T
    positions = np.concatenate((real_xy, np.full((*xx.shape, 1), cfg.altitude_m)), axis=-1)
    display_radio = radio_at(positions, spec)
    z = display_radio["sinr_db"]

    stay_xy = local(branch_xy(data["stay60"], frame))
    move_xy = local(branch_xy(data["move60"], frame))
    focus_points = np.vstack((stay_xy, move_xy))
    # Keep enough room for the forecast labels without allowing the detail
    # panel to dominate the full-corridor overview.  The axes remain equal in
    # physical scale; this changes only the displayed window and panel size.
    focus_lo = np.floor((focus_points.min(axis=0) - [250, 180]) / 100) * 100
    focus_hi = np.ceil((focus_points.max(axis=0) + [250, 180]) / 100) * 100
    focus_xe = np.arange(focus_lo[0], focus_hi[0] + 10, 10.0)
    focus_ye = np.arange(focus_lo[1], focus_hi[1] + 10, 10.0)
    fxx, fyy = np.meshgrid((focus_xe[:-1] + focus_xe[1:]) / 2,
                           (focus_ye[:-1] + focus_ye[1:]) / 2)
    focus_real = origin + np.stack((fxx, fyy), axis=-1) @ rotation.T
    focus_radio = radio_at(np.concatenate((focus_real,
        np.full((*fxx.shape, 1), cfg.altitude_m)), axis=-1), spec)

    plt.rcParams.update({
        "font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 12,
        "axes.labelsize": 13, "axes.titlesize": 15, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
        "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
        "svg.hashsalt": "uam-predictive-lateral-r0017", "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })
    plot_width = 14.2
    plot_height = plot_width * (hi[1] - lo[1]) / (hi[0] - lo[0])
    focus_width = 10.5
    focus_height = focus_width * (focus_hi[1] - focus_lo[1]) / (focus_hi[0] - focus_lo[0])
    extra_height = focus_height + 1.35
    fig_width, fig_height = 16.2, plot_height + 1.7 + extra_height
    fig = plt.figure(figsize=(fig_width, fig_height))
    ax = fig.add_axes([.85 / fig_width, (.65 + extra_height) / fig_height,
                       plot_width / fig_width, plot_height / fig_height])
    cax = fig.add_axes([15.25 / fig_width, (.65 + extra_height) / fig_height,
                        .15 / fig_width, plot_height / fig_height])
    focus_left = (fig_width - focus_width) / 2
    zoom = fig.add_axes([focus_left / fig_width, .65 / fig_height,
                         focus_width / fig_width, focus_height / fig_height])
    norm = TwoSlopeNorm(vmin=float(np.floor(min(z.min(), focus_radio["sinr_db"].min()))),
                        vcenter=cfg.threshold_db,
                        vmax=float(np.ceil(max(z.max(), focus_radio["sinr_db"].max()))))
    mesh = ax.pcolormesh(xe / 1000, ye / 1000, z, cmap="RdBu", norm=norm,
                         shading="flat", rasterized=True)
    for border in (lower_band, upper_band):
        ax.plot(border[:, 0] / 1000, border[:, 1] / 1000, color=GREY,
                lw=.8, ls=(0, (5, 3)))
    for index in (0, -1):
        cap = np.vstack((lower_band[index], upper_band[index])) / 1000
        ax.plot(cap[:, 0], cap[:, 1], color=GREY, lw=.8, ls=(0, (5, 3)))
    ax.plot(trajectory[:, 0] / 1000, trajectory[:, 1] / 1000, color=GOLD, lw=2.1,
            path_effects=[pe.Stroke(linewidth=3.3, foreground=INK), pe.Normal()], zorder=5)
    ax.plot(reference[:, 0] / 1000, reference[:, 1] / 1000, color=INK,
            lw=.85, ls=(0, (4, 4)), zorder=6)
    ax.scatter(ground[:, 0] / 1000, ground[:, 1] / 1000, marker="^", s=44,
               facecolor="white", edgecolor=INK, linewidth=.9, zorder=7)
    label_offsets = {"BS05": (-3, 14), "BS06": (0, -17), "BS07": (-7, 16),
        "BS08": (0, -19), "BS09": (0, -17), "BS10": (0, 15),
        "BS11": (-13, 17), "BS12": (14, -18), "BS13": (-16, -19),
        "BS14": (7, 17), "BS15": (0, 17), "BS16": (-3, -18)}
    for site, point in zip(spec["base_stations"], ground / 1000):
        ax.annotate(site["site_id"], point, xytext=label_offsets.get(site["site_id"], (0, 15)),
            textcoords="offset points", ha="center", va="center", fontsize=11,
            arrowprops={"arrowstyle": "-", "lw": .5, "color": INK},
            bbox={"facecolor": "white", "alpha": .88, "edgecolor": "none", "pad": .5})
    for number, event in enumerate(data["events"], 1):
        c, _, n, _, _ = frame(np.asarray([event["q_m"]]))
        decision_xy = local(c[0] + event["source"][0] * n[0]) / 1000
        ax.scatter(*decision_xy, s=45, marker="o", color=RED, edgecolor="white",
                   linewidth=.8, zorder=9)
        ax.annotate(f"D{number}", decision_xy, xytext=(6, 7), textcoords="offset points",
                    fontsize=10.5, color=RED,
                    bbox={"facecolor": "white", "alpha": .88, "edgecolor": "none", "pad": .4})
    ax.set(xlim=(lo[0] / 1000, hi[0] / 1000), ylim=(lo[1] / 1000, hi[1] / 1000),
           xlabel=r"Along-chord coordinate, $X$ (km)",
           ylabel=r"Cross-chord coordinate, $Y$ (km)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(r"(a) Predictive lateral trajectory and modeled SINR ($h=300$ m)",
                 loc="left", pad=44)
    ax.legend(handles=[
        Line2D([], [], color=INK, ls="--", lw=.9, label="Reference route"),
        Line2D([], [], color=GOLD, lw=2.1, label="Rolling 60+60 s trajectory",
               path_effects=[pe.Stroke(linewidth=3.3, foreground=INK), pe.Normal()]),
        Line2D([], [], color=GREY, lw=.8, ls=(0, (5, 3)),
               label=rf"Candidate envelope ($\pm${width / 1000:g} km)"),
        Line2D([], [], color="none", marker="^", mfc="white", mec=INK, ms=7,
               label="Base station"),
        Line2D([], [], color="none", marker="o", mfc=RED, mec="white", ms=7,
               label="Accepted decision"),
    ], loc="lower left", bbox_to_anchor=(0, 1.04), borderaxespad=0,
       ncol=5, frameon=False, fontsize=11.2, columnspacing=1.8, handlelength=2.5)
    cb = fig.colorbar(mesh, cax=cax)
    cb.set_label("SINR (dB)", labelpad=9)
    ticks = sorted(set(t for t in [norm.vmin, cfg.threshold_db, 5, 15, 25, norm.vmax]
                       if norm.vmin <= t <= norm.vmax))
    cb.set_ticks(ticks)
    cb.set_ticklabels([f"{value:g}" for value in ticks])

    zoom.pcolormesh(focus_xe / 1000, focus_ye / 1000, focus_radio["sinr_db"],
                    cmap="RdBu", norm=norm, shading="flat", rasterized=True)
    q_focus = np.linspace(max(0.0, data["decision"]["q_m"] - 400),
                          min(length, data["stay60"][-1]["q_m"] + 400), 800)
    focus_ref = local(frame(q_focus)[0])
    zoom.plot(focus_ref[:, 0] / 1000, focus_ref[:, 1] / 1000,
              color=INK, lw=.9, ls=(0, (4, 4)))
    zoom.plot(stay_xy[:, 0] / 1000, stay_xy[:, 1] / 1000,
              color=RED, lw=2.2, ls=(0, (5, 3)), label="No-action forecast")
    zoom.plot(move_xy[:, 0] / 1000, move_xy[:, 1] / 1000,
              color=GREEN, lw=2.8, label="Selected forecast")
    current = move_xy[0] / 1000
    stay_end = stay_xy[-1] / 1000
    move_end = move_xy[-1] / 1000
    zoom.scatter(*current, s=58, color=RED, edgecolor="white", linewidth=.8, zorder=8)
    zoom.scatter(*stay_end, marker="x", s=58, color=RED, linewidth=1.8, zorder=8)
    zoom.scatter(*move_end, s=52, color=GREEN, edgecolor="white", linewidth=.8, zorder=8)
    completion = data["completed"][0]
    completion_index = int(np.argmin(abs(data["trace_t"] - completion["t_s"])))
    completion_xy = local(data["trace_xyz"][completion_index, :2]) / 1000
    zoom.scatter(*completion_xy, marker="s", s=38, facecolor="white",
                 edgecolor=GREEN, linewidth=1.1, zorder=8)
    zoom.annotate("Current C", current, xytext=(4, 10), textcoords="offset points",
                  fontsize=10.5, ha="left",
                  bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .5})
    zoom.annotate("Stay at +60 s: F", stay_end, xytext=(7, 12), textcoords="offset points",
                  fontsize=10.5, color=RED, ha="left", va="bottom",
                  bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .5})
    zoom.annotate(r"$d=-100$ m at +60 s: C", move_end, xytext=(-5, -20),
                  textcoords="offset points", fontsize=10.5, color=GREEN, ha="right", va="top",
                  bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .5})
    zoom.annotate("Maneuver complete\n43.9 s", completion_xy, xytext=(8, 12),
                  textcoords="offset points", fontsize=9.8, color=GREEN, ha="left", va="bottom",
                  bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .5})
    zoom.set(xlim=(focus_lo[0] / 1000, focus_hi[0] / 1000),
             ylim=(focus_lo[1] / 1000, focus_hi[1] / 1000),
             xlabel=r"Along-chord coordinate, $X$ (km)",
             ylabel=r"Cross-chord coordinate, $Y$ (km)")
    zoom.set_aspect("equal", adjustable="box")
    zoom.set_title("(b) First rolling decision: forecast degradation and lateral upgrade",
                   loc="center", pad=11)
    zoom.legend(loc="lower left", frameon=True, fontsize=10.2, ncol=2)

    roi = Rectangle(focus_lo / 1000, *(focus_hi - focus_lo) / 1000,
                    fill=False, edgecolor=INK, lw=1.05, linestyle=(0, (4, 3)), zorder=10)
    ax.add_patch(roi)
    for x, corner in [(focus_lo[0], 0.0), (focus_hi[0], 1.0)]:
        fig.add_artist(ConnectionPatch(xyA=(x / 1000, focus_lo[1] / 1000),
            coordsA=ax.transData, xyB=(corner, 1), coordsB=zoom.transAxes,
            color=GREY, lw=.7, linestyle=(0, (4, 4)), clip_on=False, zorder=.1))

    fig.canvas.draw()
    def metric_ratio(axis):
        p0, px, py = axis.transData.transform([[0, 0], [1, 0], [0, 1]])
        return float(np.linalg.norm(px - p0) / np.linalg.norm(py - p0))
    overview_ratio, focus_ratio = metric_ratio(ax), metric_ratio(zoom)
    if abs(overview_ratio - 1.0) > 1e-10 or abs(focus_ratio - 1.0) > 1e-10:
        plt.close(fig)
        raise ValueError(f"metric scale check failed: {overview_ratio}, {focus_ratio}")

    output.mkdir(parents=True)
    fixed_f = data["fixed"]["policy_time_s"]["F"]
    rolling_f = data["summary"]["policy_time_s"]["F"]
    caption = (
        "Rolling-prediction Step-1 lateral study at fixed 300 m altitude. Panel (a) uses "
        "an equal metric scale to show the complete reference corridor, all 12 archived base "
        "stations, the ±900 m candidate envelope, and the realized 0→−100→+200 m trajectory. "
        "D1 and D2 mark the two accepted decisions. Panel (b) enlarges D1: at t=265 s the "
        "aircraft remains in C, but the 60 s no-action forecast ends in F; the selected −100 m "
        "lateral maneuver completes in 43.869 s and the corresponding 60 s forecast ends in C. "
        f"Across the corridor, F time changes from {fixed_f:g} s without lateral control to "
        f"{rolling_f:g} s with the modular 60 s warning plus 60 s evaluation controller. "
        "The SINR field is modeled rather than measured; the controller is a singleton method "
        "example and includes neither neighboring-aircraft conflict checks nor capacity."
    )
    for extension in ("png", "svg"):
        fig.savefig(output / f"01-radio-context.{extension}", dpi=300,
                    metadata={"Description": caption})
    plt.close(fig)
    (output / "caption.md").write_text(
        "# Figure caption\n\n" + caption + "\n\n"
        "## Reproduction\n\n"
        f"`python scripts/plot_predictive_lateral_radio_map.py "
        f"research/dynamic-transitions/runs/{data['run'].name} "
        f"--audit research/dynamic-transitions/validation-{data['run'].name}.json "
        "--output <new-figure-directory>`\n",
        encoding="utf-8")
    np.savez_compressed(output / "display-field.npz", x_edges_m=xe, y_edges_m=ye,
                        origin_utm_m=origin, rotation=rotation,
                        altitude_m=cfg.altitude_m, **display_radio)
    np.savez_compressed(output / "focus-field.npz", x_edges_m=focus_xe,
                        y_edges_m=focus_ye, origin_utm_m=origin, rotation=rotation,
                        altitude_m=cfg.altitude_m, **focus_radio)
    sources = [Path(__file__).resolve(), SCRIPTS / "plot_wide_radio_map.py",
               SCRIPTS / "plot_dual_horizon_briefing.py",
               SCRIPTS / "verify_spatial_study.py"]
    with zipfile.ZipFile(output / "plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(ROOT))
    report = {
        "run": str(data["run"]), "case": case_id,
        "run_manifest_sha256": sha(data["run"] / "manifest.json"),
        "audit": str(data["audit"]), "audit_sha256": sha(data["audit"]),
        "revision": "predictive-lateral-balanced-panels-v2",
        "controller": {"warning_horizon_s": 60, "evaluation_extension_s": 60},
        "accepted_decisions": len(data["events"]),
        "offset_sequence_m": data["summary"]["offset_height_sequence_m"],
        "fixed_f_time_s": fixed_f, "rolling_f_time_s": rolling_f,
        "display_grid_m": grid_m, "focus_grid_m": 10.0,
        "station_count": len(stations),
        "all_stations_in_frame": bool(np.all((ground >= lo) & (ground <= hi))),
        "rendered_metric_scale": {"overview": overview_ratio, "focus": focus_ratio},
        "sources": [{"path": str(path.relative_to(ROOT)), "sha256": sha(path)}
                    for path in sources],
        "outputs": [{"path": path.name, "sha256": sha(path)}
                    for path in sorted(output.iterdir()) if path.name != "manifest.json"],
        "note": "New presentation rendering of immutable R0017; no controller output changed."
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", default="rolling_60plus60_lateral")
    parser.add_argument("--grid-m", type=float, default=50.0)
    arguments = parser.parse_args()
    plot(arguments.run, arguments.output, arguments.audit, arguments.case, arguments.grid_m)
