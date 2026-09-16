"""Create the confirmed R0016 60+60 s Toyota briefing figure."""
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
from matplotlib.collections import LineCollection
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = Path(__file__).resolve().parent
for folder in (ROOT / "src", SCRIPTS):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from capacity_policy import load_scenario  # noqa: E402
from uam_simulator.lateral_study import SmoothCorridorFrame, offset_values, rk4_progress  # noqa: E402
from uam_simulator.policy_motion import BSRadio, policy_from_exposure  # noqa: E402
from uam_simulator.spatial_study import SpatialConfig, xyz_at  # noqa: E402
from verify_lateral_study import independent_reference, read  # noqa: E402

POLICY = {"C": "#198577", "R": "#E7AF3C", "F": "#B64C42"}
INK = "#202020"
GREY = "#6F7477"
LIGHT = "#D9DEE1"
BLUE = "#2D6688"
RED = "#B33B32"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def policy_from_history(history, cfg):
    exposure = float(np.mean([bad for _, bad in history])) if history else 0.0
    return policy_from_exposure(exposure, cfg.policy_config()), exposure


def forecast_branch(frame, radio, cfg, *, q, t, source, target, duration, history, horizon):
    """Reconstruct the archived planner branch on the policy clock."""
    state, clock = float(q), float(t)
    local_history = list(history)
    points = [{"t_s": clock, "q_m": state, "d_m": source[0], "h_m": source[1],
               "policy": policy_from_history(local_history, cfg)[0]}]
    intervals = []
    while clock < t + horizon - 1e-8:
        step = min(cfg.forecast_dt_s, t + horizon - clock)
        previous_clock = clock
        state = float(rk4_progress(frame, state, clock, step,
            lambda time: offset_values(time, t, source[0], target[0], duration)[0],
            cfg.cruise_mps))
        clock += step
        d = float(offset_values(clock, t, source[0], target[0], duration)[0])
        h = float(offset_values(clock, t, source[1], target[1], duration)[0])
        sinr = float(radio(xyz_at(frame, min(state, frame.length_m), d, h)[None, :])["sinr_db"][0])
        local_history = [(u, bad) for u, bad in local_history
                         if u >= clock - cfg.window_s - 1e-8]
        local_history.append((clock, int(sinr < cfg.threshold_db)))
        policy, exposure = policy_from_history(local_history, cfg)
        points.append({"t_s": clock, "q_m": state, "d_m": d, "h_m": h,
                       "policy": policy, "exposure": exposure, "sinr_db": sinr})
        intervals.append({"start_s": previous_clock-t, "end_s": clock-t, "policy": policy})
    return points, intervals


def colored_path(ax, x, y, policies, linewidth=3.0, zorder=3):
    xy = np.column_stack([x, y])
    collection = LineCollection(np.stack([xy[:-1], xy[1:]], axis=1),
        colors=[POLICY[p] for p in policies[1:]], linewidths=linewidth,
        capstyle="round", zorder=zorder)
    ax.add_collection(collection)
    return collection


def forecast_policy_bar(ax, intervals, y, height=.56):
    for row in intervals:
        ax.broken_barh([(row["start_s"], row["end_s"]-row["start_s"])],
            (y-height/2, height), facecolors=POLICY[row["policy"]], edgecolors="none")


def corridor_policy_bar(ax, rows, y, height=.58):
    q = np.asarray([r["q_m"]/1000 for r in rows])
    for i, row in enumerate(rows[:-1]):
        ax.broken_barh([(q[i], q[i+1]-q[i])], (y-height/2, height),
                       facecolors=POLICY[row["policy"]], edgecolors="none",
                       rasterized=True)


def plot(run, output, audit_path):
    run, output, audit_path = map(lambda p: Path(p).resolve(), (run, output, audit_path))
    if output.exists():
        raise FileExistsError("choose a new figure directory")
    manifest, audit = read(run/"manifest.json"), read(audit_path)
    if manifest["status"] != "completed" or audit["status"] != "passed":
        raise ValueError("completed run and passed audit required")
    for item in manifest["outputs"]:
        if digest(run/item["path"]) != item["sha256"]:
            raise ValueError(f"archived output changed: {item['path']}")

    spec = read(run/"resolved_config.json")
    summaries = {r["case_id"]: r for r in read(run/"summary.json")}
    cases = ["fixed_reference", "rolling_60plus0_joint", "rolling_60plus60_joint"]
    traces = {case: read(run/case/"trace.json.gz") for case in cases}
    observations = {case: read(run/case/"observations.json.gz") for case in cases}
    events = {case: read(run/case/"events.json") for case in cases}
    accepted = [e for e in events["rolling_60plus60_joint"]
                if e["status"] == "transition_accepted"]
    completed = [e for e in events["rolling_60plus60_joint"]
                 if e["status"] == "transition_completed"]
    if len(accepted) != 2 or len(completed) != 2:
        raise ValueError("R0016 briefing expects two accepted and completed transitions")
    # The first decision is the clearest local example: its 100 m maneuver
    # removes the complete forecast F interval.  The global panel still shows
    # both realized decisions and completions.
    decision = accepted[0]
    t0, q0 = float(decision["t_s"]), float(decision["q_m"])
    decision_rows = [e for e in events["rolling_60plus60_joint"]
                     if abs(e.get("t_s", -1)-t0) < 1e-8
                     and e.get("planner") == "rolling_endpoint"]
    stay_event = next(e for e in decision_rows if e["status"] == "rolling_stay_reference")

    cfg = SpatialConfig(**summaries["rolling_60plus60_joint"]["parameters"])
    scenario_record = next(e for e in manifest["inputs"] if e["path"].endswith("scenario.json"))
    scenario = load_scenario(ROOT/scenario_record["path"])
    radio = BSRadio(scenario.base_stations, scenario.radio)
    forecast_frame = SmoothCorridorFrame(scenario.corridor, cfg.center_control_step_m)
    obs = observations["rolling_60plus60_joint"]
    history_rows = [r for r in obs if t0-cfg.window_s-1e-8 <= r["t_s"] <= t0+1e-8]
    history = [(float(r["t_s"]), int(r["sinr_db"] < cfg.threshold_db)) for r in history_rows]
    source, target = tuple(decision["source"]), tuple(decision["target"])
    horizon = float(decision["planning_horizon_s"])
    with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
        frame = independent_reference(archive, manifest, cfg.center_control_step_m)
    stay_points, stay_intervals = forecast_branch(forecast_frame, radio, cfg, q=q0, t=t0,
        source=source, target=source, duration=0., history=history, horizon=horizon)
    move_points, move_intervals = forecast_branch(forecast_frame, radio, cfg, q=q0, t=t0,
        source=source, target=target, duration=decision["duration_s"],
        history=history, horizon=horizon)
    def policy_time(intervals, code):
        return sum(r["end_s"]-r["start_s"] for r in intervals if r["policy"] == code)
    if abs(policy_time(stay_intervals, "F")-stay_event["forecast_policy_time_s"]["F"]) > 1e-8:
        raise ValueError("stay forecast reconstruction mismatch")
    if abs(policy_time(move_intervals, "F")-decision["forecast_policy_time_s"]["F"]) > 1e-8:
        raise ValueError("move forecast reconstruction mismatch")

    with np.load(run/"field.npz") as archive:
        field = {key: archive[key].copy() for key in archive.files}

    plt.rcParams.update({
        "font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 11,
        "axes.labelsize": 11.5, "axes.titlesize": 13, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
        "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
        "svg.hashsalt": "uam-dual-horizon-r0016", "figure.facecolor": "white",
        "savefig.facecolor": "white", "legend.fontsize": 8.5,
    })
    output.mkdir(parents=True)
    captions = {}

    def save(fig, name, caption):
        for ext in ("png", "svg"):
            fig.savefig(output/f"{name}.{ext}", dpi=300, bbox_inches="tight",
                        pad_inches=.12, metadata={"Description": caption})
        captions[name] = caption
        plt.close(fig)

    # Figure 1 follows the previously approved template: full georeferenced
    # corridor and base stations, one enlarged maneuver segment, then the full
    # route-relative trajectory with every realized change marked.
    fig = plt.figure(figsize=(16, 8.7))
    gs = fig.add_gridspec(1, 2, left=.06, right=.97, top=.89, bottom=.11,
                          width_ratios=[1.02, 1.25], wspace=.17)
    map_ax = fig.add_subplot(gs[0, 0])
    plane_ax = fig.add_subplot(gs[0, 1])
    rolling = traces["rolling_60plus60_joint"]
    q = np.asarray([r["q_m"]/1000 for r in rolling])
    d = np.asarray([r["offset_m"] for r in rolling])

    q_map = np.linspace(0, scenario.corridor.length_m, 1500)
    center, _, _, _, _ = frame(q_map)
    origin = center[0]
    route = center-origin
    stations = scenario.base_stations.positions(scenario.radio.assumed_bs_height_m)
    station_xy = stations[:, :2]-origin
    actual_xy = np.asarray([r["position_m"][:2] for r in rolling])-origin
    map_ax.plot(route[:, 0]/1000, route[:, 1]/1000, color="#9AA4AA", lw=1.25,
                label="Reference route")
    map_ax.plot(actual_xy[:, 0]/1000, actual_xy[:, 1]/1000, color=POLICY["C"],
                lw=1.6, alpha=.9, label="Realized trajectory")
    map_ax.scatter(station_xy[:, 0]/1000, station_xy[:, 1]/1000, marker="^", s=28,
                   facecolor="white", edgecolor="#4A4A4A", linewidth=.65, zorder=4,
                   label="Base station")
    for site, xy in zip(spec["base_stations"], station_xy):
        map_ax.annotate(site["site_id"], xy/1000, xytext=(3, 3),
                        textcoords="offset points", fontsize=6.7, color="#4A4A4A")
    for index, move in enumerate(accepted, 1):
        c, _, n, _, _ = frame(np.array([move["q_m"]]))
        xy = c[0]-origin+move["source"][0]*n[0]
        map_ax.scatter(xy[0]/1000, xy[1]/1000, s=72, color=RED,
                       edgecolor="white", linewidth=.9, zorder=6)
        map_ax.annotate(f"Decision {index}", xy/1000, xytext=(6, 7),
                        textcoords="offset points", fontsize=8.2, color=RED)
    map_ax.set_aspect("equal", adjustable="datalim")
    map_ax.set(xlabel="Easting relative to route origin (km)",
               ylabel="Northing relative to route origin (km)",
               title="(a) Corridor, base stations, and maneuver locations")
    map_ax.grid(color=LIGHT, lw=.5)
    map_ax.legend(loc="lower left", frameon=True, fontsize=8)

    warning = float(decision["warning_horizon_s"])
    local_stay = [r for r in stay_points if r["t_s"] <= t0+warning+1e-8]
    local_move = [r for r in move_points if r["t_s"] <= t0+warning+1e-8]
    def map_branch(rows):
        progress = np.array([r["q_m"] for r in rows])
        offsets = np.array([r["d_m"] for r in rows])
        c, _, n, _, _ = frame(progress)
        return c-origin+offsets[:, None]*n
    stay_xy = map_branch(local_stay)
    move_xy = map_branch(local_move)
    inset = map_ax.inset_axes([.49, .64, .49, .31])
    local_mask = (q_map >= q0-350) & (q_map <= local_stay[-1]["q_m"]+350)
    inset.plot(route[local_mask, 0]/1000, route[local_mask, 1]/1000,
               color="#9AA4AA", lw=1.0)
    inset.plot(stay_xy[:, 0]/1000, stay_xy[:, 1]/1000, color=POLICY["F"],
               lw=2.2, ls="--")
    inset.plot(move_xy[:, 0]/1000, move_xy[:, 1]/1000, color=POLICY["C"], lw=2.7)
    inset.scatter(move_xy[0, 0]/1000, move_xy[0, 1]/1000, s=54, color=RED,
                  edgecolor="white", linewidth=.8, zorder=5)
    inset.scatter(stay_xy[-1, 0]/1000, stay_xy[-1, 1]/1000, marker="x", s=44,
                  color=POLICY["F"], linewidth=1.7, zorder=5)
    inset.scatter(move_xy[-1, 0]/1000, move_xy[-1, 1]/1000, s=42,
                  color=POLICY["C"], edgecolor="white", linewidth=.7, zorder=5)
    inset.annotate("Current C", move_xy[0]/1000, xytext=(5, 6),
                   textcoords="offset points", fontsize=7.4)
    inset.annotate("Stay: F", stay_xy[-1]/1000, xytext=(5, -12),
                   textcoords="offset points", fontsize=7.2, color=POLICY["F"])
    inset.annotate("−100 m: C", move_xy[-1]/1000, xytext=(5, 4),
                   textcoords="offset points", fontsize=7.2, color=POLICY["C"])
    inset.set_xticks([]); inset.set_yticks([])
    inset.set_title("Enlarged first 60-s decision segment", fontsize=8.2)
    map_ax.indicate_inset_zoom(inset, edgecolor=GREY, alpha=.75)

    # Reuse the approved warning-horizon candidate-plane visual.  The plane is
    # a spatial cross-section at t+60; the 60+60 controller subsequently uses
    # the common 120 s integrated objective documented in the method slide.
    q_warning = float(local_stay[-1]["q_m"])
    field_index = int(np.argmin(abs(field["q_m"]-q_warning)))
    values = field["sinr_db"][field_index].T
    norm = TwoSlopeNorm(vmin=min(float(values.min()), cfg.threshold_db-.05),
                        vcenter=cfg.threshold_db,
                        vmax=max(float(values.max()), cfg.threshold_db+.05))
    mesh = plane_ax.pcolormesh(field["offsets_m"], field["heights_m"], values,
                               shading="nearest", cmap="RdBu", norm=norm, rasterized=True)
    grid_d, grid_h = np.meshgrid(field["offsets_m"], field["heights_m"])
    plane_ax.scatter(grid_d.ravel(), grid_h.ravel(), s=10, facecolor="white",
                     edgecolor=GREY, linewidth=.35, alpha=.85, zorder=3)
    endpoint_policy = {}
    for row in decision_rows:
        candidate_target = tuple(row["target"])
        branch, _ = forecast_branch(forecast_frame, radio, cfg, q=q0, t=t0,
            source=source, target=candidate_target, duration=row["duration_s"],
            history=history, horizon=warning)
        endpoint_policy[candidate_target] = branch[-1]["policy"]
        feasible = row["duration_s"] <= warning+1e-8 and row["status"] != "curvature_rejected"
        selected = row["status"] == "transition_accepted"
        if selected:
            marker, size, face, edge = "*", 190, POLICY[endpoint_policy[candidate_target]], INK
        elif feasible:
            marker, size, face, edge = "o", 65, POLICY[endpoint_policy[candidate_target]], INK
        else:
            marker, size, face, edge = "o", 54, "none", GREY
        plane_ax.scatter(*candidate_target, marker=marker, s=size, facecolor=face,
                         edgecolor=edge, linewidth=1.1, zorder=5)
    plane_ax.annotate("Stay: F", source, xytext=(8, 9), textcoords="offset points", fontsize=8.6)
    plane_ax.annotate("Selected: C\n43.9 s", target, xytext=(-58, -30),
                      textcoords="offset points", fontsize=8.5)
    plane_ax.annotate("+100 m: F", (100, 300), xytext=(8, -20),
                      textcoords="offset points", fontsize=8.3)
    plane_ax.arrow(source[0], source[1], target[0]-source[0]+8, 0,
                   width=1.3, head_width=10, head_length=8,
                   length_includes_head=True, color=POLICY["C"], zorder=4)
    plane_ax.set(xlabel="Signed lateral offset, d (m)", ylabel="Altitude, h (m)",
                 title=f"(b) Policy outcomes at t+60 s (q ≈ {q_warning/1000:.2f} km)")
    plane_ax.set_xticks(field["offsets_m"]); plane_ax.set_yticks(field["heights_m"])
    plane_ax.grid(color="white", lw=.45, alpha=.55)
    plane_ax.legend(handles=[Line2D([], [], marker="*", color=POLICY["C"], markeredgecolor=INK,
                                    lw=0, markersize=10, label="Selected strict upgrade"),
                             Line2D([], [], marker="o", color=POLICY["F"], markeredgecolor=INK,
                                    lw=0, label="Feasible endpoint policy"),
                             Line2D([], [], marker="o", color="white", markeredgecolor=GREY,
                                    lw=0, label="Cannot complete in 60 s")],
                    loc="upper left", frameon=True, fontsize=8)
    colorbar = fig.colorbar(mesh, ax=plane_ax, pad=.025, fraction=.047)
    colorbar.set_label("Instantaneous SINR at t+60 s (dB)")
    fig.suptitle("Rolling 60+60 s policy decision: corridor context and candidate upgrade",
                 fontsize=17, y=.965)
    save(fig, "01-corridor-and-warning-horizon-candidate-plane",
         "R0016 decision figure using the established corridor-and-candidate-plane template. Panel (a) shows the complete georeferenced reference corridor, all archived base stations, both realized decision locations, and an enlarged view of the first 60 s decision: staying predicts F while the selected -100 m lateral maneuver predicts C. Panel (b) restores the approved offset-altitude candidate plane at the t+60 s warning cross-section; heatmap color is instantaneous SINR, filled markers show feasible endpoint policy, hollow markers cannot complete within 60 s, and the star marks the selected strict upgrade. The modular controller then evaluates actions over the documented additional 60 s interval. The candidate library permits lateral, vertical, and simultaneous lateral-vertical motion; this archived run selected two lateral maneuvers and no altitude change.")

    (output/"captions.md").write_text("# R0016 Toyota briefing captions\n\n" +
        "\n\n".join(f"## {name}\n\n{caption}" for name, caption in captions.items()) + "\n")
    sources = [Path(__file__).resolve(), SCRIPTS/"verify_lateral_study.py",
               ROOT/"src/uam_simulator/spatial_study.py",
               ROOT/"research/dynamic-transitions/dual-horizon-controller-method.md"]
    with zipfile.ZipFile(output/"plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source_path in sources:
            archive.write(source_path, source_path.relative_to(ROOT))
    record = {
        "run": str(run), "run_manifest_sha256": digest(run/"manifest.json"),
        "audit_sha256": digest(audit_path), "detail_decision_time_s": t0,
        "detail_decision_q_m": q0, "selected_target": decision["target"],
        "warning_horizon_s": decision["warning_horizon_s"],
        "evaluation_extension_s": decision["evaluation_extension_s"],
        "figures": list(captions),
        "sources": [{"path": str(p.relative_to(ROOT)), "sha256": digest(p)} for p in sources],
        "outputs": [{"path": p.name, "sha256": digest(p)} for p in sorted(output.iterdir())
                    if p.name != "manifest.json"],
    }
    (output/"manifest.json").write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps({"output": str(output), "figures": list(captions)}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    args = parser.parse_args()
    plot(args.run, args.output, args.audit)
