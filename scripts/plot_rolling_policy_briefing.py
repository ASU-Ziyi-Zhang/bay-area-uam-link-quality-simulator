"""Create the two-figure R0015 rolling-60-s policy briefing set."""
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
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
from verify_lateral_study import independent_reference, read  # noqa: E402

POLICY = {"C": "#198577", "R": "#E7AF3C", "F": "#B64C42"}
INK = "#202020"
BLUE = "#28658B"
GREY = "#737373"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quintic(u):
    u = np.asarray(u, float)
    return 10*u**3 - 15*u**4 + 6*u**5


def station_projection(frame, stations, q_grid):
    centers, _, _, _, _ = frame(q_grid)
    base = centers[0]
    nearest = np.argmin(np.linalg.norm(centers[:, None, :] - stations[None, :, :2], axis=2), axis=0)
    return q_grid[nearest], stations[:, :2] - base, centers - base


def policy_segments(ax, rows, y, height=.54):
    q = np.asarray([r["q_m"] / 1000 for r in rows])
    for i, row in enumerate(rows[:-1]):
        ax.broken_barh([(q[i], q[i+1]-q[i])], (y-height/2, height),
                       facecolors=POLICY[row["policy"]], edgecolors="none", rasterized=True)


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
    spec = read(run/"resolved_config.json"); cfg = spec["parameters"]
    cases = ["fixed_reference", "reactive_joint", "rolling_60s_joint"]
    summaries = {r["case_id"]: r for r in read(run/"summary.json")}
    traces = {c: read(run/c/"trace.json.gz") for c in cases}
    observations = {c: read(run/c/"observations.json.gz") for c in cases}
    events = {c: read(run/c/"events.json") for c in cases}
    accepted = [e for e in events["rolling_60s_joint"] if e["status"] == "transition_accepted"]
    if len(accepted) != 1:
        raise ValueError("briefing expects exactly one rolling decision")
    decision = accepted[0]; t0, q0 = float(decision["t_s"]), float(decision["q_m"])
    t1, q1 = float(decision["forecast_endpoint_s"]), float(decision["forecast_endpoint_q_m"])
    decision_rows = [e for e in events["rolling_60s_joint"] if abs(e.get("t_s", -1)-t0) < 1e-8
                     and e.get("planner") == "rolling_endpoint"]
    stay = next(e for e in decision_rows if e["status"] == "rolling_stay_reference")
    completion = next(e for e in events["rolling_60s_joint"] if e["status"] == "transition_completed")
    with np.load(run/"field.npz") as archive:
        field = {k: archive[k].copy() for k in archive.files}
    with zipfile.ZipFile(run/"source_snapshot.zip") as archive:
        frame = independent_reference(archive, manifest, cfg["center_control_step_m"])

    q_plot = np.linspace(0, float(field["q_m"][-1]), 1600)
    stations = np.asarray([[s["x_m"], s["y_m"], s.get("height_m") or 0]
                           for s in spec["base_stations"]], float)
    station_q, station_xy, route_xy = station_projection(frame, stations, q_plot)
    origin = frame(np.array([0.]))[0][0]
    c0, _, n0, _, _ = frame(np.array([q0])); c0, n0 = c0[0]-origin, n0[0]
    c1, _, n1, _, _ = frame(np.array([q1])); c1, n1 = c1[0]-origin, n1[0]
    stay_xy = c1
    selected_xy = c1 + decision["target"][0]*n1
    field_index = int(np.argmin(abs(field["q_m"]-q1)))

    plt.rcParams.update({
        "font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 11,
        "axes.labelsize": 11.5, "axes.titlesize": 13, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
        "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
        "svg.hashsalt": "uam-rolling-policy-r0015", "figure.facecolor": "white",
        "savefig.facecolor": "white", "legend.fontsize": 8.5,
    })
    output.mkdir(parents=True); captions = {}

    def save(fig, name, caption):
        for ext in ("png", "svg"):
            fig.savefig(output/f"{name}.{ext}", dpi=300, bbox_inches="tight", pad_inches=.12,
                        metadata={"Description": caption})
        captions[name] = caption; plt.close(fig)

    # Figure 1: geographic decision and 60-s endpoint candidate plane.
    fig = plt.figure(figsize=(16, 8.7))
    gs = fig.add_gridspec(1, 2, left=.055, right=.975, top=.88, bottom=.13,
                          wspace=.18, width_ratios=[1.08, 1.55])
    map_ax, plane_ax = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    map_ax.plot(route_xy[:, 0]/1000, route_xy[:, 1]/1000, color="#9AA4AA", lw=1.2,
                label="Reference route")
    mask = (q_plot >= q0) & (q_plot <= q1)
    map_ax.plot(route_xy[mask, 0]/1000, route_xy[mask, 1]/1000, color=POLICY["F"],
                lw=2.5, ls="--", label="60-s stay forecast: F")
    u = np.linspace(0, 1, 100); q_move = q0 + (q1-q0)*u
    cm, _, nm, _, _ = frame(q_move)
    d_move = decision["target"][0]*quintic(np.clip(60*u/decision["duration_s"], 0, 1))
    xy_move = cm-origin + d_move[:, None]*nm
    map_ax.plot(xy_move[:, 0]/1000, xy_move[:, 1]/1000, color=POLICY["C"], lw=2.8,
                label="Selected forecast: C")
    map_ax.scatter(station_xy[:, 0]/1000, station_xy[:, 1]/1000, marker="^", s=34,
                   facecolor="white", edgecolor="#4A4A4A", linewidth=.7, zorder=4)
    for i, xy in enumerate(station_xy):
        map_ax.text(xy[0]/1000 + (.025 if i%2==0 else -.025), xy[1]/1000 + (.022 if i%3 else -.03),
                    spec["base_stations"][i]["site_id"], fontsize=6.5, color="#4A4A4A",
                    ha="left" if i%2==0 else "right")
    map_ax.scatter(*(c0/1000), s=105, color="#C72B2B", edgecolor="white", linewidth=1.1,
                   zorder=7, label="Current state")
    map_ax.scatter(*(stay_xy/1000), marker="x", s=80, color=POLICY["F"], linewidth=2, zorder=7)
    map_ax.scatter(*(selected_xy/1000), marker="o", s=72, color=POLICY["C"], edgecolor="white",
                   linewidth=1, zorder=7)
    map_ax.annotate("q0", c0/1000, xytext=(7, 7), textcoords="offset points", fontsize=8.5)
    map_ax.set_aspect("equal", adjustable="datalim")
    map_ax.set(xlabel="Easting relative to route origin (km)",
               ylabel="Northing relative to route origin (km)",
               title="(a) Current state and 60-s route forecast")
    map_ax.grid(color="#E5E5E5", lw=.5); map_ax.legend(loc="lower left", frameon=True, fontsize=8)
    # The 100 m maneuver is necessarily tiny on the 50 km corridor map.  A
    # local inset preserves the geographic context while making the two 60-s
    # alternatives legible at their actual scale.
    inset = map_ax.inset_axes([.50, .64, .47, .31])
    local = (q_plot >= q0-350) & (q_plot <= q1+350)
    inset.plot(route_xy[local, 0]/1000, route_xy[local, 1]/1000, color="#9AA4AA", lw=1)
    inset.plot(route_xy[mask, 0]/1000, route_xy[mask, 1]/1000, color=POLICY["F"], lw=2.2, ls="--")
    inset.plot(xy_move[:, 0]/1000, xy_move[:, 1]/1000, color=POLICY["C"], lw=2.6)
    inset.scatter(*(c0/1000), s=60, color="#C72B2B", edgecolor="white", linewidth=.8, zorder=5)
    inset.scatter(*(stay_xy/1000), marker="x", s=48, color=POLICY["F"], linewidth=1.7, zorder=5)
    inset.scatter(*(selected_xy/1000), s=45, color=POLICY["C"], edgecolor="white", linewidth=.7, zorder=5)
    inset.annotate("Current C", c0/1000, xytext=(5, 7), textcoords="offset points", fontsize=7.4)
    inset.annotate("Stay: F", stay_xy/1000, xytext=(5, -13), textcoords="offset points",
                   fontsize=7.2, color=POLICY["F"])
    inset.annotate("−100 m: C", selected_xy/1000, xytext=(5, 5), textcoords="offset points",
                   fontsize=7.2, color=POLICY["C"])
    inset.set_xticks([]); inset.set_yticks([]); inset.set_title("60-s decision segment", fontsize=8.2)
    map_ax.indicate_inset_zoom(inset, edgecolor="#6F7880", alpha=.8)

    values = field["sinr_db"][field_index].T
    norm = TwoSlopeNorm(vmin=min(float(values.min()), cfg["threshold_db"]-.05),
                        vcenter=cfg["threshold_db"], vmax=max(float(values.max()), cfg["threshold_db"]+.05))
    mesh = plane_ax.pcolormesh(field["offsets_m"], field["heights_m"], values,
                               shading="nearest", cmap="RdBu", norm=norm, rasterized=True)
    dm, hm = np.meshgrid(field["offsets_m"], field["heights_m"])
    plane_ax.scatter(dm.ravel(), hm.ravel(), s=9, color="white", edgecolor="#6B6B6B",
                     linewidth=.25, alpha=.7)
    for row in decision_rows:
        d, h = row["target"]
        feasible = row["status"] not in {"maneuver_exceeds_rollout", "curvature_rejected"}
        selected = row["status"] == "transition_accepted"
        if selected:
            marker, size, color, edge = "*", 180, POLICY[row["endpoint_policy"]], INK
        elif feasible:
            marker, size, color, edge = "o", 62, POLICY[row["endpoint_policy"]], INK
        else:
            marker, size, color, edge = "o", 52, "none", GREY
        plane_ax.scatter(d, h, marker=marker, s=size, facecolor=color, edgecolor=edge,
                         linewidth=1.1, zorder=5)
    plane_ax.annotate("Stay: F", (0, 300), xytext=(8, 9), textcoords="offset points", fontsize=8.5)
    plane_ax.annotate("Selected: C\n43.9 s", (-100, 300), xytext=(-58, -29),
                      textcoords="offset points", fontsize=8.5)
    plane_ax.annotate("+100 m: F", (100, 300), xytext=(8, -20), textcoords="offset points", fontsize=8.3)
    plane_ax.arrow(0, 300, -92, 0, width=1.3, head_width=10, head_length=8,
                   length_includes_head=True, color=POLICY["C"], zorder=4)
    plane_ax.set(xlabel="Signed lateral offset, d (m)", ylabel="Altitude, h (m)",
                 title=f"(b) Policy outcomes at t+60 s (q ≈ {q1/1000:.2f} km)")
    plane_ax.set_xticks(field["offsets_m"]); plane_ax.set_yticks(field["heights_m"])
    plane_ax.grid(color="white", lw=.4, alpha=.55)
    plane_ax.legend(handles=[Line2D([], [], marker="*", color=POLICY["C"], markeredgecolor=INK,
                                    lw=0, markersize=10, label="Selected strict upgrade"),
                             Line2D([], [], marker="o", color=POLICY["F"], markeredgecolor=INK,
                                    lw=0, label="Feasible endpoint policy"),
                             Line2D([], [], marker="o", color="white", markeredgecolor=GREY,
                                    lw=0, label="Cannot complete in 60 s")],
                    loc="upper left", frameon=True, fontsize=8)
    cb = fig.colorbar(mesh, ax=plane_ax, pad=.025, fraction=.046); cb.set_label("Instantaneous SINR at t+60 s (dB)")
    fig.suptitle("Figure 1. Rolling 60-s policy decision: forecast degradation and candidate upgrade",
                 fontsize=17, y=.955)
    fig.text(.5, .052,
             "At t = 265 s the current policy is C. Staying predicts F at t = 325 s; the nearest feasible strict upgrade is d = −100 m, which predicts C and completes within the 60-s horizon.",
             ha="center", fontsize=10.1, color="#4A5961")
    save(fig, "01-rolling-decision-and-candidate-plane",
         "R0015 rolling-decision figure. Panel (a) marks the current C state, the 60-s no-action forecast ending in F, the selected lateral forecast ending in C, and all archived base stations. Panel (b) shows the offset-height candidate plane at the common 60-s endpoint. Heatmap colour is instantaneous SINR; marker labels are exposure-derived endpoint policy. The selected -100 m move is the nearest feasible strict policy upgrade.")

    # Figure 2: global trajectory and global policy comparison only.  The local
    # decision mechanism is already fully explained by Figure 1.
    fig = plt.figure(figsize=(16, 8.4))
    gs = fig.add_gridspec(1, 2, left=.06, right=.975, top=.87, bottom=.14,
                          wspace=.20, width_ratios=[1.18, 1])
    global3d = fig.add_subplot(gs[0, 0], projection="3d")
    policy_ax = fig.add_subplot(gs[0, 1])

    fixed, rolling = traces["fixed_reference"], traces["rolling_60s_joint"]
    qf = np.asarray([r["q_m"]/1000 for r in fixed])
    global3d.plot(qf, np.zeros_like(qf), np.full_like(qf, 300), color=GREY,
                  lw=1.4, ls="--", label="No-move reference")
    xyz = np.asarray([[r["q_m"]/1000, r["offset_m"], r["altitude_m"]] for r in rolling])
    if len(xyz) > 1:
        seg = np.stack([xyz[:-1], xyz[1:]], axis=1)
        global3d.add_collection3d(Line3DCollection(seg,
            colors=[POLICY[r["policy"]] for r in rolling[:-1]], linewidths=2.8))
    global3d.scatter(q0/1000, 0, 300, color="#C72B2B", s=62,
                     edgecolor="white", linewidth=.9, zorder=6)
    global3d.scatter(completion["q_m"]/1000, -100, 300, color=POLICY["C"], s=54,
                     edgecolor=INK, linewidth=.7, zorder=6)
    global3d.set(xlabel="Reference-route progress q (km)", ylabel="Lateral offset d (m)",
                 zlabel="Altitude h (m)", title="(a) Realized full-corridor trajectory")
    global3d.set_xlim(0, max(qf)); global3d.set_ylim(-115, 18); global3d.set_zlim(285, 315)
    global3d.set_box_aspect([7.4, 2.5, 1.5]); global3d.view_init(elev=23, azim=-61)
    global3d.set_proj_type("ortho")
    global3d.legend(handles=[Line2D([], [], color=GREY, ls="--", label="No-move reference"),
                             Line2D([], [], color=POLICY["C"], lw=2.8, label="Rolling trajectory: C"),
                             Line2D([], [], color=POLICY["F"], lw=2.8, label="Rolling trajectory: F"),
                             Line2D([], [], marker="o", color="none", mfc="#C72B2B", mec="white",
                                    label="Decision: q=13.26 km"),
                             Line2D([], [], marker="o", color="none", mfc=POLICY["C"], mec=INK,
                                    label="−100 m complete: q=15.47 km")],
                    loc="upper left", frameon=False, ncol=1)

    policy_segments(policy_ax, fixed, 0); policy_segments(policy_ax, rolling, 1)
    policy_ax.axvline(q0/1000, color="#C72B2B", lw=.9, ls=":")
    policy_ax.axvspan(16.278, 18.029, color=POLICY["C"], alpha=.10, lw=0)
    policy_ax.annotate("predictive decision", (q0/1000, 1), xytext=(5, 24),
                       textcoords="offset points", fontsize=8.4, color="#C72B2B")
    policy_ax.annotate("baseline F interval removed", ((16.278+18.029)/2, .5),
                       xytext=(0, 38), textcoords="offset points", ha="center",
                       arrowprops=dict(arrowstyle="-|>", color=POLICY["C"], lw=.8),
                       fontsize=8.7, color=POLICY["C"])
    policy_ax.set(yticks=[0, 1], yticklabels=["No move", "Rolling 60 s"],
                  ylim=(1.62, -.62), xlim=(0, max(qf)),
                  xlabel="Reference-route progress q (km)",
                  title="(b) Policy before and after rolling control")
    policy_ax.grid(axis="x", color="#E5E5E5", lw=.5)
    f0 = summaries["fixed_reference"]["policy_time_s"]["F"]
    f1 = summaries["rolling_60s_joint"]["policy_time_s"]["F"]
    policy_ax.text(.02, -.20,
                   f"One predictive move  |  F time: {f0:.0f} → {f1:.0f} s  |  reduction: {f0-f1:.0f} s ({(f0-f1)/f0:.1%})",
                   transform=policy_ax.transAxes, fontsize=9.2, color="#4A5961", va="top")
    policy_ax.legend(handles=[Patch(color=POLICY[p], label=p) for p in "CRF"],
                     loc="upper right", frameon=False, ncol=3)
    fig.suptitle("Figure 2. Global rolling trajectory and full-corridor policy effect",
                 fontsize=17, y=.95)
    fig.text(.5, .055,
             "The rolling controller makes one bounded 0→−100 m transition. The global policy comparison shows the eliminated fallback interval and the remaining unchanged weak-link regions.",
             ha="center", fontsize=10.1, color="#4A5961")
    save(fig, "02-global-trajectory-and-policy-outcome",
         "R0015 global outcome figure. Panel (a) shows the complete q-offset-height trajectory, the one predictive -100 m transition and the no-move reference. Panel (b) compares exposure-derived C/R/F policy over the entire corridor. The transition removes the baseline F interval from q=16.278 to 18.029 km and reduces total F time from 280 to 245 s; other fallback intervals remain.")

    (output/"captions.md").write_text("# R0015 Toyota briefing captions\n\n" +
        "\n\n".join(f"## {k}\n\n{v}" for k, v in captions.items()) + "\n")
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), SCRIPTS/"verify_lateral_study.py"]
    with zipfile.ZipFile(output/"plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(root))
    record = {"run": str(run), "run_manifest_sha256": digest(run/"manifest.json"),
              "audit_sha256": digest(audit_path), "decision_time_s": t0,
              "decision_q_m": q0, "forecast_endpoint_s": t1,
              "current_policy": "C", "stay_endpoint_policy": stay["endpoint_policy"],
              "selected_target": decision["target"], "selected_endpoint_policy": decision["endpoint_policy"],
              "figures": list(captions),
              "sources": [{"path": str(p.relative_to(root)), "sha256": digest(p)} for p in sources],
              "outputs": [{"path": p.name, "sha256": digest(p)} for p in sorted(output.iterdir())
                          if p.name != "manifest.json"]}
    (output/"manifest.json").write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps({"output": str(output), "figures": list(captions)}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path); parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    args = parser.parse_args(); plot(args.run, args.output, args.audit)
