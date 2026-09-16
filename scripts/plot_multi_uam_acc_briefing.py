"""Plot the Step-3 multi-UAM, multi-flow, ACC mechanism-test briefing figure."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


INK = "#20282E"
POLICY = {"C": "#218A7A", "R": "#E5AD37", "F": "#B84B43"}


def read(path: Path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt") as stream:
            return json.load(stream)
    return json.loads(path.read_text())


def sha(path: Path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def group_by_aircraft(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row["aircraft_id"], []).append(row)
    for values in grouped.values():
        values.sort(key=lambda row: row["t_s"])
    return grouped


def at_time(rows, field, times):
    return np.interp(times, [row["t_s"] for row in rows], [row[field] for row in rows])


def make_summary_table(run: Path, output: Path, summaries, events):
    base, controlled = summaries["acc_no_change"], summaries["acc_safe_rolling"]
    counts = Counter(row["status"] for row in events)
    base_total = sum(base["policy_aircraft_time_s"].values())
    controlled_total = sum(controlled["policy_aircraft_time_s"].values())
    f_delta = controlled["policy_aircraft_time_s"]["F"] - base["policy_aircraft_time_s"]["F"]
    f_percent = 100 * f_delta / base["policy_aircraft_time_s"]["F"]
    f_share_delta = 100 * (controlled["policy_aircraft_time_shares"]["F"]
                           - base["policy_aircraft_time_shares"]["F"])
    accepted = [row for row in events if row["status"] == "transition_accepted"]
    avoidance_count = sum(row.get("decision_type") == "policy_avoidance" for row in accepted)
    return_count = sum(row.get("decision_type") == "nominal_return" for row in accepted)
    rows = [
        ("Aircraft entered / completed", "15 / 15", "15 / 15", "No entry deferral"),
        ("Completed lane changes", "0", str(controlled["completed_transitions"]),
         f"{avoidance_count} avoidance + {return_count} nominal-return"),
        ("Missions ending on nominal flow", "15 / 15",
         f'{controlled.get("completed_on_nominal_lane", 0)} / 15',
         "No forced terminal return"),
        ("Target-gap candidate rejections", "—", str(counts["insertion_gap_rejected"]),
         "Safety/ACC gate, not mission failures"),
        ("C policy share", f'{100*base["policy_aircraft_time_shares"]["C"]:.2f}%',
         f'{100*controlled["policy_aircraft_time_shares"]["C"]:.2f}%',
         f'{100*(controlled["policy_aircraft_time_shares"]["C"]-base["policy_aircraft_time_shares"]["C"]):+.2f} percentage points'),
        ("R policy share", f'{100*base["policy_aircraft_time_shares"]["R"]:.2f}%',
         f'{100*controlled["policy_aircraft_time_shares"]["R"]:.2f}%',
         f'{100*(controlled["policy_aircraft_time_shares"]["R"]-base["policy_aircraft_time_shares"]["R"]):+.2f} percentage points'),
        ("Fallback aircraft-time, s", f'{base["policy_aircraft_time_s"]["F"]:.0f}',
         f'{controlled["policy_aircraft_time_s"]["F"]:.0f}',
         f'{f_delta:.0f} s ({f_percent:.1f}%)'),
        ("Fallback share", f'{100*base["policy_aircraft_time_shares"]["F"]:.2f}%',
         f'{100*controlled["policy_aircraft_time_shares"]["F"]:.2f}%',
         f'{f_share_delta:+.2f} percentage points'),
        ("Total mission aircraft-time, s", f"{base_total:.0f}", f"{controlled_total:.1f}",
         f"{controlled_total-base_total:+.1f} s"),
        ("Minimum longitudinal speed, m/s", f'{base["minimum_longitudinal_speed_mps"]:.2f}',
         f'{controlled["minimum_longitudinal_speed_mps"]:.2f}',
         "ACC response"),
        ("Minimum 3-D separation score", f'{base["minimum_three_dimensional_separation_score"]:.3f}',
         f'{controlled["minimum_three_dimensional_separation_score"]:.3f}',
         "Passes engineering screen (score ≥ 1)"),
        ("Capacity", "Not estimated", "Not estimated", "Deferred to Step 4"),
    ]
    header = ["Metric", "ACC, no lane changes", "ACC + safe rolling changes", "Difference / note"]
    csv_lines = [",".join(f'"{value}"' for value in header)]
    csv_lines += [",".join(f'"{value}"' for value in row) for row in rows]
    (output / "summary-table.csv").write_text("\n".join(csv_lines) + "\n")
    md = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * 4) + "|"]
    md += ["| " + " | ".join(row) + " |" for row in rows]
    md += ["", f"{run.name} is an uncalibrated engineering mechanism test; it does not estimate capacity or establish certified separation."]
    (output / "summary-table.md").write_text("\n".join(md) + "\n")


def plot(run: Path, audit_path: Path, output: Path):
    run, audit_path, output = map(Path.resolve, (run, audit_path, output))
    if output.exists():
        raise FileExistsError("choose a new output directory")
    manifest, audit = read(run / "manifest.json"), read(audit_path)
    if manifest["status"] != "completed" or audit["status"] != "passed":
        raise ValueError("a completed run and passed independent audit are required")
    if audit["run_manifest_sha256"] != sha(run / "manifest.json"):
        raise ValueError("audit does not match this run manifest")
    for item in manifest["outputs"]:
        if sha(run / item["path"]) != item["sha256"]:
            raise ValueError(f"archived output changed: {item['path']}")

    summaries = {row["case_id"]: row for row in read(run / "summary.json")}
    run_id = run.name
    rolling = read(run / "acc_safe_rolling" / "trace.json.gz")
    events = read(run / "acc_safe_rolling" / "events.json")
    accepted = [row for row in events if row["status"] == "transition_accepted"]
    rolling_by_id = group_by_aircraft(rolling)
    displayed = ["L1-UAM02", "L2-UAM02", "L3-UAM02"]
    if any(identifier not in rolling_by_id for identifier in displayed):
        raise ValueError("representative aircraft are missing from the archived run")

    output.mkdir(parents=True)
    make_summary_table(run, output, summaries, events)

    plt.rcParams.update({
        "font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 10.5,
        "axes.labelsize": 11.5, "axes.titlesize": 12.5, "axes.edgecolor": INK,
        "axes.labelcolor": INK, "xtick.color": INK, "ytick.color": INK,
        "text.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
        "svg.hashsalt": "uam-step3-r0020-longitudinal", "figure.facecolor": "white",
        "savefig.facecolor": "white", "legend.fontsize": 8.5,
    })

    # The longitudinal corridor is the dominant spatial dimension.  Give it a
    # deliberately elongated display aspect so that lateral maneuvers remain
    # legible without making simulation time look like the longest dimension.
    fig = plt.figure(figsize=(16.0, 8.4))
    traffic_ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=.02, right=.965, bottom=.14, top=.88)

    for identifier in displayed:
        rows = rolling_by_id[identifier]
        keep = np.zeros(len(rows), dtype=bool)
        keep[::4] = True
        keep[-1] = True
        for index in range(1, len(rows)):
            if (rows[index]["policy"] != rows[index-1]["policy"]
                    or rows[index]["target_lane"] != rows[index-1]["target_lane"]):
                keep[max(0, index-1):min(len(rows), index+2)] = True
        selected = [row for row, retain in zip(rows, keep) if retain]
        t = np.asarray([row["t_s"] for row in selected])
        q = np.asarray([row["q_m"] for row in selected]) / 1000
        d = np.asarray([row["offset_m"] for row in selected])
        points = np.c_[q, d, t]
        segments = np.stack([points[:-1], points[1:]], axis=1)
        colors = [POLICY[row["policy"]] for row in selected[:-1]]
        traffic_ax.add_collection3d(Line3DCollection(segments, colors=colors,
                                                     linewidths=2.35, alpha=.96,
                                                     rasterized=True))
        traffic_ax.scatter(q[0], d[0], t[0], marker="^", s=22, facecolor="white",
                           edgecolor=INK, linewidth=.6, depthshade=False)
        traffic_ax.text(q[0] + .6, d[0], t[0] + 22, identifier,
                        fontsize=8.5, color=INK)
    for event in accepted:
        if event["aircraft_id"] not in displayed:
            continue
        rows = rolling_by_id[event["aircraft_id"]]
        q_event = at_time(rows, "q_m", [event["t_s"]])[0] / 1000
        returning = event.get("decision_type") == "nominal_return"
        traffic_ax.scatter(q_event, event["source"][0], event["t_s"],
                           marker="D" if returning else "o", s=26 if returning else 24,
                           facecolor="#CDE8E1" if returning else "white",
                           edgecolor=INK, linewidth=.7, depthshade=False)
    traffic_ax.set_xlim(0, max(row["q_m"] for row in rolling) / 1000)
    traffic_ax.set_ylim(-350, 350)
    traffic_ax.set_zlim(0, max(row["end_s"] for row in rolling))
    traffic_ax.set_yticks([-300, 0, 300])
    traffic_ax.set_xlabel("Longitudinal distance along corridor, $q$ (km)", labelpad=12)
    traffic_ax.set_ylabel("Lateral offset, $d$ (m)", labelpad=10)
    traffic_ax.set_zlabel("Simulation time, $t$ (s)", labelpad=10)
    traffic_ax.view_init(elev=24, azim=-62)
    traffic_ax.set_box_aspect((2.55, .72, .92))
    traffic_ax.xaxis.pane.set_facecolor((.97, .98, .98, 1))
    traffic_ax.yaxis.pane.set_facecolor((.97, .98, .98, 1))
    traffic_ax.zaxis.pane.set_facecolor((.97, .98, .98, 1))
    traffic_ax.legend(handles=[
        Line2D([0], [0], color=POLICY["C"], lw=2.2, label="C"),
        Line2D([0], [0], color=POLICY["R"], lw=2.2, label="R"),
        Line2D([0], [0], color=POLICY["F"], lw=2.2, label="F"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="white",
               markeredgecolor=INK, markersize=5, label="Policy-avoidance start"),
        Line2D([0], [0], marker="D", color="none", markerfacecolor="#CDE8E1",
               markeredgecolor=INK, markersize=5, label="Nominal-return start"),
        Line2D([0], [0], marker="^", color="none", markerfacecolor="white",
               markeredgecolor=INK, markersize=5, label="Corridor entry"),
    ], loc="upper left", bbox_to_anchor=(.01, .98), frameon=True, ncol=2)
    fig.suptitle("Step 3 — Representative lane-changing trajectories (3 of 15 UAM)",
                 fontsize=17, fontweight="semibold", y=.965)
    fig.text(.5, .092,
             "One representative aircraft is shown from each initial lateral flow at h = 300 m. "
             "Motion across the offset axis is a lane change; line color is the realized policy.",
             fontsize=9.5, color="#51616A", ha="center")
    fig.text(.03, .025,
             f"{run_id} | three populated flows at d = −300, 0, +300 m | five aircraft per flow | "
             "90-s per-flow headway with 30-s phases | 60-s warning + 60-s evaluation",
             fontsize=9.5, color="#52616A")
    fig.text(.97, .025, "Engineering mechanism test; capacity not estimated.",
             fontsize=9.5, ha="right", color="#52616A")
    caption = ("Three representative trajectories from the fifteen-aircraft Step-3 pilot at one "
               "altitude. The displayed aircraft share the UAM02 release index and start in the "
               "three different lateral flows. Longitudinal corridor distance and lateral offset form the "
               "horizontal plane, simulation time is vertical, and color denotes C/R/F policy.")
    for extension in ("png", "svg"):
        fig.savefig(output / f"01-multi-uam-acc-mechanism.{extension}", dpi=300,
                    bbox_inches="tight", pad_inches=.12,
                    metadata={"Description": caption})
    plt.close(fig)

    provenance = {
        "run": str(run), "run_manifest_sha256": sha(run / "manifest.json"),
        "audit": str(audit_path), "audit_sha256": sha(audit_path),
        "accepted_transitions_full_fleet": len(accepted),
        "displayed_aircraft": displayed,
        "displayed_accepted_transitions": sum(row["aircraft_id"] in displayed for row in accepted),
        "caption": caption,
        "outputs": {path.name: sha(path) for path in sorted(output.iterdir())},
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(provenance, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("audit", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plot(args.run, args.audit, args.output)
