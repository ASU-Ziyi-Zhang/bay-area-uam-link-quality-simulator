"""Academic revision of the R0008 SINR figure; no simulation is rerun.

The geographic panel retains all BS ground positions. Only interior normal
projections inside the modeled offset band are overlaid on the SINR panel.
All scientific inputs come from the immutable run and its source archive.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
import matplotlib.patheffects as pe
import numpy as np

from verify_lateral_study import independent_reference


INK = "#202020"
GOLD = "#F2BD3C"


def read(path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt") as stream:
            return json.load(stream)
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def closest_reference_point(frame, point, length, sample_m=10.):
    """Refine every sampled local distance minimum, including both endpoints.

    R0008's minimum centerline radius is >976 m; a <=10 m search grid brackets
    its resolved minima. Refinement is independent of the heatmap's 50 m grid.
    Endpoint projections retain their longitudinal residual, rather than
    pretending that a beyond-end site lies in the normal-coordinate band.
    """
    point = np.asarray(point, float)
    q = np.linspace(0., length, int(np.ceil(length / sample_m)) + 1)
    centers = frame(q)[0]
    distances = np.sum((centers - point) ** 2, axis=1)
    local = np.flatnonzero((distances[1:-1] <= distances[:-2]) &
                          (distances[1:-1] <= distances[2:])) + 1
    candidates = [0., length]
    ratio = (np.sqrt(5.) - 1.) / 2.

    def objective(value):
        return float(np.sum((frame(value)[0] - point) ** 2))

    for index in local:
        lo, hi = q[index - 1], q[index + 1]
        a, b = hi - ratio * (hi - lo), lo + ratio * (hi - lo)
        fa, fb = objective(a), objective(b)
        for _ in range(60):
            if fa < fb:
                hi, b, fb = b, a, fa
                a = hi - ratio * (hi - lo)
                fa = objective(a)
            else:
                lo, a, fa = a, b, fb
                b = lo + ratio * (hi - lo)
                fb = objective(b)
        candidates.append((lo + hi) / 2.)
    nearest = min(candidates, key=objective)
    center, tangent, normal, _, _ = frame(nearest)
    delta = point - center
    return {"q_m": float(nearest), "offset_m": float(delta @ normal),
            "longitudinal_residual_m": float(delta @ tangent),
            "horizontal_distance_m": float(np.linalg.norm(delta)),
            "interior": bool(0. < nearest < length)}


def load_inputs(run):
    manifest = read(run / "manifest.json")
    if manifest["status"] != "completed":
        raise ValueError("only a completed numerical run can be illustrated")
    for entry in manifest["outputs"]:
        if sha(run / entry["path"]) != entry["sha256"]:
            raise ValueError(f"run hash mismatch: {entry['path']}")
    spec = read(run / "resolved_config.json")
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        for entry in manifest["code"] + manifest["inputs"]:
            if hashlib.sha256(archive.read(entry["path"])).hexdigest() != entry["sha256"]:
                raise ValueError(f"archive hash mismatch: {entry['path']}")
        scenario_path = next(e["path"] for e in manifest["inputs"]
                             if e["path"].endswith("/scenario.json"))
        scenario = json.loads(archive.read(scenario_path))
        frame = independent_reference(archive, manifest,
                                      spec["parameters"]["center_control_step_m"])
    # The axis labels below explicitly identify this metric geographic CRS.
    if scenario["corridor"].get("target_crs", "EPSG:26910") != "EPSG:26910":
        raise ValueError("this geographic panel requires NAD83 / UTM zone 10N")
    width = max(spec["envelopes_m"])
    case = run / f"adaptive_{width:g}"
    trace, terminal = read(case / "trace.json.gz"), read(case / "terminal.json")
    events = [e for e in read(case / "events.json") if e["status"] == "transition_accepted"]
    with np.load(run / "field.npz") as data:
        field = {key: data[key].copy() for key in data.files}
    length = float(field["q_m"][-1])
    projections = []
    for station in spec["base_stations"]:
        projected = closest_reference_point(frame, [station["x_m"], station["y_m"]], length)
        projected.update(site_id=station["site_id"], x_m=station["x_m"], y_m=station["y_m"],
                         bs_height_m=station["height_m"])
        projected["shown_in_sinr_panel"] = bool(
            projected["interior"] and abs(projected["offset_m"]) <= width
            and abs(projected["longitudinal_residual_m"]) < .001)
        projections.append(projected)
    q = np.r_[[row["q_m"] for row in trace], terminal["q_m"]]
    d = np.r_[[row["offset_m"] for row in trace], terminal["offset_m"]]
    xyz = np.vstack([[row["position_m"] for row in trace], terminal["position_m"]])
    center, _, normal, _, _ = frame(q)
    residual = float(np.max(np.linalg.norm(center + d[:, None] * normal - xyz[:, :2], axis=1)))
    if residual > 1e-6:
        raise ValueError("archived trajectory disagrees with reconstructed reference")
    return spec, frame, field, events, q, d, projections, residual


def plot(run, output):
    run, output = Path(run).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError("choose a new figure revision directory; archived figures are immutable")
    spec, frame, field, events, q, d, stations, residual = load_inputs(run)
    cfg = spec["parameters"]
    width = max(spec["envelopes_m"])
    initial = spec["initial_offset_m"]
    z = field["sinr_db"]
    plt.rcParams.update({"font.family": "STIXGeneral", "mathtext.fontset": "stix",
        "font.size": 12, "axes.labelsize": 13, "axes.titlesize": 14,
        "text.color": INK, "axes.labelcolor": INK, "axes.edgecolor": INK,
        "xtick.color": INK, "ytick.color": INK, "axes.linewidth": .7,
        "axes.titleweight": "normal", "xtick.direction": "out", "ytick.direction": "out",
        "svg.fonttype": "none", "svg.hashsalt": "uam-r0008-radio-academic-v2",
        "figure.facecolor": "white", "savefig.facecolor": "white"})

    fig = plt.figure(figsize=(16.8, 5.5))
    geo = fig.add_axes([.057, .18, .251, .65])
    ax = fig.add_axes([.388, .18, .548, .65])
    cax = fig.add_axes([.950, .18, .012, .65])
    annotations = []
    ground = np.array([[s["x_m"], s["y_m"]] for s in stations]) / 1000
    reference = frame(field["q_m"])[0] / 1000
    geo.plot(reference[:, 0], reference[:, 1], color=INK, lw=1.2, label="Reference route")
    geo.scatter(ground[:, 0], ground[:, 1], marker="^", s=40, facecolor="white",
                edgecolor=INK, linewidth=.9, zorder=4, label="Base station")
    geo.scatter(reference[[0, -1], 0], reference[[0, -1], 1], marker="s", s=18,
                color=INK, zorder=5)
    # Label offsets affect typography only; all leader endpoints are actual BS coordinates.
    label_offsets = {"BS05": (-26, -2), "BS06": (-24, -8), "BS07": (7, 10),
        "BS08": (9, -8), "BS09": (-30, -9), "BS10": (-13, 14), "BS11": (7, 8),
        "BS12": (9, -12), "BS13": (-25, -17), "BS14": (-12, 15),
        "BS15": (10, 4), "BS16": (-35, -12)}
    for station, xy in zip(stations, ground):
        annotations.append(geo.annotate(station["site_id"], xy,
            xytext=label_offsets.get(station["site_id"], (7, 7)), textcoords="offset points",
            fontsize=10.5, va="center", arrowprops={"arrowstyle": "-", "lw": .5, "color": "#606060"},
            bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .3}))
    annotations.append(geo.annotate("Start", reference[0], xytext=(4, 7),
                                    textcoords="offset points", fontsize=10.5))
    annotations.append(geo.annotate("End", reference[-1], xytext=(-22, 7),
                                    textcoords="offset points", fontsize=10.5))
    extent = np.vstack([ground, reference])
    xlo, xhi = extent[:, 0].min() - 4.5, extent[:, 0].max() + 4.5
    ycenter = (extent[:, 1].min() + extent[:, 1].max()) / 2
    # Match both panel frames without distorting the geographic aspect ratio.
    yhalf = (xhi - xlo) * (.65 * 5.5) / (.251 * 16.8) / 2
    geo.set(xlim=(xlo, xhi), ylim=(ycenter - yhalf, ycenter + yhalf),
            xlabel="Easting (km)", ylabel="Northing (km)")
    geo.set_aspect("equal", adjustable="box")
    geo.ticklabel_format(axis="both", useOffset=False, style="plain")
    geo.grid(color="#DDDDDD", lw=.45)
    geo.set_title("(a) Base-station layout", loc="left", pad=12)

    norm = TwoSlopeNorm(vmin=float(z.min()), vcenter=cfg["threshold_db"], vmax=float(z.max()))
    mesh = ax.pcolormesh(field["q_m"] / 1000, field["offsets_m"], z,
                        shading="nearest", cmap="RdBu", norm=norm, rasterized=True)
    ax.contour(field["q_m"] / 1000, field["offsets_m"], z, levels=[cfg["threshold_db"]],
               colors="#59504C", linewidths=.65, linestyles=":", alpha=.8)
    ax.axhline(initial, color=INK, ls="--", lw=1.3)
    ax.plot(q / 1000, d, color=GOLD, lw=2.5,
            path_effects=[pe.Stroke(linewidth=3.9, foreground=INK), pe.Normal()])
    for index, event in enumerate(events, 1):
        location = (event["q_m"] / 1000, event["source_m"])
        ax.plot(*location, "o", ms=6, mfc=GOLD, mec=INK, mew=.9)
        ax.annotate(str(index), location, xytext=(0, -20 if event["source_m"] >= .8 * width else 10),
                    textcoords="offset points", ha="center", fontsize=11,
                    bbox={"facecolor": "white", "alpha": .85, "edgecolor": "none", "pad": .5})
    for station in stations:
        if station["shown_in_sinr_panel"]:
            location = (station["q_m"] / 1000, station["offset_m"])
            ax.plot(*location, "^", ms=8, mfc="white", mec=INK, mew=1., zorder=8)
            ax.annotate(station["site_id"], location, xytext=(8, 4), textcoords="offset points",
                        fontsize=11, bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .4})
    ax.set(xlabel=r"Reference-route distance, $q$ (km)", ylabel=r"Lateral offset, $d$ (m)",
           xlim=(0, field["q_m"][-1] / 1000), ylim=(-width, width),
           xticks=np.arange(0, field["q_m"][-1] / 1000 + .01, 10),
           yticks=np.arange(-width, width + .01, 100))
    ax.set_title(f"(b) SINR field and lateral trajectory ($h = {cfg['altitude_m']:g}$ m)", loc="left", pad=12)
    cb = fig.colorbar(mesh, cax=cax)
    cb.set_label("SINR (dB)", labelpad=9)
    ticks = [float(z.min()), cfg["threshold_db"], 5., 15., 25.]
    cb.set_ticks(ticks)
    cb.set_ticklabels([f"{ticks[0]:.2f}", f"{ticks[1]:g}", "5", "15", "25"])
    cb.ax.tick_params(labelsize=11)
    legend = [Line2D([], [], color=INK, lw=1.2, label="Reference route"),
        Line2D([], [], color=INK, ls="--", lw=1.3, label="Fixed-offset trajectory"),
        Line2D([], [], color=GOLD, lw=2.5, label="Adaptive trajectory",
                   path_effects=[pe.Stroke(linewidth=3.9, foreground=INK), pe.Normal()]),
        Line2D([], [], marker="^", mfc="white", mec=INK, color="none", ms=7, label="Base station"),
        Line2D([], [], color="#59504C", ls=":", lw=.8,
                   label=rf"$\mathrm{{SINR}} = {cfg['threshold_db']:g}$ dB")]
    fig.legend(handles=legend, loc="lower center", bbox_to_anchor=(.515, .018), ncol=5,
               frameon=False, fontsize=11.5, handlelength=2.5, columnspacing=2.)
    # No narrative footer; scientific settings and interpretation belong in the caption.
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    text_boxes = [a.get_window_extent(renderer) for a in annotations]
    overlaps = [(i, j) for i, box in enumerate(text_boxes) for j, other in enumerate(text_boxes)
                if i < j and box.overlaps(other)]
    # Annotation extents include leader lines; retain this conservative diagnostic for QA.
    output.mkdir(parents=True)
    caption = (
        "Figure 1. Base-station distribution and communication-driven lateral trajectory at h = 300 m. "
        "(a) Ground locations of the 12 base stations and the reference route in NAD83 / UTM zone 10N. "
        "(b) Modeled SINR in route-relative coordinates: q denotes distance along the original reference "
        "polyline, and positive d denotes leftward offset relative to travel direction. The dashed line "
        "is the d = 0 baseline; the yellow curve is the adaptive trajectory initialized at d = 0 within "
        "the ±500 m envelope. Numbered circles mark transition onsets. Candidate offsets are spaced "
        "100 m apart; maneuvers are continuous and may span multiple intervals. The triangle in (b) "
        "marks the horizontal projection of BS08, the only site inside the modeled lateral band. "
        "Dotted contours indicate SINR = −1.5 dB. The color scale is separately linear below and above "
        "this threshold. Decisions use predicted exposure and policy benefits over a 240 s horizon; "
        "the trajectory is not a globally optimal SINR path."
    )
    notes = (
        "## Methods and provenance\n\n"
        "The field retains the archived 100 m lateral and approximately 50 m longitudinal grid; "
        "the image introduces no new radio samples or trajectory optimization. Reference coordinate q "
        "is not the exact arc length of the smoothed curve. Triangles identify ground positions, not "
        "base stations at the aircraft altitude. BS16 is beyond the route endpoint and is shown only "
        "at its true plan-view position; it is not artificially placed at the end of the SINR band.\n\n"
        "At each eligible 20 s decision, the controller compares a candidate maneuver against staying "
        "in the current offset. It retains the 30 s available exposure history and requires a predicted "
        "mean-exposure reduction of at least 0.02 and non-worsening policy-spacing priority. Eligible "
        "candidates are ranked by policy-spacing benefit, exposure benefit, shorter duration, then target "
        "offset. Quintic maneuvers and a 30 s cooldown constrain subsequent decisions. This is not "
        "instantaneous SINR maximization or a full-route optimum. Higher SINR is better; lower "
        "below-threshold exposure is better. No neighboring traffic, ACC response, or capacity estimate "
        "is included.\n\n"
        "Source: R0008, adaptive_500. The SINR field, four maneuvers, and all scientific results are "
        "unchanged. Original data hashes, derived station projections, plotting code, and export hashes "
        "are retained in this figure revision. See [the study report](../../../reports/R0008.md) "
        "for propagation-model assumptions and limitations.\n"
    )
    (output / "caption.md").write_text("# Figure caption\n\n" + caption + "\n\n" + notes, encoding="utf-8")
    (output / "base-station-projections.json").write_text(json.dumps(stations, indent=2) + "\n")
    outputs = []
    for ext in ("png", "svg"):
        path = output / f"01-radio-landscape.{ext}"
        fig.savefig(path, dpi=300, bbox_inches="tight", pad_inches=.10, metadata={"Description": caption})
        outputs.append({"path": path.name, "sha256": sha(path)})
    plt.close(fig)
    sources = [Path(__file__).resolve(), Path(__file__).with_name("verify_lateral_study.py").resolve()]
    with zipfile.ZipFile(output / "plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.name)
    for name in ["caption.md", "base-station-projections.json", "plot_source.zip"]:
        outputs.append({"path": name, "sha256": sha(output / name)})
    report = {"run": str(run), "case": "adaptive_500", "revision": "academic-v2",
        "run_manifest_sha256": sha(run / "manifest.json"),
        "field_sha256": sha(run / "field.npz"), "trace_sha256": sha(run / "adaptive_500/trace.json.gz"),
        "matplotlib_version": matplotlib.__version__, "numpy_version": np.__version__,
        "projection_method": "all <=10 m grid local minima refined by 60 golden-section steps plus endpoints",
        "geographic_crs": "EPSG:26910", "base_stations_in_layout": len(stations),
        "base_stations_in_sinr_panel": [s["site_id"] for s in stations if s["shown_in_sinr_panel"]],
        "maximum_reconstructed_trajectory_error_m": residual,
        "conservative_annotation_extent_overlaps": overlaps,
        "sources": [{"path": p.name, "sha256": sha(p)} for p in sources],
        "outputs": outputs, "note": "Presentation revision only; no model, radio, trajectory, or policy outputs changed."}
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(output), "stations": len(stations),
                      "stations_in_sinr_panel": report["base_stations_in_sinr_panel"],
                      "annotation_extent_overlaps": overlaps}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plot(args.run, args.output)
