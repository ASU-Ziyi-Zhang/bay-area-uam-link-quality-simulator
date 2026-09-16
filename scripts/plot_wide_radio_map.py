"""Equal-scale, all-station radio context for an archived lateral-study case.

This is a presentation export, not a flight simulation. The new background is
sampled in Cartesian coordinates using the archived radio parameters. Flight
positions, decisions and candidate limits are read without modification.
"""
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
from matplotlib.patches import ConnectionPatch, Rectangle
import matplotlib.patheffects as pe
import numpy as np

from verify_lateral_study import independent_reference, read


INK = "#202020"
GOLD = "#F2BD3C"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def chord_frame(start, end):
    """Rigid rotation only: no projection onto, or unfolding of, the route."""
    origin = np.asarray(start, dtype=float)
    direction = np.asarray(end, dtype=float) - origin
    length = np.linalg.norm(direction)
    if length <= 0:
        raise ValueError("reference endpoints must differ")
    e = direction / length
    return origin, np.column_stack((e, [-e[1], e[0]]))


def display_bounds(points, padding_m=1200., rounding_m=500.):
    """Data-driven extent includes sites, reference, envelope and trajectory."""
    points = np.asarray(points)
    lower = np.floor((points.min(axis=0) - padding_m) / rounding_m) * rounding_m
    upper = np.ceil((points.max(axis=0) + padding_m) / rounding_m) * rounding_m
    return lower, upper


def radio_at(positions, spec, chunk_size=16384):
    """Evaluate archived isotropic radio equations at arbitrary physical XYZ.

    Select nearest sites by 3-D distance, then sum interference only from that
    set. No artificial coverage radius or aircraft-load dependence is added.
    Cross-checked against the simulator kernel and archived field/observations.
    """
    positions = np.asarray(positions, dtype=float)
    shape = positions.shape[:-1]
    positions = positions.reshape(-1, 3)
    cfg = spec["radio"]
    sites = np.array([[s["x_m"], s["y_m"], s["height_m"] if s["height_m"] is not None
                       else cfg["assumed_bs_height_m"]] for s in spec["base_stations"]], float)
    if not np.all(np.isfinite(sites)):
        raise ValueError("base-station heights must be resolved")
    k = len(sites) if cfg["served_set_size"] is None else min(cfg["served_set_size"], len(sites))
    sinr, serving, outside = [], [], []
    for i in range(0, len(positions), chunk_size):
        delta = positions[i:i + chunk_size, None, :] - sites[None, :, :]
        squared = np.maximum(np.sum(delta * delta, axis=-1), np.finfo(float).tiny)
        dbm = cfg["eirp_dbm"] + cfg["receiver_gain_db"] - 28. - 11. * np.log10(squared)
        dbm -= 20. * np.log10(cfg["frequency_ghz"])
        selected = np.argsort(squared, axis=-1, kind="stable")[:, :k]
        powers = 10. ** (dbm / 10.)
        best = np.argmax(dbm, axis=-1)
        desired = powers[np.arange(len(best)), best]
        interference = np.maximum(np.take_along_axis(powers, selected, axis=-1).sum(axis=-1)
                                  - desired, np.finfo(float).tiny)
        sinr.append(10. * np.log10(desired / (interference + 10. ** (cfg["noise_dbm"] / 10.))))
        serving.append(best)
        horizontal = np.linalg.norm(delta[..., :2], axis=-1)
        outside.append(np.any(np.take_along_axis(horizontal, selected, axis=-1) > 4000., axis=-1))
    return {"sinr_db": np.concatenate(sinr).reshape(shape),
            "serving_bs": np.concatenate(serving).reshape(shape),
            "any_selected_link_beyond_4km": np.concatenate(outside).reshape(shape)}


def load_inputs(run, case_id="minimal_900"):
    run = Path(run)
    manifest = read(run / "manifest.json")
    if manifest["status"] != "completed":
        raise ValueError("only completed runs can be illustrated")
    for item in manifest["outputs"]:
        if sha(run / item["path"]) != item["sha256"]:
            raise ValueError(f"run hash mismatch: {item['path']}")
    spec = read(run / "resolved_config.json")
    case = next(c for c in spec["cases"] if c["id"] == case_id)
    if case["kind"] != "minimum_change":
        raise ValueError("this figure's labels require a minimum-change case")
    with zipfile.ZipFile(run / "source_snapshot.zip") as archive:
        for item in manifest["code"] + manifest["inputs"]:
            if hashlib.sha256(archive.read(item["path"])).hexdigest() != item["sha256"]:
                raise ValueError(f"source archive mismatch: {item['path']}")
        scenario_path = next(i["path"] for i in manifest["inputs"] if i["path"].endswith("/scenario.json"))
        scenario = json.loads(archive.read(scenario_path))
        if scenario["corridor"].get("target_crs", "EPSG:26910") != "EPSG:26910":
            raise ValueError("expected NAD83 / UTM zone 10N")
        frame = independent_reference(archive, manifest, case["parameters"]["center_control_step_m"])
    with np.load(run / "field.npz") as data:
        field = {k: data[k].copy() for k in data.files}
    folder = run / case_id
    events = [e for e in read(folder / "events.json") if e["status"] == "transition_accepted"]
    trace = read(folder / "trace.json.gz") + [read(folder / "terminal.json")]
    q = np.array([r["q_m"] for r in trace])
    d = np.array([r["offset_m"] for r in trace])
    xyz = np.array([r["position_m"] for r in trace])
    center, _, normal, _, _ = frame(q)
    residual = float(np.max(np.linalg.norm(center + d[:, None] * normal - xyz[:, :2], axis=-1)))
    if residual > 1e-6 or np.max(np.abs(d)) > max(np.abs(case["offsets_m"])):
        raise ValueError("archived trajectory disagrees with reference/envelope")
    observations = read(folder / "observations.json.gz")
    oq = np.array([r["q_m"] for r in observations])
    od = np.array([r["offset_m"] for r in observations])
    oc, _, on, _, _ = frame(oq)
    predicted = radio_at(np.column_stack((oc + od[:, None] * on,
                          np.full(len(oq), case["parameters"]["altitude_m"]))), spec)
    error = float(np.max(np.abs(predicted["sinr_db"] - [r["sinr_db"] for r in observations])))
    if error > 1e-8 or not np.array_equal(predicted["serving_bs"], [r["serving_bs"] for r in observations]):
        raise ValueError("display radio equations disagree with archived observations")
    fc, _, fn, _, _ = frame(field["q_m"])
    field_xy = fc[None, :, :] + field["offsets_m"][:, None, None] * fn[None, :, :]
    field_radio = radio_at(np.concatenate((field_xy, np.full((*field_xy.shape[:-1], 1),
                                     case["parameters"]["altitude_m"])), axis=-1), spec)
    field_error = float(np.max(np.abs(field_radio["sinr_db"] - field["sinr_db"])))
    if field_error > 1e-8 or not np.array_equal(field_radio["serving_bs"], field["serving_bs"]):
        raise ValueError("display radio equations disagree with archived field")
    return {"spec": spec, "case": case, "frame": frame, "field": field, "xyz": xyz, "q": q, "events": events,
            "time_s": np.array([r["t_s"] for r in trace]),
            "d": d, "position_error_m": residual, "observation_error_db": error,
            "field_error_db": field_error, "observation_count": len(observations)}


def focus_geometry(inputs, event_number, origin, rotation):
    """Locate exact archived event boundaries; never infer a move from pixels."""
    if not 1 <= event_number <= len(inputs["events"]):
        raise ValueError("focus event number must identify an accepted maneuver")
    event = inputs["events"][event_number - 1]
    times = [event["t_s"], event["t_s"] + event["duration_s"]]
    indices = []
    for t in times:
        matches = np.flatnonzero(np.isclose(inputs["time_s"], t, atol=1e-8, rtol=0))
        if len(matches) != 1:
            raise ValueError("exact maneuver boundary missing from archived trace")
        indices.append(int(matches[0]))
    frame = inputs["frame"]
    local = lambda p: (np.asarray(p) - origin) @ rotation
    q0, q1 = inputs["q"][indices]
    context = (inputs["q"] >= q0 - 400) & (inputs["q"] <= q1 + 400)
    points = np.vstack((local(inputs["xyz"][context, :2]), local(frame(inputs["q"][context])[0])))
    low = np.floor((points.min(axis=0) - [150, 300]) / 100) * 100
    high = np.ceil((points.max(axis=0) + [150, 300]) / 100) * 100
    target = event["target_m"]
    center, tangent, normal, _, _ = frame(q1)
    source_at_end = center + event["source_m"] * normal
    end = inputs["xyz"][indices[1], :2]
    if np.linalg.norm(end - (center + target * normal)) > 1e-6:
        raise ValueError("maneuver endpoint is not the accepted target")
    return {"event_number": event_number, "start_s": times[0], "end_s": times[1],
            "start_q_m": float(q0), "end_q_m": float(q1),
            "source_offset_m": event["source_m"], "target_offset_m": target,
            "displacement_m": float(np.linalg.norm(end - source_at_end)),
            "low_m": low.tolist(), "high_m": high.tolist(),
            "start_xy_m": local(inputs["xyz"][indices[0], :2]).tolist(),
            "end_xy_m": local(end).tolist(), "source_at_end_xy_m": local(source_at_end).tolist(),
            "end_tangent": (tangent @ rotation).tolist()}


def draw_focus(fig, ax, inputs, detail, origin, rotation, norm, fig_width, fig_height, plot_height):
    """Equal-scale detail with precise normal-offset dimension and linked ROI."""
    low, high = np.array(detail["low_m"]), np.array(detail["high_m"])
    zoom = fig.add_axes([1.75 / fig_width, .65 / fig_height, 12.4 / fig_width, plot_height / fig_height])
    xe, ye = [np.arange(lo, hi + 5, 10.) for lo, hi in zip(low, high)]
    xx, yy = np.meshgrid((xe[:-1] + xe[1:]) / 2, (ye[:-1] + ye[1:]) / 2)
    xy = origin + np.stack((xx, yy), axis=-1) @ rotation.T
    radio = radio_at(np.concatenate((xy, np.full((*xx.shape, 1), inputs["case"]["parameters"]["altitude_m"])), axis=-1), inputs["spec"])
    zoom.pcolormesh(xe / 1000, ye / 1000, radio["sinr_db"], cmap="RdBu", norm=norm,
                    shading="flat", rasterized=True)
    local = lambda p: (np.asarray(p) - origin) @ rotation / 1000
    actual = local(inputs["xyz"][:, :2])
    ref = local(inputs["frame"](inputs["q"])[0])
    zoom.plot(actual[:, 0], actual[:, 1], color=GOLD, lw=1.6,
        path_effects=[pe.Stroke(linewidth=2.5, foreground=INK), pe.Normal()])
    zoom.plot(ref[:, 0], ref[:, 1], color=INK, lw=.9, ls=(0, (4, 4)))
    zoom.set(xlim=(low[0] / 1000, high[0] / 1000), ylim=(low[1] / 1000, high[1] / 1000),
             xlabel=r"Along-chord coordinate, $X$ (km)", ylabel=r"Cross-chord coordinate, $Y$ (km)")
    zoom.set_aspect("equal", adjustable="box")
    annotations, overview_annotations = [], []
    for label, key, text in [("A", "start_xy_m", f"A: start, {detail['start_s']:g} s"),
                             ("B", "end_xy_m", f"B: end, {detail['end_s']:.1f} s")]:
        point = np.array(detail[key]) / 1000
        for panel in (ax, zoom):
            panel.plot(*point, "o", ms=4.3, mfc="white", mec=INK, mew=.9, zorder=10)
        overview_annotations.append(ax.annotate(label, point, xytext=(-8 if label == "A" else 8, 9),
            textcoords="offset points", ha="center", va="bottom", fontsize=10,
            bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .5}, zorder=11))
        annotations.append(zoom.annotate(text, point,
            xytext=(-15 if label == "B" else 0, 24), textcoords="offset points",
            ha="right" if label == "B" else "center", va="bottom", fontsize=11,
            arrowprops={"arrowstyle": "-", "lw": .65, "color": INK},
            bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .7}))
    a = np.array(detail["source_at_end_xy_m"]) / 1000
    b = np.array(detail["end_xy_m"]) / 1000
    zoom.annotate("", xy=b, xytext=a,
                  arrowprops={"arrowstyle": "|-|", "lw": .9, "color": INK, "shrinkA": 0, "shrinkB": 0}, zorder=11)
    annotations.append(zoom.annotate(f"{detail['displacement_m']:.0f} m", (a + b) / 2,
        xytext=(19, 0), textcoords="offset points", va="center", fontsize=11,
        bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none", "pad": .7}))
    box = Rectangle(low / 1000, *(high - low) / 1000, fill=False, edgecolor=INK,
                    lw=1.05, linestyle=(0, (4, 3)), zorder=9)
    ax.add_patch(box)
    for x, corner in [(low[0], 0.), (high[0], 1.)]:
        connector = ConnectionPatch(xyA=(x / 1000, low[1] / 1000), coordsA=ax.transData,
            xyB=(corner, 1), coordsB=zoom.transAxes, color="#696969", lw=.7,
            linestyle=(0, (4, 4)), clip_on=False, zorder=.1)
        fig.add_artist(connector)
    return zoom, annotations, overview_annotations, {"x_edges_m": xe, "y_edges_m": ye, **radio}


def plot(run, output, case_id="minimal_900", grid_m=50., focus_event=None):
    run, output = Path(run).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError("choose a new figure revision directory")
    if not np.isfinite(grid_m) or grid_m < 10:
        raise ValueError("display grid must be finite and at least 10 m")
    inputs = load_inputs(run, case_id)
    spec, case, frame = inputs["spec"], inputs["case"], inputs["frame"]
    cfg = case["parameters"]
    length = inputs["field"]["q_m"][-1]
    origin, rotation = chord_frame(frame(0.)[0], frame(length)[0])
    focus = None if focus_event is None else focus_geometry(inputs, focus_event, origin, rotation)
    local = lambda xy: (np.asarray(xy) - origin) @ rotation
    q = np.linspace(0, length, int(np.ceil(length / 10.)) + 1)
    center, _, normal, _, _ = frame(q)
    width = max(np.abs(case["offsets_m"]))
    lower_band, upper_band = local(center - width * normal), local(center + width * normal)
    reference, trajectory = local(center), local(inputs["xyz"][:, :2])
    stations = np.array([[s["x_m"], s["y_m"]] for s in spec["base_stations"]])
    ground = local(stations)
    lo, hi = display_bounds(np.vstack((ground, reference, trajectory, lower_band, upper_band)))
    xe = np.arange(lo[0], hi[0] + grid_m * .999, grid_m)
    ye = np.arange(lo[1], hi[1] + grid_m * .999, grid_m)
    hi = np.array([xe[-1], ye[-1]])
    xx, yy = np.meshgrid((xe[:-1] + xe[1:]) / 2., (ye[:-1] + ye[1:]) / 2.)
    display_xy = np.stack((xx, yy), axis=-1)
    real_xy = origin + display_xy @ rotation.T
    positions = np.concatenate((real_xy, np.full((*xx.shape, 1), cfg["altitude_m"])), axis=-1)
    radio = radio_at(positions, spec)
    z = radio["sinr_db"]
    plt.rcParams.update({"font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 12,
        "axes.labelsize": 13, "axes.titlesize": 15, "text.color": INK, "axes.labelcolor": INK,
        "axes.edgecolor": INK, "xtick.color": INK, "ytick.color": INK, "axes.linewidth": .7,
        "svg.fonttype": "none", "svg.hashsalt": "uam-all-bs-cartesian-v1",
        "figure.facecolor": "white", "savefig.facecolor": "white"})
    # The axes' physical dimensions follow the data range; neither axis is stretched.
    plot_width = 14.2
    plot_height = plot_width * (hi[1] - lo[1]) / (hi[0] - lo[0])
    focus_height = 0. if focus is None else 12.4 * (focus["high_m"][1] - focus["low_m"][1]) / (focus["high_m"][0] - focus["low_m"][0])
    extra_height = 0. if focus is None else focus_height + 1.2
    fig_width, fig_height = 16.2, plot_height + 1.7 + extra_height
    fig = plt.figure(figsize=(fig_width, fig_height))
    ax = fig.add_axes([.85 / fig_width, (.65 + extra_height) / fig_height, plot_width / fig_width, plot_height / fig_height])
    cax = fig.add_axes([15.25 / fig_width, (.65 + extra_height) / fig_height, .15 / fig_width, plot_height / fig_height])
    norm = TwoSlopeNorm(vmin=float(np.floor(z.min())), vcenter=cfg["threshold_db"],
                        vmax=float(np.ceil(z.max())))
    mesh = ax.pcolormesh(xe / 1000, ye / 1000, z, cmap="RdBu", norm=norm,
                         shading="flat", rasterized=True)
    for border in (lower_band, upper_band):
        ax.plot(border[:, 0] / 1000, border[:, 1] / 1000, color="#646464", lw=.8, ls=(0, (5, 3)))
    for i in (0, -1):
        cap = np.vstack((lower_band[i], upper_band[i])) / 1000
        ax.plot(cap[:, 0], cap[:, 1], color="#646464", lw=.8, ls=(0, (5, 3)))
    ax.plot(trajectory[:, 0] / 1000, trajectory[:, 1] / 1000, color=GOLD, lw=2.1,
            path_effects=[pe.Stroke(linewidth=3.3, foreground=INK), pe.Normal()])
    # Coincident portions remain visible as a dashed baseline over the yellow trace.
    ax.plot(reference[:, 0] / 1000, reference[:, 1] / 1000, color=INK, lw=.85, ls=(0, (4, 4)))
    ax.scatter(ground[:, 0] / 1000, ground[:, 1] / 1000, marker="^", s=44,
               facecolor="white", edgecolor=INK, linewidth=.9, zorder=6)
    offsets = {"BS05": (-3, 14), "BS06": (0, -17), "BS07": (-7, 16), "BS08": (0, -19),
               "BS09": (0, -17), "BS10": (0, 15), "BS11": (-13, 17), "BS12": (14, -18),
               "BS13": (-16, -19), "BS14": (7, 17), "BS15": (0, 17), "BS16": (-3, -18)}
    annotations = []
    for site, point in zip(spec["base_stations"], ground / 1000):
        annotations.append(ax.annotate(site["site_id"], point,
            xytext=offsets.get(site["site_id"], (0, 15)), textcoords="offset points",
            ha="center", va="center", fontsize=11,
            arrowprops={"arrowstyle": "-", "lw": .5, "color": INK},
            bbox={"facecolor": "white", "alpha": .88, "edgecolor": "none", "pad": .5}))
    for index, label, delta in [(0, "Origin", (3, 16)), (-1, "Reference end", (-10, 18))]:
        point = reference[index] / 1000
        ax.plot(*point, "s", ms=4, mfc="white", mec=INK, zorder=7)
        annotations.append(ax.annotate(label, point, xytext=delta, textcoords="offset points",
            ha="center", fontsize=10.5, bbox={"facecolor": "white", "alpha": .88, "edgecolor": "none", "pad": .5}))
    ax.set(xlim=(lo[0] / 1000, hi[0] / 1000), ylim=(lo[1] / 1000, hi[1] / 1000),
           xlabel=r"Along-chord coordinate, $X$ (km)", ylabel=r"Cross-chord coordinate, $Y$ (km)")
    ax.set_aspect("equal", adjustable="box")
    prefix = "" if focus is None else "(a) "
    ax.set_title(prefix + rf"Modeled SINR and base-station layout ($h = {cfg['altitude_m']:g}$ m)", loc="left", pad=44)
    handles = [Line2D([], [], color=INK, ls="--", lw=.9, label="Reference route"),
        Line2D([], [], color=GOLD, lw=2.1, label="Minimum-change trajectory",
               path_effects=[pe.Stroke(linewidth=3.3, foreground=INK), pe.Normal()]),
        Line2D([], [], color="#646464", lw=.8, ls=(0, (5, 3)), label=rf"Candidate envelope ($\pm${width / 1000:g} km)"),
        Line2D([], [], color="none", marker="^", mfc="white", mec=INK, ms=7, label="Base station")]
    ax.legend(handles=handles, loc="lower left", bbox_to_anchor=(0, 1.04), borderaxespad=0,
              ncol=4, frameon=False, fontsize=11.5, columnspacing=2.4, handlelength=2.7)
    cb = fig.colorbar(mesh, cax=cax)
    cb.set_label("SINR (dB)", labelpad=9)
    ticks = [norm.vmin, cfg["threshold_db"], 5., 15., 25., norm.vmax]
    ticks = sorted(set(t for t in ticks if norm.vmin <= t <= norm.vmax))
    cb.set_ticks(ticks)
    cb.set_ticklabels([f"{t:g}" for t in ticks])
    cb.ax.tick_params(labelsize=11)
    zoom = None
    if focus is not None:
        zoom, focus_annotations, overview_annotations, focus_field = draw_focus(fig, ax, inputs, focus, origin, rotation,
                                           norm, fig_width, fig_height, focus_height)
        annotations.extend(overview_annotations)
    fig.canvas.draw()
    # Use text extents only (leaders may cross by design); fail instead of publishing overlaps.
    boxes = [plt.Text.get_window_extent(a, fig.canvas.get_renderer()) for a in annotations]
    overlaps = [(i, j) for i, b in enumerate(boxes) for j, c in enumerate(boxes) if i < j and b.overlaps(c)]
    clipped = [i for i, b in enumerate(boxes) if not (ax.bbox.contains(*b.get_points()[0])
                                                   and ax.bbox.contains(*b.get_points()[1]))]
    pixel_origin, pixel_x, pixel_y = ax.transData.transform([[0, 0], [1, 0], [0, 1]])
    scale_ratio = float(np.linalg.norm(pixel_x - pixel_origin) / np.linalg.norm(pixel_y - pixel_origin))
    if zoom is not None:
        p0, px, py = zoom.transData.transform([[0, 0], [1, 0], [0, 1]])
        focus["rendered_x_to_y_metric_scale"] = float(np.linalg.norm(px - p0) / np.linalg.norm(py - p0))
        focus["linear_magnification"] = float(np.linalg.norm(px - p0) / np.linalg.norm(pixel_x - pixel_origin))
        zoom.set_title(f"(b) Lateral maneuver A–B ({focus['linear_magnification']:.1f}× magnification)",
                       loc="center", pad=11)
        fig.canvas.draw()
        fboxes = [plt.Text.get_window_extent(a, fig.canvas.get_renderer()) for a in focus_annotations]
        focus["annotation_overlaps"] = [(i, j) for i, b in enumerate(fboxes) for j, c in enumerate(fboxes) if i < j and b.overlaps(c)]
        focus["clipped_annotations"] = [i for i, b in enumerate(fboxes) if not (
            zoom.bbox.contains(*b.get_points()[0]) and zoom.bbox.contains(*b.get_points()[1]))]
        if focus["annotation_overlaps"] or focus["clipped_annotations"] or abs(focus["rendered_x_to_y_metric_scale"] - 1.) > 1e-10:
            plt.close(fig)
            raise ValueError(f"focus visual QA failed: {focus}")
    if overlaps or clipped or abs(scale_ratio - 1.) > 1e-10:
        plt.close(fig)
        raise ValueError(f"visual QA failed: overlaps={overlaps}, clipped={clipped}, scale={scale_ratio}")
    output.mkdir(parents=True)
    grid_spacing = float(spec["candidate_spacing_m"])
    targets = [float(inputs["d"][0])] + [float(e["target_m"]) for e in inputs["events"]]
    sequence = " → ".join(f"{d:+g}" if d else "0" for d in targets)
    caption = (
        f"Modeled SINR at h = {cfg['altitude_m']:g} m with all {len(stations)} base stations in the scenario, "
        "the reference route, and the archived minimum-change trajectory. Coordinates are a rigid "
        "translation and rotation of NAD83 / UTM zone 10N: the origin is the reference-route start, "
        "X follows the start-to-end chord, and Y is positive to its left. Both axes have the same "
        "metric scale. The reference route remains curved; Y is not the route-relative offset d, "
        "and X is not route progress q. Triangles mark actual BS ground positions, not normal "
        "projections or antennas at aircraft altitude. Dashed outer boundaries mark the "
        f"±{width:g} m candidate envelope; {grid_spacing:g} m candidate spacing and the flight result are unchanged. "
        f"The yellow trajectory contains {len(inputs['events'])} accepted lateral maneuver(s), with "
        f"offset sequence {sequence} m. Wider map coverage does not widen the permitted flight envelope."
    )
    if focus is not None:
        caption += (
            f" Panel (b) magnifies the dashed rectangle in (a) by {focus['linear_magnification']:.2f} "
            f"in both directions. A is the maneuver start (t = {focus['start_s']:g} s, "
            f"q = {focus['start_q_m'] / 1000:.3f} km); B is completion "
            f"(t = {focus['end_s']:.3f} s, q = {focus['end_q_m'] / 1000:.3f} km). "
            f"The {focus['displacement_m']:.0f} m dimension is measured along the reference normal "
            "at B between source and target offsets, not vertically in the rotated coordinates. "
            "Both panels share the same color normalization. The detail background is sampled "
            "at 10 m resolution using unchanged radio equations; the trajectory uses unchanged "
            "archived positions. No transverse exaggeration is applied."
        )
    notes = (
        "## Display field and limitations\n\n"
        f"This is a presentation-only derivative of {run.name}/{case_id}, not a new controller run. "
        f"The {grid_m:g} m × {grid_m:g} m cell-centered Cartesian field is newly evaluated from the "
        f"archived radio settings: common {spec['radio']['eirp_dbm']:g} dBm EIRP, "
        f"{spec['radio']['frequency_ghz']:g} GHz, {spec['radio']['noise_dbm']:g} dBm noise, "
        f"nearest {spec['radio']['served_set_size']} sites by 3-D distance, strongest serving site "
        "and the other selected sites as co-channel interferers. "
        f"The scale is separately linear below and above Θ = {cfg['threshold_db']:g} dB; it is not a measured "
        "coverage map. No circular coverage boundary, new stations, antenna patterns, terrain, "
        "traffic load, or capacity estimate is introduced. Finite station coverage and the "
        "propagation model's documented 4 km horizontal-distance range limit interpretation; "
        "samples using longer links are extrapolations, including outside the candidate envelope. "
        "The saved field flags every cell with any selected link beyond 4 km.\n\n"
        "The figure uses Cartesian locations specifically to avoid extending folded normal "
        "coordinates far from the reference curve. BS16 retains its actual location beyond the "
        "reference endpoint. No trajectory is bent, rescaled independently by axis, or replanned "
        "to improve the picture. Equal aspect ratio is verified from the rendered axes. Showing "
        "a gentler-looking curve is not evidence of a passenger-comfort or flight-performance limit.\n\n"
        "## Reproduction\n\n"
        f"`python scripts/plot_wide_radio_map.py research/dynamic-transitions/runs/{run.name} "
        f"--case {case_id} " + ("" if focus is None else f"--focus-event {focus_event} ") +
        "--output <new-figure-directory>`\n\n"
        "The manifest records input, source and output hashes; the source ZIP preserves the "
        "plotter, reference reconstruction and regression tests. All archived run files are "
        "validated before plotting and left untouched.\n"
    )
    (output / "caption.md").write_text("# Figure caption\n\n" + caption + "\n\n" + notes, encoding="utf-8")
    np.savez_compressed(output / "display-field.npz", x_edges_m=xe, y_edges_m=ye,
        origin_utm_m=origin, rotation=rotation, altitude_m=cfg["altitude_m"], **radio)
    geometry = {"crs": "EPSG:26910", "origin_utm_m": origin.tolist(), "rotation": rotation.tolist(),
        "display_lower_m": lo.tolist(), "display_upper_m": hi.tolist(), "candidate_envelope_m": float(width),
        "stations": [{"site_id": s["site_id"], "utm_xy_m": point.tolist(), "display_xy_m": shown.tolist()}
                      for s, point, shown in zip(spec["base_stations"], stations, ground)]}
    if focus is not None:
        geometry["focus"] = focus
        np.savez_compressed(output / "focus-field.npz", origin_utm_m=origin, rotation=rotation,
                            altitude_m=cfg["altitude_m"], **focus_field)
    (output / "geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    for ext in ("png", "svg"):
        fig.savefig(output / f"01-radio-context.{ext}", dpi=300, metadata={"Description": caption})
    plt.close(fig)
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), Path(__file__).with_name("verify_lateral_study.py").resolve(),
               root / "tests/test_wide_radio_figure.py"]
    with zipfile.ZipFile(output / "plot_source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(root))
    report = {"run": str(run), "case": case_id, "run_manifest_sha256": sha(run / "manifest.json"),
        "revision": "equal-scale-all-stations-v1" if focus is None else "equal-scale-maneuver-focus-v2",
        "focus": focus, "matplotlib_version": matplotlib.__version__,
        "numpy_version": np.__version__, "display_grid_m": grid_m, "display_cells": int(z.size),
        "sinr_range_db": [float(z.min()), float(z.max())], "color_scale_range_db": [norm.vmin, norm.vmax],
        "color_scale_midpoint_db": cfg["threshold_db"], "all_stations_in_frame": bool(np.all((ground >= lo) & (ground <= hi))),
        "station_count": len(stations), "rendered_x_to_y_metric_scale": scale_ratio,
        "annotation_overlaps": overlaps, "clipped_annotations": clipped,
        "maximum_reconstructed_position_error_m": inputs["position_error_m"],
        "archived_observations_checked": inputs["observation_count"],
        "maximum_observation_sinr_error_db": inputs["observation_error_db"],
        "archived_field_samples_checked": int(inputs["field"]["sinr_db"].size),
        "maximum_archived_field_sinr_error_db": inputs["field_error_db"],
        "display_fraction_any_selected_link_beyond_4km": float(radio["any_selected_link_beyond_4km"].mean()),
        "sources": [{"path": p.relative_to(root).as_posix(), "sha256": sha(p)} for p in sources],
        "outputs": [{"path": p.name, "sha256": sha(p)} for p in sorted(output.iterdir())],
        "note": "New display-only Cartesian radio samples; no archived flight, policy or controller changes."}
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", default="minimal_900")
    parser.add_argument("--grid-m", type=float, default=50.)
    parser.add_argument("--focus-event", type=int, help="one-based accepted maneuver to magnify")
    args = parser.parse_args()
    plot(args.run, args.output, args.case, args.grid_m, args.focus_event)
