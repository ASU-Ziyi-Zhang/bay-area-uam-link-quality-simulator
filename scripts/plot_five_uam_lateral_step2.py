"""Figures and tables for the restored step-2 five-aircraft study.

Reads a finished run folder (records, stage0.json, validation.json) and writes
figures plus Markdown/CSV tables to a new directory. Plotting a run whose
validation failed is refused unless ``--allow-failed`` is given, in which case
every figure is stamped as a diagnostic and must not be reported as a result.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from run_five_uam_lateral_step2 import make_shape  # noqa: E402

INK, GREY = "#1a1a1a", "#6b6b6b"
PALETTE = ("#1f4e79", "#b5451b", "#2e7d32", "#6a3d9a", "#b8860b")
REASON = {
    "no_admissible_candidate": ("#9e9e9e", "no admissible candidate (cannot)"),
    "gain_below_threshold": ("#9ecae1", "gain ≤ gate (not worth it)"),
    "future_start": ("#f2c14e", "best admissible starts later (wait)"),
    "gain_above_threshold": ("#7f1d1d", "change starts"),
    "continue_committed_maneuver": ("#d9534f", "changing lanes"),
    "target_lane_following": ("#5cb85c", "in target lane"),
    "lane_changes_disabled": ("#ffffff", "stay control"),
    "stay_not_admissible": ("#1a1a1a", "stay baseline not admissible (stopped)"),
    "continuation_not_admissible": ("#1a1a1a", "stopped"),
    "execution_prefix_rejected": ("#1a1a1a", "stopped"),
}
STOP_REASONS = {"stay_not_admissible", "continuation_not_admissible", "execution_prefix_rejected"}


def style():
    plt.rcParams.update({
        "font.family": "STIXGeneral", "mathtext.fontset": "stix", "font.size": 11,
        "axes.labelsize": 11.5, "axes.titlesize": 12.5, "text.color": INK,
        "axes.labelcolor": INK, "axes.edgecolor": INK, "xtick.color": INK,
        "ytick.color": INK, "axes.linewidth": .7, "svg.fonttype": "none",
        "svg.hashsalt": "uam-five-uam-lateral-step2",
        "figure.facecolor": "white", "savefig.facecolor": "white", "legend.fontsize": 9,
    })


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def shape_label(name):
    if name == "quintic":
        return "quintic = r(2,2)"
    if name == "bezier3":
        return "cubic Bézier"
    if name.startswith("beta_"):
        p, q = (int(x) for x in name.split("_")[1:3])
        return f"r({p},{q})" + (" early peak" if p < q else " late peak" if p > q else "")
    if name.startswith("clothoid"):
        return "three-clothoid"
    return name


def field_label(spec, name):
    f = spec["fields"][name]
    if f["kind"] == "sharp":
        return "sharp field (prescribed −20 / +20 dB)"
    return f"bad length {f['bad_length_m']:g} m, lateral gradient {f['gradient_db_per_100m']:g} dB/100 m"


def reason_runs(decisions, horizon):
    ds = [d for d in decisions if d.get("reason") != "before_first_decision_phase"]
    runs = []
    for i, d in enumerate(ds):
        end = ds[i + 1]["t_s"] if i + 1 < len(ds) else (d["t_s"] + 5.0 if d["reason"] in STOP_REASONS else horizon)
        if runs and runs[-1][0] == d["reason"]:
            runs[-1][2] = end
        else:
            runs.append([d["reason"], d["t_s"], end])
    return runs


def first_change(rec):
    d = next((d for d in rec["decisions"] if d.get("reason") == "gain_above_threshold"), None)
    if d is None:
        return None, None, None
    admitted = [c for c in d["candidates"] if c["admitted"]]
    best = min(admitted, key=lambda c: c["cost"])
    return d["t_s"], best["label"], d["gain"]


def stamp(fig, failed):
    if failed:
        fig.text(.5, .5, "FAILED VALIDATION — diagnostic only", rotation=28, fontsize=38,
                 color="#c62828", alpha=.22, ha="center", va="center")


def decision_figure(records, spec, dt, horizon, verdicts, failed):
    cases = spec["part_A"]
    rows = []
    variants = ([(f"allow_{name}", f"gate {name}") for name in spec["gate"]["variants"]]
                + [("stay", "stay control")])
    for case in cases:
        for variant, tag in variants:
            rec = records.get(f"A__{case['id']}__{variant}__dt{dt:g}")
            if rec is not None:
                rows.append((case, variant, tag, rec))
    fig, ax = plt.subplots(figsize=(10.2, .36 * len(rows) + 1.9))
    used = set()
    for y, (case, variant, tag, rec) in enumerate(rows):
        for reason, a, b in reason_runs(rec["decisions"], horizon):
            colour = REASON.get(reason, ("#cccccc", reason))[0]
            ax.barh(y, b - a, left=a, height=.74, color=colour, edgecolor=GREY if colour == "#ffffff" else "white", lw=.4)
            used.add(reason)
        verdict = verdicts.get(f"A__{case['id']}__{variant}__dt{dt:g}")
        ax.text(horizon + 2, y, "pass" if verdict else "FAIL", va="center", fontsize=8.5,
                color=GREY if verdict else "#c62828")
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([f"{c['id'].split('_')[0]}  {tag}" for c, _, tag, _ in rows], fontsize=8.5)
    for y, (case, variant, _, _) in enumerate(rows):
        if variant == variants[0][0] and y:
            ax.axhline(y - .5, color=GREY, lw=.5)
    ax.invert_yaxis()
    ax.set_xlim(0, horizon + 12)
    ax.set_xlabel("time  (s)")
    ax.set_title("Part A — what the decision rule answered at each 5 s decision", loc="left", pad=8)
    handles = [plt.Rectangle((0, 0), 1, 1, fc=REASON[r][0], ec=GREY, lw=.4) for r in REASON if r in used]
    labels = [REASON[r][1] for r in REASON if r in used]
    # Place the legend a fixed ~0.6 in below the axis, whatever the row count.
    ax.legend(handles, labels, frameon=False, ncol=3, loc="upper left",
              bbox_to_anchor=(0, -.62 / (.36 * len(rows) + .4)))
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    stamp(fig, failed)
    fig.tight_layout()
    return fig


def shapes_figure(records, spec, dt, shapes, failed):
    field = spec["part_B"]["field"]
    env = spec["resolved_objects"]["envelope"]
    duration = spec["part_B"]["B1"]["duration_s"]
    boundary = spec["part_B"]["scenario"]["lane_separation_m"] / 2
    fig, axes = plt.subplots(4, 1, figsize=(8.8, 12.6), sharex=True,
                             gridspec_kw={"hspace": .32, "top": .925, "bottom": .05})
    fig.text(.085, .982, f"Five lateral shapes, same D = {2 * boundary:g} m and T = {duration:g} s",
             fontsize=14.5, ha="left", va="top")
    fig.text(.085, .962, "Panels run from the third derivative down to the offset itself; each is the integral of the one above.",
             fontsize=9.5, color=GREY, va="top")
    keys = (("jerk_mps3", "lateral jerk  (m s$^{-3}$)", "(a) jerk", None),
            ("accel_mps2", "lateral acceleration  (m s$^{-2}$)", "(b) acceleration", env["lateral_accel_max_mps2"]),
            ("speed_mps", "lateral speed  (m s$^{-1}$)", "(c) speed", env.get("lateral_speed_max_mps")),
            ("offset_m", "lateral offset  (m)", "(d) offset", None))
    for ax, (key, ylabel, title, limit) in zip(axes, keys):
        ax.axhline(0, color=INK, lw=.5)
        if limit:
            for sign in ((-1, 1) if key == "accel_mps2" else (1,)):
                ax.axhline(sign * limit, color=GREY, ls=(0, (5, 3)), lw=1.0)
            ax.annotate(f"envelope limit {limit:g}", (duration + 8, limit), xytext=(0, 4),
                        textcoords="offset points", ha="right", fontsize=9, color=GREY)
        for colour, shape in zip(PALETTE, shapes):
            rec = records[f"B1__{field}__{shape.name}__dt{dt:g}"]
            t = np.asarray(rec["times_s"])
            y = np.asarray(rec["lateral"][key])
            if key == "jerk_mps3":
                ax.step(t, y, where="post", color=colour, lw=1.4, label=shape_label(shape.name))
            else:
                ax.plot(t, y, color=colour, lw=1.5, label=shape_label(shape.name))
            if key == "offset_m" and rec["metrics"].get("crossing_s") is not None:
                ax.plot(rec["metrics"]["crossing_s"], boundary, "o", ms=4.5, color=colour)
        if key == "offset_m":
            ax.axhline(boundary, color=GREY, ls=":", lw=1.0)
            ax.annotate("lane boundary: A counts as the target lane here", (1, boundary), xytext=(0, 4),
                        textcoords="offset points", fontsize=9, color=GREY)
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc="left", pad=6)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    # The offset panel's upper left is the only corner no curve passes through.
    axes[-1].legend(frameon=False, ncol=2, loc="upper left")
    axes[-1].set_xlim(0, duration + 10)
    axes[-1].set_xlabel("time  (s)")
    stamp(fig, failed)
    return fig


def field_figure(records, spec, dt, failed):
    fields = spec["part_C"]["fields"]
    threshold = spec["parameters"]["traffic"]["threshold_db"]
    fig, axes = plt.subplots(2, 2, figsize=(11.2, 7.6), sharex=True, sharey=True)
    for ax, name in zip(axes.flat, fields):
        # Stay is drawn wide and pale underneath, so it stays visible where
        # the decision rule never leaves the lane and the two curves coincide.
        for variant, colour, label, lw, alpha in (("stay", "#b0b0b0", "stay", 3.2, 1.0),
                                                  ("allow", PALETTE[0], "decision rule", 1.3, 1.0)):
            rec = records[f"C2__{name}__{variant}__dt{dt:g}"]
            obs = [(o["t_s"], o["sinr_db"]) for o in rec["observations"] if o["role"] == "A"]
            if obs:
                t, s = zip(*obs)
                ax.plot(t, s, color=colour, lw=lw, alpha=alpha, label=f"{label}: cost {rec['actual_policy_cost']:.1f}")
            if variant == "allow":
                start, _, _ = first_change(rec)
                if start is not None:
                    ax.axvline(start, color=PALETTE[0], ls=(0, (4, 3)), lw=.9)
                    ax.annotate("change starts", (start, threshold), xytext=(4, -14),
                                textcoords="offset points", fontsize=8.5, color=PALETTE[0])
        ax.axhline(threshold, color="#c62828", lw=.9, ls=(0, (5, 3)))
        ax.set_title(field_label(spec, name), fontsize=10.5, loc="left")
        ax.legend(frameon=False, loc="lower right", fontsize=8.5)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    for ax in axes[-1]:
        ax.set_xlabel("time  (s)")
    for ax in axes[:, 0]:
        ax.set_ylabel("A observed SINR  (dB)")
    fig.suptitle("Part C — real-magnitude weak zones: A's observed SINR, stay vs decision rule "
                 f"(red dashed: {threshold:g} dB threshold)", x=.07, ha="left", fontsize=12.5)
    stamp(fig, failed)
    fig.tight_layout()
    return fig


def tables(records, spec, stage0, verdicts, dt, shapes, output):
    lines = [f"# {Path(output).name} tables", ""]
    lines += ["## Stage 0 — measured error bands", "",
              "| Quantity | Value |", "|---|---:|"]
    for k, v in stage0["benefit_by_dt"].items():
        lines.append(f"| A1 benefit at dt = {k} s | {v:.4f} |")
    for k, v in stage0["benefit_by_phase"].items():
        lines.append(f"| A1 benefit at decision phase {k} s | {v:.4f} |")
    fd = stage0["first_decision"]
    lines += [f"| δ_int (integration, finest pair) | {stage0['delta_int']:.4f} |",
              f"| δ_phase (decision-phase spread) | {stage0['delta_phase']:.4f} |",
              f"| δ_pred (first decision: predicted {fd['predicted_gain']:.4f} vs executed {fd['executed_gain']:.4f}) | {stage0['delta_pred']:.4f} |",
              f"| gate, revised = δ_int + δ_pred | {stage0['gates']['revised']:.4f} |",
              f"| gate, original = δ_int + δ_phase + δ_pred | {stage0['gates']['original']:.4f} |", "",
              "| Trajectory check | Declared | Measured in Stage 0 | Threshold used |", "|---|---:|---:|---:|"]
    for k, v in stage0["thresholds"].items():
        lines.append(f"| {k} | {stage0['declared'][k]:.3g} | {stage0['measured'][k]:.3g} | {v:.3g} |")
    rows_a = []
    # One column per gate variant the configuration declares.
    names = list(spec["gate"]["variants"])
    lines += ["", "## Part A — decision-rule cases", "",
              "| Case | Tests " + " ".join(f"| {n} gate: first change" for n in names)
              + " | Stay cost | Decision-rule cost | Checks |",
              "|---|---|" + "---:|" * len(names) + "---:|---:|---|"]
    fmt = lambda t: "never" if t is None else f"{t:.0f} s"
    for case in spec["part_A"]:
        allow = {n: records[f"A__{case['id']}__allow_{n}__dt{dt:g}"] for n in names}
        stay = records.get(f"A__{case['id']}__stay__dt{dt:g}")
        starts = {n: first_change(rec)[0] for n, rec in allow.items()}
        ok = all(verdicts.get(f"A__{case['id']}__{v}__dt{dt:g}", True)
                 for v in [f"allow_{n}" for n in names] + ["stay"])
        primary = allow[names[0]]
        rows_a.append(dict(case=case["id"], tests=case["predicates"],
                           **{f"first_change_{n}_s": starts[n] for n in names},
                           stay_cost=None if stay is None else stay["actual_policy_cost"],
                           decision_rule_cost=primary["actual_policy_cost"],
                           status=primary["status"], checks_pass=ok))
        stay_cell = "—" if stay is None else f"{stay['actual_policy_cost']:.1f}"
        lines.append(f"| {case['id']} | {case['predicates']} "
                     + " ".join(f"| {fmt(starts[n])}" for n in names)
                     + f" | {stay_cell} | {primary['actual_policy_cost']:.1f} ({primary['status']}) | "
                     f"{'pass' if ok else 'FAIL'} |")
    field = spec["part_B"]["field"]
    stay_b1 = records[f"B1__{field}__stay__dt{dt:g}"]
    rows_b = []
    lines += ["", f"## Part B1 — same D and T = {spec['part_B']['B1']['duration_s']:g} s, {field_label(spec, field)}", "",
              f"Stay (no change) total policy cost: {stay_b1['actual_policy_cost']:.1f}.", "",
              "| Shape | Lane-boundary crossing (s) | Peak lateral speed (m/s) | Peak lateral accel (m/s²) | Peak jerk (m/s³) | A samples below threshold | Total cost | Saved vs stay |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for shape in shapes:
        rec = records[f"B1__{field}__{shape.name}__dt{dt:g}"]
        m = rec["metrics"]
        row = dict(shape=shape.name, crossing_s=m["crossing_s"], peak_speed=m["executed_peak_lateral_speed_mps"],
                   peak_accel=m["executed_peak_lateral_accel_mps2"], peak_jerk=m["executed_peak_lateral_jerk_mps3"],
                   bad_samples=m["ego_bad_samples"], cost=rec["actual_policy_cost"],
                   saved=stay_b1["actual_policy_cost"] - rec["actual_policy_cost"])
        rows_b.append(row)
        lines.append(f"| {shape_label(shape.name)} | {row['crossing_s']:.1f} | {row['peak_speed']:.3f} | "
                     f"{row['peak_accel']:.4f} | {row['peak_jerk']:.4f} | {row['bad_samples']} | {row['cost']:.1f} | {row['saved']:.1f} |")
    rows_c = []
    lines += ["", "## Parts B2 and C2 — closed loop, each shape at its own minimum duration × (1, 1.5, 2)", "",
              "| Field | Stay cost | Decision-rule cost | Saved | Change starts | Chosen candidate | Gain at that decision | A samples below threshold (stay / rule) |",
              "|---|---:|---:|---:|---:|---|---:|---|"]
    for part, names in (("B2", [field]), ("C2", spec["part_C"]["fields"])):
        for name in names:
            allow, stay = records[f"{part}__{name}__allow__dt{dt:g}"], records[f"{part}__{name}__stay__dt{dt:g}"]
            start, label, gain = first_change(allow)
            row = dict(part=part, field=name, stay_cost=stay["actual_policy_cost"], allow_cost=allow["actual_policy_cost"],
                       change_start_s=start, chosen=label, gain=gain,
                       bad_stay=stay["metrics"]["ego_bad_samples"], bad_allow=allow["metrics"]["ego_bad_samples"])
            rows_c.append(row)
            lines.append(f"| {field_label(spec, name)} | {row['stay_cost']:.1f} | {row['allow_cost']:.1f} | "
                         f"{row['stay_cost'] - row['allow_cost']:.1f} | {'never' if start is None else f'{start:.0f} s'} | "
                         f"{label or '—'} | {'—' if gain is None else f'{gain:.2f}'} | {row['bad_stay']} / {row['bad_allow']} |")
    lines.append("")
    (output / "tables.md").write_text("\n".join(lines))
    for name, rows in (("part-A.csv", rows_a), ("part-B1.csv", rows_b), ("parts-B2-C2.csv", rows_c)):
        with (output / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-failed", action="store_true")
    args = parser.parse_args()
    validation = json.loads((args.run / "validation.json").read_text())
    failed = validation["status"] != "passed"
    if failed and not args.allow_failed:
        raise ValueError("refusing to plot failed validation (use --allow-failed for stamped diagnostics)")
    if args.output.exists():
        raise FileExistsError("use a new figure directory")
    spec = json.loads((args.run / "resolved_config.json").read_text())
    stage0 = json.loads((args.run / "stage0.json").read_text())
    records = {p.name[:-len(".json.gz")]: json.load(gzip.open(p, "rt")) for p in sorted(args.run.glob("*.json.gz"))}
    verdicts = {r["name"]: all(r["checks"].values()) for r in validation["results"]}
    dt, horizon = spec["numerics"]["dt_s"], spec["numerics"]["horizon_s"]
    shapes = [make_shape(s) for s in spec["shapes"]]
    args.output.mkdir(parents=True)
    style()
    figures = {"01-part-A-decisions": decision_figure(records, spec, dt, horizon, verdicts, failed),
               "02-part-B1-shapes": shapes_figure(records, spec, dt, shapes, failed),
               "03-part-C-real-fields": field_figure(records, spec, dt, failed)}
    for name, fig in figures.items():
        for ext in ("png", "svg"):
            fig.savefig(args.output / f"{name}.{ext}", dpi=220, bbox_inches="tight")
        plt.close(fig)
    tables(records, spec, stage0, verdicts, dt, shapes, args.output)
    (args.output / "plot_source.py").write_bytes(Path(__file__).read_bytes())
    record = {"run": str(args.run.resolve()), "validation_status": validation["status"],
              "diagnostic_only": failed, "run_manifest_sha256": digest(args.run / "manifest.json"),
              "outputs": [{"path": p.name, "sha256": digest(p)} for p in sorted(args.output.iterdir())]}
    (args.output / "manifest.json").write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"figures": sorted(figures), "diagnostic_only": failed}, indent=2))


if __name__ == "__main__":
    main()
