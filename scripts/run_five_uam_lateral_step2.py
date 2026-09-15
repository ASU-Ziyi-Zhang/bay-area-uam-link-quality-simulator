"""Formal runner for the restored step-2 five-aircraft straight-lane study.

Stage 0 measures the numerical error bands and tightens the trajectory
thresholds; Stage 1 runs parts A, B and C under those frozen values. Every
record is archived with a source snapshot, and validation is the independent
verifier in ``scripts/verify_five_uam_lateral_step2.py``. ``--quick`` runs a
reduced matrix for debugging only and must not be registered as a result.
"""
from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import math
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
import subprocess
import sys
import time
import traceback
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from uam_simulator.five_uam_lateral import (  # noqa: E402
    QUINTIC_SHAPE, LateralProfile, Scenario, WeakZone, admissibility_check, beta_shape,
    bezier3_shape, initial_state, policy_cost, rollout, run_closed_loop, three_clothoid_shape,
)
from uam_simulator.geographic_traffic import GeographicTrafficConfig  # noqa: E402
from uam_simulator.motion_envelope import AccelerationEnvelope  # noqa: E402
from uam_simulator.research_provenance import execution_metadata  # noqa: E402
from uam_simulator.safety import SeparationVolume  # noqa: E402
from verify_five_uam_lateral_step2 import crossing_and_exposure, derive_stage0, verify  # noqa: E402

TRACE_KEYS = ("q_m", "d_m", "v_mps", "a_mps2", "policy", "lane", "controller")
LATERAL_KEYS = ("offset_m", "speed_mps", "accel_mps2", "jerk_mps3")


def serial(value):
    if is_dataclass(value):
        return serial(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [serial(v) for v in value]
    if isinstance(value, np.ndarray):
        return serial(value.tolist())
    if isinstance(value, np.generic):
        return serial(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def make_shape(spec):
    if spec["kind"] == "quintic":
        return QUINTIC_SHAPE
    if spec["kind"] == "beta":
        return beta_shape(spec["p"], spec["q"])
    if spec["kind"] == "clothoid3":
        return three_clothoid_shape(spec["fraction"])
    if spec["kind"] == "bezier3":
        return bezier3_shape()
    raise ValueError(f"unknown shape {spec}")


def make_field(spec, threshold_db):
    if spec["kind"] == "sharp":
        def radio(xyz):
            bad = ((xyz[:, 1] < spec["lane0_below_y_m"]) & (xyz[:, 0] >= spec["q_from_m"])
                   & (xyz[:, 0] < spec["q_to_m"]))
            return {"sinr_db": np.where(bad, spec["bad_db"], spec["good_db"])}
        return radio
    if spec["kind"] == "real":
        depth, margin = spec["depth_db"], spec["outside_margin_db"]
        radius = (spec["bad_length_m"] / 2) / math.sqrt(depth / (depth + margin))

        def radio(xyz):
            well = np.minimum(1.0, ((xyz[:, 0] - spec["centre_q_m"]) / radius) ** 2)
            return {"sinr_db": threshold_db - depth + (depth + margin) * well
                    + spec["gradient_db_per_100m"] * xyz[:, 1] / 100.0}
        return radio
    raise ValueError(f"unknown field {spec}")


def make_scenario(spec):
    spec = dict(spec)
    zones = tuple(WeakZone(int(z[0]), float(z[1]), float(z[2]), z[3]) for z in spec.pop("zones", []))
    pairs = spec.pop("established_pairs", None)
    if pairs is not None:
        pairs = tuple(tuple(p) for p in pairs)
    return Scenario(zones=zones, established_pairs=pairs, **spec)


def concat(arrays):
    arrays = [np.asarray(a) for a in arrays]
    return np.concatenate([a[:-1] for a in arrays[:-1]] + [arrays[-1]])


def pack(pieces, *, part, case, variant, dt, scenario_spec, status, decisions, state, gate,
         safety_events=(), extra=None):
    record = {"part": part, "case_id": case, "variant": variant, "dt_s": dt, "status": status,
              "gate": gate, "scenario": serial(scenario_spec), "decisions": serial(decisions or []),
              "safety_events": serial(list(safety_events))}
    if not pieces:
        record.update(times_s=[], trace={}, lateral={k: [] for k in LATERAL_KEYS}, observations=[],
                      profile=None, actual_policy_cost=0.0, admission=[], policy_deficit_reports=0)
        return record
    roles = list(pieces[0]["roles"])
    record["times_s"] = serial(concat([p["times_s"] for p in pieces]))
    record["trace"] = {r: {k: serial(concat([p["trace"][r][k] for p in pieces])) for k in TRACE_KEYS}
                       for r in roles}
    record["lateral"] = {k: serial(concat([p["lateral"][k] for p in pieces])) for k in LATERAL_KEYS}
    record["observations"] = serial(state.observation_rows)
    record["profile"] = serial(state.profile) if state.profile is not None else None
    record["actual_policy_cost"] = float(sum(policy_cost(p)["total"] for p in pieces))
    checks = [p.get("admission") for p in pieces]
    record["admission"] = [None if c is None else {
        "admitted": bool(c.admitted), "reasons": list(c.reasons),
        "peak_utilisation": serial(c.peak_utilisation), "minimum_separation_m": serial(c.minimum_separation_m)}
        for c in checks]
    record["policy_deficit_reports"] = int(sum(
        len(c.screen.get("policy_deficits", [])) for c in checks
        if c is not None and isinstance(getattr(c, "screen", None), dict)))
    if extra:
        record.update(serial(extra))
    return record


class Study:
    def __init__(self, spec, output, log):
        p = spec["parameters"]
        self.spec, self.output, self.log = spec, output, log
        self.cfg = GeographicTrafficConfig(**p["traffic"])
        self.env = AccelerationEnvelope(**p["envelope"])
        self.vol = SeparationVolume(**p["admission_volume"])
        self.shapes = [make_shape(s) for s in spec["shapes"]]
        self.fields = {k: make_field(v, p["traffic"]["threshold_db"]) for k, v in spec["fields"].items()}
        n = spec["numerics"]
        self.H, self.P, self.dt = n["horizon_s"], n["prediction_s"], n["dt_s"]
        self.threshold_db = p["traffic"]["threshold_db"]
        self.records = {}

    def save(self, name, record):
        path = self.output / f"{name}.json.gz"
        if path.exists():
            raise FileExistsError(path)
        with gzip.open(path, "wt") as stream:
            json.dump(record, stream, allow_nan=False)
        self.records[name] = record
        message = (f"{name}: {record['status']}, cost {record['actual_policy_cost']:.3f}, "
                   f"decisions {len(record['decisions'])}")
        print(message, flush=True)
        self.log.write(message + "\n")
        self.log.flush()
        return record

    def closed(self, scenario_spec, *, dt, allow, delays, factors, gate_parts=(0.0, 0.0),
               radio=None, shapes=None, phase=0.0):
        run = run_closed_loop(make_scenario(scenario_spec), self.cfg, self.env, self.vol,
                              horizon_s=self.H, prediction_s=self.P, dt=dt,
                              duration_factors=tuple(factors), start_delays=tuple(delays),
                              delta_num=gate_parts[0], delta_pred=gate_parts[1],
                              allow_lane_change=allow, radio=radio, shapes=shapes,
                              decision_phase_s=phase)
        return run

    def pack_run(self, run, **meta):
        return pack(run["pieces"], status=run["status"], decisions=run["decisions"],
                    state=run["state"], safety_events=run.get("safety_events", ()), **meta)

    def open_loop(self, scenario_spec, profile_shape, duration, *, dt, radio):
        scenario = make_scenario(scenario_spec)
        profile = None if profile_shape is None else LateralProfile(
            0.0, duration, scenario.lane_separation_m, profile_shape)
        result = rollout(scenario, self.cfg, profile, horizon_s=self.H, dt=dt,
                         state=initial_state(scenario), envelope=self.env, radio=radio)
        result["admission"] = admissibility_check(result, scenario, self.env, self.vol)
        return result

    def metrics(self, record):
        if not record["times_s"]:
            return {}
        cross, bad = crossing_and_exposure(record, record["scenario"]["lane_separation_m"] / 2,
                                           self.threshold_db)
        t = np.asarray(record["times_s"], float)
        per_role = {r: float(np.dot([{"C": 0., "R": 1., "F": 2.}[p] for p in tr["policy"][:-1]], np.diff(t)))
                    for r, tr in record["trace"].items()}
        lat = record["lateral"]
        return {"crossing_s": cross, "ego_bad_samples": bad, "per_role_cost": per_role,
                "executed_peak_lateral_speed_mps": float(np.max(np.abs(lat["speed_mps"]))),
                "executed_peak_lateral_accel_mps2": float(np.max(np.abs(lat["accel_mps2"]))),
                "executed_peak_lateral_jerk_mps3": float(np.max(np.abs(lat["jerk_mps3"])))}

    # --- Stage 0 ---------------------------------------------------------
    def stage0(self):
        spec = self.spec
        s0 = spec["numerics"]["stage0"]
        a1 = next(c for c in spec["part_A"] if c["id"] == s0["scenario"])
        o = a1["options"]
        for dt in s0["dt_s"]:
            for allow in (True, False):
                run = self.closed(a1["scenario"], dt=dt, allow=allow, delays=o["start_delays"],
                                  factors=o["duration_factors"])
                variant = "allow" if allow else "stay"
                self.save(f"S0__A1__{variant}__dt{dt:g}", self.pack_run(
                    run, part="S0", case="A1", variant=variant, dt=dt,
                    scenario_spec=a1["scenario"], gate=0.0 if allow else None))
        for phase in s0["decision_phases_s"]:
            if phase:
                run = self.closed(a1["scenario"], dt=self.dt, allow=True, delays=o["start_delays"],
                                  factors=o["duration_factors"], phase=phase)
                self.save(f"S0__A1__phase{phase:g}__dt{self.dt:g}", self.pack_run(
                    run, part="S0", case="A1", variant=f"phase{phase:g}", dt=self.dt,
                    scenario_spec=a1["scenario"], gate=0.0))
        b = spec["part_B"]
        for shape in self.shapes:
            for dt in spec["numerics"]["refinement_dt_s"]:
                r = self.open_loop(b["scenario"], shape, b["B1"]["duration_s"], dt=dt,
                                   radio=self.fields[b["field"]])
                self.save(f"S0__shape_{shape.name}__dt{dt:g}", pack(
                    [r], part="S0", case="shape", variant=shape.name, dt=dt,
                    scenario_spec=b["scenario"], status="open_loop", decisions=[],
                    state=r["state"], gate=None))
        stage0 = derive_stage0({k: v for k, v in self.records.items() if k.startswith("S0__")}, spec)
        (self.output / "stage0.json").write_text(json.dumps(stage0, indent=2, allow_nan=False) + "\n")
        print(json.dumps({k: stage0[k] for k in ("delta_int", "delta_phase", "delta_pred", "gates")}, indent=2))
        return stage0

    # --- Stage 1 ---------------------------------------------------------
    def part_a(self, stage0, dt):
        gates = stage0["gates"]
        # Step 2 declares the numerical gate only. A variant runs only while the
        # configuration still declares it.
        available = {"revised": (stage0["delta_int"], stage0["delta_pred"]),
                     "original": (stage0["delta_int"] + stage0["delta_phase"], stage0["delta_pred"])}
        parts = {name: available[name] for name in self.spec["gate"]["variants"]}
        for case in self.spec["part_A"]:
            o = case["options"]
            for variant, gate_parts in parts.items():
                run = self.closed(case["scenario"], dt=dt, allow=True, delays=o["start_delays"],
                                  factors=o["duration_factors"], gate_parts=gate_parts)
                self.save(f"A__{case['id']}__allow_{variant}__dt{dt:g}", self.pack_run(
                    run, part="A", case=case["id"], variant=f"allow_{variant}", dt=dt,
                    scenario_spec=case["scenario"], gate=gates[variant]))
            if case["stay_control"]:
                run = self.closed(case["scenario"], dt=dt, allow=False, delays=o["start_delays"],
                                  factors=o["duration_factors"])
                self.save(f"A__{case['id']}__stay__dt{dt:g}", self.pack_run(
                    run, part="A", case=case["id"], variant="stay", dt=dt,
                    scenario_spec=case["scenario"], gate=None))

    def shape_parts(self, part, scenario_spec, field_name, cfg_b, stage0, dt, *, open_loop=True):
        radio = self.fields[field_name]
        gate = stage0["gates"][self.spec["gate"]["primary"]]
        gate_parts = (stage0["delta_int"], stage0["delta_pred"])
        if open_loop:
            for shape in [None] + self.shapes:
                r = self.open_loop(scenario_spec, shape, cfg_b["B1"]["duration_s"], dt=dt, radio=radio)
                name = "stay" if shape is None else shape.name
                rec = pack([r], part=f"{part}1", case=field_name, variant=name, dt=dt,
                           scenario_spec=scenario_spec, status="open_loop", decisions=[],
                           state=r["state"], gate=None)
                rec["metrics"] = self.metrics(rec)
                self.save(f"{part}1__{field_name}__{name}__dt{dt:g}", rec)
        b2 = cfg_b["B2"]
        for allow in (True, False):
            run = self.closed(scenario_spec, dt=dt, allow=allow, delays=b2["start_delays"],
                              factors=b2["duration_factors"], gate_parts=gate_parts, radio=radio,
                              shapes=self.shapes)
            variant = "allow" if allow else "stay"
            rec = self.pack_run(run, part=f"{part}2", case=field_name, variant=variant, dt=dt,
                                scenario_spec=scenario_spec, gate=gate if allow else None)
            rec["metrics"] = self.metrics(rec)
            self.save(f"{part}2__{field_name}__{variant}__dt{dt:g}", rec)

    def stage1(self, stage0):
        spec, dt = self.spec, self.dt
        self.part_a(stage0, dt)
        b = spec["part_B"]
        self.shape_parts("B", b["scenario"], b["field"], b, stage0, dt)
        for field in spec["part_C"]["fields"]:
            self.shape_parts("C", spec["part_C"]["scenario"], field, b, stage0, dt)
        finer = spec["numerics"]["refinement_dt_s"][1]
        for target in spec["numerics"]["refinement_cases"]:
            if target == "B2":
                self.shape_parts("B", b["scenario"], b["field"], b, stage0, finer, open_loop=False)
            else:
                case = next(c for c in spec["part_A"] if c["id"] == target)
                saved = self.spec["part_A"]
                self.spec["part_A"] = [case]
                self.part_a(stage0, finer)
                self.spec["part_A"] = saved


def quick_spec(spec):
    spec = copy.deepcopy(spec)
    spec["status"] = "QUICK DEBUG MATRIX - not a result, must not be registered"
    spec["numerics"].update(dt_s=1.0, horizon_s=100.0, refinement_dt_s=[1.0, 0.5])
    spec["numerics"]["stage0"].update(dt_s=[1.0, 0.5], decision_phases_s=[0.0, 2.0])
    spec["numerics"]["refinement_cases"] = [spec["numerics"]["stage0"]["scenario"]]
    keep = {spec["numerics"]["stage0"]["scenario"], "A3_unsafe_but_better"}
    spec["part_A"] = [c for c in spec["part_A"] if c["id"] in keep]
    spec["shapes"] = spec["shapes"][:1] + [s for s in spec["shapes"] if s.get("p") == 2 and s.get("q") == 3]
    spec["part_C"]["fields"] = spec["part_C"]["fields"][:1]
    spec["part_B"]["B2"]["duration_factors"] = [1.0]
    return spec


def run(config, output, quick):
    if output.exists():
        raise FileExistsError("run directory already exists")
    spec = json.loads(config.read_text())
    if spec.get("study") != "five_uam_lateral_step2" or spec.get("schema_version") != 1:
        raise ValueError("unsupported experiment")
    if quick:
        spec = quick_spec(spec)
    # Every default the objects fill in is written out, so the verifier reads
    # the values actually used without importing the implementation.
    p = spec["parameters"]
    spec["resolved_objects"] = serial({
        "traffic": GeographicTrafficConfig(**p["traffic"]),
        "envelope": AccelerationEnvelope(**p["envelope"]),
        "admission_volume": SeparationVolume(**p["admission_volume"])})
    sources = sorted((ROOT / "src").rglob("*.py")) + [
        Path(__file__), ROOT / "scripts/verify_five_uam_lateral_step2.py",
        ROOT / "tests/test_five_uam_lateral.py", ROOT / "pyproject.toml",
        ROOT / "research/dynamic-transitions/three-uam-lateral-protocol.md", config]
    records = [{"path": str(p.relative_to(ROOT)), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
               for p in sources]
    output.mkdir(parents=True)
    with zipfile.ZipFile(output / "source_snapshot.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(p, str(p.relative_to(ROOT)))
    arguments = [str(Path(__file__).resolve()), "--config", str(config), "--output", str(output)]
    execution = execution_metadata(ROOT, arguments + (["--quick"] if quick else []))
    execution["source_snapshot_included"] = True
    manifest = {"status": "running", "quick_debug_matrix": quick,
                "scientific_status": "draft_uncommitted_source", "execution": execution, "sources": records}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (output / "resolved_config.json").write_text(json.dumps(spec, indent=2, ensure_ascii=False) + "\n")
    freeze = subprocess.run([sys.executable, "-m", "pip", "freeze"], capture_output=True, text=True)
    (output / "environment.txt").write_text(freeze.stdout + freeze.stderr)
    started = time.time()
    try:
        with (output / "run.log").open("w") as log:
            study = Study(spec, output, log)
            stage0 = study.stage0()
            study.stage1(stage0)
        report = verify(output)
        (output / "validation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        manifest["status"] = "validated_draft" if report["status"] == "passed" else "failed_validation"
        manifest["wall_s"] = time.time() - started
        manifest["outputs"] = [{"path": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                               for p in sorted(output.iterdir()) if p.is_file() and p.name != "manifest.json"]
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps({"status": manifest["status"], "wall_s": round(manifest["wall_s"], 1),
                          "failures": report["failures"][:40]}, indent=2, ensure_ascii=False))
        return report["status"] == "passed"
    except Exception:
        manifest["status"] = "failed_execution"
        manifest["error"] = traceback.format_exc()
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    raise SystemExit(0 if run(args.config.resolve(), args.output.resolve(), args.quick) else 1)
