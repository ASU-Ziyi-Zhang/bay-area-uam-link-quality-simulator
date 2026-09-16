"""Read-only independent audit of a policy-motion run's hashes and saved formulas."""
from __future__ import annotations

import argparse
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import zipfile


def verify(root):
    root = Path(root).resolve()
    manifest = json.loads((root/"manifest.json").read_text())
    if manifest["status"] != "completed":
        raise ValueError("run is not completed; inspect failure records first")
    for row in manifest["outputs"]:
        if hashlib.sha256((root/row["path"]).read_bytes()).hexdigest() != row["sha256"]:
            raise ValueError(f"artifact hash mismatch: {row['path']}")
    with zipfile.ZipFile(root/"source_snapshot.zip") as archive:
        for row in manifest["code"]:
            if hashlib.sha256(archive.read(row["path"])).hexdigest() != row["sha256"]:
                raise ValueError(f"archived source hash mismatch: {row['path']}")
    counts = defaultdict(int)
    for summary in json.loads((root/"summary.json").read_text()):
        case, cfg = root/summary["case_id"], summary["resolved_parameters"]
        with gzip.open(case/"observations.json.gz","rt") as stream:
            observations = json.load(stream)
        by_time = defaultdict(dict)
        histories = defaultdict(list)
        for row in observations:
            rows = by_time[row["t_s"]]
            if row["aircraft_id"] in rows:
                raise ValueError("duplicate aircraft-time observation")
            rows[row["aircraft_id"]] = row
        for t, rows in sorted(by_time.items()):
            for aid, row in rows.items():
                if row["phase"] != "CORRIDOR":
                    if row["exposure"] is not None:
                        raise ValueError("connector mixed into corridor exposure")
                    continue
                members = row["members"]
                if len(members) != row["group_size"] or aid not in members or len(members) != len(set(members)):
                    raise ValueError("invalid group membership")
                if not 1 <= len(members) <= 2*cfg["neighbors"]+1:
                    raise ValueError("group-size bounds")
                if not all(rows[j]["cell"] == row["cell"] for j in members):
                    raise ValueError("cross-flow policy membership")
                fraction = sum(rows[j]["sinr_db"] < cfg["threshold_db"] for j in members)/len(members)
                if abs(fraction-row["group_bad_fraction"]) > 1e-12:
                    raise ValueError("group fraction mismatch")
                histories[aid] = [(u,e) for u,e in histories[aid] if u >= t-cfg["window_s"]-1e-9]+[(t,fraction)]
                exposure = sum(e for _,e in histories[aid])/len(histories[aid])
                if abs(exposure-row["exposure"]) > 1e-12 or len(histories[aid]) != row["history_count"]:
                    raise ValueError("available-history exposure mismatch")
                policy = "C" if exposure <= cfg["exposure_c"]+1e-12 else "R" if exposure <= cfg["exposure_r"]+1e-12 else "F"
                if policy != row["policy"]:
                    raise ValueError("policy mismatch")
                counts["policy_records"] += 1
        with gzip.open(case/"trace.json.gz","rt") as stream:
            trace = json.load(stream)
        policy_counts = {p:sum(r["policy"] == p and r["phase"] == "CORRIDOR" for r in observations) for p in "CRF"}
        if policy_counts != summary["policy_counts"]:
            raise ValueError("summary policy counts mismatch")
        for row in trace:
            if row["phase"] != "CORRIDOR":
                continue
            v = row["velocity_mps"][0]
            tau = cfg[f"tau_{row['policy'].lower()}_s"]
            desired = cfg["d0_m"]+tau*v+cfg["buffer_s2_per_m"]*v*v
            if abs(desired-row["desired_gap_m"]) > 1e-8:
                raise ValueError("target spacing mismatch")
            raw = min([cfg["k_speed"]*(cfg["cruise_mps"]-v)]+[
                cfg["k_gap"]*(g-desired)+cfg["k_relative"]*r for _,g,r in row["leaders"]])
            if abs(raw-row["raw_acceleration"]) > 1e-8:
                raise ValueError("raw controller mismatch")
            cmd = 0. if summary["mode"] == "fixed_cruise" else max(
                -cfg["decel_limit_mps2"],min(cfg["accel_limit_mps2"],raw))
            if v <= 1e-10 and cmd < 0 or v >= cfg["cruise_mps"]-1e-10 and cmd > 0:
                cmd = 0.
            if abs(cmd-row["command_acceleration"]) > 1e-8:
                raise ValueError("command mismatch")
            if not -1e-8 <= v <= cfg["cruise_mps"]+1e-8:
                raise ValueError("speed bound")
            counts["control_records"] += 1
        if summary["aircraft_requested_or_initial"] != summary["unreleased"]+summary["completed_missions"]+summary["airborne_at_end"]:
            raise ValueError("mission count not conserved")
        counts["cases"] += 1
    return {"status":"passed", "run":str(root), **counts,
            "artifact_hashes":len(manifest["outputs"]), "archived_source_hashes":len(manifest["code"]),
            "note":"Numerical/formula and provenance audit; not an operational-safety or capacity certification."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    print(json.dumps(verify(parser.parse_args().run),indent=2))
