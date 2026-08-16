"""Verify the frozen reference artifacts and their recorded checksums."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "reproducibility" / "manifest.json"
REFERENCE_FILES = (
    "configs/scenario.json",
    "configs/simulator.json",
    "data/corridor.geojson",
    "data/base_stations.csv",
    "results/link_quality/profile.csv",
    "results/link_quality/sites.csv",
    "results/link_quality/transitions.csv",
    "results/link_quality/qa.json",
    "results/simulator/uam_state_trace.csv",
    "results/simulator/motion_state_trace.csv",
    "results/simulator/radio_measurements.csv",
    "results/simulator/control_ticks.csv",
    "results/simulator/summary.json",
    "dashboard/data/dashboard_data.js",
)
TEXT_SUFFIXES = {".csv", ".geojson", ".js", ".json"}


def sha256(path: Path) -> str:
    """Hash logical text content consistently across Windows and Linux."""
    digest = hashlib.sha256()
    content = path.read_bytes()
    if path.suffix.lower() in TEXT_SUFFIXES:
        content = content.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    digest.update(content)
    return digest.hexdigest()


def build_manifest() -> dict[str, object]:
    link_qa = json.loads((ROOT / "results/link_quality/qa.json").read_text(encoding="utf-8"))
    sim_summary = json.loads((ROOT / "results/simulator/summary.json").read_text(encoding="utf-8"))
    return {
        "schema_version": 1,
        "evidence_grade": "rerunnable deterministic planning-model baseline",
        "files": [
            {"path": relative, "sha256": sha256(ROOT / relative)}
            for relative in REFERENCE_FILES
        ],
        "expected": {
            "site_count": 18,
            "corridor_length_km": 75.423116,
            "link_quality_positions": 2001,
            "association_transitions": 13,
            "simulator_handoffs": 13,
            "simulator_radio_rows": 1510,
            "simulator_measurement_rows": 27180,
            "link_quality_status": link_qa["status"],
            "simulator_status": sim_summary["status"],
        },
    }


def verify(manifest: dict[str, object]) -> None:
    failures: list[str] = []
    for record in manifest["files"]:
        path = ROOT / record["path"]
        if not path.exists():
            failures.append(f"missing: {record['path']}")
        elif sha256(path) != record["sha256"]:
            failures.append(f"checksum mismatch: {record['path']}")

    link_qa = json.loads((ROOT / "results/link_quality/qa.json").read_text(encoding="utf-8"))
    sim = json.loads((ROOT / "results/simulator/summary.json").read_text(encoding="utf-8"))
    expected = manifest["expected"]
    observed = {
        "site_count": link_qa["site_count"],
        "corridor_length_km": link_qa["corridor_length_km"],
        "link_quality_positions": link_qa["n_longitudinal_positions"],
        "association_transitions": link_qa["association_transition_count"],
        "simulator_handoffs": sim["samples"]["handoff_count"],
        "simulator_radio_rows": sim["samples"]["radio_state_rows"],
        "simulator_measurement_rows": sim["samples"]["radio_measurement_rows"],
        "link_quality_status": link_qa["status"],
        "simulator_status": sim["status"],
    }
    for key, value in expected.items():
        actual = observed[key]
        if isinstance(value, float):
            if abs(actual - value) > 1e-6:
                failures.append(f"metric mismatch: {key}: {actual} != {value}")
        elif actual != value:
            failures.append(f"metric mismatch: {key}: {actual} != {value}")
    if failures:
        raise SystemExit("Reference verification failed:\n- " + "\n- ".join(failures))
    print("Reference verification passed: 14 files and 9 declared metrics.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="Create the manifest from current references.")
    args = parser.parse_args()
    if args.write:
        MANIFEST.parent.mkdir(parents=True, exist_ok=True)
        MANIFEST.write_text(json.dumps(build_manifest(), indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {MANIFEST}")
    verify(json.loads(MANIFEST.read_text(encoding="utf-8")))


if __name__ == "__main__":
    main()
