"""Topic paths and provenance work without depending on ignored local run artifacts."""

import json
from pathlib import Path
import subprocess
import sys

from uam_simulator.research_provenance import execution_metadata

ROOT = Path(__file__).resolve().parents[1]


def test_topic_configs_resolve_shared_scenarios_and_separate_studies():
    for relative in ["geometry-response/configs/geometry_sweep.json",
                     "geometry-response/configs/fixed_streams.json",
                     "corridor-optimization/configs/offset_search.json"]:
        path = ROOT / "research" / relative
        config = json.loads(path.read_text())
        assert all((path.parent / s).is_file() for s in config["scenarios"])
        assert not ("lanes" in config and "offsets" in config)


def test_combined_cli_requires_explicit_config(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT / "scripts/run_baseline_studies.py"),
        "--study", "all", "--output", str(tmp_path / "unused")], capture_output=True, text=True)
    assert result.returncode == 2
    assert "combined historical runs require --config" in result.stderr
    assert not (tmp_path / "unused").exists()


def test_provenance_without_git_does_not_claim_clean_source(tmp_path):
    metadata = execution_metadata(tmp_path, ["script with spaces.py", "--config", "my config.json"])
    assert metadata["git_commit"] is None
    assert metadata["git_dirty"] is None
    assert metadata["source_snapshot_included"] is False
    assert "'script with spaces.py'" in metadata["command"]


def test_migration_register_is_unique_and_retains_all_files():
    migration = json.loads((ROOT / "research/migration-20260830.json").read_text())
    records = migration["records"]
    assert len({r["new_path"] for r in records}) == len(records) == 4
    assert sum(len(r["files"]) for r in records) == 358
    for record in records:
        assert record["new_path"].startswith("research/")
        assert len({f["path"] for f in record["files"]}) == len(record["files"])
        assert "manifest.json" in {f["path"] for f in record["files"]}
