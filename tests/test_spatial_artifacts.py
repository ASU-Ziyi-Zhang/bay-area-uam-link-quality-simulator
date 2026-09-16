"""Checks for independent Stage-2 audit and deterministic figure provenance."""
import importlib.util
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest


def module(name, monkeypatch):
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location(name, scripts / f"{name}.py")
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_independent_transverse_reconstruction_preserves_holds(monkeypatch):
    audit = module("verify_spatial_study", monkeypatch)
    moves = [dict(t_s=10., duration_s=50., source=[0., 300.], target=[100., 200.]),
             dict(t_s=100., duration_s=50., source=[100., 200.], target=[0., 300.])]
    times = np.array([0., 10., 35., 60., 80., 100., 125., 150., 200.])
    p, v, a = audit.transverse(times, moves, 300.)
    np.testing.assert_allclose(p, [[0, 300], [0, 300], [50, 250], [100, 200],
                                  [100, 200], [100, 200], [50, 250], [0, 300], [0, 300]])
    np.testing.assert_allclose(v[[0, 1, 3, 4, 5, 7, 8]], 0., atol=1e-12)
    np.testing.assert_allclose(a, 0., atol=1e-12)  # endpoints and midpoints
    np.testing.assert_allclose(v[2], [3.75, -3.75])
    np.testing.assert_allclose(v[6], [-3.75, 3.75])


def test_spatial_export_cannot_overwrite_existing_revision(monkeypatch, tmp_path):
    plotter = module("plot_spatial_study", monkeypatch)
    with pytest.raises(FileExistsError, match="revision"):
        plotter.plot(tmp_path / "missing", tmp_path, tmp_path / "missing-audit")


def test_incomplete_run_cannot_pass_audit(monkeypatch, tmp_path):
    audit = module("verify_spatial_study", monkeypatch)
    (tmp_path / "manifest.json").write_text(json.dumps({"status": "running"}))
    with pytest.raises(ValueError, match="incomplete"):
        audit.audit(tmp_path)


def test_spatial_export_uses_audited_archive_without_mutation(monkeypatch, tmp_path):
    root = Path(__file__).resolve().parents[1]
    run = root / "research/dynamic-transitions/runs/R0010"
    if not (run / "manifest.json").exists() or json.loads((run / "manifest.json").read_text())["status"] != "completed":
        pytest.skip("requires the locally archived completed research run")
    audit = module("verify_spatial_study", monkeypatch)
    plotter = module("plot_spatial_study", monkeypatch)
    before = {str(p.relative_to(run)): plotter.sha(p) for p in run.rglob("*") if p.is_file()}
    result = audit.audit(run)
    audit_path = tmp_path / "audit.json"
    audit_path.write_text(json.dumps(result))
    output = tmp_path / "figures"
    plotter.plot(run, output, audit_path)
    manifest = plotter.read(output / "manifest.json")
    assert len(manifest["section_indices"]) == 3
    assert manifest["joint_lateral_paths_identical"]
    assert manifest["spatial_detail"]["case"] == "vertical"
    assert manifest["spatial_detail"]["target_offset_height_m"] == [0., 100.]
    assert manifest["spatial_detail"]["relative_metric_scale_xyz"][2] > 1
    assert len(list(output.glob("*.png"))) == len(list(output.glob("*.svg"))) == 5
    assert "00-geographic-transition-overview" in manifest["captions"]
    assert manifest["audit_sha256"] == plotter.sha(audit_path)
    for entry in manifest["outputs"]:
        assert plotter.sha(output / entry["path"]) == entry["sha256"]
    after = {str(p.relative_to(run)): plotter.sha(p) for p in run.rglob("*") if p.is_file()}
    assert before == after


def test_toyota_briefing_exports_are_complete(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    briefing = root / "research/dynamic-transitions/figures/R0010/toyota-briefing/revision-09"
    if not (briefing / "manifest.json").exists():
        pytest.skip("requires the local Toyota briefing export")
    manifest = json.loads((briefing / "manifest.json").read_text())
    assert manifest["revision"] == "09"
    expected = {"01-decision-location-and-candidate-plane", "02-candidate-motion-and-corridor-outcome"}
    assert set(manifest["figures"]) == expected
    assert manifest["accepted_targets"]["lateral"] == [100.0, 300.0]
    assert manifest["accepted_targets"]["vertical"] == [0.0, 100.0]
    assert len(list(briefing.glob("*.png"))) == len(list(briefing.glob("*.svg"))) == 2
    for entry in manifest["outputs"]:
        assert hashlib.sha256((briefing / entry["path"]).read_bytes()).hexdigest() == entry["sha256"]
