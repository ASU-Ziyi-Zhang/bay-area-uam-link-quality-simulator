"""Presentation checks: BS locations are not fabricated or clipped into a band."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def figure_module(monkeypatch):
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("radio_figure", scripts / "plot_lateral_radio_map.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def straight_frame(q):
    q = np.asarray(q)
    return (np.stack([q, np.zeros_like(q)], axis=-1), np.array([1., 0.]),
            np.array([0., 1.]), np.ones_like(q), np.zeros_like(q))


@pytest.mark.parametrize("offset", [-800., -100., 100., 800.])
def test_projection_preserves_signed_side_and_does_not_clip(figure_module, offset):
    projection = figure_module.closest_reference_point(straight_frame, [437., offset], 1000.)
    assert projection["q_m"] == pytest.approx(437., abs=1e-4)
    assert projection["offset_m"] == pytest.approx(offset)
    assert projection["horizontal_distance_m"] == pytest.approx(abs(offset), abs=1e-6)
    assert projection["interior"]


def test_beyond_end_station_retains_longitudinal_residual(figure_module):
    projection = figure_module.closest_reference_point(straight_frame, [1200., 100.], 1000.)
    assert projection["q_m"] == 1000.
    assert projection["offset_m"] == 100.
    assert projection["longitudinal_residual_m"] == 200.
    assert not projection["interior"]


def test_curved_projection_finds_nearest_normal_location(figure_module):
    radius = 1000.
    def frame(q):
        angle = np.asarray(q) / radius
        center = radius * np.stack([np.sin(angle), 1 - np.cos(angle)], axis=-1)
        tangent = np.stack([np.cos(angle), np.sin(angle)], axis=-1)
        normal = np.stack([-np.sin(angle), np.cos(angle)], axis=-1)
        return center, tangent, normal, np.ones_like(angle), np.full_like(angle, 1 / radius)
    center, _, normal, _, _ = frame(1234.)
    projected = figure_module.closest_reference_point(frame, center + 200. * normal, 2500.)
    assert projected["q_m"] == pytest.approx(1234., abs=1e-4)
    assert projected["offset_m"] == pytest.approx(200., abs=1e-6)


def test_archived_figure_keeps_all_stations_and_exact_trajectory(figure_module):
    root = Path(__file__).resolve().parents[1]
    run = root / "research/dynamic-transitions/runs/R0008"
    if not run.exists():
        pytest.skip("archived raw runs are local research artifacts")
    spec, _, field, events, q, d, stations, residual = figure_module.load_inputs(run)
    assert len(stations) == len(spec["base_stations"]) == 12
    assert [s["site_id"] for s in stations if s["shown_in_sinr_panel"]] == ["BS08"]
    assert residual < 1e-6
    assert len(events) == 4
    assert q[0] == 0 and q[-1] == field["q_m"][-1]
    assert d[0] == 0 and d[-1] == 400
    for original, projection in zip(spec["base_stations"], stations):
        assert original["site_id"] == projection["site_id"]
        assert (original["x_m"], original["y_m"]) == (projection["x_m"], projection["y_m"])


def test_published_figure_directory_cannot_be_overwritten(figure_module, tmp_path):
    with pytest.raises(FileExistsError, match="revision"):
        figure_module.plot(tmp_path / "nonexistent-run", tmp_path)
