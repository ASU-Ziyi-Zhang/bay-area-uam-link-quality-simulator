"""Scientific checks for the all-station, equal-scale Cartesian figure."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from capacity_policy.base_stations import BaseStation, BaseStationSet
from capacity_policy.radio import RadioConfig, compute_link_state
from capacity_policy.trajectory import TrajectoryState


@pytest.fixture
def wide(monkeypatch):
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("wide_radio_figure", scripts / "plot_wide_radio_map.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rigid_frame_preserves_distances_and_roundtrips(wide):
    origin, rotation = wide.chord_frame([554000., 4161000.], [594000., 4134000.])
    points = origin + np.array([[0, 0], [300, 400], [41000, -29000], [-2000, 300]])
    local = (points - origin) @ rotation
    np.testing.assert_allclose(origin + local @ rotation.T, points, atol=1e-8, rtol=0)
    np.testing.assert_allclose(np.linalg.norm(np.diff(local, axis=0), axis=1),
                               np.linalg.norm(np.diff(points, axis=0), axis=1), rtol=1e-14)
    assert np.linalg.det(rotation) == pytest.approx(1.)


def test_beyond_end_site_is_not_clamped_to_endpoint(wide):
    origin, rotation = wide.chord_frame([100, 200], [1100, 200])
    assert ((np.array([1400, 500]) - origin) @ rotation).tolist() == [1300, 300]


def test_bounds_include_distant_stations_and_leave_padding(wide):
    points = np.array([[0, 0], [50000, 0], [13000, -4300], [51000, 3900]])
    low, high = wide.display_bounds(points)
    assert np.all(points >= low + 1200)
    assert np.all(points <= high - 1200)


@pytest.mark.parametrize("served_set_size", [1, 3, None])
def test_display_radio_matches_simulator_kernel(wide, served_set_size):
    rng = np.random.default_rng(74)
    stations = BaseStationSet(tuple(BaseStation(f"BS{i:02d}", i * 1000, (-1)**i * 300,
                                  None if i == 0 else 10 + 5 * i) for i in range(6)), "EPSG:26910")
    cfg = RadioConfig(served_set_size=served_set_size, assumed_bs_height_m=18.)
    spec = {"radio": vars(cfg), "base_stations": [vars(s) for s in stations.stations]}
    xyz = rng.uniform([-500, -5000, 100], [10000, 5000, 600], size=(100, 3))
    state = TrajectoryState(np.arange(100), xyz[None, :, :], np.zeros((1, 100)), np.ones((1, 100), bool))
    expected = compute_link_state(state, stations, cfg)
    actual = wide.radio_at(xyz, spec, chunk_size=11)
    np.testing.assert_allclose(actual["sinr_db"], expected["sinr_db"][0], atol=1e-10, rtol=0)
    np.testing.assert_array_equal(actual["serving_bs"], expected["serving_bs"][0])
    delta = xyz[:, None, :2] - stations.positions(18.)[:, :2]
    expected_far = np.any((np.linalg.norm(delta, axis=-1) > 4000) & expected["served_mask"][0], axis=-1)
    np.testing.assert_array_equal(actual["any_selected_link_beyond_4km"], expected_far)


def test_archived_r0009_positions_and_radio_remain_unchanged(wide):
    run = Path(__file__).resolve().parents[1] / "research/dynamic-transitions/runs/R0009"
    if not run.exists():
        pytest.skip("raw research runs are local artifacts")
    data = wide.load_inputs(run)
    assert len(data["spec"]["base_stations"]) == 12
    assert data["position_error_m"] < 1e-8
    assert data["field_error_db"] < 1e-8
    assert data["observation_error_db"] < 1e-8
    assert data["d"][0] == 0 and data["d"][-1] == 100
    assert max(data["case"]["offsets_m"]) == 900


def test_existing_figure_cannot_be_overwritten(wide, tmp_path):
    with pytest.raises(FileExistsError, match="revision"):
        wide.plot(tmp_path / "absent-run", tmp_path)


@pytest.mark.parametrize("focus_event", [None, 1])
def test_export_all_sites_equal_scale_and_hashes_without_mutating_run(wide, tmp_path, focus_event):
    run = Path(__file__).resolve().parents[1] / "research/dynamic-transitions/runs/R0009"
    if not run.exists():
        pytest.skip("raw research runs are local artifacts")
    before = {str(p.relative_to(run)): wide.sha(p) for p in run.rglob("*") if p.is_file()}
    output = tmp_path / "figure"
    wide.plot(run, output, grid_m=100., focus_event=focus_event)
    manifest = wide.read(output / "manifest.json")
    assert manifest["station_count"] == 12
    assert manifest["all_stations_in_frame"]
    assert manifest["rendered_x_to_y_metric_scale"] == pytest.approx(1., abs=1e-10)
    assert manifest["annotation_overlaps"] == manifest["clipped_annotations"] == []
    if focus_event is not None:
        focus = manifest["focus"]
        assert focus["rendered_x_to_y_metric_scale"] == pytest.approx(1., abs=1e-10)
        assert focus["linear_magnification"] > 6
        assert focus["annotation_overlaps"] == focus["clipped_annotations"] == []
        assert (output / "focus-field.npz").is_file()
    for entry in manifest["outputs"]:
        assert wide.sha(output / entry["path"]) == entry["sha256"]
    after = {str(p.relative_to(run)): wide.sha(p) for p in run.rglob("*") if p.is_file()}
    assert before == after


def test_focus_boundaries_and_dimension_are_exact_archived_values(wide):
    run = Path(__file__).resolve().parents[1] / "research/dynamic-transitions/runs/R0009"
    if not run.exists():
        pytest.skip("raw research runs are local artifacts")
    data = wide.load_inputs(run)
    f = data["frame"]
    origin, rotation = wide.chord_frame(f(0.)[0], f(data["field"]["q_m"][-1])[0])
    focus = wide.focus_geometry(data, 1, origin, rotation)
    assert focus["start_s"] == 460.
    assert focus["end_s"] == pytest.approx(568.8691337650831)
    assert focus["start_q_m"] == data["events"][0]["q_m"]
    assert focus["displacement_m"] == pytest.approx(100., abs=1e-6)
    dimension = np.array(focus["end_xy_m"]) - focus["source_at_end_xy_m"]
    assert dimension @ focus["end_tangent"] == pytest.approx(0., abs=1e-6)
    for key in ["start_xy_m", "end_xy_m", "source_at_end_xy_m"]:
        assert np.all(np.array(focus[key]) >= focus["low_m"])
        assert np.all(np.array(focus[key]) <= focus["high_m"])


@pytest.mark.parametrize("event_number", [0, 2])
def test_focus_rejects_nonexistent_maneuver(wide, event_number):
    with pytest.raises(ValueError, match="accepted maneuver"):
        wide.focus_geometry({"events": [{}]}, event_number, np.zeros(2), np.eye(2))
