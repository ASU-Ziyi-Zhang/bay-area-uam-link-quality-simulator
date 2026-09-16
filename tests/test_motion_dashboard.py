"""Checks for the multi-UAM lane-change dashboard page and its precomputed bundle."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "dashboard" / "data" / "airport_to_airport_motion.js"


def read_bundle() -> dict:
    text = BUNDLE.read_text(encoding="utf-8")
    prefix = "window.UAM_MOTION_DATA = "
    assert text.startswith(prefix)
    return json.loads(text[len(prefix):].strip().removesuffix(";"))


def test_lane_change_page_links_and_controls():
    html = (ROOT / "dashboard" / "motion.html").read_text(encoding="utf-8")
    app = (ROOT / "dashboard" / "motion_app.js").read_text(encoding="utf-8")
    for element_id in ("case-select", "motion-map", "motion-3d", "time-slider", "capacity-chart", "grid-chart",
                       "aircraft-chart", "trajectory-chart", "settings-list", "selected-controller", "selected-move",
                       "input-theta", "input-window", "input-persistence", "input-c-tolerance", "input-r-tolerance",
                       "input-group-size", "input-group-mode", "input-reliability", "run-experiment", "reset-experiment"):
        assert f'id="{element_id}"' in html
    assert "data/airport_to_airport_motion.js" in html
    assert "traffic_engine.js" in html
    assert 'href="traffic.html' in html and 'href="index.html' in html
    assert "Lane Change" in html and "Multi-UAM lane change" in html
    assert "window.UAM_MOTION_DATA" in app and "UAM_MOTION_QA" in app


def test_motion_bundle_comes_from_a_validated_run_with_calibrated_settings():
    summary = read_bundle()["summary"]
    assert summary["validation"] in ("pass", "passed")
    p = summary["parameters"]
    assert (p["threshold_db"], p["radio_s"], p["policy_s"], p["window_s"], p["persistence_k"]) == (-2.0, 2.0, 5.0, 90.0, 3)
    assert p["group_mode"] == "neighbourhood"
    assert p["d0_m"] == 152.4
    assert {k: round(v, 1) for k, v in summary["spacing_m"].items()} == {"C": 1319.9, "R": 2069.9, "F": 3569.9}
    assert len(summary["grid"]) == 9
    assert summary["row_fields"][0] == "aircraft" and len(summary["row_fields"]) == 14
    assert "q_m" in summary["row_fields"]


def test_every_frame_matches_the_capacity_trace():
    bundle = read_bundle()
    fields = {name: i for i, name in enumerate(bundle["summary"]["row_fields"])}
    assert set(bundle["cases"]) == {"fixed_centerline", "spatial_grid"}
    for case_id, case in bundle["cases"].items():
        capacity = {row[0]: row for row in case["capacity"]}
        assert len(case["frames"]) == len(case["capacity"])
        for frame in case["frames"]:
            row = capacity[frame["t"]]
            counts = [sum(1 for r in frame["rows"] if r[fields["policy"]] == code) for code in range(3)]
            assert counts == row[2:5], (case_id, frame["t"])
            ids = [r[fields["aircraft"]] for r in frame["rows"]]
            assert len(ids) == len(set(ids))
            assert all(0 <= r[fields["cell"]] < 9 for r in frame["rows"])
        assert len(case["aircraft"]) == case["stats"]["scheduled"] == case["stats"]["completed"]
        assert case["stats"]["sampled_nmac"] is False
    stay, move = bundle["cases"]["fixed_centerline"], bundle["cases"]["spatial_grid"]
    assert stay["changes"] == [] and {r[fields["cell"]] for f in stay["frames"] for r in f["rows"]} == {4}
    assert len(move["changes"]) == move["stats"]["completed_lane_changes"] > 0


def test_motion_bundle_is_compact():
    assert BUNDLE.stat().st_size < 4_000_000
