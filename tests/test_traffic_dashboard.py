import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def read_bundle(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    prefix = "window.UAM_TRAFFIC_DATA = "
    assert text.startswith(prefix)
    return json.loads(text[len(prefix) :].strip().removesuffix(";"))


def test_traffic_page_declares_policy_colors_and_mode_links():
    html = (ROOT / "dashboard" / "traffic.html").read_text(encoding="utf-8")
    css = (ROOT / "dashboard" / "traffic.css").read_text(encoding="utf-8")
    app = (ROOT / "dashboard" / "traffic_app.js").read_text(encoding="utf-8")
    single_html = (ROOT / "dashboard" / "index.html").read_text(encoding="utf-8")
    assert "Green for coordinated" not in html
    assert "Unclassified" not in html
    assert "SERGEI REQUEST" not in html
    assert "Coordinated" in html and "Reactive" in html and "Fallback" in html
    assert "--coordinated: #238b57" in css
    assert "--reactive: #e3b735" in css
    assert "--fallback: #d65353" in css
    assert "Current local policy group" in html
    assert "Selected-aircraft radio profile" in html
    assert "three-recenter" in html
    assert "Recenter" in html
    assert "camera follows the selected aircraft" in html
    assert "FIXED OVERVIEW" not in html
    assert "assets/models/cesium-drone/CesiumDrone.glb" in app
    assert "servingLink3d" in app and "altitudeLine3d" in app
    assert "applyFollowCamera(selected)" in app
    assert "uam-traffic-marker--group" in css
    assert "Multi-UAM policy simulator" in single_html


def test_airport_traffic_bundle_matches_group_run_summary():
    bundle = read_bundle(ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js")
    summary = json.loads(
        (ROOT / "runs" / "airport-to-airport-group-policy-v3" / "summary.json").read_text()
    )
    assert bundle["summary"]["scenario_id"] == "airport_to_airport"
    assert bundle["summary"]["policy"]["shares"] == summary["policy"]["shares"]
    assert bundle["summary"]["capacity"]["q_mix_rho_uam_h"] == summary["capacity"]["q_mix_rho_uam_h"]
    assert len(bundle["frames"]) == summary["capacity"]["snapshot_count"]
    assert bundle["frames"][0]["t"] == 0.0
    assert bundle["frames"][0]["active_count"] == 1
    assert bundle["frames"][0]["policies"] == {"UAM001": "C"}
    assert bundle["frames"][0]["groups"] == {"UAM001": ["UAM001"]}
    assert bundle["summary"]["clock"]["dt_radio_s"] == 1.0
    assert bundle["summary"]["clock"]["dt_control_s"] == 1.0
    assert bundle["frames"][1]["t"] - bundle["frames"][0]["t"] == 1.0


def test_full_traffic_bundle_reproduces_slide_policy_capacity_baseline():
    bundle = read_bundle(ROOT / "dashboard" / "data" / "sf_sj_full_traffic.js")
    shares = bundle["summary"]["trb_reference_regression"]["shares"]
    assert np.isclose(shares["C"], 0.3657, atol=0.0001)
    assert np.isclose(shares["R"], 0.3582, atol=0.0001)
    assert np.isclose(shares["F"], 0.2761, atol=0.0001)
    assert np.isclose(
        bundle["summary"]["trb_reference_regression"]["q_mix_rho_uam_h"], 72.47, atol=0.01
    )


def test_each_frame_policy_count_matches_capacity_counts():
    bundle = read_bundle(ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js")
    for frame in bundle["frames"]:
        counts = {key: list(frame["policies"].values()).count(key) for key in "CRF"}
        assert counts["C"] == frame["n_C"]
        assert counts["R"] == frame["n_R"]
        assert counts["F"] == frame["n_F"]
        assert len(frame["policies"]) == frame["classified_count"]
        assert len(frame["policies"]) == frame["active_count"]
        assert set(frame["groups"]) == set(frame["policies"])
        assert all(1 <= len(group) <= 5 for group in frame["groups"].values())
