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
    single_html = (ROOT / "dashboard" / "index.html").read_text(encoding="utf-8")
    assert "Green for coordinated" not in html
    assert "Coordinated" in html and "Reactive" in html and "Fallback" in html
    assert "--coordinated: #238b57" in css
    assert "--reactive: #e3b735" in css
    assert "--fallback: #d65353" in css
    assert "Centered five-aircraft group" in html
    assert "uam-traffic-marker--group" in css
    assert "Multi-UAM policy simulator" in single_html


def test_airport_traffic_bundle_matches_group_run_summary():
    bundle = read_bundle(ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js")
    summary = json.loads(
        (ROOT / "runs" / "airport-to-airport-group-policy-v1" / "summary.json").read_text()
    )
    assert bundle["summary"]["scenario_id"] == "airport_to_airport"
    assert bundle["summary"]["policy"]["shares"] == summary["policy"]["shares"]
    assert bundle["summary"]["capacity"]["q_mix_rho_uam_h"] == summary["capacity"]["q_mix_rho_uam_h"]
    assert len(bundle["frames"]) == summary["capacity"]["snapshot_count"]
    assert min(frame["active_count"] for frame in bundle["frames"]) >= 30


def test_full_traffic_bundle_reproduces_slide_policy_capacity_baseline():
    bundle = read_bundle(ROOT / "dashboard" / "data" / "sf_sj_full_traffic.js")
    shares = bundle["summary"]["policy"]["shares"]
    assert np.isclose(shares["C"], 0.3657, atol=0.0001)
    assert np.isclose(shares["R"], 0.3582, atol=0.0001)
    assert np.isclose(shares["F"], 0.2761, atol=0.0001)
    assert np.isclose(
        bundle["summary"]["capacity"]["q_mix_rho_uam_h"], 72.47, atol=0.01
    )


def test_each_frame_policy_count_matches_capacity_counts():
    bundle = read_bundle(ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js")
    for frame in bundle["frames"]:
        counts = {key: list(frame["policies"].values()).count(key) for key in "CRF"}
        assert counts["C"] == frame["n_C"]
        assert counts["R"] == frame["n_R"]
        assert counts["F"] == frame["n_F"]
        assert len(frame["policies"]) == frame["classified_count"]
