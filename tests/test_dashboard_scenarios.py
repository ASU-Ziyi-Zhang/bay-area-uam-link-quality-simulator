"""Verify that both committed scenario bundles are selectable in the dashboard."""

from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_bundle(path: Path) -> dict:
    raw = path.read_text(encoding="utf-8")
    return json.loads(raw[raw.index("=") + 1 : raw.rstrip().rindex(";")])


def test_dashboard_exposes_both_scenario_options() -> None:
    index = (ROOT / "dashboard/index.html").read_text(encoding="utf-8")
    assert 'value="sf_sj_full"' in index
    assert 'value="airport_to_airport"' in index
    assert 'data/sf_sj_full.js' in index
    assert 'data/airport_to_airport.js' in index


def test_full_dashboard_bundle_matches_full_scenario() -> None:
    bundle = read_bundle(ROOT / "dashboard/data/sf_sj_full.js")
    assert bundle["summary"]["scenario_id"] == "sf_sj_full"
    assert bundle["summary"]["site_count"] == 18
    assert len(bundle["stations"]) == 18
    assert abs(bundle["summary"]["corridor_length_km"] - 75.423116) < 1e-6


def test_airport_dashboard_bundle_matches_airport_scenario() -> None:
    bundle = read_bundle(ROOT / "dashboard/data/airport_to_airport.js")
    assert bundle["summary"]["scenario_id"] == "airport_to_airport"
    assert bundle["summary"]["site_count"] == 12
    assert [site["id"] for site in bundle["stations"]] == [f"BS{i:02d}" for i in range(5, 17)]
    assert abs(bundle["summary"]["corridor_length_km"] - 49.542589) < 1e-6
    assert bundle["summary"]["display"]["origin_label"].startswith("Millbrae Caltrain")
    assert bundle["summary"]["display"]["destination_label"].startswith("Santa Clara Caltrain")
    assert bundle["summary"]["display"]["site_layout_url"].endswith(
        "airport-to-airport-sites-layout.svg"
    )

    first_route, last_route = bundle["route"][0], bundle["route"][-1]
    first_trace, last_trace = bundle["trace"][0], bundle["trace"][-1]
    assert math.hypot(first_route[0] - first_trace["lon"], first_route[1] - first_trace["lat"]) < 1e-10
    assert math.hypot(last_route[0] - last_trace["lon"], last_route[1] - last_trace["lat"]) < 1e-10
    assert max(row["s_km"] for row in bundle["trace"]) <= 49.542589 + 1e-6


def test_airport_layout_is_scenario_specific() -> None:
    evidence_layout = ROOT / "evidence/figures/airport_to_airport_sites.svg"
    dashboard_layout = ROOT / "dashboard/assets/site-layout/airport-to-airport-sites-layout.svg"
    svg = evidence_layout.read_text(encoding="utf-8")
    assert evidence_layout.read_bytes() == dashboard_layout.read_bytes()
    for site_id in ("BS05", "BS06", "BS16"):
        assert site_id in svg
    for excluded_id in ("BS01", "BS02", "BS17", "BS18"):
        assert excluded_id not in svg


def test_3d_endpoint_labels_are_not_hard_coded_to_the_full_corridor() -> None:
    app = (ROOT / "dashboard/app.js").read_text(encoding="utf-8")
    assert '"San Francisco", "Origin vertiport"' not in app
    assert '"San Jose Diridon", "Destination vertiport"' not in app
    assert "display.origin_label" in app
    assert "display.destination_label" in app
    assert "endpoint_mismatch_m" in app
