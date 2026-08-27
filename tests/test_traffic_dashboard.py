import json
from pathlib import Path
import shutil
import subprocess

import numpy as np

from uam_simulator.group_runner import run_group_simulation


ROOT = Path(__file__).resolve().parents[1]


def read_bundle(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    prefix = "window.UAM_TRAFFIC_DATA = "
    assert text.startswith(prefix)
    return json.loads(text[len(prefix) :].strip().removesuffix(";"))


def expand_frame(bundle: dict, frame: dict) -> tuple[list[str], dict[str, str], dict[str, list[str]]]:
    active_ids = [
        entrant["id"]
        for entrant in sorted(bundle["entrants"], key=lambda row: row["index"])
        if entrant["entry"] <= frame["t"] + 1e-9
        and entrant["exit"] >= frame["t"] - 1e-9
    ]
    policies = dict(zip(active_ids, frame["policy_codes"], strict=True))
    groups = {
        uam_id: active_ids[max(0, index - 2) : min(len(active_ids), index + 3)]
        for index, uam_id in enumerate(active_ids)
    }
    return active_ids, policies, groups


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
    assert "three-free-view" in html
    assert "Free view" in html and "Follow selected" in html
    assert "camera follows the selected aircraft" in html
    assert "assumption-strip" not in html
    assert "FIXED OVERVIEW" not in html
    assert "assets/models/cesium-drone/CesiumDrone.glb" in app
    assert "servingLink3d" in app and "altitudeLine3d" in app
    assert "applyFollowCamera(selected)" in app
    assert '"three-camera-state", "three-camera-note"' in app
    assert "Loading traffic data" in html
    assert "20260827-free-view-v11" in html
    assert "traffic_engine.js" in html
    for control_id in (
        "input-altitude", "input-offset", "input-speed", "input-departure",
        "input-theta", "input-group-size", "input-window", "input-policy-interval",
        "input-c-tolerance", "input-r-tolerance", "input-reliability",
    ):
        assert f'id="{control_id}"' in html
    assert "setPlaying(false);\n    selectedUamId = uamId" in app
    assert "uam-traffic-hit" in css
    assert "entity.billboard.image = planeBillboardCanvas(row.policy)" in app
    assert "image: stationBillboardCanvas()" in app
    assert 'image: planeBillboardCanvas(row.policy)' in app
    assert 'if (selectedUamId !== previousSelection) cameraMode = "follow";' not in app
    assert "uam-traffic-marker--group" in css
    assert "Multi-UAM policy simulator" in single_html


def test_airport_traffic_bundle_matches_group_run_summary(tmp_path):
    bundle = read_bundle(ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js")
    summary = run_group_simulation(
        ROOT / "scenarios" / "airport_to_airport" / "group_simulator.json",
        tmp_path / "airport-group-run",
    )
    assert bundle["summary"]["scenario_id"] == "airport_to_airport"
    assert bundle["summary"]["policy"]["shares"] == summary["policy"]["shares"]
    assert bundle["summary"]["capacity"]["q_mix_rho_uam_h"] == summary["capacity"]["q_mix_rho_uam_h"]
    assert len(bundle["frames"]) == summary["capacity"]["snapshot_count"]
    assert bundle["frames"][0]["t"] == 0.0
    assert bundle["frames"][0]["active_count"] == 1
    active_ids, policies, groups = expand_frame(bundle, bundle["frames"][0])
    assert active_ids == ["UAM001"]
    assert policies == {"UAM001": "C"}
    assert groups == {"UAM001": ["UAM001"]}
    assert len(bundle["frames"][0]["exposure_values"]) == 1
    assert bundle["summary"]["clock"]["dt_radio_s"] == 1.0
    assert bundle["summary"]["clock"]["dt_control_s"] == 1.0
    assert "capacity" in bundle["summary"]["model_config"]
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
        active_ids, policies, groups = expand_frame(bundle, frame)
        counts = {key: list(policies.values()).count(key) for key in "CRF"}
        assert counts["C"] == frame["n_C"]
        assert counts["R"] == frame["n_R"]
        assert counts["F"] == frame["n_F"]
        assert len(policies) == frame["classified_count"]
        assert len(policies) == frame["active_count"]
        assert len(frame["exposure_values"]) == len(active_ids)
        assert set(groups) == set(policies)
        assert all(1 <= len(group) <= 5 for group in groups.values())


def test_airport_bundle_is_compact_enough_for_interactive_startup():
    path = ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js"
    bundle = read_bundle(path)
    assert path.stat().st_size < 4_000_000
    assert "policy_codes" in bundle["frames"][0]
    assert "policies" not in bundle["frames"][0]
    assert "groups" not in bundle["frames"][0]


def test_browser_engine_exactly_reproduces_airport_baseline():
    node = shutil.which("node")
    if node is None:
        return
    script = r"""
const fs = require('fs');
const vm = require('vm');
const context = { window: {}, globalThis: {} };
vm.createContext(context);
vm.runInContext(fs.readFileSync(process.argv[2], 'utf8'), context);
vm.runInContext(fs.readFileSync(process.argv[3], 'utf8'), context);
const data = context.window.UAM_TRAFFIC_DATA;
const engine = context.globalThis.UAM_TRAFFIC_ENGINE;
const parameters = {
  speedMps: data.summary.trajectory.speed_mps,
  altitudeM: data.summary.trajectory.altitude_m,
  lateralOffsetM: data.summary.trajectory.lateral_offset_m,
  departureIntervalS: data.summary.traffic.entry_interval_s,
  sinrThresholdDb: data.summary.policy.sinr_threshold_db,
  groupSize: data.summary.policy.maximum_group_size,
  exposureWindowS: data.summary.policy.window_s,
  policyIntervalS: data.summary.clock.dt_control_s,
  coordinatedTolerance: data.summary.policy.coordinated_exposure_tolerance,
  reactiveTolerance: data.summary.policy.reactive_exposure_tolerance,
  reliabilityRho: data.summary.capacity.reliability_rho,
};
const output = engine.simulate(data, parameters);
console.log(JSON.stringify({
  frames: output.frames.length,
  entrants: output.entrants.length,
  observations: output.results.observationCount,
  shares: output.results.policyShares,
  reliability: output.results.reliabilityCapacity,
}));
"""
    bundle_path = ROOT / "dashboard" / "data" / "airport_to_airport_traffic.js"
    engine_path = ROOT / "dashboard" / "traffic_engine.js"
    completed = subprocess.run(
        [node, "--input-type=commonjs", "-", str(bundle_path), str(engine_path)],
        input=script,
        text=True,
        capture_output=True,
        check=True,
        timeout=30,
    )
    result = json.loads(completed.stdout)
    bundle = read_bundle(bundle_path)
    assert result["frames"] == len(bundle["frames"])
    assert result["entrants"] == len(bundle["entrants"])
    assert result["observations"] == bundle["summary"]["policy"]["observation_count"]
    assert result["shares"] == bundle["summary"]["policy"]["shares"]
    assert result["reliability"] == bundle["summary"]["capacity"]["q_mix_rho_uam_h"]
