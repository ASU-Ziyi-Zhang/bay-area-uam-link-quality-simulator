from dataclasses import replace

import numpy as np
import pytest

from uam_simulator.dynamic_transitions import (
    Aircraft, Layout, TransitionConfig, acc_acceleration, advance, choose_transitions,
    make_maneuver, preview, simulate, smoothstep, synthetic_radio, violation,
)


def fast_config(**changes):
    config = TransitionConfig(dt_s=0.25, forecast_dt_s=0.25, duration_s=12,
        decision_interval_s=1, forecast_horizon_s=8, cooldown_s=2, cruise_mps=10,
        min_gap_m=5, horizontal_separation_m=2, vertical_separation_m=2,
        min_maneuver_s=2, lateral_speed_limit_mps=5, vertical_speed_limit_mps=5,
        lateral_accel_limit_mps2=10, vertical_accel_limit_mps2=10,
        acc_standstill_m=3, acc_headway_s=1)
    return replace(config, **changes)


def test_quintic_endpoints_and_exact_peak_limits():
    config = TransitionConfig()
    layout = Layout((-250, 250), (300, 600))
    for cell in [(1, 0), (0, 1)]:
        maneuver = make_maneuver((0, 0), cell, 5, layout, config)
        times = np.linspace(5, 5+maneuver.duration_s, 10001)
        p, v, a = maneuver.sample(times, layout)
        np.testing.assert_allclose(p[[0, -1]], [layout.position((0, 0)), layout.position(cell)])
        np.testing.assert_allclose(v[[0, -1]], 0, atol=1e-10)
        np.testing.assert_allclose(a[[0, -1]], 0, atol=1e-10)
        assert np.max(np.abs(v[:, 0])) <= config.lateral_speed_limit_mps+1e-9
        assert np.max(np.abs(v[:, 1])) <= config.vertical_speed_limit_mps+1e-9
        assert np.max(np.abs(a[:, 0])) <= config.lateral_accel_limit_mps2+1e-9
        assert np.max(np.abs(a[:, 1])) <= config.vertical_accel_limit_mps2+1e-9


def test_arbitrary_grid_and_diagonal_rejected():
    layout = Layout((-500, 0, 500), (150, 300, 450))
    assert len(layout.neighbors((1, 1))) == 4
    assert len(layout.neighbors((0, 0))) == 2
    with pytest.raises(ValueError, match="axis-aligned"):
        make_maneuver((0, 0), (1, 1), 0, layout, TransitionConfig())
    with pytest.raises(ValueError):
        Layout((0, 0), (300,))
    with pytest.raises(ValueError):
        TransitionConfig(dt_s=0.7)


def test_acc_cruise_closing_speed_and_acceleration_bounds():
    config = TransitionConfig()
    assert acc_acceleration(50, None, None, config) == 0
    assert 0 < acc_acceleration(40, None, None, config) <= config.accel_limit_mps2
    assert acc_acceleration(50, 1000, 20, config) < acc_acceleration(50, 1000, 50, config)
    assert acc_acceleration(50, 10, 0, config) == -config.decel_limit_mps2


def test_acc_low_speed_target_does_not_undercut_hard_gap():
    config = TransitionConfig()
    assert acc_acceleration(0, config.min_gap_m, 0, config) == 0
    assert acc_acceleration(1, config.min_gap_m, 1, config) == 0


def test_ballistic_motion_no_instant_speed_jump_and_no_reverse():
    config = fast_config()
    fleet = [Aircraft("rear", 0, 10, (0, 0)), Aircraft("front", 6, 0, (0, 0))]
    advance(fleet, 0, 0.25, "acc", config)
    assert 0 <= fleet[0].v_mps < 10
    assert fleet[0].s_m > 0
    assert fleet[1].s_m >= 6
    for a in fleet:
        assert 0 <= a.v_mps <= config.cruise_mps


def test_fixed_longitudinal_speed_is_not_fixed_total_speed():
    layout = Layout((0, 10), (100,))
    a = Aircraft("a", 0, 10, (0, 0))
    a.maneuver = make_maneuver(a.cell, (1, 0), 0, layout, fast_config())
    _, velocity, _ = a.pose(a.maneuver.duration_s/2, layout)
    assert velocity[0] == 10
    assert np.linalg.norm(velocity) > 10


def test_front_and_rear_gaps_are_both_enforced():
    layout, config = Layout((0, 10), (100,)), fast_config()
    for other_s in [-4, 4]:
        own = Aircraft("own", 0, 10, (0, 0))
        own.maneuver = make_maneuver((0, 0), (1, 0), 0, layout, config)
        assert violation([own, Aircraft("other", other_s, 10, (1, 0))], 0, layout, config)["reason"] == "longitudinal_gap"


def test_forecast_checks_mid_transition_not_only_endpoint():
    layout, config = Layout((0, 10), (100,)), fast_config(min_gap_m=1)
    a = Aircraft("a", 0, 10, (0, 0), make_maneuver((0, 0), (1, 0), 0, layout, config))
    b = Aircraft("b", 0, 10, (1, 0), make_maneuver((1, 0), (0, 0), 0, layout, config))
    # Dual-cell gap reservation rejects the exchange before intersection occurs.
    positions, issue = preview([a, b], 0, "fixed_cruise", layout, config)
    assert positions is None and issue is not None


def test_future_closing_gap_rejected_even_when_initial_gap_ok():
    layout, config = Layout((0,), (100,)), fast_config()
    fleet = [Aircraft("rear", 0, 10, (0, 0)), Aircraft("front", 10, 0, (0, 0))]
    assert violation(fleet, 0, layout, config) is None
    _, issue = preview(fleet, 0, "fixed_cruise", layout, config)
    assert issue and issue["forecast_time_s"] > 0


def test_comm_benefit_required_and_one_transition_completes():
    layout, config = Layout((0, 10), (100,)), fast_config()
    initial = [Aircraft("a", 0, 10, (0, 0))]
    flat = lambda p: np.zeros(p.shape[:-1])
    assert not [e for e in choose_transitions([replace(a) for a in initial], 0, "fixed_cruise", layout, config, flat)
                if e["status"] == "accepted"]
    improving = lambda p: p[..., 1]-5
    result = simulate(initial, layout, config, improving, mode="fixed_cruise", transitions=True)
    assert result["summary"]["accepted_transitions"] == 1
    assert result["summary"]["completed_transitions"] == 1
    assert result["trace"][-1]["lane"] == 1
    assert initial[0].cell == (0, 0) and initial[0].s_m == 0  # no caller-state mutation


def test_vertical_transition_and_cooldown():
    layout, config = Layout((0,), (100, 110, 120)), fast_config()
    radio = lambda p: p[..., 2]-108
    fleet = [Aircraft("a", 0, 10, (0, 0))]
    result = simulate(fleet, layout, config, radio, mode="fixed_cruise", transitions=True)
    accepted = [e for e in result["events"] if e["status"] == "accepted"]
    assert accepted and accepted[0]["target"] == [0, 1]
    assert max(r["v_vertical_mps"] for r in result["trace"]) > 0
    if len(accepted) > 1:
        assert accepted[1]["t_s"] >= accepted[0]["maneuver_duration_s"]+config.cooldown_s


def test_serial_arbitration_prevents_two_aircraft_claiming_same_gap():
    layout, config = Layout((-10, 0, 10), (100,)), fast_config()
    fleet = [Aircraft("a", 0, 10, (0, 0)), Aircraft("b", 0, 10, (2, 0))]
    radio = lambda p: 1-np.abs(p[..., 1])
    events = choose_transitions(fleet, 0, "fixed_cruise", layout, config, radio)
    assert len([e for e in events if e["status"] == "accepted"]) == 1
    assert any(e["status"] == "gap_or_conflict_rejected" for e in events)


def test_fixed_baseline_exact_and_acc_changes_spacing_response():
    layout, config = Layout((0,), (100,)), fast_config(acc_headway_s=3)
    initial = [Aircraft("rear", 0, 10, (0, 0)), Aircraft("front", 20, 10, (0, 0))]
    radio = lambda p: np.zeros(p.shape[:-1])
    fixed = simulate(initial, layout, config, radio, mode="fixed_cruise", transitions=False)
    acc = simulate(initial, layout, config, radio, mode="acc", transitions=False)
    assert fixed["summary"]["mean_progress_loss_m"] == 0
    assert acc["summary"]["mean_progress_loss_m"] > 0
    assert acc["summary"]["status"] == "completed_sampled_checks"
    assert acc["summary"]["capacity_uam_h"] is None


def test_refined_time_step_and_determinism():
    layout, config = Layout((0, 10), (100,)), fast_config()
    initial = [Aircraft("a", 0, 10, (0, 0))]
    radio = lambda p: p[..., 1]-5
    a = simulate(initial, layout, config, radio, mode="fixed_cruise", transitions=True)
    b = simulate(initial, layout, config, radio, mode="fixed_cruise", transitions=True)
    assert a == b
    refined = simulate(initial, layout, replace(config, dt_s=0.125, forecast_dt_s=0.125), radio,
                       mode="fixed_cruise", transitions=True)
    assert refined["summary"]["accepted_transitions"] == a["summary"]["accepted_transitions"]
    assert abs(refined["summary"]["poor_link_sample_fraction"]-a["summary"]["poor_link_sample_fraction"]) < 0.03


def test_failures_are_reported_and_invalid_input_rejected():
    layout, config = Layout((0,), (100,)), fast_config()
    fleet = [Aircraft("a", 0, 10, (0, 0)), Aircraft("b", 1, 10, (0, 0))]
    result = simulate(fleet, layout, config, synthetic_radio, mode="fixed_cruise", transitions=False)
    assert result["summary"]["status"] == "stopped_on_violation"
    assert result["summary"]["elapsed_s"] == 0
    with pytest.raises(ValueError, match="initialization"):
        simulate([Aircraft("a", 0, 2, (0, 0))], layout, config, synthetic_radio,
                 mode="fixed_cruise", transitions=False)
    with pytest.raises(ValueError, match="radio"):
        simulate([fleet[0]], layout, config, lambda p: np.array([np.nan]),
                 mode="fixed_cruise", transitions=False)


def test_protected_region_checks_other_cells():
    layout, config = Layout((0, 1), (100,)), fast_config()
    fleet = [Aircraft("a", 0, 10, (0, 0)), Aircraft("b", 0, 10, (1, 0))]
    assert violation(fleet, 0, layout, config)["reason"] == "protected_ellipsoid"


def test_mechanism_runner_snapshots_source_and_refuses_overwrite(tmp_path):
    from dataclasses import asdict
    import gzip
    import hashlib
    import importlib.util
    import json
    from pathlib import Path
    import zipfile

    root = Path(__file__).resolve().parents[1]
    module_spec = importlib.util.spec_from_file_location("transition_checks", root / "scripts/run_transition_checks.py")
    cli = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(cli)
    spec = json.loads((root / "research/dynamic-transitions/configs/mechanism_checks.json").read_text())
    spec["parameters"] = asdict(fast_config())
    spec["layouts"] = [{"id": "tiny", "offsets_m": [0, 10], "heights_m": [100],
                        "initial_aircraft": [{"aircraft_id": "a", "s_m": 0, "v_mps": 10, "cell": [0, 0]}]}]
    spec["refinement"]["layout_id"] = "tiny"
    config_path = tmp_path / "input.json"
    config_path.write_text(json.dumps(spec))
    output = tmp_path / "run"
    summaries = cli.run(config_path, output)
    assert len(summaries) == 6
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["status"] == "completed" and not manifest["source_changed_during_run"]
    assert manifest["execution"]["source_snapshot_included"]
    for artifact in manifest["outputs"]:
        assert hashlib.sha256((output / artifact["path"]).read_bytes()).hexdigest() == artifact["sha256"]
    with zipfile.ZipFile(output / "source_snapshot.zip") as archive:
        for row in manifest["code"]:
            assert hashlib.sha256(archive.read(row["path"])).hexdigest() == row["sha256"]
    with gzip.open(output / "tiny_fixed_cruise_off_refine1/trace.json.gz", "rt") as stream:
        assert json.load(stream)[-1]["t_s"] == fast_config().duration_s
    with pytest.raises(FileExistsError):
        cli.run(config_path, output)
