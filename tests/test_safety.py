import numpy as np
import pytest

from uam_simulator.safety import (
    NMAC_HORIZONTAL_M, NMAC_VERTICAL_M, SeparationVolume,
    ellipsoid_contains_cylinder, horizontal_violation, nmac_screen,
    research_screening_volume, screen_trajectories, volume_contains_cylinder,
    vertical_violation,
)


def straight_pair(lateral_gap, longitudinal_gap=0.0, vertical_gap=0.0,
                  speed=50.0, duration=60.0, steps=121):
    """Two aircraft holding a constant relative geometry."""
    times = np.linspace(0.0, duration, steps)
    first = np.c_[speed * times, np.zeros_like(times), np.full_like(times, 300.0)]
    second = np.c_[speed * times + longitudinal_gap,
                   np.full_like(times, lateral_gap),
                   np.full_like(times, 300.0 + vertical_gap)]
    return times, np.stack([first, second], axis=1)


def test_single_aircraft_has_no_pairs_and_no_violation():
    times = np.linspace(0, 60, 61)
    positions = np.c_[50 * times, np.zeros_like(times), np.full_like(times, 300.0)][:, None, :]
    report = nmac_screen(times, positions)
    assert report["pairs_checked"] == 0
    assert report["violation_count"] == 0
    assert report["closest_pair"] is None
    assert report["sampled_check_not_continuous_proof"] is True


def test_abreast_aircraft_one_hundred_metres_apart_are_an_nmac():
    times, positions = straight_pair(lateral_gap=100.0)
    report = nmac_screen(times, positions)
    assert report["violation_count"] == 1
    event = report["violations"][0]
    assert event["violation_horizontal_m"] == pytest.approx(100.0)
    assert event["violation_vertical_m"] == pytest.approx(0.0)


def test_lane_spacing_of_three_hundred_metres_abreast_is_not_an_nmac():
    times, positions = straight_pair(lateral_gap=300.0)
    assert nmac_screen(times, positions)["violation_count"] == 0


def test_longitudinal_separation_defeats_a_small_lateral_gap():
    times, positions = straight_pair(lateral_gap=100.0, longitudinal_gap=1500.0)
    report = nmac_screen(times, positions)
    assert report["violation_count"] == 0
    # Horizontal range is the planar distance, not the lateral offset.
    assert report["closest_pair"]["horizontal_m"] == pytest.approx(np.hypot(1500.0, 100.0))


def test_vertical_separation_alone_defeats_the_joint_condition():
    times, positions = straight_pair(lateral_gap=100.0, vertical_gap=100.0)
    assert nmac_screen(times, positions)["violation_count"] == 0
    # The same pair at the same horizontal geometry does violate when co-altitude.
    times, positions = straight_pair(lateral_gap=100.0, vertical_gap=0.0)
    assert nmac_screen(times, positions)["violation_count"] == 1


def test_axis_minima_at_different_instants_are_not_an_event():
    """Horizontal close early, vertical close late, never both at once."""
    times = np.linspace(0.0, 100.0, 201)
    first = np.c_[np.zeros_like(times), np.zeros_like(times), np.full_like(times, 300.0)]
    horizontal = np.where(times < 50.0, 50.0, 5000.0)
    vertical = np.where(times < 50.0, 500.0, 0.0)
    second = np.c_[horizontal, np.zeros_like(times), 300.0 + vertical]
    positions = np.stack([first, second], axis=1)
    report = nmac_screen(times, positions)
    assert report["violation_count"] == 0
    assert min(np.abs(vertical)) == 0.0 and min(horizontal) < NMAC_HORIZONTAL_M


def test_parallel_non_closing_traffic_inside_the_threshold_still_violates():
    """Zero range rate does not exempt a pair already inside the distance."""
    volume = research_screening_volume(horizontal_m=500.0, vertical_m=45.0,
                                       tau_mod_s=35.0, lookahead_s=35.0)
    violated, tau = horizontal_violation(np.array([300.0, 0.0, 0.0]),
                                         np.array([0.0, 0.0, 0.0]), volume)
    assert bool(violated) and float(tau) == pytest.approx(0.0)


def test_modified_tau_predicts_a_closing_pair_before_it_arrives():
    volume = research_screening_volume(horizontal_m=200.0, vertical_m=45.0,
                                       tau_mod_s=35.0, lookahead_s=35.0)
    closing = np.array([1000.0, 0.0, 0.0]), np.array([-30.0, 0.0, 0.0])
    violated, tau = horizontal_violation(*closing, volume)
    expected = (200.0 ** 2 - 1000.0 ** 2) / (1000.0 * -30.0)
    assert float(tau) == pytest.approx(expected)
    assert bool(violated) == (0 <= expected <= 35.0)
    # Opening geometry is never a horizontal violation.
    opening, _ = horizontal_violation(np.array([1000.0, 0.0, 0.0]),
                                      np.array([30.0, 0.0, 0.0]), volume)
    assert not bool(opening)


def test_vertical_screen_predicts_entry_within_the_lookahead():
    volume = research_screening_volume(horizontal_m=500.0, vertical_m=45.0,
                                       tau_mod_s=35.0, lookahead_s=30.0)
    approaching = np.array([0.0, 0.0, 100.0]), np.array([0.0, 0.0, -5.0])
    violated, entry = vertical_violation(*approaching, volume)
    assert float(entry) == pytest.approx((100.0 - 45.0) / 5.0)
    assert bool(violated)
    diverging, _ = vertical_violation(np.array([0.0, 0.0, 100.0]),
                                      np.array([0.0, 0.0, 5.0]), volume)
    assert not bool(diverging)


def test_predictive_layer_can_flag_what_the_event_layer_does_not():
    times, positions = straight_pair(lateral_gap=300.0)
    assert nmac_screen(times, positions)["violation_count"] == 0
    volume = research_screening_volume(horizontal_m=500.0, vertical_m=45.0,
                                       tau_mod_s=35.0, lookahead_s=35.0)
    report = screen_trajectories(times, positions, volume)
    assert report["violation_count"] == 1
    assert report["label"] == "research_predictive_screen"
    assert "not RTCA DO-365" in report["provenance"]


def test_supplied_velocities_are_used_instead_of_differencing():
    times, positions = straight_pair(lateral_gap=300.0)
    velocities = np.zeros_like(positions)
    velocities[:, :, 0] = 50.0
    report = screen_trajectories(times, positions, SeparationVolume(), velocities=velocities)
    assert report["violation_count"] == 0
    assert report["sample_step_s"] == pytest.approx(times[1] - times[0])


def test_identifier_and_shape_validation():
    times, positions = straight_pair(lateral_gap=300.0)
    with pytest.raises(ValueError):
        screen_trajectories(times, positions, SeparationVolume(), identifiers=["only-one"])
    with pytest.raises(ValueError):
        screen_trajectories(times, positions[:, :, :2], SeparationVolume())


def test_containment_helpers_report_the_project_screen():
    assert volume_contains_cylinder(SeparationVolume(horizontal_m=200.0, vertical_m=100.0))
    assert not volume_contains_cylinder(SeparationVolume(horizontal_m=100.0, vertical_m=100.0))
    # The simulator's engineering ellipsoid does contain the NMAC cylinder.
    result = ellipsoid_contains_cylinder(200.0, 100.0)
    assert result["corner_value"] == pytest.approx(
        (NMAC_HORIZONTAL_M / 200.0) ** 2 + (NMAC_VERTICAL_M / 100.0) ** 2)
    assert result["contains"] is True
    # Larger semi-axes on each axis alone do not guarantee containment.
    tight = ellipsoid_contains_cylinder(160.0, 32.0)
    assert tight["contains"] is False


def test_volume_validation_rejects_bad_thresholds():
    with pytest.raises(ValueError):
        SeparationVolume(horizontal_m=0.0)
    with pytest.raises(ValueError):
        SeparationVolume(tau_mod_s=-1.0)


def test_velocity_source_is_reported_without_overwriting_its_origin():
    times, positions = straight_pair(300.)
    assert nmac_screen(times, positions)["velocities_differenced_from_samples"] is True
    velocity = np.zeros_like(positions)
    velocity[:, :, 0] = 50.
    assert nmac_screen(times, positions, velocities=velocity)["velocities_differenced_from_samples"] is False
