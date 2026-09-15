"""Two-layer separation check: an NMAC event threshold and predictive screening.

Layer 1 is an **event metric**, not a rule the aircraft flies to. Using the
quantitative convention adopted for safety modelling, a near mid-air collision
requires horizontal and vertical proximity **at the same instant**:

    exists t:  r_h(t) <= 152.4 m  and  |dz(t)| <= 30.48 m

Taking the horizontal minimum at one time and the vertical minimum at another
and combining them would misreport the event. The 500 ft / 100 ft convention is
the one used for safety modelling and simulation; the FAA reporting definition
of a near midair collision is a separate, broader notion that also covers
crew-reported collision hazard.

Layer 2 screens a candidate maneuver *before* the event boundary is reached. It
follows the structure used by NASA's DAIDALUS well-clear implementation: the
horizontal test first asks whether the current horizontal range is already
inside the threshold, and only otherwise evaluates a predicted closest approach
together with a modified tau. Two aircraft flying parallel at constant velocity
are therefore **not** excluded merely because their range rate is zero -- if
they are already inside the distance threshold they are in violation.

The Layer 2 thresholds here are **research assumptions for a cooperative,
same-direction, automated corridor**, deliberately not the DO-365 Phase 1
values. Those were sized to encompass TCAS II advisory thresholds for encounters
that include piloted traffic, and their 4000 ft horizontal threshold is much
larger than the lane spacing studied here. Choosing corridor-specific values is
an open question; this module makes the choice explicit and configurable rather
than implying an approved standard.

Sampling a trajectory is a check, not a continuous-time proof. Every result
records the sample step so a reviewer can vary it.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Quantitative NMAC convention used for safety modelling: 500 ft and 100 ft.
NMAC_HORIZONTAL_M = 152.4
NMAC_VERTICAL_M = 30.48

NMAC_PROVENANCE = (
    "quantitative convention for safety modelling and simulation: "
    "HMD <= 500 ft and VMD <= 100 ft, applied jointly at one instant"
)


@dataclass(frozen=True)
class SeparationVolume:
    """A cylinder plus, for the predictive layer, time thresholds.

    ``horizontal_m`` and ``vertical_m`` define the cylinder. ``tau_mod_s`` and
    ``lookahead_s`` are used only by the predictive screen; set them to zero to
    reduce it to a pure current-distance test.
    """

    horizontal_m: float = NMAC_HORIZONTAL_M
    vertical_m: float = NMAC_VERTICAL_M
    tau_mod_s: float = 0.0
    lookahead_s: float = 0.0
    label: str = "nmac_event"
    provenance: str = NMAC_PROVENANCE

    def __post_init__(self):
        for name in ("horizontal_m", "vertical_m"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("tau_mod_s", "lookahead_s"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


def research_screening_volume(horizontal_m, vertical_m, tau_mod_s, lookahead_s):
    """Build a Layer 2 volume and label it as a project assumption."""
    return SeparationVolume(
        horizontal_m=horizontal_m, vertical_m=vertical_m,
        tau_mod_s=tau_mod_s, lookahead_s=lookahead_s,
        label="research_predictive_screen",
        provenance=("project assumption for a cooperative same-direction automated "
                    "corridor; not RTCA DO-365 Phase 1 DAA well clear, whose "
                    "thresholds were sized for encounters including piloted traffic"))


def horizontal_state(relative_position, relative_velocity):
    """Horizontal range and range rate from relative state."""
    horizontal = np.asarray(relative_position, float)[..., :2]
    velocity = np.asarray(relative_velocity, float)[..., :2]
    distance = np.linalg.norm(horizontal, axis=-1)
    safe = np.maximum(distance, np.finfo(float).tiny)
    rate = np.sum(horizontal * velocity, axis=-1) / safe
    return distance, rate


def horizontal_violation(relative_position, relative_velocity, volume):
    """DAIDALUS-style horizontal test: inside now, or predicted inside soon.

    Returns ``(violated, tau_mod)`` where ``tau_mod`` is ``0`` when already
    inside the threshold and ``inf`` when the pair is not closing.
    """
    distance, rate = horizontal_state(relative_position, relative_velocity)
    inside = distance <= volume.horizontal_m
    closing = rate < 0
    with np.errstate(divide="ignore", invalid="ignore"):
        tau = np.where(closing,
                       (volume.horizontal_m ** 2 - distance ** 2) / (distance * rate),
                       np.inf)
    tau = np.where(inside, 0.0, tau)
    predicted = closing & (tau >= 0) & (tau <= volume.tau_mod_s)
    return inside | predicted, tau


def vertical_violation(relative_position, relative_velocity, volume):
    """Inside the vertical threshold now, or predicted inside within lookahead."""
    dz = np.asarray(relative_position, float)[..., 2]
    dvz = np.asarray(relative_velocity, float)[..., 2]
    inside = np.abs(dz) <= volume.vertical_m
    closing = dz * dvz < 0
    with np.errstate(divide="ignore", invalid="ignore"):
        entry = np.where(closing,
                         (np.abs(dz) - volume.vertical_m) / np.abs(dvz),
                         np.inf)
    entry = np.where(inside, 0.0, entry)
    predicted = closing & (entry >= 0) & (entry <= volume.lookahead_s)
    return inside | predicted, entry


def pair_violation(relative_position, relative_velocity, volume):
    """Joint horizontal-and-vertical violation, evaluated at one instant."""
    horizontal, tau = horizontal_violation(relative_position, relative_velocity, volume)
    vertical, entry = vertical_violation(relative_position, relative_velocity, volume)
    return horizontal & vertical, tau, entry


def _relative_velocity(times, positions):
    """Central-difference velocity on a possibly non-uniform time grid."""
    times = np.asarray(times, float)
    positions = np.asarray(positions, float)
    if len(times) < 2:
        return np.zeros_like(positions)
    return np.gradient(positions, times, axis=0, edge_order=1)


def screen_trajectories(times, positions, volume, *, velocities=None,
                        identifiers=None, stop_at_first=False):
    """Evaluate ``volume`` for every aircraft pair on a common time grid.

    ``positions`` has shape ``(n_time, n_aircraft, 3)`` in a common Cartesian
    frame. ``velocities`` may be supplied when the caller already has exact
    values; otherwise they are differenced from the sampled positions, which is
    accurate only to the sample step.

    A single aircraft produces no pairs and therefore no violations. That is a
    property of the metric, not evidence that the trajectory is safe.
    """
    times = np.asarray(times, float)
    positions = np.asarray(positions, float)
    if positions.ndim != 3 or positions.shape[0] != len(times) or positions.shape[2] != 3:
        raise ValueError("positions must have shape (n_time, n_aircraft, 3)")
    count = positions.shape[1]
    if identifiers is None:
        identifiers = [f"AC{index:02d}" for index in range(count)]
    if len(identifiers) != count:
        raise ValueError("identifier count must match the aircraft count")
    differenced_velocities = velocities is None
    if differenced_velocities:
        velocities = _relative_velocity(times, positions)
    velocities = np.asarray(velocities, float)

    step = float(np.min(np.diff(times))) if len(times) > 1 else 0.0
    events, minimum = [], None
    for first in range(count):
        for second in range(first + 1, count):
            relative_position = positions[:, second, :] - positions[:, first, :]
            relative_velocity = velocities[:, second, :] - velocities[:, first, :]
            violated, tau, entry = pair_violation(relative_position, relative_velocity, volume)
            horizontal = np.linalg.norm(relative_position[:, :2], axis=-1)
            vertical = np.abs(relative_position[:, 2])
            # Joint closest approach: rank by the cylinder coordinate rather
            # than by either axis alone, so the reported worst instant is one
            # the metric would actually judge.
            coordinate = np.maximum(horizontal / volume.horizontal_m,
                                    vertical / volume.vertical_m)
            worst = int(np.argmin(coordinate))
            record = {"pair": (identifiers[first], identifiers[second]),
                      "closest_t_s": float(times[worst]),
                      "horizontal_m": float(horizontal[worst]),
                      "vertical_m": float(vertical[worst]),
                      "cylinder_coordinate": float(coordinate[worst])}
            if minimum is None or record["cylinder_coordinate"] < minimum["cylinder_coordinate"]:
                minimum = record
            if violated.any():
                index = int(np.argmax(violated))
                events.append({**record, "violation": True,
                               "first_violation_t_s": float(times[index]),
                               "violation_horizontal_m": float(horizontal[index]),
                               "violation_vertical_m": float(vertical[index]),
                               "tau_mod_s": float(tau[index]),
                               "vertical_entry_s": float(entry[index])})
                if stop_at_first:
                    break
        if stop_at_first and events:
            break
    return {"label": volume.label, "provenance": volume.provenance,
            "aircraft": list(identifiers), "pairs_checked": count * (count - 1) // 2,
            "violations": events, "violation_count": len(events),
            "closest_pair": minimum, "sample_step_s": step,
            "sampled_check_not_continuous_proof": True,
            "velocities_differenced_from_samples": differenced_velocities}


def nmac_screen(times, positions, **kwargs):
    """Layer 1 convenience wrapper using the NMAC event cylinder."""
    return screen_trajectories(times, positions, SeparationVolume(), **kwargs)


def volume_contains_cylinder(volume, horizontal_m=NMAC_HORIZONTAL_M,
                             vertical_m=NMAC_VERTICAL_M):
    """Whether ``volume`` fully contains the given cylinder.

    For two cylinders this is a per-axis comparison. Reported explicitly so a
    screening layer is never assumed to dominate the event layer without check.
    """
    return bool(volume.horizontal_m >= horizontal_m and volume.vertical_m >= vertical_m)


def ellipsoid_contains_cylinder(horizontal_semi_axis_m, vertical_semi_axis_m,
                                horizontal_m=NMAC_HORIZONTAL_M,
                                vertical_m=NMAC_VERTICAL_M):
    """Whether an ellipsoid screen contains the NMAC cylinder.

    Larger semi-axes on each axis do not by themselves guarantee containment,
    so the corner of the cylinder is tested directly:
    ``(h/a)^2 + (v/b)^2 <= 1``.
    """
    value = ((horizontal_m / horizontal_semi_axis_m) ** 2
             + (vertical_m / vertical_semi_axis_m) ** 2)
    return {"corner_value": float(value), "contains": bool(value <= 1.0)}
