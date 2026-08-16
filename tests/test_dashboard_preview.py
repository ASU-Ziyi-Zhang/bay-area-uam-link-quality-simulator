"""Pin the dashboard's interactive preview to the validated Python kernel.

dashboard/app.js recomputes the flight trace in the browser, including the
default one. These tests port buildPreview()/evaluateRadio() line for line and
assert that the ported path reproduces the trace that build_dashboard.py
exported from the simulator run. If app.js and src/capacity_policy/radio.py
ever drift apart, the dashboard would silently present unvalidated numbers.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "dashboard" / "data" / "dashboard_data.js"

TINY = 5e-324  # JavaScript Number.MIN_VALUE
DEG_PER_M = 1.0 / 111320.0  # the spherical constant app.js uses


@pytest.fixture(scope="module")
def bundle() -> dict:
    raw = BUNDLE.read_text(encoding="utf-8")
    return json.loads(raw[raw.index("=") + 1 : raw.rstrip().rindex(";")])


def evaluate_radio(stations, radio, x_m, y_m, z_m):
    """Port of evaluateRadio() in dashboard/app.js, served set included."""
    links = []
    for i, site in enumerate(stations):
        dx = x_m - site["x_m"]
        dy = y_m - site["y_m"]
        dz = z_m - site["height_m"]
        d2 = max(dx * dx + dy * dy + dz * dz, TINY)
        rx = (
            radio["eirp_dbm"]
            + radio["receiver_gain_db"]
            - 28.0
            - 11.0 * math.log10(d2)
            - 20.0 * math.log10(radio["frequency_ghz"])
        )
        links.append({"i": i, "d2": d2, "rx": rx})

    k = radio.get("served_set_size")
    served = sorted(links, key=lambda link: link["d2"])[:k] if k and k < len(links) else links

    serving = max(served, key=lambda link: link["rx"])
    desired_mw = 10.0 ** (serving["rx"] / 10.0)
    total_mw = sum(10.0 ** (link["rx"] / 10.0) for link in served)
    interference_mw = max(total_mw - desired_mw, TINY)
    noise_mw = 10.0 ** (radio["noise_dbm"] / 10.0)
    return {
        "serving": stations[serving["i"]]["id"],
        "rsrp": serving["rx"] - 10.0 * math.log10(radio["resource_elements"]),
        "sinr": 10.0 * math.log10(desired_mw / (interference_mw + noise_mw)),
    }


def interpolate_centerline(base_trace, corridor_length_m, s_m):
    """Port of interpolateCenterline() in dashboard/app.js."""
    if s_m <= 0:
        first, second = base_trace[0], base_trace[1]
        return {**first, "tangentX": second["x_m"] - first["x_m"], "tangentY": second["y_m"] - first["y_m"]}
    if s_m >= corridor_length_m:
        last, prev = base_trace[-1], base_trace[-2]
        return {**last, "tangentX": last["x_m"] - prev["x_m"], "tangentY": last["y_m"] - prev["y_m"]}
    low, high = 0, len(base_trace) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if base_trace[middle]["s_km"] * 1000.0 <= s_m:
            low = middle
        else:
            high = middle
    a, b = base_trace[low], base_trace[high]
    span = b["s_km"] * 1000.0 - a["s_km"] * 1000.0
    fraction = (s_m - a["s_km"] * 1000.0) / span if span > 0 else 0.0
    return {
        "x_m": a["x_m"] + fraction * (b["x_m"] - a["x_m"]),
        "y_m": a["y_m"] + fraction * (b["y_m"] - a["y_m"]),
        "lat": a["lat"] + fraction * (b["lat"] - a["lat"]),
        "lon": a["lon"] + fraction * (b["lon"] - a["lon"]),
        "tangentX": b["x_m"] - a["x_m"],
        "tangentY": b["y_m"] - a["y_m"],
    }


def build_preview(bundle: dict, speed: float, altitude: float, offset: float):
    """Port of buildPreview() in dashboard/app.js."""
    base_trace = bundle["trace"]
    stations = bundle["stations"]
    radio = bundle["summary"]["radio"]
    corridor_length_m = bundle["summary"]["corridor_length_km"] * 1000.0
    dt = bundle["summary"]["clock"]["dt_radio_s"]
    duration = corridor_length_m / speed

    times, t = [], 0.0
    while t <= duration + 1e-9:
        times.append(min(t, duration))
        t += dt
    if times[-1] < duration - 1e-9:
        times.append(duration)

    rows, previous_serving = [], None
    for t in times:
        s_m = min(speed * t, corridor_length_m)
        center = interpolate_centerline(base_trace, corridor_length_m, s_m)
        norm = math.hypot(center["tangentX"], center["tangentY"]) or 1.0
        normal_x = -center["tangentY"] / norm
        normal_y = center["tangentX"] / norm
        raw = max(0.0, min(1.0, s_m / 2000.0, (corridor_length_m - s_m) / 2000.0))
        local_offset = offset * (raw * raw * (3.0 - 2.0 * raw))
        x_m = center["x_m"] + local_offset * normal_x
        y_m = center["y_m"] + local_offset * normal_y
        link = evaluate_radio(stations, radio, x_m, y_m, altitude)
        rows.append({
            "t": t,
            "s_km": s_m / 1000.0,
            "lat": center["lat"] + local_offset * normal_y * DEG_PER_M,
            "lon": center["lon"] + local_offset * normal_x * DEG_PER_M / math.cos(math.radians(center["lat"])),
            "altitude_m": altitude,
            "handoff": previous_serving is not None and link["serving"] != previous_serving,
            **link,
        })
        previous_serving = link["serving"]
    return rows


def test_bundle_defaults_match_the_simulated_baseline(bundle: dict) -> None:
    defaults = bundle["summary"]["defaults"]
    assert defaults["lateral_offset_m"] == 0.0, "the preview treats the baseline trace as the centerline"
    assert {row["altitude_m"] for row in bundle["trace"]} == {defaults["altitude_m"]}


def test_served_set_is_published_to_the_dashboard(bundle: dict) -> None:
    """The browser kernel cannot apply the served set unless it is exported."""
    assert "served_set_size" in bundle["summary"]["radio"]


def test_nearest_served_set_is_also_the_strongest_set(bundle: dict) -> None:
    """A common EIRP makes received power monotonic in distance, so selecting
    the nearest sites cannot change which site serves."""
    radio = bundle["summary"]["radio"]
    k = radio.get("served_set_size")
    if not k or k >= len(bundle["stations"]):
        pytest.skip("no served-set restriction configured")
    for row in bundle["trace"]:
        links = []
        for site in bundle["stations"]:
            dx = row["x_m"] - site["x_m"]
            dy = row["y_m"] - site["y_m"]
            dz = row["altitude_m"] - site["height_m"]
            d2 = dx * dx + dy * dy + dz * dz
            links.append((site["id"], d2, -11.0 * math.log10(max(d2, TINY))))
        nearest = {x[0] for x in sorted(links, key=lambda x: x[1])[:k]}
        strongest = {x[0] for x in sorted(links, key=lambda x: -x[2])[:k]}
        assert nearest == strongest
        assert max(links, key=lambda x: x[2])[0] == row["serving"]


def test_default_preview_reproduces_the_validated_trace(bundle: dict) -> None:
    defaults = bundle["summary"]["defaults"]
    preview = build_preview(
        bundle, defaults["speed_mps"], defaults["altitude_m"], defaults["lateral_offset_m"]
    )
    baseline = bundle["trace"]
    assert len(preview) == len(baseline)

    for i, (got, want) in enumerate(zip(preview, baseline)):
        assert got["serving"] == want["serving"], f"serving cell diverges at sample {i}"
        assert got["handoff"] == want["handoff"], f"handoff flag diverges at sample {i}"
        assert got["rsrp"] == pytest.approx(want["rsrp"], abs=1e-9), f"RSRP diverges at sample {i}"
        assert got["sinr"] == pytest.approx(want["sinr"], abs=1e-9), f"SINR diverges at sample {i}"
        assert got["t"] == pytest.approx(want["t"], abs=1e-6)
        assert got["s_km"] == pytest.approx(want["s_km"], abs=1e-9)


def test_preview_handoff_total_matches_the_published_summary(bundle: dict) -> None:
    defaults = bundle["summary"]["defaults"]
    preview = build_preview(
        bundle, defaults["speed_mps"], defaults["altitude_m"], defaults["lateral_offset_m"]
    )
    assert sum(row["handoff"] for row in preview) == bundle["summary"]["handoff_count"]


def test_lateral_offset_tapers_to_zero_at_both_vertiports(bundle: dict) -> None:
    defaults = bundle["summary"]["defaults"]
    preview = build_preview(bundle, defaults["speed_mps"], defaults["altitude_m"], 1000.0)
    baseline = bundle["trace"]
    # The first and last samples sit on the centerline; mid-corridor does not.
    assert preview[0]["lat"] == pytest.approx(baseline[0]["lat"], abs=1e-12)
    assert preview[-1]["lat"] == pytest.approx(baseline[-1]["lat"], abs=1e-12)
    mid = len(preview) // 2
    assert abs(preview[mid]["lat"] - baseline[mid]["lat"]) > 1e-4
