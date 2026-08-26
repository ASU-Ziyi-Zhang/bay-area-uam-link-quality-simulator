# Multi-aircraft policy baseline: review packet

## Objective

Reproduce the accepted TRB link-quality to group-policy to capacity chain on
the real corridor, expose the fraction of aircraft observations using each
policy, and animate simultaneous aircraft using the requested policy colors.

## Frozen inputs

- scenarios: `airport_to_airport` and `sf_sj_full`;
- one lane, one level, no lane or level switching;
- altitude 300 m AGL and speed 50 m/s;
- deterministic 32 s entry interval, equivalent to 112.5 UAM/h;
- centered five-aircraft group;
- green coordinated, yellow reactive, red fallback, gray unclassified.

## Review artifacts

- model: `src/uam_simulator/group_config.py` and `group_runner.py`;
- scenario configuration: each scenario's `group_simulator.json`;
- runner and dashboard builder: `scripts/run_group_simulator.py` and
  `scripts/build_traffic_dashboard.py`;
- interface: `dashboard/traffic.html`, `traffic_app.js`, and `traffic.css`;
- interpretation contract: `docs/multi_aircraft_policy.md`;
- verification: `tests/test_group_simulator.py` and
  `tests/test_traffic_dashboard.py`.

## Verified findings

| Scenario | C | R | F | Q0.95 (UAM/h) |
|---|---:|---:|---:|---:|
| Full SF-SJ regression | 36.57% | 35.82% | 27.61% | 72.470 |
| Airport-access corridor | 15.84% | 37.63% | 46.53% | 58.643 |

The full SF-SJ case reproduces the accepted slide result after rounding:
36.6%/35.8%/27.6% and 72.5 UAM/h.

## Checks

- 31 Python tests passed;
- JavaScript syntax check passed;
- repository diff whitespace check passed;
- airport and full-corridor runs were regenerated from their scenario files;
- the 2D dashboard was rendered in a headless browser and inspected;
- 3D uses the existing network-loaded Cesium dependency and remains subject to
  CDN availability, while the 2D view and results remain self-contained.

## Evidence boundary and remaining risks

- Policy fractions are focal-aircraft group-time observations, not unique-UAM
  shares or corridor-length fractions.
- Capacity is a deterministic planning output from the accepted TRB mapping;
  it is not operationally certified.
- The radio model remains a planning model, not airborne measurement data.
- No collision avoidance, lane change, level change, or MOBIL controller is in
  this baseline.

## Human decision requested

Review the multi-UAM page and decide whether this 1 x 1 fixed-control baseline
is accepted as the comparison case for the next multi-lane/multi-level stage.
