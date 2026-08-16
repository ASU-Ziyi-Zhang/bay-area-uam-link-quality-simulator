# Simulator review packet - simulator

## Objective

Run one UAM along the declared SF–SJ centerline and record motion, radio, and
control-clock traces using the multi-rate simulator.

## Acceptance checks

- Scenario: `sf_sj_real`; 18 retained base stations.
- Corridor length: 75.423116 km.
- Fixed trajectory: 300.0 m altitude, 0.0 m lateral offset, 50.0 m/s.
- Motion/radio/control clocks: 1.0/1.0/5.0 s.
- Endpoint reached: True.
- Finite position and radio values: True.
- Handoffs: 13.

## Artifacts

See `manifest.json` for input/code/output SHA-256 records. The CSV traces
are the editable numerical outputs for inspection.

## Boundary

This is a deterministic single-UAM link-quality baseline. Policy, controller,
multi-UAM conflicts, and capacity are not active.

## Decision requested

Approve this baseline, request a correction, or branch to the next policy/
controller implementation.
