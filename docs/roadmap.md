# Implementation roadmap

## S0 — interfaces and baseline (completed)

- Review the multi-rate clock and state schema.
- Confirm that one level/one lane is only the baseline configuration.
- Freeze the corridor, site, trajectory, radio, policy, and capacity interfaces.

## S1 — single-UAM runner (baseline implemented)

- Load a selected `scenarios/<name>/scenario.json` pack through the scenario
  and geometry readers.
- Integrate a fixed-speed, centerline trajectory at `dt_motion_s`.
- Evaluate RSRP/SINR at `dt_radio_s` and write a long-form trace.
- Reproduce the validated centerline association count as a diagnostic.

## S2 — communication events and policy (next)

- Add serving-cell association, hysteresis, and time-to-trigger events.
- Run policy decisions at `dt_control_s`, using a rolling policy window.
- Keep policy thresholds/configuration separate from the radio model.

## S3 — capacity and multiple UAMs

- Add departure/arrival and vertiport dwell events.
- Add multiple UAM IDs, conflict checks, and level/lane occupancy.
- Aggregate capacity over explicit windows and report bottleneck causes.

## S4 — GitHub packaging (completed locally)

- Package configuration, data, source, tests, dashboard, and compact references.
- Exclude caches, temporary runs, machine-specific paths, and redundant figures.
- Verify the repository-level checksum and metric manifest before each release.
