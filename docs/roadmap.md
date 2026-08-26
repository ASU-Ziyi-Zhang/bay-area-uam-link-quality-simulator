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

## S2A — TRB group policy reproduction (implemented)

- Assign individual link quality to each aircraft.
- Evaluate a local group for every active focal aircraft, using up to two
  neighbors on either side and shrinking startup/boundary groups as needed.
- Map group exposure to C/R/F policy and policy-weighted spacing/capacity.
- Preserve policy thresholds/configuration outside the radio model.

## S2B — serving-cell events (future)

- Add association hysteresis and time-to-trigger events.
- Preserve handoff events independently of the group-policy mapping.

## S3A — fixed-lane multi-UAM playback (implemented)

- Add deterministic entries and multiple UAM IDs.
- Animate simultaneous UAMs and color each focal aircraft by C/R/F policy.
- Report policy fractions, instantaneous mixed capacity, and `Q0.95`.

## S3B — multi-lane and multi-level baseline (next)

- Replicate the fixed corridor across explicit lane and altitude-level IDs.
- Keep each aircraft in its assigned lane and level.
- Compare capacity and communication results against the 1 x 1 baseline.

## S3C — dynamic lane/level changes (later)

- Add departure/arrival and vertiport dwell events.
- Add conflict checks and level/lane occupancy.
- Add a separately selectable MOBIL-style controller with SINR and spacing
  criteria.
- Aggregate capacity over explicit windows and report bottleneck causes.

## S4 — GitHub packaging (completed locally)

- Package configuration, data, source, tests, dashboard, and compact references.
- Exclude caches, temporary runs, machine-specific paths, and redundant figures.
- Verify the repository-level checksum and metric manifest before each release.
