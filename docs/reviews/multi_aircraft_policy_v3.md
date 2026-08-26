# One-second multi-aircraft policy baseline: review packet

## Objective and acceptance criteria

Revise the fixed-lane/fixed-level multi-aircraft simulator so motion, radio,
adaptive policy, and displayed capacity are computed once per second. Replace
the Cesium SVG billboards that rendered as black blocks, reduce visual icon
size, and restore a fixed-view `Recenter` control. Preserve the accepted
five-second TRB regression as a separately computed reference.

## Inputs and provenance

- scenarios: `airport_to_airport` and `sf_sj_full`;
- corridor/site inputs: the retained scenario packs in `scenarios/`;
- one lane, one level, no lane/level switching;
- altitude 300 m AGL, speed 50 m/s, deterministic 32 s entry interval;
- one-second motion, radio, adaptive-policy, and capacity snapshots;
- 30 s policy window using all available one-second observations;
- legacy TRB reference recomputed independently at five seconds.

## Files changed

- group simulator clock and legacy-reference separation;
- both scenario group configurations and generated dashboard bundles;
- 3D billboard renderer, fixed camera range, and Recenter interaction;
- tests, reproduction commands, interpretation notes, and this review packet.

## Commands and configurations

Exact commands are listed in `docs/multi_aircraft_policy.md`. The two immutable
local run directories are:

- `runs/airport-to-airport-group-policy-v3`;
- `runs/sf-sj-full-group-policy-v3`.

Each contains its configuration/input hashes, output hashes, traces, summary,
manifest, and run review.

## Results

| Scenario | Frames | Aircraft-time observations | C | R | F | Q0.95 (UAM/h) |
|---|---:|---:|---:|---:|---:|---:|
| Airport-access | 2,974 | 77,252 | 32.57% | 33.71% | 33.72% | 71.322 |
| Full SF-SJ | 4,527 | 178,652 | 45.81% | 32.90% | 21.28% | 82.411 |

The separately computed five-second full-corridor TRB regression remains
36.57%/35.82%/27.61% and 72.470 UAM/h.

## Engineering, numerical, and scientific checks

- dashboard frames begin at 0 s and consecutive frames differ by exactly 1 s;
- every active aircraft has one C/R/F policy at every frame;
- the 30 s adaptive window uses at most 31 one-second observations;
- legacy-reference results equal the accepted five-second baseline;
- Cesium receives browser-canvas images instead of SVG data-URI billboards;
- the Recenter control returns Cesium and fallback maps to fixed corridor bounds;
- JavaScript syntax, Python tests, repository diff checks, and browser visual QA
  must pass before acceptance.

## Evidence grade and unresolved risks

These are deterministic planning-model estimates. Increasing temporal
resolution changes the dynamic policy fractions and capacity because the
window statistic is evaluated from more samples; this is not merely animation
interpolation. Radio results remain modeled rather than airborne measurements,
and capacity is not certified.

## Artifacts for inspection

- `dashboard/traffic.html?scenario=airport_to_airport`;
- `docs/multi_aircraft_policy.md`;
- both v3 run directories and their manifests/traces.

## Proposed next action and required decision

Approve this revised 1 x 1 baseline or request another correction. Multi-lane
and multi-level implementation remains the next separate gate.
