# Multi-aircraft local-group policy baseline: review packet

## Objective and acceptance criteria

Revise the multi-UAM baseline so playback begins at 0 s, every active aircraft
has C/R/F policy, boundary groups use available neighbors, selected-aircraft
link quality is visible, and a failed external 3D engine cannot leave a black
panel.

## Inputs

- scenarios: `airport_to_airport` and `sf_sj_full`;
- one lane, one level, no lane or level switching;
- altitude 300 m AGL, speed 50 m/s, deterministic 32 s entry interval;
- maximum local group: focal aircraft plus up to two active neighbors on each
  side;
- temporal history: available observations up to 30 s;
- green coordinated, yellow reactive, red fallback.

## Files changed

- policy runner and dashboard data builder;
- scenario dashboard bundles;
- traffic HTML, CSS, and JavaScript;
- tests, architecture notes, roadmap, and interpretation documentation.

## Commands

The exact commands are listed in `docs/multi_aircraft_policy.md`. Each run also
writes `manifest.json`, `summary.json`, and `REVIEW.md` under its ignored
`runs/` directory.

## Results

| Scenario | C | R | F | Q0.95 (UAM/h) |
|---|---:|---:|---:|---:|
| Airport-access dynamic stream | 30.90% | 31.40% | 37.70% | 64.403 |
| Full SF-SJ dynamic stream | 44.68% | 32.34% | 22.97% | 76.569 |

The retained original full centered-five TRB regression remains
36.57%/35.82%/27.61% and 72.470 UAM/h.

## Checks

- first capacity/dashboard frame is 0 s with one entering aircraft;
- policy observation count equals the number of active-aircraft time records;
- every active aircraft has exactly one C/R/F policy;
- local group size is between one and five and contains its focal aircraft;
- an established six-aircraft snapshot has boundary group sizes
  3/4/5/5/4/3;
- original TRB centered-five regression remains unchanged;
- JavaScript syntax, Python tests, and repository diff checks must pass before
  acceptance;
- browser inspection must confirm the link chart and automatic 3D fallback.

## Evidence grade and risks

Results are deterministic planning-model estimates. The startup convention is
now explicit, but it is still a modeling choice. Policy fractions are
active-aircraft time shares rather than unique-aircraft shares. The radio model
is not airborne measurement evidence, and capacity is not certified.

## Artifacts for inspection

- `dashboard/traffic.html?scenario=airport_to_airport`;
- `docs/multi_aircraft_policy.md`;
- each v2 run's `summary.json`, `manifest.json`, and trace CSV files.

## Proposed next action and decision

After visual review, approve this corrected 1 x 1 baseline or request another
revision. Multi-lane/multi-level work remains locked until that decision.
