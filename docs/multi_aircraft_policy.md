# Multi-UAM local-group policy simulator

## Scope

This mode applies the existing TRB communication-to-policy-to-capacity chain
to a traffic stream on the real Bay Area corridor. It is the fixed-lane,
fixed-level baseline used before multi-lane, multi-level, or lane-change
experiments.

The implemented hierarchy is:

1. every active aircraft receives individual RSRP, SINR, and serving-site
   results;
2. every active aircraft is a focal aircraft and uses up to two active
   neighbors ahead and two behind;
3. the local group's link exposure determines coordinated (C), reactive (R),
   or fallback (F) policy for that focal aircraft;
4. the instantaneous C/R/F mix maps to policy-weighted spacing and planning
   capacity;
5. the lower fifth percentile of the complete capacity trace is reported as
   `Q0.95`.

## Startup and corridor boundaries

The simulation starts at `t = 0` with the first aircraft at the origin. It
does not jump to steady state.

Every active aircraft has a policy. The maximum local group size is five, but
the group shrinks when fewer neighbors are available:

- the first and last aircraft normally use three aircraft: self plus the two
  available inward neighbors;
- the second and second-last normally use four;
- interior aircraft use five;
- during initial loading, groups may contain one or two aircraft.

The temporal exposure window follows the same principle. Before 30 s of
history exist, policy uses all observations available since that focal
aircraft entered. There is no unclassified startup or edge state.

## Policy fractions

The displayed fractions use all active focal-aircraft time observations:

`fraction(policy) = active focal-aircraft time observations in policy / all active focal-aircraft time observations`.

They are not unique-aircraft shares and are not corridor-length fractions.

For traceability, `summary.json` separately retains the original TRB
regression definition: full centered-five groups after the legacy warmup. That
comparison remains unchanged but is not used to hide startup or edge aircraft
in the animation.

## Frozen baseline

| Quantity | Airport-access default |
|---|---:|
| Geometry | 1 lane x 1 level |
| Lane/level changes | none |
| Altitude | 300 m AGL |
| Speed | 50 m/s |
| Offered demand | 112.5 UAM/h |
| Entry interval | 32 s |
| Maximum local group | 5 aircraft |
| Policy history | up to 30 s |
| Motion/radio/adaptive policy step | 1 s |
| Legacy TRB reference step | 5 s |

All model and policy parameters are stored in each scenario's
`group_simulator.json`. No random number generator is used.

## Reproduction

```powershell
python scripts\run_group_simulator.py `
  --config scenarios\airport_to_airport\group_simulator.json `
  --output runs\airport-to-airport-group-policy-v3

python scripts\run_group_simulator.py `
  --config scenarios\sf_sj_full\group_simulator.json `
  --output runs\sf-sj-full-group-policy-v3

python scripts\build_traffic_dashboard.py `
  --run-dir runs\airport-to-airport-group-policy-v3 `
  --scenario scenarios\airport_to_airport\scenario.json `
  --output dashboard\data\airport_to_airport_traffic.js

python scripts\build_traffic_dashboard.py `
  --run-dir runs\sf-sj-full-group-policy-v3 `
  --scenario scenarios\sf_sj_full\scenario.json `
  --output dashboard\data\sf_sj_full_traffic.js

python -m pytest -q
```

Add `--overwrite` only when intentionally replacing an existing non-empty run
directory.

## Current deterministic results

| Definition | Scenario | C | R | F | Q0.95 (UAM/h) |
|---|---|---:|---:|---:|---:|
| Dynamic one-second local group, all active observations | Airport access | 32.57% | 33.71% | 33.72% | 71.322 |
| Dynamic one-second local group, all active observations | Full SF-SJ | 45.81% | 32.90% | 21.28% | 82.411 |
| Original centered-five TRB regression | Full SF-SJ | 36.57% | 35.82% | 27.61% | 72.470 |

These are planning-model outputs, not measured or certified capacities.
The one-second run changes the dynamic estimates because each 30 s exposure
window contains up to 31 observations instead of seven. It is therefore a
model-resolution revision, not merely smoother animation. The separately
computed five-second TRB row is unchanged.

## Dashboard interpretation

- aircraft color is the focal aircraft's current C/R/F policy;
- clicking an aircraft reports its serving site, RSRP, SINR, exposure, and
  current local group; the group is ringed on the 2D map;
- the selected-aircraft chart shows its RSRP and SINR profile with a current
  position cursor and the SINR threshold;
- the policy bar reports all active focal-aircraft time observations;
- `current mixed capacity` is the instantaneous policy-mix result;
- `reliability capacity` is the lower fifth percentile of the complete trace;
- `demand supported` tests whether offered demand is no greater than `Q0.95`.

The primary 3D view is synchronized with the selected aircraft used by the
right-hand metrics. It reuses the single-UAM drone and macro-site geometry,
keeps a fixed 315-degree bearing and fixed pitch while translating with that
focal aircraft, and renders the remaining active aircraft as surrounding
traffic with C/R/F silhouettes. The focal aircraft has an altitude reference
and a dashed line to its current serving site. Selecting another aircraft
immediately transfers the 3D focal view; `Recenter` restores follow mode after
free camera interaction. If Cesium is unavailable, a selected-aircraft map
fallback preserves the traffic and serving-link layers.

## Explicit non-features

This baseline does not implement collision avoidance, lane or level changes,
MOBIL, rerouting, operator handover agreements, or an airborne-validated radio
model. These belong to later, separately comparable simulator modes.
