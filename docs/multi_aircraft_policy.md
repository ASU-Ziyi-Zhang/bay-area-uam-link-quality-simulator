# Multi-UAM group-policy simulator

## Scope

This mode reproduces the existing TRB communication-to-capacity chain on a
real Bay Area corridor. It is the fixed-lane, fixed-level baseline requested
before multi-lane, multi-level, or lane-change experiments.

The implemented hierarchy is:

1. each aircraft receives individual RSRP, SINR, and serving-site results;
2. each valid focal aircraft is associated with the centered moving group
   `[i-2, i-1, i, i+1, i+2]`;
3. the group's below-threshold exposure determines coordinated (C), reactive
   (R), or fallback (F) policy for the focal aircraft;
4. the instantaneous C/R/F mix maps to policy-weighted spacing and planning
   capacity;
5. the lower fifth percentile of the capacity trace is reported as `Q0.95`.

The first and last two aircraft in the active ordering cannot form a centered
five-aircraft group and are shown as unclassified gray aircraft. They are not
silently assigned to a policy.

## Policy fractions

The reported C/R/F fractions use valid focal-aircraft group-time observations:

`fraction(policy) = valid focal-aircraft time observations in policy / all valid focal-aircraft time observations`.

They are not fractions of unique aircraft, and they are not fractions of
corridor length. Overlapping five-aircraft groups are intentional because the
policy is evaluated from each focal aircraft's local group context.

## Frozen baseline

| Quantity | Airport-access default |
|---|---:|
| Geometry | 1 lane x 1 level |
| Lane/level changes | none |
| Altitude | 300 m AGL |
| Speed | 50 m/s |
| Offered demand | 112.5 UAM/h |
| Entry interval | 32 s |
| Group size | 5 aircraft |

All model and policy parameters are stored in each scenario's
`group_simulator.json`. No random number generator is used.

## Reproduction

```powershell
python scripts\run_group_simulator.py `
  --config scenarios\airport_to_airport\group_simulator.json `
  --output runs\airport-to-airport-group-policy-v1

python scripts\run_group_simulator.py `
  --config scenarios\sf_sj_full\group_simulator.json `
  --output runs\sf-sj-full-group-policy-v1

python scripts\build_traffic_dashboard.py `
  --run-dir runs\airport-to-airport-group-policy-v1 `
  --scenario scenarios\airport_to_airport\scenario.json `
  --output dashboard\data\airport_to_airport_traffic.js

python scripts\build_traffic_dashboard.py `
  --run-dir runs\sf-sj-full-group-policy-v1 `
  --scenario scenarios\sf_sj_full\scenario.json `
  --output dashboard\data\sf_sj_full_traffic.js

python -m pytest -q
```

Add `--overwrite` to a group-simulator command only when intentionally
replacing an existing non-empty run directory.

The full SF-SJ regression case reproduces the accepted TRB result after
rounding: C/R/F = 36.6%/35.8%/27.6% and `Q0.95 = 72.5 UAM/h`.

The airport-access case currently gives C/R/F = 15.8%/37.6%/46.5% and
`Q0.95 = 58.6 UAM/h` under an offered demand of 112.5 UAM/h. These are
model-derived planning results, not measured or certified capacities.

## Dashboard interpretation

- aircraft color is the focal aircraft's current group policy;
- clicking an aircraft reports its individual serving site, RSRP, SINR, policy
  exposure, and centered five-aircraft group; that group is ringed on the 2D
  map;
- the policy bar reports all valid focal-aircraft group-time observations;
- `current mixed capacity` is the instantaneous policy-mix result;
- `reliability capacity` is the lower fifth percentile of the entire trace;
- `demand supported` tests whether offered demand is no greater than `Q0.95`.

The 3D view has a fixed third-person camera. Station cylinders retain modeled
physical height; pixel markers are added only so the sites remain visible at
corridor scale.

## Explicit non-features

This baseline does not implement collision avoidance, lane or level changes,
MOBIL, rerouting, operator handover agreements, or an airborne-validated radio
model. These belong to later, separately comparable simulator modes.
