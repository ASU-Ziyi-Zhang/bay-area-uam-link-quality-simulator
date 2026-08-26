# Architecture

## Scientific chain

The repository keeps one stable chain while allowing each layer to be replaced:

1. `Corridor` maps route progress to metric coordinates.
2. `BaseStationSet` supplies stable site IDs, positions, heights, and subsets.
3. `Trajectory` generates time-indexed UAM positions.
4. `Radio` calculates received power, serving RSRP, interference, and SINR.
5. `LinkQuality` converts radio observations into an explicit link state.
6. `Policy` maps link state to an operating policy.
7. `Capacity` consumes policy/spacing assumptions and reports capacity.

The single-UAM mode exercises layers 1-4. The fixed-lane multi-UAM mode also
exercises the accepted TRB centered-five-aircraft C/R/F mapping and its
policy-weighted planning-capacity calculation. Neither mode implements
operational conflict resolution or certified airborne capacity.

## Replaceable inputs

- Select or add a self-contained pack under `scenarios/`; each pack owns its
  route GeoJSON, site CSV, scenario JSON, simulator JSON, and provenance notes.
- Add, remove, or substitute a station by editing the station CSV and the
  `active_site_ids` list. Array position is never used as a public identifier.
- Replace the trajectory or radio model behind the existing state interfaces.
- Add future lane and flight-level graphs without changing the trace schema.

## Coordinate and height conventions

WGS84 route and site coordinates are projected to EPSG:26910 before distance
calculations. Route progress is arc length `s`; lateral offset is measured along
the local normal. Antenna heights are either source-specific or explicit class
assumptions recorded in the selected scenario's `data/base_stations.csv` and
`docs/assumptions.md`.

## Baseline boundary

The accepted radio baseline is one UAM at 50 m/s, 300 m AGL, and zero lateral
offset. The multi-UAM baseline uses the same speed and altitude, a 32 s entry
interval, and fixed one-lane/one-level geometry. Both use deterministic LOS
planning assumptions with a three-site served set (the strongest serves, the
other two interfere). They are not measured coverage, operator-network
validation, or operational capacity estimates.
