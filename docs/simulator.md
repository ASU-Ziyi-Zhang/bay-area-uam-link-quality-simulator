# Simulator architecture and timing contract

## 1. Why one fixed time step is insufficient

Different processes have different natural time scales:

| Process | Initial clock | Reason |
|---|---:|---|
| Motion/kinematics | 1 s | resolves position and speed along a curved corridor |
| Radio/RSRP/SINR | 1 s | avoids hiding association changes and short fades |
| Controller/policy | 5 s | matches a first control-policy update interval |
| Policy memory | 30 s | supports rolling windows and time-to-trigger logic |
| Capacity output | 60 s | reports flow/occupancy without tying capacity to radio `dt` |

These are defaults, not universal constants.  Every run must record them in
its manifest.  Event timestamps (departure, arrival, dwell, handoff,
controller update) are preserved independently of regular sampling.

## 2. Core interfaces

1. **CorridorProvider**: maps route progress `s_m`, `level_id`, and `lane_id`
   to a 3-D position and local heading.  The current SF--SJ case uses one
   centerline, one level, and one lane.
2. **BaseStationSet**: provides site IDs and physical/radio attributes.  Site
   selection is by stable `site_id`; adding/removing a station is a scenario
   edit, not a radio-code edit.
3. **TrajectoryIntegrator**: advances one or more UAM states at the motion
   clock.  It may later incorporate speed limits, vertiport dwell, conflicts,
   and lane/level changes.
4. **RadioModel**: evaluates RSRP/SINR for arbitrary 3-D states.  It must not
   assume that all aircraft share one altitude or lane.
5. **PolicyModel / Controller**: receives observations and returns bounded
   intent/actions.  Handoff hysteresis and time-to-trigger belong here, not in
   the geometric corridor layer.
6. **CapacityModel**: consumes time-indexed states, link quality, policy
   outcomes, and service constraints; it reports flow, occupancy, and
   bottlenecks over an explicit aggregation window.
7. **TraceSink**: writes state, radio, policy, and capacity records with
   timestamps so plots and claims are reproducible.

## 3. Multi-level and multi-lane compatibility

The baseline state contains:

```text
uam_id, corridor_id, s_m, level_id, lane_id,
position (x, y, z), speed_mps, heading_rad, timestamp_s
```

`level_id` and `lane_id` are identifiers, not assumptions about how many
levels/lanes exist.  A future corridor provider can expose a lane graph or a
level graph while preserving the same radio and output contracts.  A
controller can request a lane/level transition; the trajectory integrator
decides whether that request is feasible.

## 4. Reproducibility rules

- Store the complete clock configuration in every run manifest.
- Store the scenario path/version and the stable base-station IDs.
- Keep generated runs outside the source package (`runs/` is ignored); retain
  only the accepted compact baseline under `results/`.
- Never silently convert a policy window into a radio sampling interval.
- Keep the current validated centerline run as a baseline regression case.
