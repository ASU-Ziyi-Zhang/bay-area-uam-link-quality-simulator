# Bay Area single-stream conditional-capacity experiment

Date frozen: 2026-09-13

## Question

For the same center-entry demand stream and calibrated policy parameters, does
allowing safe spatial reconfiguration on a small lateral-by-altitude grid raise
the TRB-style policy-conditioned planning capacity relative to staying on the
centerline?

## Fixed inputs

Both cases use the real `airport_to_airport` corridor, one request every 32 s,
the same 93-request schedule, 50 m/s reference cruise, 2 s radio sampling,
5 s policy updates, a 90 s exposure window, `k=3`, `Theta=-2.0 dB`, and 5%/10%
C/R exposure thresholds. Every request enters at `(d,h)=(0,300) m`.

The candidate lattice is `d={-300,0,300} m` by `h={200,300,400} m`. A maneuver
may move only to an 8-connected neighboring cell. Consequently one action is at
most 300 m lateral and 100 m vertical; direct multi-cell jumps are excluded.
The 300 m and 100 m grid spacings exceed the modeled NMAC event dimensions of
152.4 m horizontal and 30.48 m vertical at the cell centers. This does not make
a maneuver automatically safe: insertion, all-aircraft separation, motion
envelope, speed, and complete execution checks still apply.

## Matched cases

1. `fixed_centerline`: AKS longitudinal control only; no spatial move.
2. `spatial_grid`: the same stream and controller with lateral, vertical, and
   diagonal quintic moves among adjacent grid cells.

## Capacity definition

At every 5 s snapshot, all active aircraft from the one origin stream are
counted exactly once. Their policy targets are averaged and translated by

`q_mix(t) = 3600 v_ref / mean(S_policy(t))`.

Report the mean, median, and the existing 95%-reliability lower-tail order
statistic. Empty grid cells are candidate spatial states, not extra lanes, and
contribute no additive capacity. Exit rate is reported separately as served
throughput for the tested demand, not maximum sustainable capacity.

## Acceptance and interpretation

Both cases must complete the same request schedule without a sampled NMAC or an
execution rejection. Every selected endpoint must be an adjacent grid cell and
each aircraft must appear once per capacity snapshot. The comparison estimates
conditional planning-capacity change for this demand realization. A later
requested-arrival-interval sweep is required to establish maximum sustainable
operational capacity.

## Exposure grouping (from R0060)

R0056 formed each aircraft's exposure group inside its own grid cell: two
aircraft ahead and two behind by index. Once aircraft spread over the grid this
leaves many groups of one to four. The spatial case had 40.5% five-aircraft
observations against 83.6% on the centerline, and F was 12% of singleton
observations. The classifier was calibrated on five-aircraft groups, so the two
cases were classified under different evidence.

From R0060 the group is `group_mode = "neighbourhood"`:

1. Candidates are aircraft in the focal cell or an adjacent cell, within one
   grid step laterally and vertically at their flown offset and height. The
   grid step is read from the flow coordinates.
2. The group is the focal aircraft and its `2 * neighbors_each_side` nearest
   candidates by along-track distance, with no ahead/behind quota.
3. Candidates farther along track than `neighbors_each_side * S_F(v_ref)` are
   excluded. With an all-F stream this is exactly two aircraft each side, so
   the cap never removes a member of the calibrated group, and it follows any
   change to d0, tau_F or cruise speed.

Threshold, window, 5%/10% limits and `k = 3` are unchanged. The spacing
obligation stays with the follower: an aircraft with no aircraft ahead cruises
whatever its policy. The policy targets are no longer listed in the
configuration; they follow from `S = d0 + tau v_ref + b v_ref^2`.

## Route speed governor and the matched reruns (R0060 to R0062)

After R0056 the dispatcher gained a route speed governor for the 3×1 study. It
read curvature every 10 m and did not account for the distance flown while one
acceleration command is held. R0060 (neighbourhood grouping) exposed this: in
`fixed_centerline` UAM-72 closed a gap at 66.8 m/s towards an apex at q = 8422 m
whose curve speed is 66.85 m/s, was allowed +0.24 and then +0.55 m/s², and
reached 4.62 m/s² route-normal acceleration against the 4.575 m/s² limit. The
run stopped at 2558.5 s with 47 of 93 aircraft through. `spatial_grid` was
stopped at 640 s simulated time. R0060 is not used for results.

The governor now reads curvature on a 0.5 m grid, bounds each node by its
neighbours, forms the braking-feasible speed V(x) ahead, and limits the held
command so that `v^2 + 2 a x <= V(x)^2` at every point reached before the next
command. Replaying the R0060 states, the old governor reaches 4.62 m/s² and the
new one 4.57 m/s². The limit itself is unchanged.

The unfixed governor changed the centerline case (61.6/37.6/0.8% C/R/F against
63.2/36.0/0.8% in R0056). Both groupings are rerun on the corrected code:

- R0061: lane-order grouping (the R0056 configuration, unchanged). Its
  centerline case reproduces R0056: 63.18/36.00/0.82% C/R/F, mean 113.59 and
  q95 94.28 UAM/h.
- R0062: neighbourhood grouping. Centerline case 62.74/36.56/0.70% C/R/F, mean
  114.17 and q95 96.50 UAM/h, 93/93 exited, no sampled NMAC.
