# Bay Area sparse-flow assessment-window sweep

Date frozen: 2026-09-12

## Question

When the departure interval is long enough that a focal policy group frequently
contains only one or a few UAMs, does a longer exposure window reduce operational
policy chatter and longitudinal acceleration/deceleration reversals?

This is a focused diagnostic of the earlier short-stream longitudinal result. It
does not tune the SINR threshold, exposure tolerances, persistence or controller.

## Matched cases

Two requested-arrival intervals are used:

- 32 s: reproduces the earlier short-stream setting;
- 90 s: initially places consecutive aircraft farther apart than the 3569.9 m
  policy-group radius at 50 m/s, producing substantially more singleton/small-group
  observations.

For each interval, assess windows of 30, 60, 90 and 120 s are compared. The
existing R0039 archive is the 32 s / 30 s case; all other cells receive new run IDs.
Each finite diagnostic contains the same five scheduled requests. Request count is
held fixed only to make the cases matched; departure interval is the experimental
traffic variable.

## Frozen parameters

- Real `airport_to_airport` route and retained base stations;
- one lane at 300 m, no lateral or vertical change;
- `Theta=-2.0 dB`, 2 s radio samples, 5 s policy updates;
- 5%/10% C/R bad exposure tolerances and `k=3` persistence;
- focal UAM plus up to two neighbours per side within 3569.9 m;
- C/R/F target gaps 1319.9/2069.9/3569.9 m;
- speed 30–80 m/s and longitudinal acceleration -2.0 to +1.5 m/s2;
- identical safety and numerical checks to R0039.

## Primary outputs

For every case report C/R/F aircraft-time, raw and operational policy switches,
longitudinal speed-direction reversals, applicable spacing tasks, fraction settled
before the next policy change, entry delay, group-size observation distribution,
completion and sampled safety.

The selected window must not be the one with the lowest F share alone. It should
reduce operational switching and speed reversals without excessive detection or
recovery memory, loss of mission completion or safety failure.

## Interpretation boundary

These cases diagnose finite sparse streams. They are not capacity estimates and
do not authorize lane changing. A later multi-lane experiment must be indexed by
global and per-lane departure intervals; the realized aircraft count is derived
from the fixed demand horizon.
