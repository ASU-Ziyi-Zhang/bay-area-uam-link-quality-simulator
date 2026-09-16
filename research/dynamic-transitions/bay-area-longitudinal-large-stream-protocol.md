# Real-route longitudinal control with the GitHub traffic stream

Date: 2026-09-12. Planned run: R0040.

## Question

What happens when the calibrated C/R/F policy drives longitudinal AKS under the
same 112.5 UAM/h arrival stream used by the published GitHub dashboard, while
lateral and vertical changes remain disabled?

This is a longitudinal closed-loop test, not a lane-change or capacity result.

## Frozen inputs

- Real `airport_to_airport` route and its 12 retained base-station sites.
- 93 scheduled aircraft, one every 32 s, matching the published traffic input.
- One lane at 300 m.
- `Theta=-2.0 dB`; radio 2 s; policy update 5 s; window 30 s; exposure
  5%/10%; maximum local group 5; persistence `k=3`.
- Group selection follows the GitHub definition: up to two adjacent active
  aircraft ahead and behind, without an additional distance cutoff.
- Longitudinal C/R/F targets 1319.9/2069.9/3569.9 m; speed range 30–80 m/s;
  acceleration range -2.0 to +1.5 m/s².
- No lateral or level change.

The simulation continues to 4800 s so every scheduled aircraft can be checked
for completion. A separate common-horizon statistic stops at 2972.56 s, the
published GitHub baseline horizon, so policy observations can be compared on the
same traffic-arrival interval.

## Comparators and outputs

The primary comparator is the published fixed-speed GitHub run: 93 entrants,
50 m/s, no longitudinal feedback, C/R/F shares 68.77%/28.40%/2.83% from 15,445
policy observations. R0039 is retained as a five-aircraft diagnostic and is not
the matched primary comparator.

Report:

1. common-horizon C/R/F policy-update shares and observation count;
2. full-run C/R/F aircraft-time and switches per flight;
3. group-size distribution;
4. entry delay, mission completion and final active/pending aircraft;
5. longitudinal-task dwell, error improvement and settling;
6. speed/envelope bounds, minimum sampled separation and sampled NMAC;
7. KS2 tracker versus feedback-fallback aircraft-time.

## Gate

The run passes the software/safety gate only if all 93 aircraft complete by
4800 s, no execution rejection or sampled NMAC occurs, all speed and envelope
bounds pass, no lateral motion occurs, and the independent verifier reproduces
the 2 s/5 s clocks and `k=3` persistence. Longitudinal settling and differences
from the fixed-speed policy mix are measured outcomes, not forced pass values.

R0040 is immutable whether it passes or fails. No threshold, exposure, group,
controller or acceptance parameter will be changed after inspecting its result.
