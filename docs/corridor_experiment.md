# Corridor experiment: stay at the centre or use a 3 × 3 grid

## Question

For the same demand stream and calibrated policy, does allowing aircraft to move
among nearby lateral and altitude positions raise the policy-conditioned
planning capacity compared with staying on the centreline?

## Setup

- Real `airport_to_airport` corridor (49.5 km), one request every 32 s, 93
  requests, 50 m/s reference cruise, calibrated classifier (Θ −2.0 dB, 2 s
  sampling, 5 s updates, 90 s window, k = 3, 5%/10%), AKS longitudinal control.
- Every request enters at (0 m, 300 m).
- Candidate positions: lateral offset {−300, 0, +300} m × altitude {200, 300,
  400} m. A move goes only to an 8-connected neighbour (at most 300 m lateral and
  100 m vertical) along a quintic profile.

| Case | Moves |
|---|---|
| `fixed_centerline` | none |
| `spatial_grid` | lateral, vertical or diagonal moves between adjacent cells |

Everything else is identical. Grid cells are alternative positions for one
origin stream, not extra lanes: empty cells add no capacity.

## Capacity definition

At every 5 s snapshot all active aircraft are counted once and their policy
spacing targets averaged:

    q(t) = 3600 · v_ref / mean_i S(p_i(t), v_ref)      [UAM/h]

The report gives the mean and the 95%-reliable rate (the value met or exceeded
in about 95% of snapshots) over the compared window, from the first exit to the
last entry, when the corridor is fully populated. This is the policy-conditioned
planning capacity for this demand realisation, not maximum sustainable
throughput.

## Exposure grouping

Each aircraft's policy uses the link exposure of a five-aircraft group. The
neighbourhood group is the aircraft itself plus its four nearest aircraft by
along-track distance, taken from its own cell and the adjacent cells, excluding
aircraft farther than 2 · S_F(cruise) along track
(`group_mode = "neighbourhood"`, `geographic_traffic.neighbourhood_members`). On
one lane this matches the original two-ahead, two-behind group except at the
ends of the stream. The earlier same-cell group left 41.3% of observations in
full groups once aircraft spread over the grid (R0061), against 88.6% with the
neighbourhood group (R0062).

## Result (R0062)

| | Stay at centre | Move within 3 × 3 |
|---|---:|---:|
| C aircraft-time | 62.7% | 73.8% |
| R aircraft-time | 36.6% | 25.9% |
| F aircraft-time | 0.7% | 0.3% |
| Planning rate, mean (UAM/h) | 112.2 | 118.4 (+5.5%) |
| Planning rate, 95% reliable (UAM/h) | 107.7 | 111.8 (+3.8%) |
| Completed moves | 0 | 178 |
| Aircraft held at entry | 54 | 0 |
| Speed reversals per flight | 49.4 | 3.6 |
| Sampled NMAC | none | none |

Compared window 16.5–49.1 min; 93/93 aircraft exited in both cases; independent
verification passed. On the centreline the 25–45 km segment is mostly reactive;
moving to adjacent cells there raises C time. Figures and tables:
[results/R0062-corridor-3x3](../research/dynamic-transitions/results/R0062-corridor-3x3/).

## Provenance

- R0056 used same-cell grouping. After it, a route speed governor was added that
  sampled curvature every 10 m and ignored the distance flown while one command
  is held; R0060 exposed a 4.62 m/s² route-normal acceleration against the
  4.575 m/s² limit and was voided. The governor was rewritten (0.5 m grid,
  braking-feasible speed ahead, held-command bound); the limit is unchanged.
- R0061 reran the R0056 configuration on the corrected code and reproduced its
  centreline case exactly; R0062 is the same code with neighbourhood grouping.
- Protocol: [bay-area-single-stream-capacity-protocol.md](../research/dynamic-transitions/bay-area-single-stream-capacity-protocol.md).

## Limits

One origin stream at one demand level. A requested-arrival-interval sweep is
needed for maximum sustainable capacity. Parallel lanes at one altitude
interfere during lane changes; separating lanes by altitude is the proposed next
step.

## Reproduce

```sh
python scripts/run_bay_area_three_lane_90s.py --config research/dynamic-transitions/configs/bay_area_single_stream_capacity_3x3_neighbourhood.json --output research/dynamic-transitions/runs/<ID>
python scripts/verify_bay_area_single_stream_capacity.py research/dynamic-transitions/runs/<ID>
python scripts/plot_single_stream_briefing.py --run <ID>
```
