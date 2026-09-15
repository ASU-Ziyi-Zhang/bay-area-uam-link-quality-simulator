# Fixed-stream position response

## Finding: a stream behaves the same alone or inside a multi-stream layout

In the current model, keeping a stream's path, offset, altitude, speed, entry
timing, policy parameters and sampling window unchanged, adding other streams
does not change its radio results or its C/R/F capacity series. This was checked
two ways:

1. adding other aircraft to one radio-kernel batch leaves every radio array of
   the original aircraft element-wise identical;
2. rerunning the −250 m offset, 300 m altitude stream alone reproduces the
   corresponding stream of the 2×2 results in both scenarios, snapshot by
   snapshot.

The 1×1 / 2×1 / 1×2 / 2×2 layouts are therefore examples, not the unit of study.
`evaluate_location`, `evaluate_locations` and `geometry_grid` expose the
position response directly.

## Two layers

**Radio:** for a fixed radio configuration and site layout, SINR = g(s, d, h),
with s the route position, d the local normal offset and h the altitude.
Interference sums over base stations only; there is no aircraft-count, lane-count
or other-aircraft term. The strongest of the three nearest sites serves and the
other two interfere, so a handover region where two sites are similar in power
can produce low SINR even near a site.

**Policy and capacity:** q(t) = F(d, h; stream entry interval, v, Θ, group,
exposure window, policy parameters). It also depends on how aircraft of the
stream are distributed and which weak segments a group covers. An unchanged
radio profile does not imply unchanged group capacity when the entry interval
changes.

Example, full SF–SJ corridor, offset −250 m, altitude 300 m, complete five-aircraft
groups, 5 s sampling:

| Comparison | Per-stream demand | Entry interval | Q0.95 |
|---|---:|---:|---:|
| Stream alone, or the same stream inside 2×2 at the original interval | 112.5 UAM/h | 32 s | 71.466 UAM/h |
| 112.5 UAM/h total split over four streams, stream alone or combined | 28.125 UAM/h | 128 s | 49.758 UAM/h |

Alone and combined are identical within a row. The difference between rows is
group sampling and exposure statistics, not site load or aircraft interference.

## Deliverables

A position function, a two-dimensional offset × altitude sweep and single-factor
slices, with per-stream demand held fixed. Altitudes above 300 m are labelled as
radio-model extrapolations. Estimates are planning capacity under the stated
estimator, not operational capacity.
