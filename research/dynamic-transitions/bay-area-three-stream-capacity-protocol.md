# Bay Area three-stream lateral-control capacity experiment

Date frozen: 2026-09-13

## Question

For three continuously released, same-altitude origin streams, does allowing
safe adjacent lateral lane changing improve the policy-conditioned capacity of
the complete corridor relative to keeping every aircraft in its origin lane?

## Matched design

Three physical lanes are located at lateral offsets `{-300, 0, 300} m`, all at
`h=300 m`. Each lane releases one independent stream at a requested 32 s
headway during the same 20 min release period. The three streams have zero phase
offset, so corresponding releases are simultaneous and horizontally separated
by 300 m. Both cases use the same requested releases, radio field, policy
parameters, AKS controller, motion envelope, and safety rules.

1. `three_stream_fixed_lanes`: longitudinal AKS control only; every aircraft
   remains in its origin lane.
2. `three_stream_lateral_control`: the same three streams, with adjacent
   lateral lane changes enabled. Height changes are disabled.

The only treatment variable is lateral-control availability.

## Fixed policy and control parameters

The calibrated configuration is retained: 2 s radio sampling, 5 s policy and
decision clocks, 90 s assessment window, `k=3`, `Theta=-2.0 dB`, 5%/10% C/R
exposure thresholds, and fixed policy spacings of 1319.9/2069.9/3569.9 m at a
50 m/s reference speed.

## Capacity definition

Aircraft keep their origin-stream identity even after changing physical lane.
For origin stream `j`, at each 5 s complete snapshot,

`q_j(t) = 3600 v_ref / mean(S_policy,j(t))`.

The corridor rate is

`Q_corridor(t) = q_1(t) + q_2(t) + q_3(t)`.

Only snapshots containing at least one active aircraft from every origin stream
are used for the aggregate mean, median, and 95%-reliability lower-tail rate.
This prevents warm-up or drain emptiness from being treated as zero capacity and
prevents empty candidate lanes from contributing invented capacity. Served exit
rate remains a demand-realization outcome, not a maximum sustainable capacity.

## Acceptance checks

Both cases must clear the same requested stream without execution rejection or
sampled NMAC. The no-change case must remain on the assigned lane. Every accepted
move in the treatment case must be one adjacent lateral lane, must complete, and
must satisfy the archived separation, speed, and motion-envelope checks. The
capacity archive must reproduce independently from the realized policy trace and
the immutable origin assignment.

The longitudinal controller applies a forward curvature-speed governor derived
from the same route-normal acceleration limit used by the execution audit. This
prevents a spacing-recovery acceleration from entering a downstream curve faster
than the aircraft can decelerate safely; the rule is identical in both cases.
