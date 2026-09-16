# Step 03 R0021 protocol — TRB-reference traffic on a 3×3 flow lattice

Date: 2026-09-01.

## Status and isolated change

This protocol is approved for execution as a scale test. It retains the R0020
radio model, same-flow moving policy group, exposure/policy map,
policy-spacing ACC, rolling 60+60 s decision, quintic motion, insertion and
3-D engineering safety gates, and nominal-flow recovery. Capacity remains
excluded.

Only two implementation dimensions change:

1. the three R0020 lateral flows are expanded to a fixed 3×3 lattice by adding
   the already tested Step-02 altitude levels; and
2. the 15-aircraft pilot schedule is replaced by the TRB reference traffic
   input.

## Traffic input recovered from the TRB reference case

The TRB reference demand is 112.5 UAM/h. Deterministic corridor entry therefore
uses

```math
h_\lambda=3600/112.5=32\ \mathrm{s}.
```

The half-open 4800 s generation interval produces 150 requests at
`t = 0, 32, ..., 4768 s`. Requests are assigned cyclically across the nine
flows, so only one UAM is requested from the virtual corridor origin at each
32 s release clock. Each flow receives 16 or 17 requests, while aggregate
demand remains 112.5 UAM/h. Applying 112.5 UAM/h independently to every flow
is explicitly excluded.

The dynamic simulation continues after the last request until all admitted
aircraft complete, or until the declared 7200 s safety time limit. Unlike the
TRB steady-state statistical study, this mission-completion run does not
discard a 1600 s warm-up and does not stop airborne UAM at 4800 s.

## 3×3 flow coordinates

The lattice is a new discrete traffic representation, not a geometry claimed
by the TRB paper:

```text
h = 400 m:  d = −300, 0, +300 m
h = 300 m:  d = −300, 0, +300 m
h = 200 m:  d = −300, 0, +300 m
```

The lateral coordinates are frozen from R0020. The altitude coordinates are
the Step-02 tested levels. Logical-flow order for cyclic release is
`(−300,200), (0,200), (+300,200), (−300,300), ..., (+300,400)`.

## Group, control, and recovery rules

- Policy group: focal aircraft plus up to two available predecessors and two
  available followers in the same current logical flow.
- During a maneuver, policy ownership remains with the source flow until
  completion and then switches to the target flow.
- ACC and target-flow insertion use relevant source/target longitudinal
  relationships.
- The all-aircraft Cartesian 3-D engineering screen is independent of policy
  group membership.
- Avoidance requires strict common-horizon policy-cost improvement.
- When away from nominal, immediate lateral, vertical, or diagonal grid
  neighbors that reduce Euclidean transverse distance to nominal are tested.
  A no-worse safe return is prioritized using the frozen R0020 rule.
- Global one-start-per-decision-clock arbitration remains provisional and is
  not claimed to be system-optimal.

## Matched cases and claim boundary

The two cases use identical 150-aircraft requests and all other inputs:

- `acc_no_change`: ACC and group policy, transitions disabled;
- `acc_safe_rolling`: ACC, rolling transitions and nominal recovery enabled.

Required outputs include all requests, deferrals, missions, group membership,
policy, ACC response, transition decisions, nominal-flow state, minimum
sampled separation score and computation time. Results are a scale/mechanism
test, not calibrated safety, capacity, throughput, comfort or optimal-control
evidence.
