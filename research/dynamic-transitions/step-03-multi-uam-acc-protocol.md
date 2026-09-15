# Step 03 protocol — Multi-UAM, multi-lane rolling control with ACC

**Status:** agreed research direction; baseline numerical settings below are
exploratory mechanism-test inputs, not calibrated operating standards.

## 1. Question

When every lateral flow contains multiple UAM, how do local-group
communication policy, policy-dependent ACC, target-lane gaps and the full
three-dimensional traffic screen affect a rolling communication-driven lane
change?

Step 03 does **not** estimate capacity. It reports aircraft motion, policy,
ACC response, accepted/rejected transitions and safety-screen outcomes.

## 2. Baseline traffic layout

- Bay Area airport-to-airport reference corridor and archived BS inventory.
- One cruise level: 300 m.
- Three logical lateral flows: `d = −300, 0, +300 m`.
- Five UAM requested in each flow; 15 aircraft total.
- Fixed requested headway in each flow: 90 s.
- Flow phases: 0, 30 and 60 s. Thus the combined request stream is one UAM
  every 30 s, assigned cyclically across the three flows.
- Every aircraft starts with policy C and uses only its actually available
  communication/group history.

The phase offsets avoid placing three aircraft at the same virtual origin at
the same instant. Requested and realized corridor-entry times are both saved;
any admission deferral is an output rather than a hidden change to demand.

The 90 s per-flow headway is the first sparse-traffic mechanism case. A later
60 s per-flow case with 0/20/40 s phases is reserved as a denser gap-rejection
comparison; it is not required to validate the first implementation.

The candidate-coordinate research resolution remains 100 m, but the first
three **populated** flows are separated by 300 m. This keeps two parallel
flows outside the exploratory 200 m horizontal protection radius before
longitudinal staggering is considered. The earlier internal `−100/0/+100 m`
pilot is retained as R0018: it correctly stopped when the all-aircraft
three-dimensional score fell below one and is not a presentation result.

## 3. Policy group

For focal UAM `i`, the policy group is the focal aircraft plus up to two
available predecessors and two available followers in its current logical
flow. Missing neighbors are not fabricated. During a transition, policy-group
ownership remains with the source flow until completion, then changes to the
target flow.

At each 5 s observation, the group bad-link fraction is

```math
e_i(t_k)=\frac{1}{|G_i(t_k)|}\sum_{j\in G_i(t_k)}
\mathbf 1\{\operatorname{SINR}_j(t_k)<\Theta\}.
```

Exposure is the mean of the available group-fraction history over the previous
30 s. The inherited demonstration thresholds remain `Theta = −1.5 dB`,
`C <= 0.05`, `R <= 0.10`, and `F > 0.10`.

## 4. Longitudinal ACC

ACC is active in both matched cases. The policy-dependent spacing law is the
ACC target:

```math
S_m(v)=200+\tau_m v+0.167v^2,
\qquad (\tau_C,\tau_R,\tau_F)=(15,30,60)\ \mathrm{s}.
```

The exploratory feedback is

```math
a_{\rm raw}=k_v(v_0-v)+k_g(g-S_m(v))+k_{\Delta v}(v_L-v),
```

with inherited gains `k_v = 0.2 s^-1`, `k_g = 0.005 s^-2`, and
`k_Delta-v = 0.4 s^-1`. The executed command is bounded by
`−2 <= a <= +1.5 m/s^2`. A policy downgrade changes the target spacing; it
does not instantaneously change the physical gap. ACC removes the error over
time with the bounded command.

During a maneuver, the aircraft responds conservatively to relevant source-
and target-flow predecessors. A target-flow follower is checked for insertion
and forecast disturbance but is not mislabeled as the transitioning
aircraft's leader.

## 5. Rolling transition decision

- Recompute every 5 s.
- Forecast the no-change branch for a 60 s warning horizon.
- Search only adjacent lateral flows when a C→R/F or R→F degradation is
  forecast.
- Evaluate stay and candidate branches over the same additional 60 s, giving
  a 120 s common interval.
- Use the same policy loss as Step 02: `l(C)=0`, `l(R)=1`, `l(F)=2`.
- Require at least 10 policy-cost seconds of improvement in the first mechanism
  run; this explicit anti-chatter threshold is uncalibrated and must later be
  included in sensitivity analysis.
- Execute an accepted motion with the same quintic boundary trajectory and
  current motion/curvature limits.

The candidate must improve the focal policy cost and pass every safety gate.
Communication benefit never overrides the safety screen.

## 6. Safety and traffic-interaction gates

The following are different checks and must be logged separately:

1. **Inherited source-flow spacing deficit:** retained in the ACC forecast; it
   is not an automatic lane-change rejection.
2. **New insertion relationship:** a newly created target predecessor/follower
   relationship must pass the declared policy-spacing admission rule.
3. **Induced ACC disturbance:** candidate and stay forecasts are compared;
   executed acceleration is bounded, and any optional voluntary-braking gate
   applies only to additional candidate-induced braking.
4. **Three-dimensional trajectory screen:** all airborne aircraft are checked,
   regardless of policy-group membership, throughout the forecast and realized
   maneuver. The initial implementation is an engineering screen, not an
   aviation separation certificate.

On the curved corridor, radio and conflict checks use actual Cartesian
positions. Longitudinal ACC gaps use a documented physical along-flow
coordinate, not raw Euclidean distance or an uncorrected polyline index.

## 7. Matched cases

| Case | ACC | Rolling lane changes | Purpose |
|---|---|---|---|
| `acc_no_change` | on | off | Multi-UAM reference with identical releases and policy loop |
| `acc_safe_rolling` | on | on | Adds communication search, insertion/ACC and 3-D safety gates |

No fixed-cruise or capacity case is required for the initial Step-03 meeting
example. Those can be added later as separate attribution/sensitivity studies.

## 8. Required outputs

- requested release, realized release and corridor-entry times;
- trajectory, lane, SINR, group membership, exposure and C/R/F policy per UAM;
- ACC leader, target/actual gap, raw command and executed acceleration;
- every candidate target and its rejection/acceptance reason;
- minimum predicted and realized three-dimensional separation score;
- number and duration of accepted/completed transitions;
- minimum speed, accumulated speed/progress loss and mission time;
- before/after C/R/F time by aircraft and fleet-wide aircraft-time.

The output schema must explicitly set all capacity and throughput fields to
`null` or omit them with a declared `capacity_not_estimated` status.

## 9. Verification before presentation

1. With transitions disabled, both matched cases must be byte-for-byte or
   tolerance-identical in entries, radio, group policy and ACC state.
2. Every group must contain only same-flow available aircraft and no more than
   five members.
3. ACC commands must equal independently recomputed bounded feedback.
4. Every accepted maneuver must be on the declared grid, quintic-continuous
   and preceded by a forecast policy improvement.
5. Every rejected maneuver must retain a machine-readable reason.
6. All accepted and realized trajectories must pass the declared 3-D screen.
7. Aircraft counts must be conserved through requests, deferrals, corridor
   entry and completion.
8. No result may be described as capacity, certified separation, calibrated
   comfort or operational benefit.
