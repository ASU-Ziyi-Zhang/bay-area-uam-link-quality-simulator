# Two-UAM longitudinal mechanism checks

2026-09-10. User authorised execution in the current task. This is the first
two-aircraft step of the parameter-normalisation work, not a 15-aircraft AKS
integration run. It supplements the older AKS protocol whose unconditional
2250 m policy-transition premise does not apply to actual oversized gaps.

## Scope and controls

Straight one-dimensional route, common altitude, prescribed leader, simulated
follower. No radio, lateral change, capacity or certification claim. Policy is
an explicit input so motion can be isolated. Compare the existing geographic
ACC function against an opt-in KS2 reference tracker. The latter plans from
actual gap; minimum-spacing mode holds when the gap is already sufficient;
exact-target mode may close excess gap. Keep baseline free-speed logic intact.
If a leader departs from the constant-speed assumption, invalidate the reference
and use observed-state ACC; do not silently continue the constant-leader plan.
This fallback is an integration-boundary test, not a new collision-avoidance proof.

Inputs have four blocks: vehicle bounds, spacing law, controller gains and
numerics. The resolver creates one immutable configuration and an adapter for
the existing ACC. No action duration or phase is independently configured.
Spacing target is evaluated at nominal cruise for the KS2 endpoint; baseline
ACC retains its original instantaneous-speed target. Record both definitions.

## Prespecified cases

1. 4500 m actual gap, F target, 50 m/s pair: minimum-spacing request must Hold.
2. Actual C target to F target: automatically solve opening displacement/time.
3. Actual F target to C target: exact closing with 60 m/s maximum must finish;
   the same request with 50 m/s maximum must be refused and use ACC fallback.
4. Initially sufficient gap, leader brakes from 50 to 30 m/s at -1 m/s^2
   beginning at 20 s: current-state response must occur even without a policy
   change. Also perturb a leader during an active opening reference to exercise
   invalidation.
5. Repeat opening with minimum speed 30 m/s; repeat with reference jerk limit
   0.05 m/s^3. These and the 60 m/s maximum are declared synthetic bounds,
   not calibrated aircraft capabilities. Zero minimum speed reproduces the
   mathematical legacy bound only.

## Acceptance and outputs

Execute each case/controller at dt=0.5, 0.25 and 0.125 s. Split steps at reference
phase/end and leader events; integrate actual follower state, never replace it
with the analytic trajectory. Use reference feedforward plus gap/speed feedback.
Execution acceleration and speed are bounded; jerk limit is a reference bound,
not a proven bound for saturated feedback or fallback.

Check independently from saved traces: displacement trapezoid versus positions,
finite differences versus reported acceleration, speed/acceleration limits,
positive gap and exploratory 200 m floor, terminal gap/speed and reference
closure, exact event consistency. Finite sampling is not a continuous safety proof.
Before running set: terminal error <=1 m and 0.05 m/s for feasible constant-leader
KS2 cases; numerical projection <=1e-5 m/s; pairwise step-refinement endpoint
gap difference <=0.1 m and maximum gap curve difference <=0.2 m. Saved-trace
quadrature tolerance 0.3 m and finite-difference acceleration tolerance 0.05 m/s^2
(event intervals excluded from the latter). These are mechanism-test criteria,
not inferred operational standards. Do not loosen after observing failures.
Baselines and variable-leader fallbacks report residuals without claiming exact
KS2 closure. No-motion case has <=1e-6 m/s speed change. Reference refusal and
invalidation must be explicit. Controller comparison is descriptive, not an
AKS superiority claim: no alternative smooth-reference comparison yet.

Archive resolved inputs, source snapshot, environment, traces, checks and
summary. Mark runs draft because scientific source has not been committed;
source hash/snapshot enables inspection without claiming formal release gates.
No old traffic run is overwritten. Formal full-corridor reproduction and
single-aircraft R0023 audits remain separate work.

## Amendments

The prespecified text above is left as written. Each amendment below records what
changed after the protocol was fixed, when, and on what evidence. No acceptance
threshold has been loosened in any of them.

**A1 — 2026-09-10, step sizes.** Prespecified dt = 0.5 / 0.25 / 0.125 s becomes
0.125 / 0.0625 / 0.03125 s. Evidence: R0024 failed the prespecified saved-trace
`independent_acceleration_difference` gate at the two coarse steps, on nine
traces, near saturation exits. The gate is unchanged; the discretisation was
refined until the archived traces meet it. Recorded in the config as
`numerics.refinement_reason`.

**A2 — 2026-09-10, speed envelope.** The maximum speed of 60 m/s in cases 3 and 5
becomes 80 m/s, and the minimum speed of 30 m/s becomes the nominal rather than a
variant, with zero retained as the explicit legacy contrast. Both bounds are now
sourced in the config's `provenance` block: 30 m/s is 1.2 x Vstall for a
low-wing-loading eVTOL, below which the aircraft is no longer wing-borne; 80 m/s
is 156 kt, below the 170 kt maximum cruise reported for the Joby S4. They remain
declared bounds, not calibrated capabilities for a specific airframe. The 50 m/s
refusal case in item 3 is unchanged and still exercises the infeasible junction.

**A3 — 2026-09-10, what the run records.** The single event log becomes three
fields, because one list was being asked to do three incompatible jobs:

* `events` — the decision log. Discrete content and grid times only. This is what
  the step-refinement gate compares for exact equality, and that comparison is
  now well posed: two runs at different steps produce identical lists unless they
  genuinely decided something different.
* `discontinuities` — times at which the recorded acceleration jumps, used only to
  choose which interval the finite-difference audit skips. Solved times, such as a
  bisected speed-bound contact, belong here: they carry the integrator's own
  truncation error and cannot compare equal across steps.
* `diagnostics` — measured continuous quantities, such as the gap at which a
  transition froze. Reported, never compared for equality.

The protocol sentence "event intervals excluded from the latter" is therefore
read as "declared discontinuity intervals excluded". One line of
`scripts/verify_two_uam_longitudinal.py` changed accordingly. The excluded set is
the same or smaller than before, so the audit is not weaker. Evidence: R0026
through R0029, four successive failures converging on this cause.

**A4 — 2026-09-10, one envelope for both controllers.** `resolve()` now passes the
speed bounds into the baseline configuration as well. Before this the baseline
was capped at cruise while the tracker was not, so any comparison between them
would have been a statement about a bound rather than about a controller.
Evidence: R0030, which passed validation but is not usable for comparison for
exactly this reason, and is registered as such.

**A5 — 2026-09-10, the complete policy matrix.** Prespecified items 2 and 3 cover
only C→F and F→C. Four cases are added so all six ordered pairs of {C, R, F} are
run: `open_c_to_r` (+750 m), `open_r_to_f` (+1500 m), `close_r_to_c` (−750 m) and
`close_f_to_r` (−1500 m). Opening cases use the `minimum_spacing` objective and
closing cases `exact_target`, matching the existing convention. Reason: two
transitions cannot separate the direction of a transition from its size, and the
asymmetry found in R0031 needed exactly that separation. No acceptance limit
changed; the four new cases are checked by the same gates as the existing ones.
Recorded in the config as `numerics.policy_matrix_reason`.

**A6 — 2026-09-10, fair recovery baseline (R0033).** User explicitly requests
that ACC and AKS both restore established following gaps after policy recovery,
then regenerate the figures and update Confirmed. A shared 30–80 m/s envelope
alone did not achieve this in R0032: the nominal-cruise term still vetoed positive
following acceleration. For an explicitly established follower–leader pair,
`acc_controls(..., recovery_pairs=...)` now uses the minimum of the occupied-lane
following commands, without the nominal-cruise term. All other leader constraints,
acceleration/deceleration limits and speed bounds remain. Without a matching
relationship the original cruise-limited rule remains, so an arbitrary sufficient
gap does not initiate pursuit. The callable is pure; callers supply relationship
state. The pair harness binds its prescribed existing leader from case metadata;
full-corridor automatic engagement, handover and release are not implemented by
this amendment. `acc_policy_recovery` is an experiment version switch, not a
calibrated gain. Old configs default to false and old archives are untouched.
Both methods use exact-target recovery for established policy-transition fixtures;
sufficient-gap fixtures retain minimum-spacing/hold. Gains are unchanged.
The pair initializer also now uses named position, speed and lane arguments;
previously both fixtures accidentally occupied lane 50 rather than lane 0.

**A7 — 2026-09-10, observation and evidence (R0033).** Observe both methods until
the reference duration plus 1500 s, while also reporting gap errors at the
reference endpoint and the historical endpoint (reference duration plus 120 s).
This is an observation extension, not an action duration or a relaxed tolerance.
At each recorded sample, equilibrium requires absolute cruise-target gap error
<= 1 m AND absolute cruise-speed error <= 0.05 m/s. Report the first sample after
which both conditions hold for the remaining saved trace; this is finite-horizon
sampled settling, not a proof of asymptotic or continuous-time stability. Keep all
old numerical tolerances and three time steps. Add independent reconstruction of
ACC commands, relationship-gate checks, and terminal/positive-closing checks for
feasible constant-leader recovery cases. Changing-leader variants report residuals
and invalidation, not nominal recovery success. R0032's finite-time opening
residuals did not establish a permanent deficit; that interpretation is superseded.
This run does not simulate a complete geographic fallback traversal or capacity.
