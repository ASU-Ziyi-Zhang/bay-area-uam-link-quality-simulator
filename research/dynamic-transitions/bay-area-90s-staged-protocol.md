# Bay Area staged motion-control study with a fixed 90 s assessment window

Date frozen: 2026-09-12

## Approved working point

The user selected a 90 s assessment window for the next Bay Area motion-control
experiments. The remaining policy parameters stay fixed: `Theta=-2.0 dB`, 2 s
radio sampling, 5 s policy updates, 5%/10% C/R exposure tolerances, a focal group
of up to two neighbours on either side, and `k=3` persistence.

Traffic is indexed by requested arrival interval and demand duration. The number
of generated requests is a derived run record, not the experiment label.

## Ordered stages

### 1. One lane, one altitude, longitudinal control

Use the real `airport_to_airport` route, one lane at 300 m, a 32 s global
requested-arrival interval and the existing 2972.56 s demand horizon. Disable
lateral and vertical changes. Rerun the full stream because the previous R0040
baseline used a 30 s window and cannot serve as the matched comparator for later
90 s experiments.

Report policy shares and transitions, raw-policy switches, acceleration/deceleration
direction reversals, target-gap settling, KS2 versus feedback-fallback time,
admission delay, completion, speed/envelope limits and sampled safety.

### 2. Three lateral lanes at one altitude

After Stage 1 is independently verified, create paired runs with three lanes at
300 m and the same global arrival schedule and radio field:

1. longitudinal-only stay control;
2. longitudinal plus lane-changing control.

The principal demand convention is one global request every 32 s assigned
deterministically across the three lanes, so total demand remains 112.5 UAM/h.
This implies a 96 s nominal request interval per lane. A separate case with 32 s
per lane would triple total demand to 337.5 UAM/h and must be labelled as a stress
test, not mixed with the principal comparison.

Use lateral offsets -300/0/+300 m, altitude 300 m, the confirmed NMAC-anchored
spacing targets and the September lane-change gates: a move must pass physical
admission and produce a benefit above numerical/prediction uncertainty relative
to the longitudinal-only stay forecast. Record accepted/completed changes,
rejection reasons, source/target gaps, reversals, policy benefit, entry delay,
completion and sampled safety.

### 3. Multiple lateral lanes and altitude levels

Only after the same-altitude paired comparison passes, extend the fixed-flow
layout to three lateral lanes and multiple heights. First test fixed levels with
no vertical transitions; then enable vertical and joint moves. Keep the same
demand convention and add three-dimensional group ownership, arbitration,
vertical separation and joint-envelope checks.

## Confirmation rule

The 90 s policy decision and this ordered sequence may be copied to `confirmed/`
immediately because they were explicitly selected by the user. Numerical run
results enter a later immutable confirmed revision only after the relevant run
and independent validation complete. Failed or exploratory runs remain only in
the numbered run archive.
