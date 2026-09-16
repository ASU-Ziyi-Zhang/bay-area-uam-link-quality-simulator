# Protocol — AKS longitudinal reference, with two-layer separation reporting

Date: 2026-09-09, revised to merge the separation-screening protocol. Draft,
**not authorised to run**. The run number is assigned from the registry at run
time and is not fixed here.

Two changes are tested in one run because they do not interact. The longitudinal
reference **changes the controller**. The separation layer is **report-only** and
changes nothing, so carrying it across every longitudinal case is orthogonal
reporting rather than a confound, and it answers an extra question: does a
different longitudinal reference change separation outcomes?

`separation-screening-protocol.md` is superseded by this file.

## Part A — the longitudinal reference

### The structural fact that determines the design

When the policy changes at cruise, the aircraft does not change speed. The time
gap changes, so the target spacing changes:

    S(policy, v) = d0 + tau(policy) v + buffer v^2

At `v = 50` m/s with the current parameters:

| Policy | tau (s) | Target spacing (m) |
|---|---|---|
| C | 15 | 1367.5 |
| R | 30 | 2117.5 |
| F | 60 | 3617.5 |

A C to F transition requires **`delta S = 2250` m with `delta v = 0`**.

In the AKS family, KS1 and KS1A are single-phase transitions between two endpoint
speeds. When `v1 = v2` they reduce to constant speed and close only `v1 T`; they
cannot produce a different displacement. KS2 supplies a temporary speed excursion
through a junction speed and can.

**The policy transition is a KS2 problem by construction, not by selection.** KS1
and KS1A become relevant only where a policy change coincides with a change of
cruise speed. This is a structural consequence of `delta v = 0`, and the report
must present it that way rather than as an empirical choice among three members.

Order of magnitude: opening 2250 m over duration `T` requires a mean speed
deficit of `2250/T`. `T = 300` s implies 7.5 m/s; `T = 150` s implies 15 m/s. The
junction depth and peak deceleration follow from `T`, and that deceleration
competes with lateral authority through the coupled envelope.

### Question

Does an analytical AKS reference, tracked by the existing feedback law, improve
the execution of a policy-spacing transition relative to the current step target,
and at what cost in mission time and speed excursion?

The reference form under test:

    a_cmd(t) = a_AKS(t) + k_s [s(t) - s_ref(t)] + k_v [v_AKS(t) - v_F(t)]

The speed error is taken against the **planned reference speed**, not against the
leader. That is what makes it a tracker rather than a follower and is the
substantive difference from the current law.

### Why the Step 01 base cannot be reused

Step 01 has one aircraft and `a_lon = 0`: no leader, no spacing to close, no ACC.
The smallest audited base with traffic and the policy-spacing ACC is R0019,
fifteen aircraft on three populated lateral flows. R0021 is not used; its
multi-aircraft controller does not fully implement the agreed group exposure and
neighbour non-degradation constraints, so its outputs are a defect diagnostic
rather than a valid base.

## Part B — the separation layer, report-only

### Event layer

Using the quantitative convention for safety modelling, a near mid-air collision
requires horizontal and vertical proximity **at the same instant**:

    exists t:  r_h(t) <= 152.4 m  and  |dz(t)| <= 30.48 m

Taking the horizontal minimum at one time and the vertical minimum at another and
combining them would misreport the event. The FAA reporting definition is a
separate and broader notion and is not used.

### Predictive layer

Following the structure used by NASA's DAIDALUS implementation, the horizontal
test first asks whether the current horizontal range is already inside the
threshold, and only otherwise evaluates a predicted closest approach with a
modified tau. Two aircraft flying parallel at constant velocity are therefore
**not** exempt because their range rate is zero: if they are already inside the
distance threshold they are in violation. The vertical test is analogous over a
look-ahead, and a violation requires both.

Thresholds are **research assumptions for a cooperative, same-direction,
automated corridor**, deliberately not the DO-365 Phase 1 values, which were
sized to encompass TCAS II advisory thresholds for encounters including piloted
traffic and whose 4000 ft horizontal threshold far exceeds the 300 m lane spacing
here. The run reports a small grid of candidate thresholds rather than asserting
one.

### Report-only means report-only

The existing 200 m by 100 m ellipsoid alone continues to govern admissibility. No
decision, gain, gap rule or criterion changes. The two criteria must be
comparable on identical trajectories before either influences a result. Any run
that lets a new criterion alter a decision is a separate stage with its own
number.

## Controls

| ID | Longitudinal reference | Envelope | Separation reporting |
|---|---|---|---|
| `L0_archived` | current step target plus feedback | off | on |
| `L1_minjerk` | minimum-jerk transition, same endpoints | off | on |
| `L2_aks_ks2` | AKS KS2 reference plus feedback | off | on |
| `L3_aks_ks2_coupled` | AKS KS2 reference plus feedback | **on** | on |

`L1` exists so any `L2` advantage is measured against a smooth analytical
reference rather than only against a step. A step target is an easy baseline and
beating it would not establish that the AKS structure specifically helps.

`L3` is the **first stage in which the longitudinal half of the coupled envelope
is exercised at all**. Everything before it had `a_lon = 0`.

## Acceptance criteria, declared before the run

1. **`L0` must reproduce the archived R0019 record for record**: fifteen aircraft
   completed, twenty-eight lane changes, 832 gap-candidate rejections, identical
   accepted decision times. Because `L0` also has separation reporting enabled,
   this single gate simultaneously establishes that the controller is unchanged
   **and** that reporting does not perturb motion. A mismatch stops the run and is
   reported as a defect, not as a result.
2. **Config self-consistency.** Every limit that governs executed motion is
   declared once. Where an envelope is active, the case's `parameters` block
   carries the same limits as its envelope. This criterion exists because a
   previous run declared `lateral_accel_limit_mps2` twice with different values
   and was correctly refused by the independent verifier with
   `component limits exceeded`.
3. **The run must pass `verify_geographic_traffic_study.py` before it is
   registered.** Registration follows validation, not the run.
4. The ellipsoid is verified to contain the NMAC cylinder before the run by
   evaluating the cylinder corner, `(152.4/200)^2 + (30.48/100)^2 = 0.674 < 1`.
   Containment on each axis separately would not establish this.
5. Every case reports peak deceleration, minimum longitudinal speed, and, where
   the envelope is active, the maximum coupled utilisation and which axis drove it.

## Metrics

**Per policy transition:** requested `delta S`, terminal spacing error, transition
duration, KS2 junction speed and phase split, minimum beta-Safety margin along the
path, peak and mean deceleration, `J_a` and `J_j`, coupled envelope utilisation,
and transitions for which no feasible reference existed.

**Per run:** completed flights, policy time shares, fallback aircraft-time, total
mission aircraft-time, minimum longitudinal speed across the fleet.

**Separation, all cases:** NMAC events with the instant, horizontal range and
vertical separation at that instant; minimum cylinder coordinate and where it
occurs; predictive violations with modified tau and vertical entry time at first
alert; lead time between a predictive alert and any subsequent event. Cross-
tabulated against the existing screens: how many target-gap rejections would also
have been rejected by NMAC, by the predictive layer, by both, or by neither.

The expectation is that **NMAC almost never binds** at this density. The
policy-dependent target gap at 50 m/s is 1367.5 m for C, 2117.5 m for R and
3617.5 m for F, nine to twenty-four times the 152.4 m NMAC horizontal threshold.
Confirming that is the point: a metric that never fires is a floor, not a
constraint, and the report should say so plainly rather than presenting zero
violations as a safety result.

The cost side is reported for every case, not only the benefit side. A reference
that closes spacing faster while depressing fleet speed has moved the problem
rather than solved it, and the R0021 experience shows that has to be visible.

## Model boundaries

* The AKS reference and the beta-Safety spacing relation come from work in
  preparation. This run tests them as a control structure in this simulator; it
  does not validate that manuscript and must not be presented as doing so.
* `tau = 15 / 30 / 60` s are target time gaps, not measured communication response
  times, and must not be relabelled as such.
* `buffer = 0.167` s^2/m is an inherited planning value. Reading it as a
  braking-asymmetry or reserve coefficient is an assumption under discussion, not
  an established derivation.
* The coupling ellipse remains an assumption. `L3` tests its consequences, not its
  validity.
* No actuator lag, sensor delay, string-stability analysis or gain calibration.
  Existing gains are exploratory.
* Sampled trajectories are a check, not a continuous-time proof; the sample step is
  reported and a refinement is run at half the step.
* No capacity estimate, no separation standard claim, no operational concept. The
  AFR lateral separation in the notional tables is explicitly undefined and this
  run does not propose a value for it.

## Sequence

1. Minimal leader-follower drawn from the same scenario and controller: `L0` and
   `L2` only, to establish whether KS2 closes the required displacement at all and
   at what speed cost, without flow interaction.
2. Add `L1` and `L3` on the same minimal configuration.
3. Repeat on the fifteen-aircraft configuration once the minimal case is understood
   and its acceptance criteria are met.

Each stage is a separate run number with its own registry entry. A stage is not
started while the previous one is unvalidated.

## Implementation not yet written

Unlike the coupled envelope, this protocol needs new code before it can run:

* a longitudinal KS2 profile that closes a prescribed `delta S` at `delta v = 0`,
  with the junction speed and phase split solved from displacement closure;
* the feedforward-plus-feedback tracker replacing the step target in
  `geographic_traffic.py`;
* the separation reporting hook, using the existing `safety.py` module, wired so
  it observes trajectories without touching admissibility.

The first item is the substantive one. The scale should be estimated before the
run is authorised.
