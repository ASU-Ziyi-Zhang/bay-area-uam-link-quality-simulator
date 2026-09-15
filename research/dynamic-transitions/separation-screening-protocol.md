# Protocol — two-layer separation check on multi-aircraft traffic

> **Superseded 2026-09-09 by `aks-longitudinal-protocol.md`.** The separation layer is
> report-only, so it does not interact with the longitudinal reference and is carried
> across every case of that run instead of needing a run of its own. Kept for the
> definitions and the reasoning; do not run this as a separate experiment.

Date: 2026-09-09. The run number is assigned from the registry at run time and is not fixed here. Draft protocol, **not yet authorised to run**, and **blocked on R0022**.
It covers the event threshold and predictive screening described to the collaborator on
2026-09-08. It does not estimate capacity and does not change how any decision is made.

## Why this cannot be part of R0022

A single aircraft forms no pairs, so every separation metric returns zero violations by
construction. That is a property of the metric, not evidence that a trajectory is safe.
Screening therefore has to be exercised on multi-aircraft traffic. The smallest suitable
configuration already audited is R0019: fifteen aircraft on three populated lateral flows.

## Question

The current admissibility screen is an ellipsoid with 200 m horizontal and 100 m vertical
semi-axes, which the workspace review records as "not approved for formal claims". Adding
a citable aviation event metric and a predictive layer raises three questions:

1. How often do the ellipsoid screen and the NMAC event metric disagree?
2. Does the predictive layer flag encounters the event layer does not, and how early?
3. Are the existing target-gap rejections explained by either criterion, or by neither?

## Stage 1 is report-only

The first stage **changes no decision**. NMAC and the predictive screen are computed and
reported alongside the existing ellipsoid, which alone continues to govern admissibility.
This is deliberate: the two criteria must be comparable on identical trajectories before
either is allowed to influence a result. Any run that alters decisions is a separate stage
with its own number.

## Definitions

**Event layer.** Using the quantitative convention for safety modelling, a near mid-air
collision requires horizontal and vertical proximity **at the same instant**:

    exists t:  r_h(t) <= 152.4 m  and  |dz(t)| <= 30.48 m

Taking the horizontal minimum at one time and the vertical minimum at another and combining
them would misreport the event. The FAA reporting definition of a near midair collision is
a separate and broader notion and is not used here.

**Predictive layer.** Following the structure used by NASA's DAIDALUS implementation, the
horizontal test first asks whether the current horizontal range is already inside the
threshold, and only otherwise evaluates a predicted closest approach together with a
modified tau. Two aircraft flying parallel at constant velocity are therefore **not**
excluded merely because their range rate is zero: if they are already inside the distance
threshold they are in violation. The vertical test is analogous over a look-ahead interval,
and a violation requires both.

Thresholds for this layer are **research assumptions for a cooperative, same-direction,
automated corridor**, deliberately not the DO-365 Phase 1 values. Those were sized to
encompass TCAS II advisory thresholds for encounters that include piloted traffic, and
their 4000 ft horizontal threshold is far larger than the 300 m lane spacing studied here.
The run reports results across a small grid of candidate thresholds rather than asserting
one.

## Frozen inputs

The R0019 configuration unchanged: fifteen aircraft, three populated lateral flows at
-300 / 0 / +300 m, 300 m altitude, 90 s in-flow headway, 30 s inter-flow phasing, the
policy-spacing ACC, target-gap insertion check and existing ellipsoid screen. No scenario
input, controller gain or admissibility rule is edited.

## Controls

| ID | Reported screens | Decisions governed by | Purpose |
|---|---|---|---|
| `S0_baseline` | ellipsoid only | ellipsoid | Reproduction gate against R0019 |
| `S1_nmac_report` | ellipsoid + NMAC event | ellipsoid | Do the two criteria agree? |
| `S2_predictive_grid` | ellipsoid + NMAC + predictive at several thresholds | ellipsoid | How early, and how often, would a predictive layer fire? |

## Acceptance criteria, declared before the run

1. **S0 must reproduce R0019 record for record**: fifteen aircraft completed, twenty-eight
   lane changes, 832 gap-candidate rejections, identical accepted decision times. A mismatch
   stops the run and is reported as a defect, not as a result.
2. **S1 and S2 must produce trajectories identical to S0.** Reporting a metric must not
   perturb motion. Any difference is a defect.
3. The ellipsoid is verified to contain the NMAC cylinder before the run, by evaluating the
   cylinder corner: `(152.4/200)^2 + (30.48/100)^2 = 0.674 < 1`. Containment on each axis
   separately would not establish this.

If S0 fails, R0023 is void; the number is retired and not reused.

## Metrics

Per pair and per run: NMAC events with the instant, horizontal range and vertical
separation at that instant; minimum cylinder coordinate over the run and where it occurs;
predictive violations with modified tau and vertical entry time at first alert; and the
lead time between a predictive alert and any subsequent event.

Cross-tabulated against the existing screens: how many target-gap rejections would also
have been rejected by NMAC, by the predictive layer, by both, or by neither. The
expectation is that **NMAC almost never binds** at this density -- the policy-dependent
target gap at 50 m/s is 1367.5 m for C, 2117.5 m for R and 3617.5 m for F, which is nine to
twenty-four times the 152.4 m NMAC horizontal threshold -- and confirming that is the point.
A metric that never fires is a floor, not a constraint, and the run should say so plainly.

## Model boundaries

* Report-only. No decision, gain, gap rule or admissibility criterion is changed.
* Sampled trajectories are a check, not a continuous-time proof. The sample step is reported
  and a refinement is run at half the step.
* Velocities are taken from the simulator where available rather than differenced from
  sampled positions; where differencing is used it is recorded.
* No capacity, no separation standard claim, no operational concept. The AFR lateral
  separation in the notional tables is explicitly undefined, and this run does not propose a
  value for it.
* R0021 is not used. Its multi-aircraft controller does not fully implement the agreed group
  exposure and neighbour non-degradation constraints, so its outputs are a defect diagnostic
  rather than a scientific result, and it is not a valid base for a separation study.

## Known gap this run does not close

The coupled acceleration envelope from R0022 has a longitudinal half that Stage 01 never
exercises, because `a_lon = 0` there. R0019 traffic does run the policy-spacing ACC, so a
later stage can exercise it. R0023 as specified is report-only and does not, and should not
be presented as validating the coupling.
