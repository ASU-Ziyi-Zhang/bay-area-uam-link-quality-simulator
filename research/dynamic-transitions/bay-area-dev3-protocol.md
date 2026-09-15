# Dev3 — calibrated policy in the Bay Area longitudinal/lateral controller

Date: 2026-09-12.

This protocol reconnects the September 1 motion and safety revisions to the
current `airport_to_airport` radio/corridor input and the policy working point
selected on September 11. It does not reuse R0021 as evidence: that run used an
older policy threshold, an ACC-only multi-flow controller, and incomplete
multi-aircraft benefit constraints.

The September 1 slide deck and follow-up note are source material, not runtime
instructions. Their five requested checks map to the implementation as follows:

1. the 0.1 g maneuver component limit is retained;
2. longitudinal and maneuver accelerations share the elliptical envelope;
3. the quintic profile remains the baseline, with curve-family comparison kept
   separate from the main stay/change experiment;
4. the 152.4 m × 30.48 m NMAC volume is the hard event metric, while predictive
   screening remains a distinct pre-maneuver layer;
5. radio sampling and policy updates use separate clocks and policy transitions
   require persistence.

## Frozen communication working point

| Quantity | Value |
|---|---:|
| SINR threshold | -2.0 dB |
| Radio sampling | 2 s |
| Assessment window | 30 s |
| Policy update | 5 s |
| Target group | focal aircraft + at most two neighbours per side |
| C/R bad exposure | 5% / 10% |
| Persistence | k=3 policy updates |

The values are a simulation working point, not a receiver-certified
specification. Receiver quantisation, correlated error and reporting behaviour
remain external inputs to obtain from TEMA.

## Stepwise execution

### Dev3A — single lane, one fixed level, longitudinal AKS only

Use the real Bay Area route, twelve retained base-station sites, five scheduled
aircraft at 300 m, and the R0036 policy distances (1319.9/2069.9/3569.9 m).
Disable lane and level changes. Record policy transitions, KS2-valid and feedback
fallback time, gap error, completion before the next policy transition, speed,
joint-envelope use, NMAC events, entry delay and mission completion.

The gate passes when the configured clocks and persistence replay exactly, all
scheduled aircraft complete without an execution rejection or NMAC event, no
lateral motion occurs, and all declared motion bounds pass. Longitudinal target
completion is a measured result rather than a forced 100% acceptance condition.

### Dev3B — multiple lateral lanes at 300 m

Keep the Dev3A policy/controller settings. Compare paired AKS-stay and
AKS-plus-lane-change cases under identical arrivals and radio input. Add complete
transition rate, candidate rejection reason, new/source neighbour gaps,
replanning, reversal, F aircraft-time and common-horizon benefit.

### Dev3C — fixed multi-lane, multi-level layout

Add fixed altitude layers without permitting vertical transitions. Validate
three-dimensional neighbour discovery, layer separation, group ownership,
contention for a target gap and deterministic arbitration before increasing
traffic duration.

### Dev3D — vertical and joint transitions

Only after Dev3C passes, enable climb/descent candidates and validate vertical
speed/acceleration, the three-axis joint envelope, predictive separation and
relationship transfer. Capacity remains outside Dev3; it follows the G2 gate.

## Run registration

The first Dev3A archive attempt is R0037. It completed simulation but failed
strict JSON archival because “no pairwise separation available” was represented
as infinity. R0037 is retained as a failed run; the corrected representation is
JSON `null`, with no physics or threshold change. R0038 passed that gate, but
its first summary called every policy transition a longitudinal action, including
the lead aircraft with no predecessor. The reporting-only correction excludes
those non-applicable transitions and records initial/final gap error, relationship
changes and isolated-reference timing compatibility. The unchanged rerun is
R0039. Later stages receive new immutable R numbers; a failed gate is retained
and never overwritten.

## Post-R0039 stage decision (not part of the preregistered R0039 gate)

R0039 passed the software, replay, mission-completion and sampled-safety checks,
but its realized longitudinal timing was not adequate to advance directly to
Dev3B: 0/55 applicable tasks fully settled before the next policy change and
only 8/55 had a dwell at least as long as the isolated R0036 reference. A short
Dev3A reconciliation set will therefore separate (i) monitor-only policy replay,
(ii) frozen-policy five-aircraft motion, (iii) dynamic policy with fixed neighbour
identity, and (iv) the full current coupling. This is a diagnostic staging
decision; it does not retroactively alter the R0039 configuration, acceptance
criteria or result.
