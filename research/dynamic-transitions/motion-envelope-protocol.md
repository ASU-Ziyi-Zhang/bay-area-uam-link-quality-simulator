# Protocol — coupled acceleration envelope for the single-aircraft lane change

Date: 2026-09-09. The run number is assigned from the registry at run time and is not fixed here. Draft protocol, **not yet authorised to run**. It fixes the assumptions,
controls, metrics and acceptance criteria before any archived run, per the workspace
workflow. This is Step 01 only: one aircraft, fixed altitude, lateral actions.
It does not touch Step 03, multi-aircraft traffic, separation screening or capacity.

## Question

The current model bounds only the maneuver-induced offset acceleration `d_ddot`. Following
the reference alignment produces a second normal term that has never been bounded, and the
lateral jerk has never been bounded at all. Does adding

1. a separated treatment of route turning and maneuver acceleration,
2. a coupled longitudinal-lateral envelope, and
3. a jerk bound

change the Step 01 rolling-prediction result, and if so by how much?

The expected answer is **that it should not change much**. The purpose is to make the
constraint chain explicit and attributable, not to improve a metric. A null result is a
valid and useful outcome.

## Frozen inputs

Everything R0017 used, unchanged: `scenarios/airport_to_airport`, the archived site
inventory and radio configuration, 300 m altitude, `-900:100:+900` m lateral candidate
grid, the 60 s warning plus 60 s evaluation rolling controller, `Theta = -1.5` dB,
5 s policy sampling, 30 s exposure window, C tolerance 0.05, R tolerance 0.10, and
`V = 50` m/s as the physical tangential component. No scenario input is edited.

## What is added

Route turning and maneuver acceleration are separated. In the corridor frame

    a_T = -d_dot kappa V/(1 - kappa d) + a_lon
    a_N = V^2 kappa/(1 - kappa d) + d_ddot
          |________ route ________|   |maneuver|

Measured on this corridor at `V = 50` m/s the route term alone reaches 2.561 m/s^2
(minimum radius 976.2 m), which is a 14.63 degree coordinated bank and a 3.35% load-factor
increase, and exceeds 0.981 m/s^2 over 5.27% of the route with no maneuver at all. A single
limit applied to the sum would therefore declare part of the reference alignment
infeasible.

The route term is treated as a steady coordinated turn, flown by banking, and checked
against its own bank-angle allowance. The 0.1 g figure is applied to the maneuver term.
This reading follows the source remark, which was made while comparing lane-change path
families -- "for those lane changes ... the curvature would give you your maximum lateral
acceleration" -- and was motivated by steering authority in the car context, where road
curvature and lane-change curvature do not separate. The alternative reading is retained
as a sensitivity case rather than discarded.

The coupled envelope is

    (a_T/a_lon_limit)^2 + (a_N_binding/a_lat_limit)^2 <= 1

with the longitudinal limit selected per sign of `a_T` (+1.5 accelerating, -2.0 braking).
This coupling is an **assumption**. The automotive friction circle arises from tyre-road
contact and does not establish that an eVTOL shares one budget across axes.

Quintic peak factors, exact and verified numerically:

| Quantity | Peak | Location |
|---|---|---|
| `\|d_dot\|` | `1.875 D/T` | `u = 1/2` |
| `\|d_ddot\|` | `(10/sqrt(3)) D/T^2` | `u = 1/2 -/+ 1/(2 sqrt(3))` |
| `\|jerk\|` | `60 D/T^3` | `u = 0` and `u = 1` |

The jerk peak sits at the endpoints and steps from zero, so the quintic is C2 but not C3
across the start and end of a maneuver. Each limit inverts to a lower bound on `T`, and
the binding one is the largest. The curve shape carries no free parameter; `T` is the only
design variable.

## Controls

All cases share every frozen input and differ only in the envelope.

| ID | Envelope | Purpose |
|---|---|---|
| `C0_archived` | none | Reproduction gate against R0017 |
| `C1_mirror` | maneuver_only, `a_lat = 0.3` | Does the new code path reproduce C0? |
| `C2_01g` | maneuver_only, `a_lat = 0.981` | The advised nominal case |
| `C3_02g` | maneuver_only, `a_lat = 1.962` | The advised higher case |
| `C4_01g_total` | total, `a_lat = 0.981` | Sensitivity on the reading of 0.1 g |
| `C5_01g_jerk` | maneuver_only, `a_lat = 0.981`, `J = 0.5` | Jerk bound at the EN 13803 line value |

`V_lat = 8` m/s and `T_min = 30` s are held at their existing values in every case, so the
comparison isolates the acceleration and jerk treatment. Both are recorded as unsourced;
the workspace review already lists the lateral speed limit as an "old exploratory
component value" and the 30 s figure is a numerical floor rather than a physical limit.

The route bank allowance is 4.575 m/s^2, i.e. a 25 degree coordinated bank. That is the
usual normal-operations bank limit for transport aircraft and is **not** an eVTOL figure.

## Acceptance criteria, declared before the run

1. **C0 must reproduce R0017 record for record**: two lateral moves, fallback time
   280 s to 230 s, identical accepted decision times and targets. A mismatch stops the run
   and is reported as a defect, not as a result.
2. **C1 must reproduce C0** in completed moves, policy time shares and accepted targets.
   Maneuver durations may differ by at most 0.05 s, which is the declared duration-search
   refinement tolerance, not a physical difference.
3. Every case must report the maximum route bank angle reached, and no case may exceed the
   declared allowance without being flagged.
4. Any candidate rejected by the envelope must record which condition rejected it.

If C0 or C1 fails, R0022 is void; the number is retired and not reused.

## Metrics

Per case: completed moves with accepted times, sources and targets; policy time shares and
observation counts; fallback time; total maneuver time; corridor time and path length;
maximum route bank angle; peak lateral speed, acceleration and jerk per maneuver; envelope
utilisation maximum with the sampling step; and the count of envelope rejections by reason.

Peak demands are reported for every executed maneuver so the jerk column is visible even
when jerk does not bind.

## Model boundaries

* One aircraft. No neighbouring traffic, no ACC, no separation screening, no capacity.
* **`a_lon = 0` throughout Stage 01**, so the longitudinal half of the coupled envelope is
  never exercised. Its first real test requires a scenario with ACC. This is a declared gap,
  not an oversight, and R0022 cannot validate the coupling.
* The envelope check samples the maneuver at a fixed time resolution. Sampling is a check,
  not a continuous-time proof; the step is reported.
* The duration search returns *a* feasible duration, not a proven minimum. Lengthening a
  maneuver reduces the maneuver terms but carries it over a longer stretch of route, so
  feasibility is not monotone in duration.
* No attitude, antenna pattern, roll-rate, energy, wind or vehicle-performance model. Bank
  angle is a reporting convention derived from acceleration, not a simulated state.
* 0.1 g, 0.2 g, the 25 degree allowance and the 0.5 m/s^3 jerk value are research
  assumptions with recorded provenance, not certified limits.

## Outputs

Configuration snapshot, input/code/output hashes, Git SHA, Python and NumPy versions and
the command, per the workspace runner convention. Registry entry in the topic README with
ID, question, config, code version, status, result and limitations.

## Relationship to R0023

R0023 covers the two-layer separation check, an NMAC event threshold plus predictive
screening. It is **not** part of R0022 because a single aircraft forms no pairs and
therefore produces no separation events by construction. R0023 requires a multi-aircraft
scenario; the smallest suitable one is the R0019 fifteen-aircraft configuration. Its first
stage reports the NMAC event layer alongside the existing ellipsoid screen without changing
any decision, so the two criteria can be compared before either is allowed to affect a
result.
