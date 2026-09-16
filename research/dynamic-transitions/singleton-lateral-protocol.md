# Stage 1 protocol — single-altitude lateral communication response

Date: 2026-08-31. User authorized executing the 100 m candidate-spacing study and generating images. This protocol fixes the assumptions before the archived run. It implements Stage 1 of the [Toyota research plan](../toyota-research-plan.md), not Stages 2–4.

## Input and controls

Use `airport_to_airport`: Millbrae Caltrain to Santa Clara Caltrain, the repository's SFO/SJC access proxies, not actual airport vertiport coordinates. Retain the scenario's 12 physical BS sites and radio configuration. Height is 300 m under the existing flat-height/AGL convention. No terrain elevations or measured airborne radio data are added.

Eleven independent static flights use d = −500, −400, …, +500 m. Five adaptive flights all start at d=0 and use envelopes ±100, ±200, ±300, ±400, ±500 m, with candidate spacing 100 m. Positive d is left relative to travel from Millbrae toward Santa Clara. The common d=0 static flight is the paired no-change reference for all adaptive flights. Static off-center flights are context, not matched initial-assignment comparisons. Each flight has one aircraft.

The offset envelope is exploratory and does not imply validated coverage or airspace availability. No extra sites outside the saved inventory are invented. Report the fraction of observations with any of the three served/interfering links beyond the model's documented 4 km horizontal range.

## Continuous geographic motion

Raw segment-normal GIS offsets jump at non-collinear polyline vertices. Instead, resample the unchanged reference polyline at uniform progress q, with control spacing at most 500 m, and interpolate the points with the existing natural cubic implementation. Preserve endpoints and report equal-q centerline deviation, route-length change, curvature, and coordinate regularity. This is a numerical representation of the source route, not an optimized new route and not an imposition of the separate 8 km curvature requirement.

Let r(q) be the smooth center, T its unit tangent, N its left normal, l the center's arc length, and kappa its signed curvature. The aircraft position is

    p_xy(t) = r(q(t)) + d(t) N(q(t)),  z(t) = 300 m.

Fix the **physical tangential component** V=50 m/s, rather than dq/dt. Since

    dp/dt = |r'(q)| (1−kappa*d) dq/dt * T + d_dot * N,

integrate

    dq/dt = V / ( |r'(q)| (1−kappa*d) ).

Then actual total speed is sqrt(V²+d_dot²), not exactly V during lateral motion. Integrate q with error-controlled RK4 inside each 0.5 s nominal interval, splitting at observation/decision/maneuver-completion events and solving the corridor-exit time. Compare a full step with two half steps and subdivide until reference-progress disagreement is at most 1e-6 m per nominal interval. This is necessary at spline knots, where curvature derivatives can jump. Check that 1−kappa*d stays positive throughout the allowed envelope, using cubic curvature extrema. No folds or teleportation are accepted.

Quintic lateral displacement uses f(u)=10u³−15u⁴+6u⁵. Duration is the maximum of 30 s, the exact lateral-speed bound, and the exact lateral-acceleration bound. Bounds remain |d_dot|≤8 m/s and |d_ddot|≤0.3 m/s². A direct 100 m move lasts approximately 43.87 s; larger moves take longer and traverse all intermediate positions continuously. Physical duration is not rounded to the integration grid, so motion-step refinement does not redefine the action. The event loop splits at exact completion times.

Cartesian acceleration includes turning of the reference frame:

    a_T = −d_dot*kappa*V/(1−kappa*d)
    a_N = V²*kappa/(1−kappa*d) + d_ddot.

The 0.3 m/s² bound applies to d_ddot, **not** total Cartesian lateral acceleration. Turning acceleration is reported separately. Natural cubic centers are C2; offset positions/velocities are continuous, while acceleration can have finite jumps at knots. There is no jerk, banking, energy, full aircraft-performance, or certified separation model. The study is a communication/kinematic model experiment, not a flyability certification.

## Exposure and decisions

Retain Theta=−1.5 dB, 5 s policy observations, the last 30 s of available history including both endpoints, C tolerance 0.05, and R tolerance 0.10. A singleton's current group bad fraction is 0 or 1. Average its historical fractions using the original available-history convention, verified against the multi-UAM policy kernel. Start with C before the first observation; the t=0 observation can immediately change the policy. History is retained during and after a move.

With these singleton clocks, available history contains at most seven binary samples. Its smallest nonzero exposure is 1/7 > 0.10 (and is larger during startup). Therefore R cannot occur in this particular experiment: the resulting C/F distinction is an intentional consequence of retaining the original estimator, not missing data. Do not infer the absence of R for multi-aircraft groups or other sampling windows.

Decide every 20 s when not moving or cooling down. Forecast staying and each candidate on the known static radio map, using 1 s integration, the same physical speed convention, and a 240 s horizon. Do not forecast further discretionary moves. Each candidate must complete with at least 30 s of common post-maneuver observation remaining before corridor exit. Compare only common in-corridor policy timestamps; do not invent post-exit history. Shortened remaining horizons are recorded.

Keep the reviewed communication gate: predicted mean exposure improves by at least 0.02 and mean policy-spacing priority does not worsen. Rank candidates by larger policy-spacing benefit, then exposure benefit, shorter maneuver duration, then target offset. The priority uses S_C/R/F(50) from the existing model only as a discrete policy preference; **it is not an estimated capacity or enforced singleton gap**. There is no ACC braking or neighboring traffic in this stage. Cooldown is 30 s.

Allow a direct continuous move to any candidate in the envelope: a 100 m candidate grid does not constrain every maneuver to a 100 m hop. This avoids hardcoding a lane count and is feasible only as a traffic-free action-set assumption. Occupied-lane crossing and safety will require Stage 3.

## Outputs and contrasts

Archive every motion interval, Cartesian position/velocity/acceleration, radio/policy observation, candidate score/rejection, accepted/completed move, and exact corridor-exit time. Report:

- Time-weighted exposure and C/R/F shares, holding each observed policy until the next sample or exit; also preserve raw observation counts separately.
- Completed moves, source/target offsets, start/end time and route progress, total maneuver time.
- Actual path length and corridor travel time; changes can be signed because inside/outside offsets alter route length. Extra length is not an energy estimate.
- Static along-route SINR field, serving-site IDs, and model-domain cautions.
- A 0.25 s motion-step refinement and a separate 0.5 s forecast-step refinement for ±500 m, leaving all other parameters unchanged. These are numerical diagnostics, not parameter tuning for benefit.

Generate a communication heatmap with paths, a paired SINR/exposure/policy profile, an envelope/move/cost comparison, and static-offset policy context. PNG/SVG figures must be reproducible from the archived run alone and label the run, scenario, singleton convention, speed and threshold. No capacity figure is produced.

## Provenance and validation

R0006 preserves the first attempt, which failed independent displacement/velocity consistency checks at some spline knots. Its outputs and provisional figures are not presentation results. R0007 preserves a subsequent failed run: recursive error subdivision requested agreement below floating-point precision. R0008 is allocated to the corrected error-controlled integrator, with a roundoff floor of 16 machine epsilons times reference-coordinate magnitude, and integration-independent maneuver durations. Never overwrite these runs. Keep source/input ZIP snapshots with hashes, environment versions, command, resolved inputs and case parameters, output hashes, and run status. Figures live in `dynamic-transitions/figures/<run-ID>/`, with their own manifest. Source modules remain under `src/`, entry points under `scripts/`.

Tests cover analytic versus finite-difference Cartesian kinematics, knot continuity, quintic bounds, singleton policy equivalence, horizon boundaries, no-gain/no-move behavior, and radio consistency. Independent post-run audit checks saved radio/policy formulas, time shares, motion continuity/velocity consistency, candidate limits, and hashes. Preserve any failed run and allocate a new ID after correction. Do not label a failed or unverified run a research result.
