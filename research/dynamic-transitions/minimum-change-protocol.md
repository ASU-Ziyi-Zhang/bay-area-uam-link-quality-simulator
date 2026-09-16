# Minimum necessary lateral movement — protocol revision

User agreement: retain the physical reference curve and the 100 m candidate
grid, plan the smallest policy-restoring displacement in advance, and let
turning constraints govern expansion of the lateral envelope. This revision
does not replace R0008 or change its results.

## Scope and explicit provisional settings

Keep the same airport-access proxy route, 12-site inventory, 300 m altitude,
50 m/s physical tangential component, radio kernel, Theta −1.5 dB, 30 s exposure
window, 5 s policy observations, 20 s decision clock, 240 s prediction horizon,
30 s cooldown, and quintic motion law. Candidate spacing remains 100 m.
Test envelopes ±500 and ±900 m. The latter is an exploratory expansion, not
certified airspace or calibrated radio coverage; source coordinates are unchanged.

Use **976 m** as a provisional horizontal trajectory-radius lower bound. It is
the rounded-down minimum radius of this reference curve (976.198 m), **not**
an aircraft certification, passenger-comfort threshold, or the separate 8 km
research constraint. Retain |d_dot|≤8 m/s and |d_ddot|≤0.3 m/s². No numerical
jerk or bank-angle limit is invented. With the fixed physical tangential-speed
convention, actual acceleration is continuous; jerk can jump at cubic knots
and quintic endpoints. This smoothness wording was clarified after the run
without changing the motion equations or numerical outputs.

## Decision rule

1. If the observed policy is C, do not initiate a communication-driven move.
2. Otherwise predict staying and moving to candidates, retaining the actual
   available exposure history. Search absolute displacement in 100 m increments.
3. A candidate must restore C and maintain it continuously through a full 30 s
   sampled observation window beginning at the first policy observation at or
   after maneuver completion. Here 30 s is the existing exposure window, not
   a separately calibrated stability parameter. No samples after exit count.
4. The start of this continuous C episode must be at least one policy sample
   (5 s) earlier than the first 30 s continuous-C episode predicted for staying,
   over the common in-corridor horizon. If staying recovers no later, do not
   move. If no candidate qualifies, retain the current offset and record why.
5. Prefer the smallest absolute displacement; among equally near feasible
   candidates, prefer the shortest maneuver duration, then the earlier C
   recovery and smaller target offset for a deterministic tie. The old mean
   exposure/spacing-benefit maximization is not the new selection objective.
6. Plan the complete maneuver in advance, with zero lateral velocity and
   acceleration at both ends. Do not interrupt it by zeroing a nonzero lateral
   velocity. Mid-maneuver continuous replanning is deferred; this revision
   selects a smaller endpoint before departure from the current flow.

The analytic minimum duration is

    T0 = max(30, 1.875 |Delta d| / 8,
             sqrt((10/sqrt(3)) |Delta d| / 0.3)).

Search T0, T0+5 s, T0+10 s, ... while completion and the post-move observation
window fit inside the 240 s forecast. Five seconds is the **declared duration
search resolution**, not a proof of the continuous-time optimum. The result is
the shortest accepted tested duration within the smallest feasible displacement
shell. At fixed tangential speed, shorter duration also shortens the path of
the zero-boundary quintic family (|d_dot|≤8 m/s < V=50 m/s). It is not a globally
shortest geographic route or a minimum-distance maneuver over all motion laws.

## Continuous conservative curvature checks

Find all stationary roots and endpoints of signed reference curvature in each
cubic segment. For a fixed offset d, its signed planar curvature is

    kappa_d = kappa / (1 − kappa d).

Reject folded coordinates; in particular 1−kappa*d must remain above 0.05.
The entire ±900 m coordinate envelope passes this check on the current curve;
±1000 m does not. This is a coordinate limitation, not a flight-range limit.

During a quintic movement at tangential speed V, the exact planar curvature is

    kappa_p = V*kappa / ((1−kappa*d)*sqrt(V²+d_dot²))
              + V*d_ddot / (V²+d_dot²)^(3/2).

Bound its magnitude continuously by

    max |kappa/(1−kappa*d)| + max|d_ddot|/V²,

using the reference-curvature extrema over the traveled interval, the entire
source-to-target offset interval, and the analytic quintic acceleration maximum.
This bound is conservative: it may reject a trajectory whose exact curvature
benefits from cancellation, but it does not rely on a sparse curvature sample.
Check fixed-target curvature over the **remaining route to the endpoint** too.
This intentionally conservative terminal condition avoids needing a later
geometric escape maneuver while C would otherwise prohibit a move. It can
exclude paths that a more general future sequence of maneuvers could make valid.

## Outputs and validation

Retain the no-change baseline and the R0008 gain-priority controller as paired
comparators. Save every proposed endpoint/duration, communication rejection,
curvature bound, recovery time, selected candidate, and realized motion. Repeat
the expanded-envelope case with separate motion and forecast step refinements.
Check archived hashes, radio/exposure/policy, actual quintic trajectories,
realized post-completion C windows, minimum-displacement/duration selection,
and radius bounds. Report F duration, time share, movement, and actual distance
without interpreting them as capacity, energy, or passenger-comfort validation.
