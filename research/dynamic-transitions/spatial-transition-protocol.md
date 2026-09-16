# Stage 2 — isolated lateral, vertical and diagonal transitions

This prospective protocol extends R0009 without changing its approved figures
or numerical archive. It is a single-aircraft communication/motion experiment,
not occupied-lane ACC, capacity, or certified flight planning.

## Frozen comparison and provisional parameters

Retain the Bay Area airport-access proxy route, 12 BS locations, radio kernel,
initial d=0 m and h=300 m, 50 m/s physical reference-tangent velocity, Theta
−1.5 dB, 30 s available-history exposure, 5 s policy clock, 20 s decisions,
240 s lookahead, 30 s cooldown and R0009's degradation-only recovery gate.
Candidate coordinates are parameterized arrays, not a hardcoded 2×2/3×3.

The first experiment uses d=−900:100:+900 m and h=100:100:500 m. All four
matched conditions begin at (d,h)=(0,300): no transitions, lateral only,
vertical only, and joint lateral/vertical (including direct diagonals).
Motion-step and forecast-step refinements of the joint case are separate.

Lateral limits remain 8 m/s and 0.3 m/s². Vertical limits are symmetric
3 m/s and 0.2 m/s², inherited from the previously reviewed exploratory
policy-motion configuration, not newly calibrated aircraft capabilities.
The vertical coordinate uses the inherited numerical BS-height convention;
terrain clearance, absolute altitude, bank, jerk, and energy are unvalidated.
Heights above 300 m and selected horizontal links beyond 4 km are explicitly
flagged as propagation-model extrapolation. The finite BS inventory is unchanged.

## Continuous motion and objective

For u=(t−t0)/T, f(u)=10u³−15u⁴+6u⁵, clamped to [0,1], use

    d(t)=d0+Delta_d*f(u), h(t)=h0+Delta_h*f(u).

The two coordinates share one duration. Zero endpoint transverse velocities
and accelerations are retained; no instantaneous reset occurs when C returns.

    T0=max(30,
           1.875*|Delta_d|/8, sqrt((10/sqrt(3))*|Delta_d|/0.3),
           1.875*|Delta_h|/3, sqrt((10/sqrt(3))*|Delta_h|/0.2)).

Search T0,T0+5,... within the forecast horizon. A 100 m horizontal move has
T0≈43.87 s; a 100 m vertical move has T0=62.5 s. A diagonal with 100 m in
each component has displacement 141.42 m, not 100 m, and T0=62.5 s.

Stable post-completion C must span 30 s and begin at least 5 s before the
matched staying forecast. Search increasing Euclidean displacement
rho=sqrt(Delta_d²+Delta_h²), then shortest accepted tested duration, earlier C
recovery, target d, target h. This is an unweighted geometric displacement
objective, not energy, comfort, or global path optimality. Component motion
costs remain different and are reported separately.

Use x(q,d,h)=[r(q)+d*N(q),h] and

    dq/dt=V/(|r'(q)|*(1−kappa*d)),
    velocity_xy=V*T+d_dot*N, velocity_z=h_dot,
    acceleration_xy=−d_dot*kappa*V/(1−kappa*d)*T
                    +(V²*kappa/(1−kappa*d)+d_ddot)*N,
    acceleration_z=h_ddot.

Total speed is sqrt(V²+d_dot²+h_dot²), not exactly V during a transition.
Progress integration does not depend on altitude in this height-independent
reference frame. Position, velocity and acceleration are continuous; jerk
may jump at spline knots and maneuver boundaries.

## Geometric admissibility

Retain the R0009 coordinate regularity guard 1−kappa*d>0.05. The 976 m
reference-derived radius is an explicitly provisional geometric benchmark.
Extend the same conservative bound to spatial curvature:

    max |kappa_3D| <= max |kappa/(1−kappa*d)|
                     + (10/sqrt(3))*rho/(T²*V²).

For v=(V,d_dot,h_dot), the reference-turn contribution to |v×a|/|v|³ is
|K|*V*sqrt(V²+d_dot²)/(V²+d_dot²+h_dot²) <= |K|, with
K=kappa/(1−kappa*d). The remaining cross-product contribution is bounded
by sqrt(d_ddot²+h_ddot²)/V². Both transverse accelerations share f'', giving
the bound above. It reduces to R0009's bound when Delta_h=0.

Apply the bound over the entire maneuver and check fixed-target curvature
through the remaining route. This remains conservative and does not establish
physical aircraft feasibility. Whole-flight sampled 3D curvature and acceleration
are diagnostics, not replacements for the continuous geometric bound.

## Analysis and reporting

Compare matched flights and keep singleton exposure/C/R/F distinct from SINR.
Report accepted/completed moves by direction, target sequence, fallback time,
path length, maneuver duration, total transverse displacement, total climb
and descent, and actual velocities/accelerations. No forced descent or diagonal
is disguised as an adaptively selected action. If the planner selects none,
report none; software tests must nevertheless verify both directions and
simultaneous motion.

Visualization uses paired route-progress profiles (d,h,policy), selected
offset–height cross-sections with a common SINR color scale, and motion-cost
comparisons. A 3D path is supplementary and labeled if axes are exaggerated;
the unaltered first-stage figure remains the approved geographic overview.

Each run preserves resolved inputs, source ZIP, observations, all evaluated
candidate decisions, motion traces and hashes. Verify Cartesian kinematics,
independent radio/exposure, C recovery, selection order and refinement results.
Do not infer a benefit of joint motion merely from its larger action set.
