# Motion control: longitudinal spacing, lane change and safety limits

This document describes the motion layer added in v0.2.0: how an aircraft
follows a communication-dependent spacing target, how it changes lane, and the
limits every manoeuvre is checked against. Results quoted here come from
validated run archives; the archives themselves are local (see
[research/README.md](../research/README.md)).

## 1. Policy-dependent spacing

The TRB spacing law becomes the target for longitudinal control:

    S(p, v) = d0 + τ_p v + b v²

| Symbol | Value | Meaning |
|---|---|---|
| d0 | 152.4 m | NMAC horizontal threshold (500 ft) |
| τ_C / τ_R / τ_F | 15 / 30 / 60 s | policy headway (research assumption) |
| b | 0.167 s²/m | quadratic term (research assumption) |

At 50 m/s: **C 1319.9 m, R 2069.9 m, F 3569.9 m**. Every spacing target, group
bound and planning rate in the code is derived from these parameters, so a
change of τ propagates everywhere (`GeographicTrafficConfig.spacing`).

Interpretation: after the leader brakes and the follower responds τ seconds
later, the two are still outside the NMAC distance. The margin against
uncertainty sits in τ v, which is the quantity the communication policy sets.

## 2. Longitudinal control

### ACC (baseline)

Classic linear adaptive cruise control with the policy target
(VanderWerf et al. 2001; Shladover et al. 2012):

    a_cruise = k_s (v_c − v)
    a_follow = k_g (g − S_P) + k_v (v_L − v)
    a        = clip( min(a_cruise, a_follow), −b_max, a_max )

Gains k_s = 0.2 s⁻¹, k_g = 0.005 s⁻², k_v = 0.4 s⁻¹; b_max = 2.0 m/s²,
a_max = 1.5 m/s²; speed 30–80 m/s. For an established follower–leader pair the
cruise term is dropped (`recovery_pairs`), so a follower can run above cruise to
close a gap after a policy upgrade; an arbitrary sufficient gap does not start a
pursuit. Implementation: `geographic_traffic.acc_controls`.

**Limitation.** When the policy changes, S_P jumps (C → F: 1320 → 3570 m), so the
gap error jumps by 2250 m at one instant. k_g(g − S_P) ≈ −11 m/s² saturates at
−2.0 m/s², after which the error decays slowly: C → F settles only after 1201 s.

### AKS (Adaptive Kinematic Smoothing)

AKS plans a bounded reference from the current gap to the new target and tracks
it (Zhou & Zhang, r-Safety and Adaptive Kinematic Smoothing, manuscript in
preparation). With a constant-speed leader over the transition duration T:

    Δx_F    = Δx_L + g(0) − S_P                      distance the follower must cover
    g_ref(t) = g(0) + Δx_L(t) − Δx_AKS(t)            planned gap, from g(0) to S_P at T
    a        = a_AKS + k_g (g − g_ref) + k_v (v_AKS − v)

The reference is chosen from KS1 (symmetric), KS1A (shifted timing, the r-kernel)
and KS2 (two phases) within the speed and acceleration limits. The gains are the
ACC gains; only the reference differs, so the error starts near zero instead of
at −2250 m. If the leader departs from the constant-speed assumption the
reference is invalidated and the follower falls back to ACC feedback, which is
recorded. Implementation: `aks.py`, pair harness `two_uam_longitudinal.py`.

### Result (R0036, confirmed)

Two aircraft, leader at 50 m/s, same envelope for both controllers. Settling
requires |gap − S(p, 50)| ≤ 1 m and |v − 50| ≤ 0.05 m/s for the rest of the trace.

| Transition | AKS | ACC |
|---|---:|---:|
| C → R | 73.8 s | 829.1 s |
| R → F | 147.5 s | 1137.5 s |
| C → F | 221.3 s | 1200.9 s |
| R → C | 50.7 s | 731.2 s |
| F → R | 99.0 s | 921.0 s |
| F → C | 148.5 s | 858.2 s |

72 traces, zero failed checks. Details:
[confirmed step 04](../research/dynamic-transitions/confirmed/step-04-longitudinal-policy-transitions/revision-04/),
protocol [two-uam-longitudinal-protocol.md](../research/dynamic-transitions/two-uam-longitudinal-protocol.md).

## 3. Lane-change profiles

A lane change of lateral distance D over duration T:

    y(t) = y0 + D · s(u),   u = t / T,   s(0) = 0, s(1) = 1, s′(0) = s′(1) = 0

| Profile | Lateral speed s′(u) ∝ | End acceleration |
|---|---|---|
| Quintic r(2,2) | u²(1 − u)² | zero (C2) |
| r(2,3), early peak | u²(1 − u)³ | zero |
| r(3,2), late peak | u³(1 − u)² | zero |
| Cubic Bézier | u(1 − u) | steps to ±6 D/T² (C1) |
| Three-clothoid | s‴ = +J, −J, +J over ¼, ½, ¼ | zero |

The r-kernel r(p, q) peaks at u = p/(p + q); its coefficients only normalise the
move to D. The three-clothoid follows Oh et al. (2025), IEEE TCST 33(4).
Implementation: `five_uam_lateral.py` (`beta_shape`, `bezier3_shape`,
`three_clothoid_shape`, `QUINTIC_SHAPE`; code names keep the earlier β notation).

### Decision rule

At each 5 s decision, a lane change is taken only if an admissible candidate
exists **and** it lowers the predicted policy cost
J = Σ_i ∫ c(p_i) dt, c(C) = 0, c(R) = 1, c(F) = 2, over the affected aircraft
by more than the numerical error band. Admission (joint envelope, NMAC volume
over the whole prediction, insertion spacing for relationships the aircraft
creates) filters candidates before ranking. During the move the longitudinal
acceleration is held at zero, and the aircraft counts as in the new lane once it
crosses the midline. Full rule: [three-uam-lateral-protocol.md](../research/dynamic-transitions/three-uam-lateral-protocol.md).

### Result (R0063)

Five aircraft on two straight lanes 300 m apart; aircraft A decides whether to
leave a lane that becomes −20 dB from 1000 m ahead.

Same D = 300 m and T = 90 s for every shape:

| Profile | Crosses midline | Peak lateral speed (m/s) | Peak lateral accel. (m/s²) | Peak jerk (m/s³) | Policy cost saved |
|---|---:|---:|---:|---:|---:|
| Quintic r(2,2) | 45.0 s | 6.25 | 0.214 | 0.0247 | 154 |
| r(2,3) | 37.9 s | 6.91 | 0.301 | 0.0494 | 168 |
| r(3,2) | 52.5 s | 6.91 | 0.301 | 0.0469 | 138 |
| Cubic Bézier | 45.0 s | 5.00 | 0.222 | 0.0049* | 154 |
| Three-clothoid | 45.0 s | 6.67 | 0.296 | 0.0132 | 154 |

\* excludes the acceleration step at start and end.

The saving follows the midline crossing time, not the curve family. With each
shape at its own minimum feasible duration the decision rule chose the cubic
Bézier (cost 387 against 573 staying). Its peak lateral speed is 1.5 D/T against
1.875 D/T for the quintic, so it meets the 8 m/s limit at a shorter duration; the
price is the acceleration step at its ends. At the four measured Bay Area weak-zone
magnitudes the saving falls to 0 / 6 / 6 / 21. The quintic remains the working
profile. Tables: [R0063 results](../research/dynamic-transitions/results/R0063-lane-change-profiles/).

## 4. Motion envelope

The normal acceleration of an aircraft following a curved route at offset d is

    a_N = V² κ / (1 − κ d) + a_LC

(route turn plus lane change). Longitudinal and lane-change acceleration share
one budget:

    (a_T / a_T,max)² + (a_LC / a_LC,max)² ≤ 1

The route turn is checked separately against its own bank allowance
(`coupling_mode = "maneuver_only"`).

| Quantity | Limit | Source |
|---|---|---|
| Longitudinal acceleration | +1.5 / −2.0 m/s² | research assumption |
| Speed | 30–80 m/s | 1.2 V_stall of a low-wing-loading eVTOL; below 170 kt Joby S4 cruise |
| Lane-change acceleration | 0.981 m/s² (0.1 g) | Paielli (2023), NASA Ames |
| Route-turn acceleration | 4.575 m/s² (25° bank) | transport-aircraft normal-operations bank; research assumption |
| Lateral / vertical speed | 8 / 3 m/s | research assumption |
| Vertical acceleration | 0.2 m/s² | research assumption |
| Jerk | reported, not bounded | |

At 50 m/s the lane-change limit corresponds to κ ≤ 3.9 × 10⁻⁴ m⁻¹ and the route
limit to R ≥ 546 m; the tightest curve on the airport_to_airport corridor is
976 m. The coupling is an assumption: the automotive friction circle does not
establish that an eVTOL shares one budget across axes. Implementation:
`motion_envelope.py`; protocol [motion-envelope-protocol.md](../research/dynamic-transitions/motion-envelope-protocol.md).

The corridor dispatcher also enforces the route limit ahead of time: a speed
governor on a 0.5 m curvature grid limits each held command so that
v² + 2ax ≤ V(x)² at every point reached before the next command
(`bay_area_dispatch.route_constraints`).

## 5. Conflict: near mid-air collision (NMAC)

An NMAC is a horizontal distance ≤ 152.4 m (500 ft) **and** a vertical distance
≤ 30.48 m (100 ft) at the same instant (Truitt et al. 2016, DOT/FAA/TC-16/11).
It is an event to count, not a distance to fly.

- The spacing law starts at the NMAC radius (d0 = 152.4 m), so every policy
  spacing lies outside it.
- A lane change is admitted only if every forecast aircraft stays outside the
  volume and above policy spacing.
- NMAC is counted in every run. The lane-change study checks each step's
  trajectory polynomials over the whole interval; the corridor runs check each
  step at its start, midpoint and end.

Implementation: `safety.py` (`NMAC_HORIZONTAL_M`, `NMAC_VERTICAL_M`,
`SeparationVolume`, screening).

## 6. Reproduce

```sh
python scripts/run_two_uam_longitudinal.py --config research/dynamic-transitions/configs/two_uam_longitudinal_nmac.json --output research/dynamic-transitions/runs/<ID>
python scripts/verify_two_uam_longitudinal.py research/dynamic-transitions/runs/<ID>
python scripts/plot_two_uam_longitudinal.py --run research/dynamic-transitions/runs/<ID> --output <figure folder>

python scripts/run_five_uam_lateral_step2.py --config research/dynamic-transitions/configs/five_uam_lateral_step2_bezier.json --output research/dynamic-transitions/runs/<ID>
python scripts/verify_five_uam_lateral_step2.py research/dynamic-transitions/runs/<ID>
python scripts/plot_five_uam_lateral_step2.py --run research/dynamic-transitions/runs/<ID> --output <figure folder>
```

Each script's `--help` lists its exact arguments. Runners refuse to overwrite an
existing run directory.
