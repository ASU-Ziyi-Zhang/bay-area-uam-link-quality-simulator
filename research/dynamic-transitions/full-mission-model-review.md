# Multi-UAM Communication–Policy–Motion Model
## Detailed specification for review before implementation

**Revision:** 2026-08-31, version 3 — bounded-control admission correction  
**Research topic:** dynamic transitions  
**Status:** architecture approved for execution on 2026-08-30; exploratory parameters remain research assumptions, not calibrated operating limits. The first straight-corridor closed-loop implementation and pilot are documented in [R0003](reports/R0003.md). The geographic extension is still pending.

This document replaces the earlier full-mission review draft at this same path. The earlier proposals for mandatory complete five-aircraft groups, mandatory complete 30-second histories, unclassified startup policies, fallback guards, and vertiport-resource scheduling are withdrawn.

**2026-08-31 correction:** the user confirmed that a policy-target jump is handled by the saturated ACC, not an absolute raw-feedback rejection. Sections 8/10/11 and the parameter/check registers below supersede the R0003/R0004 admission rules. Initial insertion checks apply to newly introduced following relationships; inherited source-flow deficits remain recorded and dynamically evaluated. The physical forecast still rejects conflicts. The approved research sequence is [communication geometry → 3D maneuver trade-offs → traffic interaction → capacity](../toyota-research-plan.md). No Bay Area maneuver result is implied by this correction.

The purpose is to make the complete modeling chain inspectable before coding:

~~~text
virtual departure connector
        ↓
corridor entry with initial C
        ↓
actual position → BS link quality → local-group exposure → C/R/F
        ↑                                                 ↓
new position ← longitudinal control ← policy-dependent spacing
        ↑
continuous lane/level transition, if safety and benefit tests pass
        ↓
virtual arrival connector → mission complete
~~~

The original documentation-only revision did not modify scientific code or run experiments. After user approval, the first implementation was added separately; the existing R0001/R0002 remain unchanged historical synthetic mechanism checks.

### Reading guide

- [Sections 1–4](#1-decisions-assumptions-and-research-boundaries) define the scope, aircraft geometry, and departure-to-arrival chain.
- [Sections 5–7](#5-bs-link-quality-model) define link quality, exposure, policy, and spacing.
- [Sections 8–11](#8-longitudinal-control) define ACC, continuous transitions, safety screening, and transition decisions.
- [Sections 12–15](#12-observation-clocks-and-execution-order) define execution order, [parameters](#13-parameter-register), outputs, and verification.
- [Section 16](#16-review-and-approval-checklist) lists the remaining approval items.
- [Section 17](#17-sources-and-provenance) documents sources and their limits.

## 1. Decisions, assumptions, and research boundaries

### 1.1 Status labels

| Label | Meaning |
|---|---|
| Agreed | The user has approved this modeling choice. |
| Inherited | A value or convention retained from the TRB paper or existing simulator for comparison; not necessarily calibrated. |
| Proposed | An explicit implementation convention supplied for this review; approval is still needed. |
| Exploratory | A numerical research setting, not an aviation operating limit. |
| Not modeled | A deliberately excluded process; its effects cannot be claimed as simulated. |

Approval of the architecture does not certify its parameters or establish real-world operating safety.

### 1.2 Agreed architecture

1. Departure and arrival are **virtual geometric connectors**, not a vertiport-operations study.
2. Each aircraft starts with policy **C** as an initialization assumption.
3. A policy group has a **variable number of available members**: the focal aircraft and up to two predecessors and two followers in its flow.
4. Exposure uses **available actual history**, up to the specified trailing-window duration.
5. Policy neighborhoods are organized by **traffic flow**, not by an unrestricted three-dimensional sphere.
6. Safety screening considers **all relevant three-dimensional traffic**, whether or not it belongs to the focal aircraft's policy group.
7. The TRB policy-dependent spacing law becomes the ACC target spacing.
8. Fixed-cruise and ACC modes share the same transition framework.
9. Lateral and vertical motion uses continuous quintic boundary trajectories.
10. A discretionary transition must pass safety/gap checks before communication benefit and traffic disturbance are considered.

### 1.3 What this study does not add

The initial implementation does not model pad counts, passenger processing, charging, turnaround, booking, destination-slot allocation, or commercial vertiport capacity. It does not optimize a new shortest route between airports.

It also does not model aerodynamic interaction, rotor wakes, stochastic packet delivery, finite BS bandwidth allocation, aircraft-generated radio interference, wind, sensor errors, actuator lag, or a certified emergency-avoidance controller.

These omissions matter. In particular:

- Independent BS link calculations do not imply independent traffic motion.
- A point-mass trajectory with no predicted geometric conflict is not an airworthiness or separation certification.
- A modeled operating-policy downgrade is not a simulation of a real communications protocol switching mode.
- Virtual connectors can affect entry timing and nearby traffic; being outside the research focus does not make their trajectories physically irrelevant.

### 1.4 Three different aircraft relationships

| Relationship | Question it answers | Proposed baseline set |
|---|---|---|
| Policy neighborhood | Whose communications jointly determine my operating policy? | Same-flow focal aircraft plus up to two ahead and two behind |
| ACC leader set | Whose longitudinal motion constrains my following control? | Relevant predecessor in the current flow; source/target predecessors during a transition |
| Conflict-check set | Who could conflict with my actual future trajectory? | All relevant aircraft across lanes, levels, and virtual connectors |

A target-flow follower is not the transitioning aircraft's ACC leader. Nevertheless, its required braking and predicted trajectory must be checked before insertion.

A cross-flow aircraft can be safety-relevant without contributing to the focal aircraft's exposure. Conversely, two policy-group members need not be each other's immediate ACC leader.

## 2. Geometry, state, and speed definitions

### 2.1 Configurable flows

Let the lateral offsets and cruise levels be

$$
\mathcal D=\{d_1,\ldots,d_{N_d}\},\qquad
\mathcal H=\{h_1,\ldots,h_{N_h}\}.
$$

A flow is a pair

$$
\ell=(a,b),\qquad (d_\ell,h_\ell)=(d_a,h_b).
$$

The model accepts arbitrary valid lists; it is not restricted to 1×1, 2×2, or 3×3. Lists are sorted, duplicate coordinates are rejected, and neighboring entries define adjacency.

Examples inherited from the mechanism tests:

| Layout | Offsets, m | Levels, m | Number of flows |
|---|---|---|---:|
| Single-flow reference | 0 | 300 | 1 |
| 2×2 mechanism fixture | −250, +250 | 300, 600 | 4 |
| 3×3 mechanism fixture | −500, 0, +500 | 150, 300, 450 | 9 |

The two multi-flow fixtures differ in geometry as well as flow count. Comparing their results alone cannot identify a pure “number of flows” effect.

All traffic in the initial study travels in the same longitudinal direction.

### 2.2 Local straight-corridor coordinates

Use a right-handed local frame:

- s: longitudinal progress, in meters.
- d: lateral offset from the reference corridor, in meters.
- h: vertical coordinate, in meters.

For the straight baseline,

$$
\mathbf r_i=(s_i,d_i,h_i),\qquad v_i=\dot s_i.
$$

The longitudinal gap is reference-point to reference-point:

$$
g_{ij}=s_j-s_i,\qquad s_j>s_i.
$$

It is not a road-vehicle bumper gap. No vehicle length is silently subtracted from the inherited spacing law.

The fixed-cruise baseline holds **longitudinal speed**, not total three-dimensional speed, constant during the corridor phase:

$$
v_i=v_{\rm cruise},\qquad
V_i=\|\dot{\mathbf r}_i\|
=\sqrt{v_i^2+\dot d_i^2+\dot h_i^2}.
$$

Thus an aircraft can have longitudinal speed 50 m/s while its total speed is slightly greater during a lateral or vertical transition. Both quantities must be recorded.

The corresponding Cartesian acceleration magnitude is

$$
A_i=\sqrt{\dot v_i^2+\ddot d_i^2+\ddot h_i^2}.
$$

Longitudinal and transverse component limits alone do not establish a vehicle-specific allowable resultant acceleration.

### 2.3 Geographic corridor mapping

A curved map must not be treated as a straight coordinate system. For a smooth planar reference curve parameterized by arc length s, write

$$
\mathbf r_{xy}(s,d)=\mathbf r_0(s)+d\,\mathbf n(s),
\qquad J(s,d)=1-\kappa(s)d.
$$

Here \(\mathbf n\) is the oriented unit normal and \(\kappa\) is signed curvature. A valid local offset map requires \(J>0\) and no ambiguous folds or self-intersections in the modeled region.

Then

$$
V^2=J(s,d)^2\dot s^2+\dot d^2+\dot h^2.
$$

On a fixed offset, physical arc length is

$$
L_d(s)=\int_0^s J(\sigma,d)\,d\sigma.
$$

**Proposed geographic convention:** use physical longitudinal tangential speed \(v=J(s,d)\dot s\), reducing to \(\dot s\) in the straight case. Fixed cruise then means \(v=50\) m/s, and longitudinal integration uses \(\dot s=v/J\).

For a fixed flow, use \(L_d(s_j)-L_d(s_i)\) for gap and its time derivative for relative speed. For a transitioning aircraft, one consistent proposed comparison coordinate is its instantaneous offset curve:

$$
g_{ij}(t)=\int_{s_i(t)}^{s_j(t)}[1-\kappa(\sigma)d_i(t)]\,d\sigma,
$$

$$
\dot g_{ij}=
J(s_j,d_i)\dot s_j-v_i
-\dot d_i\int_{s_i}^{s_j}\kappa(\sigma)\,d\sigma.
$$

Only selected source/target leaders are compared this way; projecting another aircraft does not make it a leader. This gap is an along-corridor control coordinate, not a three-dimensional closest-approach distance. Safety always uses actual Cartesian trajectories.

The formulas above are a proposed geometric extension derived for this study, not a feature already present in the dynamic prototype. They require explicit numerical and geometry verification. On curves, the world-frame acceleration is

$$
\ddot{\mathbf r}_{xy}
=(\dot v-\kappa\dot s\,\dot d)\mathbf T
+(\kappa\dot s\,v+\ddot d)\mathbf n.
$$

Consequently, limiting the ACC command \(\dot v\) alone does not limit the total physical acceleration.

The existing geographic polyline cannot be assumed differentiable at its vertices. A smooth, documented corridor representation and a checked projection are prerequisites for geographic dynamic experiments. Any geometric smoothing must be versioned and its effects on the static radio baseline reported. Do not silently substitute a new path or reinterpret the old dashboard's speed.

### 2.4 Vertical datum

For a local virtual-origin experiment, set the origin height to zero in a declared synthetic vertical datum. This does not assert that a real airport lies at zero elevation.

For Bay Area radio calculations, aircraft and BS vertical coordinates must use the same documented convention. The existing kernel consumes its numerical BS height column directly; this is not proof that a terrain-corrected absolute antenna altitude has been established.

Terrain-following, ground clearance, and building avoidance are not validated by the current dataset. A geographic run must preserve and state the inherited height convention, or introduce a separately documented elevation correction.

### 2.5 Minimum aircraft state

Save at least: aircraft ID; phase; actual position; longitudinal and Cartesian velocity/acceleration; assigned/current/source/target flow; transition timing; relevant leader/follower IDs; current policy and its source; radio observations; historical group memberships; group exposure; actual and desired gaps; request/release/entry/exit/completion times; and any failure or rejection reason.

A flow identifier is a logical membership label. It must not overwrite the actual spatial position during a transition.

## 3. Departure, entry, exit, and arrival

### 3.1 Minimal full-mission chain

| Phase | Motion | Communication/policy treatment | Transition to next phase |
|---|---|---|---|
| SCHEDULED | No airborne aircraft yet | No fabricated observations | Scheduled release passes boundary feasibility screening |
| DEPARTURE_CONNECTOR | Continuous motion from virtual origin to assigned flow | Record actual radio as connector diagnostics; proposed initialization label C | Reach entry state continuously |
| CORRIDOR | Fixed cruise or policy-spacing ACC | Actual same-flow group exposure drives C/R/F | Optional transition or corridor exit |
| FLOW_TRANSITION | Longitudinal control plus continuous lateral/vertical motion | Actual positions drive radio; proposed group ownership rule in Section 6 | Reach target flow without resetting state/history |
| ARRIVAL_CONNECTOR | Continuous motion to virtual destination | Diagnostic radio; no new discretionary lane decisions | Reach prescribed endpoint state |
| COMPLETE | No longer airborne | No further observations | Mission ends |

**Proposed phase convention:** operational group exposure is assessed during CORRIDOR and FLOW_TRANSITION. Connector radio is retained separately and is not inserted into the corridor's exposure history. This isolates the corridor research question while retaining a complete displayed flight.

This is a phase-boundary proposal, not a requirement imposed by the agreed “available history” rule. It is listed for approval in Section 16.

Initial C is an initialization label, not a claim that startup links are good. Once the first eligible corridor observation is available, actual exposure may immediately produce R or F. There is no mandatory 30-second C grace period.

### 3.2 Requested release schedule and initial assignment

For a deterministic demand \(\lambda_{\rm total}>0\), measured in aircraft/hour,

$$
t_i^{\rm request}=t_0+(i-1)\frac{3600}{\lambda_{\rm total}}.
$$

Proposed first assignment rule: deterministic round-robin over the configured flows, using a saved flow order. A single-flow control assigns everyone to that flow. More advanced radio-aware initial assignment is a separate experiment.

Distinguish:

- total demand held fixed as flows are added;
- demand per flow held fixed, so total demand increases;
- actual corridor entry times, which can differ because connector durations and feasibility differ.

The assignment rule itself must not quietly optimize communications in a comparison intended to isolate in-flight lane changes.

### 3.3 Release feasibility without a vertiport model

A shared virtual origin does not authorize spawning several airborne aircraft at the same point and time. Proposed release behavior:

1. Construct the assigned connector and its intended corridor-entry state.
2. Screen its actual three-dimensional path against relevant airborne traffic.
3. Check that the projected insertion and its normal-control response are feasible.
4. Release if the checks pass.
5. Otherwise retain the request and record a deferred release; retry at a specified boundary-check interval.

This is a boundary-admission guard, not a pad-service or vertiport-capacity model. It has no pad count, turnaround time, commercial booking, or destination-resource reservation.

The requested schedule and actual release/entry records must remain visible. A run limited by this artificial connector/admission boundary cannot be presented as a measurement of unrestricted corridor capacity.

For an initial low-density pilot, requests should be chosen to avoid making virtual-origin admission the dominant phenomenon. Dense cases then explicitly report its effect.

### 3.4 Initial policy and entry spacing

The initial policy is C, but neither C nor \(\tau_C=15\) s specifies the release interval.

For a same-speed stream,

$$
g=vH_{\rm entry}.
$$

At 50 m/s, a 15-second entry interval corresponds to 750 m, while the inherited C target is 1367.5 m. Therefore “initial C” does not imply that arbitrary 15-second entries satisfy the C target.

The proposed admission guard checks the full target spacing and forecast response, not just the \(\tau v\) component. It should not create an inadmissible insertion and then claim that ACC guarantees recovery.

### 3.5 Arrival boundary

Define corridor exit before the final virtual connector. At exit, match the aircraft's actual position and velocity; do not reset speed to cruise before descent.

There is no destination pad-occupancy model. Nevertheless, two arrival connectors that converge to the same point must be checked for overlapping airborne occupancy. Endpoint removal cannot be used to hide a conflict just before completion.

Proposed first failure behavior: if a connector cannot be made feasible within the modeled normal-control assumptions, mark the mission/run infeasible and save the state. Do not invent holding patterns, emergency maneuvers, or a destination queue. Geometry and demand should be screened in pilot tests before larger runs.

## 4. Common trajectory generator

### 4.1 Quintic transverse displacement

For a maneuver starting at \(t_0\), duration D, and normalized time \(u=(t-t_0)/D\in[0,1]\),

$$
f(u)=10u^3-15u^4+6u^5.
$$

For a lateral or vertical coordinate \(q\in\{d,h\}\),

$$
q(t)=q_0+\Delta q\,f(u),
\quad
\dot q=\frac{\Delta q}{D}f'(u),
\quad
\ddot q=\frac{\Delta q}{D^2}f''(u),
$$

$$
f'(u)=30u^2(1-u)^2,\qquad
f''(u)=60u-180u^2+120u^3.
$$

The endpoints have specified positions and zero transverse velocity and acceleration. Six boundary conditions determine the six coefficients of a general quintic. This is the mathematical rationale, as explained in the trajectory-generation treatment in [Modern Robotics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/).

The choice does not itself enforce flight dynamics. In particular, jerk need not be continuous at a splice, and a smooth coordinate change can still violate a speed, acceleration, curvature, or separation limit.

### 4.2 Duration from component limits

The normalized extrema are

$$
\max f'(u)=\frac{15}{8},\qquad
\max |f''(u)|=\frac{10}{\sqrt 3}.
$$

For zero transverse endpoint velocity/acceleration, choose at least

$$
D\ge
\max\left[
D_{\min},
\max_{q:\Delta q\ne0}\frac{15|\Delta q|}{8v_{q,\max}},
\max_{q:\Delta q\ne0}
\sqrt{\frac{10|\Delta q|}{\sqrt3\,a_{q,\max}}}
\right].
$$

A zero-displacement component imposes no duration bound. The duration is rounded upward to an integration boundary, never downward.

Illustrations using the old exploratory component limits:

| Motion | Displacement | Dominating speed bound | Resulting minimum duration, including other stated bounds |
|---|---:|---:|---:|
| Lateral change | 500 m | 8 m/s | 117.1875 s |
| Vertical change | 300 m | 3 m/s | 187.5 s |

These are calculated examples, not newly simulated flight times. They show why “minimum maneuver duration 30 s” does not mean every maneuver lasts 30 s.

For curved routes, actual combined trajectory constraints must also be checked.

### 4.3 General boundary polynomial for connectors

Departure and arrival cannot use zero longitudinal velocity at both ends if they must join a moving stream. For any Cartesian or local coordinate p, specify endpoint position, velocity, and acceleration:

$$
p(u)=p_0+Dv_0u+\frac12D^2a_0u^2+c_3u^3+c_4u^4+c_5u^5.
$$

Let

$$
A=p_1-p_0-Dv_0-\frac12D^2a_0,\qquad
B=D(v_1-v_0)-D^2a_0,\qquad
C=D^2(a_1-a_0).
$$

Then

$$
c_3=10A-4B+\frac12C,\qquad
c_4=-15A+7B-C,\qquad
c_5=6A-3B+\frac12C.
$$

This gives the required p, \(\dot p\), and \(\ddot p\) at both endpoints. Endpoint constraints alone are insufficient: the full curve must be checked for overshoot, reverse progress, vertical undershoot, and dynamic limits.

Proposed local departure boundary conditions:

$$
(s,d,h)_{\rm start}=(0,0,0),\qquad
(\dot s,\dot d,\dot h)_{\rm start}=(0,0,0),
$$

$$
(s,d,h)_{\rm entry}=(s_{\rm in},d_\ell,h_\ell),\qquad
(\dot s,\dot d,\dot h)_{\rm entry}=(v_{\rm entry},0,0).
$$

Use zero endpoint acceleration for the nominal isolated connector. For arrival, start from the actual exit state and terminate at the virtual destination with zero velocity and acceleration.

On a curved route, endpoint **Cartesian** velocity and acceleration must match the corridor tangent and curvature, not merely set every acceleration component to zero. An ACC mode/leader change can still introduce a bounded acceleration jump; this version does not promise globally continuous acceleration or jerk across all controller events.

### 4.4 Worked origin-to-flow example

Consider an isolated aircraft assigned to \(d=-250\) m and \(h=300\) m. As an illustrative boundary choice, let \(D=240\) s and \(v_{\rm entry}=50\) m/s, with \(s_{\rm in}=v_{\rm entry}D/2=6000\) m.

The general boundary polynomial simplifies to

$$
s(u)=v_{\rm entry}D\left(u^3-\frac12u^4\right),\qquad
\dot s=v_{\rm entry}(3u^2-2u^3),
$$

$$
d(u)=-250f(u),\qquad h(u)=300f(u).
$$

The longitudinal polynomial is a special case of the general quintic with a zero fifth-order coefficient. The aircraft starts at rest and arrives at \((6000,-250,300)\) with longitudinal speed 50 m/s and zero lateral/vertical speed.

This 240-second connector is an illustration, not a locked experiment parameter or an actual departure procedure. Its long duration illustrates how a conservative vertical-speed limit influences connector length.

The same geometric method connects a corridor exit to a virtual landing point. When the initial longitudinal acceleration is nonzero, use the full boundary polynomial rather than the simplified example.

### 4.5 One controller per degree of freedom

During corridor motion, longitudinal progress comes from the selected fixed-cruise/ACC module; the transverse coordinates come from the maneuver generator.

During a prescribed connector, the connector supplies the full trajectory. Do not simultaneously overwrite its longitudinal coordinate with an incompatible ACC update. Its feasibility must include other aircraft's responses. If the connector prediction becomes infeasible, perform an explicitly defined state-matching replan or report failure; no silent teleportation or controller override is allowed.

## 5. BS link-quality model

### 5.1 Position-to-link calculation

For aircraft i and BS b, use common metric coordinates:

$$
D_{ib}(t)=\|\mathbf r_i(t)-\mathbf r_b\|.
$$

The inherited deterministic propagation kernel is

$$
PL_{ib}=28+22\log_{10}(D_{ib}/{\rm m})
+20\log_{10}(f_c/{\rm GHz}),
$$

$$
P_{ib}^{\rm dBm}
=P_{\rm EIRP}^{\rm dBm}+G_{\rm rx}^{\rm dB}-PL_{ib}.
$$

This is the formula currently implemented in the [radio kernel](../../src/capacity_policy/radio.py). It is a planning approximation, not a new propagation calibration over every departure height.

Coincident aircraft/BS coordinates are physically invalid for this model and must be rejected or explicitly handled as a documented model limit. A numerical log floor is not a near-field propagation model.

### 5.2 Serving and interfering BS sets

For the current [airport-to-airport scenario](../../scenarios/airport_to_airport/scenario.json):

1. Select the three nearest BSs by three-dimensional distance.
2. The strongest received signal in that set is the serving BS.
3. The other two sites contribute interference.
4. Sites outside the selected set do not contribute to that run's interference sum.

The inventory contains 12 retained physical sites. “Three-site set” does not mean three jointly serving transmitters.

Let \(\mathcal B_i\) be the selected set and \(b_i^*\) its strongest site. In milliwatts,

$$
p_{ib}=10^{P_{ib}^{\rm dBm}/10},\qquad n=10^{N^{\rm dBm}/10},
$$

$$
\gamma_i=
\frac{p_{ib_i^*}}
{\sum_{b\in\mathcal B_i\setminus\{b_i^*\}}p_{ib}+n},
\qquad
{\rm SINR}_i^{\rm dB}=10\log_{10}\gamma_i.
$$

The baseline uses \(f_c=5\) GHz, EIRP 46 dBm, receive gain 0 dB, and noise −99 dBm.

Association is an instantaneous modeled maximum-power choice. Handover delay, packet interruption, antenna orientation, and load-dependent scheduling are not simulated.

### 5.3 RSRP versus SINR

The displayed per-resource-element RSRP is

$$
{\rm RSRP}_i=P_{ib_i^*}^{\rm dBm}-10\log_{10}(N_{\rm RE}),
\qquad N_{\rm RE}=300.
$$

It is a reporting normalization in this kernel. The SINR calculation uses carrier powers and its matching carrier-noise convention. Do not subtract the per-RE normalization from only one term of the SINR ratio.

C/R/F classification uses the common SINR threshold below, not an additional RSRP gate.

### 5.4 Per-aircraft link state

$$
\chi_i(t)=\mathbf1[{\rm SINR}_i^{\rm dB}(t)\ge\Theta],
\qquad \Theta=-1.5\ {\rm dB}.
$$

Equivalently, bad-link indicator \(b_i=1-\chi_i\). Exact threshold equality is treated as link-good.

The threshold is an inherited study parameter, not a universal receiver requirement. All members use their own position, serving site, and SINR.

### 5.5 Relation to the TRB and static multi-flow studies

The current Bay Area nearest-three-site interference convention differs from the paper's all-site co-channel convention. Preserve each as a separately named configuration; do not claim numerical equivalence.

There is no utilization penalty or aircraft-to-aircraft radio interference in this baseline. Two aircraft at the same position in the same radio configuration therefore receive the same modeled link quality regardless of how many other flows are open.

Once motion and policy are coupled, other aircraft can still change a focal aircraft's exposure, control, and future positions. Static radio independence does not remove this operational coupling.

## 6. Variable-size local groups, exposure, and C/R/F

### 6.1 Same-flow neighborhood

At each policy-observation time, sort eligible aircraft in each logical flow by current longitudinal position, not by ID or original departure order.

For focal aircraft i, let \(\mathcal P_i\) contain its nearest two available predecessors and \(\mathcal F_i\) its nearest two available followers. Then

$$
G_i(t)=\{i\}\cup\mathcal P_i(t)\cup\mathcal F_i(t),\qquad
1\le |G_i(t)|\le5.
$$

If two predecessors are unavailable, do not compensate by taking extra followers.

| Position in a sufficiently long flow | Available group |
|---|---|
| First aircraft | Self + up to two behind |
| Second aircraft | One ahead + self + up to two behind |
| Interior aircraft | Up to two ahead + self + up to two behind |
| Last aircraft | Up to two ahead + self |
| Only aircraft | Self |

These are overlapping focal groups, not disjoint batches of five. Each focal aircraft receives its own policy.

The group is topological: “two ahead” is not a fixed distance in meters. Adding a spherical neighborhood would change the policy hypothesis and its exposure denominator; it is not included in this baseline.

### 6.2 Instantaneous group bad-link fraction

At observation time \(t_k\),

$$
e_i(t_k)=\frac{1}{|G_i(t_k)|}
\sum_{j\in G_i(t_k)} b_j(t_k).
$$

Example: one bad link in a three-aircraft boundary group gives \(e_i=1/3\), not \(1/5\). Nonexistent aircraft contribute neither good nor bad samples.

### 6.3 Available-history exposure

Let \(\mathcal K_i(t_k)\) be the set of actual eligible observation times in the trailing interval \([t_k-W,t_k]\), with \(W=30\) s.

$$
\zeta_i(t_k)=
\frac{1}{|\mathcal K_i(t_k)|}
\sum_{t_r\in\mathcal K_i(t_k)} e_i(t_r).
$$

Use whatever actual history exists. There is no requirement to wait until \(W\) seconds have elapsed.

For uniform 5-second observations, a full inclusive window contains seven samples. A new entrant can initially have one sample, then two, and so on. The sample count and group size are recorded as diagnostics, not eligibility gates.

Crucially, this is the **time average of the group fractions at their original observation times**. It is not a pooled count divided by the total number of aircraft-time observations when group sizes vary.

For example, two observations with one bad link out of three and zero bad links out of five give

$$
\zeta=\frac12\left(\frac13+0\right)=\frac16,
$$

not \(1/8\). This matches the available-group convention in the [multi-UAM baseline runner](../../src/uam_simulator/group_runner.py).

Past fractions retain their past memberships. Do not recompute yesterday's group using today's neighbors.

### 6.4 Policy map

$$
m_i(t_k)=
\begin{cases}
C,&\zeta_i(t_k)\le0.05,\\
R,&0.05<\zeta_i(t_k)\le0.10,\\
F,&\zeta_i(t_k)>0.10.
\end{cases}
$$

C means coordinated, R reactive, and F fallback. Between policy events, hold the latest policy while ACC continues at the finer motion step.

At equal group size five and seven observations, one bad member-time contributes \(1/35\approx2.857\%\); two contribute \(5.714\%\), and four contribute \(11.429\%\). These illustrate C, R, and F outcomes. With variable group size, this simple denominator no longer applies.

The threshold tolerances are exposure fractions. They are not the fraction of aircraft already labeled F, and they are not a second total-flight weak-link metric.

### 6.5 Initialization and missing data

At creation, initialize \(m_i=C\) and label its source as initialization. At the first actual eligible sample, apply the exposure rule immediately. For example, an isolated entrant with a bad first sample has \(\zeta=1\) and changes to F.

Do not:

- fabricate pre-entry good-link samples;
- fill missing neighbors with good links;
- require a complete five-aircraft group;
- assign an unclassified or fallback-guard operating mode solely because history is short.

A numerical failure or genuinely missing radio measurement is different from short history. In this deterministic first version, treat invalid numerical data as an explicit diagnostic/run failure; do not silently label it good.

### 6.6 Flow ownership during a transition — proposed convention

Keep the focal aircraft's **policy membership** in its source flow until the transverse maneuver is complete. At completion, move its membership to the target flow before the next eligible group observation.

During that same maneuver:

- radio is calculated at the actual intermediate position;
- ACC and conflict screening consider relevant source/target traffic;
- the aircraft is counted once, not once per occupied flow, in policy totals;
- source and target groups are rebuilt from their actual logical memberships;
- its own exposure history is retained across the change of flow.

An aircraft entering a well-connected target flow therefore does not instantly inherit the target predecessor's C label. Its preceding window may still contain poor observations from its source flow.

This convention separates policy membership from safety occupancy and avoids an arbitrary nearest-lane switch halfway through a maneuver. It is an explicit proposal to approve, not a user-confirmed detail yet.

### 6.7 Paper regression versus operational policy

The paper's complete-neighborhood/complete-window estimator remains a separate reference calculation in the [original policy kernel](../../src/capacity_policy/policy.py). It is not the operational boundary rule for this extension.

In a static case with complete five-aircraft groups, full seven-sample windows, matched sampling, and matched radio assumptions, both formulations should agree. Boundary and startup differences are intentional and must be labeled.

## 7. Policy-dependent spacing

### 7.1 One spacing target for policy and ACC

For policy \(m\in\{C,R,F\}\),

$$
S_m(v)=d_0+\tau_m v+c_bv^2,
$$

with inherited values

$$
d_0=200\ {\rm m},\quad
(\tau_C,\tau_R,\tau_F)=(15,30,60)\ {\rm s},\quad
c_b=0.167\ {\rm s^2/m}.
$$

The focal aircraft's policy determines its target gap to the relevant predecessor:

$$
g_i^{\rm des}(t)=S_{m_i(t)}(v_i(t)).
$$

The prototype's independent \(200+15v\) ACC target is replaced in the proposed next implementation; it is not kept in parallel.

The extra 300 m longitudinal floor from the mechanism tests is not inherited automatically. Any independent hard separation condition must have a separate name, provenance, and feasibility check.

### 7.2 Meaning of each term

| Term | Role in this study | What it is not |
|---|---|---|
| \(d_0\) | Base/low-speed planning separation | A universal 3D collision-protection radius |
| \(\tau_m v\) | Policy-dependent speed-scaled spacing allowance | A literal controller delay or a release interval |
| \(c_bv^2\) | Inherited quadratic planning buffer | A calibrated stopping-distance law for the selected aircraft |
| \(S_m(v)\) | ACC operating target | A guarantee that every transient gap satisfies it |

The parameters were not calibrated to a specific eVTOL. Preserve them as a transparent reference and examine sensitivity.

If \(c_b=1/(2b)\) were interpreted literally, it would imply \(b\approx2.994\) m/s². That is not the proposed normal deceleration limit of 2 m/s². This version therefore treats the quadratic term as a planning buffer, not a physical braking derivation.

### 7.3 Numerical reference at 50 m/s

| Policy | Target spacing, m | Same-speed spacing/speed, s | Conditional single-flow planning rate, aircraft/h |
|---|---:|---:|---:|
| C | 1367.5 | 27.35 | 131.63 |
| R | 2117.5 | 42.35 | 85.01 |
| F | 3617.5 | 72.35 | 49.76 |

The rate is \(3600v/S_m(v)\), not a result from the proposed dynamic simulation. Effective time headway is

$$
H_m(v)=\frac{S_m(v)}v=\frac{d_0}{v}+\tau_m+c_bv,\qquad v>0.
$$

This explains why \(\tau_C=15\) s does not mean the full cruising time headway is 15 s.

### 7.4 Policy changes do not teleport gaps

A C-to-R change at 50 m/s increases the target by 750 m; R-to-F adds another 1500 m. The actual gap and velocity remain continuous.

Record the policy-spacing deficit

$$
\delta_i(t)=\max[0,S_{m_i(t)}(v_i(t))-g_i(t)].
$$

A deficit is a failure to meet the current planning target. It is not automatically a geometric collision, nor should it be hidden just because the acceleration command has been clipped.

## 8. Longitudinal control

### 8.1 Fixed-cruise baseline

Inside the corridor, including a discretionary transition,

$$
v_i=v_{\rm cruise},\qquad \dot v_i=0.
$$

Policy is still computed. It changes the conditional planning-spacing/capacity outputs but does not change longitudinal motion in this control condition.

This is intentionally an open-motion comparator. It isolates the effect of communication-driven transition decisions without ACC adaptation. New insertion gaps and continuous 3D screening still apply to the actual fixed-speed trajectory. A predicted conflict cannot be hidden by keeping speed fixed. Conversely, the hypothetical ACC raw feedback is not a braking requirement for this open-motion comparator: existing policy-spacing deficits are reported, not reclassified as collisions.

Connector acceleration and deceleration are separate prescribed boundary motions, not a violation of the definition of fixed *corridor* cruise.

### 8.2 ACC control law

For each relevant predecessor j, define

$$
e_{ij}=g_{ij}-S_{m_i}(v_i).
$$

For the straight corridor, \(\dot g_{ij}=v_j-v_i\). For the geographic proposal, use the matched gap derivative in Section 2.3.

$$
a_{ij}^{\rm follow}=k_g e_{ij}+k_{\rm rel}\dot g_{ij},
\qquad
a_i^{\rm cruise}=k_v(v_{\rm cruise}-v_i).
$$

Let \(\mathcal L_i\) be the relevant leader set. Then

$$
a_i^{\rm raw}=
\begin{cases}
a_i^{\rm cruise},&\mathcal L_i=\varnothing,\\
\min\left(a_i^{\rm cruise},
\min_{j\in\mathcal L_i}a_{ij}^{\rm follow}\right),&\text{otherwise},
\end{cases}
$$

$$
a_i^{\rm cmd}=\operatorname{clip}
(a_i^{\rm raw},-b_{\rm normal},a_{\rm normal}).
$$

This linear spacing-error/relative-speed feedback with a cruise-control branch has a road-ACC modeling precedent in [He et al., Section 2.1.3, Eq. 6](https://arxiv.org/pdf/2107.07832). Coupling it to the present TRB spacing law and UAM geometry is our proposed extension. That source does not calibrate our gains or validate an aircraft controller.

### 8.3 Proposed controller parameters

| Parameter | Candidate value | Units | Interpretation |
|---|---:|---|---|
| \(k_v\) | 0.2 | s⁻¹ | Return toward cruise speed |
| \(k_g\) | 0.005 | s⁻² | Response to spacing error |
| \(k_{\rm rel}\) | 0.4 | s⁻¹ | Response to gap closing/opening |
| \(a_{\rm normal}\) | 1.5 | m/s² | Normal positive longitudinal-command limit |
| \(b_{\rm normal}\) | 2.0 | m/s² | Magnitude of normal negative limit |

The gains are exploratory values from the old mechanism prototype, not literature-calibrated UAM values. They must be rechecked after the spacing law changes.

The −2/+1.5 range comes from Sergei's March correspondence, as summarized in Section 17. Treat its use as longitudinal component limits as a proposed interpretation: the email did not explicitly distinguish component from resultant acceleration.

Emergency ±5 m/s² is recorded as correspondence context only. It is not available to make normal lane-change candidates pass.

### 8.4 Sparse and dense traffic examples

At cruise speed with no leader, the command is zero. With an equal-speed leader farther away than the target, the following command is positive but the cruise branch is zero; the minimum remains zero. The aircraft does not accelerate above cruise merely to close a large gap.

At \(v_i=v_j=50\) m/s and \(g=1200\) m under C,

$$
a^{\rm follow}=0.005(1200-1367.5)=-0.8375\ {\rm m/s^2}.
$$

For a C equilibrium gap of 1367.5 m, an instantaneous downgrade to R produces

$$
a^{\rm raw}=0.005(1367.5-2117.5)=-3.75\ {\rm m/s^2},
$$

which clips to −2 m/s² under the proposed limit. This is an initial response calculation, not a claim that constant −2 braking continues until 750 m has opened. As speed and gap change, the target, relative-speed term, and command change.

An immediate upgrade can remove the spacing constraint, but the cruise branch and acceleration limit still control the return to cruise.

### 8.5 Leader selection during a transition

Proposed conservative first rule: during the maneuver, consider relevant source- and target-flow predecessors. Apply the most restrictive normal-control demand after putting gaps and relative speeds into the same declared comparison coordinate.

The source/target followers also predict their response to the transitioning aircraft. Their own policy, not the transitioning aircraft's policy, determines their desired spacing.

Do not select the nearest aircraft by Euclidean distance as an ACC leader. A nearby aircraft in another level may have no longitudinal-following relation.

### 8.6 Time integration and policy switching

Use a common fleet state at time \(t_k\) to compute all acceleration commands. Advance synchronously, rather than updating one aircraft before calculating the next aircraft's command.

For a straight segment with constant acceleration over a step,

$$
v_{k+1}=v_k+a_k\Delta t,\qquad
s_{k+1}=s_k+v_k\Delta t+\frac12a_k\Delta t^2.
$$

If a speed boundary is reached partway through a step, split the step at the crossing and integrate the remainder consistently. Do not clip velocity afterward while leaving an incompatible displacement.

Proposed mathematical bounds are \(0\le v\le v_{\rm cruise}\) for the point-mass controller. Reaching zero in a corridor is a flagged stalled case, not evidence that a selected aircraft can safely hover there. An operational minimum-speed envelope requires separate vehicle information.

Policy is held between policy updates; the spacing target is recalculated with the current speed at each ACC step. A policy change can cause a bounded acceleration jump. No jerk limit or actuator lag is currently included.

This controller is not a collision-avoidance proof and does not guarantee string stability. Required tests include equilibrium, sparse/dense following, leader braking, policy downgrades/recoveries, and insertion disturbances.


## 9. Continuous lane and level transitions

### 9.1 Candidate moves

For flow \((a,b)\), eligible neighbors are

$$
(a-1,b),\ (a+1,b),\ (a,b-1),\ (a,b+1),
$$

when the indices exist.

A lateral change holds h fixed. A vertical change holds d fixed. Both retain longitudinal motion throughout the maneuver.

The initial design uses one adjacent-axis move at a time. Moving from the lower-left flow to the upper-right flow requires two separately approved transitions, such as lateral then vertical. The intermediate flow must be feasible. A directly diagonal move is a later variant, not a hidden shortcut.

“Climbing to another cruise level” is a flow transition; it is distinct from the initial virtual departure connector.

### 9.2 Eligibility

Proposed conditions before evaluating a discretionary move:

- the aircraft is in the corridor phase;
- it has no maneuver already in progress;
- its cooldown has expired;
- the target flow exists;
- a complete candidate trajectory can be constructed;
- the remaining corridor and forecast support cover the maneuver and a post-maneuver assessment interval.

An initial C label alone is neither an incentive nor a prohibition. A C aircraft may consider a transition if a forecast predicts a meaningful deterioration without it.

### 9.3 Occupancy and handover

Reserve source/target *traffic occupancy* for conflict coordination during the complete maneuver; this does not duplicate its policy membership.

Only after the actual transverse endpoint is reached are d/h snapped within numerical tolerance to the declared target coordinates. A snap is permitted only to remove integration roundoff, not to skip unfinished motion.

At completion, change logical policy ownership, retain exposure history, and start cooldown. No second move begins until the next eligible decision event after cooldown.

The conservative dual-flow occupancy approximation may cause earlier braking or fewer accepted moves than a more detailed swept-volume interaction model. Report it as a modeling choice.

## 10. Safety and gap acceptance

### 10.1 Separate constraints

Safety screening contains distinct checks:

1. Longitudinal insertion feasibility relative to relevant predecessors/followers.
2. Three-dimensional separation throughout the maneuver and its aftermath.
3. Normal acceleration/deceleration feasibility.
4. Transverse velocity/acceleration and geometric feasibility.
5. Compatibility with already accepted maneuvers and boundary connectors.

Policy target spacing, hard separation, and aircraft-performance limits must be separately named in the output.

### 10.2 Policy-aware insertion checks

For an intended insertion between target-flow predecessor p and follower f, the basic target-gap comparison at the projected insertion state is

$$
g_{ip}\ \text{versus}\ S_{m_i}(v_i),
\qquad
g_{fi}\ \text{versus}\ S_{m_f}(v_f).
$$

Each side uses the follower's own predicted policy. Source-flow neighbors must also be checked as their leaders and group memberships change.

**Proposed admission convention:** do not intentionally create a new gap below the receiving follower's current target at initial insertion. Subsequent target changes are evaluated through the predicted ACC response and reported deficits rather than treated as instantaneous gap jumps.

**Implemented correction, 2026-08-31:** compare the follower–leader ID pairs immediately before and after adding the candidate's target occupancy. Apply the initial target-gap test only to pairs newly introduced by that action, using each follower's current policy and speed. An unchanged source-flow pair is not a new insertion, even if its target gap is currently unmet. Log both follower and leader IDs, actual gap, and desired gap on rejection. Existing maneuvers are included before this comparison, so serial decisions respect already accepted target occupancy. Equal longitudinal positions do not form a strict predecessor pair; the continuous 3D forecast remains responsible for detecting such merging conflicts.

Since physical interaction develops continuously, a start/end gap test is not sufficient. The full occupancy and motion predictions remain authoritative. If the source or target flow is so dense that normal control cannot accommodate insertion, reject the move.

### 10.3 Bounded-control forecast and incremental braking criterion

The MOBIL author description separates an incentive test from a target-follower braking test; see the [MOBIL model explanation](https://traffic-simulation.de/info/info_MOBIL.html). We borrow that safety idea, not its road parameter values or original acceleration-benefit objective.

The R0003/R0004 absolute **raw-demand** gate is withdrawn. Raw feedback is a diagnostic proportional-control error, not a minimum physically necessary braking acceleration. Both staying and changing are forecast with the same saturated control law, normal limits, speed bounds, and continuous interval conflict checks.

Let \(\mathcal A_i\) contain the focal aircraft and affected source/target followers. Define executed braking magnitudes on the overlapping forecast interval:

$$
b_j^{(0)}(\tau)=\max(0,-a_j^{\rm cmd,stay}(\tau)),\qquad
b_j^{(c)}(\tau)=\max(0,-a_j^{\rm cmd,candidate}(\tau)).
$$

An optional stricter voluntary-braking screen rejects only when **both** conditions hold:

$$
b_j^{(c)}(\tau)>b_{\rm accept}
\quad\text{and}\quad
b_j^{(c)}(\tau)>b_j^{(0)}(\tau),\qquad
0<b_{\rm accept}\le b_{\rm normal}.
$$

Compare piecewise-constant commands on exact overlapping time intervals, not array indices: branch event times can differ. The implementation uses a numerical tolerance of 1e-8 m/s². An inherited −2 m/s² response in both branches, or a candidate that reduces braking from −2 to −1.8, is not rejected by this screen.

At the default \(b_{\rm accept}=b_{\rm normal}=2\) m/s², this screen adds no restriction beyond the command limit. That is intentional and is **not** a safety proof. Safety screening instead evaluates whether the actual bounded trajectories satisfy new-insertion and 3D protection checks. If bounded braking cannot avoid a predicted conflict, reject the candidate; do not increase the limit or silently execute an emergency maneuver.

The retained configuration field `accept_braking_mps2` now has the above executed-command/paired-baseline meaning. A stricter 1.5 m/s² value remains an optional future sensitivity, not the current default and not an aviation standard. Raw and executed accelerations and all policy-spacing deficits remain logged.

The non-transition baseline is also forecast. If it is already infeasible, the discretionary lane-change module does not become an emergency escape planner. Record baseline infeasibility separately.

### 10.4 Three-dimensional separation function

A configurable diagnostic protection model may use

$$
\Phi_{ij}(t)=
\frac{\|\mathbf r_{i,xy}(t)-\mathbf r_{j,xy}(t)\|^2}{R_H^2}
+
\frac{(h_i(t)-h_j(t))^2}{R_V^2}.
$$

The corresponding modeled separation condition is

$$
\Phi_{ij}(t)\ge1.
$$

Here \(R_H,R_V\) are **pairwise separation semiaxes**, not radii of two aircraft volumes that should then be summed again. This ellipsoid permits combined horizontal/vertical separation; it is not equivalent to a cylindrical “horizontal threshold OR vertical threshold” rule.

The ellipsoid is an exploratory choice, not a regulation. A cylinder or another protection model can produce different outcomes and must have its own configuration and provenance.

The old \(R_H=200\) m, \(R_V=100\) m values remain mechanism-test settings only. There is **no approved operational separation value** in this review. They must not be silently promoted into a formal research baseline without being explicitly identified as assumptions.

For a first modest fleet size, checking every airborne pair avoids ambiguity in “relevant traffic.” A later spatial broad phase is allowed only if it conservatively retains every pair whose reachable trajectories could violate the condition, including cross-level and connector traffic.

### 10.5 Continuous interval checks

A check only at discrete samples can miss a conflict between samples. For a polynomial or piecewise-polynomial forecast, minimize \(\Phi_{ij}\) over each integration interval by checking endpoints and interior stationary points; alternatively use a justified conservative bounding method.

Non-polynomial geographic mapping requires a correspondingly conservative numerical method. A halved time step is useful convergence evidence, not a proof of continuous-time separation.

Perform the same separation check during execution, not only at decision time.

### 10.6 Curvature and the 8000 m discussion

For a differentiable planar actual trajectory,

$$
\kappa_{\rm path}
=\frac{\dot x\ddot y-\dot y\ddot x}
{(\dot x^2+\dot y^2)^{3/2}},
\qquad
R_{\rm path}=1/|\kappa_{\rm path}|.
$$

Curvature is evaluated only when planar speed is nonzero. At low speed, explicit acceleration constraints are more meaningful than dividing by a vanishing speed.

The 8000 m value from Sergei's August follow-up is a **corridor curvature-radius constraint for a separate path-optimization question**, not an 8000 m lateral-offset requirement, BS-search radius, or longitudinal spacing.

If it is additionally adopted for local maneuvers, that must be a declared experiment variant. At 50 m/s, \(v^2/8000=0.3125\) m/s²; reference-route turning and maneuver acceleration must be combined, not checked as unrelated budgets.

### 10.7 Failure semantics

A failure must preserve the last valid state, implicated aircraft, candidate, time, predicted/actual separation, and raw versus clipped commands.

Do not silently remove an aircraft, increase speed/braking limits, suppress failed samples, or call emergency limits to turn a rejected normal maneuver into an accepted one.

The model provides research diagnostics. It does not replace a certified detect-and-avoid or flight-control system.

## 11. Communication incentive, traffic disturbance, and arbitration

The following supplies a concrete **proposed** decision rule for review. The user has agreed to the three-stage structure, but not yet to these particular score definitions or numerical tolerances.

### 11.1 Paired predictions

At a decision time t, compare:

- branch 0: the focal aircraft stays in its current flow;
- branch c: the focal aircraft executes one candidate transition.

Both branches start from the same actual fleet state and stored exposure histories. Both recompute radio, group memberships, exposure, policies, and ACC responses along their predicted trajectories.

The radio map and any announced deterministic release requests are known inputs. The predictor applies the same boundary-admission rules in both branches; realized entry times may differ. It does not know future unannounced disturbances or later discretionary decisions by other aircraft.

Already committed maneuvers are included. Uncommitted future maneuvers are not assumed to happen.

Choose forecast duration \(H_p\) to cover the candidate duration D plus at least one exposure window:

$$
H_p\ge D+W.
$$

Additional following relaxation may require a longer horizon; use horizon sensitivity tests. If a common valid corridor comparison interval cannot cover the maneuver and the required post-maneuver window, reject the candidate for insufficient forecast support rather than assume a favorable unseen future.

### 11.2 Focal exposure benefit

Let \(\mathcal K_i^p\) be paired future policy times where the focal aircraft is in the corridor in both branches. Define

$$
J_{\zeta,i}^{(b)}
=\frac{1}{|\mathcal K_i^p|}
\sum_{\tau\in\mathcal K_i^p}\zeta_i^{(b)}(\tau),
\qquad b\in\{0,c\},
$$

$$
B_{\zeta,i}=J_{\zeta,i}^{(0)}-J_{\zeta,i}^{(c)}.
$$

Require

$$
B_{\zeta,i}\ge\eta_\zeta>0.
$$

This averages forecast **group exposure values**. It is not the old individual's entire-flight weak-link sample fraction.

Proposed debugging value: \(\eta_\zeta=0.02\), meaning a two-percentage-point reduction in the average predicted group exposure. It has no claimed calibration. Proposed sensitivity values are 0.01, 0.02, and 0.05.

### 11.3 Policy-spacing externalities

A move can improve the focal aircraft while worsening its source or target neighbors. Let \(\mathcal A_i^p\) be the union of affected focal groups in the two forecasts, including followers whose leader relationship changes and groups whose member lists change.

For each affected aircraft j, use its paired valid future observations and compute reference-speed spacing:

$$
J_{S,j}^{(b)}
=\operatorname{mean}_{\tau}
S_{m_j^{(b)}(\tau)}(v_{\rm ref}),
\qquad v_{\rm ref}=50\ {\rm m/s}.
$$

Define

$$
B_S=\frac{1}{|\mathcal A_i^p|}
\sum_{j\in\mathcal A_i^p}
\left[J_{S,j}^{(0)}-J_{S,j}^{(c)}\right].
$$

Proposed additional gate: \(B_S\ge0\), within a declared numerical tolerance. Thus a candidate must improve focal exposure without increasing average reference-speed policy-spacing burden over the compared affected aircraft.

This is a local screening heuristic, not proof of a system-capacity increase. It can still redistribute benefit and harm; individual changes must be logged. Paired sample counts, unpaired entries/exits, and affected-set definitions must also be reported. Empty comparisons cannot be filled with zeros.

### 11.4 Induced braking cost

For affected neighbors, define extra braking relative to staying:

$$
D_{\rm brake}(c)=
\sum_{j\in\mathcal A_i^p\setminus\{i\}}
\int_{\mathcal T_j^p}
\max\left[
0,\,
(-a_j^{(c)})_+-(-a_j^{(0)})_+
\right]d\tau,
$$

where \(x_+=\max(x,0)\), and \(\mathcal T_j^p\) is the paired airborne comparison interval. The units are m/s: it is accumulated additional braking magnitude, not energy or delay.

New-insertion gaps, bounded-trajectory 3D checks, and the optional paired executed-braking screen remain prior gates. A low disturbance cost cannot compensate for a predicted conflict.

### 11.5 Selection rule

Among candidates passing all gates, the proposed deterministic lexicographic ranking is:

1. larger \(B_S\);
2. larger \(B_{\zeta,i}\);
3. smaller \(D_{\rm brake}\);
4. shorter maneuver duration;
5. stable target-flow ID as a final tie-break.

This avoids adding exposure, meters, acceleration, and seconds into an unexplained weighted scalar. Alternative priorities are possible, but they must be named as different decision rules.

There is no original-MOBIL politeness parameter hidden in this proposal.

### 11.6 Multiple simultaneous requests

Proposed simple arbitration:

1. Collect eligible candidates at a decision event.
2. Process aircraft in a saved deterministic order.
3. After accepting a candidate, reserve its trajectory/occupancy.
4. Recompute affected predictions and revalidate later candidates.
5. Reject conflicting claims on the same gap or incompatible crossing trajectories.

An ID-based order is reproducible but not a fairness guarantee. Record the order and later check a rotated/reversed-order sensitivity.

An accepted maneuver is not executed instantly. It starts a continuous trajectory whose interactions are checked throughout execution.

## 12. Observation clocks and execution order

### 12.1 Proposed clocks

| Clock | Proposed first value | Purpose |
|---|---:|---|
| Motion and ACC | 0.5 s | Update positions and velocities |
| Radio/group/policy | 5 s | Match the paper's nominal seven-sample window |
| Discretionary decision | 20 s | Reevaluate possible transitions |
| Forecast integration | No coarser than 0.5 s initially | Predict coupled motion consistently |
| Cooldown | 30 s after completion | Prevent immediate repeated switching |

Use a 0.25-second motion/forecast refinement. The old prototype used a 2-second forecast step; that is not automatically adequate for the revised safety checks.

An optional 1-second radio/policy clock is a separate sensitivity, not a silent replacement. Changing the clock changes the discretization of exposure. A 30-second window at 1 second contains 31 inclusive samples, not seven.

No additional off-grid policy observations are fabricated for entrants. Initialize C at entry and evaluate on the next scheduled global policy tick, including the entry time if it coincides with one. This delay is less than one policy interval and must be recorded as initialization-sourced control.

### 12.2 Proposed event ordering

At time t:

1. Resolve any completed connector/maneuver events and update logical flow ownership.
2. Consider scheduled boundary releases against the current state.
3. If it is a radio/policy tick, compute actual positions, radio, same-flow groups, current group fractions, available-history exposure, and policies.
4. If it is a decision tick, construct paired forecasts, apply gates, and arbitrate candidates.
5. Build current relevant leader sets and compute all longitudinal commands from the same state.
6. Advance the fleet to the next time/event boundary using the active trajectory/controller.
7. Check interval separation, kinematic bounds, and state validity.
8. Save outputs and failures; repeat.

An event inside a nominal time step requires splitting the step. Policy sampling remains on its declared clock.

The predictor must use the same event-order rules as execution. Otherwise a predicted benefit can be an artifact of using different policy timing.

### 12.3 Closed-loop pseudocode

~~~text
load immutable geometry, BS data, and reviewed configuration
construct requested releases and fixed initial flow assignments
initialize empty airborne fleet

while requests or unfinished missions remain:
    resolve phase-completion events
    attempt eligible boundary releases with C initialization

    if policy observation event:
        evaluate actual radio
        sort corridor aircraft within each policy flow
        store current variable-size group fractions and member IDs
        average each focal aircraft's available trailing history
        assign C/R/F

    if discretionary decision event:
        compare stay/transition forecasts from the same state/history
        screen geometry, 3D conflicts under bounded commands, new insertion gaps,
            and any executed braking above the optional limit that worsens staying
        evaluate exposure benefit, policy externalities, and disturbance
        arbitrate and commit only feasible candidates

    compute synchronous fixed-cruise or policy-spacing ACC commands
    integrate to next event with continuous connector/transverse motion
    check interval feasibility and save state
    stop explicitly if an unrecoverable modeled failure occurs
~~~

This is specification pseudocode, not an executable algorithm already installed in the repository.

## 13. Parameter register

### 13.1 Inherited communication and policy parameters

| Parameter | Value | Status / interpretation |
|---|---:|---|
| Cruise speed | 50 m/s | Inherited; longitudinal, not total 3D speed |
| Common SINR threshold | −1.5 dB | Inherited study threshold |
| C exposure tolerance | 0.05 | Inherited |
| R exposure tolerance | 0.10 | Inherited |
| Neighbor counts | Up to 2 ahead, 2 behind | Agreed variable-size local group |
| Maximum group size | 5 including self | Not a minimum |
| History window | 30 s | Inherited duration; available history is agreed |
| Initial policy | C | Agreed initialization assumption |
| BS carrier frequency | 5 GHz | Current Bay Area configuration |
| EIRP / receive gain | 46 dBm / 0 dB | Current configuration |
| Carrier noise | −99 dBm | Current configuration |
| RSRP normalization | 300 resource elements | Reporting convention |
| Selected BS set | Nearest 3 by 3D distance | Current Bay Area configuration |
| Reliability level | 0.95 | Conditional capacity statistic, not collision probability |

The paper's full-group estimator and all-site interference variant remain separately identified references.

### 13.2 Spacing and control parameters

| Parameter | Value or proposal | Status |
|---|---|---|
| \(d_0\) | 200 m | Inherited, uncalibrated planning base |
| \(\tau_C,\tau_R,\tau_F\) | 15, 30, 60 s | Inherited effective time-gap terms |
| \(c_b\) | 0.167 s²/m | Inherited planning buffer |
| \(k_v,k_g,k_{\rm rel}\) | 0.2 s⁻¹, 0.005 s⁻², 0.4 s⁻¹ | Exploratory controller gains |
| Normal limits | −2.0 to +1.5 m/s² | Correspondence-informed proposed longitudinal interpretation |
| Accepted insertion braking | 2.0 m/s²; optional stricter sensitivity 1.5 | Executed braking, rejected only if also worse than staying; raw-feedback gate withdrawn |
| Emergency ±5 m/s² | Not used | Correspondence context only |
| Extra 300 m gap floor | Not inherited | Old mechanism test only |
| Actuator lag / sensor delay | Omitted in first model | Not simulated, not physically proven zero |
| Jerk limit | Not imposed in first model | Acceleration may jump at control events |

### 13.3 Maneuver and boundary parameters

| Parameter | Proposed value / treatment | Status |
|---|---|---|
| Lateral speed limit | 8 m/s | Old exploratory component value |
| Vertical speed limit | 3 m/s, symmetric climb/descent trial | Old exploratory value; not vehicle-specific |
| Lateral acceleration limit | 0.3 m/s² | Old exploratory component value |
| Vertical acceleration limit | 0.2 m/s² | Old exploratory component value |
| Minimum maneuver duration | 30 s | Lower bound; actual D from displacement/limits |
| Direct diagonal moves | Disabled | First-design restriction |
| Source-flow policy membership during movement | Until completion | Proposed ownership rule |
| Connector geometry and duration | Generated from specified endpoints and verified bounds | No universal 240-second connector default |
| Shared origin/destination | Virtual points | Agreed; no pad-resource model |
| Initial assignment | Deterministic round-robin | Proposed simple control |
| Boundary retry interval | Motion/event step | Proposed admission convention |
| Pairwise protection shape / semiaxes | Explicit choice required | Old 200/100 m ellipsoid not approved for formal claims |
| Total speed/acceleration envelope | Report resultant; no calibrated vehicle envelope yet | Limitation to resolve for vehicle-specific claims |
| 8000 m radius | Separate corridor-optimization condition | Not silently applied to all maneuvers |

### 13.4 Experiment settings that must be named before a run

A reviewed run configuration must specify the flow coordinates, reference curve, vertical convention, BS dataset and interference set, requested demand, assignment order, release phases, connector boundaries, longitudinal mode, transitions on/off, protection assumptions, clocks, forecast horizon, scoring tolerances, arbitration order, evaluation window, stop/clearance rule, and any random seed.

No final dense-demand grid or full-mission duration is fixed by this document. The old 240-second closed-fleet run length is not a full-route duration.

## 14. Outputs and scientific interpretation

### 14.1 Primary communication and policy outputs

Report:

- per-aircraft SINR, serving BS, and actual 3D position;
- current group member IDs and actual group sizes;
- instantaneous group fraction \(e_i\), historical sample count, and exposure \(\zeta_i\);
- C/R/F time histories and policy changes;
- C/R/F fractions over clearly identified aircraft-time observations;
- policy initialization intervals separately from measured-policy intervals.

With a fixed observation grid, policy fraction is the fraction of eligible aircraft-time records assigned to each policy. Connector initialization labels are not counted as measured C observations under the proposed phase convention.

Changing neighborhood sizes is intentional. Report their distribution to help interpret exposure changes.

### 14.2 Conditional planning capacity

At a fixed reference speed \(v_{\rm ref}\), for a flow with eligible focal observations,

$$
\bar S_\ell(t)=
\frac{1}{N_\ell(t)}
\sum_{i\in\ell} S_{m_i(t)}(v_{\rm ref}),
\qquad
q_{\ell}^{\rm plan}(t)
=\frac{3600v_{\rm ref}}{\bar S_\ell(t)}.
$$

This is the legacy-style policy-conditioned planning rate. It is not the observed throughput of an accelerating fleet.

For a lower reliability floor, preserve the existing implementation's order-statistic convention: sort n samples increasingly and take zero-based index

$$
k=\left\lfloor(1-\rho)n+10^{-12}\right\rfloor,
$$

bounded to \([0,n-1]\). At \(\rho=0.95\), this is a lower-tail estimate, not the ordinary upper 95th percentile.

A theoretical multi-flow sum must be formed from time-aligned flow estimates before calculating its reliability floor. A sum of separate 95%-reliable values is not automatically a jointly 95%-reliable result.

For an empty flow, no observed policy distribution exists; mark its policy-conditioned capacity unestimated, not zero or full-C capacity. Any assumed empty-flow capacity belongs to a different explicitly specified estimator.

This sum also does not remove merge or connector interactions. It cannot establish actual network capacity.

### 14.3 Actual corridor and mission performance

Measure corridor throughput at an explicit downstream gate:

$$
q_{\rm out}
=3600\frac{N_{\rm crossings}}{T_{\rm eval}}
\quad{\rm aircraft/hour}.
$$

Also report requests, releases, corridor entries, exits, completed missions, deferred requests, and unfinished missions. Keep connector travel time separate from corridor travel time.

Measured throughput at a low demand is usually a served demand, not a capacity estimate. Estimating capacity requires controlled demand sweeps, declared evaluation periods, and evidence on sustained service and accumulation.

For paired transition-on/off cases, report changes in corridor travel time, progress, and throughput, together with changed entry times. Do not attribute an improvement caused by different boundary admission solely to in-corridor lane changing.

### 14.4 Safety and control diagnostics

Record minimum modeled separation, raw/commanded acceleration, time at normal bounds, policy-spacing deficits and their duration, affected-follower braking, maneuver counts/durations, rejected-candidate reasons, and predicted-versus-realized benefits.

The old “minimum longitudinal speed” and “mean lost progress” remain optional diagnostics:

$$
\Delta s_i(T)=
v_{\rm ref}T-\int_0^T v_i(t)\,dt
$$

for matched-duration straight-corridor comparisons. It is neither arrival delay nor an independent capacity estimate. Curved-route progress must use the declared physical coordinate, not mix reference s with physical meters.

There is no predetermined minimum speed such as 45.431 m/s in the new model. That number was an output of the old synthetic three-aircraft controller configuration.

### 14.5 Phase separation and conservation

Connector radio and travel statistics are separate because prescribed takeoff/landing kinematics are not the tested communication-driven corridor controller. This does not assert that real takeoff and landing are unrelated to communication.

At each event, every request must belong to exactly one accounting state: not yet released, airborne, completed, or explicitly failed/cancelled. No demand or aircraft silently disappears.

For finite runs, report right-censored unfinished missions. End-of-window incompletion is not automatically a lost aircraft or an operational failure.

## 15. Verification plan and research workflow

### 15.1 Required checks before substantive experiments

| Test | Required behavior |
|---|---|
| Single isolated aircraft | Initial C; actual first eligible sample determines exposure/policy |
| Boundary groups | First/second/last aircraft use available neighbors; no artificial full-group requirement |
| Variable group history | Average historical per-time fractions, not pooled member-time counts |
| Historical ownership | Changing flow retains past observations; current groups do not rewrite history |
| Static regression | Complete matched groups/windows reproduce original policy results |
| Radio regression | Same positions/configuration reproduce the existing kernel's powers and SINR |
| Threshold equality | SINR equal to threshold is good; exposure at 0.05/0.10 maps to C/R |
| Sparse ACC | Equal-speed distant leader does not cause acceleration above cruise |
| Dense ACC | Spacing/relative-speed errors generate correctly signed acceleration |
| Policy downgrade | Target jumps, but position and speed do not; raw/clipped commands both recorded |
| Policy recovery | Bounded recovery, no instant return to cruise |
| Curved-coordinate consistency | Gap derivative and Cartesian speed agree with independent finite differences |
| Quintic endpoints | Position/velocity/acceleration match prescribed boundaries |
| Maneuver extrema | Duration satisfies component bounds; combined trajectory checked separately |
| Source/target insertion | New predecessor/follower pairs must meet targets; inherited pairs remain in bounded ACC and 3D screening |
| Saturation versus safety | Raw feedback below −2 does not reject; bounded trajectories with a predicted conflict still reject |
| Cross-level conflict | Non-group traffic cannot be excluded from a conflicting swept path |
| Inter-sample encounter | An interior minimum separation is detected |
| Two aircraft choose one gap | Arbitration revalidates and prevents incompatible acceptance |
| Fixed-cruise infeasibility | Actual fixed-speed conflict forecast rejects; hypothetical ACC feedback is diagnostic only |
| Departure/arrival connector | No teleportation, reverse motion, invalid intersection, or duplicate completion |
| Numerical refinement | Step/horizon sensitivity reported without calling it a safety proof |
| Bookkeeping | Requests and aircraft conserved; failed cases retained |

### 15.2 Experimental progression after approval

1. Implement and test the actual-group/policy-spacing loop on a straight deterministic fixture.
2. Verify isolated connectors and low-density full missions.
3. Test controlled leader/follower interactions and policy changes.
4. Test single lateral, single vertical, and sequential two-axis transitions.
5. Test contested gaps, changing groups, and predicted externalities.
6. Verify smooth geographic mapping and the existing Bay Area radio baseline.
7. Run matched fixed-cruise/ACC × transitions-off/on cases with common demand and input data.
8. Expand layout, demand, and parameter sensitivities only after the pilot behavior is understood.

Stages 1–5 are mechanism verification. A meaningful Bay Area research conclusion requires the later geographic and radio integration and a suitable experiment design.

### 15.3 Organized and reproducible files

Keep this topic's work under:

~~~text
research/dynamic-transitions/
    README.md
    protocol.md
    full-mission-model-review.md
    implementation-v1.md
    configs/
    reports/
    runs/
        R0001/    historical mechanism checks
        R0002/    historical mechanism checks
        R0003/    first policy-spacing closed-loop pilot, now archived
~~~

Reusable scientific algorithms remain in the repository's source package; command entry points remain under scripts. Do not duplicate algorithms into each run folder.

Each new run must archive the exact configuration, source snapshot/identifier, input hashes, environment, command, clocks, seed if any, event logs, trajectories, summaries, and failure information. Reports cite the topic and run ID.

Do not overwrite old runs. R0003 and R0004 preserve the previous admission implementation. R0005 is allocated to the 2026-08-31 correction comparison; consult the topic README before allocating further IDs. Raw run directories are ignored by Git; a later commit/push alone would not back them up.

### 15.4 Difference from current executable prototype

| Component | Current local mechanism prototype | Reviewed next design |
|---|---|---|
| Mission | Three initially airborne aircraft, 240 s | Scheduled virtual origin-to-destination missions |
| Radio | Synthetic prescribed SINR field | Existing BS kernel, then verified Bay Area geometry |
| Policy | No original C/R/F group feedback | Available-group/history exposure and actual C/R/F |
| ACC target | Fixed headway plus old 300 m floor | \(S_{m_i}(v_i)\), without silently inheriting that floor |
| Normal limits | −1.5/+1.0 m/s² | Proposed −2.0/+1.5 |
| Transition benefit | Individual forecast bad-sample fraction | Proposed group exposure plus policy externality check |
| Safety | Sampled exploratory protection | Declared 3D protection under bounded control, new-pair insertion and paired executed-braking checks |
| Results | Software mechanism diagnostics | Policy/control/throughput experiments only after validation |

The “reviewed next design” column is not a claim that those changes have already been implemented.

## 16. Review and approval checklist

The user has already agreed to the scope and architecture in Section 1.2. The following are the **remaining concrete implementation proposals**, grouped so approval does not require reopening that agreement.

| Review item | Proposal in this document |
|---|---|
| Phase and initialization convention | Connector radio is diagnostic; measured policy begins at the corridor gate on the global policy clock; C applies before the first sample |
| Flow ownership and history | Source policy flow until maneuver completion; retain past exposure observations after switching |
| Boundary handling | Deterministic schedule/round-robin assignment; virtual-connector feasibility guard; defer requests without modeling pad resources |
| Coordinate convention | Straight local formulas first; verified physical-tangential/curved-gap mapping for geographic runs |
| Controller trial package | TRB spacing; exploratory gains; normal command limits −2/+1.5 m/s²; no actuator lag or jerk limiter initially |
| Voluntary insertion braking | Paired executed-command screen at 2 m/s²; raw-demand rejection withdrawn on 2026-08-31 |
| Communication decision rule | Paired group-exposure benefit ≥0.02; nonnegative affected-group policy-spacing benefit; lexicographic selection |
| Geometry and separation | Explicitly approve a research protection shape and parameter set; do not treat old 200/100 m values as validated |
| Pilot configuration | Fix actual flow coordinates, connectors, demand, forecast horizon, and evaluation window in a reviewed configuration before running |

The separation choice is a substantive unresolved numerical assumption. The scoring tolerances and gains are also exploratory, but have explicit trial proposals. Neither kind of assumption should be described as supplied by Toyota unless a source actually says so.

**Execution gate:** the user subsequently approved execution. R0003 covers the first local straight-corridor mechanism stage only. It explicitly uses exploratory protection assumptions and does not establish vehicle-calibrated safety or Bay Area capacity. Documentation arithmetic checks remain distinct from scientific runs.

## 17. Sources and provenance

### 17.1 Primary methodological references

- **ACC structure:** He et al., *Physics-augmented models to simulate commercial adaptive cruise control (ACC) systems* (2022), Section 2.1.3, Eq. 6. Supports linear spacing/relative-speed feedback with a cruise branch; it does not support UAM parameter calibration. [Author manuscript](https://arxiv.org/pdf/2107.07832).
- **Boundary trajectories:** Lynch and Park, *Modern Robotics*, Sections 9.1–9.2. Supports polynomial endpoint construction. No eVTOL performance or separation validation is inferred. [Author textbook resource](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/).
- **Lane-change safety concept:** MOBIL author explanation. Supports distinguishing incentive from target-follower braking acceptance. The present exposure incentive, 3D geometry, and control law are an adaptation, not original MOBIL. [Model description](https://traffic-simulation.de/info/info_MOBIL.html).

### 17.2 TRB and repository sources

- [Airport-to-airport scenario](../../scenarios/airport_to_airport/scenario.json): actual Bay Area scenario selection and parameters.
- [Radio kernel](../../src/capacity_policy/radio.py) and [link-state classifier](../../src/capacity_policy/link_quality.py): received powers, selected sites, interference, RSRP, and common SINR gate.
- [Available-group baseline runner](../../src/uam_simulator/group_runner.py): established variable-size boundary groups and available-history estimator.
- [Paper policy kernel](../../src/capacity_policy/policy.py) and [capacity kernel](../../src/capacity_policy/capacity.py): preserved reference definitions.
- [Current dynamic prototype](../../src/uam_simulator/dynamic_transitions.py), [mechanism configuration](configs/mechanism_checks.json), and [R0002 report](reports/R0002.md): evidence of implemented mechanisms and old exploratory values only.

### 17.3 Private research correspondence

The following are provenance notes for the project owner, not public aviation standards. Full email bodies are not reproduced.

- **Sergei, March 9, 2026, quoted in the March 17 email chain:** normal eVTOL acceleration range −2.0 to +1.5 m/s², emergency range up to ±5.0 m/s², and a reported UAM time-gap range of 45–90 s with a turbulence caveat. The correspondence does not identify a calibrated vehicle envelope or clearly specify component versus resultant acceleration. [Private Gmail record](https://mail.google.com/mail/#all/19cfd1fe5f002f8c).
- **Sergei, August 20 local-time follow-up to the August 18 meeting:** first analyze multiple lanes/levels, then consider a lane-changing model such as MOBIL with SINR/spacing criteria; separately consider offset-path optimization with a minimum curvature radius of 8000 m. [Private Gmail record](https://mail.google.com/mail/#all/1a021a8f1f3fc9e8).

The quoted 45–90 s range is not substituted for \(\tau_C,\tau_R,\tau_F\): the definitions are not established as identical. The complete TRB time headways at 50 m/s are listed in Section 7.3.

### 17.4 What this review establishes

This document establishes a traceable candidate model and a clear approval boundary. It does not establish calibrated aircraft behavior, regulatory compliance, safe operational separation, or a measured capacity gain.

**Implementation update:** the documentation-only revision preceded execution. The first approved mechanism pilot is now archived separately as R0003; see its report for results and remaining geographic work.
