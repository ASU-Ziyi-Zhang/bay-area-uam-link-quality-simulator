# Modular dual-horizon communication-response controller

Status: **method-development baseline for discussion with Toyota**. The time
values and policy-loss weights below are transparent demonstration assumptions,
not calibrated operational requirements or claimed optima.

## 1. State, radio observation, exposure and policy

At decision time \(t_k\), the singleton demonstration state is

\[
x_k=[q_k,d_k,h_k]^\mathsf{T},
\]

where \(q\) is progress along the fixed reference corridor, \(d\) is signed
lateral offset and \(h\) is altitude. The archived propagation model supplies
\(\mathrm{SINR}(q,d,h)\). At each 5 s policy sample,

\[
b_i=\mathbf{1}\{\mathrm{SINR}_i<\Theta\},\qquad \Theta=-1.5\ \mathrm{dB}.
\]

Available samples in the preceding \(W=30\) s form the exposure

\[
E(t)=\frac{1}{|\mathcal H(t)|}\sum_{i\in\mathcal H(t)}b_i.
\]

The existing policy map is retained:

\[
P(E)=
\begin{cases}
C,&E\le 0.05,\\
R,&0.05<E\le 0.10,\\
F,&E>0.10.
\end{cases}
\]

This run uses singleton exposure only. In the later multi-UAM model,
\(\mathcal H(t)\) will contain the available observations of the variable-size
local policy group rather than one aircraft alone.

## 2. Two separately configurable horizons

The controller separates **warning** from **action evaluation**:

\[
H_w=60\ \mathrm{s},\qquad H_e=60\ \mathrm{s},\qquad H_p=H_w+H_e=120\ \mathrm{s}.
\]

Every \(\Delta t_d=5\) s, the no-action branch is forecast over \([t_k,t_k+H_w]\).
Candidate search opens if any forecast policy is worse than the current policy:

\[
\exists\tau\in(0,H_w]:\quad
\operatorname{rank}(P_{\rm stay}(t_k+\tau))
<\operatorname{rank}(P(t_k)),
\]

with \(C>R>F\). Therefore, a short degradation that recovers before exactly
\(t_k+60\) is not hidden by an endpoint-only trigger.

After a warning, staying and all candidate actions are simulated on the same
120 s planning interval. A maneuver may last longer than the 60 s warning
horizon, but it must complete within the 120 s planning horizon. The warning
and evaluation extensions are independent configuration fields; future
sensitivity analysis can change them without changing controller code.

## 3. Candidate state and continuous maneuver

The current demonstration candidate grid is

\[
d'\in\{-200,-100,0,100,200\}\ \mathrm{m},\qquad
h'\in\{200,300,400\}\ \mathrm{m}.
\]

For source \(y_0=[d_0,h_0]^\mathsf{T}\) and target
\(y_1=[d_1,h_1]^\mathsf{T}\), transverse motion is

\[
y(t)=y_0+s(u)(y_1-y_0),\quad
s(u)=10u^3-15u^4+6u^5,\quad
u=\frac{t-t_k}{T_m}.
\]

The duration is the smallest value satisfying the declared component bounds:

\[
T_m=\max\left(
T_{\min},
\frac{15|\Delta d|}{8V_d},
\sqrt{\frac{10|\Delta d|}{\sqrt3 A_d}},
\frac{15|\Delta h|}{8V_h},
\sqrt{\frac{10|\Delta h|}{\sqrt3 A_h}}
\right).
\]

The current uncalibrated inputs are \(T_{\min}=30\) s,
\(V_d=8\) m/s, \(A_d=0.3\) m/s², \(V_h=3\) m/s and
\(A_h=0.2\) m/s². The complete path also passes the provisional 976 m minimum
turn-radius guard. These are replaceable inputs, not certified eVTOL limits.

## 4. Common-horizon policy objective and selection

To avoid judging an action only at one endpoint, define the demonstration
policy loss

\[
\ell(C)=0,\qquad \ell(R)=1,\qquad \ell(F)=2,
\]

and its planning-horizon integral

\[
J_P(a)=\int_{t_k}^{t_k+H_p}\ell(P_a(t))\,dt.
\]

An action is communication-beneficial only if

\[
J_P(a)<J_P(\mathrm{stay}).
\]

The weights are exposed as configuration parameters and require sensitivity
analysis. They are used here only to construct a runnable example. Among
beneficial actions, the deterministic order is:

1. minimum integrated policy loss \(J_P\);
2. better policy at the common planning endpoint;
3. minimum transverse displacement \(\|y_1-y_0\|_2\);
4. minimum maneuver time;
5. deterministic coordinate tie-break.

Keeping `stay` as the reference prevents a maneuver when the degradation
naturally clears and movement does not reduce total policy loss.

## 5. Modular interfaces for the next research stages

The controller is deliberately divided into replaceable modules:

1. **radio predictor:** \((q,d,h)\mapsto\mathrm{SINR}\);
2. **group exposure:** observations \(\mapsto E(t)\);
3. **policy map:** \(E(t)\mapsto C/R/F\);
4. **warning trigger:** current and forecast policy \(\mapsto\) search/no search;
5. **motion generator:** source/target \(\mapsto y(t),T_m\);
6. **communication objective:** policy trajectory \(\mapsto J_P\);
7. **safety gate:** candidate trajectory and traffic \(\mapsto\) feasible/infeasible;
8. **selector:** feasible actions \(\mapsto\) executed target.

The next multi-UAM stage will replace singleton exposure with the agreed
variable-size local group, add ACC longitudinal response and apply a separate
three-dimensional traffic-conflict gate to every candidate. Communication
benefit will never override the safety gate.

## 6. Claims that may and may not be made

The runnable example may demonstrate that the full chain works and that a
longer-than-60-second maneuver can be evaluated after a 60-second warning. It
does **not** establish that 60+60 s, the policy-loss weights, candidate grid,
motion limits or SINR threshold are optimal or operationally validated.

Required later studies include horizon sensitivity, policy-loss sensitivity,
radio-model uncertainty, candidate-envelope sensitivity, multi-step MPC or
dynamic programming, multi-UAM group exposure, ACC interaction, three-
dimensional conflict resolution, passenger-comfort/energy measures and
aircraft-specific calibration.
