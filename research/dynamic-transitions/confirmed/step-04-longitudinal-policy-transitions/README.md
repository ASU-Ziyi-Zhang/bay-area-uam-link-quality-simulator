# Step 04 — longitudinal policy-gap recovery, two aircraft

This folder is an index. Each revision is a self-contained confirmed item and
none is ever overwritten or edited in place.

| Revision | Standing | Run | What it says |
|---|---|---|---|
| **[revision-04](revision-04/)** | **Current** — 2026-09-11 | R0036 | Same experiment with the spacing constant anchored to NMAC: d0 = 152.4 m instead of an uncited 200 m. All six gaps still recover and the KS2 reference is still about 5.5× faster; only the equilibrium spacings move, to C 1319.9 / R 2069.9 / F 3569.9 m. Step 2 (R0035) uses the same law. |
| [revision-03](revision-03/) | **Superseded by the constant change** — findings unchanged | R0033 | Both controllers recover all six C/R/F gaps. The KS2 reference is about 5.5× faster in both directions: C→F 221.3 s against 1200.9 s, F→C 148.5 s against 858.2 s. |
| [revision-02](revision-02/) | **Superseded** — do not cite | R0032 | The cruise-limited baseline. Claimed the baseline reaches none of the six targets. Kept as provenance only. |

## Why revision-02 was superseded

The R0032 baseline took its command as

    a = min( k_speed·(cruise − v),  k_gap·(gap − S(policy,v)) + k_rel·(v_L − v_F) )

which is the standard automotive ACC form. At cruise the first term is exactly
zero, so the command can never be positive however large the gap error is. A
follower restoring a tighter policy spacing must run ahead of its leader for a
while, so under that form it never closed a gap at all — the R0032 conclusion.

That form was judged wrong for a UAM corridor, where a temporary excursion above
nominal cruise is exactly what a policy upgrade requires. The law was changed to
signed gap and relative-speed feedback without the cruise veto, keeping the
physical bounds and the constraints from other occupied-lane leaders, and the
experiment was rerun as R0033.

**Withdrawn:** that the baseline reaches none of the six targets, and that its
closing shortfall equals the whole transition.

**Unchanged:** the six-transition case set, the 30–80 m/s envelope, the
±1.5/−2 m/s² bounds, the acceptance thresholds and the independent validation
procedure. No tolerance was relaxed in the correction.

## What both revisions still do not claim

Two prescribed aircraft on a straight route with perfect leader observation. No
string stability across a chain, no lateral movement, no capacity result. Gains
and vehicle bounds are declared and uncalibrated.
