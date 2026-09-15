# Step 04 — longitudinal policy transitions, two aircraft (R0032, SUPERSEDED)

> ⚠️ **Superseded on 2026-09-10 by [revision-03](revision-03/), run R0033. Do not
> cite the conclusions on this page.**
>
> The baseline ACC law used in R0032 took its command as
> `min(k_speed·(cruise − v), following term)`. The free-flow term is exactly zero
> at cruise, so the baseline could never exceed its set speed and never closed a
> gap. That is the standard automotive ACC form, but it was judged wrong for a
> UAM corridor, where an aircraft restoring a tighter policy spacing must be
> allowed a temporary excursion above its nominal cruise. The law was changed to
> use signed gap and relative-speed feedback without the cruise veto, keeping the
> physical bounds, and the experiment was rerun as R0033.
>
> **What is withdrawn:** the claim below that the baseline reaches none of the six
> targets, and specifically that its closing shortfall equals the whole
> transition. Under the corrected law both controllers recover all six gaps.
>
> **What replaced it:** both settle, and the reference is about 5.5× faster in
> both directions — C→F 221.3 s against 1200.9 s, F→C 148.5 s against 858.2 s.
>
> This page is kept unedited below the line as the provenance record of what was
> approved on the earlier law. Everything under it should be read as history.

---

**Original status line, retained:** confirmed by the user on 2026-09-10. Source run R0032.

One follower behind one leader on a straight route, under an explicit 30–80 m/s
speed envelope that both controllers fly. The question is narrow and complete:
when the follower's communication policy changes, can it move to the spacing the
new policy requires? All six ordered pairs of {C, R, F} are run, so the size of a
transition and its direction vary independently.

## What is confirmed

The KS2 reference reaches all six targets exactly. The baseline ACC reaches none
of them, and it fails for two different reasons in the two directions.

*Opening* (C→R, R→F, C→F). The baseline moves the right way and stops 100, 135 and
125 m short, saturating its 2.0 m/s² braking limit in all three. The shortfall
does not grow with the transition size, which rules out slowness as the
explanation. Two effects stop it together: opening a gap requires the follower to
be slower than the leader, which the relative-speed term penalises, and the target
is evaluated at the follower's own speed, so slowing down lowers the target being
chased.

*Closing* (R→C, F→R, F→C). The baseline shortfall is 750, 1500 and 2250 m — the
full transition, to the metre. It does not move at all. Its command is

    a = min( k_speed·(cruise − v),  k_gap·(s − S(policy,v)) + k_rel·(v_L − v_F) )

and at cruise the first term is exactly zero, so the positive following term
(+3.75, +7.50, +11.25 m/s²) is discarded by the minimum. **This is structural.**
No value of `k_gap` changes it, because the failure is in the `min` rather than in
the term being minimised. Closing a policy gap requires a deliberate excursion
above the set speed, and the classic ACC form has no term that produces one. The
KS2 junction is that excursion: 79.3, 80.0 and 80.0 m/s at the three sizes.

Effort runs opposite to achievement. In the opening cases the baseline spends the
full braking limit and still falls short while the reference uses 0.93, 0.47 and
0.31 m/s² and arrives.

The closing failure has a capacity consequence that the opening one does not. An
aircraft that cannot close its gap goes on holding up to 2250 m more separation
than its policy requires. That is safe, and it is airspace no one is using, so
every failed upgrade leaves a permanent excess in the corridor.

## Confirmed presentation

Two figures, one per direction, each in the same three panels — speed, gap, and
the acceleration each controller commands.

1. **`01-policy-degradation-C-to-F`** — the policy gets worse, the required
   spacing rises, and the follower must fall behind. The gap panel carries the
   moving target the baseline is actually chasing, `S(F, v_F)`, which drops from
   3618 to 2577 m within ten seconds because the follower slows.
2. **`02-policy-upgrade-F-to-C`** — the policy gets better, the required spacing
   falls, and the follower must run ahead for a while. The baseline's commanded
   acceleration is a flat zero line from start to finish.

The remaining ten transition rows and the six variant rows are a table,
`results-table.md` / `.csv`, not further figures. An earlier revision of this item
used two bar charts in that place; they compressed twelve trajectories into bar
heights and lost the mechanism the comparison exists to show.

`Gap − target` in the table is signed. Negative means the follower ended closer to
the leader than its policy allows. Positive on a `planned` closing row means the
opposite failure and is the capacity-relevant one: the aircraft goes on holding
more separation than it needs.

## What is explicitly not confirmed

The leader-braking case is reported, not approved as a controller comparison: it
is the diagnostic of a state that has no longitudinal exit at all, where the
leader sits on the speed floor and neither controller can open the gap. The
reference is the worse of the two there, 652 m short against 416 m, because it
committed to a constant-leader forecast that was invalidated at t = 20 s. That
question belongs to `open-question-wave-relief-by-lane-change.md`.

Nothing here is a string-stability, multi-aircraft, lateral, or capacity result.
Two aircraft only. The speed bounds are sourced but not calibrated to a specific
airframe. The comparison is against the classic linear ACC form; no alternative
smooth-reference scheme has been compared yet.

## Canonical sources

- [R0032 report](../../reports/R0032.md)
- [Protocol, with amendments A1–A5](../../two-uam-longitudinal-protocol.md)
- [AKS longitudinal protocol](../../aks-longitudinal-protocol.md)
- [R0032 run archive](../../runs/R0032/)
- [Figure manifest](../../figures/R0032/manifest.json)
- [Independent validation](../../validation-R0032.json)

Independent validation: passed, 72 traces, 0 failed checks, 48 of 48 step
refinement comparisons. 252 unit tests pass.
