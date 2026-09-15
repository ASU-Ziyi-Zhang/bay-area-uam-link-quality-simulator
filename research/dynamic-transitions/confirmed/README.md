# Confirmed research results

This directory is the curated, user-approved layer for the dynamic-transition
research. It is intentionally separate from `runs/`, which contains immutable
execution archives, including failed or exploratory runs.

Each confirmed item contains only the presentation figure(s), approval record,
and audit summary needed to identify a result. The canonical report, protocol,
and raw run remain in their numbered R-directory; links below point back to
those sources. A confirmed item is never overwritten. A later interpretation
or figure change gets a new dated item or revision.

In the public package, links from confirmed items to `runs/`, `reports/` and
`figures/` point to the author's local audit archive, which is not published;
each item's own figures, tables, validation record and hashes are included.

## Confirmed items

Every item carries a standing note saying what it still supports and what has
moved on since it was approved. A confirmed item is never overwritten and never
deleted, so "superseded" here means the interpretation was replaced, not that the
archive was removed.

| Item | Standing | Scope |
|---|---|---|
| [Step 01 — pure lateral movement](step-01-pure-lateral/) | Confirmed 2026-08-31. **Not contradicted; acceleration bookkeeping superseded.** | R0009 single-aircraft lateral study. Its `0.3 m/s²` lateral limit never declared whether it bounds the lane-change component or the total, and R0022 later measured the corridor's own turning at 2.561 m/s². Still the only confirmed link-quality-to-policy evidence. |
| [Step 02 — predictive spatial response](step-02-predictive-spatial-response/) | Confirmed 2026-08-31. **Not contradicted; superseded in scope.** | R0016 60+60 s singleton response, 280 → 230 s fallback. Same undeclared-limit caveat. The three-aircraft lateral experiment will make this the empty-target-lane special case. |
| **[Step 03 — Bay Area demand-stream motion control](step-03-bay-area-demand-stream/revision-03/)** | **Current external result, 2026-09-12.** | R0048 final-only presentation: fixed 90 s single-lane test, C/R/F=63.2/36.0/0.8%, 93/93 complete, no sampled NMAC and 1.22 km minimum sampled separation. No window-selection comparison. Longitudinal settling remains monitored; next is the same-altitude three-lane pair. |
| **[Step 04 — policy-gap recovery](step-04-longitudinal-policy-transitions/revision-04/)** | **Current, 2026-09-11.** | R0036: ACC and AKS both recover all six C/R/F gaps under the NMAC-anchored 152.4 m residual constant; the reference remains about 5.5× faster in both directions. Two figures in acceleration → speed → gap order, plus the full table. 72 traces and zero failed validation checks. |
| [Step 04 — R0032 presentation](step-04-longitudinal-policy-transitions/) | **Superseded 2026-09-10.** Do not cite. | The cruise-limited baseline. Its claim that ACC reaches none of the six targets is withdrawn: that came from the `min(free-flow, following)` form, whose free-flow term is zero at cruise. Kept as provenance. |

The current Step 03 revision confirms the new parameter, execution sequence and
verified R0048 one-lane baseline, not the older R0019–R0021 numerical results.
R0021's multi-aircraft benefit constraint remains incomplete and is still not
confirmed evidence.

R0010 remains a completed and audited exploratory run. Its broader spatial
figures are not separately marked as confirmed; the confirmed Step 02 item is
the later R0016 predictive-response example identified above.
