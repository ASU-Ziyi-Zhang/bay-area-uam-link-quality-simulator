# Corridor shape optimization protocol (prototype, pending review)

Question: with the airport-proxy endpoints fixed, can changing the continuous
corridor shape improve conditional planning capacity?

- Prototype: fixed 300 m altitude, altitude not optimised; smooth path with six
  control points; deterministic, feasibility-first search.
- Constraints: minimum radius of curvature 8000 m; offset envelope 2000 m from the
  reference line; length at most 1.1 × the reference. Apart from the 8000 m radius,
  which was given by e-mail, these are exploratory settings, not confirmed
  operating standards.
- Objective: Q0.95 with complete groups, 1 s sampling, and a common warm-up and
  evaluation window.
- Comparators: original centreline, smoothed zero-control curve, feasible
  initial seed; an infeasible baseline must be marked.
- Report: best tested feasible candidate, C/R/F, capacity, geometric constraints
  and refined arc-length verification; no claim of global optimality.
- Excluded: dynamic lane changes, airspace certification, terrain clearance,
  shared terminals and resource-utilisation coupling.

Implementation details and existing results are in the
[historical protocol](../_archive/itl-baselines/protocol.md). New runs never
overwrite old results.
