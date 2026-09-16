# Fixed-stream and constrained-offset baseline studies

Historical combined-study protocol. New experiments are separated into
`research/geometry-response/` and `research/corridor-optimization/`.
Historical manifests/configs retain their original paths; see this archive's
README and `research/migration-20260830.json` for relocation, not a rerun.

## Scope and provenance

This is the first backend research extension, not a change to the published
dashboard or an implementation of dynamic flight control. The accepted scenario
files, dashboard bundles and single-stream runner remain unchanged.

The method was checked against the user's TRB source in `TRB UAM 2027`:

- `TRB_2027_Overleaf/manuscript.tex`: **Methodology**, moving-group link quality
  and policy-field/capacity evaluation; source SHA-256
  `93224515d58f86ecf05f1aef4a3ab85b8c7dc2c5fbfb54481ab1b872afb09b1b`.
- `experiment/sinr_policy_pipeline.py`: `classify_sinr_common_threshold`.
- `experiment/mixed_capacity.py`: snapshot mixed spacing and the empirical
  lower-reliability order statistic.
- `experiment/config.py`: nominal group, window and spacing constants.

The optional verifier reads those local sources but they are **not** a runtime
or CI dependency of this repository. It checks the same synthetic input through
both implementations without changing the paper or generating seed files:

```sh
python scripts/verify_trb_methods.py --trb-root "/path/to/TRB UAM 2027"
```

On the inspected sources, policy codes, exposure, validity masks and all
snapshot-capacity arrays were exactly equal (6,061 valid group-time observations).
Reliability floors at 0.90, 0.95 and 0.99 also matched exactly.
This is a **kernel parity test**, not a claim that the real-corridor scenario
reproduces the paper's stochastic cross-section ensemble. In particular the
real-corridor scenario uses a nearest-three served/interference set, whereas
the inspected paper SINR pipeline sums all its configured sites.

## Reused science, explicit estimator differences

Every aircraft is evaluated by the existing `compute_link_state` and
`evaluate_link_quality` kernels. The common SINR threshold and exposure limits
come from the selected scenario. Spacing remains:

`S_m = d0 + tau_m * v + C_buffer * v^2`

For one stream, each valid focal group contributes **one** spacing, not five
duplicated physical segments. Snapshot equivalent flow is
`q(t) = 3600 * v / mean(S_m(t))`. Reliability is the existing lower order
statistic `floor((1-rho) * N)` in zero-based sorted values.

Two separately labeled estimators are included in the lane experiment:

| Estimator | Policy neighborhood / observation window | Sampling |
|---|---|---|
| `dashboard_1s` | Every active focal UAM; smaller groups/history at boundaries/startup | 1 s, no warmup |
| `paper_5s` | Complete centered-five groups and complete trailing windows, post-transit warmup | 5 s |

The new 1x1 cases match both committed scenario bundles exactly, including
policy shares, observation counts and reliability capacities. The offset
optimization uses **full groups at 1 s**, a declared resolution extension of
the paper estimator, not either of the above labels. Do not compare its Q
directly to an all-active dashboard Q as if only geometry had changed.

## Study A: independent fixed lanes and levels

For the more general position-response formulation, arbitrary coordinate APIs
and same-stream independence checks, see `research/geometry-response/protocol.md`. The four
layouts below are initial examples, not a restriction of the evaluation kernel.

The original `configs/research_baselines.json` is now frozen under this archive's
`configs/`. For new work edit the separate topic configurations instead.
Initial exploratory layouts are 1x1, 2x1, 1x2 and 2x2. Two-lane offsets are
-250/+250 m; levels are 300/600 m. These are experiment parameters, **not**
validated lateral or vertical separation minima. Both the full corridor and
the independently retained airport-access corridor are evaluated.
In addition, the 600 m level extrapolates the radio model's height range as
documented in `docs/assumptions.md`; this is not only a separation-rule caveat.

- Each aircraft keeps its lane/level for its entire transit.
- Groups include only aircraft in the same lane and level. Shared membership
  across neighboring focal groups remains allowed, as in the paper.
- Streams start at t=0 with deterministic entry times. No common merge or
  terminal is modeled; synchronous first entries do not imply merge feasibility.
- `fixed_total`: divide the declared demand equally among all streams.
- `fixed_per_stream`: hold each stream's demand fixed, so total demand grows
  with the number of streams. Never conflate these two comparisons.
- The inherited fixed-offset trajectory advances in **reference** arc length,
  with positive-left segment normals. This retains exact baseline parity but
  offset positions can jump at GIS corners. Study A is a prescribed radio/flow
  geometry comparison, not a validated flyable multi-lane design.

At common valid timestamps, independent-stream planning flows are summed:

`q_total(t) = sum_stream q_stream(t)`

Only **then** is `Q_rho` evaluated. A sum of individual stream quantiles is not
the quantile of their simultaneous sum. A second metric checks whether every
stream supports its **assigned** demand at a snapshot, then tests the fraction
of jointly supported snapshots against rho. Sufficient pooled capacity alone
does not prove feasibility with fixed allocation and no lane switching.

Missing full-group observations remain `NaN` in traces and `null` in JSON
estimates. They are not filled with zero or an optimistic C capacity. Support
fractions are conditional on common valid samples; coverage is reported.
Sparse demand may produce no estimate under the complete-group definition.

The radio model does not introduce traffic-dependent BS resource loading or
aircraft-generated interference. It is possible for identical independent
streams to scale exactly linearly; that is a model assumption check, not a
prediction that real infrastructure offers unlimited scaling.

## Study B: endpoint-fixed lateral-control optimization

1. Retain the raw constant-offset scan as a legacy diagnostic. Nonzero offsets
   move its endpoints, and the raw GIS polyline is not curvature-constrained.
   It is **not** part of the feasible optimization candidate set.
2. Sample six controls at equal reference arc-length fractions, displace the
   interior controls along local normals, and fit a natural cubic XY spline.
   The first and last controls stay at the original endpoints.
3. Constrain the entire cubic curve to `|curvature| <= 1/8000 m` and reject
   degenerate/cusped curves. Check all segment endpoints and stationary roots
   of the curvature polynomial, with dense spot checks as a numerical guard.
4. Screen a configurable 2,000 m correspondence envelope at <=25 m reference
   spacing plus all original GIS vertices. This is the Euclidean distance to
   the reference point at equal normalized parameter; it is stricter than a
   nearest-point corridor buffer, but is still a **sampled numerical check**.
   Also cap path length at 1.1 times the original length.
5. Search bounded interior offsets deterministically, first reducing constraint
   violation, then maximizing capacity among feasible candidates. Initial
   offsets and coordinate step sizes are fully recorded; the objective is
   threshold-based and may be flat over sizeable regions.
6. Simulate aircraft at constant **physical path speed** by numerical arc-length
   inversion. Every candidate uses the same demand, duration and warmup
   `1.1 * original transit time + policy window`. Warmup therefore cannot
   improve an objective merely by discarding a different initial time window.
7. Re-evaluate the selected path using a 5 m rather than 10 m arc table.
   Report both curvature/envelope checks and the change in Q. A separate
   all-active 1 s diagnostic is labeled as such.

The zero-control spline can itself differ from the original GIS path and can
violate the envelope. Its constraint status is always recorded; an infeasible
zero-control baseline must not be called a feasible comparator. The search
also reports its best feasible seed and first feasible candidate so geometric
feasibility recovery can be distinguished from subsequent capacity improvement.

The selected result means **best tested feasible candidate**, not a globally
optimal route. Failure to find a feasible candidate is not an infeasibility
proof and must never trigger a silent fallback to raw GIS. Exported GeoJSON
is a sampled visualization of the spline; its straight segments are not the
continuous path used for the curvature check. Control coordinates are retained
for reconstruction.

## Reproduce and inspect

```sh
python -m pip install -e '.[dev]'
python scripts/run_baseline_studies.py --study lanes --output research/geometry-response/runs/R0002
python scripts/run_baseline_studies.py --study offsets --output research/corridor-optimization/runs/R0001
python -m pytest -q
python scripts/verify_reference.py
```

Nonempty output directories are refused. Use a new run name for each revision.
No simulator bundles or published files are overwritten by the research run.

Each scenario produces:

- `lanes/conditions.csv`: layout/demand/estimator comparisons and joint support;
- `lanes/streams.csv`: stream-specific radio and policy diagnostics;
- `lanes/*.npz`: aligned per-stream/total capacity, active counts and C/R/F counts;
- `offsets/legacy_fixed_offset_scan.csv` and `offsets/candidates.csv`;
- `offsets/summary.json`: objective, constraints, comparators and fine-resolution checks;
- `offsets/selected_path.geojson` and `selected_path_controls.csv`, if feasible;
- `offsets/comparison_capacity.npz`: raw, smoothed, selected and fine-resolution traces.

The run manifest records configuration/data/source hashes and output hashes.
The TRB source tree remains read-only. The first review is in
`research/_archive/itl-baselines/report.md`.

## Deliberate exclusions and next gate

No MOBIL, lane/level transition, trajectory conflict controller, merge capacity,
vertiport service process, operator-access model, terrain clearance, regulatory
airspace constraint or complete aircraft dynamics is added here. The 8 km
curvature constraint is a research design input, not a safety certification.
Before turning these experiments into an operational or web-experiment mode,
review the stream-independence assumptions, lane/level choices, policy estimator
and path envelope; then add visual integration without relabeling these
conditional planning estimates as realized throughput.
