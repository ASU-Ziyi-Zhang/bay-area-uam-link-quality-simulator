# Changelog

## 0.2.0 — 2026-09-14

Multi-UAM motion control on the Bay Area corridor. The link-quality kernel, the
scenario packs, the dashboards and the frozen reference results of 0.1.0 are
unchanged.

### Added

- **Longitudinal control** (`aks.py`, `two_uam_longitudinal.py`,
  `geographic_traffic.acc_controls`): policy-dependent spacing target
  S = d0 + τ_p v + b v² with d0 = 152.4 m (NMAC); classic ACC baseline with an
  established-pair recovery mode; Adaptive Kinematic Smoothing (KS1, KS1A, KS2)
  reference planning with feedback tracking and ACC fallback.
- **Lane change** (`five_uam_lateral.py`): quintic, r(p, q) kernel, cubic Bézier
  and three-clothoid profiles as exact polynomial pieces; rolling change/stay
  decision with admission filtering before cost ranking; exact interval audits.
- **Motion envelope and safety** (`motion_envelope.py`, `safety.py`): route-turn
  and lane-change normal acceleration separated, coupled longitudinal/lane-change
  budget, NMAC event counting (152.4 m and 30.48 m jointly), separation screening.
- **Geographic multi-UAM traffic** (`geographic_traffic.py`,
  `bay_area_dispatch.py`, `capacity_analysis.py`): request stream on the real
  corridor, lateral/altitude grid moves, neighbourhood exposure grouping, route
  speed governor, policy-conditioned planning capacity.
- **Calibration and experiments**: Bay Area classifier calibration, closed-loop
  window sweep, two-aircraft ACC/AKS comparison, five-aircraft lane-change study,
  single-lane before/after calibration, stay-versus-3 × 3 corridor experiment.
  Protocols, configurations, confirmed results and compact result tables under
  `research/`; method summaries in `docs/motion_control.md`,
  `docs/bay_area_calibration.md` and `docs/corridor_experiment.md`.
- Run, verify and plot entry points for each study, including
  `scripts/plot_calibration_comparison.py`.
- Research-provenance helpers: runners archive the resolved configuration,
  source snapshot and hashes and refuse to overwrite a run.
- Tests for all new modules (337 tests pass; 7 skip when local raw run archives
  are absent).

- **Multi-UAM lane-change dashboard** (`dashboard/motion.html`, `motion_app.js`,
  `motion.css`, built by `scripts/build_motion_dashboard.py` from verified run
  R0062): no lane change against lane change within the 3 × 3 grid, with map, 3D
  view at flown altitude, cross-section of cell occupancy, per-aircraft speed,
  gap and spacing target, controller mode, lane-change progress, planning-rate
  trace and the comparison table. An experiment panel recomputes the policy layer
  (Θ, exposure window, persistence k, C/R tolerances, group size and form,
  reliability ρ) on the flown trajectories and redraws the page; flight paths,
  spacing control and lane-change decisions stay as flown, and the recomputation
  uses the 5 s replay grid rather than the run's 2 s radio sampling, so with the
  archived settings it reports C/R/F 71.2/28.2/0.6% against the run's
  73.8/25.9/0.3%. One click restores the archived run.
  The page uses the same aircraft symbol as the other two, with altitude on a
  badge; the cross-section reports a count and a C/R/F bar per cell instead of
  one dot per aircraft; policy fractions for the case and the selected
  aircraft's SINR against the threshold with the policy it held are shown as on
  the policy page; both per-aircraft charts carry legends.

### Changed

- **Multi-UAM policy dashboard uses the calibrated settings**: SINR threshold
  −2.0 dB, 2 s sampling, 5 s updates, 90 s window, k = 3 and d0 = 152.4 m, for
  both scenario packs. `group_simulator.json` gained optional `policy.window_s`,
  `link_quality.sinr_threshold_db` and `capacity.standstill_distance_m`; the
  `trb_reference_regression` block keeps the scenario pack settings, so the TRB
  reproduction (sf_sj_full C/R/F 36.57/35.82/27.61%, Q0.95 72.47 UAM/h) is
  unchanged. Airport corridor now C/R/F 66.5/31.7/1.8%, Q0.95 108.9 UAM/h.
- Navigation links between the three dashboard pages.
- `scripts/verify_bay_area_single_stream_capacity.py` checks the cases a
  configuration declares, so single-case reference archives can be verified.
  Checks and thresholds are unchanged.
- `.gitignore` excludes raw research run archives (`research/**/runs/*`).

### Not included

Raw run archives, working reports and superseded one-off briefing scripts.
