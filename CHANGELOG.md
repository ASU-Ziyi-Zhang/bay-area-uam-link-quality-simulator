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

### Changed

- `scripts/verify_bay_area_single_stream_capacity.py` checks the cases a
  configuration declares, so single-case reference archives can be verified.
  Checks and thresholds are unchanged.
- `.gitignore` excludes raw research run archives (`research/**/runs/*`).

### Not included

Raw run archives, working reports and superseded one-off briefing scripts. The
open-loop calibration tables were produced with the companion TRB classifier
package, which is not part of this repository.
