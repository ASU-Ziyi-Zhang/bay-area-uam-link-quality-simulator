# Confirmed Step 02 — predictive spatial response

**Status:** confirmed by the user on 2026-08-31.

This confirmed item contains one presentation figure and one outcome table for
the modular R0016 60+60 s predictive response example. The previously proposed
second outcome chart is deliberately excluded. The candidate library supports
lateral, vertical, and simultaneous lateral-vertical maneuvers; the archived
example selects two lateral maneuvers at a constant 300 m altitude.

## Confirmed presentation

The single figure uses the approved template:

1. the complete georeferenced corridor, all archived base stations, both
   decision locations, and an enlarged first maneuver segment;
2. the offset-altitude SINR candidate plane at the 60 s warning horizon, with
   feasible endpoint policy, motion feasibility, and the selected candidate.

The numerical outcome is kept as a table rather than a second figure. Relative
to no lane change, the two-maneuver example reduces fallback time from 280 to
230 s and fallback share from 28.289% to 23.244%: a 50 s (17.9%) reduction and
5.045 percentage-point reduction.

These values confirm the current illustrative software example only. The
60+60 s horizons, policy-cost weights, motion bounds, radio assumptions and
candidate grid are not calibrated operational standards. This is not a
multi-UAM capacity, safety, energy, or passenger-comfort conclusion.

## Standing as of 2026-09-10

**Not contradicted; superseded in scope, and shortly in mechanism.** Nothing in
the later work overturns the 280 → 230 s fallback reduction recorded here. As with
Step 01 this is a singleton study, so the speed envelope, the KS2 reference and
the corrected ACC law do not reach it.

Two things have moved on:

* The same undeclared-reading problem as Step 01 applies to this run's
  `lateral_accel_limit_mps2 = 0.3` and `vertical_accel_limit_mps2 = 0.2`. See the
  standing note on Step 01 for the measurement that makes the reading matter.
* The mechanism itself — one aircraft choosing a lateral offset to improve its own
  future policy — is about to be replaced by the three-aircraft experiment in
  `three-uam-lateral-protocol.md`, where a neighbour already occupies the target
  lane and the choice has a separation consequence. That experiment will make this
  one the special case of an empty target lane.

Retained as the only confirmed predictive-response result until that work exists.

## Canonical sources

- [R0016 report](../../reports/R0016.md)
- [Controller equations and modular interfaces](../../dual-horizon-controller-method.md)
- [R0016 run archive](../../runs/R0016/)
- [Approved figure revision](../../figures/R0016/toyota-briefing/revision-04/)
- [Independent validation](../../validation-R0016.json)

## Confirmed files

| File | Source | SHA-256 |
|---|---|---|
| `APPROVED.json` | User confirmation record | `bbb860561e3a5e863d5ac6cee63bbdb8dbde2c338c094ead92547e7f50335f69` |
| `validation.json` | `validation-R0016.json` | `27f479a980e283b6de21b33304e13d01c225285f0ac2be7490d301b2ac33f0d5` |
| `figures/01-corridor-and-candidate-plane.png` | `figures/R0016/toyota-briefing/revision-04/01-corridor-and-warning-horizon-candidate-plane.png` | `a71fa813de7136e9b85a6dd1f941b5e0568f1f7413d73a3022a5c5d20e67928a` |
| `figures/01-corridor-and-candidate-plane.svg` | `figures/R0016/toyota-briefing/revision-04/01-corridor-and-warning-horizon-candidate-plane.svg` | `48712fc21da64565e9ea8471ed547c9d7f203713eacac598d995b3379d01df3c` |
| `summary-table.md` | R0016 fixed and 60+60 summaries | `bac340b4f07e44b104c61dce5382dfce19a6aadd7d66fcbe9fe62b3e19d18d54` |
| `summary-table.csv` | R0016 fixed and 60+60 summaries | `b7f4bf8cd38e0152ccd4f7a16d393fc2ec56fbb83bcede5307d169676f3d6b02` |

