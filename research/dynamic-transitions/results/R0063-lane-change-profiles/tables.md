# R0063 tables

## Stage 0 — measured error bands

| Quantity | Value |
|---|---:|
| A1 benefit at dt = 0.25 s | 228.0000 |
| A1 benefit at dt = 0.5 s | 228.0000 |
| A1 benefit at dt = 1 s | 228.0000 |
| A1 benefit at decision phase 0 s | 228.0000 |
| A1 benefit at decision phase 1 s | 226.0000 |
| A1 benefit at decision phase 2 s | 224.0000 |
| A1 benefit at decision phase 3 s | 222.0000 |
| A1 benefit at decision phase 4 s | 220.0000 |
| δ_int (integration, finest pair) | 0.0000 |
| δ_phase (decision-phase spread) | 8.0000 |
| δ_pred (first decision: predicted 148.9375 vs executed 148.9375) | 0.0000 |
| gate, revised = δ_int + δ_pred | 0.0000 |
| gate, original = δ_int + δ_phase + δ_pred | 8.0000 |

| Trajectory check | Declared | Measured in Stage 0 | Threshold used |
|---|---:|---:|---:|
| lon_quadrature_m | 0.3 | 1.82e-12 | 1e-06 |
| lon_fd_accel_mps2 | 0.05 | 1.42e-14 | 1e-06 |
| lat_quadrature_m | 0.05 | 0.00926 | 0.05 |
| lat_fd_accel_mps2 | 0.005 | 0.000102 | 0.00102 |
| lon_refine_curve_m | 0.2 | 0 | 1e-06 |
| lon_refine_endpoint_m | 0.1 | 0 | 1e-06 |
| lat_refine_curve_m | 0.05 | 5e-12 | 1e-06 |
| lat_refine_endpoint_m | 0.02 | 3.69e-12 | 1e-06 |

## Part A — decision-rule cases

| Case | Tests | revised gate: first change | Stay cost | Decision-rule cost | Checks |
|---|---|---:|---:|---:|---|
| A1_safe_and_better | safe + benefit → change | 0 s | 580.0 | 352.0 (completed) | pass |
| A2_safe_no_gain | safe + no benefit → stay | never | 882.0 | 882.0 (completed) | pass |
| A3_unsafe_but_better | unsafe + benefit → stay | never | 580.0 | 580.0 (completed) | pass |
| A9_unsafe_and_no_gain | unsafe + no benefit → stay | never | 1100.0 | 1100.0 (completed) | pass |

## Part B1 — same D and T = 90 s, sharp field (prescribed −20 / +20 dB)

Stay (no change) total policy cost: 573.0.

| Shape | Lane-boundary crossing (s) | Peak lateral speed (m/s) | Peak lateral accel (m/s²) | Peak jerk (m/s³) | A samples below threshold | Total cost | Saved vs stay |
|---|---:|---:|---:|---:|---:|---:|---:|
| quintic = r(2,2) | 45.0 | 6.250 | 0.2138 | 0.0247 | 25 | 419.0 | 154.0 |
| r(3,2) late peak | 52.5 | 6.912 | 0.3013 | 0.0469 | 33 | 435.0 | 138.0 |
| r(2,3) early peak | 37.9 | 6.912 | 0.3013 | 0.0494 | 18 | 405.0 | 168.0 |
| cubic Bézier | 45.0 | 5.000 | 0.2222 | 0.0049 | 25 | 419.0 | 154.0 |
| three-clothoid | 45.0 | 6.667 | 0.2963 | 0.0132 | 25 | 419.0 | 154.0 |

## Parts B2 and C2 — closed loop, each shape at its own minimum duration × (1, 1.5, 2)

| Field | Stay cost | Decision-rule cost | Saved | Change starts | Chosen candidate | Gain at that decision | A samples below threshold (stay / rule) |
|---|---:|---:|---:|---:|---|---:|---|
| sharp field (prescribed −20 / +20 dB) | 573.0 | 387.0 | 186.0 | 0 s | shape=bezier3,delay=0,factor=1 | 207.04 | 131 / 9 |
| bad length 700 m, lateral gradient 0.05 dB/100 m | 106.0 | 106.0 | 0.0 | never | — | — | 13 / 13 |
| bad length 700 m, lateral gradient 0.16 dB/100 m | 100.0 | 94.0 | 6.0 | 0 s | shape=quintic,delay=0,factor=1 | 6.00 | 13 / 10 |
| bad length 1650 m, lateral gradient 0.05 dB/100 m | 171.0 | 165.0 | 6.0 | 0 s | shape=bezier3,delay=0,factor=1 | 6.00 | 33 / 30 |
| bad length 1650 m, lateral gradient 0.16 dB/100 m | 161.0 | 140.0 | 21.0 | 0 s | shape=bezier3,delay=0,factor=1 | 21.00 | 33 / 23 |
