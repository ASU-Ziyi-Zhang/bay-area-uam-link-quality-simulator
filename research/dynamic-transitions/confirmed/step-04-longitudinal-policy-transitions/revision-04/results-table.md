# Two-UAM policy recovery results

T is the KS2 reference duration, not the ACC completion time. Both methods are observed until T + 1500 s.

Sampled settling requires both |gap − S(policy, 50)| ≤ 1 m and |v − 50| ≤ 0.05 m/s for the remainder of the saved trace. Not reached is censored at the observation endpoint.

The initial planner request does not describe the complete outcome of a leader-braking case; those cases can invalidate the reference and use feedback fallback.

## Transition cases

| Case | Method | T (s) | Settling (s) | Gap error at T+120 (m) | Final gap error (m) | Speed min/max (m/s) | Accel min/max (m/s²) |
|---|---|---:|---:|---:|---:|---|---|
| C → R | acc | 75.0 | 829.1 | -159.24 | -0.003 | 44.30 / 50.00 | -2.000 / 0.044 |
| C → R | aks | 75.0 | 73.8 | -0.00 | -0.000 | 30.00 / 50.00 | -0.933 / 0.700 |
| R → F | acc | 150.0 | 1137.5 | -266.12 | -0.037 | 40.67 / 50.00 | -2.000 / 0.059 |
| R → F | aks | 150.0 | 147.5 | -0.00 | -0.000 | 30.00 / 50.00 | -0.467 / 0.350 |
| C → F | acc | 225.0 | 1200.9 | -246.85 | -0.034 | 36.01 / 50.00 | -2.000 / 0.089 |
| C → F | aks | 225.0 | 221.3 | -0.00 | -0.000 | 30.00 / 50.00 | -0.311 / 0.233 |
| R → C | acc | 51.2 | 731.2 | +163.00 | +0.001 | 50.00 / 56.30 | -0.053 / 1.500 |
| R → C | aks | 51.2 | 50.7 | -0.00 | -0.000 | 50.00 / 79.28 | -2.000 / 1.500 |
| F → R | acc | 100.0 | 921.0 | +270.51 | +0.004 | 50.00 / 61.08 | -0.082 / 1.500 |
| F → R | aks | 100.0 | 99.0 | +0.00 | +0.000 | 50.00 / 80.00 | -1.050 / 0.787 |
| F → C | acc | 150.0 | 858.2 | +210.60 | +0.001 | 50.00 / 68.22 | -0.148 / 1.500 |
| F → C | aks | 150.0 | 148.5 | +0.00 | +0.000 | 50.00 / 80.00 | -0.700 / 0.525 |

## Variant cases

| Case | Method | T (s) | Settling (s) | Gap error at T+120 (m) | Final gap error (m) | Speed min/max (m/s) | Accel min/max (m/s²) |
|---|---|---:|---:|---:|---:|---|---|
| sufficient gap; no established pair | acc | 0.0 | not reached | +930.10 | +930.100 | 50.00 / 50.00 | 0.000 / 0.000 |
| sufficient gap; no established pair | aks | 0.0 | not reached | +930.10 | +930.100 | 50.00 / 50.00 | 0.000 / 0.000 |
| closing; 50 m/s ceiling | acc | 0.0 | not reached | +2250.00 | +2250.000 | 50.00 / 50.00 | 0.000 / 0.000 |
| closing; 50 m/s ceiling | aks | 0.0 | not reached | +2250.00 | +2250.000 | 50.00 / 50.00 | 0.000 / 0.000 |
| leader brakes; initially sufficient gap | acc | 0.0 | not reached | -164.90 | -1467.106 | 30.00 / 50.00 | -0.610 / 0.000 |
| leader brakes; initially sufficient gap | aks | 0.0 | not reached | -164.90 | -1467.106 | 30.00 / 50.00 | -0.610 / 0.000 |
| leader brakes during opening | acc | 225.0 | not reached | -1883.61 | -1883.613 | 30.00 / 50.00 | -2.000 / 0.089 |
| leader brakes during opening | aks | 225.0 | not reached | -2119.46 | -2119.455 | 30.00 / 50.00 | -2.000 / 0.000 |
| opening; legacy zero-speed floor | acc | 90.0 | 1200.9 | -589.85 | -0.082 | 36.01 / 50.00 | -2.000 / 0.089 |
| opening; legacy zero-speed floor | aks | 90.0 | 89.1 | -0.00 | -0.000 | 0.00 / 50.00 | -1.944 / 1.458 |
| opening; reference jerk bound 0.05 | acc | 225.0 | 1200.9 | -246.85 | -0.034 | 36.01 / 50.00 | -2.000 / 0.089 |
| opening; reference jerk bound 0.05 | aks | 225.0 | 221.3 | -0.00 | -0.000 | 30.00 / 50.00 | -0.311 / 0.233 |

The CSV also records the reference-endpoint error, final instantaneous-policy error, final speed and observation endpoint. All final errors above use the common 50 m/s equilibrium target, not the moving ACC target.

A sufficient unbound gap correctly stays oversized; the ceiling-limited refusal cannot close. Neither should be counted as a failed nominal recovery. Leader-braking variants are diagnostics, not constant-leader controller rankings.
