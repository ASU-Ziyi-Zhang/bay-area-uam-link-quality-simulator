# Bay Area policy calibration

Open-loop screen of the C/R/F classifier on the `airport_to_airport` corridor,
first run 2026-09-11. Summary and the selected settings:
[docs/bay_area_calibration.md](../../docs/bay_area_calibration.md).

## Conditions

Real route, 12 scenario base stations, 112.5 UAM/h, 300 m altitude, 50 m/s,
fixed single stream; longitudinal feedback and lane changes off, so only the
classifier is compared. Policy updates every 5 s.

| Parameter | Values |
|---|---|
| SINR threshold Θ | −4 to +1 dB, 0.1 dB steps |
| Radio sampling interval | 1, 2, 5 s |
| Assessment window | 15, 30, 60, 120 s |
| Target group size | 3, 5, 7 aircraft |
| Persistence k | 1, 2, 3 policy updates |
| Exposure | nominal 5% / 10%; for N = 35 all integer budgets 0 ≤ b_C < b_R ≤ 5 |
| Illustrative measurement error σ | 0, 0.5, 1.0 dB (20 iid Gaussian seeds each) |

Two group definitions are kept: `complete_group` (full centred groups, full
history, route-crossing warm-up) and `adaptive_focal` (every aircraft as focal,
edge groups and available history, as in the corridor runner). The main
judgement uses `adaptive_focal`.

Not screened: receiver reporting granularity, hysteresis bands, time-correlated
or biased error, transmit power, and closed-loop motion. The closed-loop window
sweep that fixed 90 s is in
[../dynamic-transitions/results/R0041-R0047-window-sweep.md](../dynamic-transitions/results/R0041-R0047-window-sweep.md).

## Files

| Path | Content |
|---|---|
| `results/*_summary.csv`, `results/*_curves.csv` | policy shares, switches, dwell, AKS timing compatibility and capability boundaries per setting, for both group definitions |
| `results/integer_budget_summary.csv` | integer exposure-budget scan |
| `results/iid_noise_seed_summary.csv` | measurement-error stress test |
| `results/resolved_config.json` | resolved screening configuration |
| `figures/` | overview, integer budget, noise stress and policy timelines (`captions.md`) |
| `run.py`, `plot.py` | screening run and figures |
| `deliverables/TEMA/report.md`, `plot_report.py` | technical note with the staged analysis and figures |

Reproduce from the repository root (uses `src/capacity_policy`):

```sh
python research/policy-calibration/run.py --output research/policy-calibration/results-new
python research/policy-calibration/plot.py
```
