# Bay Area policy calibration

The C/R/F classifier inherited from the TRB study was re-examined on the
Millbrae–Santa Clara (`airport_to_airport`) corridor. The TRB policy logic is
kept; only its settings change.

## What was tested

| Parameter | Values tested | Selected | What decided it |
|---|---|---|---|
| SINR threshold Θ | −4 to +1 dB, 0.1 dB steps | **−2.0 dB** | TRB −1.5 dB gives 33–38% F; −2.0 dB gives 2–7% |
| Radio sampling | 1, 2, 5 s | **2 s** | fewest switches near the operating threshold: 4.7 per flight (5 s: 12.9) |
| Persistence k | 1, 2, 3 policy updates | **3** | switches 4.7 → 2.4 per flight |
| Assessment window | 15, 30, 60, 120 s open loop; 30, 60, 90, 120 s closed loop | **90 s** | 32 s stream: F 18.7% at 30 s, 9.5% at 90 s |
| Group size | 3, 5, 7 aircraft | **5** | F 17.6 / 2.1 / 5.0% |
| Group form | same cell (two ahead, two behind); neighbourhood (own + adjacent cells) | **neighbourhood** | full five-aircraft groups on the 3 × 3 grid: 41.3% → 88.6% |
| Exposure tolerance C / R | 5% / 10%; integer budgets up to 5 bad observations | **5% / 10%** | kept from TRB |
| Measurement noise σ | 0, 0.5, 1.0 dB, 20 seeds each | not calibrated | stress test only: σ = 1 dB raises F from 41% to 69% |

The policy update interval is fixed at 5 s throughout.

**Persistence k** was not part of the TRB classifier. A new raw classification
must appear in k consecutive 5 s updates before the reported policy changes
(k = 3: at least 10 s confirmation). It filters short excursions after
classification and does not change SINR samples or the exposure equation.

## Stages

1. **Open-loop screen** (threshold, sampling, group size, persistence). Real
   route, 112.5 UAM/h, 300 m, 50 m/s, longitudinal feedback and lane changes
   off, 30 s window, so only the classifier is compared. Report and figures:
   [research/policy-calibration/deliverables/TEMA/report.md](../research/policy-calibration/deliverables/TEMA/report.md).
2. **Closed-loop window sweep** (R0039, R0041–R0047). 32 s and 90 s requested
   arrival intervals × 30/60/90/120 s windows with longitudinal control on.
   Longer windows reduce F severity but not all chatter, and the effect is not
   monotone (60 s is worse than 30 s). 90 s was fixed for motion control.
   [Report and source table](../research/dynamic-transitions/results/R0041-R0047-window-sweep.md).
3. **Grouping on a grid** (R0061 against R0062). With aircraft spread over a
   3 × 3 grid, same-cell groups fragment into groups of one to four and the two
   cases are classified on different evidence. The neighbourhood group takes the
   four nearest aircraft by along-track distance from the own and adjacent cells,
   capped at 2 S_F(cruise). See [corridor_experiment.md](corridor_experiment.md).

## Before and after on one lane (R0064 against R0062)

Same route, radio field, 32 s demand (93 requests), spacing law, AKS
controller and dispatcher; one lane at 300 m, no lane change. R0064 uses the TRB
classifier settings; R0062 (`fixed_centerline`) the calibrated ones. Compared
window 16.5–49.1 min.

| | TRB settings (R0064) | Calibrated (R0062) |
|---|---:|---:|
| Settings | Θ −1.5 dB, 5 s, 30 s, k = 1, lane order | Θ −2.0 dB, 2 s, 90 s, k = 3, neighbourhood |
| C / R / F aircraft-time | 38.8 / 18.9 / 42.3% | 62.7 / 36.6 / 0.7% |
| Planning rate, mean / 95% reliable | 75.9 / 69.3 UAM/h | 112.2 / 107.7 UAM/h |
| Policy switches per flight | 27.7 | 6.6 |
| Aircraft held at entry (longest) | 85 (701 s) | 54 (107.5 s) |
| Last aircraft out | 84.6 min | 68.3 min |
| Completed / sampled NMAC | 93/93, none | 93/93, none |

R0064 changes all classifier settings at once, so it shows the combined effect
of the calibration, not the contribution of each parameter. Figures:
[results/R0064-calibration-before-after](../research/dynamic-transitions/results/R0064-calibration-before-after/).

## Scope

Threshold, sampling, exposure and persistence are working values for this
corridor and radio model. The measurement-noise screen shows that receiver
error, reporting granularity, update period and correlation are needed before a
final calibration. No certified capacity is claimed.

## Reproduce

```sh
python scripts/run_bay_area_three_lane_90s.py --config research/dynamic-transitions/configs/bay_area_single_stream_trb_settings.json --output research/dynamic-transitions/runs/<TRB_ID>
python scripts/run_bay_area_three_lane_90s.py --config research/dynamic-transitions/configs/bay_area_single_stream_capacity_3x3_neighbourhood.json --output research/dynamic-transitions/runs/<CAL_ID>
python scripts/verify_bay_area_single_stream_capacity.py research/dynamic-transitions/runs/<TRB_ID>
python scripts/plot_calibration_comparison.py --trb research/dynamic-transitions/runs/<TRB_ID> --calibrated research/dynamic-transitions/runs/<CAL_ID> --output <folder>
```

`run_bay_area_three_lane_90s.py` is the shared corridor runner for all
`bay_area_*` study configurations (the name is historical).
