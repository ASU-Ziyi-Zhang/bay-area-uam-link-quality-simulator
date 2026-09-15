# Research workspace

Research is organised by question. Reusable algorithms live in `src/`, entry
points in `scripts/`, shared scenario data in `scenarios/`. Each topic keeps its
own protocols, configurations and run registry here.

## What is published

This public package contains the protocols, configurations, confirmed results
and compact result tables and figures. **Raw run archives (`runs/`) and working
reports are not published**: they are large, include superseded and failed
runs, and stay in the author's local audit archive. Links inside protocols to
`runs/` or `reports/` therefore point to that archive. Every result quoted in
`docs/` names its run ID and can be regenerated from the configuration listed
below.

Start with the method documents:

- [Motion control: spacing, ACC/AKS, lane-change profiles, envelope, NMAC](../docs/motion_control.md)
- [Bay Area policy calibration](../docs/bay_area_calibration.md)
- [Corridor experiment: stay at the centre or use a 3 × 3 grid](../docs/corridor_experiment.md)

## Topics

| Topic | Question | Status |
|---|---|---|
| [dynamic-transitions](dynamic-transitions/README.md) | How should aircraft adjust spacing, change lane and use nearby positions when link quality changes? | longitudinal control, lane-change study and 3 × 3 corridor experiment complete |
| [policy-calibration](policy-calibration/README.md) | Which classifier settings suit the Bay Area coverage? | open-loop screen complete; settings fixed |
| [geometry-response](geometry-response/README.md) | How do fixed lateral offset and altitude change radio, policy and capacity? | exploratory position sweep |
| [corridor-optimization](corridor-optimization/README.md) | Can a smooth corridor shape between fixed endpoints improve planning capacity? | exploratory prototype |
| [_archive/itl-baselines](_archive/itl-baselines/README.md) | Early combined fixed-stream and path-search runs | read-only history |

## Key runs

| Run | Question | Configuration | Result |
|---|---|---|---|
| R0036 | ACC against AKS for all six C/R/F transitions, two aircraft | `dynamic-transitions/configs/two_uam_longitudinal_nmac.json` | AKS settles 5–14× sooner; 72 traces, zero failed checks ([confirmed](dynamic-transitions/confirmed/step-04-longitudinal-policy-transitions/revision-04/)) |
| R0063 | Change lane or stay; five lane-change profiles, five aircraft | `dynamic-transitions/configs/five_uam_lateral_step2_bezier.json` | saving follows the midline crossing; 0–21 at Bay Area magnitudes ([tables](dynamic-transitions/results/R0063-lane-change-profiles/)) |
| R0041–R0047 | Assessment window × arrival interval, closed loop | `dynamic-transitions/configs/bay_area_sparse_window_sweep.json` | 90 s window fixed ([report](dynamic-transitions/results/R0041-R0047-window-sweep.md)) |
| R0048 | Calibrated single lane, 32 s demand, longitudinal control | `dynamic-transitions/configs/bay_area_longitudinal_90s_demand.json` | 93/93, C/R/F 63.2/36.0/0.8% ([confirmed](dynamic-transitions/confirmed/step-03-bay-area-demand-stream/revision-03/)) |
| R0062 | Stay at centre against moving within a 3 × 3 grid | `dynamic-transitions/configs/bay_area_single_stream_capacity_3x3_neighbourhood.json` | C 62.7 → 73.8%; 95% planning rate 107.7 → 111.8 UAM/h ([results](dynamic-transitions/results/R0062-corridor-3x3/)) |
| R0064 | Single lane with TRB classifier settings (reference for R0062) | `dynamic-transitions/configs/bay_area_single_stream_trb_settings.json` | F 42.3% vs 0.7%; 95% planning rate 69.3 vs 107.7 UAM/h ([results](dynamic-transitions/results/R0064-calibration-before-after/)) |

## Workflow for a new experiment

1. Write or amend the topic protocol: question, controls, fixed inputs, metrics,
   acceptance thresholds and model limits, before running.
2. Put the configuration under the topic's `configs/`.
3. Run with the next unused run ID of that topic. Runners never overwrite an
   existing run directory and archive the resolved configuration, source
   snapshot and hashes.
4. Verify with the matching `scripts/verify_*.py`. A failed verification is
   recorded, not relabelled; thresholds are never loosened after a result is seen.
5. Only verified results enter `confirmed/` or `results/`.
