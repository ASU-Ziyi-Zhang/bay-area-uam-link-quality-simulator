# Dynamic transitions

How an aircraft adjusts its spacing, changes lane or moves to a nearby position
when its communication policy changes. Method summary:
[docs/motion_control.md](../../docs/motion_control.md) and
[docs/corridor_experiment.md](../../docs/corridor_experiment.md).

## Layout

| Folder | Content |
|---|---|
| `configs/` | every study configuration; the `protocol` field names its protocol |
| `confirmed/` | curated, approved results with figures, validation and hashes; never overwritten |
| `results/` | compact tables and figures regenerated from verified runs |
| `runs/` | raw archives (kept locally, not published) |

## Protocols

| Stage | Protocol | Runs |
|---|---|---|
| Two-aircraft longitudinal control (ACC, AKS) | [two-uam-longitudinal-protocol.md](two-uam-longitudinal-protocol.md), [aks-longitudinal-protocol.md](aks-longitudinal-protocol.md) | R0024–R0033, **R0036** |
| Coupled motion envelope | [motion-envelope-protocol.md](motion-envelope-protocol.md) | R0022, R0023 |
| Separation and NMAC screening | [separation-screening-protocol.md](separation-screening-protocol.md) | |
| Five-aircraft lane change | [three-uam-lateral-protocol.md](three-uam-lateral-protocol.md) | R0034, R0035, **R0063** |
| Bay Area single-lane demand stream | [bay-area-90s-staged-protocol.md](bay-area-90s-staged-protocol.md), [bay-area-longitudinal-large-stream-protocol.md](bay-area-longitudinal-large-stream-protocol.md), [bay-area-dev3-protocol.md](bay-area-dev3-protocol.md) | R0037–R0040, **R0048** |
| Assessment-window sweep | [bay-area-sparse-window-sweep-protocol.md](bay-area-sparse-window-sweep-protocol.md) | **R0041–R0047** |
| 3 × 3 corridor capacity | [bay-area-single-stream-capacity-protocol.md](bay-area-single-stream-capacity-protocol.md) | R0056, R0060 (void), R0061, **R0062**, **R0064** |
| Three parallel streams (exploratory) | [bay-area-three-stream-capacity-protocol.md](bay-area-three-stream-capacity-protocol.md) | |
| Earlier single-aircraft and multi-aircraft stages | [singleton-lateral-protocol.md](singleton-lateral-protocol.md), [minimum-change-protocol.md](minimum-change-protocol.md), [spatial-transition-protocol.md](spatial-transition-protocol.md), [step-03-multi-uam-acc-protocol.md](step-03-multi-uam-acc-protocol.md), [step-03-nominal-recovery-protocol.md](step-03-nominal-recovery-protocol.md), [step-03-trb-traffic-scale-protocol.md](step-03-trb-traffic-scale-protocol.md), [dual-horizon-controller-method.md](dual-horizon-controller-method.md) | R0001–R0021 |
| Model review | [full-mission-model-review.md](full-mission-model-review.md) | |

Bold runs are the current results. Earlier runs in each row are superseded or
diagnostic; their failures and the corrections they led to are recorded in the
protocol amendments.

## Current results

| Run | Where |
|---|---|
| R0036 | [confirmed/step-04-longitudinal-policy-transitions/revision-04](confirmed/step-04-longitudinal-policy-transitions/revision-04/) |
| R0048 | [confirmed/step-03-bay-area-demand-stream/revision-03](confirmed/step-03-bay-area-demand-stream/revision-03/) |
| R0041–R0047 | [results/R0041-R0047-window-sweep.md](results/R0041-R0047-window-sweep.md) |
| R0063 | [results/R0063-lane-change-profiles](results/R0063-lane-change-profiles/) |
| R0062 | [results/R0062-corridor-3x3](results/R0062-corridor-3x3/) |
| R0064 | [results/R0064-calibration-before-after](results/R0064-calibration-before-after/) |
