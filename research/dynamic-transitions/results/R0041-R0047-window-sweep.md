# Sparse-flow assessment-window sweep

Date: 2026-09-12  
Runs: R0039 and R0041--R0047  
Status: all new runs independently replayed and passed.

## Question and controlled comparison

This experiment tests whether the 30 s exposure window is too short when a
departure stream produces singleton or otherwise small policy groups. The SINR
threshold, 2 s radio sampling, 5 s policy update, 5%/10% exposure tolerances,
`k=3`, route, radio field and longitudinal controller are fixed. Only requested
arrival interval and assessment-window duration vary.

The 32 s cases revisit the earlier short five-request stream. The 90 s cases
start consecutive aircraft farther apart than the 3569.9 m policy-group radius;
all 2475 observations in each of those cases are therefore singleton observations.
The finite request count is a matched diagnostic input, not the traffic variable.

## Results

### 32 s requested-arrival interval

| Window | C/R/F aircraft-time | Operational switches/flight | Raw switches/flight | Speed-direction reversals/flight | Timing compatible | Fully settled |
|---:|---:|---:|---:|---:|---:|---:|
| 30 s (R0039) | 66.0 / 15.3 / 18.7% | 13.0 | 24.4 | 16.8 | 14.5% | 0/55 |
| 60 s (R0041) | 56.3 / 31.2 / 12.5% | 14.2 | 19.2 | 17.6 | 21.1% | 1/57 |
| 90 s (R0042) | 57.1 / 33.4 / 9.5% | 10.4 | 12.0 | 15.6 | 37.5% | 0/40 |
| 120 s (R0043) | 57.7 / 36.1 / 6.2% | 8.8 | 11.2 | 13.6 | 38.9% | 0/36 |

Longer windows reduce F after the non-monotonic 60 s case. At 120 s, F is
12.5 percentage points lower than at 30 s, operational switching is 32% lower,
and speed-direction reversal is 19% lower. Much of the removed F becomes R rather
than C. Most importantly, the controller still fully settles only 0 of 36 tasks,
so smoothing the classifier does not by itself solve longitudinal tracking.

### 90 s requested-arrival interval: singleton policy groups

| Window | C/R/F aircraft-time | Operational switches/flight | Raw switches/flight | Speed-direction reversals/flight | Timing compatible | Fully settled |
|---:|---:|---:|---:|---:|---:|---:|
| 30 s (R0044) | 84.8 / 0.0 / 15.2% | 8.0 | 16.0 | 0.0 | 12.5% | 0/32 |
| 60 s (R0045) | 74.7 / 0.0 / 25.3% | 8.0 | 16.0 | 0.0 | 12.5% | 0/32 |
| 90 s (R0046) | 63.6 / 18.2 / 18.2% | 8.0 | 12.0 | 0.0 | 42.9% | 0/28 |
| 120 s (R0047) | 52.5 / 47.5 / 0.0% | 8.0 | 8.0 | 0.0 | 71.4% | 0/28 |

The singleton result confirms the discrete-duration concern. A 120 s window
eliminates operational F in this particular route trace, but it does so mainly by
turning the weak periods into R; it does not reduce the eight operational policy
switches per flight. The 60 s window is worse than 30 s in F share, showing that a
longer rolling average need not improve every metric monotonically.

All 90 s cases remain exactly at 50 m/s. Their preceding gaps exceed all commanded
policy spacings, so the controller does not chase the leader or alternate between
acceleration and deceleration. These cases are therefore valid classifier-memory
tests, but not longitudinal-response comparisons.

## Decision

The 30 s window is too sensitive to use as the only setting for sparse/singleton
policy groups, but the present results do not justify replacing it blindly with
120 s. The longer window suppresses F severity while retaining operational policy
chatter and introduces substantially longer memory.

Use 30 s as the existing dense-stream reference and carry 90 s and 120 s as the
sparse-flow candidates. The next discriminating test should inject or select
known 2/4/6/10/15/20 s below-threshold episodes and measure both entry into F and
recovery delay. That test can distinguish genuine chatter suppression from merely
delaying or relabelling degradation.

For the later lane-change study, define demand by global and per-lane requested
arrival interval. Run matched longitudinal-only and lane-change-enabled cases at
the same demand. Do not use a fixed aircraft count as the experiment label.

## Reproducibility

- Protocol: [`bay-area-sparse-window-sweep-protocol.md`](../bay-area-sparse-window-sweep-protocol.md)
- Base configuration: [`bay_area_sparse_window_sweep.json`](../configs/bay_area_sparse_window_sweep.json)
- Source table: [`R0041-R0047-window-sweep.csv`](R0041-R0047-window-sweep.csv)
- Run archives: [`R0041`](../runs/R0041/), [`R0042`](../runs/R0042/),
  [`R0043`](../runs/R0043/), [`R0044`](../runs/R0044/),
  [`R0045`](../runs/R0045/), [`R0046`](../runs/R0046/),
  [`R0047`](../runs/R0047/)
- Analysis script: [`analyze_bay_area_window_sweep.py`](../../../scripts/analyze_bay_area_window_sweep.py)

No Git commit or push was made.
