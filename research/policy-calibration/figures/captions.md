# Figure captions

## Figure 1 — Parameter overview

Fixed Bay Area geographic route and traffic, with no longitudinal feedback or lane change. Panels a–b show adaptive-focal C/R/F shares over the tested SINR threshold range. The dashed line marks the historical -1.5 dB setting. Panels c–d use 1 s SINR sampling, 5 s policy updates, k=3 persistence and Theta=-1.5 dB. Actionability is the fraction of evaluable policy transitions whose observed dwell lasts at least the corresponding nominal R0036 AKS settling time.

## Figure 2 — Integer exposure budget

Complete five-aircraft groups, 30 s window, 5 s sampling and k=1. Each cell is one valid pair b_C<b_R among 35 member-time observations. The black outline marks the historical (1,3) budget. Blank cells violate the ordered budget. Capability boundaries use a 95% policy-share requirement and the tested 0.1 dB threshold grid.

## Figure 3 — Noise-resolution stress

Complete five-aircraft groups, 30 s window and historical 5%/10% exposure tolerances. Nonzero sigma conditions show means and 5th–95th percentiles across 20 iid Gaussian seeds; sigma=0 is deterministic. This is an illustrative stress test, not receiver calibration. It excludes quantization, temporal correlation and slow bias.

## Figure 4 — Policy timelines

Adaptive-focal C/R/F policy for all 93 fixed-trajectory aircraft at Theta=-1.5 dB. White denotes time outside an aircraft's active route interval. SINR sampling is 5 or 1 s as labeled; policy updates remain fixed at 5 s in every panel. These traces visualize classification persistence only and do not include longitudinal or lateral feedback.
