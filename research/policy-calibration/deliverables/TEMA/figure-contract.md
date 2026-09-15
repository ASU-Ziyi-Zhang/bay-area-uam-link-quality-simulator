# Figure contract — staged Bay Area policy calibration

Core conclusion: With the original exposure tolerances held fixed, the SINR threshold is the dominant control of the C/R/F distribution; 1 s and 2 s sampling give similar policy fields, while 5 s sampling is coarser and produces more switching, and persistence must be evaluated separately rather than mixed into the threshold comparison.

Figure archetype: quantitative grid.

Target output: concise technical report for ASU/Toyota discussion.

Backend: Python/matplotlib only.

Final size: 183 mm double-column width; SVG with editable text, PDF, and 300 dpi PNG.

Panel map:

- Figure 1: three matched single-factor threshold sweeps (1, 2, and 5 s sampling), plus switching-rate comparison.
- Figure 2: threshold × sampling heatmaps for C, R, F, and the AKS timing-compatibility proxy.
- Figure 3: preliminary group-size comparison after fixing the provisional threshold at -2.0 dB.
- Figure 4: persistence definition and effect, using identical radio/policy settings for k=1,2,3.

Evidence hierarchy:

- Hero evidence: matched C/R/F-versus-threshold curves.
- Validation evidence: two-variable heatmaps and group-size comparison.
- Control/robustness evidence: persistence-only policy histories and actionability proxy.

Statistics/source data: deterministic fixed-trajectory Bay Area run; no sampling uncertainty for the main sweep. All plotted rows are exported to CSV. The noise study remains separate.

Image integrity: policy timelines are categorical model outputs; white means inactive time and is not missing-data imputation.

Reviewer risks:

- -2.0 dB is a provisional simulation working point, not a receiver-certified requirement.
- “AKS timing-compatible” is a dwell-time comparison against R0036 settling times, not a new closed-loop AKS completion experiment.
- The policy screen uses fixed trajectories with no longitudinal or lateral feedback.
- Adaptive focal groups include smaller edge groups and available-history startup, matching the current geographic runner rather than the paper's complete-group abstraction.
