# Link-quality review packet - SF-SJ real corridor

## Objective and acceptance criteria

Generate full-corridor serving-RSRP and SINR profiles from the retained 18-site scenario. Required checks are encoded in `qa.json`.

## Inputs and provenance

- Scenario, corridor and site-table checksums are recorded in `manifest.json`.
- 18 active sites; BS19 is not part of this run.
- Site-specific/class-imputed heights are read from the versioned station CSV.

## Specification

- Centerline trajectory: altitude 300 m.
- Spatial envelope: lateral ±500 m and altitude 300±60 m; 147 points at each of 2,001 longitudinal positions.
- Radio: 5 GHz, 46 dBm full-carrier EIRP, 0 dB receiver gain, −99 dBm full-carrier noise, deterministic LOS.
- All 18 sites are treated as co-channel and simultaneously active.
- Strict RSRP is reported per resource element using 300 RE; SINR uses full-carrier desired/interference/noise powers.

## Outputs

- `profile.csv`
- `sites.csv`
- `transitions.csv`
- Three figures in editable SVG and preview PNG formats.

## Validation

- QA status: **pass**.
- Longitudinal positions: 2,001.
- Cross-section evaluations: 294,147.
- Serving sites on centerline: 14 of 18.
- Association transitions: 13.
- All values finite: True.
- Serving association equals maximum received power: True.

## Results boundary

These are model-estimated spatial diagnostics. The shaded envelope is prescribed geometric variation, not statistical uncertainty. No C/R/F policy or capacity conclusion is made.

## Decision requested

Review the two profile figures and either approve this baseline, request a figure revision, or request a new sensitivity branch (for example operator-specific active sets or antenna-pattern assumptions).
