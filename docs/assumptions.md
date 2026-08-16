# Modeling assumptions

## Geometry

- Corridor: Caltrain-referenced SF–SJ centerline in WGS84, projected to
  EPSG:26910 before distance calculations.
- Baseline trajectory: one UAM, one lane, one level, 300 m AGL, no lateral
  offset, and 50 m/s constant speed.
- The simulator starts and ends on the corridor endpoints; lateral-offset
  previews taper to zero near both endpoints in the dashboard.

## Base stations

- Active set: BS01–BS18, all within 5 km of the centerline.
- Heights use official/site-record values where available. Otherwise the
  declared planning defaults are 15.24 m AGL for `Macro_Building` and
  18.288 m AGL for `Macro_Tower`.
- Coordinates and physical-site evidence do not establish live service,
  ownership of every antenna, UAM readiness, or one shared operator network.

## Radio

- Deterministic LOS planning kernel at 5 GHz.
- Full-carrier EIRP: 46 dBm; receiver gain: 0 dB; full-carrier noise: −99 dBm.
- RSRP is full-carrier received power minus `10 log10(300)` resource elements.
- All 18 sites are active and co-channel in the baseline interference
  sensitivity. This is a modeling scenario, not an operator-interoperability
  claim.
- No shadow fading, measured antenna pattern, terrain diffraction, or airborne
  calibration is applied.

## Policy and capacity boundary

Policy, controller, multi-UAM conflict logic, and capacity aggregation remain
interfaces only. Current outputs are link-quality diagnostics and simulator
traces, not corridor-capacity results.
