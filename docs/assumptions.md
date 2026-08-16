# Modeling assumptions

## Geometry

- Corridor: Caltrain-referenced SF–SJ centerline in WGS84, projected to
  EPSG:26910 before distance calculations.
- Baseline trajectory: one UAM, one lane, one level, 300 m AGL, no lateral
  offset, and 50 m/s constant speed.
- The simulator starts and ends on the corridor endpoints; lateral-offset
  previews taper to zero near both endpoints in the dashboard.

## Base stations

- Active set: BS01–BS18, all within 5 km of the centerline. The UAM associates
  with only the nearest three at any instant; see the served-set section below.
- Heights use official/site-record values where available. Otherwise the
  declared planning defaults are 15.24 m AGL for `Macro_Building` and
  18.288 m AGL for `Macro_Tower`.
- Coordinates and physical-site evidence do not establish live service,
  ownership of every antenna, UAM readiness, or one shared operator network.

## Radio

- Deterministic LOS planning kernel at 5 GHz.
- Full-carrier EIRP: 46 dBm; receiver gain: 0 dB; full-carrier noise: −99 dBm.
- RSRP is full-carrier received power minus `10 log10(300)` resource elements
  (−24.77 dB).
- No shadow fading, measured antenna pattern, sector orientation, downtilt,
  traffic loading, frequency reuse, terrain diffraction, or airborne
  calibration is applied.

### Path-loss model and its provenance

The kernel in `src/capacity_policy/radio.py` evaluates

```text
PL(dB) = 28.0 + 22 log10(d_3D / m) + 20 log10(f_c / GHz)
```

which is the LOS path loss of the **3GPP TR 36.777 UMa-AV** (urban macro,
aerial vehicle) model. The constants are taken from that model rather than
fitted here. Two consequences follow.

- The aerial LOS branch is specified for UE heights in `(22.5 m, 300 m]` and
  carries **no breakpoint term**; the piecewise breakpoint form belongs to the
  terrestrial branch for heights of 1.5–22.5 m. The single-slope evaluation
  used here is therefore the specified form, not a truncation of a piecewise
  one. The 300 m baseline sits exactly at the model's height ceiling.
- TR 36.777 reports LOS probability approaching 1 for UE altitudes above
  100 m at short range, which is why the kernel evaluates a deterministic LOS
  link instead of sampling a LOS/NLOS state.

### Served set and interference

`radio.served_set_size` is **3**. At each instant the UAM associates with the
nearest three sites; the strongest of that set serves and the remaining two are
co-channel interferers at full EIRP. Every site is still evaluated and reported
in `received_power_dbm`, but only the served set enters the association and the
interference sum.

Because every site radiates a common EIRP and the path loss is monotonic in
3D distance, **the nearest set is exactly the strongest set**. Selecting by
distance therefore cannot change which site serves, and the run QA asserts both
forms (`serving_is_max_received_power` and
`serving_is_global_max_received_power`). Switching from the earlier
all-18-sites construction left serving association, RSRP, and the handoff count
bit-for-bit unchanged and moved only SINR:

| Quantity | All 18 co-channel | Nearest 3 |
| --- | --- | --- |
| Interferers per sample | 17 | 2 |
| Serving site sequence | — | identical |
| Serving RSRP (min / median / max) | −102.63 / −95.64 / −74.85 | unchanged |
| Association transitions | 13 | 13 |
| Centerline SINR (min / median / max) | −5.49 / 0.56 / 19.64 dB | −2.67 / 2.16 / 22.16 dB |
| Cross-section p05 minimum SINR | −5.64 dB | −2.81 dB |
| Maximum noise fraction | 0.62 % | 1.36 % |

Setting `served_set_size` to `null` restores the all-sites worst-case bound for
comparison. Sector patterns, downtilt, frequency reuse, and traffic-dependent
loading remain unmodelled in both cases.

### Declared extrapolation beyond the model's validity range

TR 36.777 states the UMa-AV LOS branch for **`d_2D ≤ 4 km`**. The corridor
exceeds this, and restricting the served set reduces the exceedance without
removing it, because 18 sites along 75 km leave the second and third neighbours
far away:

| Quantity | Value |
| --- | --- |
| Serving-link 3D distance | min 0.29 km, median 2.53 km, max 5.26 km |
| Samples whose serving site is within 4 km | 1262 / 1510 (83.6 %) |
| Farthest served link, 2D | median 5.93 km, max 12.74 km |
| Samples with all three served links within 4 km | 212 / 1510 (14.0 %) |
| Interference power contributed from beyond 4 km | mean 68.0 % (was 72.8 %) |

The serving link, and therefore RSRP, stays close to the specified range. The
two interferers usually do not: their median separation is about 5.9 km. SINR
must therefore still be read as an extrapolation of TR 36.777 beyond its stated
distance range, not as a standards-validated prediction. Reducing the served
set from 18 to 3 addressed the size of the interference sum, not this range
question.

## Policy and capacity boundary

Policy, controller, multi-UAM conflict logic, and capacity aggregation remain
interfaces only. Current outputs are link-quality diagnostics and simulator
traces, not corridor-capacity results.
