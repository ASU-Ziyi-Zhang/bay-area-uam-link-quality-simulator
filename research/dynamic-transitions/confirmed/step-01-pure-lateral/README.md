# Confirmed Step 01 — pure lateral movement

**Status:** confirmed by the user on 2026-08-31.

This is the settled first step of the Toyota sequence: one aircraft remains at
300 m altitude and may change only its signed lateral offset from the smoothed
airport-to-airport reference route. The minimum-change controller accepts one
`0 → +100 m` maneuver after an observed C-to-F degradation. It is a link-
quality/policy response result, not a multi-aircraft capacity result.

## Confirmed conclusion

With the R0009 radio model, singleton exposure history, and provisional
976 m curvature guard, the approved minimum-change flight completes one
100 m lateral maneuver beginning at `t=460 s` and ending at `t=568.869 s`.
Stable C is observed from `t=570 s`; staying on the centerline recovers at
`t=575 s`. The result is an illustrative baseline for the next spatial and
multi-UAM studies, not a calibrated aircraft or coverage certificate.

## Standing as of 2026-09-10

**Not contradicted, but read with two caveats.** No later run has overturned any
number on this page. This is a single aircraft with no following relationship, no
ACC, and no policy-spacing dynamics, so the three changes made since — the
30–80 m/s speed envelope, the KS2 longitudinal reference, and the corrected ACC
recovery law — do not touch it.

What has moved on is the acceleration bookkeeping:

* This run used a **provisional 976 m curvature guard**, a binary gate on where a
  maneuver may begin. R0022 later measured that the same 976 m minimum radius
  corresponds to **2.561 m/s² of corridor normal acceleration at 50 m/s**, a
  14.63° coordinated bank, before any lane-change component is added.
* The configured `lateral_accel_limit_mps2 = 0.3` **never declared whether it
  bounds the lane-change component alone or the total normal acceleration**. Under
  the component reading the result stands as approved. Under the total reading the
  corridor term alone already exceeds it. That ambiguity is the same defect that
  failed R0022's independent audit, and it is why the coupled envelope
  `(a_T/a_lon,max)² + (a_N/a_lat,max)² ≤ 1` with an explicit `maneuver_only` /
  `total` mode now exists.

**Still the only confirmed evidence** for the link-quality-to-policy response, so
it is retained rather than retired. Any reuse of the 0.3 m/s² limit should state
which reading it means.

## Canonical sources

- [R0009 report](../../reports/R0009.md)
- [R0009 protocol](../../minimum-change-protocol.md)
- [R0009 run archive](../../runs/R0009/)
- [Approved figure revision](../../figures/R0009/revision-02/)

The copied files in this folder are a presentation convenience. Their source
hashes are recorded below so the confirmed layer cannot silently drift from
the approved archive.

## Confirmed files

| File | Source | SHA-256 |
|---|---|---|
| `APPROVED.json` | `figures/R0009/APPROVED.json` | `bed0c7d9359007e1b0fae352e17d29a996ea05ea00e647dae3dfaedcffc01547` |
| `validation.json` | `validation-R0009.json` | `319b3d32851637fdc3f0d23bf0e2587eeaf4330049cf01d320c40226efa2f302` |
| `figures/01-radio-context.png` | `figures/R0009/revision-02/01-radio-context.png` | `8d538f88a108aeba3d04511dbfd8433277d8bb0f4aa1bfea53d94c14cccfab67` |
| `figures/01-radio-context.svg` | `figures/R0009/revision-02/01-radio-context.svg` | `ad90baabd74df527b3cbc9fd6959d6e2b9597336e76042f496b6406075d81c76` |

