# Confirmed Step 03 revision 01 — 90 s window and staged Bay Area sequence

**Status:** configuration and execution sequence explicitly confirmed by the user
on 2026-09-12. This revision confirms the decision, not unrun or unreviewed
numerical outcomes.

## Confirmed configuration

| Parameter | Value |
|---|---:|
| SINR threshold | -2.0 dB |
| Radio sampling | 2 s |
| Assessment window | **90 s** |
| Policy update | 5 s |
| C/R exposure | 5% / 10% bad |
| Persistence | k=3 |
| Primary traffic input | requested arrival interval and fixed demand horizon |

The 90 s window replaces 30 s as the main setting for the next Bay Area
motion-control runs. A 120 s window remains a robustness comparison; 30 s remains
historical/dense-stream reference only.

## Confirmed order

1. Rerun the 32 s demand stream on one lane at 300 m with longitudinal control
   only and the fixed 90 s window.
2. At 300 m, run a matched three-lane pair: longitudinal-only stay versus
   longitudinal plus lane changing, using the same global requested arrivals.
3. After the same-altitude gate passes, add multiple fixed height levels and then
   enable vertical/joint transitions.

For the principal three-lane comparison, one global request every 32 s is assigned
across the lanes, preserving total demand at 112.5 UAM/h. A 32 s interval on every
lane would instead be a separately labelled 337.5 UAM/h stress test.

## Evidence and boundary

The window sweep found that, for the 32 s short stream, increasing the window from
30 to 90 s reduced F from 18.7% to 9.5% and operational switches from 13.0 to 10.4
per flight. A 120 s window reduced them further to 6.2% and 8.8, but carries longer
memory. The user selected 90 s as the main operating compromise.

This revision does not confirm capacity, lane-change benefit, full longitudinal
settling, receiver accuracy or any future numerical result. Those enter later
revisions only after their run and independent validation exist.

## Canonical sources

- [Approved staged protocol](../../../bay-area-90s-staged-protocol.md)
- [90 s one-lane configuration](../../../configs/bay_area_longitudinal_90s_demand.json)
- [Window-sweep report](../../../reports/R0041-R0047-window-sweep.md)
- [Window-sweep source table](../../../reports/R0041-R0047-window-sweep.csv)

Source hashes and the exact authorization scope are recorded in `APPROVED.json`.
