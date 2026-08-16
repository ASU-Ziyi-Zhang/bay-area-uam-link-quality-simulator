# Release review packet

## Objective

Publish one clean, standalone, reproducible repository containing the SF-SJ
corridor, retained base stations, link-quality analysis, and simulator dashboard.

## Canonical inputs

- `data/corridor.geojson`: 75.423116 km Caltrain-referenced centerline.
- `data/base_stations.csv`: 18 retained physical macro sites (BS01-BS18).
- `configs/scenario.json`: radio and scenario assumptions.
- `configs/simulator.json`: one-UAM timing and trajectory baseline.

## Reproduction commands

```powershell
python -m pip install -e ".[dev]"
python -m pytest -q
python scripts\run_link_quality.py --output runs\link_quality
python scripts\run_simulator.py --output runs\simulator
python scripts\build_dashboard.py --run-dir runs\simulator
python scripts\verify_reference.py
```

## Verified results

- Tests: 6 passed.
- Link quality: QA passed; 2,001 longitudinal positions and 294,147
  cross-section evaluations; 14 serving sites and 13 association transitions.
- Simulator: 1,510 radio-state rows, 27,180 per-site measurements, and 13
  handoffs; endpoint reached.
- Reference verification: 14 files and 9 declared metrics passed.

## Evidence grade and limitations

This is a rerunnable deterministic planning-model baseline. It has not been
calibrated against airborne measurements and does not establish carrier
interoperability, C/R/F policy performance, multi-UAM conflict resolution, or
corridor capacity.

## Release decision

Approved release target: public repository
`bay-area-uam-link-quality-simulator`, with MIT for code and CC BY 4.0 for
original data, figures, and documentation.
