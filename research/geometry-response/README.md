# Geometry response

How a fixed lateral offset d and altitude h change the radio, policy and
capacity of one stream. No dynamic lane change. Read the [protocol](protocol.md).

## Configurations

- `configs/geometry_sweep.json`: two scenarios, 90 independent offset/altitude
  positions each.
- `configs/fixed_streams.json`: fixed layouts, demand allocation and estimator
  comparison.

Run from the repository root with the next unused run ID:

```sh
python scripts/run_geometry_sweep.py --output research/geometry-response/runs/<ID>
python scripts/run_baseline_studies.py --study lanes --output research/geometry-response/runs/<ID>
```

## Runs

| Run | Question / configuration | Status |
|---|---|---|
| R0001 | fixed per-stream demand 112.5 UAM/h, 9 offsets × 10 altitudes, two scenarios | development result (local archive) |

Altitudes above 300 m are radio-model extrapolations. The early 1×1 / 2×1 /
1×2 / 2×2 layouts are in [_archive/itl-baselines](../_archive/itl-baselines/README.md).
