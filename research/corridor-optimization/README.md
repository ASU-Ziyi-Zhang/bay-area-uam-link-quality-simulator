# Corridor optimization

Continuous corridor shape between fixed endpoints; not a constant-offset scan
and not aircraft moving between lanes. Read the [protocol](protocol.md); the
shared method is in the
[historical protocol](../_archive/itl-baselines/protocol.md#study-b-endpoint-fixed-lateral-control-optimization).

Configuration: `configs/offset_search.json`. Run from the repository root with the
next unused run ID:

```sh
python scripts/run_baseline_studies.py --study offsets --output research/corridor-optimization/runs/<ID>
```

No new independent run yet. The existing prototype results are the `offsets/`
folders of run R0003 in [_archive/itl-baselines](../_archive/itl-baselines/README.md).
