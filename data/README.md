# Scenario data

- `corridor.geojson`: frozen Caltrain-referenced SF–SJ centerline in EPSG:4326.
- `base_stations.csv`: frozen retained BS01–BS18 macro-site register, coordinates,
  class, physical form, source links, and modeling height.

The station CSV is an evidence-backed planning layer, not a live operator RF
inventory. See `../evidence/site_register.md` and `../docs/assumptions.md`.

These root-level files are retained to verify the published reference results.
New runs should load a self-contained pack under `scenarios/`; the canonical
full-corridor copies are in `scenarios/sf_sj_full/data/`.
