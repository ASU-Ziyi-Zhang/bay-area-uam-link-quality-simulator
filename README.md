# Bay Area UAM Link-Quality Simulator

A reproducible research toolkit for Caltrain-referenced Bay Area UAM
corridors. The repository packages four connected layers:

1. replaceable corridor scenario packs;
2. documented physical macro-site locations selected for each route;
3. deterministic RSRP/SINR link-quality analysis;
4. interactive single-UAM link-quality and multi-UAM group-policy simulators.

The current release is a communication-planning baseline. It includes a
deterministic reproduction of the TRB five-aircraft C/R/F policy-to-capacity
chain, but does **not** claim measured airborne coverage, verified operator
interoperability, operational conflict resolution, or certified corridor
capacity.

## Interactive dashboard

A hosted build is published from `dashboard/` on every push to `main`:

**https://asu-ziyi-zhang.github.io/bay-area-uam-link-quality-simulator/**

The header scenario selector switches between:

- `?scenario=sf_sj_full` — full 75.423 km corridor;
- `?scenario=airport_to_airport` — Millbrae–Santa Clara airport-access case.

The same query links work on the local server at `http://127.0.0.1:8765/`.
Use the **Multi-UAM policy** tab, or open
`traffic.html?scenario=airport_to_airport`, to inspect simultaneous aircraft,
policy fractions, and reliability-qualified planning capacity.

The map header links to the complete
[18-site macro-site layout](evidence/figures/corridor_sites.svg), showing the
corridor-wide `Macro_Tower`, `Macro_Building`, and `Macro_Other` classes and
marking sites with retained images.

No local environment is needed to open it. The 2D map, radio traces, telemetry,
and site records are self-contained. Two things are fetched at view time:
CesiumJS for the 3D panel, and OpenStreetMap raster tiles for both basemaps. If
a network blocks either, the 3D panel reports the failure and falls back while
the 2D corridor, charts, and site evidence continue to work; Leaflet is vendored
under `dashboard/vendor/leaflet/` so the 2D view never depends on a CDN.

Opening `dashboard/index.html` straight from the filesystem is not supported —
the 3D panel is disabled under `file://` because the aircraft model cannot be
loaded cross-origin. Use the hosted build or the local server below.

## Repository map

```text
configs/       frozen legacy settings retained for reference verification
.github/       verification and dashboard-publishing workflows
scenarios/     self-contained route, site, scenario, and simulator packs
data/          frozen legacy inputs retained for reference verification
evidence/      site register, selection rules, source links, and corridor map
src/           reusable link-quality and simulator packages
scripts/       run, dashboard, and verification entry points
dashboard/     interactive real-map and fixed-bearing 3D playback
results/       frozen reference results used for comparison
tests/         standalone tests with no external TRB dependency
runs/          local generated runs (ignored by Git)
docs/          architecture, assumptions, and reproducibility notes
```

## Install

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
```

## Run the link-quality analysis

```powershell
python scripts\run_link_quality.py `
  --scenario scenarios\sf_sj_full\scenario.json `
  --output runs\link_quality_full

python scripts\run_link_quality.py `
  --scenario scenarios\airport_to_airport\scenario.json `
  --output runs\link_quality_airport
```

The analysis evaluates a 300 m centerline trajectory and a prescribed
cross-section envelope at 2,001 longitudinal positions. It writes editable
CSV data, QA, figures, and a checksum manifest.

## Run the simulator

```powershell
python scripts\run_simulator.py `
  --config scenarios\airport_to_airport\simulator.json `
  --output runs\simulator_airport
python scripts\build_dashboard.py `
  --scenario scenarios\airport_to_airport\scenario.json `
  --run-dir runs\simulator_airport
python scripts\serve_dashboard.py
```

Open [http://127.0.0.1:8765/](http://127.0.0.1:8765/). The 3D view uses a
fixed 315° bearing and −25° pitch; it translates with the UAM but does not
rotate with aircraft heading.

## Run the multi-UAM group-policy simulator

```powershell
python scripts\run_group_simulator.py `
  --config scenarios\airport_to_airport\group_simulator.json `
  --output runs\airport-to-airport-group-policy-v2
python scripts\build_traffic_dashboard.py `
  --run-dir runs\airport-to-airport-group-policy-v2 `
  --scenario scenarios\airport_to_airport\scenario.json `
  --output dashboard\data\airport_to_airport_traffic.js
python scripts\serve_dashboard.py
```

Open
[http://127.0.0.1:8765/traffic.html?scenario=airport_to_airport](http://127.0.0.1:8765/traffic.html?scenario=airport_to_airport).
Green, yellow, and red aircraft denote coordinated, reactive, and fallback
group policy. Every active aircraft is classified; startup and corridor-edge
groups use the available local neighbors up to a maximum of five. See
[the simulator definition](docs/multi_aircraft_policy.md) before interpreting
the fractions or capacity.

## Verify

```powershell
python -m pytest -q
python scripts\verify_reference.py
```

See [docs/reproducibility.md](docs/reproducibility.md) for the exact evidence
level and [evidence/site_register.md](evidence/site_register.md) for the source
links behind the retained sites.

## Included scenarios

- `sf_sj_full`: the frozen 75.423 km SF 4th & King–San Jose Diridon baseline
  with BS01–BS18.
- `airport_to_airport`: the 49.543 km Millbrae Caltrain–Santa Clara Caltrain
  subcorridor requested by Toyota ITL, with BS05–BS16 under the same 5 km
  inclusion rule. These stations are airport-access proxies, not proposed
  vertiports on airport property.

See [scenarios/README.md](scenarios/README.md) and
[scenarios/registry.json](scenarios/registry.json).

## Publication note

Third-party Street View screenshots and downloaded municipal/FCC PDFs are not
redistributed here. The public package retains source URLs and derived research
data; the original private working directory remains the audit archive.

## Tooling and AI assistance

Parts of the code, tests, and documentation in this repository were developed
with AI coding assistants (OpenAI Codex and Anthropic Claude). Their use was
limited to implementation, refactoring, test authoring, and drafting; the
scenario definition, site selection and evidence review, modeling assumptions,
and the interpretation of all results are the author's own, and every reported
figure is reproducible from the committed inputs with the commands above.

## License

- Source code and software configuration: [MIT License](LICENSE).
- Original data, figures, and documentation: [CC BY 4.0](LICENSE-DATA).
- Third-party assets retain their own notices; see the license beside each asset.
  Vendored Leaflet 1.9.4 is BSD-2-Clause
  ([dashboard/vendor/leaflet/LICENSE](dashboard/vendor/leaflet/LICENSE)); the
  bundled aircraft model carries its own notice; CesiumJS is loaded from its
  pinned release URL under Apache-2.0.
