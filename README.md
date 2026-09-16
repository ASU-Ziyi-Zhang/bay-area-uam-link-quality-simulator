# Bay Area UAM Link-Quality Simulator

A reproducible research toolkit for Caltrain-referenced Bay Area UAM
corridors. The repository packages four connected layers:

1. replaceable corridor scenario packs;
2. documented physical macro-site locations selected for each route;
3. deterministic RSRP/SINR link-quality analysis;
4. interactive single-UAM link-quality and multi-UAM group-policy simulators;
5. multi-UAM motion control: policy-dependent longitudinal spacing, lane
   changes, motion limits and corridor capacity experiments (new in 0.2.0).

The release is a communication-planning research toolkit. It includes a
deterministic reproduction of the TRB five-aircraft C/R/F policy-to-capacity
chain and the motion-control studies built on it, but does **not** claim
measured airborne coverage, verified operator interoperability, certified
separation, or certified corridor capacity. See [CHANGELOG.md](CHANGELOG.md).

## Interactive dashboard

A hosted build is published from `dashboard/` on every push to `main`:

- [Single-UAM link-quality dashboard](https://asu-ziyi-zhang.github.io/bay-area-uam-link-quality-simulator/)
- [Multi-UAM policy dashboard](https://asu-ziyi-zhang.github.io/bay-area-uam-link-quality-simulator/traffic.html?scenario=airport_to_airport)
- [Multi-UAM lane-change dashboard](https://asu-ziyi-zhang.github.io/bay-area-uam-link-quality-simulator/motion.html): one entry stream with spacing control, staying on the centre lane or changing lane within a 3 × 3 lateral/altitude grid

The header scenario selector switches between:

- `scenario=sf_sj_full` — full 75.423 km corridor;
- `scenario=airport_to_airport` — Millbrae–Santa Clara airport-access case.

The same query links work on the local server at `http://127.0.0.1:8765/`.
Use the **Multi-UAM policy** tab, or open
`traffic.html?scenario=airport_to_airport`, to inspect simultaneous aircraft,
policy fractions, and reliability-qualified planning capacity.

The multi-UAM page can deterministically rerun the selected scenario in the
browser. Editable inputs include altitude, route-relative lateral offset,
speed, departure interval, SINR threshold, local-group size, exposure window,
policy-update interval, C/R exposure limits, and the reliability level.
Offered demand is then derived as `3600 / departure interval`; the 1-lane x
1-level geometry and fixed control are scenario status in this baseline mode.
Clicking an aircraft on the 2D corridor synchronizes its detail record, radio
profile, serving link, and the focal-aircraft 3D camera.

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
research/      protocols, study configs, confirmed results and result tables (raw runs ignored)
runs/          local generated runs (ignored by Git)
docs/          architecture, methods, assumptions, and reproducibility notes
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
  --output runs\airport-to-airport-group-policy-calibrated
python scripts\build_traffic_dashboard.py `
  --run-dir runs\airport-to-airport-group-policy-calibrated `
  --scenario scenarios\airport_to_airport\scenario.json `
  --output dashboard\data\airport_to_airport_traffic.js
python scripts\serve_dashboard.py
```

Open
[http://127.0.0.1:8765/traffic.html?scenario=airport_to_airport](http://127.0.0.1:8765/traffic.html?scenario=airport_to_airport).
Green, yellow, and red aircraft denote coordinated, reactive, and fallback
group policy. Every active aircraft is classified; startup and corridor-edge
groups use the available local neighbors up to a maximum of five. The calibrated
default uses a -2.0 dB SINR threshold, 2 s radio sampling, 90 s exposure window,
5 s policy updates, 5%/10% C/R exposure tolerances, persistence k=3 and a
152.4 m spacing constant (the Bay Area calibration). See
[the simulator definition](docs/multi_aircraft_policy.md) before interpreting
the fractions or capacity. Radio observations are sampled every 2 s and policy
snapshots are updated every 5 s; the accepted five-second TRB baseline remains
a separately labeled regression reference.

## Run the lane-change dashboard

The page replays both cases of the corridor experiment from a verified run
archive. To rebuild its data from a run of
`research/dynamic-transitions/configs/bay_area_single_stream_capacity_3x3_neighbourhood.json`:

```powershell
python scripts\build_motion_dashboard.py `
  --run research\dynamic-transitions\runs\<ID> `
  --scenario scenarios\airport_to_airport\scenario.json `
  --output dashboard\data\airport_to_airport_motion.js
python scripts\serve_dashboard.py
```

Open [http://127.0.0.1:8765/motion.html](http://127.0.0.1:8765/motion.html).
Switch between **No lane change** and **Lane change in 3 × 3**; select an aircraft to
follow its policy, controller (cruise, AKS reference tracking or ACC feedback),
gap against its spacing target, lane change and serving link. The cross-section
shows how many aircraft fly in each lateral/altitude cell.

## Motion control and corridor experiments

Version 0.2.0 adds how aircraft respond when their communication policy
changes. The spacing target is S = d0 + τ_p v + b v² with d0 = 152.4 m (the NMAC
distance) and τ_C / τ_R / τ_F = 15 / 30 / 60 s, so at 50 m/s C, R and F require
1320, 2070 and 3570 m.

| Study | Question | Result | Method |
|---|---|---|---|
| Longitudinal control (R0036) | ACC or AKS when the spacing target jumps? | AKS settles all six C/R/F transitions 5–14× sooner (C → F: 221 s against 1201 s) | [motion_control.md](docs/motion_control.md) |
| Lane change (R0063) | Five lane-change profiles, change or stay? | the saving follows when the aircraft crosses the lane midline, not the curve family; the quintic is kept | [motion_control.md](docs/motion_control.md) |
| Motion limits | Which limits must every manoeuvre meet? | coupled longitudinal/lane-change envelope, route-turn limit, NMAC (500 ft / 100 ft) counted in every run | [motion_control.md](docs/motion_control.md) |
| Calibration (R0064 vs R0062) | Which classifier settings suit the Bay Area coverage? | Θ −2.0 dB, 2 s sampling, 90 s window, k = 3: on one lane F falls from 42% to 0.7% and the 95% planning rate rises from 69 to 108 UAM/h | [bay_area_calibration.md](docs/bay_area_calibration.md) |
| 3 × 3 corridor (R0062) | Stay at the centre or move among nearby positions? | C time 62.7% → 73.8%; 95% planning rate +3.8% for the same demand | [corridor_experiment.md](docs/corridor_experiment.md) |

Every study has a protocol, a configuration and an independent verifier;
[research/README.md](research/README.md) lists them with the commands to rerun.
Raw run archives are not part of the package.

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
