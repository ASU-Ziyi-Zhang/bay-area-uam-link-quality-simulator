# Reproducibility contract

The repository is intended to be rerunnable in a clean Python environment.
Every generated run records configuration, software versions, input hashes,
output hashes, QA results, and limitations.

## Reference contract

The frozen files under `results/` are the comparison baseline. A reproduction
passes when:

- all declared input SHA-256 values match; text hashes normalize line endings
  to LF so Windows and Linux checkouts are comparable;
- the scenario loads 18 sequential sites and a 75.3–75.6 km corridor;
- all link-quality values are finite;
- centerline association is the maximum received power;
- the simulator reaches the exact corridor endpoint;
- deterministic CSV outputs match the reference hashes.

Figures may contain metadata differences between Matplotlib versions. Their
editable CSV source data and QA values are the scientific comparison objects.

## Scenario-pack contract

The root-level `configs/`, `data/`, and `results/` remain frozen so the original
published baseline can still be verified. New experiments use a selected
`scenarios/<name>/` pack. A pack is accepted when its route and site files
load, all active site IDs and antenna heights resolve, its declared spatial
selection rule passes, and both the link-quality and simulator entry points
complete without changing shared model code.

The `airport_to_airport` inputs are deterministically regenerated from
`build.json` by `scripts/build_subcorridor_scenario.py`. Its endpoint and site
selection audit is stored in `build_report.json`.

## Evidence grade

The current package is **rerunnable** and is tested for same-machine
repeatability. It is not yet independently validated, calibrated to airborne
measurements, or demonstrated across multiple clean operating systems.

## External dependencies

- CesiumJS and OpenStreetMap tiles are loaded from the network by the dashboard.
- The CesiumDrone glTF asset is stored locally with its Apache-2.0 license.
- Original municipal/FCC documents and Street View images remain external;
  their URLs are recorded in the site register and CSV.
