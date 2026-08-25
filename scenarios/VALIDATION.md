# Scenario migration validation

Validated on 2026-08-24 with the repository's shared geometry, radio, and
simulator kernels.

## Geometry and selection

| Scenario | Corridor | Length | Active sites | Site IDs |
|---|---|---:|---:|---|
| `sf_sj_full` | SF 4th & King–San Jose Diridon | 75.423116 km | 18 | BS01–BS18 |
| `airport_to_airport` | Millbrae Caltrain–Santa Clara Caltrain | 49.542589 km | 12 | BS05–BS16 |

For the airport-access case, Millbrae trims 21.726500 km from the north and
Santa Clara trims 4.154027 km from the south of the source route. Both supplied
station proxies lie on the retained GTFS polyline within numerical precision.
The same 5 km shortest-distance site rule excludes BS01–BS04 and BS17–BS18.

## Executed checks

```powershell
python scripts\build_subcorridor_scenario.py
python -m pytest -q
python scripts\verify_reference.py
python scripts\run_simulator.py --config scenarios\sf_sj_full\simulator.json --output runs\verify_sf_sj_full
python scripts\run_simulator.py --config scenarios\airport_to_airport\simulator.json --output runs\verify_airport_to_airport
python scripts\run_link_quality.py --scenario scenarios\sf_sj_full\scenario.json --output runs\verify_link_sf_sj_full
python scripts\run_link_quality.py --scenario scenarios\airport_to_airport\scenario.json --output runs\verify_link_airport_to_airport
```

Both link-quality runs passed finite-value, height-resolution, serving-power,
interference-identity, and figure-export checks. The full and airport-access
simulator smoke runs reached their endpoints and produced 13 and 9 handoffs,
respectively. These counts are deterministic outputs under the current radio
assumptions, not measured network performance.

## Evidence boundary

This migration is traceable and rerunnable. The airport-access endpoints are
Caltrain station proxies suggested by Toyota ITL, not approved airport
vertiport coordinates. The macro-site layer remains a documented physical-site
planning inventory, not a current operator-certified RF deployment.
