"""Small command-line entry point for scenario QA."""

from __future__ import annotations

import argparse
import json

from .scenario import load_scenario


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect_parser = subparsers.add_parser("inspect")
    inspect_parser.add_argument("scenario")
    arguments = parser.parse_args()
    scenario = load_scenario(arguments.scenario)
    print(
        json.dumps(
            {
                "scenario_id": scenario.scenario_id,
                "corridor_length_km": scenario.corridor.length_m / 1000.0,
                "base_station_count": len(scenario.base_stations.stations),
                "site_ids": scenario.base_stations.site_ids,
                "transit_time_s": scenario.transit_time_s,
                "radio_ready": scenario.radio_ready,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
