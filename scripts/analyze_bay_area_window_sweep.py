"""Build the R0039/R0041--R0047 assessment-window comparison table."""
from __future__ import annotations

import csv
import gzip
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "research/dynamic-transitions/runs"
OUTPUT = ROOT / "research/dynamic-transitions/reports/R0041-R0047-window-sweep.csv"
RUN_IDS = ["R0039", "R0041", "R0042", "R0043",
           "R0044", "R0045", "R0046", "R0047"]


def read_json(path: Path):
    with (gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz"
          else path.open(encoding="utf-8")) as stream:
        return json.load(stream)


def switch_count(values) -> int:
    return sum(left != right for left, right in zip(values, values[1:]))


def main() -> None:
    output = []
    for run_id in RUN_IDS:
        run = RUNS / run_id
        config = read_json(run / "resolved_config.json")
        summary = read_json(run / "summary.json")
        trace = read_json(run / "trace.json.gz")
        observations = read_json(run / "observations.json.gz")
        metrics = summary["metrics"]

        traces = {}
        for row in trace:
            traces.setdefault(row["aircraft_id"], []).append(row)
        reversals = []
        speed_ranges = []
        for rows in traces.values():
            directions = []
            for left, right in zip(rows, rows[1:]):
                delta = float(right["v_mps"]) - float(left["v_mps"])
                if abs(delta) > 1e-7:
                    directions.append(1 if delta > 0 else -1)
            reversals.append(switch_count(directions))
            speeds = [float(row["v_mps"]) for row in rows]
            speed_ranges.append(max(speeds) - min(speeds))

        observed = {}
        group_counts = {str(size): 0 for size in range(1, 6)}
        for row in observations:
            observed.setdefault(row["aircraft_id"], []).append(row)
            group_counts[str(row["group_size"])] += 1
        raw_switches = []
        for rows in observed.values():
            rows.sort(key=lambda row: float(row["t_s"]))
            raw_switches.append(switch_count([
                row.get("raw_policy", row["policy"]) for row in rows]))

        shares = metrics["policy_shares"]
        tasks = int(metrics["longitudinal_task_count"])
        output.append({
            "run_id": run_id,
            "entry_headway_s": config["traffic"]["entry_headway_s"],
            "window_s": config["parameters"]["window_s"],
            "c_share": shares["C"], "r_share": shares["R"],
            "f_share": shares["F"],
            "mean_operational_switches_per_flight": metrics["mean_switches_per_flight"],
            "mean_raw_switches_per_flight": sum(raw_switches) / len(raw_switches),
            "mean_speed_direction_reversals_per_flight": sum(reversals) / len(reversals),
            "mean_speed_range_mps": sum(speed_ranges) / len(speed_ranges),
            "longitudinal_tasks": tasks,
            "settled_tasks": metrics["completed_before_next_change_count"],
            "settled_fraction": (metrics["completed_before_next_change_count"] / tasks
                                 if tasks else ""),
            "timing_compatible_fraction": metrics["timing_compatible_fraction"],
            "group_1_share": group_counts["1"] / len(observations),
            "group_5_share": group_counts["5"] / len(observations),
            "completed_aircraft": summary["completed_aircraft"],
            "scheduled_aircraft": summary["scheduled_aircraft"],
            "minimum_sampled_horizontal_separation_m":
                metrics["minimum_sampled_horizontal_separation_m"],
            "sampled_nmac": metrics["nmac_sampled"],
        })

    with OUTPUT.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    print(OUTPUT)


if __name__ == "__main__":
    main()
