"""Run fixed multi-stream baselines and endpoint-fixed offset optimization."""
from pathlib import Path
import argparse
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from uam_simulator.baseline_studies import run_baseline_studies


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="Override the topic-specific configuration")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--study", choices=("all", "lanes", "offsets"), required=True)
    args = parser.parse_args()
    if args.config is None:
        if args.study == "all":
            parser.error("combined historical runs require --config; use lanes or offsets for topic runs")
        args.config = ROOT / {
            "lanes": "research/geometry-response/configs/fixed_streams.json",
            "offsets": "research/corridor-optimization/configs/offset_search.json",
        }[args.study]
    run_baseline_studies(args.config, args.output, args.study)
    print(f"Research artifacts: {args.output.resolve()}")


if __name__ == "__main__":
    main()
