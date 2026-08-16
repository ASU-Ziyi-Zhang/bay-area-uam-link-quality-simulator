"""Run the declared SF–SJ single-UAM simulator baseline."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from uam_simulator.runner import run_simulation  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs" / "simulator.json")
    parser.add_argument("--output", type=Path, default=ROOT / "runs" / "simulator")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    summary = run_simulation(args.config, args.output, overwrite=args.overwrite)
    print(f"Simulator complete: {args.output.resolve()}")
    print(f"Handoffs: {summary['samples']['handoff_count']}")


if __name__ == "__main__":
    main()
