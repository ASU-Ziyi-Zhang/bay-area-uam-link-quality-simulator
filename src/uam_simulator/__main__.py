"""Command line entry point for the first simulator runner."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from .runner import run_simulation


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the modular UAM corridor baseline.")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    output = args.output
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%SZ")
        output = Path("output") / "runs" / f"{args.config.stem}-{stamp}"
    summary = run_simulation(args.config, output, overwrite=args.overwrite)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
