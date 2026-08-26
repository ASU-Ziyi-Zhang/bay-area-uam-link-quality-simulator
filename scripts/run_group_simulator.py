"""Run the fixed-lane multi-UAM group-policy simulator."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from uam_simulator.group_runner import run_group_simulation  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "scenarios" / "airport_to_airport" / "group_simulator.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "runs" / "airport-to-airport-group-policy-v2",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    summary = run_group_simulation(args.config, args.output, overwrite=args.overwrite)
    print(f"Group simulator run: {args.output.resolve()}")
    print(
        "Policy shares C/R/F: "
        + "/".join(f"{summary['policy']['shares'][key]:.4f}" for key in ("C", "R", "F"))
    )
    print(f"Q0.95: {summary['capacity']['q_mix_rho_uam_h']:.3f} UAM/h")


if __name__ == "__main__":
    main()
